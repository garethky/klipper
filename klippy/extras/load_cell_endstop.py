import logging
import chelper
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync
import sys


#LoadCellEndstop implements mcu_endstop
class LoadCellEndstop:
    def __init__(self, config, sensor):
        self._printer = printer = config.get_printer()
        self._mcu = mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = self._reset_cmd = None
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
        min_int = -sys.maxint - 1
        self.deadband = config.getint("deadband", default=1, minval=1
            , maxval=0xffffff)
        self.settling_count = config.getint("settling_count", default=100
            , minval= 1, maxval=sys.maxint)
        self.setpoint_alpha = config.getint("setpoint_alpha", default=4
            , minval= 1, maxval=31)
        self.trend_alpha = config.getint("trend_alpha", default=1
            , minval= 1, maxval=31)
        self.crash_min = config.getint("crash_min", default = -1
            , minval= min_int, maxval=sys.maxint)
        self.crash_max = config.getint("crash_max", default = 1
            , minval= min_int, maxval=sys.maxint)
        if self.crash_min > self.crash_max:
            "Crash minimum must be less than crash maximum"
        self._mcu.add_config_cmd("config_load_cell_endstop oid=%d deadband=%d "\
            "crash_min=%d crash_max=%d sample_filter_alpha=%d " \
            "trend_filter_alpha=%d settling_count=%d" % (self._oid
            , self.deadband, self.crash_min, self.crash_max
            , self.setpoint_alpha, self.trend_alpha, self.settling_count))
        self._mcu.add_config_cmd("load_cell_endstop_home oid=%d trsync_oid=0 " \
            "trigger_reason=0 sample_count=0"
            % (self._oid), on_restart=True)
        self._mcu.register_config_callback(self._build_config)
    def get_mcu(self):
        return self._mcu
    def get_oid(self):
        return self._oid
    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")
    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]
    def _build_config(self):
        # Lookup commands
        cmd_queue = self._trsyncs[0].get_command_queue()
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_endstop_query_state oid=%c", "load_cell_endstop_state " \
            "oid=%c homing=%c is_triggered=%c trigger_ticks=%u sample=%i " \
            "ticks=%u sample_avg=%i trend_avg=%i", oid=self._oid, cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command("load_cell_endstop_home " \
            "oid=%c trsync_oid=%c trigger_reason=%c sample_count=%c"
            , cq=cmd_queue)
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # not used:
        sample_time = rest_time = triggered = None
        # duplicode
        clock = self._mcu.print_time_to_clock(print_time)
        reactor = self._mcu.get_printer().get_reactor()
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        # duplicode end
        self._home_cmd.send([self._oid, etrsync.get_oid()
            , etrsync.REASON_ENDSTOP_HIT, sample_count], reqclock=clock)
        return self._trigger_completion
    def home_wait(self, home_end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        # trigger has happened, now to find out why...
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self._mcu.is_fileoutput():
            return home_end_time
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        next_clock = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        trigger_t = self._mcu.clock_to_print_time(next_clock - self._rest_ticks)
        return trigger_t
    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        # TODO: if the time since last sample is larger than the sample interval
        # throw an error because the endstop is not being fed fresh data
        return params['is_triggered'] == 1
