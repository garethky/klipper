import logging, chelper
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync

DEFAULT_SAMPLE_COUNT = 2
WATCHDOG_MAX = 3
MIN_MSG_TIME = 0.100

#LoadCellEndstop implements mcu_endstop and PrinterProbe
class LoadCellEndstop:
    def __init__(self, config, load_cell):
        
        self._config = config
        self._config_name = config.get_name()
        self._printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        printer.register_event_handler('klippy:mcu_identify',
                                        self.handle_mcu_identify)
        self._load_cell = load_cell
        self._mcu = mcu = load_cell.get_sensor().get_mcu()
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = self._set_range_cmd = None
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
        self.trigger_counts = 0
        self.tare_counts = 0
        self.last_trigger_time = 0
        self.trigger_force_grams = config.getfloat('trigger_force_grams'
                    , minval=10., maxval=250, default=50.)
        self.sample_count = config.getint("sample_count"
            , default=DEFAULT_SAMPLE_COUNT, minval=1, maxval=5)
        self._mcu.add_config_cmd("config_load_cell_endstop oid=%d"
                                  % (self._oid))
        self._mcu.add_config_cmd("load_cell_endstop_home oid=%d trsync_oid=0" \
            " trigger_reason=0 clock=0 sample_count=0 rest_ticks=0 timeout=0"
            % (self._oid), on_restart=True)
        self._mcu.register_config_callback(self._build_config)
    def _build_config(self):
        # Lookup commands
        cmd_queue = self._trsyncs[0].get_command_queue()
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_endstop_query_state oid=%c", 
            "load_cell_endstop_state oid=%c homing=%c homing_triggered=%c" \
            " is_triggered=%c trigger_ticks=%u sample=%i sample_ticks=%u",
            oid=self._oid, cq=cmd_queue)
        self._set_range_cmd = self._mcu.lookup_command(
            "set_range_load_cell_endstop oid=%c trigger_counts=%u" \
                " tare_counts=%i"
            , cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c" \
            " clock=%u sample_count=%c rest_ticks=%u timeout=%u", cq=cmd_queue)
    def get_status(self, eventtime):
        return {
            'trigger_counts': self.trigger_counts,
            'tare_counts': self.tare_counts,
            'sample_count': self.sample_count,
            'last_trigger_time': self.last_trigger_time
        }
    def set_range(self, tare_counts, counts_per_gram):
        self.tare_counts = int(tare_counts)
        self.trigger_counts = abs(int(self.trigger_force_grams 
                                      * counts_per_gram))
        self._set_range_cmd.send([self._oid, self.trigger_counts,
                                self.tare_counts])
    def get_mcu(self):
        return self._mcu
    def get_oid(self):
        return self._oid
    def handle_mcu_identify(self):
        kinematics = self._printer.lookup_object('toolhead').get_kinematics()
        for stepper in kinematics.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
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
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # not used:
        sample_time = rest_time = triggered = sample_count = None
        # do not permit homing if the load cell is not calibrated
        if not self._load_cell.is_calibrated():
            raise self._printer.command_error("Load Cell not calibrated")
        # re-compute rest_time based on the sample rate
        rest_ticks = self._load_cell.sensor.get_clock_ticks_per_sample()
        # tare the sensor just before probing
        # this uses pause(), requiring a print_time update
        self._load_cell.pause_and_tare()
        reactor = self._mcu.get_printer().get_reactor()
        now = reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now) + MIN_MSG_TIME
        clock = self._mcu.print_time_to_clock(print_time)
        self._printer.send_event("load_cell_endstop:homing_start_time"
                                , print_time)
        # copied from endstop.py
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        # end copy
        self._home_cmd.send([self._oid, etrsync.get_oid()
            , etrsync.REASON_ENDSTOP_HIT, clock
            , self.sample_count, rest_ticks, WATCHDOG_MAX]
            , reqclock=clock)
        return self._trigger_completion
    def home_wait(self, home_end_time):
        self.home_end_time = home_end_time
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
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        trigger_time = self._mcu.clock_to_print_time(trigger_ticks)
        self.last_trigger_time = trigger_time
        return trigger_time
    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        if params['homing'] == 1:
            return params['homing_triggered'] == 1
        else:
            return params['is_triggered'] == 1
    def multi_probe_begin(self):
        pass
    def multi_probe_end(self):
        pass
    def probe_prepare(self, hmove):
        pass
    def probe_finish(self, hmove):
        pass
