import logging
import chelper
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync
import sys

class LoadCellEndstopCalibrator:
    def __init__(self, config, lc_endstop, load_cell):
        self._printer = config.get_printer()
        self._load_cell = load_cell
        self._lc_endstop = lc_endstop
        self._collector = self._load_cell.get_collector()
        self.name = config.get_name().split()[-1]
        self.register_commands(self.name)
    def register_commands(self, name):
        gcode = self._printer.lookup_object('gcode')
        gcode.register_mux_command("LOAD_CELL_ENDSTOP_CALIBRATE", "LOAD_CELL",
                            name, self.cmd_LOAD_CELL_ENDSTOP_CALIBRATE,
                            desc=self.cmd_LOAD_CELL_ENDSTOP_CALIBRATE_help)
    cmd_LOAD_CELL_ENDSTOP_CALIBRATE_help = "Calibrate Load Cell Endstop"
    def cmd_LOAD_CELL_ENDSTOP_CALIBRATE(self, gcmd):
        # if 1, apply the settings immediatly
        apply_settings = gcmd.get_int("APPLY", default=0, minval=0, maxval=1)
        # the default is 1 seconds worth of samples at whatever the sample rate
        # is, or minval if that is larger.
        sample_seconds = gcmd.get_int("SAMPLE_SECONDS", default=5,
                        minval=1, maxval=60)
        # The maximum % of the raw signal width that the sample filter should
        # produce.
        sample_smoothing = gcmd.get_int("SAMPLE_SMOOTHING", default=25,
                        minval=5, maxval=100)
        # Deadband will be +/- 10% larger than the sample width
        deadband_gap = gcmd.get_int("DEADBAND_GAP", default=50, minval=1,
                        maxval=200)
        # Over 1 second, what is the maximum allowed change in the deadband
        # centerline value as a % of the width of the deadband.
        trend_rate = gcmd.get_float("TREND_RATE", default=0.75, minval=0.01,
                                    maxval=10.0)
        # This is the number of multiples of the raw sample width (+/-) where
        # the crash ban dwill be located. 1 = max sample + sample width & min
        # sample - sample width
        crash_gap = gcmd.get_int("CRASH_GAP", default=5, minval=1, maxval=100)
        
        if (not self._load_cell.is_capturing()):
            gcmd.error("Calibration aborted: load cell not capturing")
            return
        sample_count = self._load_cell.sps() * sample_seconds
        samples = self._collector.collect(sample_count)
        samples = [sample[1] for sample in samples]
        results = self.calibrate(samples, sample_smoothing, deadband_gap, trend_rate, crash_gap)
        gcmd.respond_info("Calibration Complete: Deadband: %i, Crash Min: %i" \
        " Crash Max: %i, Setpoint Alpha: %i, Trend Alpha: %i," \
        " Settling Count: %i" % results)
        if apply_settings:
            self._lc_endstop.reset_config(*results)
    def calibrate(self, samples, sample_smoothing, deadband_gap, trend_rate, 
                    crash_gap):
        deadband = sample_alpha = trend_alpha = settling_count = 0
        crash_min = crash_max = 0

        sample_width = self._width(samples)
        sample_avg = self._avg(samples)
        sample_seconds = float(len(samples)) / float(self._load_cell.sps())
        logging.info("sample width: %i sample average: %i" % (sample_width, sample_avg))
        crash_max = int(sample_avg + (sample_width * crash_gap))
        crash_min = int(sample_avg - (sample_width * crash_gap))

        # TODO: do sample alpha based on force in grams when conversion to grams is implemented
        ema_width = sample_width
        ema_target_width = (sample_width * (sample_smoothing / 100.0))
        logging.info("target width: %i, multiplier: %f, smoothing: %i" % (ema_target_width, (sample_smoothing / 100.0), sample_smoothing))
        while (ema_width > ema_target_width and sample_alpha < 32):
            sample_alpha += 1
            sample_ema = self._ema(samples, sample_alpha, sample_avg)
            ema_width = self._width(sample_ema)
            logging.info("ema width: %i, target width: %i" % (ema_width, ema_target_width))
        
        deadband = int((ema_width / 2.) * (1. + (deadband_gap / 100.)))
        
        trend_alpha = sample_alpha + 1
        deadband_samples = [(deadband - 1) for i in range(5)]
        trend_delta = deadband
        target_delta = ((deadband * 2) * (trend_rate / 100.0)) / sample_seconds
        while (trend_delta > target_delta and trend_alpha < 32):
            trend_alpha += 1
            trend_delta = self._ema(deadband_samples, trend_alpha, 0)[-1]
            logging.info("trend delta: %i" % (trend_delta))
        
        settling_count = 1  # means the minimum value is at least 2!
        settling_avg = sample_avg + sample_width
        while abs(sample_avg - settling_avg) > (sample_width * 0.01):
            settling_count += 1
            settling_avg = self._avg(samples, settling_count)

        return (deadband, crash_min, crash_max, sample_alpha, trend_alpha
                , settling_count)
    def _pick_sample_size(self, user_sample_size):
        load_cell_sps = self._load_cell.sps() * user_sample_size
        return max(load_cell_sps, self._min_samples)
    def _width(self, data):
        return max(data) - min(data)
    def _avg(self, data, points=None):
        points = len(data) if points is None else points
        avg = 0
        for index in range(points):
            logging.info("avg: %i" % (avg))
            avg = avg + int((data[index] - avg) / (index + 1))
        return avg
    def _ema(self, data, alpha, setpoint=0):
        # replicates the way the EMA filter works in C
        half = 1 << (alpha - 1)
        average = setpoint
        state = (setpoint << alpha) - setpoint
        ema_data = []
        for sample in data:
            state += sample
            neg = 1 if state < 0 else 0
            average = int(state - neg + half) >> alpha
            state -= average
            ema_data.append(int(average))
        return ema_data

DEFAULT_SAMPLE_COUNT = 2
#LoadCellEndstop implements mcu_endstop
class LoadCellEndstop:
    def __init__(self, config, sensor):
        self._config = config
        self._config_name = config.get_name()
        self._printer = printer = config.get_printer()
        printer.register_event_handler('klippy:mcu_identify',
                                        self.handle_mcu_identify)
        self._mcu = mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = self._reset_cmd = None
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
        self._rest_ticks = 0
        min_int = -sys.maxint - 1
        self.deadband = config.getint("deadband", default=1, minval=1
            , maxval=0xffffff)
        self.crash_min = config.getint("crash_min", default = -1
            , minval=min_int, maxval=sys.maxint)
        self.crash_max = config.getint("crash_max", default = 1
            , minval=min_int, maxval=sys.maxint)
        self.setpoint_alpha = config.getint("setpoint_alpha", default=4
            , minval=1, maxval=31)
        self.trend_alpha = config.getint("trend_alpha", default=1
            , minval=1, maxval=31)
        self.settling_count = config.getint("settling_count", default=100
            , minval=1, maxval=sys.maxint)
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
    def _build_config(self):
        # Lookup commands
        cmd_queue = self._trsyncs[0].get_command_queue()
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_endstop_query_state oid=%c", "load_cell_endstop_state " \
            "oid=%c homing=%c is_triggered=%c trigger_ticks=%u sample=%i " \
            "ticks=%u sample_avg=%i trend_avg=%i", oid=self._oid, cq=cmd_queue)
        self._reset_cmd = self._mcu.lookup_command(
            "reset_load_cell_endstop oid=%c deadband=%i crash_min=%i " \
            "crash_max=%i sample_filter_alpha=%c trend_filter_alpha=%c " \
            "settling_count=%i", cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command("load_cell_endstop_home " \
            "oid=%c trsync_oid=%c trigger_reason=%c sample_count=%c"
            , cq=cmd_queue)
        self._load_cell = self._printer.lookup_object(self._config.get_name())
        LoadCellEndstopCalibrator(self._config, self, self._load_cell)
    def get_status(self, eventtime):
        return {
            'deadband': self.deadband,
            'settling_count': self.settling_count,
            'setpoint_alpha': self.setpoint_alpha,
            'trend_alpha': self.trend_alpha,
            'crash_min': self.crash_min,
            'crash_max': self.crash_max
        }
    def get_mcu(self):
        return self._mcu
    def get_oid(self):
        return self._oid
    def handle_mcu_identify(self):
        kinematics = self._printer.lookup_object('toolhead').get_kinematics()
        for stepper in kinematics.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def reset(self):
        self._reset_cmd.send([self._oid, self.deadband, self.crash_min
            , self.crash_max, self.setpoint_alpha, self.trend_alpha
            , self.settling_count])
    def reset_config(self, deadband, crash_min, crash_max, setpoint_alpha
                        , trend_alpha, settling_count):
        self.deadband = deadband
        self.crash_min = crash_min
        self.crash_max = crash_max
        self.setpoint_alpha = setpoint_alpha
        self.trend_alpha = trend_alpha
        self.settling_count = settling_count
        # Store results for SAVE_CONFIG
        configfile = self._printer.lookup_object('configfile')
        name = self._config_name
        configfile.set(name, 'deadband', "%.3f" % (deadband,))
        configfile.set(name, 'crash_min', "%.3f" % (crash_min,))
        configfile.set(name, 'crash_max', "%.3f" % (crash_max,))
        configfile.set(name, 'setpoint_alpha', "%.3f" % (setpoint_alpha,))
        configfile.set(name, 'trend_alpha', "%.3f" % (trend_alpha,))
        configfile.set(name, 'settling_count', "%.3f" % (settling_count,))
        self.reset()
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
        # re-compute rest_time based on the sample rate
        clock = self._mcu.print_time_to_clock(print_time)
        self._rest_ticks = self._mcu.print_time_to_clock(print_time 
                                + (1. / self._load_cell.sps())) - clock
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
            , etrsync.REASON_ENDSTOP_HIT, DEFAULT_SAMPLE_COUNT]
            , reqclock=clock)
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
    def multi_probe_begin(self):
        pass
    def multi_probe_end(self):
        pass
    def probe_prepare(self, hmove):
        pass
    def probe_finish(self, hmove):
        pass
