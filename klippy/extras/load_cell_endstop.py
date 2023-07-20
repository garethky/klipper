import logging, time, chelper
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync
from . import motion_report

# TODO:is this access to motion_report/DumpTrapQ internals OK?
#get recent history of the toolhead motion
def get_trapq_history(printer, start_time):
    return printer.lookup_object('motion_report') \
            .trapqs['toolhead'] \
            .extract_trapq(start_time, motion_report.NEVER_TIME)

class CollisionAnalyzer:
    def __init__(self, samples, home_start_time, home_end_time, trigger_time):
        try:
            import numpy as np
        except:
            raise config.error("LoadCell requires the numpy module")
        self._samples = np.asarray(samples).astype(float)
        self._home_start_time = home_start_time
        self._home_end_time = home_end_time
        self._trigger_time = trigger_time
        self._time = self._samples[:, 0]
        self._force = self._samples[:, 1]
        self._pre_collision_line = None
        self._post_collision_line = None
        self._elbow_index = self._collision_force = self._collision_time \
            = self._start_index = self._end_index = self._end_index = 0
        self._analyze()
    def _analyze(self):
        start_time = time.time()
        start_index, end_index = self.trim_to_probing()
        self._start_index = start_index
        self._end_index = end_index
        elbow_index = self.select_elbow_point(start_index, end_index)
        self._elbow_index = elbow_index
        end_time = time.time()
        elapsed = end_time - start_time
        logging.info("start_index: %s, elbow index, %s, end_index: %s. Time taken: %ss" % (start_index, elbow_index, end_index, elapsed))
        self._collision_force, self._collision_time = self.calculate_collision(start_index, \
                                                    elbow_index, end_index)
        end_time = time.time()
        elapsed = end_time - start_time
        logging.info("collision force: %s, collision time, %s.  Time taken: %ss" % (self._collision_force, self._collision_time, elapsed))
    def trim_to_probing(self):
        #TODO: use movement data to trim start_time
        start_index = 0
        #get_trapq_history()...

        end_index = len(self._samples) - 1
        while self._samples[end_index][0] > self._trigger_time: 
            end_index -= 1
        return 0, end_index + 2  # TODO: 2 is a magic number for my sample rate
    def select_elbow_point(self, start_index, end_index):
        best_fit = float("inf")
        # this assumes fit will improve as the elbow point goes left
        elbow_index = end_index - 2
        for i in reversed(range(start_index + 2 + 20, elbow_index)):
            #TODO: 20 is a magic number and should be based on time!
            # also it could overflow the front of the array!
            new_fit = self.check_elbow_fit(i - 20, i, end_index)
            if new_fit <= best_fit:
                best_fit = new_fit
                elbow_index = i
            else:
                break
        return elbow_index # best fit found
    def calculate_collision(self, start_index, elbow_index, end_index):
        #slope, c = self.least_squares(self._time, self._force,
        #                                        elbow_index, end_index)[0]
        pre_elbow_time = self._samples[elbow_index - 1][0]
        pre_elbow_force = self._samples[elbow_index - 1][1]
        elbow_time = self._samples[elbow_index][0]
        elbow_force = self._samples[elbow_index][1]
        end_force = self._samples[end_index][1]

        # calculate average force per unit time (assumes axis moves ~constant velocity)
        avg_force = (end_force - pre_elbow_force) / (end_index - elbow_index - 1)
        # what percentage of that average force change is the elbow point away from the point just before it?
        # this has to be clamped to 1. because large force changes are not physically possible
        elbow_force_percent = min(1., abs((elbow_force - pre_elbow_force) / avg_force))
        # time between the two points
        time_d = elbow_time - pre_elbow_time
        time_delta = time_d * elbow_force_percent
        return pre_elbow_force, pre_elbow_time + (time_d - time_delta)
    def check_elbow_fit(self, start_index, elbow_index, end_index):
        import numpy as np
        m1, resid_pre = self.least_squares(self._time, self._force,
                                                start_index, elbow_index)
        m2, resid_post = self.least_squares(self._time, self._force,
                                                elbow_index + 1, end_index)
        logging.info("Pre collision fit: %s, post collision fit: %s" 
                     % (np.sum(resid_pre), np.sum(resid_post)))
        
        t0 = self._samples[start_index][0]
        t1 = self._samples[elbow_index][0]
        self._pre_collision_line = [{"time": t0, "force": m1[0] * t0 + m1[1]},
                                    {"time": t1, "force": m1[0] * t1 + m1[1]}]
        # solve the second line for elbow - 1 to end
        t2 = self._samples[elbow_index + 1][0]
        t3 = self._samples[end_index][0]
        self._post_collision_line = [{"time": t2, "force": m2[0] * t2 + m2[1]},
                                     {"time": t3, "force": m2[0] * t3 + m2[1]}]

        return np.sum(resid_pre) + np.sum(resid_post)
    # compute least squares of a sub-array
    def least_squares(self, x, y, start_index, end_index):
        import numpy as np
        sub_x = x[start_index:end_index]
        sub_y = y[start_index:end_index]
        x_stacked = np.vstack([sub_x, np.ones(len(sub_x))]).T
        mxb, residuals, _, _ = np.linalg.lstsq(x_stacked, sub_y, rcond=None)
        return mxb, residuals
    def get_endstop_event(self):
        series = []
        collision_area = self._samples[self._elbow_index - 20 : self._end_index + 5]
        for i in range(len(collision_area)):
            series.append ({
                "time": collision_area[i][0],
                "force": collision_area[i][1],
                "counts": collision_area[i][2]
            })
        return {
            "series": series,
            "pre_collision_line": self._pre_collision_line,
            "post_collision_line": self._post_collision_line,
            "collision_point": [{"time": self._collision_time,
                                 "force": self._collision_force}],
            "elbow_point": [{"time": self._samples[self._elbow_index][0],
                             "force": self._samples[self._elbow_index][1]}],
            "trigger_point": [{"time": self._trigger_time,
                             "force": self._samples[self._end_index - 2][1]}],
            "end_point": [{"time": self._samples[self._end_index][0],
                           "force": self._samples[self._end_index][1]}],
            "home_start_time": self._home_start_time,
            "trigger_time": self._trigger_time,
            "homing_end_time": self._home_end_time,
        }
    # get the print time when the collision started
    def get_collision_time(self):
        return self._collision_time
    # get the collision quality
    def is_good_tap(self):
        return True

DEFAULT_SAMPLE_COUNT = 2
WATCHDOG_MAX = 3
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
        self._sample_collector = load_cell.get_collector()
        self._sensor = sensor = load_cell.get_sensor()
        self._mcu = mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = self._set_range_cmd = None
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
        self.trigger_counts = 0  # this needs to be read from the conifg and maybe pushed in when the load cell calibrates
        self.tare_counts = 0  # this needs to be pushed in every time the load cell tares
        self._home_start_time = 0
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
            "set_range_load_cell_endstop oid=%c trigger_counts=%u tare_counts=%i"
            , cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c" \
            " clock=%u sample_count=%c rest_ticks=%u timeout=%u", cq=cmd_queue)
    def get_status(self, eventtime):
        return {
            'trigger_counts': self.trigger_counts,
            'tare_counts': self.tare_counts,
            'sample_count': self.sample_count
        }
    def set_range(self, trigger_counts, tare_counts):
        self.tare_counts = int(tare_counts)
        self.trigger_counts = abs(int(trigger_counts))
        self._set_range_cmd.send([self._oid, self.trigger_counts,
                                self.tare_counts])
        logging.info("LOAD_CELL_ENDSTOP: Range Set: trigger:counts %i,\
                    tare_counts: %i" % (self.trigger_counts, self.tare_counts))
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
        self._home_start_time = print_time
        clock = self._mcu.print_time_to_clock(print_time)
        rest_ticks = self._load_cell.sensor.get_clock_ticks_per_sample()

        # duplicode
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

        logging.info("LOAD_CELL_ENDSTOP: Homing Started")
        self._set_range_cmd.send([self._oid, self.trigger_counts,
                                self.tare_counts])
        self._home_cmd.send([self._oid, etrsync.get_oid()
            , etrsync.REASON_ENDSTOP_HIT, clock
            , self.sample_count, rest_ticks, WATCHDOG_MAX]
            , reqclock=clock)
        self._sample_collector.start_collecting()
        return self._trigger_completion
    def home_wait(self, home_end_time):
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        logging.info("LOAD_CELL_ENDSTOP: Endstop Triggered")
        # trigger has happened, now to find out why...
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            self._sample_collector.stop_collecting()
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            self._sample_collector.stop_collecting()
            return 0.
        if self._mcu.is_fileoutput():
            self._sample_collector.stop_collecting()
            return home_end_time
        logging.info("LOAD_CELL_ENDSTOP: Trigger Is Good")
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        trigger_time = self._mcu.clock_to_print_time(trigger_ticks)
        # keep recording until at least when the homing stopped
        samples = self._sample_collector.collect_until(home_end_time)
        analyzer = CollisionAnalyzer(samples, self._home_start_time, home_end_time, trigger_time)
        self._load_cell.send_endstop_event(analyzer.get_endstop_event())
        return analyzer.get_collision_time()
    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        logging.info("load_cell_endstop_state oid=%u homing=%u homing_triggered=%u is_triggered=%u trigger_ticks=%u sample=%i sample_ticks=%u", params['oid'], params['homing'], params['homing_triggered'], params['is_triggered'], params['trigger_ticks'], params['sample'], params['sample_ticks'])
        if params['homing'] == 1:
            return params['homing_triggered'] == 1
        else:
            return params['is_triggered'] == 1
    def multi_probe_begin(self):
        pass
    def multi_probe_end(self):
        pass
    def probe_prepare(self, hmove):
        # Before beginning probing, make sure any retract moves have completed
        # this makes sure the retract happens before the trsync gets armed
        toolhead = self._printer.lookup_object('toolhead')
        toolhead.dwell(0.1)
        toolhead.wait_moves()
        #self.gcode.run_script_from_command("QUERY_PROBE")
    def probe_finish(self, hmove):
        #self.gcode.run_script_from_command("QUERY_PROBE")
        pass
