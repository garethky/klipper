import logging, time, chelper
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync

class CollisionAnalyzer:
    def __init__(self, samples, probe_start_time, probe_end_time,
                 trigger_time):
        import numpy as np
        self._samples = np.asarray(samples).astype(float)
        self._probe_start_time = probe_start_time
        self._probe_end_time = probe_end_time
        self._trigger_time = trigger_time
        self._time = self._samples[:, 0]
        self._force = self._samples[:, 1]
        self._pre_collision_line = None
        self._post_collision_line = None
        self._elbow_index = self._collision_force = self._collision_time \
            = self._start_index = 0
        self._end_index = len(samples) -1
        self._analyze()
    def _analyze(self):
        start_time = time.time()
        # trim to interesting samples
        self.trim_to_probing()
        # select elbow
        self._elbow_index = self.select_elbow_point(self._start_index,
                                                    self._end_index)
        end_time = time.time()
        elapsed = end_time - start_time
        #logging.info("start_index: %s, elbow index, %s, end_index: %s. Time taken: %ss"
        #             % (self._start_index, self._elbow_index, self._end_index, elapsed))
        self._collision_force = 0
        self._collision_time = self.calculate_collision(
            self._start_index, self._elbow_index, self._end_index)
        #self._collision_time = self.calculate_dumb_collision(
        #    self._start_index, self._elbow_index, self._end_index)
        end_time = time.time()
        elapsed = end_time - start_time
        #logging.info("collision force: %s, collision time, %s.  Time taken: %ss"
        #             % (self._collision_force, self._collision_time, elapsed))
    def trim_to_probing(self):
        import numpy as np
        # find the trigger index:
        trigger_index = 0
        for i in reversed(range(len(self._samples))):
            trigger_index = i
            if self._samples[i][0] <= self._trigger_time:
                break
        # TODO: make this some multiple of the probing speed
        # - OR - 
        # take up to 200ms of data from before the probe triggered:
        half_second = self._trigger_time - 0.2
        for i in reversed(range(0, trigger_index)):
            if self._samples[i][0] <= half_second:
                self._start_index = i
                break
        # look forward from the trigger_index for increasing force
        self._end_index = len(self._samples) - 1
        #trigger_force = self._samples[trigger_index][1]
        #for i in range(trigger_index + 1, len(self._samples)):
        #    f1 = self._samples[i][1]
        #    f2 = self._samples[i - 1][1]
        #    if trigger_force >= 0 and f1 > f2:
        #        self._end_index += 1
        #    elif trigger_force < 0 and f1 < f2:
        #        self._end_index += 1
        #    else:
        #        break
        with open('/home/pi/printer_data/logs/loadcell.log', 'a') as log:
            log.write("{\"time\": %s," % (np.array2string(self._time[self._start_index:self._end_index], separator=',', threshold=50000)))
            log.write("\"force\": %s," % (np.array2string(self._force[self._start_index:self._end_index], separator=',', threshold=50000)))
            log.write("\"trigger_index\": %s," % (trigger_index - self._start_index))
            log.write("\"trigger_time\": %s," % (self._trigger_time))
    # perpendicular distance from point p to line l1, l2
    def _perpendicular_distance(self, l1_x, l1_y, l2_x, l2_y, p_x, p_y):
        import numpy as np
        l1=np.array([l1_x, l1_y])
        l2=np.array([l2_x, l2_y])
        point = np.array([p_x, p_y])
        return abs(np.cross(l2-l1,point-l1)/np.linalg.norm(l2-l1))
    def select_elbow_point(self, start_index, end_index):
        import numpy as np
        sub_time = self._time[start_index:end_index]
        sub_force = self._force[start_index:end_index]
        x_coords = [sub_time[0], sub_time[-1]]
        y_coords = [sub_force[0], sub_force[-1]]
        x_stacked = np.vstack([x_coords, np.ones(2)]).T
        mx, b = np.linalg.lstsq(x_stacked, y_coords, rcond=None)[0]
        # now compute the Y delta of the line value for every x
        delta_y = np.abs(((mx * sub_time) + b) - sub_force)
        elbow_index = np.argmax(delta_y)
        # elbow point is point of max curvature, but which series does it
        # belong to, approach or collision?
        delta_to_approach = self._perpendicular_distance(
                sub_time[0], delta_y[0],
                sub_time[elbow_index - 1], delta_y[elbow_index - 1],
                sub_time[elbow_index], delta_y[elbow_index])
        delta_to_collision = self._perpendicular_distance(
                sub_time[elbow_index + 1], delta_y[elbow_index + 1],
                sub_time[-1], delta_y[-1],
                sub_time[elbow_index], delta_y[elbow_index])
        # use the perpendicular distance to decide which its closer to:
        #if (delta_to_approach < delta_to_collision):
        elbow_index += 1
        return start_index + elbow_index  # best fit found
    def calculate_dumb_collision(self, start_index, elbow_index, end_index):
        return (self._time[elbow_index - 1] + self._time[elbow_index]) / 2
    def polyfit_collision(self, elbow_index):
        # https://stackoverflow.com/questions/16827053/solving-for-x-values-of-polynomial-with-known-y
        from numpy.polynomial import Polynomial as P
        end_i = elbow_index + 5
        sub_time = self._time[elbow_index:end_i]
        sub_force = self._force[elbow_index:end_i]
        elbow_time = self._time[elbow_index]
        #TODO: make the order of the polynomial configurable in klipper config
        # note: order 1 == linear fit, which might be a + for perfect machines
        p = P.fit(sub_time, sub_force, 4)
        roots = (p - 0.).roots()
        # want a root that is closest to elbow_time
        selected_root = None
        root_distance = float("inf")
        for root in roots:
            if abs(elbow_time - root) < root_distance:
                root_distance = abs(elbow_time - root)
                selected_root = root
        #if not selected_root.imag == 0.0:
            #logging.error("Polynomial Root is not real %s, %s" % (selected_root, roots))
        return selected_root.real
    def calculate_collision(self, start_index, elbow_index, end_index):
        lower_bound = self._time[elbow_index - 1]
        upper_bound = self._time[elbow_index]
        result_1 = self.polyfit_collision(elbow_index)
        if result_1 >= lower_bound and result_1 <= upper_bound:
            return result_1
        result_2 = self.polyfit_collision(elbow_index - 1)
        if result_2 < lower_bound:
            return lower_bound
        if result_2 > upper_bound:
            return upper_bound
        return result_2
    def get_endstop_event(self):
        series = []
        collision_area = self._samples[self._start_index : self._end_index]
        for i in range(len(collision_area)):
            series.append ({
                "time": collision_area[i][0],
                "force": collision_area[i][1],
                "counts": collision_area[i][2]
            })
        return {
            "series": series,
            #"pre_collision_line": self._pre_collision_line,
            #"post_collision_line": self._post_collision_line,
            "collision_point": [{"time": self._collision_time,
                                 "force": self._collision_force}],
            "elbow_point": [{"time": self._time[self._elbow_index],
                             "force": self._force[self._elbow_index]}],
            "trigger_point": [{"time": self._trigger_time,
                             "force": self._force[self._end_index - 2]}],
            "end_point": [{"time": self._time[self._end_index],
                           "force": self._force[self._end_index]}],
            "home_start_time": self._probe_start_time,
            "trigger_time": self._trigger_time,
            "homing_end_time": self._probe_end_time,
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
        self.probe_move = None
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
        self._set_range_cmd.send([self._oid, self.trigger_counts,
                                self.tare_counts])
        self._home_cmd.send([self._oid, etrsync.get_oid()
            , etrsync.REASON_ENDSTOP_HIT, clock
            , self.sample_count, rest_ticks, WATCHDOG_MAX]
            , reqclock=clock)
        self._sample_collector.start_collecting()
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
            self._sample_collector.stop_collecting()
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            self._sample_collector.stop_collecting()
            return 0.
        if self._mcu.is_fileoutput():
            self._sample_collector.stop_collecting()
            return home_end_time
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        trigger_time = self._mcu.clock_to_print_time(trigger_ticks)
        self.last_trigger_time = trigger_time
        return trigger_time
    def pullback_end(self, pullback_end_time):
        samples = self._sample_collector.collect_until(pullback_end_time)
        analyzer = CollisionAnalyzer(samples,
                    self._home_start_time,
                    pullback_end_time,
                    self.last_trigger_time)
        self._load_cell.send_endstop_event(analyzer.get_endstop_event())
        analyzer.get_collision_time()
        with open('/home/pi/printer_data/logs/loadcell.log', 'a') as log:
            log.write("\"home_end_time\": %s," % (self.home_end_time))
            log.write("\"pullback_end_time\": %s," % (pullback_end_time))
        return pullback_end_time
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
        #TODO: BLTouch has a more sophisticated version of this idea that may save some time
        # Before beginning probing, make sure any retract moves have completed
        # this makes sure the retract happens before the trsync gets armed
        toolhead = self._printer.lookup_object('toolhead')
        toolhead.dwell(0.001)
        toolhead.wait_moves()        
    def probe_finish(self, hmove):
        pass
