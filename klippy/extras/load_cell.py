# Load Cell Implementation
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import copy
import chelper
import logging, collections
import probe
from . import probe, load_cell_endstop, multiplex_adc
from probe import PrinterProbe

class SaturationException(Exception):
    pass

class SampleException(Exception):
    pass

# An API Dump Helper for broadcasting events to clients
class WebhooksApiDumpRepeater:
    def __init__(self, printer, endpoint, key, value, header):
        self.clients = {}
        self.header = header
        wh = printer.lookup_object('webhooks')
        wh.register_mux_endpoint(endpoint, key, value,
                                 self._add_webhooks_client)
    def _add_webhooks_client(self, web_request):
        self.add_client(web_request)
        if self.header:
            web_request.send(self.header)
    def add_client(self, web_request):
        cconn = web_request.get_client_connection()
        template = web_request.get_dict('response_template', {})
        self.clients[cconn] = template
    def send(self, msg):
        for cconn, template in list(self.clients.items()):
            if cconn.is_closed():
                del self.clients[cconn]
                continue
            tmp = dict(template)
            tmp['params'] = msg
            cconn.send(tmp)

#########################
# Math Support Functions

# compute the time some value was surpassed
def index_near(x, x_val):
    import numpy as np
    return np.argmax(np.asarray(x) >= x_val) or len(x) -1 

def linear_regression(x1, y1, x2, y2):
    import numpy as np
    time1 = np.asarray(x1)
    force1 = np.asarray(y1)
    m1, b1 = lstsq(time1, force1)[0]
    time2 = np.asarray(x2)
    force2 = np.asarray(y2)
    m2, b2 = lstsq(time2, force2)[0]
    numerator = -b1 + b2
    denominator = m1 - m2
    end_time = numerator / denominator
    return end_time

# Least Squares on x[] y[] points
def lstsq(x, y):
    import numpy as np
    x_stacked = np.vstack([x, np.ones(len(x))]).T
    return np.linalg.lstsq(x_stacked, y, rcond=None)

# Kneedle algorithm
def _kneedle(x, y, x_coords, y_coords):
    import numpy as np
    x_stacked = np.vstack([x_coords, np.ones(2)]).T
    mx, b = np.linalg.lstsq(x_stacked, y_coords, rcond=None)[0]
    # now compute the Y of the line value for every x
    fit_y = []
    for time in x:
        fit_y.append((mx * time) + b)
    fit_y = np.asarray(fit_y)
    delta_y = fit_y - y
    return np.argmax(delta_y), np.argmin(delta_y), delta_y

# Find Kneedle with additional width
def find_kneedle(x, y, additional_width = 0.0):
    start = x[0]
    end = x[-1]
    extension = (end - start) * (additional_width / 2.0)
    start = start - extension
    end = end + extension
    x_coords = [start, end]
    y_coords = [y[0], y[-1]]
    return _kneedle(x, y, x_coords, y_coords)

DISCARD_POINTS = 3
ADDITIONAL_WIDTH = 2.0
class ProbePointAnalysis(object):
    def __init__(self, printer, home_end_time, pullback_end_time):
        self.samples = None
        self.time = None
        self.force = None
        self.z = None
        self.moves = None
        self.discard = DISCARD_POINTS
        self.additional_width = ADDITIONAL_WIDTH
        self.printer = printer
        #self.probe_speed = probe_speed
        #self.pullback_speed = pullback_speed
        #self.trigger_time = trigger_time
        self.home_end_time = home_end_time
        self.pullback_end_time = pullback_end_time
    def set_samples(self, samples):
        import numpy as np
        # todo: does this need trimming?
        # convert samples to force and time arrays
        np_samples = np.array(samples)
        self.time = np_samples[:, 0]
        self.force = np_samples[:, 1]
        self.moves = self.get_moves()
        self.z = self.get_toolhead_positions(self.time)
    def get_moves(self):
        trapq = self.printer.lookup_object('motion_report').trapqs['toolhead']
        moves, cdata  = trapq.extract_trapq(self.time[0], self.time[-1])
        return moves
    # build toolhead Z position for the time/force points
    def get_toolhead_positions(self, time):
        # pull the movement queue data for the time of the move
        z_pos = []
        next_t = 0
        # TODO: assumes this goes from old moves to new moves, like times does
        for move in self.moves:
            while next_t < len(time) \
                    and self.is_time_in_move(time[next_t], move):
                z_pos.append(self.calculate_z(time[next_t], move))
                next_t += 1
        return z_pos
    # test if a print time is inside a move
    def is_time_in_move(self, time, move):
        start_time = move.print_time
        end_time = move.print_time + move.move_t
        return time >= start_time and time < end_time
    # given a move and a print time inside that move, calculate z
    def calculate_z(self, print_time, move):
        move_time = max(0., min(move.move_t, print_time - move.print_time))
        dist = (move.start_v + .5 * move.accel * move_time) * move_time
        return move.start_z + move.z_r * dist
    def analyze(self):
        discard = self.discard
        additional_width = self.additional_width
        force = self.force
        time = self.time
        pullback_start_index = index_near(time, self.home_end_time)
        pullback_end_index = index_near(time, self.pullback_end_time)
        pullback_time = time[pullback_start_index:pullback_end_index]
        pullback_force = force[pullback_start_index:pullback_end_index]
        min_elbow, max_elbow, delta_y = find_kneedle(pullback_time
                        , pullback_force, additional_width = additional_width)
        # The above finds 2 elbows, we want the one that is a maximum
        # +1 to move to the first point after the elbow
        max_elbow += 1
        # split the pullback data in 2 about the elbow,
        # discarding points near the elbow and ends
        rise_time = pullback_time[min_elbow + discard: max_elbow - discard]
        rise_force = pullback_force[min_elbow + discard: max_elbow - discard]
        decompress_time = pullback_time[max_elbow + discard: -1]
        decompress_force = pullback_force[max_elbow + discard: -1]
        # use linear regression to compute the time when the two lines intersect
        true_t0 = linear_regression(rise_time, rise_force, decompress_time
                                , decompress_force)
        # find the pullback move from the trapq:
        for move in self.moves:
            if self.is_time_in_move(true_t0, move):
                # z coordinate at true_t0
                return self.calculate_z(true_t0, move)
        raise Exception('Pullback move not found in trapq')

class LoadCellCommandHelper:
    def __init__(self, config, load_cell):
        try:
            import numpy as np
        except:
            raise config.error("LoadCell requires the numpy module")
        self.printer = config.get_printer()
        self.load_cell = load_cell
        self.name = config.get_name().split()[-1]
        self.register_commands(self.name)
        if self.name == "load_cell":
            self.register_commands(None)
    def register_commands(self, name):
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("TARE_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_TARE_LOAD_CELL,
                                   desc=self.cmd_TARE_LOAD_CELL_help)
        gcode.register_mux_command("CALIBRATE_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_CALIBRATE_LOAD_CELL,
                                   desc=self.cmd_CALIBRATE_LOAD_CELL_help)
        gcode.register_mux_command("READ_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_READ_LOAD_CELL,
                                   desc=self.cmd_READ_LOAD_CELL_help)
        gcode.register_mux_command("LOAD_CELL_DIAGNOSTIC", "LOAD_CELL", name,
                                   self.cmd_LOAD_CELL_DIAGNOSTIC,
                                   desc=self.cmd_LOAD_CELL_DIAGNOSTIC_help)
    cmd_TARE_LOAD_CELL_help = "Set the Zero point of the load cell"
    def cmd_TARE_LOAD_CELL(self, gcmd):
        tare_counts = self.load_cell.avg_counts()
        tare_percent = self.load_cell.counts_to_percent(tare_counts)
        self.load_cell.tare(tare_counts)
        gcmd.respond_info("Load cell tare value: %.2f%% (%i)" 
                          % (tare_percent, tare_counts))
    cmd_CALIBRATE_LOAD_CELL_help = "Start interactive calibration tool"
    def cmd_CALIBRATE_LOAD_CELL(self, gcmd):
        LoadCellGuidedCalibrationHelper(self.printer, self.load_cell)
    cmd_READ_LOAD_CELL_help = "Take a reading from the load cell"
    def cmd_READ_LOAD_CELL(self, gcmd):
        counts = self.load_cell.avg_counts()
        percent = self.load_cell.counts_to_percent(counts)
        force = self.load_cell.counts_to_grams(counts)
        if percent >= 100 or percent<= -100:
            gcmd.respond_info("Err (%.2f%%)" % (percent))
        if force is None:
            gcmd.respond_info("---.-g (%.2f%%)" % (percent))
        else:
            gcmd.respond_info("%.1fg (%.2f%%)" % (force, percent))
    cmd_LOAD_CELL_DIAGNOSTIC_help = "Check the health of the load cell"
    def cmd_LOAD_CELL_DIAGNOSTIC(self, gcmd):
        import numpy as np
        gcmd.respond_info(
            "Collecting load cell data for 10 seconds...")
        sps = self.load_cell.sensor.get_samples_per_second()
        collector = self.load_cell.get_collector()
        samples = collector.collect_samples(sps * 10)
        counts = np.asarray(samples)[:, 2].astype(int)
        min, max = self.load_cell.saturation_range()
        good_count = 0
        saturation_count = 0
        for sample in counts:
            if sample >= max or sample <= min:
                saturation_count += 1
            else:
                good_count += 1
        unique_counts = np.unique(counts)
        gcmd.respond_info("Samples Collected: %i" % (len(samples)))
        gcmd.respond_info(
            "Good samples: %i, Saturated samples: %i, Unique values: %i"
                          % (good_count, saturation_count, len(unique_counts)))
        max_pct = self.load_cell.counts_to_percent(np.amax(counts))
        min_pct = self.load_cell.counts_to_percent(np.amin(counts))
        gcmd.respond_info("Sample range: [%.2f%% to %.2f%%]"
                          % (min_pct, max_pct))
        gcmd.respond_info("Sample range / sensor capacity: %.5f%%"
                          % ((max_pct - min_pct) / 2.))
        #gcmd.respond_info("Errors: %i" % (total_errors))
        #if (total_errors > 0):
        #    gcmd.respond_info("Error breakdown: %s" % errors)

class LoadCellGuidedCalibrationHelper:
    def __init__(self, printer, load_cell):
        self.printer = printer
        self.gcode = printer.lookup_object('gcode')
        self.load_cell = load_cell
        self._tare_counts = self._counts_per_gram = None
        self.register_commands()
        self.gcode.respond_info(
            "Starting load cell calibration. \n"
            "1.) Remove all load and run TARE. \n"
            "2.) Apply a known load an run CALIBRATE GRAMS=nnn. \n"
            "Complete calibration with the ACCEPT command.\n"
            "Use the ABORT command to quit.")
    def register_commands(self):
        register_command = self.gcode.register_command
        register_command("ABORT", self.cmd_ABORT, desc=self.cmd_ABORT_help)
        register_command("ACCEPT", self.cmd_ACCEPT, desc=self.cmd_ACCEPT_help)
        register_command("TARE", self.cmd_TARE, desc=self.cmd_TARE_help)
        register_command("CALIBRATE", self.cmd_CALIBRATE, desc=self.cmd_CALIBRATE_help)
    def _avg_counts(self):
        try:
            return self.load_cell.avg_counts()
        except SaturationException:
            raise self.printer.command_error(
                "ERROR: Some load cell readings were saturated!\n"
                " (100% load). Use less force.")
        except SampleException:
            raise self.printer.command_error(
                "ERROR: Load cell failed to read reliably!\n "
                "Check the sensor's wiring and configuration.")
    # convert the delta of counts to a counts/gram metric
    def counts_per_gram(self, grams, cal_counts):
        return float(abs(int(self._tare_counts - cal_counts))) / grams
    # calculate min grams that the load cell can register
    # given tare bias, at saturation
    def capacity_kg(self, counts_per_gram):
        min, max = self.load_cell.saturation_range()
        return int((max - abs(self._tare_counts)) / counts_per_gram) / 1000.
    def finalize(self, save_results = False):
        for name in ['ABORT', 'ACCEPT', 'TARE', 'CALIBRATE']:
            self.gcode.register_command(name, None)
        if not save_results:
            self.gcode.respond_info("Load cell calibration aborted")
            return
        if self._counts_per_gram is None or self._tare_counts is None:
            self.gcode.respond_info("Calibration process is incomplete, "
                                     "aborting")
        self.load_cell.tare(self._tare_counts)
        self.load_cell.set_counts_per_gram(self._counts_per_gram)
        self.gcode.respond_info("Load cell calibrated, counts per gram: %.6f\n"
            "The SAVE_CONFIG command will update the printer config file "
            "with the above and restart the printer." % (self._counts_per_gram))
    cmd_ABORT_help = "Abort load cell calibration tool"
    def cmd_ABORT(self, gcmd):
        self.finalize(False)
    cmd_ACCEPT_help = "Accept calibration results and apply to load cell"
    def cmd_ACCEPT(self, gcmd):
        self.finalize(True)
    cmd_TARE_help = "Tare the load cell"
    def cmd_TARE(self, gcmd):
        self._tare_counts = self._avg_counts()
        self._counts_per_gram = None  #require re-calibration on tare
        self.tare_percent = self.load_cell.counts_to_percent(self._tare_counts)
        gcmd.respond_info("Load cell tare value: %.2f%% (%i)" 
                          % (self.tare_percent, self._tare_counts))
        if self.tare_percent > 2.:
            gcmd.respond_info(
                "WARNING: tare value is more than 2% away from 0!\n"
                "The load cell's range will be impacted.\n"
                "Check for external force on the load cell.")
        gcmd.respond_info("Now apply a known force to the load cell and enter \
                         the force value with:\n CALIBRATE GRAMS=nnn")
    cmd_CALIBRATE_help = "Enter the load cell value in grams"
    def cmd_CALIBRATE(self, gcmd):
        grams = gcmd.get_float("GRAMS", minval=50., maxval=25000.)
        cal_counts = self._avg_counts()
        cal_percent = self.load_cell.counts_to_percent(cal_counts)
        c_per_g = self.counts_per_gram(grams, cal_counts)
        cap_kg = self.capacity_kg(c_per_g)
        gcmd.respond_info("Calibration value: %.2f%% (%i), Counts/gram: %.5f, \
            Total capacity: +/- %0.2fKg"
                % (cal_percent, cal_counts, c_per_g, cap_kg))
        min, max = self.load_cell.saturation_range()
        if cal_counts >= max or cal_counts <= min:
            raise self.printer.command_error(
                "ERROR: Sensor is saturated with too much load!\n"
                "Use less force to calibrate the load cell.")
        if cal_counts == self._tare_counts:
            raise self.printer.command_error(
                "ERROR: Tare and Calibration readings are the same!\n"
                "Check wiring and validate sensor with READ_LOAD_CELL command.")
        if (abs(cal_percent - self.tare_percent)) < 1.:
            raise self.printer.command_error(
                "ERROR: Tare and Calibration readings are less than 1% "
                "different!\n"
                "Use more force when calibrating or a higher sensor gain.")
        # only set _counter_per_gram after all errors are raised
        self._counts_per_gram = c_per_g
        if cap_kg < 1.:
            gcmd.respond_info("WARNING: Load cell capacity is less than 1kg!\n"
                "Check wiring and consider using a lower sensor gain.")
        if cap_kg > 25.:
            gcmd.respond_info("WARNING: Load cell capacity is more than 25Kg!\n"
                "Check wiring and consider using a higher sensor gain.")
        gcmd.respond_info("Accept calibration with the ACCEPT command.")

# Utility to easily collect some samples for later analysis
# Optionally blocks execution while collecting with reactor.pause()
# can collect a minimum n samples or collect until a specific print time
# samples returned in [[time],[force],[counts]] arrays for easy processing
RETRY_DELAY = 0.01  # 100Hz
class LoadCellSampleCollector():
    def __init__(self, load_cell, reactor):
        self._load_cell = load_cell
        self._reactor = reactor
        self._mcu = load_cell.sensor.get_mcu()
        self.target_samples = 1
        self._samples = collections.deque()
        self._errors = collections.deque()
        self._client = None
    def _on_samples(self, data):
        self._samples.extend(data["params"]['samples'])
    def start_collecting(self):
        self.stop_collecting()
        self._samples.clear()
        self._errors.clear()
        self._client = self._load_cell.subscribe(self._on_samples)
    def stop_collecting(self):
        if self.is_collecting():
            self._client.close()
            self._client = None
    def get_samples(self):
        return copy.copy(self._samples)
    def is_collecting(self):
        return self._client is not None
    # return true when the last sample has a time after the target time
    def _time_test(self, print_time):
        def test():
            return (not len(self._samples) == 0) \
                        and self._samples[-1][0] >= print_time
        return test
    # return true when a set number of samples have been collected
    def _count_test(self, target):
        def test():
            return len(self._samples) >= target
        return test
    def _collect_until_test(self, test, timeout):
        if not self.is_collecting():
            self.start_collecting()
        while self.is_collecting() and not test():
            now = self._reactor.monotonic()
            print_time = self._mcu.estimated_print_time(now)
            if (print_time > timeout):
                logging.error("LoadCellSampleCollector.collect_? timed out")
                break
            self._reactor.pause(now + RETRY_DELAY)
        self.stop_collecting()
        return self.get_samples()
    # block execution until n samples are collected
    def collect_samples(self, samples=None):
        target = self.target_samples if samples == None else samples
        now = self._reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now)
        timeout = print_time + 1. + \
            (target / self._load_cell.sensor.get_samples_per_second())
        return self._collect_until_test(self._count_test(target), timeout)
    # block execution until a sample is returned with a timestamp after time
    def collect_until(self, time=None):
        now = self._reactor.monotonic()
        target = now if time == None else time
        timeout = 1. + target
        return self._collect_until_test(self._time_test(target), timeout)

class LoadCellPrinterProbe(PrinterProbe):
    def __init__(self, config, mcu_probe, load_cell):
        super(LoadCellPrinterProbe, self).__init__(config, mcu_probe)
        self._load_cell = load_cell
        self.pullback_distance = 0.1
        self.pullback_speed = 400. * 0.001
        self.collector = load_cell.get_collector()
    def _probe(self, speed):
        self._load_cell.continuous_tare()
        self.collector.start_collecting()
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        phoming = self.printer.lookup_object('homing')
        pos = toolhead.get_position()
        pos[2] = self.z_position
        try:
            epos = phoming.probing_move(self.mcu_probe, pos, speed)
        except self.printer.command_error as e:
            self.collector.stop_collecting()
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)
        
        homing_end_time = self.last_move_time()
        pullback_end_time = self.pullback_move()
        ppa = ProbePointAnalysis(self.printer
                                 , homing_end_time, pullback_end_time)
        ppa.set_samples(self.collector.collect_until(pullback_end_time))
        z_point = ppa.analyze()
        epos[2] = z_point
        self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
        return epos[:3]
    def pullback_move(self):
        toolhead = self.printer.lookup_object('toolhead')
        halt_pos = toolhead.get_position()
        pullback_pos = list(halt_pos)
        pullback_pos[2] += self.pullback_distance
        toolhead.move(pullback_pos, self.pullback_speed)
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        pullback_end_pos = toolhead.get_position()
        with open('/home/pi/printer_data/logs/loadcell.log', 'a') as log:
            log.write("\"trigger_z\": %s," % (halt_pos[2]))
            log.write("\"pullback_end_time\": %s," % (pullback_end))
            log.write("\"pullback_trigger_z\": %s,}," % (pullback_end_pos[2]))
        return pullback_end
    def last_move_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_last_move_time()

# Printer class that controls the load cell
class LoadCell:
    def __init__(self, config):
        try:
            import numpy
        except Exception:
            raise config.error("LoadCell requires the numpy module")
        self.printer = printer = config.get_printer()
        self.name = name = config.get_name()
        sensor_name = config.get('sensor')
        self.sensor = sensor = printer.lookup_object(sensor_name)
        self.configfile = self.printer.lookup_object('configfile')
        self.load_cell_endstop = None
        if config.getboolean('is_probe', default=False):
            self.load_cell_endstop = \
                load_cell_endstop.LoadCellEndstop(config, self)
            printer.add_object('load_cell_endstop:' + name
                        , self.load_cell_endstop)
            sensor.attach_load_cell_endstop(self.load_cell_endstop.get_oid())
            printer.add_object('probe'
                        , LoadCellPrinterProbe(config, self.load_cell_endstop, self))
        # sensor must implement LoadCellDataSource
        self.tare_counts = None
        self.trailing_counts = collections.deque(maxlen=24)
        self.is_in_use = False
        self.in_use_print_time = 0
        self.counts_per_gram = config.getfloat('counts_per_gram', minval=1.
                                            , default=None)
        LoadCellCommandHelper(config, self)
        self.trigger_force_grams = config.getfloat('trigger_force_grams'
                                        , minval=10., maxval=250, default=50.)
        # webhooks support
        self.api_dump = WebhooksApiDumpRepeater(printer, "load_cell/dump_force",
                        "load_cell", name,
                        {"header": ["time", "force (g)", "counts"]})
        # startup, when klippy is ready, start capturing data
        printer.register_event_handler("klippy:ready", self._handle_ready)
    def _handle_ready(self):
        self.sensor_client = self.sensor.subscribe(self._sensor_data_event)
    # convert raw counts to grams and broadcast to clients
    def _sensor_data_event(self, data):
        if not data.get("params", {}).get("samples"):
            return
        samples = []
        
        for row in data["params"]["samples"]:
            # [time, grams, counts]
            samples.append([row[0],
                            self.counts_to_grams(row[1]),
                            row[1]])
        self.api_dump.send({'samples': samples})
        
    def send_endstop_event(self, endstop_data):
        self.api_dump.send({'endstop': endstop_data})
    # get internal events of force data
    def subscribe(self, callback):
        client = multiplex_adc.EventDumpClient(callback)
        self.api_dump.add_client(multiplex_adc.WebRequestShim(client))
        return client
    def tare(self, tare_counts):
        self.tare_counts = tare_counts
        self.set_endstop_range()
    def get_tare_counts(self):
        return self.tare_counts
    def set_counts_per_gram(self, counts_per_gram):
        if counts_per_gram is None or abs(counts_per_gram) < 1.:
            raise self.printer.command_error("Invalid counts per gram value")
        self.counts_per_gram = abs(counts_per_gram)
        self.set_endstop_range()
        self.configfile.set(self.name, 'counts_per_gram',
                             "%.5f" % (counts_per_gram))
    def set_trigger_force(self, trigger_force_grams):
        self.trigger_force_grams = trigger_force_grams
    def set_endstop_range(self):
        if not self.load_cell_endstop or not self.is_calibrated():
            return
        trigger_counts = int(self.trigger_force_grams * self.counts_per_gram)
        self.load_cell_endstop.set_range(trigger_counts, self.tare_counts)
    def counts_to_grams(self, sample):
        if not self.is_calibrated():
            return None
        return float(sample - self.tare_counts) / (self.counts_per_gram)
    def saturation_range(self):
        size = pow(2, (self.get_bits() - 1))
        return -1 * (size), (size - 1)
    def counts_to_percent(self, counts):
        min, max = self.saturation_range()
        return (float(counts) / float(max)) * 100.
    # grab 1 second of load cell data and average it
    # performs safety checks for saturation and under-counts
    def avg_counts(self, num_samples=None):
        import numpy as np
        if num_samples is None:
            num_samples = self.sensor.get_samples_per_second()
        samples = self.get_collector().collect_samples(num_samples)
        # check samples for saturated readings
        min, max = self.saturation_range()
        for sample in samples:
            # TODO: what if there were errors while sampling?
            if sample[2] >= max or sample[2] <= min:
                raise SaturationException(
                    "Some samples are saturated (+/-100%)")
        counts = np.asarray(samples)[:, 2].astype(float)
        return np.average(counts)
    def _last_move_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_last_move_time()
    def continuous_tare(self):
        import numpy as np
        collector = self.get_collector()
        # collect and discard samples up to the end of the current
        collector.collect_until(self._last_move_time())
        # then collect the next n samples
        samples = collector.collect_samples(16)
        tare_counts = int(np.average(samples, axis=0)[2])
        self.tare(tare_counts)
    # I swear on a stack of dictionaries this is correct english...
    def is_tared(self):
        return self.tare_counts is not None
    def is_calibrated(self):
        return self.is_tared() and self.counts_per_gram is not None
    def get_sensor(self):
        return self.sensor
    def get_bits(self):
        return self.sensor.sensor.get_bits()
    def get_collector(self):
        return LoadCellSampleCollector(self, self.printer.get_reactor())
    def get_status(self, eventtime):
        return {'is_calibrated': self.is_calibrated()
                , 'tare_counts': self.tare_counts
                , 'counts_per_gram': self.counts_per_gram
                , 'is_probe': self.load_cell_endstop is not None}

def load_config_prefix(config):
    return LoadCell(config)
