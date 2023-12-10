# Load Cell Implementation
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import collections
from . import probe, load_cell_endstop, multiplex_adc
import chelper
from probe import PrinterProbe

class SaturationException(Exception):
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

# point on a time/force graph
class ForcePoint(object):
    def __init__(self, time, force):
        self.time = time
        self.force = force

# compute the time some value was surpassed
def index_near(x, x_val):
    import numpy as np
    return np.argmax(np.asarray(x) >= x_val) or len(x) -1 

# finds the intersection of 2 groups of points
def intersection(x1, y1, x2, y2):
    line1 = lstsq_line(x1, y1)
    line2 = lstsq_line(x2, y2)
    return line_intersection(line1, line2)

# finds the intersection of lines
def line_intersection(line1, line2):
    m1, b1 = line1
    m2, b2 = line2
    numerator = -b1 + b2
    denominator = m1 - m2
    intersection_time = numerator / denominator
    intersection_force = find_y(line1, intersection_time)
    return ForcePoint(intersection_time, intersection_force)

# given a line, return y coordinate for some x value
def find_y(line, x_value):
    mx, b = line
    return mx * x_value + b

# Least Squares on x[] y[] points, returns line = (mx, b)
def lstsq_line(x, y):
    import numpy as np
    x = np.asarray(x)
    y = np.asarray(y)
    x_stacked = np.vstack([x, np.ones(len(x))]).T
    return np.linalg.lstsq(x_stacked, y, rcond=None)[0]

# Kneedle algorithm
def _kneedle(x, y, x_coords, y_coords):
    import numpy as np
    line = lstsq_line(x_coords, y_coords)
    # now compute the Y of the line value for every x
    fit_y = []
    for time in x:
        fit_y.append(find_y(line, time))
    fit_y = np.asarray(fit_y)
    delta_y = fit_y - y
    max_i = np.argmax(delta_y)
    min_i = np.argmin(delta_y)
    #return (max_i, min_i)
    # data may have either polarity, so the order is non deterministic
    # always returns elbows in left-to-right order
    return (min(max_i, min_i), max(max_i, min_i))
    

# Find Kneedle with optional additional width
def find_kneedle(x, y, additional_width = 0.0):
    start = x[0]
    end = x[-1]
    extension = (end - start) * additional_width
    start = start - extension
    end = end + extension
    x_coords = [start, end]
    y_coords = [y[0], y[-1]]
    return _kneedle(x, y, x_coords, y_coords)

# split a group of points into 3 lines using 2 junction points
def split_to_lines(x, y, j1, j2, discard=0):
    l1 = lstsq_line(x[0 + discard : j1 - discard],
                    y[0 + discard : j1 - discard])
    l2 = lstsq_line(x[j1 + discard : j2 - discard],
                    y[j1 + discard : j2 - discard])
    l3 = lstsq_line(x[j2 + discard : -1 - discard],
                    y[j2 + discard : -1 - discard])
    return (l1, l2, l3)

# split a line into 3 parts by elbows. Return lines and intersection points
def elbow_split(x, y, discard=0, additional_width=0):
    left_elbow_idx, right_elbow_idx = find_kneedle(x, y, additional_width)
    # try to leave all points in the middle segment
    l1, l2, l3 = split_to_lines(x, y, left_elbow_idx - 1,
                                right_elbow_idx + 1, discard)
    i1 = line_intersection(l1, l2)
    i2 = line_intersection(l2, l3)
    # return (line, point, line, point, line)
    return l1, i1, l2, i2, l3

# break a tap event down into 6 points and 5 lines:
#    *-----*|       /*-----*
#           |      /
#           *----*/
def tap_decompose(time, force, homing_end_idx, pullback_start_idx,
                  discard=0, additional_width=0):
    start_x = time[0: pullback_start_idx - 1]
    start_y = force[0: pullback_start_idx - 1]
    # discard=0 because there are so few points in the probe line
    l1, i1, l2, i2, l3 = elbow_split(start_x, start_y, 0, additional_width)
    start = ForcePoint(time[0], find_y(l1, time[0]))
    homing_end_idx = index_near(time, i2.time) + discard
    # the points after homing stop are very noisy, exclude half of them
    midpoint = homing_end_idx + ((pullback_start_idx - homing_end_idx) // 2)
    end_x = time[midpoint: -1]
    end_y = force[midpoint: -1]
    _, i3, l4, i4, l5 = elbow_split(end_x, end_y, discard, additional_width)
    end = ForcePoint(time[-1], find_y(l5, time[-1]))
    return (start, i1, i2, i3, i4, end), (l1, l2, l3, l4, l5)

class TrapqMove(object):
    def __init__(self, move):
        # copy cdata to python memory
        self.print_time = float(move.print_time)
        self.move_t = float(move.move_t)
        self.start_v = float(move.start_v)
        self.accel = float(move.accel)
        self.start_x = float(move.start_x)
        self.start_y = float(move.start_y)
        self.start_z = float(move.start_z)
        self.x_r = float(move.x_r)
        self.y_r = float(move.y_r)
        self.z_r = float(move.z_r)
    def to_string(self):
        return ("print_time=%.6f move_t=%.6f start_v=%.6f accel=%.6f"
            " start_pos=(%.6f,%.6f,%.6f) ar=(%.6f,%.6f,%.6f)"
            % (self.print_time, self.move_t, self.start_v, self.accel,
                self.start_x, self.start_y, self.start_z, self.x_r,
                self.y_r, self.z_r))

def trapq_move_to_dict(move):
    return {'print_time': float(move.print_time),
            'move_t': float(move.move_t),
            'start_v': float(move.start_v),
            'accel': float(move.accel),
            'start_x': float(move.start_x),
            'start_y': float(move.start_y),
            'start_z': float(move.start_z),
            'x_r': float(move.x_r),
            'y_r': float(move.y_r),
            'z_r': float(move.z_r)
        }

#TODO: maybe discard points can scale with sample rate from 1 to 3
DISCARD_POINTS = 3
ADDITIONAL_WIDTH = 1.0
class TapAnalysis(object):
    def __init__(self, printer, samples):
        import numpy as np
        self.printer = printer
        self.trapq = printer.lookup_object('motion_report').trapqs['toolhead']
        # TODO: tune this based on SPS and speed
        self.discard = DISCARD_POINTS
        #REVIEW: does this factor need to be configuration?
        self.additional_width = ADDITIONAL_WIDTH
        np_samples = np.array(samples)
        self.time = np_samples[:, 0]
        self.force = np_samples[:, 1]
        self.moves = self.get_moves()
        self.home_end_time = self.moves[2]['print_time']
        self.pullback_start_time = self.moves[3]['print_time']
        self.fix_homing_end()
        self.pos = self.get_toolhead_positions()
        self.tap_points = None
        self.tap_lines = None
        #self.log_moves()
    # build toolhead Z position for the time/force points
    def get_toolhead_positions(self):
        z_pos = []
        for time in self.time:
            z_pos.append(self.get_toolhead_position(time))
        return z_pos
    def get_toolhead_position(self, print_time):
        for i, move in enumerate(self.moves):
            start_time = move['print_time']
            # time before first move, printer was stationary
            if i == 0 and print_time < start_time:
                return move['start_z']
            end_time = float('inf')
            if i < (len(self.moves) - 1):
                end_time = self.moves[i + 1]['print_time']
            if print_time >= start_time and print_time < end_time:
                # we have found the move
                move_t = move['move_t']
                move_time = max(0., 
                        min(move_t, print_time - move['print_time']))
                dist = ((move['start_v'] + .5 * move['accel'] * move_time)
                            * move_time)
                pos = ((move['start_x'] + move['x_r'] * dist,
                        move['start_y'] + move['y_r'] * dist,
                        move['start_z'] + move['z_r'] * dist))
                return pos
            else:
                continue
        raise Exception("Move not found, thats impossible!")
    # adjust move_t of move 1 to match the toolhead position of move 2
    def fix_homing_end(self):
        # REVIEW: This takes some logical shortcuts, does it need to be more
        # generalized? e.g. to all 3 axes?
        homing_move = self.moves[1]
        # acceleration should be 0!
        accel = homing_move['accel']
        if (accel != 0.):
            raise Exception('Unexpected acceleration in probing move')
        start_v = homing_move['start_v']
        start_z = homing_move['start_z']
        end_z = self.moves[2]['start_z']
        # how long did it take to get to end_z?
        move_t = abs((end_z - start_z) / start_v)
        self.home_end_time = homing_move['print_time'] + move_t
        self.moves[1]['move_t'] = move_t
    def log_moves(self):
        for i, move in enumerate(self.moves):
            logging.info("Move %s: %s" % (i, move))
    def get_moves(self):
        moves, cdata  = self.trapq.extract_trapq(self.time[0], self.time[-1])
        moves_out = []
        for move in moves:
            moves_out.append(trapq_move_to_dict(move))
        # it could be 5, in theory, but if it is, thats bad
        if (len(moves_out) != 6):
            raise Exception("Expected 6 moves from trapq")
        return moves_out
    def analyze(self):
        discard = self.discard
        additional_width = self.additional_width
        force = self.force
        time = self.time
        homing_end_index = index_near(time, self.home_end_time)
        pullback_start_index = index_near(time, self.pullback_start_time)
        points, lines = tap_decompose(time, force, homing_end_index,
                            pullback_start_index, discard, additional_width)
        self.tap_points = points
        self.tap_lines = lines
        p1, p2, p3, p4, p5, p6 = points
        pos = self.get_toolhead_position(p5.time)
        return pos[2]
    def get_tap_data(self):
        p1, p2, p3, p4, p5, p6 = self.tap_points
        return {
            'graph': {
                'time': self.time.tolist(),
                'force': self.force.tolist(),
                'position': None,
            },
            'points': [
                {'time': p1.time, 'force': p1.force},
                {'time': p2.time, 'force': p2.force},
                {'time': p3.time, 'force': p3.force},
                {'time': p4.time, 'force': p4.force},
                {'time': p5.time, 'force': p5.force},
                {'time': p6.time, 'force': p6.force}
            ],
            'home_end_time': self.home_end_time,
            'pullback_start_time': self.pullback_start_time,
        }

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
            "2.) Apply a known load, run CALIBRATE GRAMS=nnn. \n"
            "Complete calibration with the ACCEPT command.\n"
            "Use the ABORT command to quit.")
    def register_commands(self):
        register_command = self.gcode.register_command
        register_command("ABORT", self.cmd_ABORT, desc=self.cmd_ABORT_help)
        register_command("ACCEPT", self.cmd_ACCEPT, desc=self.cmd_ACCEPT_help)
        register_command("TARE", self.cmd_TARE, desc=self.cmd_TARE_help)
        register_command("CALIBRATE", self.cmd_CALIBRATE
                                                , desc=self.cmd_CALIBRATE_help)
    def _avg_counts(self):
        try:
            return self.load_cell.avg_counts()
        except SaturationException:
            raise self.printer.command_error(
                "ERROR: Some load cell readings were saturated!\n"
                " (100% load). Use less force.")
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
# can collect a minimum n samples or collect until a specific print_time
# samples returned in [[time],[force],[counts]] arrays for easy processing
RETRY_DELAY = 0.05  # 20Hz
class LoadCellSampleCollector():
    def __init__(self, load_cell, reactor):
        self._load_cell = load_cell
        self._reactor = reactor
        self._mcu = load_cell.sensor.get_mcu()
        self.min_time = 0.
        self.max_time = float("inf")
        self.finished = False
        self._samples = collections.deque()
        self._client = None
    def _on_samples(self, data):
        samples = data["params"]['samples']
        for sample in samples:
            time = sample[0]
            if time >= self.min_time and time <= self.max_time:
                self._samples.append(sample)
            if time > self.max_time:
                self.stop_collecting()
    def start_collecting(self, min_time=0., max_time=float("inf")):
        self.stop_collecting()
        self._samples.clear()
        self.min_time = min_time
        self.max_time = max_time
        self.finished = False
        self._client = self._load_cell.subscribe(self._on_samples)
    def stop_collecting(self):
        if self.is_collecting():
            self._client.close()
            self._client = None
    def get_samples(self):
        return list(self._samples)
    def is_collecting(self):
        return self._client is not None
    # return true when the last sample has a time after the target time
    def _time_test(self, print_time):
        collector = self
        def test():
            return not collector.is_collecting()
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
                raise Exception("LoadCellSampleCollector timed out")
            self._reactor.pause(now + RETRY_DELAY)
        self.stop_collecting()
        return self.get_samples()
    # block execution until at least n samples are collected
    def collect_samples(self, samples=1):
        now = self._reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now)
        timeout = print_time + 1. + \
            (samples / self._load_cell.sensor.get_samples_per_second())
        return self._collect_until_test(self._count_test(samples), timeout)
    # block execution until a sample is returned with a timestamp after time
    def collect_until(self, time=None):
        now = self._reactor.monotonic()
        target = now if time is None else time
        if not self.is_collecting():
            self.start_collecting(max_time=target)
        self.max_time = target
        timeout = 1. + target
        return self._collect_until_test(self._time_test(target), timeout)

class BadTapModule(object):
    def is_bad_tap(self, tap_data):
        return False

class NozzleCleanerModule(object):
    def clean_nozzle(self):
        pass

NOZZLE_CLEANER = "{action_respond_info(\"Bad tap detected, nozzle needs" \
        " cleaning. nozzle_cleaner_gcode not configured!\")}"
class LoadCellPrinterProbe(PrinterProbe):
    def __init__(self, config, load_cell_endstop, load_cell):
        super(LoadCellPrinterProbe, self).__init__(config, load_cell_endstop)
        self.load_cell = load_cell
        self.load_cell_endstop = load_cell_endstop
        printer = config.get_printer()
        self._load_cell = load_cell
        self.pullback_dist = config.getfloat('pullback_dist'
                                        , minval=0.01, maxval=2.0, default=0.1)
        sps = load_cell.sensor.get_samples_per_second()
        default_pullback_speed = sps * 0.001
        #TODO: Math: set the minimum pullback speed such that at least enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        self.pullback_speed = config.getfloat('pullback_speed'
                    , minval=0.01, maxval=1.0, default=default_pullback_speed)
        self.pullback_distance = 0.1
        self.pullback_speed = 400. * 0.001
        self.bad_tap_module = self.load_module(config
                            , 'bad_tap_module', BadTapModule())
        self.nozzle_cleaner_module = self.load_module(config
                            , 'nozzle_cleaner_module', None)
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.nozzle_cleaner_gcode = gcode_macro.load_template(config,
                                'nozzle_cleaner_gcode', NOZZLE_CLEANER)
        self.collector = None
        self.printer.register_event_handler(
            "load_cell_endstop:homing_start_time"
            , self._handle_homing_start_time)
    def load_module(self, config, name, default):
        module = config.get(name, default=None)
        return default if module is None else self.printer.lookup_object(module)
    def _handle_homing_start_time(self, print_time):
        if self.collector is not None:
            self.collector.start_collecting(print_time)
    def check_tap(self, tap_data):
        if self.bad_tap_module.is_bad_tap(tap_data):
            #TODO: what params to pass to nozzle cleaners?
                # X,Y,Z of the failed probe?
                # original requested probe location
                # how many times this has happened?
            if self.nozzle_cleaner_module is not None:
                self.nozzle_cleaner_module.clean_nozzle()
            else:
                macro = self.nozzle_cleaner_gcode
                context = macro.create_template_context()
                context['params'] = []
                macro.run_gcode_from_command(context)
    def _probe(self, speed):
        self.collector = self.load_cell.get_collector()
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
            self.collector = None
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)
        pullback_end_time = self.pullback_move()
        samples = self.collector.collect_until(pullback_end_time)
        self.collector = None
        ppa = TapAnalysis(self.printer, samples)
        z_point = ppa.analyze()
        tap_data = ppa.get_tap_data()
        self.load_cell.send_endstop_event(tap_data)
        is_bad = self.check_tap(tap_data)
        if not is_bad:
            epos[2] = z_point
            self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
            return epos[:3]
        else:
            #TODO: if tap is bad, prevent probe from using it
            raise Exception("Bad tap handling not implemented")
    def pullback_move(self):
        toolhead = self.printer.lookup_object('toolhead')
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self.pullback_distance
        toolhead.move(pullback_pos, self.pullback_speed)
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end

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
                        , LoadCellPrinterProbe(config, self.load_cell_endstop
                                               , self))
        # sensor must implement LoadCellDataSource
        self.tare_counts = None
        self.trailing_counts = collections.deque(maxlen=24)
        self.is_in_use = False
        self.in_use_print_time = 0
        self.counts_per_gram = config.getfloat('counts_per_gram', minval=1.
                                            , default=None)
        LoadCellCommandHelper(config, self)
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
    def set_endstop_range(self):
        if not self.load_cell_endstop \
            or not self.is_calibrated() or not self.is_tared():
            return
        self.load_cell_endstop.set_range(self.tare_counts, self.counts_per_gram)
    def counts_to_grams(self, sample):
        if not self.is_calibrated() or not self.is_tared():
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
            if sample[2] >= max or sample[2] <= min:
                raise SaturationException(
                    "Some samples are saturated (+/-100%)")
        counts = np.asarray(samples)[:, 2].astype(float)
        return np.average(counts)    
    # pauses for the last move to complete and then tares the loadcell
    # returns the last sample record used in taring
    def pause_and_tare(self):
        import numpy as np
        toolhead = self.printer.lookup_object('toolhead')
        collector = self.get_collector()
        # collect and discard samples up to the end of the current
        collector.collect_until(toolhead.get_last_move_time())
        # then collect the next n samples
        # Collect 4x60hz power cycles of data to average across power noise
        sps = self.sensor.get_samples_per_second()
        num_samples = max(2, round(sps * ((1 / 60) * 4)))
        tare_samples = collector.collect_samples(num_samples)
        tare_counts = np.average(np.array(tare_samples)[:,2].astype(float))
        self.tare(tare_counts)
    # I swear on a stack of dictionaries this is correct english...
    def is_tared(self):
        return self.tare_counts is not None
    def is_calibrated(self):
        return self.counts_per_gram is not None
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
