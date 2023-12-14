# Load Cell Implementation
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import collections
import math
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
    
class ForceLine(object):
    def __init__(self, slope, intercept):
        self.slope = slope
        self. intercept = intercept
    # measure angles between lines at the 1g == 1ms scale
    # returns +/- 0-180. Positive values represent clockwise rotation
    def angle(self, line, time_scale=0.001):
        radians = (math.atan2(self.slope * time_scale, 1) - 
                    math.atan2(line.slope * time_scale, 1))
        return math.degrees(radians)
    def find_force(self, time):
        return self.slope * time + self.intercept
    def find_time(self, force):
        return (force - self.intercept) / self.slope
    def intersection(self, line):
        numerator = -self.intercept + line.intercept
        denominator = self.slope - line.slope
        intersection_time = numerator / denominator
        intersection_force = self.find_force(intersection_time)
        return ForcePoint(intersection_time, intersection_force)

# compute the index in the time array when some value was surpassed
def index_near(time, instant):
    import numpy as np
    return int(np.argmax(np.asarray(time) >= instant) or len(time) -1)

# Least Squares on x[] y[] points, returns ForceLine
def lstsq_line(x, y):
    import numpy as np
    x = np.asarray(x)
    y = np.asarray(y)
    x_stacked = np.vstack([x, np.ones(len(x))]).T
    mx, b = np.linalg.lstsq(x_stacked, y, rcond=None)[0]
    return ForceLine(mx, b)

# Kneedle algorithm, finds min and max elbow points
def find_kneedles(x, y):
    import numpy as np
    x_coords = [x[0], x[-1]]
    y_coords = [y[0], y[-1]]
    line = lstsq_line(x_coords, y_coords)
    # now compute the Y of the line value for every x
    fit_y = []
    for time in x:
        fit_y.append(line.find_force(time))
    fit_y = np.asarray(fit_y)
    delta_y = fit_y - y
    max_i = np.argmax(delta_y)
    min_i = np.argmin(delta_y)
    # data may have either polarity, so the order is non deterministic
    # always returns elbows in left-to-right order
    return (min(max_i, min_i), max(max_i, min_i))

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
def elbow_split(x, y, discard=0):
    left_elbow_idx, right_elbow_idx = find_kneedles(x, y)
    # try to leave all points in the middle segment
    l1, l2, l3 = split_to_lines(x, y, left_elbow_idx - 1,
                                right_elbow_idx + 1, discard)
    i1 = l1.intersection(l2)
    i2 = l2.intersection(l3)
    # return (line, point, line, point, line)
    return l1, i1, l2, i2, l3

# break a tap event down into 6 points and 5 lines:
#    *-----*|       /*-----*
#           |      /
#           *----*/
def tap_decompose(time, force, homing_end_idx, pullback_start_idx,
                  discard=0):
    start_x = time[0: pullback_start_idx - 1]
    start_y = force[0: pullback_start_idx - 1]
    # discard=0 because there are so few points in the probe line
    l1, i1, l2, i2, l3 = elbow_split(start_x, start_y, 0)
    start = ForcePoint(time[0], l1.find_force(time[0]))
    homing_end_idx = index_near(time, i2.time) + discard
    # the points after homing stop are very noisy, exclude 1/5th of them
    dwell_start = homing_end_idx + ((pullback_start_idx - homing_end_idx) // 5)
    end_x = time[dwell_start: -1]
    end_y = force[dwell_start: -1]
    _, i3, l4, i4, l5 = elbow_split(end_x, end_y, discard)
    end = ForcePoint(time[-1], l5.find_force(time[-1]))
    return [start, i1, i2, i3, i4, end], [l1, l2, l3, l4, l5]

# calculate variance between a ForceLine and a region of force data
def segment_variance(force, time, start, end, line):
    import numpy as np
    mean = np.average(force[start:end])
    total_var = 0
    delta_var = 0
    for i in range(start, end):
        load = force[i]
        instant = time[i]
        total_var += pow(load - mean, 2)
        delta_var += pow(line.find_force(instant) - load, 2)
    return (total_var, delta_var)

# decide how well the ForceLines predict force near an elbow
# bad predictions == blunt elbow, good predictions == sharp elbow
def elbow_r_squared(force, time, elbow_idx, widths, left_line, right_line):
    r_squared = []
    for width in widths:
        l_tv, l_dv = segment_variance(force, time,
                                elbow_idx - width, elbow_idx, left_line)
        r_tv, r_dv = segment_variance(force, time,
                                elbow_idx, elbow_idx + width, right_line)
        r2 = 1 - ((l_dv + r_dv) / (l_tv + r_tv))
        # returns r squared as a percentage -350% is bad. 80% is good!
        r_squared.append(round((r2 * 100.), 1))
    return r_squared

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
class TapAnalysis(object):
    def __init__(self, printer, samples):
        import numpy as np
        self.printer = printer
        self.trapq = printer.lookup_object('motion_report').trapqs['toolhead']
        # TODO: tune this based on SPS and speed
        self.discard = DISCARD_POINTS
        np_samples = np.array(samples)
        self.time = np_samples[:, 0]
        self.force = np_samples[:, 1]
        self.moves = self.get_moves()
        self.home_end_time = self.moves[2]['print_time']
        self.pullback_start_time = self.moves[3]['print_time']
        self.fix_homing_end()
        self.pos = self.get_toolhead_positions()
        self.is_valid = False
        self.tap_pos = None
        self.tap_points = None
        self.tap_lines = None
        self.tap_angles = None
        self.tap_r_squared = None
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
                return (move['start_x'], move['start_y'], move['start_z'])
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
        moves, _ = self.trapq.extract_trapq(self.time[0], self.time[-1])
        moves_out = []
        for move in moves:
            moves_out.append(trapq_move_to_dict(move))
        # it could be 5, in theory, but if it is, thats bad
        if (len(moves_out) != 6):
            raise Exception("Expected 6 moves from trapq")
        return moves_out
    def analyze(self):
        discard = self.discard
        force = self.force
        time = self.time
        homing_end_index = index_near(time, self.home_end_time)
        pullback_start_index = index_near(time, self.pullback_start_time)
        points, lines = tap_decompose(time, force, homing_end_index,
                            pullback_start_index, discard)
        self.tap_points = points
        self.tap_lines = lines
        if not self.validate_order():
            logging.info('Tap failed chronology check')
            return
        self.tap_angles = self.calculate_angles()
        if not self.validate_elbow_rotation():
            logging.info('Tap failed elbow rotation check')
            return
        break_contact_time = points[4].time
        if (break_contact_time < self.pullback_start_time 
            or break_contact_time > time[-1]):
            logging.info('Tap break-contact time is invalid')
            return
        self.tap_pos = self.get_toolhead_position(break_contact_time)
        self.tap_r_squared = self.calculate_r_squared()
        self.is_valid = True
    # validate that a set of ForcePoint objects are in chronological order
    def validate_order(self):
        p = self.tap_points
        return p[0].time < p[1].time < p[2].time < p[3].time
    def calculate_angles(self):
        l1, l2, l3, l4, l5 = self.tap_lines
        return [l1.angle(l2), l2.angle(l3), l3.angle(l4), l4.angle(l5)]
    # Validate that the rotations in the graph form a tap shape
    def validate_elbow_rotation(self):
        a1, a2, a3, a4 = self.tap_angles
        # with two polarities there are 2 valid tap shapes:
        return ((a1 > 0 and a2 < 0 and a3 < 0 and a4 > 0) or
                (a1 < 0 and a2 > 0 and a3 > 0 and a4 < 0))
    def calculate_r_squared(self):
        import numpy as np
        sample_time = np.average(np.diff(self.time))
        widths = [int((n * 0.01) // sample_time) for n in range(2, 7)]
        r_squared = []
        for i, elbow in enumerate(self.tap_points[1 : -1]):
            elbow_idx = index_near(self.time, elbow.time)
            logging.info("calculate r^2 for: %s, elbow_idx: %s, elbow_time: %s, time: [%s ... %s]" 
                     % (i, elbow_idx, elbow.time, self.time[0], self.time[-1]))
            r_squared.append(elbow_r_squared(self.force, self.time, elbow_idx,
                            widths, self.tap_lines[i], self.tap_lines[i + 1]))
        return r_squared
    def get_tap_data(self):
        tap_pos = None
        if self.tap_pos is not None:
            tap_pos = {'x': self.tap_pos[0],
                        'y': self.tap_pos[1],
                        'z': self.tap_pos[2],
            }
        return {
            'graph': {
                'time': self.time.tolist(),
                'force': self.force.tolist(),
                'position': None,
            },
            'points': [
                {'time': point.time, 'force': point.force}
                    for point in self.tap_points
            ],
            'lines': [
                {'slope': line.slope, 'intercept': line.intercept} 
                    for line in self.tap_lines
            ],
            'tap_pos': tap_pos,
            'home_end_time': self.home_end_time,
            'pullback_start_time': self.pullback_start_time,
            'angles': self.tap_angles,
            'r_squared': self.tap_r_squared,
            'is_valid': str(self.is_valid),
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
        #TODO: Math: set the minimum pullback speed such that at least 
        # enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        self.pullback_speed = config.getfloat('pullback_speed'
                    , minval=0.01, maxval=1.0, default=default_pullback_speed)
        self.pullback_extra_time = config.getfloat('pullback_extra_time'
                    , minval=0.00, maxval=1.0, default=0.3)
        self.pullback_distance = 0.1
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
    def clean_nozzle(self, retries):
        #TODO: what params to pass to nozzle cleaners?
        # X,Y,Z of the failed probe?
        # original requested probe location
        # how many times this has happened?
        if self.nozzle_cleaner_module is not None:
            self.nozzle_cleaner_module.clean_nozzle()
        else:
            macro = self.nozzle_cleaner_gcode
            context = macro.create_template_context()
            context['params'] = {
                'RETRIES': retries,
            }
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
        pullback_end_pos = toolhead.get_position()
        samples = self.collector.collect_until(pullback_end_time 
                                               + self.pullback_extra_time)
        self.collector = None
        ppa = TapAnalysis(self.printer, samples)
        ppa.analyze()
        tap_data = ppa.get_tap_data()
        self.load_cell.send_endstop_event(tap_data)
        if ppa.is_valid:
            is_bad = self.bad_tap_module.is_bad_tap(tap_data)
            if not is_bad:
                epos[2] = ppa.tap_pos[2]
                self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
                return epos[:3], pullback_end_pos[:3]
        return None, pullback_end_pos[:3]
    def pullback_move(self):
        toolhead = self.printer.lookup_object('toolhead')
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self.pullback_distance
        toolhead.move(pullback_pos, self.pullback_speed)
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end
    def run_probe(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin()
        probexy = self.printer.lookup_object('toolhead').get_position()[:2]
        retries = 0
        positions = []
        while len(positions) < sample_count:
            # Probe position
            logging.info("probe start")
            probe_pos, halt_pos = self._probe(speed)
            logging.info("probe complete")
            # handle bad taps by cleaning and retry:
            if probe_pos is None:
                if retries >= samples_retries:
                    raise gcmd.error("Probe had too many retries")
                gcmd.respond_info("Bad tap detected. Cleaning Nozzle...")
                # tap was invalid or bad, clean nozzle
                self.clean_nozzle(retries)
                gcmd.respond_info("Retrying Probe...")
                retries += 1
            else:
                positions.append(probe_pos)
                # Check samples tolerance
                z_positions = [p[2] for p in positions]
                if max(z_positions) - min(z_positions) > samples_tolerance:
                    if retries >= samples_retries:
                        raise gcmd.error("Probe samples exceed \
                            samples_tolerance")
                    gcmd.respond_info("Probe samples exceed tolerance. \
                        Retrying...")
                    retries += 1
                    positions = []
            # Retract
            if len(positions) < sample_count:
                logging.info("retract move start")
                start_from = halt_pos if probe_pos is None else probe_pos
                self._move(probexy + [start_from[2] + sample_retract_dist],
                            lift_speed)
                logging.info("retract move complete")
        if must_notify_multi_probe:
            self.multi_probe_end()
        # Calculate and return result
        if samples_result == 'median':
            return self._calc_median(positions)
        return self._calc_mean(positions)

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
        # TODO: make configurable
        settling_time = 0.375
        collector.collect_until(toolhead.get_last_move_time() + settling_time)
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
