# Load Cell Probe
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import mcu
from . import probe, load_cell, hx71x, ads1220

######## Types

class BadTapModule(object):
    def is_bad_tap(self, tap_analysis):
        return False

class NozzleCleanerModule(object):
    def clean_nozzle(self):
        pass

class TrapezoidalMove(object):
    def __init__(self, move):
        # copy c data to python memory
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

    def to_dict(self):
        return {'print_time': float(self.print_time),
                'move_t': float(self.move_t),
                'start_v': float(self.start_v),
                'accel': float(self.accel),
                'start_x': float(self.start_x),
                'start_y': float(self.start_y),
                'start_z': float(self.start_z),
                'x_r': float(self.x_r),
                'y_r': float(self.y_r),
                'z_r': float(self.z_r)
        }

# point on a time/force graph
class ForcePoint(object):
    def __init__(self, time, force):
        self.time = time
        self.force = force

    def to_dict(self):
        return {'time': self.time, 'force': self.force}

# slope/intercept based line where x is time and y is force
class ForceLine(object):
    def __init__(self, slope, intercept):
        self.slope = slope
        self.intercept = intercept

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

    def to_dict(self):
        return {'slope': self.slope, 'intercept': self.intercept}

# convert double to signed fixed point Q12.19 integer
Q12_INT_BITS = 12
Q12_FRAC_BITS = (32 - (1 + Q12_INT_BITS))
def as_fixedQ12(val):
    return int(val * (2 ** Q12_FRAC_BITS))

# Digital filter designer and container
class DigialFilter:
    def __init__(self, sps, cfg_error, highpass=None, lowpass=None,
                    notches=None, notch_quality=2.0):
        self.filter_sections = None
        self.sample_frequency = sps
        if not (highpass or lowpass or notches):
            return
        try:
            import scipy.signal as signal
        except:
            raise cfg_error("Filters require the SciPy module")
        sos = []
        if highpass:
            sos.append(self._butter(highpass, "highpass"))
        for notch_freq in notches:
            sos.append(self._notch(notch_freq, notch_quality))
        if lowpass:
            sos.append(self._butter(lowpass, "lowpass"))
        import numpy as np
        self.filter_sections = np.vstack(sos)

    def _butter(self, frequency, btype):
        import scipy.signal as signal
        return signal.butter(2, Wn=frequency, btype=btype,
                             fs=self.sample_frequency, output='sos')

    def _notch(self, freq, quality):
        import scipy.signal as signal
        b, a = signal.iirnotch(freq, Q=quality, fs=self.sample_frequency)
        return signal.tf2sos(b, a)

    # convert the sos matrix to fixed point Q12 format
    def sos_fixed_q12(self):
        if not self.has_filters():
            return None
        sos_fixed = []
        for section in self.filter_sections:
            fixed_section = []
            for col, coeff in enumerate(section):
                if col != 3:  # omit column 3, its always 1.0 and not used
                    fixed_section.append(as_fixedQ12(coeff))
            sos_fixed.append(fixed_section)
        return sos_fixed

    # apply the forwards+backwards "filter-filter" algorithm to data
    def filtfilt(self, data):
        if not self.has_filters():
            return data
        import scipy.signal as signal
        return signal.sosfiltfilt(self.filter_sections, data)

    # true if the filter is configured with any filter sections
    def has_filters(self):
        return self.filter_sections is not None

#########################
# Math Support Functions

# compute the index in the time array when some value was surpassed
def index_near(time, instant):
    import numpy as np
    return int(np.argmax(np.asarray(time) >= instant) or len(time) -1)

# helper class for working with a time/force graph
# work with subsections to find elbows and best fit lines
class ForceGraph:
    def __init__(self, time, force):
        self.time = time
        self.force = force

    # Least Squares on x[] y[] points, returns ForceLine
    def _lstsq_line(self, x, y):
        import numpy as np
        x = np.asarray(x)
        y = np.asarray(y)
        x_stacked = np.vstack([x, np.ones(len(x))]).T
        mx, b = np.linalg.lstsq(x_stacked, y, rcond=None)[0]
        return mx, b

    def _lstsq_error(self, x, y):
        import numpy as np
        x = np.asarray(x)
        y = np.asarray(y)
        x_stacked = np.vstack([x, np.ones(len(x))]).T
        residuals = np.linalg.lstsq(x_stacked, y, rcond=None)[1]
        return residuals[0] if residuals else 0

    # Local best fit elbow finder, finds first point of decreasing fitness
    def _two_lines_best_fit(self, x, y, search_direction):
        sweep = range(2, len(x) - 2)
        if search_direction == -1:
            sweep = reversed(sweep)
        min_error = float('inf')
        for i in sweep:
            r1 = self._lstsq_error(x[0:i], y[0:i])
            r2 = self._lstsq_error(x[i:], y[i:])
            error = r1 + r2
            if error < min_error:
                min_error = error
            else:
                return i
        raise "no elbow found!"

    def _split(self, start_idx, end_idx, discard_left=0, discard_right=0):
        t = self.time[start_idx + discard_left:end_idx - discard_right]
        f = self.force[start_idx + discard_left:end_idx - discard_right]
        return t, f

    def find_elbow(self, start_idx, end_idx, search_direction=1):
        t, f = self._split(start_idx, end_idx)
        elbow = self._two_lines_best_fit(t, f, search_direction)
        return start_idx + elbow

    def index_near(self, instant):
        import numpy as np
        return int(np.argmax(np.asarray(self.time) >= instant)
                   or len(self.time) - 1)

    def line(self, start_idx, end_idx, discard_left=0, discard_right=0):
        t, f = self._split(start_idx, end_idx, discard_left, discard_right)
        mx, b = self._lstsq_line(t, f)
        return ForceLine(mx, b)

    # break a tap event down into 6 points and 5 lines:
    #    *-----*|       /*-----*
    #           |      /
    #           *----*/
    def tap_decompose(self, homing_end_time, pullback_start_time,
                      discard=0):
        homing_end_idx = self.index_near(homing_end_time)
        pullback_start_idx = self.index_near(pullback_start_time)
        contact_elbow_idx = self.find_elbow(0, homing_end_idx, -1)
        # l1 is the approach line
        l1 = self.line(0, contact_elbow_idx, discard, discard)
        # sometime after contact_elbow_idx is the peak force and the start of
        # the dwell line
        dwell_start_idx = self.find_elbow(contact_elbow_idx,
                                           pullback_start_idx)
        # l2 is the compression line, it may have very few points so no discard
        l2 = self.line(contact_elbow_idx, dwell_start_idx)
        # l3 is the dwell line
        l3 = self.line(dwell_start_idx, pullback_start_idx, discard, discard)
        # discard the first 1/5th of the line because it can contain ringing
#                        ((pullback_start_idx - dwell_start_idx) // 5), discard)
        # find the elbow where the probe breaks contact
        break_contact_idx = self.find_elbow(pullback_start_idx, -1)
        l4 = self.line(pullback_start_idx, break_contact_idx, discard, discard)
        l5 = self.line(break_contact_idx, -1, discard, discard)

        # Line intersections:
        p0 = ForcePoint(self.time[0], l1.find_force(self.time[0]))
        p1 = l1.intersection(l2)
        p2 = l2.intersection(l3)
        p3 = l3.intersection(l4)
        p4 = l4.intersection(l5)
        p5 = ForcePoint(self.time[-1], l5.find_force(self.time[-1]))
        return [p0, p1, p2, p3, p4, p5], [l1, l2, l3, l4, l5]

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
    return total_var, delta_var

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
        # returns r squared as a percentage. -350% is bad. 80% is good!
        r_squared.append(round((r2 * 100.), 1))
    return r_squared

# TODO: maybe discard points can scale with sample rate from 1 to 3
DEFAULT_DISCARD_POINTS = 3
class TapAnalysis(object):
    def __init__(self, printer, samples, tap_filter,
                 discard=DEFAULT_DISCARD_POINTS):
        import numpy as np
        self.printer = printer
        self.discard = discard
        np_samples = np.array(samples)
        self.time = np_samples[:, 0]
        self.force = tap_filter.filtfilt(np_samples[:, 1])
        self.force_graph = ForceGraph(self.time, self.force)
        self.sample_time = np.average(np.diff(self.time))
        self.r_squared_widths = [int((n * 0.01) // self.sample_time)
                                 for n in range(2, 7)]
        trapq = printer.lookup_object('motion_report').trapqs['toolhead']
        self.moves = self._extract_trapq(trapq)
        self.home_end_time = self._recalculate_homing_end()
        self.pullback_start_time = self.moves[-3].print_time
        self.pullback_end_time = (self.moves[-1].print_time
                                  + self.moves[-1].move_t)
        self.position = self._extract_pos_history()
        self.is_valid = False
        self.tap_pos = None
        self.tap_points = []
        self.tap_lines = []
        self.tap_angles = []
        self.tap_r_squared = None

    # build toolhead position history for the time/force graph
    def _extract_pos_history(self):
        z_pos = []
        for time in self.time:
            z_pos.append(self.get_toolhead_position(time))
        return z_pos

    def get_toolhead_position(self, print_time):
        for i, move in enumerate(self.moves):
            start_time = move.print_time
            # time before first move, printer was stationary
            if i == 0 and print_time < start_time:
                return move.start_x, move.start_y, move.start_z
            end_time = float('inf')
            if i < (len(self.moves) - 1):
                end_time = self.moves[i + 1].print_time
            if start_time <= print_time < end_time:
                # we have found the move
                move_t = move.move_t
                move_time = max(0.,
                        min(move_t, print_time - move.print_time))
                dist = ((move.start_v + .5 * move.accel * move_time)
                        * move_time)
                pos = ((move.start_x + move.x_r * dist,
                        move.start_y + move.y_r * dist,
                        move.start_z + move.z_r * dist))
                return pos
            else:
                continue
        raise self.printer.command_error("Move not found, that is impossible!")

    # adjust move_t of move 1 to match the toolhead position of move 2
    def _recalculate_homing_end(self):
        #TODO: REVIEW: This takes some logical shortcuts, does it need to be
        # more generalized? e.g. to all 3 axes?
        homing_move = self.moves[-5]
        halt_move = self.moves[-4]
        # acceleration should be 0! This is the 'coasting' move:
        if homing_move.accel != 0.:
            raise self.printer.command_error(
                    'Unexpected acceleration in coasting move')
        # how long did it take to get to end_z?
        homing_move.move_t = abs((halt_move.start_z - homing_move.start_z)
                                 / homing_move.start_v)
        return homing_move.print_time + homing_move.move_t

    def _extract_trapq(self, trapq):
        moves, _ = trapq.extract_trapq(self.time[0], self.time[-1])
        moves_out = []
        for move in moves:
            moves_out.append(TrapezoidalMove(move))
            # DEBUG: enable to see trapq contents
            # logging.info("trapq move: %s" % (moves_out[-1].to_dict(),))
        num_moves = len(moves_out)
        if num_moves < 5 or num_moves > 6:
            raise self.printer.command_error(
                "Expected tap to be 5 to 6 moves long was %s" % (num_moves,))
        return moves_out

    def analyze(self):
        points, lines = self.force_graph.tap_decompose(self.home_end_time,
                            self.pullback_start_time, self.discard)
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
        if not self.validate_break_contact_time(break_contact_time):
            logging.info('Tap break-contact time is invalid')
            return
        self.tap_pos = self.get_toolhead_position(break_contact_time)
        if not self.validate_elbow_clearance():
            logging.info('Elbow too near tap ends')
            return
        self.tap_r_squared = self.calculate_r_squared()
        self.is_valid = True

    # validate peak force within 50ms of homing end
    def validate_peak_force(self, peak_force_index, home_end_index):
        delta = peak_force_index - home_end_index
        delta_t = abs(self.sample_time * delta)
        return delta <= 1 or delta_t < 0.05

    # validate that a set of ForcePoint objects are in chronological order
    def validate_order(self):
        p = self.tap_points
        return (p[0].time < p[1].time < p[2].time
                < p[3].time < p[4].time < p[5].time)

    # Validate that the rotations between lines form a tap shape
    def validate_elbow_rotation(self):
        a1, a2, a3, a4 = self.tap_angles
        # with two polarities there are 2 valid tap shapes:
        return ((a1 > 0 and a2 < 0 and a3 < 0 and a4 > 0) or
                (a1 < 0 and a2 > 0 and a3 > 0 and a4 < 0))

    # check for space around elbows to calculate r_squared
    def validate_elbow_clearance(self):
        width = self.r_squared_widths[-1]
        start_idx = self.force_graph.index_near(self.tap_points[1].time) + width
        end_idx = self.force_graph.index_near(self.tap_points[4].time) - width
        return start_idx > 0 and end_idx < len(self.time)

    # the proposed break contact point must fall inside the pullback move
    def validate_break_contact_time(self, break_contact_time):
        return (self.pullback_start_time < break_contact_time
                < self.pullback_end_time)

    def calculate_angles(self):
        l1, l2, l3, l4, l5 = self.tap_lines
        return [l1.angle(l2), l2.angle(l3), l3.angle(l4), l4.angle(l5)]

    def calculate_r_squared(self):
        r_squared = []
        for i, elbow in enumerate(self.tap_points[1: -1]):
            elbow_idx = self.force_graph.index_near(elbow.time)
            r_squared.append(elbow_r_squared(self.force, self.time, elbow_idx,
                            self.r_squared_widths,
                            self.tap_lines[i], self.tap_lines[i + 1]))
        return r_squared

    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            'time': self.time.tolist(),
            'force': self.force.tolist(),
            'position': self.position,
            'points': [point.to_dict() for point in self.tap_points],
            'lines': [line.to_dict() for line in self.tap_lines],
            'tap_pos': self.tap_pos,
            'moves': [move.to_dict() for move in self.moves],
            'home_end_time': self.home_end_time,
            'pullback_start_time': self.pullback_start_time,
            'pullback_end_time': self.pullback_end_time,
            'tap_angles': self.tap_angles,
            'tap_r_squared': self.tap_r_squared,
            'is_valid': self.is_valid,
        }

# support for validating individual options in a config list
def getfloatlist(config, option, above=None, below=None, max_len=None):
    values = config.getfloatlist(option, default=[])
    if max_len is not None and len(values) > max_len:
        raise config.error("Option '%s' in section '%s' must have maximum"
                           " length %s" % (option, config.get_name(), max_len))
    for value in values:
        validatefloat(config, option, value, above, below)
    return values

def validatefloat(config, option, value, above, below):
    if above is not None and value <= above:
        raise config.error("Option '%s' in section '%s' must be above %s"
                    % (option, config.get_name(), above))
    if below is not None and value >= below:
        raise config.error("Option '%s' in section '%s' must be below %s"
                     % (option, config.get_name(), below))

NOZZLE_CLEANER = "{action_respond_info(\"Bad tap detected, nozzle needs" \
        " cleaning. nozzle_cleaner_gcode not configured!\")}"
# Helper to track multiple probe attempts in a single command
class LoadCellProbeSessionHelper:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.mcu_probe = mcu_probe
        gcode = self.printer.lookup_object('gcode')
        self.dummy_gcode_cmd = gcode.create_gcode_command("", "", {})
        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)
        self.homing_helper = probe.HomingViaProbeHelper(config, mcu_probe)
        # Configurable probing speeds
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)
        # Session state
        self.multi_probe_pending = False
        self.results = []
        # Register event handlers
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        # load cell probe options
        self.bad_tap_retries = config.getint('bad_tap_retries', 1, minval=0)
        self.nozzle_cleaner_module = self.load_module(
                                        config, 'nozzle_cleaner_module', None)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.nozzle_cleaner_gcode = gcode_macro.load_template(config,
                                    'nozzle_cleaner_gcode', NOZZLE_CLEANER)

    def _handle_command_error(self):
        if self.multi_probe_pending:
            try:
                self.end_probe_session()
            except:
                logging.exception("Multi-probe end")

    def _probe_state_error(self):
        raise self.printer.command_error(
            "Internal probe error - start/end probe session mismatch")

    def load_module(self, config, name, default):
        module = config.get(name, default=None)
        return default if module is None else self.printer.lookup_object(module)

    def start_probe_session(self, gcmd):
        if self.multi_probe_pending:
            self._probe_state_error()
        self.mcu_probe.multi_probe_begin()
        self.multi_probe_pending = True
        self.results = []
        return self

    def end_probe_session(self):
        if not self.multi_probe_pending:
            self._probe_state_error()
        self.results = []
        self.multi_probe_pending = False
        self.mcu_probe.multi_probe_end()

    def get_probe_params(self, gcmd=None):
        if gcmd is None:
            gcmd = self.dummy_gcode_cmd
        probe_speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        samples = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        return {'probe_speed': probe_speed,
                'lift_speed': lift_speed,
                'samples': samples,
                'sample_retract_dist': sample_retract_dist,
                'samples_tolerance': samples_tolerance,
                'samples_tolerance_retries': samples_retries,
                'samples_result': samples_result}

    # execute nozzle cleaning routine
    def clean_nozzle(self, retries):
        #TODO: what params to pass to nozzle cleaners?
        # [X,Y,Z] of the failed probe?
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

    def single_tap(self, speed):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        pos = toolhead.get_position()
        pos[2] = self.z_position
        try:
            epos, is_good = self.mcu_probe.tapping_move(pos, speed)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # Allow axis_twist_compensation to update results
        self.printer.send_event("probe:update_results", epos)
        # Report results
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                           % (epos[0], epos[1], epos[2]))
        return epos[:3], is_good

    def probe_cycle(self, probexy, params):
        retries = 0
        while retries <= self.bad_tap_retries:
            epos, is_good = self.single_tap(params['probe_speed'])
            if is_good:
                return epos
            if retries < self.bad_tap_retries:
                self.retract(probexy, epos, params)
                self.clean_nozzle(retries)
                retries += 1
                # TODO: maybe goto the probing location after cleaning?
            else:
                raise self.printer.command_error(
                    'Bad taps exceeded bas_tap_retries')

    def retract(self, probexy, pos, params):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.manual_move(
            probexy + [pos[2] + params['sample_retract_dist']],
            params['lift_speed'])

    def run_probe(self, gcmd):
        if not self.multi_probe_pending:
            self._probe_state_error()
        params = self.get_probe_params(gcmd)
        toolhead = self.printer.lookup_object('toolhead')
        probexy = toolhead.get_position()[:2]
        retries = 0
        positions = []
        sample_count = params['samples']
        while len(positions) < sample_count:
            # Probe position
            pos = self.probe_cycle(probexy, params)
            positions.append(pos)
            # Check samples tolerance
            z_positions = [p[2] for p in positions]
            if max(z_positions)-min(z_positions) > params['samples_tolerance']:
                if retries >= params['samples_tolerance_retries']:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                self.retract(probexy, pos, params)
        # Calculate result
        epos = probe.calc_probe_z_average(positions, params['samples_result'])
        self.results.append(epos)
    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res

#class to keep context across probing/homing events
class ProbeSessionContext():
    def __init__(self, config, load_cell_inst):
        self.printer = printer = config.get_printer()
        self.load_cell = load_cell_inst
        self.collector = None
        self.pullback_distance = config.getfloat('pullback_dist', minval=0.01,
                                                 maxval=2.0, default=0.1)
        sps = self.load_cell.get_sensor().get_samples_per_second()
        # TODO: Math: set the maximum pullback speed such that at least
        # enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        default_pullback_speed = sps * 0.001
        self.pullback_speed = config.getfloat('pullback_speed', minval=0.01,
                                              maxval=1.0,
                                              default=default_pullback_speed)
        self.pullback_extra_time = config.getfloat('pullback_extra_time',
                                                   minval=0.00, maxval=1.0,
                                                   default=0.3)
        self.bad_tap_module = self.load_module(config, 'bad_tap_module',
                                               BadTapModule())
        # optional filter config
        tap_notches = getfloatlist(config, "tap_filter_notch", above=0,
                                   below=math.floor(sps / 2.), max_len=5)
        notch_quality = config.getfloat("tap_filter_notch_quality",
                                        minval=0.5, maxval=3.0, default=2.0)
        self.tap_filter = DigialFilter(sps, config.error, notches=tap_notches,
                                       notch_quality=notch_quality)
        # webhooks support
        self.wh_helper = load_cell.WebhooksHelper(printer)
        name = config.get_name()
        header = {"header": ["probe_tap_event"]}
        self.wh_helper.add_mux_endpoint("load_cell_probe/dump_taps",
                                        "load_cell_probe", name, header)

    def load_module(self, config, name, default):
        module = config.get(name, default=None)
        return default if module is None else self.printer.lookup_object(module)

    # Perform the pullback move and returns the time when the move will end
    def pullback_move(self):
        toolhead = self.printer.lookup_object('toolhead')
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self.pullback_distance
        toolhead.move(pullback_pos, self.pullback_speed)
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end

    def notify_probe_start(self, print_time):
        if self.collector is not None:
            self.collector.start_collecting(print_time)

    def tapping_move(self, mcu_probe, pos, speed):
        self.collector = self.load_cell.get_collector()
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        phoming = self.printer.lookup_object('homing')
        epos = phoming.probing_move(mcu_probe, pos, speed)
        pullback_end_time = self.pullback_move()
        pullback_end_pos = toolhead.get_position()
        samples, errors = self.collector.collect_until(pullback_end_time
                                               + self.pullback_extra_time)
        if errors:
            raise self.printer.command_error(
                "Sensor reported errors while homing: %i errors, %i overflows"
                % (errors[0], errors[1]))
        self.collector = None
        ppa = TapAnalysis(self.printer, samples, self.tap_filter)
        ppa.analyze()
        # broadcast tap event data:
        self.wh_helper.send({'tap': ppa.to_dict()})
        is_good = ppa.is_valid and not self.bad_tap_module.is_bad_tap(ppa)
        if is_good:
            epos[2] = ppa.tap_pos[2]
        else:
            epos = pullback_end_pos[:3]
        return epos, is_good


WATCHDOG_MAX = 3
MIN_MSG_TIME = 0.100
# LoadCellEndstop implements both MCU_endstop and ProbeEndstopWrapper
class LoadCellEndstop:
    REASON_SENSOR_ERROR = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1
    def __init__(self, config, load_cell_inst, helper):
        self._config = config
        self._config_name = config.get_name()
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        printer.register_event_handler('klippy:mcu_identify',
                                       self.handle_mcu_identify)
        self._load_cell = load_cell_inst
        self._helper = helper
        self._sensor = sensor = load_cell_inst.get_sensor()
        self._mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._rest_time = 1. / float(sensor.get_samples_per_second())
        self.settling_time = config.getfloat('settling_time', default=0.375,
                                             minval=0, maxval=1)

        # Static triggering
        self.trigger_force_grams = config.getfloat('trigger_force',
                                    minval=10., maxval=250., default=75.)
        self.safety_limit_grams = config.getfloat('safety_limit',
                                    minval=100., maxval=5000., default=2000.)
        #TODO: Review: In my view, this should always be 1
        self.trigger_count = config.getint("trigger_count",
                                           default=1, minval=1, maxval=5)
        # optional continuous tearing
        sps = self._load_cell.get_sensor().get_samples_per_second()
        # Collect 4x60hz power cycles of data to average across power noise
        default_tare_samples = max(2, round(sps * ((1 / 60) * 4)))
        self.tare_samples = config.getfloat('tare_samples',
                                            default=default_tare_samples,
                                            minval=2, maxval=sps)
        max_filter_frequency = math.floor(sps / 2.)
        hp_option = "continuous_tare_highpass"
        highpass = config.getfloat(hp_option, minval=0.1,
                                   below=max_filter_frequency, default=None)
        lowpass = config.getfloat("continuous_tare_lowpass",
                above=highpass or 0., below=max_filter_frequency, default=None)
        notches = getfloatlist(config, "continuous_tare_notch", max_len=2,
                above=(highpass or 0.), below=(lowpass or max_filter_frequency))
        notch_quality = config.getfloat("continuous_tare_notch_quality",
                                        minval=0.5, maxval=6.0, default=2.0)
        self.continuous_trigger_force = config.getfloat(
            'continuous_tare_trigger_force',
            minval=1., maxval=250., default=40.)
        if (lowpass or notches) and highpass is None:
            raise config.error("Option %s is section %s must be set to use"
                    " continuous tare" % (hp_option, config.get_name(),))

        self._continuous_tare_filter = DigialFilter(sps, config.error, highpass,
                                                lowpass, notches, notch_quality)
        # activate/deactivate gcode
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.position_endstop = config.getfloat('z_offset')
        self.activate_gcode = gcode_macro.load_template(config,
                                                        'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(config,
                                                        'deactivate_gcode', '')
        # multi probes state
        self.multi = 'OFF'
        self.deactivate_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        self._home_cmd = self._query_cmd = None
        self._set_range_cmd = _config_filter_cmd = None
        # internal tare tracking
        self.tare_counts = 0
        self.last_trigger_time = 0
        self._config_commands()
        self._mcu.register_config_callback(self._build_config)
        printer.register_event_handler("klippy:ready", self._ready_handler)
        printer.register_event_handler("load_cell:tare",
                                       self._handle_load_cell_tare)

    def _config_commands(self):
        self._mcu.add_config_cmd("config_load_cell_endstop oid=%d"
                                 % (self._oid,))
        self._mcu.add_config_cmd("load_cell_endstop_home oid=%d trsync_oid=0"
            " trigger_reason=0 error_reason=%i clock=0 sample_count=0"
            " rest_ticks=0 timeout=0" % (self._oid, self.REASON_SENSOR_ERROR)
                                 , on_restart=True)
        # configure filter:
        cmd = ("config_filter_section_load_cell_endstop oid=%d n_sections=%d"
               " section_idx=%d sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i")
        if not self._continuous_tare_filter.has_filters():
            return
        sos_fixed = self._continuous_tare_filter.sos_fixed_q12()
        n_section = len(sos_fixed)
        for i, section in enumerate(sos_fixed):
            args = (self._oid, n_section, i, section[0], section[1],
                    section[2], section[3], section[4])
            # TODO: are both needed??
            self._mcu.add_config_cmd(cmd % args, is_init=True)
            self._mcu.add_config_cmd(cmd % args, on_restart=True)

    def _build_config(self):
        # Lookup commands
        cmd_queue = self._dispatch.get_command_queue()
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_endstop_query_state oid=%c",
            "load_cell_endstop_state oid=%c homing=%c homing_triggered=%c"
            " is_triggered=%c trigger_ticks=%u sample=%i sample_ticks=%u"
            , oid=self._oid, cq=cmd_queue)
        self._set_range_cmd = self._mcu.lookup_command(
            "set_range_load_cell_endstop"
            " oid=%c safety_counts_min=%i safety_counts_max=%i"
            " filter_counts_min=%i filter_counts_max=%i"
            " trigger_counts_min=%i trigger_counts_max=%i tare_counts=%i"
            " trigger_grams=%i round_shift=%c grams_per_count=%i"
            , cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
            " error_reason=%c clock=%u sample_count=%c rest_ticks=%u timeout=%u"
            , cq=cmd_queue)
        self._config_filter_cmd = self._mcu.lookup_command(
            "config_filter_section_load_cell_endstop oid=%c n_sections=%c"
            " section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i"
            , cq=cmd_queue)

    def _ready_handler(self):
        self._sensor.attach_endstop(self._oid)

    def get_status(self, eventtime):
        return {
            'endstop_tare_counts': self.tare_counts,
            'last_trigger_time': self.last_trigger_time
        }

    def _handle_load_cell_tare(self, lc):
        if lc is self._load_cell:
            logging.info("load cell tare event: %s" % (lc.get_tare_counts(),))
            self.set_endstop_range(lc.get_tare_counts())

    def set_endstop_range(self, tare_counts):
        if not self._load_cell.is_calibrated():
            raise self.printer.command_error("Load cell not calibrated")
        tare_counts = int(tare_counts)
        self.tare_counts = tare_counts
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # calculate the safety band
        reference_tare = self._load_cell.get_reference_tare_counts()
        safety_margin = int(counts_per_gram * self.safety_limit_grams)
        safety_min = int(reference_tare - safety_margin)
        safety_max = int(reference_tare + safety_margin)
        # narrow to trigger band:
        trigger_margin = int(counts_per_gram * self.trigger_force_grams)
        trigger_min = max(tare_counts - trigger_margin, safety_min)
        trigger_max = min(tare_counts + trigger_margin, safety_max)
        # the filter is restricted to no more than +/- 2^Q_12 - 1 grams (2048)
        # this cant be changed without also changing from q12 format in MCU
        safe_bits = (Q12_INT_BITS - 1)
        filter_margin = math.floor(counts_per_gram * (2 ** safe_bits))
        filter_min = max(tare_counts - filter_margin, safety_min)
        filter_max = min(tare_counts + filter_margin, safety_max)
        # truncate extra bits for sensors with a large counts_per_gram
        storage_bits = int(math.ceil(math.log(counts_per_gram, 2)))
        rounding_shift = int(max(0, storage_bits - safe_bits))
        # grams per count, in rounded units
        grams_per_count = 1. / (counts_per_gram / (2 ** rounding_shift))
        logging.info("Set endstop range: %s, %s, %s" % (trigger_min, trigger_max, tare_counts))
        args = [self._oid, safety_min, safety_max, filter_min, filter_max,
                trigger_min, trigger_max, tare_counts,
                as_fixedQ12(self.continuous_trigger_force),
                rounding_shift, as_fixedQ12(grams_per_count)]

        self._set_range_cmd.send(args)

    # pauses for the last move to complete and then tares the load_cell
    # returns the last sample record used in taring
    def pause_and_tare(self):
        import numpy as np
        toolhead = self.printer.lookup_object('toolhead')
        collector = self._load_cell.get_collector()
        # collect tare_samples AFTER current move ends
        collector.start_collecting(min_time=toolhead.get_last_move_time())
        tare_samples, errors = collector.collect_min(self.tare_samples)
        if errors:
            raise self.printer.command_error(
            "Sensor reported errors while homing: %i errors, %i overflows"
                              % (errors[0], errors[1]))
        tare_counts = np.average(np.array(tare_samples)[:, 2].astype(float))
        self.set_endstop_range(int(tare_counts))

    def get_oid(self):
        return self._oid

    def handle_mcu_identify(self):
        kinematics = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kinematics.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    # Interface for MCU_endstop
    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # do not permit homing if the load cell is not calibrated
        if not self._load_cell.is_calibrated():
            raise self.printer.command_error("Load Cell not calibrated")
        # tare the sensor just before probing
        # this uses pause(), requiring a print_time update
        self.pause_and_tare()
        reactor = self._mcu.get_printer().get_reactor()
        now = reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now) + MIN_MSG_TIME
        clock = self._mcu.print_time_to_clock(print_time)
        # collector only used when probing
        self._helper.notify_probe_start(print_time)
        trigger_completion = self._dispatch.start(print_time)
        rest_ticks = self._mcu.seconds_to_clock(self._rest_time)
        self._home_cmd.send([self._oid, self._dispatch.get_oid(),
            mcu.MCU_trsync.REASON_ENDSTOP_HIT, self.REASON_SENSOR_ERROR, clock,
            self.trigger_count, rest_ticks, WATCHDOG_MAX])
        return trigger_completion

    def clear_home(self):
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        return self._mcu.clock_to_print_time(trigger_ticks)

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        # trigger has happened, now to find out why...
        res = self._dispatch.stop()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            if res == mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
                raise self.printer.command_error(
                    "Communication timeout during homing")
            raise self.printer.command_error("Load cell sensor error")
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.
        if self._mcu.is_fileoutput():
            return home_end_time
        self.last_trigger_time = self.clear_home()
        return self.last_trigger_time

    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        if params['homing'] == 1:
            return params['homing_triggered'] == 1
        else:
            return params['is_triggered'] == 1

    def _raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    def _lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")

    # Interface for ProbeEndstopWrapper
    def probing_move(self, pos, speed):
        raise self.printer.command_error("Not Implemented")

    def tapping_move(self, pos, speed):
        return self._helper.tapping_move(self, pos, speed)

    def multi_probe_begin(self):
        self.multi = 'FIRST'

    def multi_probe_end(self):
        self._raise_probe()
        self.multi = 'OFF'

    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self._lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'

    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self._raise_probe()

    def get_position_endstop(self):
        return self.position_endstop


class LoadCellPrinterProbe():
    def __init__(self, config, load_cell_inst, load_cell_endstop):
        self.printer = config.get_printer()
        self.mcu_probe = load_cell_endstop
        self._load_cell = load_cell_inst
        self.cmd_helper = probe.ProbeCommandHelper(config, self,
                                             self.mcu_probe.query_endstop)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.probe_session = LoadCellProbeSessionHelper(config, self.mcu_probe)

    # Copy of PrinterProbe methods
    def get_probe_params(self, gcmd=None):
        return self.probe_session.get_probe_params(gcmd)

    def get_offsets(self):
        return self.probe_offsets.get_offsets()

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def get_status(self, eventtime):
        status = probe.PrinterProbe.get_status(self, eventtime)
        status.update(self._load_cell.get_status(eventtime))
        status.update(self.mcu_probe.get_status(eventtime))
        return status


def load_config(config):
    # Sensor types supported by load_cell_probe
    sensors = {}
    sensors.update(hx71x.HX71X_SENSOR_TYPES)
    sensors.update(ads1220.ADS1220_SENSOR_TYPE)
    sensor_class = config.getchoice('sensor_type', sensors)
    sensor = sensor_class(config)
    lc = load_cell.LoadCell(config, sensor)
    printer = config.get_printer()
    name = config.get_name().split()[-1]
    lc_name = 'load_cell' if name == "load_cell_probe" else 'load_cell ' + name
    printer.add_object(lc_name, lc)
    lce = LoadCellEndstop(config, lc, ProbeSessionContext(config, lc))
    lc_probe = LoadCellPrinterProbe(config, lc, lce)
    #TODO: for multiple probes this cant be static value 'probe'
    printer.add_object('probe', lc_probe)
    return lc_probe
