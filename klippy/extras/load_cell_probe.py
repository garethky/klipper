# Load Cell Probe
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, time
import mcu
from . import probe, sos_filter, load_cell, hx71x, ads1220


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
        self.time = float(time)
        self.force = float(force)

    def to_dict(self):
        return {'time': self.time, 'force': self.force}

# slope/intercept based line where x is time and y is force
class ForceLine(object):
    def __init__(self, slope, intercept):
        self.slope = float(slope)
        self.intercept = float(intercept)

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
        # lines are parallel, will not intersect
        if denominator == 0.:
            # to get debuggable data we want to return a clearly bad value here
            return ForcePoint(0., 0.)
        intersection_time = numerator / denominator
        intersection_force = self.find_force(intersection_time)
        return ForcePoint(intersection_time, intersection_force)

    def to_dict(self):
        return {'slope': self.slope, 'intercept': self.intercept}


#########################
# Math Support Functions

# helper class for working with a time/force graph
# work with subsections to find elbows and best fit lines
class ForceGraph:
    def __init__(self, time, force):
        import numpy as np
        self.time = time
        self.force = force
        # prepare arrays for numpy to save re-allocation costs
        self.time_float32 = np.asarray(time, dtype=np.float64)
        ones = np.ones(len(time), dtype=np.float64)
        self.time_nd = np.vstack([self.time_float32, ones]).T
        self.force_nd = np.asarray(force, dtype=np.float64)

    # Least Squares on x[] y[] points, returns ForceLine
    def _lstsq_line(self, x_stacked, y):
        import numpy as np
        mx, b = np.linalg.lstsq(x_stacked, y, rcond=None)[0]
        return mx, b

    # returns the residual sum for a best fit line
    def _lstsq_error(self, x_stacked, y):
        import numpy as np
        residuals = np.linalg.lstsq(x_stacked, y, rcond=None)[1]
        return residuals[0] if residuals.size > 0 else 0

    # split a chunk of the graph in to 2 lines at i and return the residual sum
    def _two_lines_error(self, time, force, i):
        r1 = self._lstsq_error(time[0:i], force[0:i])
        r2 = self._lstsq_error(time[i:], force[i:])
        return r1 + r2

    # search exhaustively for the 2 lines that best fit the data
    # return the elbow index
    def _two_lines_best_fit(self, time, force):
        best_error = float('inf')
        best_fit_index = -1
        for i in range(1, len(force) - 2):
            error = self._two_lines_error(time, force, i)
            if error < best_error:
                best_error = error
                best_fit_index = i
        return best_fit_index

    # slice the internal nd arrays with optional discards
    def _slice_nd(self, start_idx, end_idx, discard_left=0, discard_right=0):
        # TODO: check that discarding wont result in fewer than 2 points
        t = self.time_nd[start_idx + discard_left:end_idx - discard_right]
        f = self.force_nd[start_idx + discard_left:end_idx - discard_right]
        return t, f

    def find_elbow(self, start_idx, end_idx):
        t, f = self._slice_nd(start_idx, end_idx)
        elbow_index = self._two_lines_best_fit(t, f)
        return start_idx + elbow_index

    def index_near(self, instant):
        import numpy as np
        # TODO: is there an off by 1 bug here?
        return int(np.searchsorted(self.time_float32, instant) - 1)

    # construct a line from 2 points
    def _points_to_line(self, a, b):
        import numpy as np
        t = np.asarray([[a.time, 1], [b.time, 1]], dtype=np.float64)
        f = np.asarray([a.force, b.force], dtype=np.float64)
        mx, b = self._lstsq_line(t, f)
        return ForceLine(mx, b)

    # construct a line using a subset of the graph
    def line(self, start_idx, end_idx, discard_left=0, discard_right=0):
        t, f = self._slice_nd(start_idx, end_idx, discard_left, discard_right)
        mx, b = self._lstsq_line(t, f)
        return ForceLine(mx, b)

    # given a line and a range, calculate the standard deviation of the noise
    def noise_std(self, start_idx, end_idx, line):
        import numpy as np
        f = self.force_nd[start_idx:end_idx]
        t = self.time_float32[start_idx:end_idx]
        noise = []
        for i in range(len(f)):
            noise.append(f[i] - line.find_force(t[i]))
        return np.std(noise, dtype=np.float64)

    # true if the reference force won't be confused for noise in the graph chunk
    # reference force must be more than 3 standard deviations away from the line
    # at the reference index
    def is_clear_signal(self, start_idx, end_idx, line, reference_idx,
                        force_idx):
        noise = self.noise_std(start_idx, end_idx, line)
        noise_3_std = noise * 3 # TODO: can this be lowered safely?
        base_force = line.find_force(self.time[reference_idx])
        return abs(base_force - self.force[force_idx]) > noise_3_std

    # return the first index that exceeds the median force between
    # start and end index
    def _split_by_force(self, start_idx, end_idx):
        import numpy as np
        start_f = self.force_nd[start_idx]
        end_f = self.force_nd[end_idx]
        median_f = np.median([start_f, end_f])
        scan = range(start_idx, end_idx)
        # if force is ascending, swap the scan direction
        if start_f > end_f:
            scan = reversed(scan)
        for i in scan:
            if self.force[i] > median_f:
                return i
        return None

    # break a tap event down into 6 points and 5 lines:
    #    *-----*|       /*-----*
    #           |      /
    #           *----*/
    def tap_decompose(self, homing_end_time, pullback_start_time,
                             pullback_cruise_time, pullback_cruise_duration,
                             discard=0):
        homing_end_idx = self.index_near(homing_end_time)
        # use the pullback duration to trim the amount of approach data used
        homing_start_time = homing_end_time - pullback_cruise_duration
        homing_start_idx =  max(0, self.index_near(homing_start_time))
        # locate the point where the probe made contact with the bed
        contact_elbow_idx = self.find_elbow(homing_start_idx, homing_end_idx)

        pullback_start_idx = self.index_near(pullback_start_time)
        pullback_cruise_idx = self.index_near(pullback_cruise_time)
        # limit use of additional data after the pullback move ends
        pullback_end_time = (pullback_cruise_time +
                             (pullback_cruise_duration * 1.5))
        pullback_end_idx = self.index_near(pullback_end_time)

        # l1 is the approach line
        l1 = self.line(homing_start_idx, contact_elbow_idx, discard, discard)
        # sometime after contact_elbow_idx is the peak force and the start of
        # the dwell line
        dwell_end = self.time[contact_elbow_idx] + pullback_cruise_duration
        dwell_end_idx = min(pullback_start_idx, self.index_near(dwell_end))
        dwell_start_idx = self.find_elbow(contact_elbow_idx, dwell_end_idx)
        # l2 is the compression line, it may have very few points so no discard
        # also +1 the last index in case its sequential [1, 2]
        l2 = self.line(contact_elbow_idx, dwell_start_idx + 1)
        # l3 is the dwell line
        l3 = self.line(dwell_start_idx, pullback_start_idx, discard, discard)

        # find the approximate elbow location
        break_contact_idx = self.find_elbow(pullback_cruise_idx,
                                            pullback_end_idx)
        # l5 is the line after decompression ends
        l5 = self.line(break_contact_idx, pullback_end_idx,
                       discard, discard)
        # split the points between the elbow and the start of movement by force
        midpoint_idx = self._split_by_force(pullback_cruise_idx,
                                            break_contact_idx)
        # elbow finding success depends on their being good signal-to-noise:
        use_curve_optimization = False
        if midpoint_idx is not None:
            clear_dwell = self.is_clear_signal(dwell_start_idx, dwell_end_idx,
                                l3, dwell_end_idx, midpoint_idx - 1)
            clear_decomp = self.is_clear_signal(break_contact_idx,
                          pullback_end_idx, l5, break_contact_idx, midpoint_idx)
            use_curve_optimization = clear_dwell and clear_decomp
        if use_curve_optimization:
            # perform iterative refinement
            l4_start = self.line(pullback_cruise_idx, midpoint_idx, discard, 0)
            # real break contact index
            break_contact_idx = self.find_elbow(midpoint_idx, pullback_end_idx)
            l4_end = self.line(midpoint_idx, break_contact_idx)
            l5 = self.line(break_contact_idx, pullback_end_idx,
                           discard, discard)
            # a synthetic l4 is built from 2 points:
            l4 = self._points_to_line(l4_start.intersection(l3),
                                      l4_end.intersection(l5))
        else:
            # noise is too high, don't use the curve optimization
            l4 = self.line(pullback_cruise_idx, break_contact_idx)
            #TODO: surface this to users in the tap results

        # Line intersections:
        p0 = ForcePoint(self.time[homing_start_idx],
                        l1.find_force(self.time[homing_start_idx]))
        p1 = l1.intersection(l2)
        p2 = l2.intersection(l3)
        p3 = l3.intersection(l4)
        p4 = l4.intersection(l5)
        p5 = ForcePoint(self.time[pullback_end_idx],
                        l5.find_force(self.time[pullback_end_idx]))
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
        r_squared.append(round((float(r2) * 100.), 1))
    return r_squared

class ValidationError(Exception):
    def __init__(self, error_tuple):
        self.error_code = error_tuple[0]
        self.message = error_tuple[1]
        pass

MOVE_NOT_FOUND = ('MOVE_NOT_FOUND', 'A Move references time t')
COASTING_MOVE_ACCELERATION = ('COASTING_MOVE_ACCELERATION',
    'Probing move is accelerating/decelerating which is invalid')
TOO_FEW_PROBING_MOVES = ('TOO_FEW_PROBING_MOVES',
                         '5 Probing moves expected but there were fewer')
TOO_MANY_PROBING_MOVES = ('TOO_MANY_PROBING_MOVES',
                          '5 Probing moves expected but there were more')

TAP_CHRONOLOGY = ('TAP_CHRONOLOGY', 'Tap failed chronology check')
TAP_ELBOW_ROTATION = ('TAP_ELBOW_ROTATION', 'Tap failed elbow rotation check')
TAP_BREAK_CONTACT_TOO_EARLY = ('TAP_BREAK_CONTACT_TOO_EARLY',
                               'Tap break-contact time too early, invalid')
TAP_BREAK_CONTACT_TOO_LATE = ('TAP_BREAK_CONTACT_TOO_LATE',
                              'Tap break-contact time too late, invalid')
TAP_PULLBACK_TOO_SHORT = ('TAP_PULLBACK_TOO_SHORT',
              'Tap break-contact time too late, pullback move may be too short')
TAP_SEGMENT_TOO_SHORT = ('TAP_SEGMENT_TOO_SHORT',
                 'A tap segment is too short to perform r squared calculations')

# TODO: maybe discard points can scale with sample rate from 1 to 3
DEFAULT_DISCARD_POINTS = 3
# Move index constants. The PROBE_START move may be deleted from the trapq if
# the probe takes longer than 30s. Indexing from the end of the list
# is always consistent:
PROBE_START = -6
PROBE_CRUISE = -5
PROBE_HALT = -4
PULLBACK_START = -3
PULLBACK_CRUISE = -2
PULLBACK_END = -1
class TapAnalysis(object):
    def __init__(self, printer, samples, discard=DEFAULT_DISCARD_POINTS):
        import numpy as np
        self.printer = printer
        self.samples = samples
        self.discard = discard
        self.is_valid = False
        self.tap_pos = None
        self.tap_points = []
        self.tap_lines = []
        self.tap_angles = []
        self.tap_r_squared = None
        self.elapsed = 0.
        self.errors = []
        self.warnings = []
        self.home_end_time = None
        self.pullback_start_time = None
        self.pullback_end_time = None
        self.pullback_cruise_time = None
        self.pullback_duration = None
        self.position = None
        nd_samples = np.asarray(samples, dtype=np.float64)
        self.time = nd_samples[:, 0]
        self.force = nd_samples[:, 1]
        self.force_graph = ForceGraph(self.time, self.force)
        trapq = printer.lookup_object('motion_report').trapqs['toolhead']
        self.moves = self._extract_trapq(trapq)
        gcode = self.printer.lookup_object('gcode')
        self.report_error = gcode.respond_raw

    # build toolhead position history for the time/force graph
    def _extract_pos_history(self):
        z_pos = []
        for time in self.time:
            z_pos.append(self.get_toolhead_position(float(time)))
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
        raise ValidationError(MOVE_NOT_FOUND)

    # adjust move_t of move 1 to match the toolhead position of move 2
    def _recalculate_homing_end(self):
        #TODO: REVIEW: This takes some logical shortcuts, does it need to be
        # more generalized? e.g. to all 3 axes?
        homing_move = self.moves[PROBE_CRUISE]
        halt_move = self.moves[PROBE_HALT]
        # acceleration should be 0! This is the 'coasting' move:
        if homing_move.accel != 0.:
            raise ValidationError(COASTING_MOVE_ACCELERATION)
        # how long did it take to get to end_z?
        homing_move.move_t = abs((halt_move.start_z - homing_move.start_z)
                                 / homing_move.start_v)
        return homing_move.print_time + homing_move.move_t

    def _extract_trapq(self, trapq):
        moves, _ = trapq.extract_trapq(float(self.time[0])
                                       , float(self.time[-1]))
        moves_out = []
        for move in moves:
            moves_out.append(TrapezoidalMove(move))
            # DEBUG: enable to see trapq contents
            # logging.info("trapq move: %s" % (moves_out[-1].to_dict(),))
        return moves_out

    def analyze(self):
        t_start = time.time()
        try:
            self._try_analysis()
        except ValidationError as ve:
            self.errors.append(ve.error_code)
            logging.warning(ve.message)
            self.report_error('!! %s' % (ve.message,))
        self.elapsed = time.time() - t_start

    # try analysis, throws exceptions
    def _try_analysis(self):
        import numpy as np
        num_moves = len(self.moves)
        if num_moves < 5:
            raise ValidationError(TOO_FEW_PROBING_MOVES)
        elif num_moves > 6:
            raise ValidationError(TOO_MANY_PROBING_MOVES)

        self.home_end_time = self._recalculate_homing_end()
        self.pullback_start_time = self.moves[PULLBACK_START].print_time
        self.pullback_end_time = (self.moves[PULLBACK_END].print_time +
                                  self.moves[PULLBACK_END].move_t)
        self.pullback_cruise_time = self.moves[PULLBACK_CRUISE].print_time
        self.pullback_duration = (self.pullback_end_time -
                                  self.pullback_start_time)
        self.position = self._extract_pos_history()

        points, lines = self.force_graph.tap_decompose(self.home_end_time,
                           self.pullback_start_time, self.pullback_cruise_time,
                           self.pullback_duration)
        self.tap_points = points
        self.tap_lines = lines
        self.validate_order()
        self.tap_angles = self.calculate_angles()
        self.validate_elbow_rotation()
        break_contact_time = points[4].time
        self.validate_break_contact_time(break_contact_time)
        self.tap_pos = self.get_toolhead_position(break_contact_time)
        # calculate the average time between samples
        self.sample_time = np.average(np.diff(self.time))
        self.r_squared_widths = [int((n * 0.01) // self.sample_time)
                                 for n in range(2, 7)]
        self.validate_elbow_clearance()
        self.tap_r_squared = self.calculate_r_squared()
        self.is_valid = True

    # validate that a set of ForcePoint objects are in chronological order
    def validate_order(self):
        p = self.tap_points
        if not (p[0].time < p[1].time < p[2].time
                < p[3].time < p[4].time < p[5].time):
            raise ValidationError(TAP_CHRONOLOGY)

    # Validate that the rotations between lines form a tap shape
    def validate_elbow_rotation(self):
        a1, a2, a3, a4 = self.tap_angles
        # with two polarities there are 2 valid tap shapes:
        if not ((a1 > 0 and a2 < 0 and a3 < 0 and a4 > 0) or
                (a1 < 0 and a2 > 0 and a3 > 0 and a4 < 0)):
            raise ValidationError(TAP_ELBOW_ROTATION)

    # check for space around elbows to calculate r_squared
    def validate_elbow_clearance(self):
        width = self.r_squared_widths[-1]
        start_idx = self.force_graph.index_near(self.tap_points[1].time) + width
        end_idx = self.force_graph.index_near(self.tap_points[4].time) - width
        if not (start_idx > 0 and end_idx < len(self.time)):
            raise ValidationError(TAP_SEGMENT_TOO_SHORT)

    # The proposed break contact point must fall inside the first
    # 3/4s of the pullback move
    def validate_break_contact_time(self, break_contact_time):
        start_t = self.pullback_start_time
        end_t = self.pullback_end_time
        safety_margin = (end_t - start_t) / 4.
        if break_contact_time < start_t:
            raise ValidationError(TAP_BREAK_CONTACT_TOO_EARLY)
        elif break_contact_time > end_t:
            raise ValidationError(TAP_BREAK_CONTACT_TOO_LATE)
        elif break_contact_time > (end_t - safety_margin):
            raise ValidationError(TAP_PULLBACK_TOO_SHORT)

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
            'elapsed': self.elapsed,
            'is_valid': self.is_valid,
            'errors': self.errors,
            'warnings': self.warnings
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
# TODO: this is, substantially, a copy of ProbeSessionHelper
# Whats needed to make that re-usable:
# * ability to run the pullback move
# * ability to submit the probe result after the retract move
# * access to probe location so nozzle cleaning can take place
# * Ability to re-run a probe to get a valid result
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
        epos, is_good = self.single_tap(params['probe_speed'])
        retries = 0
        while not is_good and retries < self.bad_tap_retries:
            self.retract(probexy, epos, params)
            self.clean_nozzle(retries)
            epos, is_good = self.single_tap(params['probe_speed'])
            retries += 1
        if not is_good:
            raise self.printer.command_error(
                                    'Too many bad taps. (bad_tap_retries: %i)'
                                    %  (self.bad_tap_retries,))
        return epos

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
class ProbeSessionContext:
    def __init__(self, config, load_cell_inst):
        self.printer = printer = config.get_printer()
        self.load_cell = load_cell_inst
        self.collector = None
        self.pullback_distance = config.getfloat('pullback_distance',
                                    minval=0.01, maxval=2.0, default=0.2)
        sps = self.load_cell.get_sensor().get_samples_per_second()
        # TODO: Math: set the maximum pullback speed such that at least
        # enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        default_pullback_speed = sps * 0.001
        self.pullback_speed = config.getfloat('pullback_speed', minval=0.1,
                                              maxval=10.0,
                                              default=default_pullback_speed)
        self.bad_tap_module = self.load_module(config, 'bad_tap_module',
                                               BadTapModule())
        # webhooks support
        self.clients = load_cell.ApiClientHelper(printer)
        name = config.get_name()
        header = {"header": ["probe_tap_event"]}
        self.clients.add_mux_endpoint("load_cell_probe/dump_taps",
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
        samples, errors = self.collector.collect_until(pullback_end_time)
        if errors:
            raise self.printer.command_error(
                "Sensor reported errors while homing: %i errors, %i overflows"
                % (errors[0], errors[1]))
        self.collector = None
        ppa = TapAnalysis(self.printer, samples)
        ppa.analyze()  # TODO: do this on a thread and return the lambda
        # broadcast tap event data:
        self.clients.send({'tap': ppa.to_dict()})
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
                                       self._handle_mcu_identify)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        self._load_cell = load_cell_inst
        self._helper = helper
        self._sensor = sensor = load_cell_inst.get_sensor()
        self._mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        self._sos_filter = sos_filter.SosFilter(self._mcu)
        self._dispatch = mcu.TriggerDispatch(self._mcu)
        self._rest_time = 1. / float(sensor.get_samples_per_second())
        self._home_cmd = None
        self._query_cmd = None
        self._set_range_cmd = None
        # internal tare tracking
        self.tare_counts = 0
        self.last_trigger_time = 0
        self._position_endstop = config.getfloat('z_offset')
        # triggering options
        self.trigger_force_grams = config.getint('trigger_force',
                                    minval=10, maxval=250, default=75)
        self.force_safety_limit_grams = config.getint('force_safety_limit',
                                          minval=100, maxval=5000, default=2000)
        # optional continuous tearing
        self._continuous_tare_filter = self._build_sos_filter(config)
        # configure MCU objects
        self._config_commands()
        self._mcu.register_config_callback(self._build_config)
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("load_cell:tare",
                                       self._handle_load_cell_tare)
        # multi probes state
        self.multi = 'OFF'

    def _config_commands(self):
        self._sos_filter.create_filter()
        sos_filters = self._continuous_tare_filter.get_filter_sections()
        self._sos_filter.configure_filter(sos_filters)
        self._mcu.add_config_cmd("config_load_cell_endstop oid=%d"
                                 " sos_filter_oid=%d"
                                 % (self._oid, self._sos_filter.oid))

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
            " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
            " trigger_grams=%u grams_per_count=%i"
            , cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
            " error_reason=%c clock=%u rest_ticks=%u timeout=%u"
            , cq=cmd_queue)

    def _build_sos_filter(self, config):
        # optional continuous tearing
        sps = self._load_cell.get_sensor().get_samples_per_second()
        # Collect 4x60hz power cycles of data to average across power noise
        default_tare_samples = max(2, round(sps * ((1 / 60) * 4)))
        self.tare_samples = config.getfloat('tare_samples',
                                            default=default_tare_samples,
                                            minval=2, maxval=sps)
        max_filter_frequency = math.floor(sps / 2.)
        hp_option = "drift_filter_cutoff_frequency"
        highpass = config.getfloat(hp_option, minval=0.1, maxval=20.0,
                                    below=max_filter_frequency, default=None)
        lowpass = config.getfloat("buzz_filter_cutoff_frequency",
                                    above=min(80.0, max_filter_frequency - 1.0),
                                    below=max_filter_frequency, default=None)
        notches = sos_filter.getfloatlist(config, "notch_filter_frequencies",
                                    max_len=2,
                                    above=(highpass or 0.),
                                    below=(lowpass or max_filter_frequency))
        notch_quality = config.getfloat("notch_filter_quality",
                                        minval=0.5, maxval=6.0, default=2.0)
        return sos_filter.DigitalFilter(sps, config.error, highpass,
                             lowpass, notches, notch_quality)

    def _handle_ready(self):
        self._sensor.attach_endstop(self._oid)

    def _handle_load_cell_tare(self, lc):
        if lc is self._load_cell:
            # logging.info("load cell tare event: %s" % (lc.get_tare_counts(),))
            self._set_endstop_range(int(lc.get_tare_counts()))

    def _handle_mcu_identify(self):
        kinematics = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kinematics.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def get_status(self, eventtime):
        return {
            'endstop_tare_counts': self.tare_counts,
            'last_trigger_time': self.last_trigger_time
        }

    def _get_safety_range(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # calculate the safety band
        zero = self._load_cell.get_reference_tare_counts()
        safety_counts = int(counts_per_gram * self.force_safety_limit_grams)
        return int(zero - safety_counts), int(zero + safety_counts)

    def _get_filter_converter(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # The counts_per_gram could be so large that it becomes 0.0 when
        # converted to Q1 format. This would mean the ADC range only measures a
        # few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= 2**sos_filter.Q1_FRAC_BITS:
            raise OverflowError("Counts per gram value too large to filter")
        grams_per_count = sos_filter.as_fixed_q1(1. / counts_per_gram)
        return grams_per_count

    def _clear_home(self):
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        return self._mcu.clock_to_print_time(trigger_ticks)

    def _set_endstop_range(self, tare_counts):
        cmd_err = self.printer.command_error
        if not self._load_cell.is_calibrated():
            raise cmd_err("Probe aborted, Load cell not calibrated")
        # update internal tare value
        tare_counts = int(tare_counts)
        self.tare_counts = tare_counts
        safety_min, safety_max = self._get_safety_range()
        # counts to grams conversion factors
        grams_per_count  = self._get_filter_converter()
        args = [self._oid, safety_min, safety_max, tare_counts,
                self.trigger_force_grams, grams_per_count]
        self._set_range_cmd.send(args)

    # pauses for the last move to complete and then tares the load_cell
    # returns the last sample record used in taring
    def _pause_and_tare(self):
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
        self._set_endstop_range(int(tare_counts))

    def get_oid(self):
        return self._oid

    ####
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
        self._pause_and_tare()
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
            rest_ticks, WATCHDOG_MAX])
        return trigger_completion

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
        self.last_trigger_time = self._clear_home()
        return self.last_trigger_time

    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        logging.info("Query Load Cell Endstop params returned: %s" % (params,))
        if params['homing'] == 1:
            return params['homing_triggered'] == 1
        else:
            return params['is_triggered'] == 1

    ####
    # Interface for ProbeEndstopWrapper
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
        return self._position_endstop


class LoadCellPrinterProbe:
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
        status = self.cmd_helper.get_status(eventtime)
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
    printer.add_object('probe', lc_probe)
    return lc_probe
