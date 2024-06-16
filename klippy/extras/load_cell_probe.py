# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, time
import mcu
from . import probe, sos_filter, load_cell, hx71x, ads1220
np = None  # delay NumPy import until configuration time

# constants for fixed point numbers
Q2_INT_BITS = 2
Q2_FRAC_BITS = (32 - (1 + Q2_INT_BITS))
Q16_INT_BITS = 16
Q16_FRAC_BITS = (32 - (1 + Q16_INT_BITS))

######## Types

class TapClassifierModule(object):
    def classify(self, tap_analysis):
        pass

class NozzleCleanerModule(object):
    def clean_nozzle(self, attempt, retries, probe_pos):
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
        self.time = time
        self.force = force
        # prepare arrays for numpy to save re-allocation costs
        self.time_float32 = np.asarray(time, dtype=np.float64)
        ones = np.ones(len(time), dtype=np.float64)
        self.time_nd = np.vstack([self.time_float32, ones]).T
        self.force_nd = np.asarray(force, dtype=np.float64)

    # Least Squares on x[] y[] points, returns ForceLine
    def _lstsq_line(self, x_stacked, y):
        mx, b = np.linalg.lstsq(x_stacked, y, rcond=None)[0]
        return mx, b

    # returns the residual sum for a best fit line
    def _lstsq_error(self, x_stacked, y):
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
        # TODO: is there an off by 1 bug here?
        return int(np.searchsorted(self.time_float32, instant) - 1)

    # construct a line from 2 points
    def _points_to_line(self, a, b):
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
        gcode = printer.lookup_object('gcode')
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

    def analyze(self, report_errors=True):
        t_start = time.time()
        try:
            self._try_analysis()
        except ValidationError as ve:
            self.errors.append(ve.error_code)
            logging.warning(ve.message)
            if report_errors:
                self.report_error('!! %s' % (ve.message,))
        self.elapsed = time.time() - t_start

    # try analysis, throws exceptions
    def _try_analysis(self):
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

    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            'time': self.time.tolist(),
            'force': self.force.tolist(),
            'position': self.position,
            'tap_points': [point.to_dict() for point in self.tap_points],
            'tap_lines': [line.to_dict() for line in self.tap_lines],
            'tap_pos': self.tap_pos,
            'moves': [move.to_dict() for move in self.moves],
            'home_end_time': self.home_end_time,
            'pullback_start_time': self.pullback_start_time,
            'pullback_end_time': self.pullback_end_time,
            'tap_angles': self.tap_angles,
            'elapsed': self.elapsed,
            'is_valid': self.is_valid,
            'errors': self.errors,
            'warnings': self.warnings
        }


# Access a parameter from config or GCode command via a consistent interface
# stores name and constraints to keep things DRY
class ParamHelper:
    def __init__(self, config, name, type_name, default=None, minval=None,
                 maxval=None, above=None, below=None, max_len=None):
        self._config_section = config.section
        self._config_error = config.error
        self.name = name
        self._type_name = type_name
        self.value = default
        self.minval = minval
        self.maxval = maxval
        self.above = above
        self.below = below
        self.max_len = max_len
        # read from config once
        self.value = self.get(config=config)

    def _get_name(self, gcmd):
        return self.name.upper() if gcmd else self.name

    def _validate_float(self, description, error, value, above, below):
        above = above or self.above
        if above is not None and value <= above:
            raise error("%s must be above %s" % (description, above))
        below = below or self.below
        if below is not None and value >= below:
            raise error("%s must be below %s" % (description, below))

    # support for validating individual options in a list of floats
    def _validate_float_list(self, gcmd, values, above, below):
        if gcmd:
            description = ("Error on '%s': %s" %
                           (gcmd.get_commandline(), self._get_name(gcmd)))
            error = gcmd.error
        else:
            description = ("Option '%s' in section '%s'" %
                           (self._get_name(gcmd), self._config_section))
            error = self._config_error
        if self.max_len is not None and len(values) > self.max_len:
            raise error("%s has maximum length %s" %
                        (description, self.max_len))
        for value in values:
            self._validate_float(description, error, value, above, below)

    def _get_int(self, config, gcmd, minval, maxval):
        get = gcmd.get_int if gcmd else config.getint
        return get(self._get_name(gcmd), self.value,
            minval or self.minval, maxval or self.maxval)

    def _get_float(self, config, gcmd, minval, maxval,
                  above, below):
        get = gcmd.get_float if gcmd else config.getfloat
        return get(self._get_name(gcmd), self.value,
            minval or self.minval, maxval or self.maxval,
            above or self.above, below or self.below)

    def _get_float_list(self, config, gcmd, above, below):
        # this code defaults to the empty list, never return None
        default = (self.value or [])
        if gcmd:
            # if the parameter isn't part of the command, return the default
            if not self._get_name(gcmd) in gcmd.get_command_parameters():
                return default
            # parameter exists, always prefer whatever is in the command
            value = gcmd.get(self._get_name(gcmd), default='')
            # Return an empty list for empty value
            if len(value.strip()) == 0:
                return []
            try:
                float_list = [float(p.strip()) for p in value.split(',')]
            except:
                raise gcmd.error("Error on '%s': unable to parse %s"
                                 % (gcmd.get_commandline(), value))
        else:
            float_list = config.getfloatlist(
                            self._get_name(gcmd), default=default)
        if float_list:
            self._validate_float_list(gcmd, float_list, above, below)
        return float_list

    def get(self, gcmd=None, minval=None, maxval=None,
                     above=None, below=None, config=None):
        if config is None and gcmd is None:
            return self.value
        if self._type_name == 'int':
            return self._get_int(config, gcmd, minval, maxval)
        elif self._type_name == 'float':
            return self._get_float(config, gcmd, minval, maxval,
                                   above, below)
        else:
            return self._get_float_list(config, gcmd, above, below)

def intParamHelper(config, name, default=None, minval=None, maxval=None):
    return ParamHelper(config, name, 'int', default,
                       minval=minval, maxval=maxval)

def floatParamHelper(config, name, default=None, minval=None, maxval=None,
                     above=None, below=None):
    return ParamHelper(config, name, 'float', default,
                       minval=minval, maxval=maxval, above=above, below=below)

def floatListParamHelper(config, name, default=None, above=None, below=None,
                         max_len=None):
    return ParamHelper(config, name, 'float_list', default,
                       above=above, below=below, max_len=max_len)

# container for filter parameters
# allows different filter configurations to be compared
class ContinuousTareFilter:
    def __init__(self, sps=None, drift=None, drift_delay=None, buzz=None,
                 buzz_delay=None, notches=None, notch_quality=None):
        self.sps = sps
        self.drift = drift
        self.drift_delay = drift_delay
        self.buzz = buzz
        self.buzz_delay = buzz_delay
        self.notches = notches
        self.notch_quality = notch_quality

    def __eq__(self, other):
        if not isinstance(other, ContinuousTareFilter):
            return False
        return (self.sps == other.sps and self.drift == other.drift
                and self.drift_delay == other.drift_delay
                and self.buzz == other.buzz
                and self.buzz_delay == other.buzz_delay
                and self.notches == other.notches
                and self.notch_quality == other.notch_quality)

    # create a filter design from the parameters
    def design_filter(self, error_func):
        design = sos_filter.DigitalFilter(self.sps, error_func,
            self.drift, self.drift_delay, self.buzz, self.buzz_delay,
            self.notches, self.notch_quality)
        fixed_filter = sos_filter.FixedPointSosFilter(
                design.get_filter_sections(), design.get_initial_state(),
                Q2_INT_BITS, Q16_INT_BITS)
        return fixed_filter


# Combine ContinuousTareFilter and SosFilter into an easy-to-use class
class ContinuousTareFilterHelper:
    def __init__(self, config, sensor, cmd_queue):
        self._sensor = sensor
        self._sps = self._sensor.get_samples_per_second()
        max_filter_frequency = math.floor(self._sps / 2.)
        # setup filter parameters
        self._drift_param = floatParamHelper(config,
            "drift_filter_cutoff_frequency",
            default=None, minval=0.1, maxval=20.0)
        self._drift_delay_param = intParamHelper(config, "drift_filter_delay",
            default=2, minval=1, maxval=2)
        self._buzz_param = floatParamHelper(config,
            "buzz_filter_cutoff_frequency",
            default=None, above=min(80.0, max_filter_frequency - 1.0),
            below=max_filter_frequency)
        self._buzz_delay_param = intParamHelper(config, "buzz_filter_delay",
            default=2, minval=1, maxval=2)
        self._notches_param = floatListParamHelper(config,
            "notch_filter_frequencies",
            default=[], above=0., below=max_filter_frequency, max_len=2)
        self._notch_quality_param = floatParamHelper(config,
            "notch_filter_quality",
            default=2.0, minval=0.5, maxval=6.0)
        # filter design specified in the config file, used for defaults
        self._config_design = ContinuousTareFilter()  # empty filter
        self._config_design = self._build_filter()
        # filter design currently inside the MCU
        self._active_design = self._config_design
        self._sos_filter = self._create_filter(
            self._active_design.design_filter(config.error), cmd_queue)

    def _build_filter(self, gcmd=None):
        drift = self._drift_param.get(gcmd)
        drift_delay = self._drift_delay_param.get(gcmd)
        buzz = self._buzz_param.get(gcmd)
        buzz_delay = self._buzz_delay_param.get(gcmd)
        # notches must be between drift and buzz:
        notches = self._notches_param.get(gcmd, above=drift, below=buzz)
        notch_quality = self._notch_quality_param.get(gcmd)
        return ContinuousTareFilter(self._sps, drift, drift_delay,
            buzz, buzz_delay, notches, notch_quality)

    def _create_filter(self, fixed_filter, cmd_queue):
        return sos_filter.SosFilter(
            self._sensor.get_mcu(), cmd_queue, fixed_filter, 4)

    def update_from_command(self, gcmd, cq=None):
        gcmd_filter = self._build_filter(gcmd)
        # if filters are identical, no change required
        if self._active_design == gcmd_filter:
            return
        # update MCU filter from GCode command
        self._sos_filter.change_filter(
            self._active_design.design_filter(gcmd.error))

    def get_sos_filter(self):
        return self._sos_filter


# check results from the collector for errors and raise an exception is found
def check_sensor_errors(results, printer):
    samples, errors = results
    if errors:
        raise printer.command_error("Load cell sensor reported errors while"
            " probing: %i errors, %i overflows"
            % (errors[0], errors[1]))
    return samples

class LoadCellProbeConfigHelper:
    def __init__(self, config, load_cell_inst):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        gcode = config.get_printer().lookup_object('gcode')
        self.dummy_gcode_cmd = gcode.create_gcode_command("", "", {})
        self._sensor = load_cell_inst.get_sensor()
        self._rest_time = 1. / float(self._sensor.get_samples_per_second())
        # Collect 4 x 60hz power cycles of data to average across power noise
        self._tare_time_param = floatParamHelper(config, 'tare_time',
            default=4. / 60., minval=0.01, maxval=1.0)
        # triggering options
        self._trigger_force_param = intParamHelper(config,
            'trigger_force', default=75, minval=10, maxval=250)
        self._force_safety_limit_param = intParamHelper(config,
            'force_safety_limit', minval=100, maxval=5000, default=2000)
        # pullback move
        self._pullback_distance_param = floatParamHelper(config,
            'pullback_distance', minval=0.01, maxval=2.0, default=0.2)
        # TODO: Math: set the maximum pullback speed such that at least
        # enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        sps = self._sensor.get_samples_per_second()
        self._pullback_speed_param = floatParamHelper(config,
            'pullback_speed', minval=0.1, maxval=1.0, default=sps * 0.001)
        self._bad_tap_retries_param = intParamHelper(config,
            'bad_tap_retries', 1, minval=0, maxval=10)

    def _gcmd(self, gcmd=None):
        if gcmd is None:
            return self.dummy_gcode_cmd
        return gcmd

    def get_tare_samples(self, gcmd=None):
        tare_time = self._tare_time_param.get(gcmd)
        sps = self._sensor.get_samples_per_second()
        return max(2, math.ceil(tare_time * sps))

    def get_trigger_force_grams(self, gcmd=None):
        return self._trigger_force_param.get(gcmd)

    def get_safety_limit_grams(self, gcmd=None):
        return self._force_safety_limit_param.get(gcmd)

    def get_pullback_speed(self, gcmd=None):
        return self._pullback_speed_param.get(gcmd)

    def get_pullback_distance(self, gcmd=None):
        return self._pullback_distance_param.get(gcmd)

    def get_bad_tap_retries(self, gcmd=None):
        return self._bad_tap_retries_param.get(gcmd)

    def get_rest_time(self):
        return self._rest_time

    def get_safety_range(self, gcmd=None):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # calculate the safety band
        zero = self._load_cell.get_reference_tare_counts()
        safety_counts = int(counts_per_gram * self.get_safety_limit_grams(gcmd))
        safety_min = int(zero - safety_counts)
        safety_max = int(zero + safety_counts)
        # don't allow a safety range outside the sensor's real range
        sensor_min, sensor_max = self._load_cell.get_sensor().get_range()
        if safety_min <= sensor_min or  safety_max >= sensor_max:
            cmd_err = self._printer.command_error
            raise cmd_err("Load cell force_safety_limit exceeds sensor range!")
        return safety_min, safety_max

    # calculate 1/counts_per_gram in Q2 fixed point
    def get_grams_per_count(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # The counts_per_gram could be so large that it becomes 0.0 when
        # converted to Q2 format. This would mean the ADC range only measures a
        # few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= 2**Q2_FRAC_BITS:
            raise OverflowError("counts_per_gram value is too large to filter")
        return sos_filter.to_fixed_32((1. / counts_per_gram), Q2_INT_BITS)


# McuLoadCellProbe is the interface to `load_cell_probe` on the MCU
# This also manages the SosFilter so all commands use one command queue
class McuLoadCellProbe:
    WATCHDOG_MAX = 3
    ERROR_SAFETY_RANGE = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1
    ERROR_OVERFLOW = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 2
    ERROR_WATCHDOG = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 3
    def __init__(self, config, load_cell_inst, sos_filter_inst,
                 config_helper, trigger_dispatch):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        self._sos_filter = sos_filter_inst
        self._config_helper = config_helper
        self._sensor = load_cell_inst.get_sensor()
        self._mcu = self._sensor.get_mcu()
        # configure MCU objects
        self._dispatch = trigger_dispatch
        self._cmd_queue = self._dispatch.get_command_queue()
        self._oid = self._mcu.create_oid()
        self._config_commands()
        self._home_cmd = None
        self._query_cmd = None
        self._set_range_cmd = None
        self._mcu.register_config_callback(self._build_config)
        self._printer.register_event_handler(
            "klippy:connect", self._on_connect)

    def _config_commands(self):
        self._sos_filter.create_filter()
        self._mcu.add_config_cmd(
            "config_load_cell_probe oid=%d sos_filter_oid=%d"
             % (self._oid, self._sos_filter.get_oid()))

    def _build_config(self):
        # Lookup commands
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_probe_query_state oid=%c",
            "load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u"
            , oid=self._oid, cq=self._cmd_queue)
        self._set_range_cmd = self._mcu.lookup_command(
            "load_cell_probe_set_range"
            " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
            " trigger_grams=%u grams_per_count=%i"
            , cq=self._cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c"
            " error_reason=%c clock=%u rest_ticks=%u timeout=%u"
            , cq=self._cmd_queue)

    # the sensor data stream is connected on the MCU at the ready event
    def _on_connect(self):
        self._sensor.attach_load_cell_probe(self._oid)

    def get_oid(self):
        return self._oid

    def get_mcu(self):
        return self._mcu

    def get_load_cell(self):
        return self._load_cell

    def get_dispatch(self):
        return self._dispatch

    def set_endstop_range(self, tare_counts, gcmd=None):
        # update the load cell so it reflects the new tare value
        self._load_cell.tare(tare_counts)
        # update internal tare value
        safety_min, safety_max = self._config_helper.get_safety_range(gcmd)
        args = [self._oid, safety_min, safety_max, int(tare_counts),
                self._config_helper.get_trigger_force_grams(gcmd),
                self._config_helper.get_grams_per_count()]
        self._set_range_cmd.send(args)
        self._sos_filter.reset_filter()

    def home_start(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        rest_time = self._config_helper.get_rest_time()
        rest_ticks = self._mcu.seconds_to_clock(rest_time)
        self._home_cmd.send([self._oid, self._dispatch.get_oid(),
            mcu.MCU_trsync.REASON_ENDSTOP_HIT, self.ERROR_SAFETY_RANGE, clock,
            rest_ticks, self.WATCHDOG_MAX], reqclock=clock)

    def clear_home(self):
        params = self._query_cmd.send([self._oid])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        return self._mcu.clock_to_print_time(trigger_ticks)


# Execute probing moves using the McuLoadCellProbe
class LoadCellProbingMove:
    ERROR_MAP = {
        mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            "Communication timeout during homing",
        McuLoadCellProbe.ERROR_SAFETY_RANGE:
            "Load Cell Probe Error: load exceeds safety limit",
        McuLoadCellProbe.ERROR_OVERFLOW:
            "Load Cell Probe Error: fixed point math overflow",
        McuLoadCellProbe.ERROR_WATCHDOG:
            "Load Cell Probe Error: timed out waiting for sensor data"
    }
    def __init__(self, config, mcu_load_cell_probe,
                 param_helper, continuous_tare_filter_helper, config_helper):
        self._printer = config.get_printer()
        self._mcu_load_cell_probe = mcu_load_cell_probe
        self._param_helper = param_helper
        self._continuous_tare_filter_helper = continuous_tare_filter_helper
        self._config_helper = config_helper
        self._mcu = mcu_load_cell_probe.get_mcu()
        self._load_cell = mcu_load_cell_probe.get_load_cell()
        self._z_min_position = probe.lookup_minimum_z(config)
        self._dispatch = mcu_load_cell_probe.get_dispatch()
        probe.LookupZSteppers(config, self._dispatch.add_stepper)
        # internal state tracking
        self._tare_counts = 0
        self._last_trigger_time = 0

    def _start_collector(self):
        toolhead = self._printer.lookup_object('toolhead')
        # homing uses the toolhead last move time which gets special handling
        # to significantly buffer print_time if the move queue has drained
        print_time = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        return collector

    # pauses for the last move to complete and then
    # sets the endstop tare value and range
    def _pause_and_tare(self, gcmd):
        collector = self._start_collector()
        num_samples = self._config_helper.get_tare_samples(gcmd)
        # use collect_min collected samples are not wasted
        results = collector.collect_min(num_samples)
        tare_samples = check_sensor_errors(results, self._printer)
        tare_counts = np.average(np.array(tare_samples)[:, 2].astype(float))
        # update sos_filter with any gcode parameter changes
        self._continuous_tare_filter_helper.update_from_command(gcmd)
        self._mcu_load_cell_probe.set_endstop_range(tare_counts, gcmd)

    def _home_start(self, print_time):
        # start trsync
        trigger_completion = self._dispatch.start(print_time)
        self._mcu_load_cell_probe.home_start(print_time)
        return trigger_completion

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        return self._home_start(print_time)

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        # trigger has happened, now to find out why...
        res = self._dispatch.stop()
        # clear the homing state so it stops processing samples
        self._last_trigger_time = self._mcu_load_cell_probe.clear_home()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            error = "Load Cell Probe Error: unknown reason code %i" % (res,)
            if res in self.ERROR_MAP:
                error = self.ERROR_MAP[res]
            raise self._printer.command_error(error)
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.
        return self._last_trigger_time

    def get_steppers(self):
        return self._dispatch.get_steppers()

    # Probe towards z_min until the load_cell_probe on the MCU triggers
    def probing_move(self, gcmd):
        # do not permit probing if the load cell is not calibrated
        if not self._load_cell.is_calibrated():
            raise self._printer.command_error("Load Cell not calibrated")
        # tare the sensor just before probing
        self._pause_and_tare(gcmd)
        # get params for the homing move
        toolhead = self._printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[2] = self._z_min_position
        speed = self._param_helper.get_probe_params(gcmd)['probe_speed']
        phoming = self._printer.lookup_object('homing')
        # start collector after tare samples are consumed
        collector = self._start_collector()
        # do homing move
        return phoming.probing_move(self, pos, speed), collector

    # Wait for the MCU to trigger with no movement
    def probing_test(self, gcmd, timeout):
        self._pause_and_tare(gcmd)
        toolhead = self._printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self._home_start(print_time)
        return self.home_wait(print_time + timeout)

    def get_status(self, eventtime):
        return {
            'tare_counts': self._tare_counts,
            'last_trigger_time': self._last_trigger_time,
        }


# Perform a single complete tap
class TappingMove:
    def __init__(self, config, load_cell_probing_move, tap_classifier_module,
                 config_helper):
        self._printer = config.get_printer()
        self._load_cell_probing_move = load_cell_probing_move
        self._tap_classifier_module = tap_classifier_module
        self._config_helper = config_helper
        # track results of the last tap
        self._last_result = None
        self._is_last_result_valid = False
        # webhooks support
        self._clients = load_cell.ApiClientHelper(config.get_printer())
        name = config.get_name()
        header = {"header": ["probe_tap_event"]}
        self._clients.add_mux_endpoint("load_cell_probe/dump_taps",
                                      "load_cell_probe", name, header)

    # Perform the pullback move and returns the time when the move will end
    def pullback_move(self, gcmd):
        toolhead = self._printer.lookup_object('toolhead')
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self._config_helper.get_pullback_distance(gcmd)
        pos = [None, None, pullback_pos[2]]
        toolhead.manual_move(pos, self._config_helper.get_pullback_speed(gcmd))
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end

    # perform a probing move and a pullback move
    def run_tap(self, gcmd, report_errors=True):
        # do the descending move
        epos, collector = self._load_cell_probing_move.probing_move(gcmd)
        # do the pullback move
        pullback_end_time = self.pullback_move(gcmd)
        results = collector.collect_until(pullback_end_time)
        samples = check_sensor_errors(results, self._printer)
        # Analyze the tap data
        ppa = TapAnalysis(self._printer, samples)
        ppa.analyze(report_errors)
        # allow the TapClassifier to override TapAnalysis result
        self._tap_classifier_module.classify(ppa)
        # broadcast tap event data:
        self._clients.send({'tap': ppa.to_dict()})
        self._is_last_result_valid = ppa.is_valid
        if ppa.is_valid:
            epos[2] = ppa.tap_pos[2]
        self._last_result = epos[2]
        return epos, ppa.is_valid

    def get_status(self, eventtime):
        return {
            'last_z_result': self._last_result,
            'is_last_tap_valid': self._is_last_result_valid
        }

# Probe `activate_gcode` and `deactivate_gcode` support
class ProbeActivationHelper:
    def __init__(self, config):
        self._printer = config.get_printer()
        gcode_macro = self._printer.load_object(config, 'gcode_macro')
        self._activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self._deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
    def activate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self._activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def deactivate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self._deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")


# ProbeSession that implements Tap and retry logic
class TapSession:
    def __init__(self, config, tapping_move, probe_params_helper,
                 nozzle_cleaner, config_helper):
        self._printer = config.get_printer()
        self._tapping_move = tapping_move
        self._probe_params_helper = probe_params_helper
        self._nozzle_cleaner_module = nozzle_cleaner
        self._config_helper = config_helper
        self._activator = ProbeActivationHelper(config)
        # Session state
        self._results = []

    def start_probe_session(self, gcmd):
        self._activator.activate_probe()
        return self

    def end_probe_session(self):
        self._activator.deactivate_probe()
        self._results = []

    # execute nozzle cleaning routine
    def clean_nozzle(self, retries, bad_taps, probe_pos):
        self._nozzle_cleaner_module.clean_nozzle(
            retries, bad_taps, probe_pos)

    def retract(self, params):
        # Note: retract at the current location, to allow nozzle cleaner
        # to move the probing location to avoid fouling
        toolhead = self._printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        z = pos[2] + params['sample_retract_dist']
        toolhead.manual_move([None, None, z], params['lift_speed'])

    # probe until a single good sample is returned or retries are exhausted
    def run_probe(self, gcmd):
        toolhead = self._printer.lookup_object('toolhead')
        origin_pos = toolhead.get_position()
        params = self._probe_params_helper.get_probe_params(gcmd)
        bad_taps = self._config_helper.get_bad_tap_retries(gcmd)
        for retry in range(0, bad_taps):
            if retry > 0:
                self.retract(params)
                self.clean_nozzle(retry, bad_taps, origin_pos)
            epos, is_good = self._tapping_move.run_tap(gcmd)
            if is_good:
                self._results.append(epos)
                return
        raise self._printer.command_error(
                        'Too many bad taps. (bad_tap_retries: %i)'
                        %  (bad_taps,))

    def pull_probed_results(self):
        res = self._results
        self._results = []
        return res


# A nozzle cleaner implementation that uses GCode from the probe's config
class GcodeNozzleCleaner(NozzleCleanerModule):
    # if no custom gcode is set up this warning message is printed instead
    NOZZLE_CLEANER = "{action_respond_info(\"Bad tap detected, nozzle needs" \
                     " cleaning. nozzle_cleaner_gcode not configured!\")}"
    def __init__(self, config):
        printer = config.get_printer()
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self._nozzle_cleaner_gcode = gcode_macro.load_template(config,
            'nozzle_cleaner_gcode', GcodeNozzleCleaner.NOZZLE_CLEANER)
    def clean_nozzle(self, attempt, retries, probe_pos):
        context = self._nozzle_cleaner_gcode.create_template_context()
        context['params'] = {
            'RETRIES': attempt,
            'RETRIES_REMAINING': retries - attempt,
            'ORIGINAL_PROBE_X': probe_pos[0],
            'ORIGINAL_PROBE_Y': probe_pos[1]
        }
        self._nozzle_cleaner_gcode.run_gcode_from_command(context)

class LoadCellProbeCommands:
    def __init__(self, config, load_cell_probing_move,
                 tapping_move, config_helper):
        self._printer = config.get_printer()
        self._load_cell_probing_move = load_cell_probing_move
        self._tapping_move = tapping_move
        self._config_helper = config_helper
        self._register_commands()

    def _register_commands(self):
        # Register commands
        gcode = self._printer.lookup_object('gcode')
        # for now there can only be 1 probe so no need for mux_command
        gcode.register_command("LOAD_CELL_TAP",
            self.cmd_LOAD_CELL_TAP, desc=self.cmd_LOAD_CELL_TAP_help)
        gcode.register_command("LOAD_CELL_TEST_TAP",
            self.cmd_LOAD_CELL_TEST_TAP, desc=self.cmd_LOAD_CELL_TEST_TAP_help)

    cmd_LOAD_CELL_TAP_help="Perform a single tap"
    def cmd_LOAD_CELL_TAP(self, gcmd):
        # run a single tap, don't log errors to console
        self._tapping_move.run_tap(gcmd, report_errors=False)

    cmd_LOAD_CELL_TEST_TAP_help = "Tap the load cell probe to verify operation"
    def cmd_LOAD_CELL_TEST_TAP(self, gcmd):
        taps = gcmd.get_int("TAPS", 3, minval=1, maxval=10)
        timeout = gcmd.get_float("TIMEOUT", 30., minval=1., maxval=120.)
        gcmd.respond_info("Tap the load cell %s times:" % (taps,))
        reactor = self._printer.get_reactor()
        for i in range(0, taps):
            result = self._load_cell_probing_move.probing_test(gcmd, timeout)
            if result == 0.:
                # notify of error, likely due to timeout
                raise gcmd.error("Test timeout out")
            gcmd.respond_info("Tap Detected!")
            # give the user some time for their finger to move away
            reactor.pause(reactor.monotonic() + 0.2)
        gcmd.respond_info("Test complete, %s taps detected" % (taps,))


class LoadCellPrinterProbe:
    def __init__(self, config):
        cfg_error = config.error
        try:
            global np
            import numpy as np
        except:
            raise cfg_error("[load_cell_probe] requires the NumPy module")
        self._printer = config.get_printer()
        # Sensor types supported by load_cell_probe
        sensors = {}
        sensors.update(hx71x.HX71X_SENSOR_TYPES)
        sensors.update(ads1220.ADS1220_SENSOR_TYPE)
        sensor_class = config.getchoice('sensor_type', sensors)
        sensor = sensor_class(config)
        self._load_cell = load_cell.LoadCell(config, sensor)
        # Read all user configuration and resolve plugin modules
        tap_classifier = self._lookup_object(
            config, 'tap_classifier_module', TapClassifierModule())
        nozzle_cleaner = self._lookup_object(
            config, 'nozzle_cleaner_module', GcodeNozzleCleaner(config))
        config_helper = LoadCellProbeConfigHelper(config, self._load_cell)
        self._mcu = self._load_cell.get_sensor().get_mcu()
        trigger_dispatch = mcu.TriggerDispatch(self._mcu)
        continuous_tare_filter_helper = ContinuousTareFilterHelper(
            config, sensor, trigger_dispatch.get_command_queue())
        # Probe Interface
        self._param_helper = probe.ProbeParameterHelper(config)
        self._cmd_helper = probe.ProbeCommandHelper(config, self)
        self._probe_offsets = probe.ProbeOffsetsHelper(config)
        self._mcu_load_cell_probe = McuLoadCellProbe(config, self._load_cell,
            continuous_tare_filter_helper.get_sos_filter(), config_helper,
            trigger_dispatch)
        load_cell_probing_move = LoadCellProbingMove(config,
            self._mcu_load_cell_probe, self._param_helper,
            continuous_tare_filter_helper, config_helper)
        self._tapping_move = TappingMove(
            config, load_cell_probing_move, tap_classifier, config_helper)
        tap_session = TapSession(config, self._tapping_move,
            self._param_helper, nozzle_cleaner, config_helper)
        self._probe_session = probe.ProbeSessionHelper(
            config, self._param_helper, tap_session.start_probe_session)
        # printer integration
        LoadCellProbeCommands(
            config, load_cell_probing_move, self._tapping_move, config_helper)
        probe.ProbeVirtualEndstopDeprecation(config)
        self._printer.add_object('probe', self)

    def _lookup_object(self, config, key, default):
        config_section_name = config.get(key, default=None)
        if config_section_name is None:
            return default
        return self._printer.lookup_object(config_section_name)

    def get_probe_params(self, gcmd=None):
        return self._param_helper.get_probe_params(gcmd)

    def get_offsets(self):
        return self._probe_offsets.get_offsets()

    def start_probe_session(self, gcmd):
        return self._probe_session.start_probe_session(gcmd)

    def get_status(self, eventtime):
        status = self._cmd_helper.get_status(eventtime)
        status.update(self._load_cell.get_status(eventtime))
        status.update(self._tapping_move.get_status(eventtime))
        return status


def load_config(config):
    return LoadCellPrinterProbe(config)
