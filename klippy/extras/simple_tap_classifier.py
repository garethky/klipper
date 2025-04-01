from . import load_cell_probe
from .load_cell_probe import TapValidationError


# Simple tap classifier rules:
# 1) The compression force must greater than the trigger force
# 2) The decompression force should be at least 2/3s of the trigger force
# 3) The baseline force, before and after the tap, should differ by
#    less than 20% of the compression force.
# 4) The angle between the approach baseline and the compression line, should
#    be about 90 degrees, +/- 20 degrees.
# 4) The angle at the pullback elbow should be shallower,
#    be about 110 degrees, +/- 20 degrees.
class SimpleTapClassifier(load_cell_probe.TapClassifierModule):
    def __init__(self, config):
        printer = config.get_printer()
        self._min_decomp_force_pct = 0.01 * config.getfloat(
            'min_decompression_force_percentage',
            minval=20., default=66.6, maxval=100.)
        self._max_baseline_change_pct = 0.01 * config.getfloat(
            'max_baseline_force_change_percentage',
            above=0., maxval=50., default=20.)
        self._compression_angles = config.getfloatlist('compression_angles',
            count=2, default=[90. - 20., 90. + 20.])
        if self._compression_angles[0] >= self._compression_angles[1]:
            raise printer.config_error("compression_angles should be in min, "
                                       "max order")
        for angle in self._compression_angles:
            if angle < 0 or angle > 180:
                raise printer.config_error(
                    "compression_angles must be in the range 0 to 180")
        self._decompression_angles = config.getfloatlist('decompression_angles',
            count=2, default=[115. - 20, 115. + 20.])
        if self._decompression_angles[0] >= self._decompression_angles[1]:
            raise printer.config_error("decompression_angles should be in min, "
                                       "max order")
        for angle in self._decompression_angles:
            if angle < 0 or angle > 180:
                raise printer.config_error(
                    "decompression_angles must be in the range 0 to 180")

    def classify(self, tap_analysis):
        # This module cant rescue bad data
        if not tap_analysis.is_valid():
            return

        trigger_force = tap_analysis.get_trigger_force()
        raw_angles = tap_analysis.get_tap_angles()
        # find the absolute supplementary angle so the measured angles are
        # "inside" the bends between each pair of lines. These feel like the
        # "obvious" points to label as a human, since they are between 2
        # lines actually depicted in the graph:
        #       |---------\
        #       | b      c \
        #     a |           \ d
        #  -----|            \------
        angles = [180. - abs(angle) for angle in raw_angles]

        # compression line check
        tap_points = tap_analysis.get_tap_points()
        comp_start = tap_points[1]
        comp_end = tap_points[2]
        compression_force = abs(comp_end.force - comp_start.force)
        if compression_force < trigger_force:
            raise TapValidationError("LOW_COMPRESSION_FORCE",
                "Compression force was less than the trigger force")

        # compression angle check
        compression_angle = angles[0]
        min_angle = self._compression_angles[0]
        max_angle = self._compression_angles[1]
        if not (min_angle < compression_angle < max_angle):
            raise TapValidationError("COMPRESSION_ANGLE",
                "Compression angle violated constraints: %f < %f < %f" % (
                    min_angle, compression_angle, max_angle))

        # decompression line check
        decomp_start = tap_points[3]
        decomp_end = tap_points[4]
        decompression_force = abs(decomp_start.force - decomp_end.force)
        min_decompression_force = trigger_force * self._min_decomp_force_pct
        if decompression_force < min_decompression_force:
            raise TapValidationError("LOW_DECOMPRESSION_FORCE",
                "Force dropped too much before the pullback move")

        # compression angle check
        decompression_angle = angles[3]
        min_angle = self._decompression_angles[0]
        max_angle = self._decompression_angles[1]
        if not (min_angle < decompression_angle < max_angle):
            raise TapValidationError("DECOMPRESSION_ANGLE",
                "Decompression angle violated constraints: %f < %f < %f" % (
                    min_angle, decompression_angle, max_angle))

        # baseline force check
        baseline_force_delta = abs(comp_start.force - decomp_end.force)
        max_baseline_delta = compression_force * self._max_baseline_change_pct
        if baseline_force_delta > max_baseline_delta:
            raise TapValidationError("BASELINE_FORCE_INCONSISTENT",
                "The baseline force before and after the tap are inconsistent")


def load_config(config):
    return SimpleTapClassifier(config)
