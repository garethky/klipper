from . import load_cell_probe

# Naive tap classifier rules:
# 1) It should pass initial TapAnalysis as a valid tap
# 2) The compression force must be least the trigger force
# 3) The decompression line force should be at least 75% of the force
#    in the compression line
# 4) There should be less than a 20% difference in force between the collision
#    and the pullback elbows relative to the compression force.

class SimpleTapClassifier(load_cell_probe.TapClassifierModule):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.trigger_force = float(config.getint(
            'trigger_force', minval=10, default=75))
        self.min_decomp_force_pct = 0.01 * config.getfloat(
            'min_decompression_force_percentage',
            minval=50. , default=66.66, maxval=100.)
        self.max_baseline_change_pct = 0.01 * config.getfloat(
            'max_baseline_force_change_percentage',
            above=0., maxval=50., default=20.)

    def classify(self, tap_analysis):
        # compression line check
        tap_points = tap_analysis.tap_points
        comp_start = tap_points[1]
        comp_end = tap_points[2]
        compression_force = abs(comp_end.force - comp_start.force)
        if compression_force < self.trigger_force:
            tap_analysis.errors.append("INSUFFICIENT_COMPRESSION_FORCE")

        # decompression line check
        decomp_start = tap_points[3]
        decomp_end = tap_points[4]
        decompression_force = abs(decomp_start.force - decomp_end.force)
        min_decompression_force = (
                (self.trigger_force * self.min_decomp_force_pct) +
                ((compression_force - self.trigger_force) * 0.33))
        if decompression_force < min_decompression_force:
            tap_analysis.errors.append("INSUFFICIENT_DECOMPRESSION_FORCE")

        # baseline check
        baseline_force_delta = abs(comp_start.force - decomp_end.force)
        max_baseline_delta = compression_force * self.max_baseline_change_pct
        if baseline_force_delta > max_baseline_delta:
            tap_analysis.errors.append("BASELINE_FORCE_INCONSISTENT")

        #TODO: sharpness check of pullback elbow
        # 5) The pullback elbow should be "sharp", on both sides
        #     Make this a simple %. Default value? Over what time scale?

        # if all these checks pass but there were other errors return
        # mostly doing it this way to see how this performs in the cases
        # where the tap is invalid
        if len(tap_analysis.errors) > 0:
            tap_analysis.is_valid = False

def load_config(config):
    return SimpleTapClassifier(config)
