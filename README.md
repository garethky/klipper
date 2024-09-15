Load Cell Community Testing Branch

If you are testing load cells in klipper this is the branch to use!

Please review the documentation:
* [Load Cell](./docs/Load_Cell.md)
* [Load Cell Probe](./docs/Load_Cell_Probe.md)

There is a debugging tool that will let you see the lod cell output and 
probing taps:[Klipper Load Cell Debugging Tool](https://observablehq.com/@garethky/klipper-load-cell-debugging-tool)
If you are testing you should get this set up to see how things are working.

#### Progress Tracker:
This is being slowly shipped to klipper's mainline branch. It's a very large 
change and has to be broken up into multiple PRs to make reviewing easier:

PRs to klipper Mainline:
1. [Bulk ADC Sensors](https://github.com/Klipper3d/klipper/pull/6555) - Shipped
2. [Add input_mux and vref options to ADS1220 sensor](https://github.com/Klipper3d/klipper/pull/6713) - Shipped
3. [Load cell gram scale](https://github.com/Klipper3d/klipper/pull/6729) - Shipped
4. [PR: Enable multiple z_thermal_adjust sections](https://github.com/Klipper3d/klipper/pull/6855) - Active
5. [PR: Load cell endstop](https://github.com/Klipper3d/klipper/pull/6871) - Active

PRs not yet submitted:
1. Pullback move & Tap Analysis
2. `average_delta` metric for `PROBE_ACCURACY` command
---

Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).
