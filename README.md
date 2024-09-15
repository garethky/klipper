## Load Cell Community Testing Branch 

This is a marker branch for the klipper community to use when testing Load Cell
Probing. This branch contains all the proposed changes that will eventually be
submitted as Pull Requests (PR's) to klipper mainline.

When klipper merges one of these changes this branch will be rebased on top of
that change. This branch will also contain the live updates as the PR's are
worked on and will be updated frequently as the work progresses.

Changes that break printer config are expected to happen in this branch. It is
best to keep track of the ongoing PR's as that is where any config updates will
be discussed.

### Load Cell Help & Resources:
Two docs cover load cells and load cell probing:

* [Load Cells](docs/Load_Cell.md)
* [Load Cell Probing](docs/Load_Cell_Probe.md)

#### Load Cell Klipper Documentation Shortcuts:
* [Configuration Reference](docs/Config_Reference.md#load-cells)
* [G-Code Commands](docs/G-Codes.md#load_cell)
* [Status Reference](docs/Status_Reference.md#load_cell)
* [API Server Reference](docs/API_Server.md#load_celldump_force)

#### Tools
* [Load Cell Live Monitor](https://observablehq.com/@garethky/klipper-load-cell-debugging-tool)
to see load cell data in real time from Moonraker
* [Filter Workbench](scripts/filter_workbench.ipynb) for working with continuous
tearing filters (requires [Jupyter](https://jupyter.org/))

### PR Progress
#### Active PRs
* none

#### Merged PRs
* [Bulk ADS Sensors](https://github.com/Klipper3d/klipper/pull/6555)

### Issues

* ADS1220: allow selecting the Reference and Input channels. Currently it
defaults to  
* 

---

Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use Klipper.

To begin using Klipper start by
[installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the
[documentation](https://www.klipper3d.org/Overview.html). We depend on
the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).
