# Load Cell Probes

## Related Documentation

* [load_cell_probe](Config_Reference.md#load_cell_probe) Config Reference
* [load_cell](Config_Reference.md#load_cell) Config Reference
* [load_cell](G-Codes.md#load_cell) G-Code Commands

## Load Cell Probe Safety

The load cell probing system includes a number of safety checks that try to
keep your machine safe from excessive force to the toolhead. It's important to
understand what they are and how they work as you can defeat most of them with
badly chosen config values.

* Calibration Check: every time a homing move starts, load_cell_probe checks
that the load_cell is calibrated. If not it will stop the move with an error:
`!! Load Cell not calibrated`.
* `counts_per_gram`: this setting is used to convert raw sensor counts into
grams. All the safety limits are in gram units for your convenience. If the
`counts_per_gram` setting is not accurate you can easily exceed the safe force
on the toolhead.
* `reference_tare_counts`: this is the baseline tare value and can only be
changed by config. Think of this as the "true zero" of the load cell. This value
works with `safety_limit_grams` to limit the maximum force on the toolhead.
* `safety_limit_grams`: this is the maximum absolute force in relation to
`reference_tare_counts` that the probe will allow while homing or probing. If
the load_cell_endstop sees this force it will shut down the machine.
Klipper does not retract the probe when doing a single `PROBE`. This can result
in force applied to the toolhead at the end of a probing cycle. If you repeat
the `PROBE` command, load_cell_probe will tear the endstop at the current force.
Multiple cycles of this could result in ever-increasing force on the toolhead.
`safety_limit_grams` stops this cycle from running out of control. If this limit
is violated the resulting error is `!! Load cell endstop: too much force!`.
* `trigger_count`: This setting is similar to the one on regular pin based
endstops. It is intended to debounce the trigger when a pin doesn't make "solid"
contact. In the load_cell_probe this defaults to 1, meaning no debouncing.
Because load cells are analog devices being measured by an ADC they
don't generally need to be debounced.
* `trigger_force_grams`: this is the force in grams that triggers the endstop to
halt the homing move. When a homing move starts the endstop tears itself with
the current reading from the load cell. `trigger_force_grams` is an absolute
delta from the tear value. There is always some overshoot of this value, so be
conservative. e.g. a setting of 100g may result in 350g of peak force before the
toolhead stops. This overshoot will increase with a low sample rate and/or
[multi MCU homing](Multi_MCU_Homing.md).
* Load Cell Endstop Watchdog Task: when homing the load_cell_endstop starts a
task on the MCU to track measurements arriving from the sensor. If the sensor
fails to send measurements for 2 sample periods the watchdog will shutdown the
machine: `!! "LoadCell Endstop timed out waiting on ADC data"`. If this happens
the most likely cause is a fault from the ADC. Inadequate grounding of your
printer can be the root cause. The frame, power supply case and pint bed should
all be connected to ground. You may need to ground the frame in multiple places.
Anodized aluminum extrusions do not conduct electricity well. You might need to
sand the area where the grounding wire is attached to make good electrical
contact.

## Load Cell Probe Setup

This section covers the process for commissioning up a load cell probe.

### Verify the Load Cell First

A `[load_cell_probe]` is also a `[load_cell]` and G-code commands related to
`[load_cell]` work with `[load_cell_probe]`. Before attempting to use a load
cell probe, follow the directions for
[calibrating the load cell](Load_Cell.md#calibrating-a-load-cell) with
`CALIBRATE_LOAD_CELL` and checking its operation with `LOAD_CELL_DIAGNOSTIC`.

### Verify Probe Operation Before Homing

Once you get the load cell part of `[load_cell_probe]` working you should verify
that the probe functionality works before probing or homing the machine:

1. With nothing touching the load cell probe, run `TARE_LOAD_CELL`.
2. Run `QUERY_PROBE`, it should return:

   `// probe: open`

3. Apply a small force to the load cell and run `QUERY_PROBE`. It should report:

   `// probe: TRIGGERED`

If not you should not attempt to use the probe, it may crash your printer. Check
your configuration and `LOAD_CELL_DIAGNOSTIC` carefully to look for issues.

### Viewing Live Load Cell Graphs

It is strongly suggested that you set up the Klipper Load Cell Debugging Tool.
Follow the instruction in the
[Load Cell](Load_Cell.md#viewing-live-load-cell-graphs) documentation.

This tool plots extra graphs and details about probe taps which can be very
helpful to see. Hopefully in the future something similar is built into the
klipper front ends.

### Suggested Probing Temperature

Currently, we suggest keeping the nozzle temperature below the level that causes
the filament to ooze while homing and probing. For most filaments this is a
limit of 150C, but you may need to lower it for PLA. 140C is a good starting
point. This temperature is also low enough not to scar PEI build surfaces.

Fouling of the nozzle and the print bed due to oozing filament is the #1 source
of probing error with the load cell probe. Klipper does not yet have a universal
way to detect poor quality taps due to filament ooze. The existing code may
decide that a tap is valid when it is of poor quality. Classifying these poor
quality taps is an area of active research.

Klipper also lacks support for re-locating a probe point if the
location has become fouled by filament ooze. Modules like `quad_gantry_level`
will repeatedly probe the same coordinates even if a probe previously failed
there.

Give the above it is strongly suggested not to probe at printing temperatures.

### Nozzle Cleaning

Before probing the nozzle should be clean. You can do this manually before
every print. You can also implement a nozzle scrubber and automate the process.
Here is a suggested sequence:

1. Wait for the nozzle to heat up to probing temp (e.g. `M109 S140`)
1. Home the machine (`G28`)
1. Scrub the nozzle on a brush
1. Heat soak the print bed
1. Perform probing tasks: QGL, bed mesh etc.

#### Nozzle Cleaner GCode
[load_cell_probe] support a `nozzle_cleaner_gcode` option. This is invoked when
an invalid tap is detected during a probe. This is of limited use with the
current code because klipper cant detect the poor quality taps caused by ooze.
This is mainly intended to be used in combination with a bad_tap module that can
detect poor quality taps.

### Hot Nozzle Protection

The Voron project has a great macro for protecting your print surface from the
hot nozzle. See [Voron Tap's `activate_gcode`](https://github.com/VoronDesign/Voron-Tap/blob/main/config/tap_klipper_instructions.md)

It is highly suggested to add something like this to your config.

### Temperature Compensation

The nozzle will expand after heating to printing temperatures. This will cause
the nozzle to get longer and closer to the print surface. If we know how much
the nozzle expands its possible to compensate for this with
[[z_thermal_adjust]](Config_Reference.md#z_thermal_adjust). This adjustment will work across a range of printing
temperatures from PLA to ABS.

#### Calculating the `temp_coeff` for `[z_thermal_adjust]`

The easiest way to do this is to test it with a print.

* Print a 100mm x 100mm square of material 1 layer thick.
* Use Baby Stepping to adjust the Z offset while you print until the first layer
looks good. You should find that the print is too low and you have to adjust the
nozzle up, away from the bed.

Take the adjustment value and divide it by the temperature change between the
probing temp and the printin temp. e.g. probing at 140C and printing at 280C and
the adjustment is -0.05mm up.

```
temp_coeff = -0.05 / (280 - 140) = -0.00036
```

Positive values for `temp_coeff` move the nozzle closer to the bed and negative
values move it further away.

#### Configure `[z_thermal_adjust]`
Set up z_thermal_adjust to reference the `extruder` as the source of temperature
data. E.g.:

```
[z_thermal_adjust nozzle]
temp_coeff=-0.00055
sensor=extruder
max_z_adjustment: 0.1
```

## Continuous Tear Filters for Toolhead Load Cells

Klipper implements a configurable IIR filter on the MCU to provide continuous
tearing of the load cell while probing. Continuous tearing means the 0 value
moves with drift caused by external factors like bowden tubes and thermal
changes. This is aimed at toolhead sensors and moving beds that experience lots
of external forces that change while probing.

### Installing SciPy

The filtering code uses the excellent [SciPy](https://scipy.org/) library to
compute the filter coefficients based on the values your enter into the config.
It is also used to filter the force data if you use the `tap_filter_notch`
option.

Pre-compiled SciPi builds are available for Python 3 on 32 bit Raspberry Pi
systems. 32 bit + Python 3 is strongly recommended because it will streamline
your install experience. It does work with Python 2 but installation can take
30+ minutes and require installing additional tools. 

```bash
~/klippy-env/bin/pip install scipy
```

### Filter Workbench

The filter parameters should be selected based on drift seen on the printer
during normal operation. A Jupyter notebook is provided in scripts,
[filter_workbench.ipynb](../scripts/filter_workbench.ipynb), to perform a
detailed investigation with real captured data and FFTs.

### Filtering Suggestions

For those just trying to get a filter working follow these suggestions:

* The only essential option is `continuous_tare_highpass`. A conservative
starting value is `0.5`Hz. Prusa shipped the MK4 with a setting of `0.8`Hz and
the XL with `11.2`Hz. This is probably a safe range to experiment with. This
value should be increased only until normal drift due to bowden tube force is
eliminated. Setting this value too high will result in slow triggering and
excess force going through the tool head.
* Keep `continuous_tare_trigger_force_grams` low. The default is `40`g. The
filter keeps the internal grams value very close to 0 so a large trigger force
is not needed.
* Keep `safety_limit_grams` to a conservative value. The default value is 1Kg
and should keep your toolhead safe while experimenting. If you hit this limit
the `continuous_tare_highpass` value may be too high, or your
`reference_tare_counts` may need adjusting to be closer to the sensors 0 at
startup.

## Suggestions for Load Cell Tool Boards

This section covers suggestions for those developing toolhead boards that want
to support [load_cell_probe]

### ADC Sensor Selection

Ideally a sensor would meet these criteria:

* At least 24 bits wide
* Use SPI communications
* Has a pin can be used to indicate sample ready without SPI communications.
This is often called the "data ready" or "DRDY" pin. Checking a pin is much
faster than running an SPI query.
* Has a programmable gain amplifier gain setting of 128. This should eliminate
the need for a separate amplifier.
* Indicates via SPI if the sensor has been reset. Detecting resets avoids
timing errors in homing and using noisy data at startup. It can also help users
track down wiring and grounding issues.
* A selectable sample rate between 500Hz and 2Khz. Higher sample rates don't
turn out to be beneficial in our 3D printers because they produce so much noise
when moving fast. Sample rates below 250Hz will require slower probing speeds.
They also increase the force on the toolhead due to longer delays between
measurements. E.g. a 500Hz sensor moving at 5mm/s has the same safety factor as
a 100Hz sensor moving at only 1mm/s.
* If designing for under-bed applications, and you want to sense multiple load
cells, use a chip that can sample all of its inputs simultaneously. Multiplex
ADCs that require switching channels have a settling of several samples after
each channel switch making them unsuitable for probing applications.

Implementing support for a new sensor chip is not particularly difficult with
Klipper's `bulk_sensor` infrastructure.

### 5V Power Filtering

We strongly suggest using larger capacitors than specified by the ADC chip
manufacturer. ADC chips are usually targeted at low noise environments, like
battery powered devices. Sensor manufacturers suggested application notes
generally assume a quiet power supply. Treat their suggested capacitor values as
minimums.

3D printers put huge amounts of noise onto the 5V bus and this can ruin the
sensor's accuracy. Test the sensor on the board with a typical 3D printer power
supply and active stepper drivers before deciding on smoothing capacitor sizes.

### Grounding & Ground Planes

Analog ADC chips contain components that are very vulnerable to noise and
ESD. A large ground plane on the first board layer under the chip can help with
noise. Keep the chip away from power section/DC to DC converters. The board
should have proper grounding back to the DC supply or earth is available.

### HX711 and HX717 Notes

We know this sensor is popular because of its low cost and availability in the
supply chain. However, this is a sensor with several drawbacks:

* The HX71x sensors use bit-bang communication which has a high overhead on the
MCU. Using a sensor that communicates via SPI would save resources on the tool
board's CPU.
* The HX71x lacks a way to communicate reset events to the MCU. Klipper detects
resets with a timing heuristic but this is not ideal.
* For probing applications we greatly prefer the HX717 version because of its
higher sample rate (320 vs 80). Probing speed on the HX711 should be limited to
less than 5mm/s.
* If designing a board for an under-bed sensor with multiple chips, the clock
lines should be tied to an external clock source. Klipper can compensate for a
single chip's clock drift. But for multiple chips with independent clock drift
the estimated measurement time will be less accurate. A simultaneous sampling
4-channel ADC chip should be preferred.
