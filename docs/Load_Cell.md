# Load Cell

This document describes Klipper's support for load cells and load cell based
probes.

# Configuration Reference
TODO: move this to the Configuration Reference document

## [load_cell]
A load_cell wraps a sensor and turns it into a digital scale.

```
[load_cell my_scale]
sensor_type: hx717
#   This must be the name of one of the supported sensor types
counts_per_gram: 1.12345
#   The number of sensor counts that indicates 1 gram of force. This is
#   calculated by the CALIBRATE_LOAD_CELL command. The default value is None.
#   DO NOT copy this value from another printers config, run CALIBRATE_LOAD_CELL
reference_tare_counts: 661806
#   This is the tare value in raw sensor counts taken when CALIBRATE_LOAD_CELL
#   is run. The default value is None.
#   DO NOT copy this value from another printers config, run CALIBRATE_LOAD_CELL
```

### Sensor Support
Currently 2 sensors chips are supported: HX711 and HX717

#### HX71x Chips
The HX717 and HX711 are low cost 24 bit ADC chips that support 2 channels and a
limited number of gain settings. They only support bit bang communications and 
the protocol is based on tight timing which requires turning off the interrupts 
on the MCU (similar to the Neopixel chips). The HX711 is limited to 80 SPS and 
the HX717 to 320SPS. Sample rate is selected by either supplying a voltage or 
grounding one of the pins on the chip, so it cant be changed by klipper.

##### hx711
```
[load_cell my_scale]
sensor_type: hx711
sclk_pin: PB7
#   connected to the clock pin
dout_pin: PB6
#   connected to the data output pin
#gain: A-128
#   Valid values for `gain` are `A-128`, `A-64`, `B-32`. The default is `A-128`. 
#   `A` denotes the input channel and the number denotes the gain. Only the 3 
#   listed combinations are supported by the chip. Note that changing the gain 
#   setting also selects the channel being read.
#sample_rate: 10
#   Valid values for `sample_rate` are `10` or `80`. The default value is `10`.
#   This must match the wiring of the chip. The sample rate cannot be changed 
#   in software.
```

##### hx717
```
[load_cell my_scale]
sensor_type: hx711
sclk_pin: PB7
#   connected to the clock pin
dout_pin: PB6
#   connected to the data output pin
#gain: A-64
#   Valid values for `gain` are `A-128`, `B-64`, `A-64`, `B-8`. 
#   `A` denotes the input channel and the number denotes the gain setting. 
#   Only the 4 listed combinations are supported by the chip. Note that 
#   changing the gain setting also selects the channel being read.
#sample_rate: 320
#   Valid values for `sample_rate` are `320`, `80`, `20` or `10`. 
#   The default is `320`. This must match the wiring of the chip.
#   The sample rate cannot be changed in software.
```

## [load_cell_probe]
[load_cell_probe] wraps [load_cell] and turns it into a [probe]. Consult [probe]
for the full list of options

```
[load_cell_probe]
load_cell: load_cell my_scale
# The complete name of a load cell printer object. Required.
safety_limit_grams: 1000g
#   The safe limit for probing force relative to the reference_tare_counts on
#   the load_cell. The default is +/-1Kg.
trigger_force_grams: 50.0
#   The force that the probe will trigger at. 50g is the default.
continuous_tear_highpass: 0.8
#   Enable optional continuous tearing while homing & probing to reject drift.
#   The value is a frequency, in Hz, below which drift will be ignored.This
#   option requires the SciPy library. Default: None
continuous_tear_lowpass: 100.0
#   The value is a frequency, in Hz, above which high frequency noise in the
#   load cell will be igfiltered outnored. If this option is set,
#   continuous_tear_highpass must also be set. Default: None
continuous_tear_notch: 50, 60
#   1 or 2 frequencies, in Hz, to filter out of the load cell data. This is
#   intended to reject power line noise. If this option is set,
#   continuous_tear_highpass must also be set. Default: None
continuous_tear_notch_quality: 2.0
#   Controls how narrow the range of frequencies are that the notch filter
#   removes. Larger numbers produce a narrower filter. Minimum value is 0.5 and
#   maximum is 3.0. Default: 2.0
continuous_tear_trigger_force_grams: 40.0
#   The force that the probe will trigger at whe using the continuous tearing
#   filter. 40g is the default.
tap_filter_notch: 60.0
tap_filter_notch_quality: 2.0
#   Filters the load cell data before the tap is evaluated. This option may
#   provide marginal accuracy improvement when notch filtering at the mains
#   power frequency. Requires SciPy. Default: None
trigger_count: 1
#   The number of samples over the trigger_force_grams threshold that will cause
#   the probe to trigger. 1 is the default.
pullback_dist: 0.1
#   The distance of the pullback move in mm. This move needs to be long enough 
#   to bring the probe away from the bed after it makes contact.
pullback_speed: 0.4
#   Speed of the pullback move. The default value is to move at a speed of 1
#   sample every 1 micron based one the sensors sample rate is.
settling_time: 0.375
#   Additional time to wait before taring the probe. This allows any vibrations
#   to settle and bowden tubes time to flex etc. This improves repeatability.
#   If the continuous_tear_filter is used this may be set to 0.
pullback_extra_time: 0.3
#   Time to collect additional samples after the pullback move ends in seconds.
#   This improves accuracy by giving the algorithm more points after the probe
#   breaks contact with the bed. Disabling this entirely may impact reliability.
tare_count: 16
#   The number of samples to use when automatically taring the load_cell before
#   each probe. The default value is: sample_per_second * (1 / 60) * 4. This
#   collects samples from 4 cycles of 60Hz mains power to cancel power line
#   noise.
bad_tap_module:
#   Name of a printer object that implements the BadTapModule interface. This
#   checks taps to see if they meet minimum requirements and can
nozzle_cleaner_module:
#   Name of a printer object that implements the NozzleCleanerModule interface
#   than can handle nozzle cleaning. If one is provided the nozzle_cleaner_gcode
#   is disabled.
nozzle_cleaner_gcode:
#   A Gcode macro that is called when a bad tap is detected and the nozzle needs
#   to be cleaned. The default Gcode prints a warning to the console.
z_offset:
speed:
samples:
sample_retract_dist:
lift_speed:
samples_result:
samples_tolerance:
samples_tolerance_retries:
activate_gcode:
deactivate_gcode:
#   See the "probe" section for a description of the above parameters.
```

## [load_cell] GCode Commands

### TARE_LOAD_CELL load_cell="my_scale"
This command works just like the tare button on scale. It sets the current raw
reading of the load cell to be the zero point reference value. The response is
the percentage of the sensors range that was read and the raw value in counts.

```
Load cell tare value: 0.05% (32)
```

Each load cell need to be tared each time the printer starts. Think of this as
similar to a stepper axis that needs to be homed. You should make sure that
there is no load on the load cell (e.g. by lifting/dropping the Z axis) before
taring the load cell in your startup script.

TODO: allow tare value in config

### CALIBRATE_LOAD_CELL load_cell="my_scale"
Start the guided calibration utility. Calibration is a 3 step process:
1. First you remove all load from the load cell and run the `TARE` command
1. Next you apply a known load to the load cell and run the
`CALIBRATE GRAMS=nnn` command
1. Finally use the `ACCEPT` command to save the results

You can cancel the calibration process at any time with `ABORT`.

The second step can be accomplished in a number of ways. If your load cell is
under a platform like a bed or filament spool it might be easy to put a known
mass on the platform. If your load cell is in the printers toolhead it will be
difficult to add a known mass directly to the load cell. in this case its easier
to put a gram scale on the printers bed and gently lower the toolhead onto the
scale (or raise the bed into the toolhead) until you see a good value on the
gram scale. Then enter that value for `CALIBRATE GRAMS=nnn`.

A good calibration force would ideally be a large percentage of the load cell's
rated capacity. E.g. if you have a 5Kg load cell you would ideally calibrate it
with a 5kg mass. However with toolhead probes this may not be a load that your
printer bed or toolhead can tolerate without damage. Do try to use at least 500g
of force, most printers should tolerate that without issue.

A typical calibration session looks like this:
```
$ CALIBRATE_LOAD_CELL LOAD_CELL="tool1"
// Starting load cell calibration.
// 1.) Remove all load and run the TARE command.
// 2.) Apply a known load an run CALIBRATE GRAMS=nnn command.
// Complete calibration with the ACCEPT command.
// Use the ABORT command to quit.
$ TARE
// Load cell tare value: -0.90% (-19266026)
// Now apply a known force to the load cell and enter the force value with:
// CALIBRATE GRAMS=nnn
$ CALIBRATE GRAMS=555
// Calibration value: -2.78% (-59803108), Counts/gram: 73039.78739,
Total capacity: +/- 29.14Kg
// WARNING: Load cell capacity is more than 25Kg!
// Check wiring and consider using a higher sensor gain.
// Accept calibration with ACCEPT command.
$ ACCEPT
// Load cell calibrated, counts per gram: 73039.787387
// The SAVE_CONFIG command will update the printer config file with the above
// and restart the printer.
```

When calibrating make careful note of the values reported:
```
$ CALIBRATE GRAMS=555
// Calibration value: -2.78% (-59803108), Counts/gram: 73039.78739,
Total capacity: +/- 29.14Kg
```
The `Total capacity` should be close to the rating of the load cell itself. If
it is much larger you could have used a higher gain setting in the sensor or a
more sensitive load cell. This isn't as critical for 32bit and 24bit sensors but
is much more critical for low bit width sensors.

### READ_LOAD_CELL load_cell="name"
This command takes a reading from the load cell and displays it as a single line
in the console. Normal calibrated readings include the grams and the percentage
of the sensors range:
```
150.3g (5.12%)
```

Readings from a load cell that is not calibrated or has not been tared will
result in output with no grams:
```
---.-g (15.25%)
```

Readings at the limit of the sensors range will result in an error. This means
that the sensor is at saturation and the true force is unknown.
```
Err (100.00%)
```

### LOAD_CELL_DIAGNOSTIC load_cell="name"
This command collects 10 seconds of load cell data and reports statistics that
may help you diagnose problems.

```
$ LOAD_CELL_DIAGNOSTIC LOAD_CELL="tool0"
// Collecting load cell data for 10 seconds...
// Samples Collected: 108
// Good samples: 108, Saturated samples: 0, Unique values: 21
// Sample range: [-6.05% to -0.78%]
// Sample range / sensor capacity: 2.64%
// Errors: 0
```

Each line explained:
```
// Samples Collected: 108
```
The total number of readings that the load cell received from the sensor during
the test.

```
// Good samples: 108, Saturated samples: 0, Unique values: 21
```
* `Good samples` indicate a regular value between the sensors minimum and
maximum possible value.
* `Saturated samples` indicates a value at the minimum or maximum of the sensors
range. This would indicate that the true force is even higher and the reading is
unusable.
* `Unique values` indicates the number of distinct readings returned. If this
number is 1 it probably indicates that the sensor is disconnected.

```
// Sample range: [-6.05% to -0.78%]
```
Sensors report a range from +100% to -100% based on the number of bits that the
sensor has. This gives a total range of 200%. When no load is applied these
values should be close to 0%. If you tap or push on the sensor while running
`LOAD_CELL_DIAGNOSTICS` this range should increase.

```
// Sample range / sensor capacity: 2.64%
```
This gives the size of the detected values as a percentage of the total 200%
range of the sensor. If no force is applied this is a direct measurement of the
noise and should be < 5%, lower is better. If you tap or push on the sensor the
range should increase. If you do not see an increase there may be a wiring
issue.
