# Load Cell

This document describes Klipper's support for load cells and load cell based probing.

## Sensor Support
In development 3 sensors chips are supported: HX711, HX717 and ADS1263.

### HX71x Chips
The HX717 and HX711 are low cost 24 bit ADC chips that support2 channels and a limited number of gain settings. They only support bit bang communications and the protocol is based on tight timing which requires turning off the interrupts on the MCU (similar to the Neopixel chips). The HX711 is limited to 80 SPS and the HX711 to 340SPS. Sample rate is selected by either supplying a voltage or grounding on of the legs on the chip.

#### [hx711]
```
[hx711 sensor_name]
sclk_pin: PB7
#   connected to the clock pin
dout_pin: PB6
#   connected to the data output pin
gain: A-128
#   Valid values for `gain` are `A-128`, `A-64`, `B-32`. The default is `A-128`. 
#   `A` denotes the input channel and the number denotes the gain. Only the 3 
#   listed combinations are supported by the chip. Note that changing the gain 
#   setting also selects the channel being read.
sample_rate: 10
#   Valid values for `sample_rate` are `10` or `80`. The default value is `10`.
#   This must match the wiring of the chip. The sample rate cannot be changed 
#   in software.
```

#### [hx717]
```
[hx717 sensor_name]
sclk_pin: PB7
#   connected to the clock pin
dout_pin: PB6
#   connected to the data output pin
gain: A-64
#   Valid values for `gain` are `A-128`, `B-64`, `A-64`, `B-8`. 
#   `A` denotes the input channel and the number denotes the gain setting. 
#   Only the 4 listed combinations are supported by the chip. Note that 
#   changing the gain setting also selects the channel being read.
sample_rate: 10
#   Valid values for `sample_rate` are `320`, `80`, `20` or `10`. 
#   The default is `320`. This must match the wiring of the chip.
#   The sample rate cannot be changed in software.
```

`sclk_pin` is connected to the clock pin and `dout_pin` is connected to the data output pin. These parameters are required and have no defaults.

Valid values for `gain` are `A-128`, `B-64`, `A-64`, `B-8`. `A` denotes the input channel and the number denotes the gain setting. Only the 4 listed combinations are supported by the chip. Note that changing the gain setting also selects the channel being read.


### ADS1263 Chip

The ADS1263 is a more capable 32 bit sensor with 4 input channels, PGA and up to 32Khz read speeds. Unfortunately its both large and expensive. It was included primarily as a development platform because it can mimic the characteristics of other sensors.

This chip supports CRC for sensor reading integrity and has a chip power cycle event flag so restarts of the chip can be detected. These are important safety features missing in the HX71x chips.

#### [ads1263]
```
[ads1263 sensor_name]
spi_bus: spi3
cs_pin: SPI3_CS
#   See the "common SPI settings" section for a description of the
#   above parameters.
sample_rate: 8
#   The chip supports sampling at 2.5, 5, 10, 16.6, 20, 50, 60, 100, 400, 1200,
#   2400, 4800, 7200, 14400, 19200, and 38400 samples per second.
#   The value selects an index in that list. e.g. 8 is 400 SPS.
gain: 5
#   The chip supports gain settings of 1, 2, 4, 8, 16 and 32. 
#   The `gain` parameter is the index in that list. e.g. 5 is a gain of 32.
```
Channel, gain, filtering and sample frequency and much more are all independently configurable in software. If interested, read the code in ads1263.py for full options list.

This chip also has a set of GCode commands that let you configure more advanced chip functions, dump internal registers etc. Again, if interested, read ads1263.py.

## [load_cell]
A load_cell wraps a sensor and turns it into a digital scale and, optionally, a printer probe.

```
[load_cell my_scale]
sensor: hx717 sensor_name
#   This must be the full name of one of the compatible sensors
counts_per_gram: 1
#   This value converts raw sensor counts to grams.
#   It is set with the CALIBRATE_LOAD_CELL command.
#   The default is 1 as a safety measure.
#   DO NOT COPY this value from another printers config.
is_probe: True
#   Enable probe support, the default is False
trigger_force_grams: 50.0
#   The force that the probe will trigger at. 50g is the default.
z_offset: 0.0
#   Probe z_offset, see [probe] for details
```

## [load_cell] GCode Commands

### TARE_LOAD_CELL load_cell="name"
This command works just like the tare button on scale. It sets the current raw reading of the load cell to be the zero point reference value. The response is the percentage of the sensors range that was read and the raw value in counts.
```
Load cell tare value: 0.05% (32)
```

Each load cell need to be tared each time the printer starts. Think of this as similar to a stepper axis that needs to be homed. You should make sure that there is no load on the load cell (e.g. by lifting/dropping the Z axis) before taring the load cell in your startup script.

When using the load cell as a probe this operation also configured the zero point and range for the endstop on the MCU. If the tare value changes significantly before probing you should use `TARE_LOAD_CELL` before probing (TK: maybe this can be automated with continuous tare)

### CALIBRATE_LOAD_CELL load_cell="name"
This command started the guided calibration utility. Calibration is a 3 step process:
1. First you remove all load from the load cell and run the `TARE` command
1. Next you apply a known load to the load cell and run the `CALIBRATE GRAMS=nnn` command
1. Finally use the `ACCEPT` command to  save the results

You can cancel the calibration process at any time with `ABORT`.

The second step can be accomplished in a number of ways. If your load cell is under a platform like a bed or filament spool it might be easy to put a known mass on the platform. If your load cell is in the printers toolhead it will be difficult to add a known mass directly to the load cell. in this case its easier to put a gram scale on the printers bed and gently lower the toolhead onto the scale (or raise the bed into the toolhead) until you see a good value on the gram scale. Then enter that value for `CALIBRATE GRAMS=nnn`.

A good calibration force would ideally be a large percentage of the load cell's rated capacity. E.g. if you have a 5Kg load cell you would ideally calibrate it with a 5kg mass. However with toolhead probes this may not be a load that your printer bed or toolhead can tolerate without damage. Do try to use at least 500g of force, most printers should tolerate that without issue.

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
The `Total capacity` should be close to the rating of the load cell itself. If it much larger you could have used a higher gain setting in the sensor or a more sensitive load cell. This isn't as critical for 32bit and 24bit sensors but is much more critical for low but width sensors.

### READ_LOAD_CELL load_cell="name"
This command takes a reading from the load cell and displays it as a single line in the console:

Normal calibrated readings include the grans and the percentage of the sensors
 range:
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
This command collects 10 seconds of load cell data and reports statistics that may help you diagnose problems.

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
The total number of numeric readings that the load cell received from the sensor during the test

```
// Good samples: 108, Saturated samples: 0, Unique values: 21
```
* `Good samples` indicate a regular value between the sensors minimum and maximum possible value.
* `Saturated samples` indicates a value at the minimum or maximum of the sensors range. This would indicate that the true force is even higher and the reading is unusable
* `Unique values` indicates the number of distinct counts returned. If this number is 1 it probably indicates that the sensor is disconnected.

```
// Sample range: [-6.05% to -0.78%]
```
Sensors report a range from +100% to -100% based on the number of bits that the sensor has. This gives a total range of 200%. When no load is applied these values should be close to 0%. If you tap or push on the sensor while running `LOAD_CELL_DIAGNOSTICS` this range should increase.

```
// Sample range / sensor capacity: 2.64%
```
This gives the size of the detected values as a percentage of the total 200% range of the sensor. If no force is applied this is a direct measurement of the noise and should be < 5%, lower is better. If you tap or push on the sensor the range should increase. If you do not see an increase there may be a wiring issue.

```
// Errors: 0
```
The number of errors that the sensor returned during the sample period. If the
 load cell receives any errors from the sensor these will be printed as a map:

```
// Errors: 40
// Error breakdown: {'ERROR_SAMPLE_NOT_READY': 10, 'ERROR_CRC': 10, 
'ERROR_READ_TIME': 10, 'ERROR_UNKNOWN': 10}
```

Here is what each of these errors mean:
1. `ERROR_SAMPLE_NOT_READY` - The sensor was not ready to produce a new sample value when it was read. This is mostly a warning, but large numbers of these errors indicate that the `sample_rate` of the sensor is misconfigured. The reading was lost.
1. `ERROR_CRC` - The cyclic redundancy check on the sensor reading failed, meaning the reading is corrupt. Check sensor wiring for loose connections or sources of interference. The reading was discarded.
1. `ERROR_READ_TIME` - The read process took longer than is allowed on the MCU. This can be due to high MCU load or a greedy MCU task that interrupted the sensor read for a long time. This indicates that the reading has been lost.
1. `ERROR_UNKNOWN` - The error code is unregistered in klippy. If this happens its due to a bug. (protects against future code changes that introduce new error types)



