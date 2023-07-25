# Load Cell

This document describes Klipper's support for load cells and load cell based probing.

## TARE_LOAD_CELL load_cell="name"
This command works just like the tare button on scale. It sets the current raw reading of the load cell to be the zero point reference value. The response is the percentage of the sensors range that was read and the raw value in counts.
```
Load cell tare value: 0.05% (32)
```

Each load cell need to be tared each time the printer starts. Think of this as similar to a stepper axis that needs to be homed. You should make sure that there is no load on the load cell (e.g. by lifting/dropping the Z axis) before taring the load cell in your startup script.

When using the load cell as a probe this operation also configured the zero point and range for the endstop on the MCU. If the tare value changes significantly before probing you should use `TARE_LOAD_CELL` before probing (TK: maybe this can be automated with continuous tare)

## CALIBRATE_LOAD_CELL load_cell="name"
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

## READ_LOAD_CELL load_cell="name"
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

## LOAD_CELL_DIAGNOSTICS load_cell="name"
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
The total number of same readings that the load cell received from the sensor during the test

```
// Good samples: 108, Saturated samples: 0, Unique values: 21
```
* `Good samples` indicate a regular value between the sensors minimum and maximum possible value.
* `Saturated samples` indicates a value at the minimum or maximum of the sensors range. This would indicate that the true force is even higher and the reading is unusable
* `Unique values` indicates the number of distinct counts returned. If this number is 1 is probably indicates that the sensor is disconnected.

```
// Sample range: [-6.05% to -0.78%]
```
Sensors report a range from +100% to -100% based on the number of bits that the sensor has. This gives a total range of 200%. When no load is applied these values should be close to 0%. If you tap or push on the sensor while running `LOAD_CELL_DIAGNOSTICS` this range should increase.

// Sample range / sensor capacity: 2.64%
This gives the size of the detected values as a percentage of the total 200% range of the sensor. If no force is applied this is a direct measurement of the noise and should be < 5%, lower is better. If you tap or push on the sensor ths range should increase. If you do not see an increase there may be a wiring issue.

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
1. `ERROR_SAMPLE_NOT_READY` - The sensor was not ready to produce a new sample value when it was read. This is mostly a warning, but large numbers of these errors indicate that the Sample Per Second of the sensor is misconfigured.
1. `ERROR_CRC` - The cyclic redundancy check on the sensor reading failed, meaning the reading is corrupt. Check sensor wiring for loose connections or sources of interference.
1. `ERROR_READ_TIME` - The read process took longer than is allowed on th eMCU. This can be due to high MCU load or a gredy MCU task that interrupts the sensor read for a long time. This indicates that the reading has been lost.
1. `ERROR_UNKNOWN` - The error code is unregistered in the klippy client. If this happens its due to a bug. (protects against future code changes that introduce new errors)



