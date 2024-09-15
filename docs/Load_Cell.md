# Load Cells

This document describes Klipper's support for load cells and load cell based
probes.

## Related Documentation

* [load_cell](Config_Reference.md#load_cell) Config Reference
* [load_cell](G-Codes.md#load_cell) G-Code Commands

## Calibrating a Load Cell

Load cells are calibrated using the `CALIBRATE_LOAD_CELL` command. This is an
interactive calibration utility that walks you though a 3 step process:
1. First use the `TARE` command to establish the zero force value. This is the
`reference_tare_counts` config value.
2. Next you apply a known load or force to the load cell and run the
`CALIBRATE GRAMS=nnn` command. From this the `counts_per_gram` value is
calculated. See [the next section](#applying-a-known-force-or-load) for some
suggestions on how to do this.
3. Finally, use the `ACCEPT` command to save the results.

You can cancel the calibration process at any time with `ABORT`.

### Applying a Known Force or Load

The `CALIBRATE GRAMS=nnn` step can be accomplished in a number of ways. If your
load cell is under a platform like a bed or filament holder it might be easiest
to put a known mass on the platform. E.g. you could use a couple of 1KG filament
spools.

If your load cell is in the printer's toolhead a different approach is easier.
Put a digital scale on the printers bed and gently lower the toolhead onto the
scale (or raise the bed into the toolhead if your bed moves). You may be able to
do this using the `FORCE_MOVE` command. But more likely you will have to
manually moving the z axis with the motors off until the toolhead presses on the
scale.

A good calibration force would ideally be a large percentage of the load cell's
rated capacity. E.g. if you have a 5Kg load cell you would ideally calibrate it
with a 5kg mass. This might work well with under-bed sensors that have to
support a lot of weight. For toolhead probes this may not be a load that your
printer bed or toolhead can tolerate without damage. Do try to use at least 1Kg
of force, most printers should tolerate this without issue.

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

## Using `LOAD_CELL_DIAGNOSTIC`

When you first connect a load cell its good practice to check for issues by
running `LOAD_CELL_DIAGNOSTIC`. This tool collects 10 seconds of data from the
load cell and resport statistics:

```
$ LOAD_CELL_DIAGNOSTIC
// Collecting load cell data for 10 seconds...
// Samples Collected: 3211
// Measured samples per second: 332.0
// Good samples: 3211, Saturated samples: 0, Unique values: 900
// Sample range: [4.01% to 4.02%]
// Sample range / sensor capacity: 0.00524%
```

Things you can check with this data:
* The configured sample rate of the sensor should be close to the 'Measured
samples per second' value. If it is not you may have a configuration or wiring
issue.
* 'Saturated samples' should be 0. If you have saturated samples it means the
load sell is seeing more force than it can measure.
* 'Unique values' should be a good a larger percentage of the 'Samples
Collected' value. If 'Unique values' is 1 it is very likely a wiring issue.
* Tap or push on the sensor while `LOAD_CELL_DIAGNOSTIC` runs. If
things are working correctly ths should increase the 'Sample range'.

## Viewing Live Load Cell Graphs

You can see a live view of the load cell and get plots of load cell probe taps
using a web tool available here:

[Klipper Load Cell Debugging Tool](https://observablehq.com/@garethky/klipper-load-cell-debugging-tool)

The page loads from the secure [observablehq.com](http://observablehq.com)
domain. It doesn't send anything back to observablehq.com and communicates with
the moonraker websocket over your local network. You will need to set up HTTPS
in moonraker to get the websocket connection working. Here's how:

#### 1. Add CORS domains in `moonraker.config`

```
[authorization]
cors_domains:
  *.static.observableusercontent.com
  *.observablehq.com
```

#### 2. Set up HTTPS

Follow these steps:

##### 2.1 Enable the secure port in `moonraker.config`
```
[server]
  ssl_port: 7130
```

##### 2.2 Use OpenSSL to generate a self signed cert for Moonraker:

This assumes you set up Moonraker in the default location in `/home/pi/`:

```
cd ~/printer_data/certs
sudo openssl req -new -x509 -days 365 -nodes -out moonraker.cert -keyout moonraker.key
chmod +r ./*
```

This creates a cert in the directory where Moonraker looks for certificates. The
certificate will last for a year. Then its adds the read permission (`+r`) to
the cert so Moonraker can open it.

#### 3. Restart Moonraker
You can usually do this via your front end of choice. Or run:

```
sudo service moonraker restart
```

#### 4. Accept the self signed cert in your browser
Visit your printers web interface over HTTPS (i.e.
https://yourprinter.local:7130/) to load a page with the cert. You will get
a warning about the cert, after you accept the warning you will be able to
connect to the web socket.

### Configuring the Page

There are 2 boxes on the page:
- `Moonraker hostname`: enter the hostname of your printer without the
`https://` or `:7130`.
e.g. `voron24.local`
- `[load_cell]`, `[load_cell name], [load_cell_probe]`: Enter the name of the
config section that you want to monitor just as it appears in your config.

You might need to reload the page after entering these values to get the
connection to work.
