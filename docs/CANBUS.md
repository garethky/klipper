This document describes Klipper's CAN bus support.

# Device Hardware

Klipper currently only supports CAN on stm32 chips. In addition, the
micro-controller chip must support CAN and it must be on a board that
has a CAN transceiver.

To compile for CAN, run "make menuconfig", unselect "Use USB for
communication (instead of serial)" and then select "Use CAN for
communication (instead of serial)". Finally, compile the
micro-controller code and flash it to the target board.

# Host Hardware

In order to use a CAN bus, it is necessary to have a host adapter.
There are currently two common options:

1. Use a [Waveshare Raspberry Pi CAN
   hat](https://www.waveshare.com/rs485-can-hat.htm) or one of its
   many clones.

2. Use a USB CAN adapter (for example
   [https://hacker-gadgets.com/product/cantact-usb-can-adapter/](https://hacker-gadgets.com/product/cantact-usb-can-adapter/)). There
   are many different USB to CAN adapters available - when choosing
   one, we recommend verifying it can run the [candlelight
   firmware](https://github.com/candle-usb/candleLight_fw). (Unfortunately,
   we've found some USB adapters run defective firmware and are locked
   down, so verify before purchasing.)

It is also necessary to configure the host operating system to use the
adapter. This is typically done by creating a new file named
`/etc/network/interfaces.d/can0` with the following contents:
```
auto can0
iface can0 can static
    bitrate 500000
    up ifconfig $IFACE txqueuelen 128
```

Note that the "Raspberry Pi CAN hat" also requires [changes to
config.txt](https://www.waveshare.com/wiki/RS485_CAN_HAT).

# Run canbus_assigner.py

It is necessary to run a new system process in order for Klipper to
identify CAN devices. There is a one-time step needed to setup this
process so that it runs on every boot.

For those using an OctoPi image, run:
```
~/klipper/scripts/canbus_assigner/install_canbus.sh
```

For those running on other host computers or using a different OS
image, it will be necessary to manually install the canbus_assigner.py
code. See the [install_canbus.sh
script](../scripts/canbus_assigner/install_canbus.sh) for details.

# Configuring Klipper

Each micro-controller on the CAN bus is assigned a unique id based on
the factory chip identifier encoded into each micro-controller. The
canbus_assigner.py code creates a mapping file that contains these
device identifiers.

To find each micro-controller device id, make sure the
canbus_assigner.py code is running, make sure the hardware is powered
and wired correctly, and then run:
```
cat /home/pi/canbus_ids.cfg
```

If CAN devices were detected the above should report lines like the
following:
```
node079 = 11aa22bb33cc
```

Each device will have a unique identifier. In the above example,
`11aa22bb33cc` is the micro-controller's "canbus_uuid". (The number
next to "node" is randomly assigned and can be ignored.)

Update the Klipper configuration to use CAN with this id - for example:
```
[mcu my_can_mcu]
canbus_uuid: 11aa22bb33cc
```
