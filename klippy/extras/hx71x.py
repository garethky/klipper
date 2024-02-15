# HX711/HX717 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging, struct
from . import bulk_sensor
from .bulk_sensor_adc import (BulkSensorAdc, LoadCellEndstopSensor,
                              TimestampHelper)

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_BLOCK = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10
MAX_CHIPS = 4

# Implementation of HX711 and HX717
# This supports up to 4 sensor being read in parallel for under-bed load cell
# applications. It exposes a sum of all sensor outputs via subscribe(). Also,
# each sensor's data can be read individually via subscribe_sensor(1-4). If any
# sensor becomes saturated the output of the sum is the saturated value.
class HX71xBase(BulkSensorAdc, LoadCellEndstopSensor):
    def __init__(self, config,
                 sample_rate_options, default_sample_rate,
                 gain_options, default_gain, allocate_endstop_oid=False):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.query_hx71x_cmd = None
        self.reset_hx71x_cmd = None
        ## Chip options
        chips = []
        for i in range(1, 5):
            dout_pin = config.get('dout_pin%i' % (i,), default=None)
            if dout_pin is None:
                break
            sclk_pin = config.get('sclk_pin%i' % (i,), default=None)
            if sclk_pin is None:
                raise config.error("HX71x config missing sclk_pin%i" % (i,))
            chips.append((dout_pin, sclk_pin))
        self.chip_count = len(chips)
        if self.chip_count < 1:
            raise config.error("HX71x config error: "
                "The minimum number of sensor chips is 1")
        ppins = printer.lookup_object('pins')
        dout_pins = []
        sclk_pins = []
        for i, (dout_pin, sclk_pin) in enumerate(chips):
            dout_pins.append(ppins.lookup_pin(dout_pin))
            sclk_pins.append(ppins.lookup_pin(sclk_pin))
        self.mcu = dout_pins[0]['chip']
        self.oid = self.mcu.create_oid()
        # once the MCU is known the endstop oid can be created:
        self.lce_oid = 0
        if allocate_endstop_oid:
            self.lce_oid = self.mcu.create_oid()
        for pin in (dout_pins + sclk_pins):
            if pin['chip'] is not self.mcu:
                raise config.error("HX71x config error: "
                    "All HX71x chips must be connected to the same MCU")
        #TODO: REVIEW: what do about unused pins in the command?
        # the pin names have to be valid, and they have to be strings
        # copying names feels hacky, is there a 'NONE' value?
        for i in range(self.chip_count, MAX_CHIPS):
            dout_pins.append(dout_pins[0])
            sclk_pins.append(sclk_pins[0])
        self.dout_pins = [pin_params['pin'] for pin_params in dout_pins]
        self.sclk_pins = [pin_params['pin'] for pin_params in sclk_pins]
        # Samples per second choices
        self.sps = config.getchoice('sample_rate', sample_rate_options,
                                    default=default_sample_rate)
        # set rest_ticks as a % of the sample_rate
        self.duty_cycle = config.getfloat('duty_cycle',
                                          minval=0.1, maxval=1.0, default=0.7)
        # gain/channel choices
        self.gain_channel = int(config.getchoice('gain', gain_options,
                                                 default=default_gain))
        ## Command Configuration
        self.mcu.register_config_callback(self._build_config)
        ## Measurement conversion
        self.bytes_per_block = BYTES_PER_SAMPLE * self.chip_count
        self.blocks_per_msg = (bulk_sensor.MAX_BULK_MSG_SIZE
                               // self.bytes_per_block)
        block_format = "<%di" % (self.chip_count,)
        self._unpack_block = struct.Struct(block_format).unpack_from
        ## Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.clock_sync = bulk_sensor.ClockSyncRegression(self.mcu, chip_smooth)
        self.clock_updater = bulk_sensor.ChipClockUpdater(self.clock_sync,
                                                          self.bytes_per_block)
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)

        fields = ['time', 'total_counts']
        for i in range(0, self.chip_count):
            fields.append("counts%i" % (i,))
        # publish raw samples to the socket
        self.batch_bulk.add_mux_endpoint("hx71x/dump_hx71x", "sensor",
                                         self.name, {'header': tuple(fields)})
    def _build_config(self):
        dout_pins = self.dout_pins
        sclk_pins = self.sclk_pins
        logging.info("%s" % (dout_pins[3]))
        temp = ("config_hx71x oid=%d"
            " chip_count=%d gain_channel=%d load_cell_endstop_oid=%d"
            " dout1_pin=%s sclk1_pin=%s dout2_pin=%s sclk2_pin=%s"
            " dout3_pin=%s sclk3_pin=%s dout4_pin=%s sclk4_pin=%s"
            % (self.oid, self.chip_count, self.gain_channel, self.lce_oid,
               dout_pins[0], sclk_pins[0], dout_pins[1], sclk_pins[1],
               dout_pins[2], sclk_pins[2], dout_pins[3], sclk_pins[3]))
        self.mcu.add_config_cmd(temp)
        self.mcu.add_config_cmd("query_hx71x oid=%d rest_ticks=0"
                                % (self.oid,), on_restart=True)
        self.query_hx71x_cmd = self.mcu.lookup_command(
            "query_hx71x oid=%c rest_ticks=%u")
        self.clock_updater.setup_query_command(self.mcu,
            "query_hx71x_status oid=%c", self.oid)
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.sps
    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        range_max = (2 ** 24) * self.chip_count
        return -range_max, range_max
    # add_Client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)
    def get_load_cell_endstop_oid(self):
        return self.lce_oid
    # Measurement decoding
    def _extract_samples(self, raw_samples):
        # local variables to optimize inner loop below
        unpack_block = self._unpack_block
        bytes_per_block = self.bytes_per_block
        # Process every message in capture_buffer
        max_samples = (len(raw_samples) * self.blocks_per_msg)
        samples = collections.deque(maxlen=max_samples)
        timestamps = TimestampHelper(self.clock_sync, self.clock_updater,
                                     self.blocks_per_msg)
        for params in raw_samples:
            timestamps.update_sequence(params['sequence'])
            data = bytearray(params['data'])
            for i in range(len(data) // bytes_per_block):
                counts = unpack_block(data, offset=bytes_per_block * i)
                msg = (timestamps.time_of_msg(i), sum(counts)) + counts
                samples.append(msg)
        timestamps.set_last_chip_clock()
        return list(samples)
    # Start, stop, and process message batches
    def _start_measurements(self):
        # Start bulk reading
        self.bulk_queue.clear_samples()
        rest_ticks = self.mcu.seconds_to_clock(self.duty_cycle / self.sps)
        self.query_hx71x_cmd.send([self.oid, rest_ticks])
        logging.info("HX71x '%s' starting measurements", self.name)
        # Initialize clock tracking
        self.clock_updater.note_start()
    def _finish_measurements(self):
        # Halt bulk reading
        self.query_hx71x_cmd.send_wait_ack([self.oid, 0])
        self.bulk_queue.clear_samples()
        logging.info("HX71x '%s' finished measurements", self.name)
    def _handle_reset(self):
        # HX71x suffered a reboot or timing error, chip was shut down on MCU
        logging.error("HX71x chip '%s' reset", self.name)
        self._finish_measurements()
        self._start_measurements()
    def _process_batch(self, eventtime):
        self.clock_updater.update_clock()
        raw_samples = self.bulk_queue.pull_samples()
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples}

class HX711(HX71xBase):
    def __init__(self, config, allocate_endstop_oid=False):
        super(HX711, self).__init__(config,
            # HX711 sps options
            {80: 80, 10: 10}, 80,
            # HX711 gain/channel options
            {'A-128': 1, 'B-32': 2, 'A-64': 3}, 'A-128',
            allocate_endstop_oid)

class HX717(HX71xBase):
    def __init__(self, config, allocate_endstop_oid=False):
        super(HX717, self).__init__(config,
            # HX717 sps options
            {320: 320, 80: 80, 20: 20, 10: 10}, 320,
            # HX717 gain/channel options
            {'A-128': 1, 'B-64': 2, 'A-64': 3, 'B-8': 4}, 'A-128',
            allocate_endstop_oid)

HX71X_SENSOR_TYPES = {
    "hx711": HX711,
    "hx717": HX717
}
