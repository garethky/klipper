# ADS1220 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bulk_sensor, bus

#
# Constants
#
BYTES_PER_SAMPLE = 4  # samples are 4 byte wide unsigned integers
MAX_SAMPLES_PER_MESSAGE = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE
UPDATE_INTERVAL = 0.10
RESET_CMD = 0x06
START_SYNC_CMD = 0x08
RREG_CMD = 0x20
WREG_CMD = 0x40
NOOP_CMD = 0x0
RESET_STATE = bytearray([0x0, 0x0, 0x0, 0x0])

# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(byte_array):
    return "[%s]" % (", ".join([hex(b) for b in byte_array]))


class ADS1220():
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Chip options
        self.regs = []
        # Configuration Register 0
        adc_MUX = config.getint('adc_MUX', 0x00, minval=0x00, maxval=0x0F)
        # Gain
        gain_options = {'1': 0x0, '2': 0x1, '4': 0x2, '8': 0x3, '16': 0x4,
                             '32': 0x5, '64': 0x6, '128': 0x7}
        adc_GAIN = config.getchoice('gain', gain_options, default='128')
        adc_PGA_BYPASS = config.getint('adc_PGA_BYPASS', 0x00, minval=0x00, maxval=0x01)
        reg_00_value = (adc_MUX << 4) | (adc_GAIN << 1) | adc_PGA_BYPASS
        self.regs.append(reg_00_value)
        # Configuration Register 1
        # Sample rate
        self.sps_normal = {'20': 20, '45': 45, '90': 90, '175': 175,
                           '330': 330, '600': 600, '1000': 1000}
        self.sps_duty = {'5': 5, '11.25': 11.25, '22.5': 22.5, '44': 44,
                         '82.5': 82.5, '150': 150, '250': 250}
        self.sps_turbo = {'40': 40, '90': 90, '180': 180, '350': 350,
                          '660': 660, '1200': 1200, '2000': 2000}
        self.sps_options = self.sps_normal.copy()
        self.sps_options.update(self.sps_duty)
        self.sps_options.update(self.sps_turbo)
        self.sps = config.getchoice('sps', self.sps_options, default='660')
        if str(self.sps) in self.sps_turbo:
            adc_MODE_default = 0x02 # Turbo mode (512-kHz modulator clock)
            sps_list = self.sps_turbo
        elif str(self.sps) in self.sps_duty:
            adc_MODE_default = 0x01 # Duty-cycle mode (internal duty cycle of 1:4)
            sps_list = self.sps_duty
        elif str(self.sps) in self.sps_normal:
            adc_MODE_default = 0x00 # Normal mode (256-kHz modulator clock, default)
            sps_list = self.sps_normal
        else:
            raise config.error("ADS1220 config error: This SPS configuration"
                               " is not supported")
        adc_DR_default = list(sps_list.keys()).index(str(self.sps))
        adc_DR = config.getint('adc_DR', adc_DR_default, minval=0x00, maxval=0x07)
        adc_MODE = config.getint('adc_MODE', adc_MODE_default, minval=0x00, maxval=0x03)
        # 0x01 is Continuous conversion mode
        adc_CM = config.getint('adc_CM', 0x01, minval=0x00, maxval=0x01)
        adc_TS = config.getint('adc_TS', 0x00, minval=0x00, maxval=0x01)
        adc_BCS = config.getint('adc_BCS', 0x00, minval=0x00, maxval=0x01)
        reg_01_value = (adc_DR << 5) | (adc_MODE << 3) | (adc_CM << 2) | (adc_TS << 1) | adc_BCS
        self.regs.append(reg_01_value)
        # Configuration Register 2
        adc_VREF = config.getint('adc_VREF', 0x00, minval=0x00, maxval=0x03)
        adc_5060 = config.getint('adc_5060', 0x00, minval=0x00, maxval=0x03)
        adc_PSW = config.getint('adc_PSW', 0x00, minval=0x00, maxval=0x01)
        adc_IDAC = config.getint('adc_IDAC', 0x00, minval=0x00, maxval=0x07)
        reg_02_value = (adc_VREF << 6) | (adc_5060 << 4) | (adc_PSW << 3) | adc_IDAC
        self.regs.append(reg_02_value)
        # Configuration Register 3
        adc_I1MUX = config.getint('adc_I1MUX', 0x00, minval=0x00, maxval=0x07)
        adc_I2MUX = config.getint('adc_I2MUX', 0x00, minval=0x00, maxval=0x07)
        adc_DRDYM = config.getint('adc_DRDYM', 0x00, minval=0x00, maxval=0x01)
        reg_03_value = (adc_I1MUX << 5) | (adc_I2MUX << 2) | (adc_DRDYM << 1)
        self.regs.append(reg_03_value)
        # SPI Setup
        self.spi = bus.MCU_SPI_from_config(config, 1)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        # Data Ready (DRDY) Pin
        drdy_pin = config.get('data_ready_pin')
        ppins = printer.lookup_object('pins')
        drdy_ppin = ppins.lookup_pin(drdy_pin)
        self.data_ready_pin = drdy_ppin['pin']
        drdy_pin_mcu = drdy_ppin['chip']
        if drdy_pin_mcu != self.mcu:
            raise config.error("ADS1220 config error: SPI communication and"
                               " data_ready_pin must be on the same MCU")
        # Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        # Measurement conversion
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # publish raw samples to the socket
        self.batch_bulk.add_mux_endpoint("ads1220/dump_ads1220", "sensor",
                                         self.name,
                                         {'header': ('time', 'counts')})
        # Command Configuration
        self.config_endstop_cmd = None
        mcu.add_config_cmd(
            "config_ads1220 oid=%d spi_oid=%d data_ready_pin=%s"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin))
        mcu.add_config_cmd("query_ads1220 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.query_ads1220_cmd = None

    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.query_ads1220_cmd = self.mcu.lookup_command(
            "query_ads1220 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.config_endstop_cmd = self.mcu.lookup_command(
            "attach_endstop_ads1220 oid=%c load_cell_endstop_oid=%c")
        self.ffreader.setup_query_command("query_ads1220_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        return -0x800000, 0x7FFFFF

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_endstop(self, endstop_oid):
        self.config_endstop_cmd.send_wait_ack([self.oid, endstop_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Start bulk reading
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads1220_cmd.send([self.oid, rest_ticks])
        logging.info("ADS1220 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_ads1220_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS1220 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    def reset_chip(self):
        # the reset command takes 50us to complete
        self.send_command(RESET_CMD)
        # read startup register state and validate
        val = self.read_reg(0x0, 4)
        if hexify(val) != hexify(RESET_STATE):
            raise self.printer.command_error(
                "Invalid ads1220 reset state (got %s vs %s).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty adxl345 chip."
                % (hexify(val), hexify(RESET_STATE)))

    def setup_chip(self):
        for i, reg in enumerate(self.regs):
            if reg > 0x00:
                self.write_reg(i, [reg])
        # start measurements immediately
        self.send_command(START_SYNC_CMD)

    def read_reg(self, reg, byte_count):
        read_command = [RREG_CMD | (reg << 2) | (byte_count - 1)]
        read_command += [NOOP_CMD] * byte_count
        params = self.spi.spi_transfer(read_command)
        return bytearray(params['response'][1:])

    def send_command(self, cmd):
        self.spi.spi_send([cmd])

    def write_reg(self, reg, register_bytes):
        write_command = [WREG_CMD | (reg << 2) | (len(register_bytes) - 1)]
        write_command.extend(register_bytes)
        self.spi.spi_send(write_command)
        stored_val = self.read_reg(reg, len(register_bytes))
        if hexify(register_bytes) != hexify(stored_val):
            raise self.printer.command_error(
                "Failed to set ADS1220 register [0x%x] to %s: got %s. "
                "This may be a connection problem (e.g. faulty wiring)" % (
                    reg, hexify(register_bytes), hexify(stored_val)))


ADS1220_SENSOR_TYPE = {"ads1220": ADS1220}
