# ADS1262/1263 ADC Support
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from . import bus, multiplex_adc

# ADS 1263 Commands
CMD_NOP     = 0x00 # The No Op command
CMD_RESET   = 0x06 # Reset the ADC
CMD_START1  = 0x08 # Start ADC1 conversions
CMD_STOP1   = 0x0A # Stop ADC1 conversions
CMD_SYOCAL1 = 0x16 # ADC1 system offset calibration
CMD_SYGCAL1 = 0x17 # ADC1 system gain calibration
CMD_SFOCAL1 = 0x19 # ADC1 self offset calibration
CMD_RREG    = 0x20 # Read registers
CMD_WREG    = 0x40 # Write registers


RESET_DELAY_MS     = 10

REVISIONS_TABLE = []
REVISIONS_TABLE.extend(range(0, 2^5))

CHECKSUM_MODE = ["Disabled", "Enabled, Checksum mode (default)",
"Enabled, CRC mode", "Reserved"]

DATA_RATE_TABLE = ["2.5 SPS", "5 SPS", "10 SPS", "16.6SPS", "20 SPS (default)",
"50 SPS", "60 SPS", "100 SPS", "400 SPS", "1200 SPS", "2400 SPS", "4800 SPS",
"7200 SPS", "14400 SPS", "19200 SPS", "38400 SPS"]
SAMPLES_PER_S = [2.5, 5, 10, 16.6, 20, 50, 60, 100, 400, 1200, 2400, 4800, 7200,
14400, 19200, 38400]

PGA_GAIN_TABLE = ["1 V/V (default)", "2 V/V", "4 V/V", "8 V/V", "16 V/V",
"32 V/V", "Reserved", "Reserved", "Reserved"]

DIGITAL_FILTER_MODE_TABLE = ["Sinc1 mode", "Sinc2 mode", "Sinc3 mode",
"Sinc4 mode", "FIR mode (default)", "Reserved", "Reserved", "Reserved"]

SENSOR_BIAS_MAGNITUDE_TABLE = ["No sensor bias current or resistor (default)",
"0.5-\u00b5A sensor bias current", "2-\u00b5A sensor bias current",
"10-\u00b5A sensor bias current", "50-\u00b5A sensor bias current",
"200-\u00b5A sensor bias current", "10-M\u03a9 resistor", "Reserved"]

CONVERSION_DELAY_TABLE = ["no delay (default)", "8.7 \u00b5s", "17 \u00b5s",
"35 \u00b5s", "69 \u00b5s", "139 \u00b5s", "278 \u00b5s", "555 \u00b5s",
"1.1 ms", "2.2 ms", "4.4 ms", "8.8 ms"]
CONVERSION_DELAY_TABLE.extend(["Reserved"] * 4)
CONVERSION_DELAY_TIME_MS = [0, 1, 1, 1, 1, 1, 1, 1, 2, 3, 5, 9]

POS_INPUT_MUX_TABLE = ["AIN0 (default)", "AIN1", "AIN2", "AIN3", "AIN4", "AIN5",
"AIN6", "AIN7", "AIN8", "AIN9", "AINCOM", "Temperature sensor monitor positive",
"Analog power supply monitor positive", "Digital power supply monitor positive",
"TDAC test signal positive", "Float (open connection)", ]

NEG_INPUT_MUX_TABLE = ["AIN0", "AIN1 (default)", "AIN2", "AIN3", "AIN4", "AIN5",
"AIN6", "AIN7", "AIN8", "AIN9", "AINCOM", "Temperature sensor monitor negative",
"Analog power supply monitor negative", "Digital power supply monitor negative",
"TDAC test signal negative", "Float (open connection)"]

POSITIVE_VOLTAGE_REF_TABLE = ["Internal 2.5 V reference - P (default)",
"External AIN0", "External AIN2", "External AIN4",
"Internal analog supply (VAVDD )", "Reserved", "Reserved", "Reserved"]

NEGAITIVE_VOLTAGE_REF_TABLE = ["Internal 2.5 V reference - N (default)",
"External AIN1", "External AIN3", "External AIN5",
"Internal analog supply (VAVSS)", "Reserved", "Reserved", "Reserved"]

CHOP_MODE_TABLE = ["Disabled (default)", "Input chop enabled",
"IDAC rotation enabled", "Input chop and IDAC rotation enabled"]

REF_MUX_POSITIVE_TABLE = ["Internal 2.5 V reference - P (default)",
"External AIN0", "External AIN2", "External AIN4",
"Internal analog supply (VAVDD)"]

REF_MUX_NEGATIVE_TABLE = ["Internal 2.5 V reference - N (default)",
"External AIN1", "External AIN3", "External AIN5",
"Internal analog supply (VAVSS)"]

YES_NO_TABLE = ["No", "Yes"]

INPUT_SELECTION_TABLE = [
    (0 << 4) | 1, # AIN0-AIN1
    (2 << 4) | 3, # AIN2-AIN3
    (4 << 4) | 5, # AIN4-AIN5
    (6 << 4) | 7, # AIN6-AIN7
    (8 << 4) | 9] # AIN8-AIN9

# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(bytes):
    return "[%s]" % (",".join([hex(b) for b in bytes]))

# a field stored in a range of bits inside a byte
class Field:
    def __init__(self, name, width, rmb_index, value_table):
        self.name = name
        self.width = width
        self.rmb_index = rmb_index
        self.value_table = value_table
    def get_value(self, reg_value):
        bitmask = (0xff >> (8 - self.width)) << self.rmb_index
        return (reg_value & bitmask) >> self.rmb_index
    def set_value(self, reg_value, field_value):
        if field_value is None: return reg_value
        mask = (0xff >> (8 - self.width)) << self.rmb_index
        return (reg_value & ~mask) | ((field_value << self.rmb_index) & mask);
    def get_value_string(self, reg_value):
        return self.value_table[self.get_value(reg_value)]
    def to_string(self, value):
        return "\---- %s: %s" % (self.name, self.get_value_string(value))

class Register:
    def __init__(self, name, address, byte_width, fields=[],
                little_endian=True):
        self.name = name
        self.addr = address
        self.byte_width = byte_width
        self.fields = fields
        self.little_endian = little_endian
    def slice(self, bytes):
        return bytes[:self.byte_width]
    def to_string(self, bytes):
        value = self.slice(bytes)
        out = "%s %s:%s" % (self.name, hexify([self.addr]), hexify(value))
        if len(self.fields):
            for field in self.fields:
                out = "%s\n%s" % (out, field.to_string(value[0]))
        else:
            out = "%s = %i" % (out, + self.to_int(value))
        return out
    def to_int(self, byte_buf):
        num = 0x0
        length = len(byte_buf)
        for i in range(0, length):
            offset = i if self.little_endian else (length - i -1)
            num |= (byte_buf[i] << (8 * offset))
        return num
    def read(self, chip):
        return chip.read_reg(self.addr, register_count=self.byte_width)
    def write(self, chip, bytes):
        return chip.write_reg(self.addr, bytes)
    def write_int(self, chip, int_val):
        bytes = []
        for i in range(0, self.byte_width):
            mask = 0xff << (i * 8)
            next_byte = int_val & mask
            bytes.append(next_byte >> (i * 8))
        if self.little_endian == False: bytes.reverse()
        return self.write(chip, bytes)

# Chip Identification
FIELD_ID_MODEL = Field("Model", 3, 5, ["ADS1262", "ADS1263"])
FIELD_ID_REVISION = Field("Revision", 5, 0, REVISIONS_TABLE)
REG_ID = Register("DeviceId", 0x00, 1, [FIELD_ID_MODEL, FIELD_ID_REVISION])

# Power settings
FIELD_POWER_RESET = Field("Reset Indicator", 1, 4, YES_NO_TABLE)
FIELD_POWER_SHIFT = Field("Level Shift Voltage Enabled", 1, 1, YES_NO_TABLE)
FIELD_POWER_INTERNAL_REF = Field("Internal Reference Enabled", 1, 0,
    YES_NO_TABLE)
REG_POWER = Register("Power", 0x01, 1, [FIELD_POWER_RESET, FIELD_POWER_SHIFT,
    FIELD_POWER_INTERNAL_REF])

# Serial interface & readback settings
FIELD_INTERFACE_AUTO_TIMEOUT = Field("Interface Timeout Enabled", 1, 3,
    YES_NO_TABLE)
FIELD_INTERFACE_STATUS = Field("Include Staus Byte", 1, 2, YES_NO_TABLE)
FIELD_INTERFACE_CHECKSUM = Field("Checksum Mode", 2, 0, CHECKSUM_MODE)
REG_INTERFACE = Register("Interface", 0x02, 1, [FIELD_INTERFACE_AUTO_TIMEOUT,
FIELD_INTERFACE_STATUS, FIELD_INTERFACE_CHECKSUM])

# Mode 0
FIELD_MODE0_REF_POLARITY = Field("ADC1 Reference Mux Polarity", 1, 7,
    ["Normal", "Reversed"])
FIELD_MODE0_RUN_MODE = Field("Run Mode", 1, 6,
    ["Continuous (default)", "One-shot"])
FIELD_MODE0_CHOP_MODE = Field("Chop Mode", 2, 5, CHOP_MODE_TABLE)
FIELD_MODE0_DELAY = Field("Delay", 4, 0, CONVERSION_DELAY_TABLE)
REG_MODE0 = Register("Mode 0", 0x03, 1, [FIELD_MODE0_REF_POLARITY,
    FIELD_MODE0_RUN_MODE, FIELD_MODE0_CHOP_MODE, FIELD_MODE0_DELAY])

# Mode 1
FIELD_MODE1_FILTER = Field("Filter", 3, 5, DIGITAL_FILTER_MODE_TABLE)
FIELD_MODE1_BIAS = Field("Sensor Bias ADC Connection", 1, 4,
    ["ADC1 (default)", "ADC2"])
FIELD_MODE1_POLARITY = Field("Sensor Bias Polarity", 1, 3,
    ["Pull up (default)", "Pull down"])
FIELD_MODE1_MAGNITUDE = Field("Sensor Bias Magnitued", 3, 0,
    SENSOR_BIAS_MAGNITUDE_TABLE)
REG_MODE1 = Register("Mode 1", 0x04, 1, [FIELD_MODE1_FILTER, FIELD_MODE1_BIAS,
    FIELD_MODE1_POLARITY, FIELD_MODE1_MAGNITUDE])

# Mode 2
FIELD_MODE2_PGA_ENABLE = Field("PGA", 1, 7, ["Enabled (default)", "Bypassed"])
FIELD_MODE2_GAIN = Field("PGA Gain", 3, 4, PGA_GAIN_TABLE)
FIELD_MODE2_DATA_RATE = Field("Data Rate", 4, 0, DATA_RATE_TABLE)
REG_MODE2 = Register("Mode 2", 0x05, 1, [FIELD_MODE2_PGA_ENABLE,
    FIELD_MODE2_GAIN, FIELD_MODE2_DATA_RATE])

# Input Multiplexer
FIELD_INPUT_MUX_POS = Field("Positive", 4, 4, POS_INPUT_MUX_TABLE)
FIELD_INPUT_MUX_NEG = Field("Negative", 4, 0, NEG_INPUT_MUX_TABLE)
REG_INPUT_MUX = Register("Inputs", 0x06, 1, [FIELD_INPUT_MUX_POS,
    FIELD_INPUT_MUX_NEG])

# Calibration registers
REG_OFFSET_CAL = Register("Offset Cal", 0x07, 3, little_endian = True)
REG_FULL_SCALE_CAL = Register("Full Scale Cal", 0x0A, 3, little_endian = True)

# Voltage Reference Multiplexer
FIELD_REF_MUX_POS = Field("Positive", 3, 3, REF_MUX_POSITIVE_TABLE)
FIELD_REF_MUX_NEG = Field("Negative", 3, 0, REF_MUX_NEGATIVE_TABLE)
REG_REF_MUX = Register("Reference Voltage", 0x0F, 1, [FIELD_REF_MUX_POS,
    FIELD_REF_MUX_NEG])

# All registers that are mapped:
ALL_REGISTERS = [REG_ID, REG_POWER, REG_INTERFACE, REG_MODE0, REG_MODE1,
    REG_MODE2, REG_INPUT_MUX, REG_OFFSET_CAL, REG_FULL_SCALE_CAL, REG_REF_MUX]

REG_MIN_ADDRESS = 0
REG_MAX_ADDRESS = 26

# Helper class for G-Code commands
class ADS1263CommandHelper:
    def __init__(self, config, chip):
        self.printer = config.get_printer()
        self.chip = chip
        self.name = config.get_name().split()[-1]
        self.register_commands(self.name)
        if self.name == "ads1263":
            self.register_commands(None)
    def register_commands(self, name):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("RESET_ADS1263", "CHIP", name,
            self.cmd_RESET_ADS1263, desc=self.cmd_RESET_ADS1263_help)
        gcode.register_mux_command("DUMP_ADS1263", "CHIP", name,
            self.cmd_DUMP_ADS1263, desc=self.cmd_DUMP_ADS1263_help)
        gcode.register_mux_command("CONFIGURE_ADS1263", "CHIP", name,
            self.cmd_CONFIGURE_ADS1263,desc=self.cmd_CONFIGURE_ADS1263_help)
        gcode.register_mux_command("OFFSET_CALIBRATION_ADS1263", "CHIP", name,
            self.cmd_OFFSET_CALIBRATION_ADS1263,
            desc=self.cmd_OFFSET_CALIBRATION_ADS1263_help)
        gcode.register_mux_command("FULL_SCALE_CALIBRATION_ADS1263", "CHIP",
            name, self.cmd_FULL_SCALE_CALIBRATION_ADS1263,
            desc=self.cmd_FULL_SCALE_CALIBRATION_ADS1263_help)
        gcode.register_mux_command("START_CAPTURE_ADS1263", "CHIP", name,
            self.cmd_START_CAPTURE_ADS1263,
            desc=self.cmd_START_CAPTURE_ADS1263_help)
        gcode.register_mux_command("STOP_CAPTURE_ADS1263", "CHIP", name,
            self.cmd_STOP_CAPTURE_ADS1263,
            desc=self.cmd_STOP_CAPTURE_ADS1263_help)
        gcode.register_mux_command("DEBUG_READ_ADS1263", "CHIP", name,
            self.cmd_DEBUG_READ_ADS1263, desc=self.cmd_DEBUG_READ_ADS1263_help)
        gcode.register_mux_command("DEBUG_WRITE_ADS1263", "CHIP", name,
            self.cmd_DEBUG_WRITE_ADS1263,
            desc=self.cmd_DEBUG_WRITE_ADS1263_help)
    def _not_if_capturing(self, gcmd):
        if (not self.chip.is_capturing):
            return False
        raise self.printer.command_error("Opperation not allowed while sensor \
            is capturing data!")
    cmd_DUMP_ADS1263_help = "Print all ADS1263 registers with descriptions"
    def cmd_DUMP_ADS1263(self, gcmd):
        val = self.chip.read_reg(REG_MIN_ADDRESS, REG_MAX_ADDRESS - 1)
        out = [reg.to_string(val[reg.addr:]) for reg in ALL_REGISTERS]
        gcmd.respond_info("\n".join(out))
    cmd_RESET_ADS1263_help = "Send the RESET command"
    def cmd_RESET_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        gcmd.respond_info(self.chip.reset())
    cmd_CONFIGURE_ADS1263_help = "Configure settings of the ADS1263"
    def cmd_CONFIGURE_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        input = gcmd.get_int("INPUT", default=None, minval=0, maxval=4)
        rate = gcmd.get_int("RATE", default=None, minval=0, maxval=15)
        pga_enable = gcmd.get_int("PGA", default=None, minval=0, maxval=1)
        gain = gcmd.get_int("GAIN", default=None, minval=0, maxval=5)
        filter = gcmd.get_int("FILTER", default=None, minval=0, maxval=4)
        chop = gcmd.get_int("CHOP", default=None, minval=0, maxval=1)
        vref = gcmd.get_int("VREF", default=None, minval=0, maxval=4)
        offset = gcmd.get_int("OFFSET", minval=0, maxval=16777215, default=None)
        range = gcmd.get_int("RANGE", minval=0, maxval=16777215, default=None)
        results = self.chip.configure(input, rate, pga_enable, gain, filter,
            chop, vref, offset, range)
        if results:
            gcmd.respond_info(results)
    cmd_OFFSET_CALIBRATION_ADS1263_help = "Perform Offset Calibration"
    def cmd_OFFSET_CALIBRATION_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        self.chip.calibrate(CMD_SFOCAL1)
        val = REG_OFFSET_CAL.read(self.chip)
        gcmd.respond_info(REG_OFFSET_CAL.to_string(val))
    cmd_FULL_SCALE_CALIBRATION_ADS1263_help = "Perform Full Scale Calibration"
    def cmd_FULL_SCALE_CALIBRATION_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        self.chip.calibrate(CMD_SYGCAL1)
        val = REG_FULL_SCALE_CAL.read(self.chip)
        gcmd.respond_info(REG_FULL_SCALE_CAL.to_string(val))
    cmd_START_CAPTURE_ADS1263_help = "Start capturing samples"
    def cmd_START_CAPTURE_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        self.chip.start_capture()
    cmd_STOP_CAPTURE_ADS1263_help = "Stop capturing samples"
    def cmd_STOP_CAPTURE_ADS1263(self, gcmd):
        if self._not_if_capturing(gcmd):
            return
        self.chip.stop_capture()
    cmd_DEBUG_READ_ADS1263_help = "Query ADS1263 registers (for debugging)"
    def cmd_DEBUG_READ_ADS1263(self, gcmd):
        reg = gcmd.get_int("REG", minval=REG_MIN_ADDRESS,
                            maxval=REG_MAX_ADDRESS)
        count = gcmd.get_int("COUNT", minval=1, maxval=REG_MAX_ADDRESS - 1,
                            default=1)
        val = self.chip.read_reg(reg, count)
        gcmd.respond_info("ADS1263 REG[0x%x] = [%s]" % (reg, hexify(val)))
    cmd_DEBUG_WRITE_ADS1263_help = "Set a single ADS1263 register \
                                    (for debugging)"
    def cmd_DEBUG_WRITE_ADS1263(self, gcmd):
        if self._not_if_capturing():
            return
        reg = gcmd.get_int("REG", minval=0, maxval=26)
        val = gcmd.get_int("VAL", minval=0, maxval=255)
        self.chip.write_reg(reg, bytearray(val))

# Printer class that controls ADS1263 chip
class ADS1263(multiplex_adc.MultiplexAdcSensor):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        # Setup SPI communications on the MCU hosting the sensor
        self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=25000000)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = mcu.create_oid()
        self.is_capturing = False
        self.gain = config.getint('gain', minval=0, maxval=5, default=0)
        # 8 = 400 SPS
        self.samples_per_second = config.getint('sample_rate', minval=0
                                                     , maxval=15, default=8)
        #TODO: read the rest of the properties from the config
        ADS1263CommandHelper(config, self)
        mcu.register_config_callback(self._build_config)
        printer.register_event_handler("klippy:firmware_restart", self._connect)
        printer.register_event_handler("klippy:connect", self._connect)
    def _build_config(self):
        self.mcu.add_config_cmd("config_ads1263 oid=%d spi_oid=%d"
            % (self.oid, self.spi.get_oid()))
    def _connect(self):
        self.reset()
        self.configure(gain = self.gain, rate=self.samples_per_second)
    # LoadCellDataSource methods
    def get_oid(self):
        return self.oid
    def get_mcu(self):
        return self.mcu
    def get_mux_adc_sensor_type(self):
        return "ads1263"
    def get_samples_per_second(self):
        return SAMPLES_PER_S[self.samples_per_second]
    def is_capturing(self):
        return self.is_capturing
    def start_capture(self):
        if (self.is_capturing):
            return
        self.send_command(CMD_START1)
        self.is_capturing = True
    def stop_capture(self):
        if (not self.is_capturing):
            return
        self.send_command(CMD_STOP1)
        self.is_capturing = False
    def _wait(self, milliseconds = 10):
        systime = self.reactor.monotonic()
        print_time = self.mcu.estimated_print_time(systime)
        self.reactor.pause(print_time + (milliseconds / 1000))
    def reset(self):
        self.stop_capture()
        self.send_command(CMD_RESET)
        self._wait(RESET_DELAY_MS)
        #clear the reset register
        reg_val = REG_POWER.read(self)[0]
        reg_val = FIELD_POWER_RESET.set_value(reg_val, 0)
        return REG_POWER.to_string(REG_POWER.write(self, reg_val))
    def _calibration_delay_ms(self, data_rate_index, delay_index):
        delay_ms = CONVERSION_DELAY_TIME_MS[delay_index]
        time_per_sample = math.ceil(1000 / SAMPLES_PER_S[data_rate_index])
        return delay_ms + (time_per_sample * 16)
    def calibrate(self, calibration_cmd):
        self.stop_capture()
        #TODO: configure inputs as required by the calibration being performed
        self.start_capture()
        self.send_command(calibration_cmd)
        self._wait(self._calibration_delay_ms(1, 1))
        self.stop_capture()
    def set_input(self, input):
        if input is None: return
        return REG_INPUT_MUX.to_string(REG_INPUT_MUX.write(self,
            INPUT_SELECTION_TABLE[input]))
    def set_mode0(self, chop):
        if chop is None: return
        reg_val = REG_MODE0.read(self)[0]
        reg_val = FIELD_MODE0_CHOP_MODE.set_value(reg_val, chop)
        return REG_MODE0.to_string(REG_MODE0.write(self, reg_val))
    def set_mode1(self, filter):
        if filter is None: return
        val = REG_MODE1.read(self)[0]
        val = FIELD_MODE1_FILTER.set_value(val, filter)
        return REG_MODE1.to_string(REG_MODE1.write(self, val))
    def set_mode2(self, rate, pga_enable, gain):
        if all(arg is None for arg in [rate, pga_enable, gain]): return
        currentSetting = REG_MODE2.read(self)[0]
        val = currentSetting
        if rate is not None:
            self.samples_per_second = rate
            val = FIELD_MODE2_DATA_RATE.set_value(val, rate)
        if pga_enable is not None:
            val = FIELD_MODE2_PGA_ENABLE.set_value(val, pga_enable)
        if gain is not None:
            self.gain = gain
            val = FIELD_MODE2_GAIN.set_value(val, gain)
        return REG_MODE2.to_string(REG_MODE2.write(self, val))
    def set_ref_mux(self, vref):
        if vref is None: return
        return REG_REF_MUX.to_string(REG_REF_MUX.write(self, (vref << 3) | vref))
    def set_offset_cal(self, offset):
        if offset is None: return
        return REG_OFFSET_CAL.to_string(REG_OFFSET_CAL.write_int(self, offset))
    def set_range_cal(self, range):
        if range is None: return
        return REG_FULL_SCALE_CAL.to_string(
            REG_FULL_SCALE_CAL.write_int(self, range))
    def configure(self, input=None, rate=None, pga_enable=None, gain=None
                  , filter_val=None, chop=None, vref=None, offset=None
                  , range=None):
        self.stop_capture()
        results = [self.set_input(input), self.set_mode0(chop),
            self.set_mode1(filter_val), self.set_mode2(rate, pga_enable, gain),
            self.set_ref_mux(vref), self.set_offset_cal(offset),
            self.set_range_cal(range)]
        results = list(filter(None, results))
        if len(results):
            return "\n".join(results)
        return None
    def send_command(self, command):
        self.spi.spi_send([command])
    def read_reg(self, reg, register_count=1):
        register_count = min(register_count, (REG_MAX_ADDRESS + 1) - reg)
        read_command = [CMD_RREG | reg, register_count - 1]
        read_command += [CMD_NOP] * register_count
        params = self.spi.spi_transfer(read_command)
        return bytearray(params['response'][2:])
    def write_reg(self, reg, val):
        #TODO: this is a mess, simplify to just byte arrays!
        if not isinstance(val, list):
            val = [val]
        to_write = [CMD_WREG | reg, len(val) - 1]
        to_write.extend(val)
        self.spi.spi_send(to_write)
        stored_val = self.read_reg(reg, len(val))
        stored_hex = hexify(stored_val)
        val_hex = hexify(val)
        if val_hex != stored_hex:
            raise self.printer.command_error(
                    "Failed to set ADS1263 register [0x%x] to %s: got %s. "
                    "This is generally indicative of connection problems "
                    "(e.g. faulty wiring) or a faulty ads1263 chip." % (
                        reg, val_hex, stored_hex))
        return stored_val
