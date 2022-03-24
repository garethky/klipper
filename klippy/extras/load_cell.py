# ADS1262/1263 ADC Support
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, threading, logging
from probe import PrinterProbe
from adc_endstop import ADC_endstop
from . import bus, motion_report

# Things to go here:
# responsible for a config section that has:
# * Reference to what chip should be used, this supplies the chip name to use for selecting the read function in c
# * The chip must implement methods for start/stop capture
# * setup code for telling the MCU about the commands in C
# * adc_endstop configuration?


# Interface for Sensors that want to supply data to run a Load Cell
class LoadCellDataSource:
    def get_spi(self):
        # return the spi object for the sensor
        pass
    def get_capture_name(self):
        # return the name for looking up the capture routine in firmware
        pass
    def start_capture(self):
        # start sensor capture
        pass
    def stop_capture(self):
        # stop sensor capture
        pass
    def get_samples_per_second(self):
        # stop sensor capture
        pass

MIN_MSG_TIME = 0.100
TCODE_ERROR = 0xff
ERROR_OVERFLOW = 0
ERROR_SCHEDULE = 1
ERROR_SPI_TIME = 2
ERROR_CRC = 3
ERROR_DUPELICATE = 4
MAX_SAMPLES = 16

class LoadCellCaptureHelper:
    def __init__(self, printer, sensor, oid):
        self.printer = printer
        self.sensor = sensor
        self.oid = oid
        self.mcu = mcu = sensor.get_spi().get_mcu()
        self.lock = threading.Lock()
        self.last_sequence = 0
        # Capture message storage (accessed from background thread)
        self.capture_buffer = []
         # Measurement conversion
        self.start_clock = 0
        self.time_shift = 0
        self.sample_ticks = 0
        self.last_sequence = 0
        self.last_angle = 0
        self.last_chip_mcu_clock = 0
        self.last_chip_clock = 0
        self.oversample_rate = 1
        self.static_delay = 0
        freq = mcu.seconds_to_clock(1.)
        while (float(TCODE_ERROR << self.time_shift) / freq) < 0.002:
            self.time_shift += 1
        mcu.register_response(self._add_samples, "load_cell_data", oid)
        cmdqueue = self.sensor.get_spi().get_command_queue()
        query = "query_load_cell oid=%c clock=%u rest_ticks=%u time_shift=%c"
        query_end = "load_cell_end oid=%c sequence=%hu"
        self.query_load_cell_cmd = self.mcu.lookup_command(query, cq=cmdqueue)
        self.query_load_cell_end_cmd = self.mcu.lookup_query_command(
            query, query_end, oid=oid, cq=cmdqueue)
    def _add_samples(self, new_capture):
        with self.lock:
            self.capture_buffer.append(new_capture)
        logging.info("Samples received from MCU! %s", str(new_capture))
    def empty_buffer(self):
        with self.lock:
            last_batch = self.capture_buffer
            self.capture_buffer = []
        return last_batch
    def reset(self):
        self.empty_buffer()
        self.last_sequence = 0
        self.start_clock = 0
    def start_capture(self):
        self.reset()
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        self.start_clock = reqclock = self.mcu.print_time_to_clock(print_time)
        sample_period = 1. / (self.sensor.get_samples_per_second()
                        * self.oversample_rate)
        rest_ticks = self.mcu.seconds_to_clock(sample_period)
        self.sample_ticks = rest_ticks
        # DEBUG
        # logging.info("Sample Period: %f, Samples per second: %i, MCU frequency: %i", sample_period, self.sensor.get_samples_per_second(), self.mcu._mcu_freq)
        # logging.info("sending: query_load_cell oid=%i clock=%i rest_ticks=%i time_shift=%i", self.oid, reqclock, rest_ticks, self.time_shift)
        self.query_load_cell_cmd.send([self.oid, reqclock, rest_ticks,
                                       self.time_shift], reqclock=reqclock)
    def stop_capture(self):
        self.query_load_cell_end_cmd.send([self.oid, 0, 0, 0])
        self.reset()
    def is_measuring(self):
        return self.start_clock != 0
    def _extract_byte(self, b, i):
        return (b[i], i+1)
    def _extract_int32(self, b, i):
        return (b[i] | b[i+1] << 8 | b[i+2] << 16 | b[i+3] << 24, i+4)
    def _process_error(self, data, i, error_count):
        error, i = self._extract_byte(data, i, error_count)
        # this implementation support over-sampling
        if error != ERROR_DUPELICATE:
            error_count += 1
        return error_count, i
    def flush(self):
        raw_samples = self.empty_buffer()
        # Load variables to optimize inner loop below
        static_delay = self.static_delay
        sample_ticks = self.sample_ticks
        start_clock = self.start_clock
        time_shift = self.time_shift
        clock_to_print_time = self.mcu.clock_to_print_time
        # this isnt thread protected....
        last_sequence = self.last_sequence
        # Process every message in raw_samples
        i = count = error_count = 0
        samples = [None] * (len(raw_samples) * MAX_SAMPLES)
        for params in raw_samples:
            seq = (last_sequence & ~0xffff) | params['sequence']
            if seq < last_sequence:
                seq += 0x10000  # WUT?? just like, lets add some shit?
            last_sequence = seq
            msg_mclock = start_clock + seq*16*sample_ticks
            d = bytearray(params['data'])
            data_len = len(d)
            while i < data_len:
                tcode = d[i]
                timestamp, i = self._extract_byte(d, i)
                if tcode == TCODE_ERROR:
                    error_count, i = self._process_error(d, i, error_count)
                else:
                    sample, i = self._extract_int32(d, i)
                    mclock = msg_mclock + (i * sample_ticks)
                    # timestamp is mcu clock offset shifted by time_shift
                    sclock = mclock + (timestamp << time_shift)
                    ptime = round(clock_to_print_time(sclock) - static_delay, 6)
                    samples[count] = (ptime, sample)
                    count += 1
        self.last_sequence = last_sequence
        del samples[count:]
        return samples, error_count

class LoadCellCommandHelper:
    def __init__(self, config, load_cell):
        self.printer = config.get_printer()
        self.load_cell = load_cell
        self.bg_client = None
        self.name = config.get_name().split()[-1]
        self.register_commands(self.name)
        if self.name == "load_cell":
            self.register_commands(None)
    def register_commands(self, name):
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("LOAD_CELL_CAPTURE", "LOAD_CELL", name,
                                   self.cmd_LOAD_CELL_CAPTURE,
                                   desc=self.cmd_LOAD_CELL_CAPTURE_help)
    cmd_LOAD_CELL_CAPTURE_help = "Start/stop Load Cell capture"
    def cmd_LOAD_CELL_CAPTURE(self, gcmd):
        bg_client = self.bg_client
        if bg_client is None:
            # Start measurements
            self.bg_client = self.load_cell.start_internal_client()
            gcmd.respond_info("Load Cell capture started")
            return
        # End measurements
        self.bg_client = None
        bg_client.finalize()  # is that all?
        gcmd.respond_info("Load Cell capture stopped")

# Printer class that controls ADS1263 chip
class LoadCell:
    def __init__(self, config, sensor, adc_endstop):
        self.printer = config.get_printer()
        # sensor must implement LoadCellDataSource
        self.sensor = sensor
        LoadCellCommandHelper(config, self)
        self.adc_endstop = adc_endstop
        self.adc_endstop_oid = 0
        if adc_endstop is not None:
            self.adc_endstop_oid = adc_endstop.get_oid()
        # Setup mcu sensor_load_cell bulk query code
        self.mcu = mcu = self.sensor.get_spi().get_mcu()
        self.oid = oid = mcu.create_oid()
        spi_oid = self.sensor.get_spi().get_oid()
        self.samples = None;
        mcu.add_config_cmd("config_load_cell oid=%d spi_oid=%d \
                            load_cell_sensor_type=%s adc_endstop_oid=%d"
            % (oid, spi_oid, self.sensor.get_capture_name()
            , self.adc_endstop_oid))
        mcu.add_config_cmd(
            "query_load_cell oid=%d clock=0 rest_ticks=0 time_shift=0"
            % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        # API server endpoints
        self.api_dump = motion_report.APIDumpHelper(
            self.printer, self._api_update, self._api_startstop, 0.100)
        self.name = config.get_name()
        wh = self.printer.lookup_object('webhooks')
        wh.register_mux_endpoint("load_cell/dump", "sensor", self.name,
                                 self._handle_dump_load_cell)
    def _build_config(self):
        self.samples = LoadCellCaptureHelper(self.printer, self.sensor, self.oid)
    def _start_capture(self):
        if self.samples.is_measuring():
            return
        logging.info("Starting load cell '%s' capture", self.name)
        self.sensor.start_capture()
        self.samples.start_capture()
        logging.info("Stared load cell '%s' capture", self.name)
    def _stop_capture(self):
        if not self.samples.is_measuring():
            return
        logging.info("Stopping load cell '%s' capture", self.name)
        self.sensor.stop_capture()
        self.samples.stop_capture()
        logging.info("Stopped load cell '%s' capture", self.name)
    def _api_startstop(self, is_start):
        if is_start:
            self._start_capture()
        else:
            self._stop_capture()
    def _api_update(self, eventtime):
        samples, errors = self.samples.flush()
        samples_str = ', '.join(str(s) for s in samples)
        # DEBUG
        #logging.info("Samples: %i, %s Errors: %i", len(samples), samples_str, errors)
        return {'data': samples, 'errors': errors}
    def _handle_dump_load_cell(self, web_request):
        self.api_dump.add_client(web_request)
        hdr = ('time', 'volts')
        web_request.send({'header': hdr})
    def start_internal_client(self):
        cconn = self.api_dump.add_internal_client()
        return cconn

def load_config(config):
    printer = config.get_printer()
    sensor = printer.lookup_object(config.get('sensor'))
    if config.getboolean('is_probe', default=False):
        endstop = ADC_endstop(config, sensor)
        config.get_printer().add_object('probe'
                , PrinterProbe(config, endstop))
    return LoadCell(config, sensor, endstop)

def load_config_prefix(config):
    #TODO: re-implement name support
    return LoadCell(config)
