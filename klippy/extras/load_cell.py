# Load Cell Implementation
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading, logging, struct
from probe import PrinterProbe
from load_cell_endstop import LoadCellEndstop

# TODO: move to some sort of logging utility
# turn bytearrays into pretty hex strings: [0xff, 0x1]
def hexify(bytes):
    return "[%s]" % (",".join([hex(b) for b in bytes]))

# Things to go here:
# responsible for a config section that has:
# * Reference to what chip should be used, this supplies the chip name to use for selecting the read function in c
# * The chip must implement methods for start/stop capture
# * setup code for telling the MCU about the commands in C
# * load_cell_endstop configuration?

def average(data, points=None):
        points = len(data) if points is None else points
        avg = 0
        for index in range(points):
            avg = avg + int((data[index] - avg) / (index + 1))
        return avg

# Interface for Sensors that want to supply data to run a Load Cell
class LoadCellSensor:
    # return the spi object for the sensor
    def get_spi(self):
        pass
    # return the name for looking up the capture routine in firmware
    def get_capture_name(self):
        pass
    # start sensor capture
    def start_capture(self):
        pass
    # stop sensor capture
    def stop_capture(self):
        pass
    # get the number of samples per second
    def get_samples_per_second(self):
        pass
    # convert a raw sample to Volts
    def sample_to_volts(self):
        pass

# Interface for subscribers of load cell data
class LoadCellSubscriber:
    # report the status of the load cell, is it capturing data and at what SPS
    def status(self, is_capturing, sps):
        pass
    # report new samples from the load cell
    def on_capture(self, samples):
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
    def __init__(self, printer, load_cell):
        self.printer = printer
        self.load_cell = load_cell
        self.oid = oid = load_cell.oid
        self.sensor = load_cell.sensor
        self.mcu = mcu = self.sensor.get_spi().get_mcu()
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
        self.oversample_rate = 1.
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
        # DEBUG
        #d = bytearray(new_capture['data'])
        #logging.info("Load Cell Samples: %s (%i)", hexify(d), len(d))
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
        # bits = b[i] | b[i+1] << 8 | b[i+2] << 16 | b[i+3] << 24
        value = struct.unpack_from('<i', b, offset=i)[0]
        # out = 0.0
        # TODO: do we want to return Volts or raw Counts??
        # TODO: this conversion should be happening only in the ADS1263 code, it cant be safe to do it here as its sensor specific
        # TODO: Where do we get the reference voltage??
        #REF = 5.00
        #if (value >> 31 == 1):
        #    out = (REF * 2 - value * REF / 0x80000000)
        #else:
        #    out = (value * REF / 0x7fffffff)
        return (value, i + 4)
    def _process_error(self, data, i, error_count, duplicate_count):
        error, i = self._extract_byte(data, i)
        # this implementation supports over-sampling
        if error == ERROR_DUPELICATE:
            duplicate_count += 1
        else:
            error_count += 1
        return error_count, duplicate_count, i
    def flush(self):
        raw_samples = self.empty_buffer()
        # Load variables to optimize inner loop below
        static_delay = self.static_delay
        sample_ticks = self.sample_ticks
        start_clock = self.start_clock
        time_shift = self.time_shift
        clock_to_print_time = self.mcu.clock_to_print_time
        # this isn't thread protected....
        last_sequence = self.last_sequence
        # Process every message in raw_samples
        count = error_count = duplicate_count = 0
        samples = [None] * (len(raw_samples) * MAX_SAMPLES)
        for params in raw_samples:
            seq = (last_sequence & ~0xffff) | params['sequence']
            if seq < last_sequence:
                seq += 0x10000
            last_sequence = seq
            msg_mclock = start_clock + (seq * 16 * sample_ticks)
            d = bytearray(params['data'])
            data_len = len(d)
            i = 0
            try:
                while i < data_len:
                    tcode, i = self._extract_byte(d, i)
                    if tcode == TCODE_ERROR:
                        error_count, duplicate_count, i = self._process_error(d, i, error_count, duplicate_count)
                    else:
                        sample, i = self._extract_int32(d, i)
                        mclock = msg_mclock + (i * sample_ticks)
                        # timestamp is mcu clock offset shifted by time_shift
                        sclock = mclock + (tcode << time_shift)
                        ptime = round(clock_to_print_time(sclock) - static_delay, 6)
                        grams = self.load_cell.sample_to_grams(sample)
                        samples[count] = (ptime, sample, grams)
                        count += 1
            except Exception:
                logging.info("Load Cell Samples threw an error: %s (%i) i: %i, e: %i, d: %i", hexify(d), data_len, i, error_count, duplicate_count)
        self.last_sequence = last_sequence
        # trim empty entries from the end of the list
        del samples[count:]
        logging.info("GOT SAMPLES: %i, %i, ERRORS: %i, duplicates: %i" % (count, len(samples), error_count, duplicate_count))
        return samples, error_count, duplicate_count

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
        gcode.register_mux_command("START_LOAD_CELL_CAPTURE", "LOAD_CELL", name,
                                   self.cmd_START_LOAD_CELL_CAPTURE,
                                   desc=self.cmd_START_LOAD_CELL_CAPTURE_help)
        gcode.register_mux_command("STOP_LOAD_CELL_CAPTURE", "LOAD_CELL", name,
                                   self.cmd_STOP_LOAD_CELL_CAPTURE,
                                   desc=self.cmd_STOP_LOAD_CELL_CAPTURE_help)
        gcode.register_mux_command("CONFIG_LOAD_CELL_CAPTURE", "LOAD_CELL",
                                   name,
                                   self.cmd_CONFIG_LOAD_CELL_CAPTURE,
                                   desc=self.cmd_CONFIG_LOAD_CELL_CAPTURE_help)
        gcode.register_mux_command("TARE_LOAD_CELL", "LOAD_CELL",
                                   name,
                                   self.cmd_TARE_LOAD_CELL,
                                   desc=self.cmd_TARE_LOAD_CELL_help)
        gcode.register_mux_command("CALIBRATE_LOAD_CELL", "LOAD_CELL",
                                   name,
                                   self.cmd_CALIBRATE_LOAD_CELL,
                                   desc=self.cmd_CALIBRATE_LOAD_CELL_help)
        gcode.register_mux_command("READ_LOAD_CELL", "LOAD_CELL",
                                   name,
                                   self.cmd_READ_LOAD_CELL,
                                   desc=self.cmd_READ_LOAD_CELL_help)
    cmd_START_LOAD_CELL_CAPTURE_help = "Start Load Cell capture"
    def cmd_START_LOAD_CELL_CAPTURE(self, gcmd):
        if self.load_cell.start_capture():
            gcmd.respond_info("Load Cell capture started")
        else:
            raise self.printer.command_error("Load Cell is capturing")
    cmd_STOP_LOAD_CELL_CAPTURE_help = "Start/stop Load Cell capture"
    def cmd_STOP_LOAD_CELL_CAPTURE(self, gcmd):
        if self.load_cell.stop_capture():
            gcmd.respond_info("Load Cell capture stopped")
        else:
            raise self.printer.command_error("Load Cell capture not started")
    cmd_CONFIG_LOAD_CELL_CAPTURE_help = "Change capture settings on the fly"
    def cmd_CONFIG_LOAD_CELL_CAPTURE(self, gcmd):
        report_rate = gcmd.get_int("REPORT_RATE", default=1, minval=1
                                    , maxval=65535)
        self.load_cell.set_report_rate(report_rate)
    cmd_TARE_LOAD_CELL_help = "Set the Zero point of the load cell"
    def cmd_TARE_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(50)
        samples = [sample[1] for sample in samples]
        # if there is already some tare weight applied, account for that:
        tare_weight = self.load_cell.tare_weight + average(samples)
        self.load_cell.tare(tare_weight)
        gcmd.respond_info("Load Cell tare weight value: %i" % (tare_weight))
    cmd_CALIBRATE_LOAD_CELL_help = "Set the conversion from raw counts to grams"
    def cmd_CALIBRATE_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(1000)
        samples = [sample[1] for sample in samples]
        avg = average(samples)
        volts = self.load_cell.sensor.sample_to_voltage(avg)
        grams = gcmd.get_float("GRAMS", default=1., minval=1., maxval=5000.)
        volts_per_gram = (volts / grams)
        self.load_cell.set_volts_per_gram(volts_per_gram)
        gcmd.respond_info("Load Cell volts per gram: %f" % (volts_per_gram))
    cmd_READ_LOAD_CELL_help = "Take a reading from the load cell"
    def cmd_READ_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(1000)
        samples = [sample[1] for sample in samples]
        min_sample = self.load_cell.sample_to_grams(min(samples))
        max_sample = self.load_cell.sample_to_grams(max(samples))
        sample_width = abs(max_sample - min_sample) / 2.
        avg = average(samples)
        #TODO: get std from numpy after rebasing for Python3 support
        standard_deviation = self.load_cell.sample_to_grams(avg)
        try_trigger_weight = standard_deviation * 5
        grams = self.load_cell.sample_to_grams(avg)
        gcmd.respond_info("Load Cell reading: raw average: %i, weight: %fg," /
            " min: %fg, max: %fg, noise: +/-%fg, standard deviation: %fg," /
            " trigger weight: %fg, samples: %i" % (avg, grams, min_sample
            , max_sample, sample_width, standard_deviation, try_trigger_weight
            , len(samples)))

# Utility for GCode commands to easily collect some samples for analysis
# blocks execution while collecting with reactor.pause()
class LoadCellSampleCollector(LoadCellSubscriber):
    def __init__(self, load_cell, reactor):
        self._load_cell = load_cell
        self._reactor = reactor
        self.is_capturing = False
        self.sps = 1
        self.samples = []
    def on_capture(self, samples):
        self.samples += samples
    def status(self, is_capturing, sps):
        self.is_capturing = is_capturing
        self.sps = sps
    def collect(self, samples=None):
        self.samples = []
        self._load_cell.subscribe(self)
        target = self.sps if samples == None else samples
        retry_delay = .1
        timeout = self._reactor.monotonic() + 1. + (target / self.sps)
        while (len(self.samples) < target and self.is_capturing):
            clock = self._reactor.monotonic()
            if (clock > timeout):
                break
            self._reactor.pause(self._reactor.monotonic() + retry_delay)
        self._load_cell.unsubscribe(self)
        self.samples = self.samples[:target]
        return self.samples

UPDATE_INTERVAL = .5
# Printer class that controls the load cell
class LoadCell:
    def __init__(self, config, sensor, load_cell_endstop):
        self.printer = printer = config.get_printer()
        # sensor must implement LoadCellDataSource
        self.sensor = sensor
        self.tare_weight = 0
        self.volts_per_gram = 1.
        LoadCellCommandHelper(config, self)
        self.load_cell_endstop = load_cell_endstop
        self.load_cell_endstop_oid = 0
        if load_cell_endstop is not None:
            self.load_cell_endstop_oid = load_cell_endstop.get_oid()
        # Setup mcu sensor_load_cell bulk query code
        self.mcu = mcu = self.sensor.get_spi().get_mcu()
        self.oid = oid = mcu.create_oid()
        spi_oid = self.sensor.get_spi().get_oid()
        self.samples = None
        mcu.add_config_cmd("config_load_cell oid=%d spi_oid=%d \
                            load_cell_sensor_type=%s load_cell_endstop_oid=%d"
            % (oid, spi_oid, self.sensor.get_capture_name()
            , self.load_cell_endstop_oid))
        mcu.add_config_cmd(
            "query_load_cell oid=%d clock=0 rest_ticks=0 time_shift=0"
            % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.name = config.get_name()
        self.update_timer = None
        self.dump = []
        self.subscribers = []
    def _build_config(self):
        self.samples = LoadCellCaptureHelper(self.printer, self)
        cmd_queue = self.sensor.get_spi().get_command_queue()
        self._set_report_rate_cmd = self.mcu.lookup_command(
            "set_load_cell_report_rate oid=%c report_rate=%u", cq=cmd_queue)
        self._tare_cmd = self.mcu.lookup_command(
            "set_load_cell_tare_weight oid=%c tare_weight=%i", cq=cmd_queue)
    def is_capturing(self):
        return self.samples.is_measuring()
    def sps(self):
        return self.sensor.get_samples_per_second()
    def start_capture(self):
        if self.is_capturing():
            return False
        logging.info("Starting load cell '%s' capture", self.name)
        self.sensor.start_capture()
        self.samples.start_capture()
        self._start_timer()
        logging.info("Stared load cell '%s' capture", self.name)
        for subscriber in self.subscribers:
            subscriber.status(self.is_capturing(), self.sps())
        return True
    def stop_capture(self):
        if not self.is_capturing():
            return False
        logging.info("Stopping load cell '%s' capture", self.name)
        self._stop_timer()
        self.samples.stop_capture()
        self.sensor.stop_capture()
        logging.info("Stopped load cell '%s' capture", self.name)
        for subscriber in self.subscribers:
            subscriber.status(self.is_capturing(), self.sps())
        return True
    def tare(self, tare_weight):
        self._tare_cmd.send([self.oid, tare_weight])
        self.tare_weight = tare_weight
    def set_volts_per_gram(self, volts_per_gram):
        self.volts_per_gram = volts_per_gram
    def sample_to_grams(self, sample):
        return self.sensor.sample_to_voltage(sample) / self.volts_per_gram
    def _start_timer(self):
        reactor = self.printer.get_reactor()
        systime = reactor.monotonic()
        waketime = systime + UPDATE_INTERVAL
        self.update_timer = reactor.register_timer(self._update_subscribers, waketime)
    def _stop_timer(self):
        reactor = self.printer.get_reactor()
        reactor.unregister_timer(self.update_timer)
        self.update_timer = None
    def _update_subscribers(self, eventtime):
        samples, errors, duplicates = self.samples.flush()
        if len(samples) == 0 and errors == 0 and duplicates == 0:
            return
        self.dump.append(samples)
        for subscriber in self.subscribers:
            subscriber.on_capture(samples)
        return eventtime + UPDATE_INTERVAL
    def set_report_rate(self, report_rate):
        sample_rate = self.sps() / min(report_rate, self.sps())
        self._set_report_rate_cmd.send([self.oid, sample_rate])
    def get_status(self, eventtime):
        dump_temp = self.dump
        self.dump = []
        return {'dump': dump_temp, 'is_capturing': self.is_capturing()
                , 'sps': self.sps(), 'tare_weight': self.tare_weight
                , 'volts_per_gram': self.volts_per_gram}
    def subscribe(self, load_cell_subscriber):
        load_cell_subscriber.status(self.is_capturing(), self.sps())
        self.subscribers.append(load_cell_subscriber)
    def unsubscribe(self, load_cell_subscriber):
        self.subscribers.remove(load_cell_subscriber)
    def get_collector(self):
        return LoadCellSampleCollector(self, self.printer.get_reactor())

def load_config(config):
    printer = config.get_printer()
    sensor = printer.lookup_object(config.get('sensor'))
    name = config.get_name().split()
    endstop = None
    if (len(name) == 2):
        name = " " + name[1]
    else:
        name = ""
    if config.getboolean('is_probe', default=False):
        endstop = LoadCellEndstop(config, sensor)
        printer.add_object('load_cell_endstop' + name, endstop)
        printer.add_object('load_cell_probe' + name, PrinterProbe(config, endstop))
    return LoadCell(config, sensor, endstop)

def load_config_prefix(config):
    #TODO: re-implement name support
    return LoadCell(config)
