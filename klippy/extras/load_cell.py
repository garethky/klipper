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
    # return the oid for looking up the sensor on the MCU
    def get_oid(self):
        pass
    # return the name for looking up the sensor on the MCU
    def get_senor_type(self):
        pass
    # get the MCU where the sensor is
    def get_mcu(self):
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
        # TODO: load over/under sample rate from the config
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
        value = struct.unpack_from('<i', b, offset=i)[0]
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
                        samples[count] = (ptime, sample)
                        count += 1
            except Exception:
                logging.exception("Load Cell Samples threw an error: %s (%i) i: %i, e: %i, d: %i", hexify(d), data_len, i, error_count, duplicate_count)
        self.last_sequence = last_sequence
        # trim empty entries from the end of the list
        del samples[count:]
        #logging.info("GOT SAMPLES: %i, %i, ERRORS: %i, duplicates: %i" % (count, len(samples), error_count, duplicate_count))
        return samples, error_count, duplicate_count

from numpy import std
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
    cmd_TARE_LOAD_CELL_help = "Set the Zero point of the load cell"
    def cmd_TARE_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(self.load_cell.sps())
        samples = [sample[1] for sample in samples]
        tare_counts = average(samples)
        self.load_cell.tare(tare_counts)
        gcmd.respond_info("Load Cell tare weight value: %i" % (tare_counts))
    cmd_CALIBRATE_LOAD_CELL_help = "Set the conversion from raw counts to grams"
    def cmd_CALIBRATE_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(1000)
        samples = [sample[1] for sample in samples]
        avg = average(samples)
        grams = gcmd.get_float("GRAMS", default=1., minval=1., maxval=5000.)
        counts_per_gram = abs(int(avg / grams))
        self.load_cell.set_counts_per_gram(counts_per_gram)
        max_weight = int(int(0x7FFFFFFF) / counts_per_gram)
        gcmd.respond_info("Load Cell Calibrates. counts per gram: %i, max\
                           weight: %ig" % (counts_per_gram, max_weight))
    cmd_READ_LOAD_CELL_help = "Take a reading from the load cell"
    def cmd_READ_LOAD_CELL(self, gcmd):
        samples = self.load_cell.get_collector().collect(1000)
        samples = [sample[1] for sample in samples]
        min_sample = self.load_cell.counts_to_grams(min(samples))
        max_sample = self.load_cell.counts_to_grams(max(samples))
        sample_width = abs(max_sample - min_sample) / 2.
        avg = average(samples)
        standard_deviation = abs(self.load_cell.counts_to_grams(int(std(samples))))
        try_trigger_weight = standard_deviation * 5.
        grams = self.load_cell.counts_to_grams(avg)
        gcmd.respond_info("Load Cell reading: raw average: %i, weight: %fg, \
             min: %fg, max: %fg, noise: +/-%fg, standard deviation: %sg, \
             trigger weight: %sg, samples: %i" % (avg, grams, min_sample
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
        self.tare_counts = None
        self.counts_per_gram = config.getint('counts_per_gram', minval=1
                                            , default=None)
        self.is_calibrated = not self.counts_per_gram is None
        LoadCellCommandHelper(config, self)
        self.load_cell_endstop = load_cell_endstop
        self.load_cell_endstop_oid = 0
        if load_cell_endstop is not None:
            self.load_cell_endstop_oid = load_cell_endstop.get_oid()
            # TODO: get the trigger_force into the load_cell_endstop
        # Setup mcu sensor_load_cell bulk query code
        self.mcu = mcu = self.sensor.get_mcu()
        self.oid = oid = mcu.create_oid()
        pi_oid = self.sensor.get_spi().get_oid()
        self.samples = None
        self.trigger_force_grams = config.getfloat('trigger_force_grams'
                                                   , minval=10., default=10.)
        mcu.add_config_cmd("config_load_cell oid=%d sensor_type=%s \
                            sensor_oid=%d load_cell_endstop_oid=%d"
            % (oid, self.sensor.get_capture_name(), self.sensor.get_oid()
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
    def tare(self, tare_counts):
        self.tare_counts = tare_counts
        self.set_endstop_range()
    def set_counts_per_gram(self, counts_per_gram):
        if counts_per_gram is None:
            self.counts_per_gram = None
            self.is_calibrated = False
        else:
            self.counts_per_gram = abs(counts_per_gram)
            self.is_calibrated = True
            self.set_endstop_range()
    def set_endstop_range(self):
        if not self.load_cell_endstop:
            return
        trigger_counts = int(self.trigger_force_grams * self.counts_per_gram)
        self.load_cell_endstop.set_range(trigger_counts, self.tare_counts)
    def counts_to_grams(self, sample):
        if self.counts_per_gram is None:
            return None
        return float(sample) / self.counts_per_gram
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
    def get_status(self, eventtime):
        samples_temp = self.dump
        self.dump = []
        return {'is_capturing': self.is_capturing()
                , 'is_calibrated': self.is_calibrated
                , 'sps': self.sps()
                , 'tare_counts': self.tare_counts
                , 'counts_per_gram': self.counts_per_gram
                , 'samples': samples_temp}
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
        printer.add_object('load_cell_probe' + name
                           , PrinterProbe(config, endstop))
    return LoadCell(config, sensor, endstop)

def load_config_prefix(config):
    #TODO: re-implement name support
    return LoadCell(config)
