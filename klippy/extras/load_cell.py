# Load Cell Implementation
#
# Copyright (C) 2022 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, logging

from . import probe, load_cell_endstop, multiplex_adc

# An API Dump Helper for broadcasting events to clients
class WebhooksApiDumpRepeater:
    def __init__(self, printer, endpoint, key, value, header):
        self.clients = {}
        self.header = header
        wh = printer.lookup_object('webhooks')
        wh.register_mux_endpoint(endpoint, key, value,
                                 self._add_webhooks_client)
    def _add_webhooks_client(self, web_request):
        self.add_client(web_request)
        if self.header:
            web_request.send(self.header)
    def add_client(self, web_request):
        cconn = web_request.get_client_connection()
        template = web_request.get_dict('response_template', {})
        self.clients[cconn] = template
    def send(self, msg):
        for cconn, template in list(self.clients.items()):
            if cconn.is_closed():
                del self.clients[cconn]
                continue
            tmp = dict(template)
            tmp['params'] = msg
            cconn.send(tmp)

class LoadCellCommandHelper:
    def __init__(self, config, load_cell):
        try:
            import numpy as np
        except:
            raise config.error("LoadCell requires the numpy module")
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
        gcode.register_mux_command("TARE_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_TARE_LOAD_CELL,
                                   desc=self.cmd_TARE_LOAD_CELL_help)
        gcode.register_mux_command("CALIBRATE_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_CALIBRATE_LOAD_CELL,
                                   desc=self.cmd_CALIBRATE_LOAD_CELL_help)
        gcode.register_mux_command("READ_LOAD_CELL", "LOAD_CELL", name,
                                   self.cmd_READ_LOAD_CELL,
                                   desc=self.cmd_READ_LOAD_CELL_help)
    cmd_TARE_LOAD_CELL_help = "Set the Zero point of the load cell"
    def cmd_TARE_LOAD_CELL(self, gcmd):
        sps = self.load_cell.sensor.get_samples_per_second()
        samples = self.load_cell.get_collector().collect_samples(sps)
        counts = samples[2]
        import numpy as np
        tare_counts = np.average(counts)
        self.load_cell.tare(tare_counts)
        gcmd.respond_info("Load Cell tare counts value: %i" % (tare_counts))
    cmd_CALIBRATE_LOAD_CELL_help = "Set the conversion from raw counts to grams"
    def cmd_CALIBRATE_LOAD_CELL(self, gcmd):
        #if the tare value is not set, return an error
        if not self.load_cell.is_tared():
            gcmd.respond_error("TARE_LOAD_CELL before calibrating!")
            return
        grams = gcmd.get_float("GRAMS", default=1., minval=1., maxval=25000.)
        sps = self.load_cell.sensor.get_samples_per_second()
        samples = self.load_cell.get_collector().collect_samples(sps * 5)
        counts = samples[2]
        import numpy as np
        avg = np.average(counts)
        counts_per_gram = abs(int(avg / grams))
        self.load_cell.set_counts_per_gram(counts_per_gram)
        capacity = int(int(0x7FFFFFFF) / counts_per_gram)
        gcmd.respond_info("Load Cell Calibrated, Counts/gram: %i, \
                          Capacity: +/- %ig" % (counts_per_gram, capacity))
    cmd_READ_LOAD_CELL_help = "Take a reading from the load cell"
    def cmd_READ_LOAD_CELL(self, gcmd):
        sps = self.load_cell.sensor.get_samples_per_second()
        samples = self.load_cell.get_collector().collect_samples(sps * 5)
        import numpy as np
        counts = samples[2]
        avg = np.average(counts)
        if not self.load_cell.is_calibrated():
            gcmd.respond_info("Load Cell reading: raw average: %i, \
                               samples: %i" % (avg, len(counts)))
            return
        force = samples[1]
        grams = np.average(force)
        min_force = min(force)
        max_force = max(force)
        noise = round(abs(max_force - min_force) / 2., 6)
        sd = round(abs(np.std(force)), 6)
        five_sigma = round(np.std(force) * 5., 6)
        gcmd.respond_info("Load Cell reading: raw average: %i, weight: %fg, \
             min: %fg, max: %fg, noise: +/-%fg, standard deviation: %fg, \
             6 Sigma: %fg, samples: %i" % (avg, grams, min_force, max_force,
            noise, sd, five_sigma, len(counts)))

# Utility to easily collect some samples for later analysis
# Optionally blocks execution while collecting with reactor.pause()
# collet minimum n samples, collect until a specific sample time
# samples returned in [[time],[force]] arrays for easy processing
RETRY_DELAY = 0.01  # 100Hz
class LoadCellSampleCollector():
    def __init__(self, load_cell, reactor):
        self._load_cell = load_cell
        self._reactor = reactor
        self._mcu = load_cell.sensor.get_mcu()
        self.target_samples = 1
        self._counts = []
        self._force = []
        self._time = []
        self._client = None
    def _on_samples(self, data):
        # filter non "samples" type messages
        # TODO: what about errors?
        if not data.get("params", {}).get("samples"):
            return
        for sample in data["params"]["samples"]:
            # TODO: take out this reshaping, its not helping clients
            self._time.append(sample[0])
            self._force.append(sample[1])
            self._counts.append(sample[2])
    def start_collecting(self, samples=None):
        self.stop_collecting()
        self._counts = []
        self._force = []
        self._time = []
        self._client = self._load_cell.subscribe(self._on_samples)
    def stop_collecting(self):
        if self.is_collecting():
            self._client.close()
            self._client = None
    def get_samples(self):
        return (self._time, self._force, self._counts)
    def is_collecting(self):
        return self._client is not None
    # return true when the last sample has a time after the target time
    def _time_test(self, print_time):
        def test():
            return (not len(self._time) == 0) and self._time[-1] >= print_time
        return test
    # return true when a set number of samples have been collected
    def _count_test(self, target):
        def test():
            return len(self._force) >= target
        return test
    def _collect_until_test(self, test, timeout):
        if not self.is_collecting():
            self.start_collecting()
        while self.is_collecting() and not test():
            now = self._reactor.monotonic()
            print_time = self._mcu.estimated_print_time(now)
            if (print_time > timeout):
                logging.warn("LoadCellSampleCollector.collect_? timed out")
                break
            self._reactor.pause(now + RETRY_DELAY)
        return self.get_samples()
    # block execution until n samples are collected
    def collect_samples(self, samples=None):
        target = self.target_samples if samples == None else samples
        now = self._reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now)
        timeout = print_time + 1. + \
            (target / self._load_cell.sensor.get_samples_per_second())
        return self._collect_until_test(self._count_test(target), timeout)
    # block execution until a sample is returned with a timestamp after time
    def collect_until(self, time=None):
        now = self._reactor.monotonic()
        target = now if time == None else time
        timeout = 1. + target
        return self._collect_until_test(self._time_test(target), timeout)

# Printer class that controls the load cell
class LoadCell:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = name = config.get_name()
        sensor_name = config.get('sensor')
        self.sensor = sensor = printer.lookup_object(sensor_name)
        self.configfile = self.printer.lookup_object('configfile')
        self.load_cell_endstop = None
        if config.getboolean('is_probe', default=False):
            logging.info("DEBUG: " + name)
            self.load_cell_endstop = \
                load_cell_endstop.LoadCellEndstop(config, self)
            printer.add_object('load_cell_endstop:' + name
                        , self.load_cell_endstop)
            sensor.attach_load_cell_endstop(self.load_cell_endstop.get_oid())
            printer.add_object('load_cell_probe:' + name
                        , probe.PrinterProbe(config, self.load_cell_endstop))
        # sensor must implement LoadCellDataSource
        self.tare_counts = None
        self.counts_per_gram = config.getint('counts_per_gram', minval=1
                                            , default=None)
        LoadCellCommandHelper(config, self)
        self.trigger_force_grams = config.getfloat('trigger_force_grams'
                                                   , minval=10., default=50.)
        # webhooks support
        self.api_dump = WebhooksApiDumpRepeater(printer, "load_cell/dump_force",
                        "load_cell", name,
                        {"header": ["time", "force (g)", "counts"]})
        # startup, when klippy is ready, start capturing data
        printer.register_event_handler("klippy:ready", self._handle_ready)
    def _handle_ready(self):
        self.sensor_client = self.sensor.subscribe(self._sensor_data_event)
    # convert raw counts to grams and broadcast to clients
    def _sensor_data_event(self, data):
        if data.get("params", {}).get("samples"):
            samples = []
            for row in data["params"]["samples"]:
                if row[0] == 'sample':
                    # [time, grams, counts]
                    samples.append([row[1],
                                    self.counts_to_grams(row[2]),
                                    row[2]])
            self.api_dump.send({'samples': samples})
    def send_endstop_event(self, endstop_data):
        self.api_dump.send({'endstop': endstop_data})
    # get internal events of force data
    def subscribe(self, callback):
        client = multiplex_adc.EventDumpClient(callback)
        self.api_dump.add_client(multiplex_adc.WebRequestShim(client))
        return client
    def tare(self, tare_counts):
        self.tare_counts = tare_counts
        self.set_endstop_range()
    def set_counts_per_gram(self, counts_per_gram):
        if counts_per_gram is None:
            self.counts_per_gram = None
        else:
            self.counts_per_gram = abs(counts_per_gram)
            self.set_endstop_range()
    def set_trigger_force(self, trigger_force_grams):
        self.trigger_force_grams = trigger_force_grams
    def set_endstop_range(self):
        if not self.load_cell_endstop:
            return
        trigger_counts = int(self.trigger_force_grams * self.counts_per_gram)
        self.load_cell_endstop.set_range(trigger_counts, self.tare_counts)
    def counts_to_grams(self, sample):
        if not self.is_calibrated():
            return None
        return float(sample - self.tare_counts) / (self.counts_per_gram)
    def save_to_config(self):
        # Store results for SAVE_CONFIG
        self.configfile.set(self.name, 'counts_per_gram',
                             "%.3f" % (self.counts_per_gram,))
        self.configfile.set(self.name, 'trigger_force_grams',
                             "%.3f" % (self.trigger_force_grams,))
    # I swear on a stack of dictionaries this is correct english...
    def is_tared(self):
        return self.tare_counts is not None
    def is_calibrated(self):
        return self.is_tared() and self.counts_per_gram is not None
    def get_sensor(self):
        return self.sensor
    def get_collector(self):
        return LoadCellSampleCollector(self, self.printer.get_reactor())
    def get_status(self, eventtime):
        return {'is_calibrated': self.is_calibrated()
                , 'tare_counts': self.tare_counts
                , 'counts_per_gram': self.counts_per_gram
                , 'is_probe': self.load_cell_endstop != None}

def load_config_prefix(config):
    return LoadCell(config)
