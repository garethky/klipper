# Load Cell Implementation
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import binascii
import logging, threading, logging, struct, collections
from . import motion_report

#
# Types
#

# Interface for Multiplex ADC Sensors that want to supply high performance
# ADC data from the MCU
class MultiplexAdcSensor:
    # get the MCU where the sensor is physically connected
    def get_mcu(self):
        pass
    # return the oid for looking up the sensor by oid on the MCU
    def get_oid(self):
        pass
    # return the name for looking up the sensor type on the MCU
    def get_mux_adc_sensor_type(self):
        pass
    # command the sensor to start sensor capturing
    def start_capture(self):
        pass
    # command the sensor to stop sensor capturing
    def stop_capture(self):
        pass
    # get the number of samples per second that the sensor is configured for
    def get_samples_per_second(self):
        pass
    # return a status object which will be extended
    def get_status(self, eventtime):
        return {}

# A client that allows subscribers to get notifications when an event happens
# i.e. its not driven by a timer
class EventDumpClient:
    def __init__(self, callback):
        self._callback = callback
        self._is_closed = False
    def is_closed(self):
        return self._is_closed
    def close(self):
        self._is_closed = True
    def send(self, msg):
        self._callback(msg)

# allows internal clients to look like web requests
class WebRequestShim:
    def __init__(self, conn):
        self.conn = conn
    def get_client_connection(self):
        return self.conn
    def get_dict(self, template_key, map):
        # internal clients don't need templates
        return {}

#
# Constants
# 
MIN_MSG_TIME = 0.100
UPDATE_INTERVAL = .5  # update printer object at 2Hz
TIME_CODE_ERROR = 0xff
SAMPLES_PER_TRANSMISSION = 9
# Sample error types
ERROR_OVERFLOW = 0
ERROR_SCHEDULE = 1
ERROR_READ_TIME = 2
ERROR_CRC = 3
ERROR_SAMPLE_NOT_READY = 4


class MultiplexAdcCaptureHelper:
    def __init__(self, printer, oid, mcu):
        self.printer = printer
        self.oid = oid
        self.mcu = mcu
        self.lock = threading.Lock()
        self.last_sequence = 0
        # Capture message storage (accessed from background thread)
        self.capture_buffer = collections.deque([], None)
         # Measurement conversion
        self.start_clock = 0
        self.time_shift = 0
        self.sample_ticks = 0
        self.last_sequence = 0
        self.last_angle = 0
        self.last_chip_mcu_clock = 0
        self.last_chip_clock = 0
        self.static_delay = 0
        self._unpack_int32_from = struct.Struct('<i').unpack_from
        freq = mcu.seconds_to_clock(1.)
        while (float(TIME_CODE_ERROR << self.time_shift) / freq) < 0.002:
            self.time_shift += 1
        mcu.register_response(self._add_samples, "multiplex_adc_data", oid)
        query = "query_multiplex_adc oid=%c clock=%u rest_ticks=%u" \
                " time_shift=%c"
        query_end = "multiplex_adc_end oid=%c sequence=%hu"
        self.query_load_cell_cmd = self.mcu.lookup_command(query)
        self.query_load_cell_end_cmd = self.mcu.lookup_query_command(
            query, query_end, oid=oid)
    def _add_samples(self, new_capture):
        #deque is thread safe, no locking is required
        self.capture_buffer.append(new_capture)
    def reset(self):
        self.capture_buffer.clear()
        self.last_sequence = 0
        self.start_clock = 0
    def start_capture(self, sample_ticks):
        self.reset()
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        self.start_clock = reqclock = self.mcu.print_time_to_clock(print_time)
        self.sample_ticks = sample_ticks
        self.query_load_cell_cmd.send([self.oid, reqclock, sample_ticks,
                                       self.time_shift], reqclock=reqclock)
    def stop_capture(self):
        self.query_load_cell_end_cmd.send([self.oid, 0, 0, 0])
        self.reset()
    def is_capturing(self):
        return self.start_clock != 0
    def _process_error(self, error):
        sample = None
        if error == ERROR_SAMPLE_NOT_READY:
            sample = ("error_not_ready", 0, 1)
        elif error == ERROR_CRC:
            sample = ("error_crc", 0, 1)
        elif error == ERROR_READ_TIME:
            sample = ("error_read_time", 0, 1)
        else:
            sample = ("error_unknown", 0, 1)
        logging.error("MultiplexADC returned an ERROR: %s" % (sample[0]))
        return sample
    def flush(self):
        # local variables to optimize inner loop below
        capture_buffer = self.capture_buffer
        static_delay = self.static_delay
        sample_ticks = self.sample_ticks
        start_clock = self.start_clock
        #time_shift = self.time_shift
        clock_to_print_time = self.mcu.clock_to_print_time
        unpack_int32_from = self._unpack_int32_from
        last_sequence = self.last_sequence
        # Process every message in capture_buffer
        num_samples = len(capture_buffer)
        samples = collections.deque([], num_samples * SAMPLES_PER_TRANSMISSION) #[None] * ()
        for k in range(num_samples):
            params = capture_buffer.popleft()
            # This clears the bottom 16 bits of last_sequence and puts the 16 bits of params['sequence'] there
            seq = (last_sequence & ~0xffff) | params['sequence']
            # detect the 16 bit sequence number wrapping back to 0 and increment the 17+ bits by 1
            if seq < last_sequence:
                seq += 0x10000
            last_sequence = seq
            # Each data frame that comes from the sensor has SAMPLES_PER_TRANSMISSION samples inside it
            msg_mclock = start_clock + (seq * SAMPLES_PER_TRANSMISSION * sample_ticks)
            # the call to bytearray is only required for python2
            d = bytearray(params['data'])
            data_len = len(d)
            record_clock = msg_mclock
            i = 0
            try:
                while i < data_len:
                    record_clock += sample_ticks
                    time_diff = d[i]
                    i += 1
                    sample = unpack_int32_from(d, offset=i)[0]
                    i += 4
                    if time_diff == TIME_CODE_ERROR:
                        samples.append(self._process_error(sample))
                    else:
                        # timestamp is record_clock + offset shifted by time_shift
                        sample_clock = record_clock + time_diff
                        samples.append(["sample",
                                        clock_to_print_time(sample_clock),
                                        sample])
                    
            except Exception:
                logging.exception("Load Cell Samples threw an exception: " \
                    "%s (%i) i: %i" % (binascii.hexlify(d), data_len, i))
        # Debug
        #logging.debug("multiplex_adc precessed samples: " /
        #   "%i, %i" % (count, len(samples)))
        self.last_sequence = last_sequence
        return list(samples)

# Printer class that forms the basis of Multiplex ADC functionality
class MultiplexAdcSensorWrapper():
    def __init__(self, config, sensor):
        self.printer = printer = config.get_printer()
        self.sensor = sensor
        self.name = config.get_name()
        #name_parts = name.split()
        #sensor_name = ' '.join(name.split()[1:])
        self.load_cell_endstop_oid = 0
        self.mcu = mcu = self.sensor.get_mcu()
        self.mux_adc_oid = mcu.create_oid()
        self.samples = None
        self.update_timer = None
        self.dump = []
        #self.subscribers = []
        # API server endpoint
        self.api_dump = motion_report.APIDumpHelper(
            self.printer, self._api_update, self._api_startstop, 0.100)
        wh = self.printer.lookup_object('webhooks')
        wh.register_mux_endpoint("multiplex_adc/dump_adc", "sensor", self.name,
                                 self._add_webhooks_client)
        # startup
        mcu.register_config_callback(self._config_callback)
    def _config_callback(self):
        mcu = self.mcu
        mcu.add_config_cmd("config_multiplex_adc oid=%d mux_adc_sensor_type=%s \
                            sensor_oid=%d load_cell_endstop_oid=%d"
            % (self.mux_adc_oid, self.sensor.get_mux_adc_sensor_type()
               , self.sensor.get_oid(), self.load_cell_endstop_oid))
        mcu.add_config_cmd(
            "query_multiplex_adc oid=%d clock=0 rest_ticks=0 time_shift=0"
            % (self.mux_adc_oid,), on_restart=True)
        self.samples = MultiplexAdcCaptureHelper(self.printer
                                                 , self.mux_adc_oid, self.mcu)
    def _api_startstop(self, is_start):
        if is_start:
            self.start_capture()
        else:
            self.stop_capture()
    def _api_update(self, eventtime):
        samples = self.samples.flush()
        if len(samples) == 0:
            return {}
        return {'samples': samples}
    def _add_webhooks_client(self, web_request):
        self.api_dump.add_client(web_request)
        web_request.send({'header': ['type', 'time', 'value']})
    def attach_load_cell_endstop(self, endstop_oid):
        if self.load_cell_endstop_oid != 0:
            self.printer.invoke_shutdown("Endstop already configured on %s"
                                          % (self.name))
        self.load_cell_endstop_oid = endstop_oid
    def is_capturing(self):
        return self.samples.is_capturing()
    def get_samples_per_second(self):
        return self.sensor.get_samples_per_second()
    def get_clock_ticks_per_sample(self):
        sample_period = 1. / self.get_samples_per_second()
        return self.mcu.seconds_to_clock(sample_period)
    def start_capture(self):
        if self.is_capturing():
            return False
        logging.debug("Starting multiplex_adc '%s' capture", self.name)
        self.sensor.start_capture()
        self.samples.start_capture(self.get_clock_ticks_per_sample())
        logging.debug("Stared multiplex_adc '%s' capture", self.name)
        return True
    def stop_capture(self):
        if not self.is_capturing():
            return False
        logging.debug("Stopping multiplex_adc '%s' capture", self.name)
        self.samples.stop_capture()
        self.sensor.stop_capture()
        logging.debug("Stopped multiplex_adc '%s' capture", self.name)
        return True
    def get_mcu(self):
        return self.mcu
    def get_status(self, eventtime):
        sensor_status = self.sensor.get_status(eventtime)
        sensor_status.update({'is_capturing': self.is_capturing()
                , 'sps': self.sensor.get_samples_per_second()})
        return sensor_status
    def subscribe(self, callback):
        client = EventDumpClient(callback)
        self.api_dump.add_client(WebRequestShim(client))
        return client