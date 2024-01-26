# Load Cell Implementation
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import binascii
import logging, math, threading, logging, struct, collections
from . import motion_report, adxl345

#
# Types
#

# Interface for Multiplex ADC Sensors that want to supply high performance
# ADC data from the MCU
class MultiplexAdcSensor(object):
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
    # return the number of bits this sensor supports
    def get_bits(self):
        return 32
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

# This is a wrapper for ClockSyncRegression that also manages tracking 16bit
# sequence counters and generating times for messages
class MessageSequence:
    def __init__(self, mcu, samples_per_message, sps):
        self.mcu = mcu
        self.samples_per_message = samples_per_message
        self.last_sample_time = 0
        # the sequence count of the last clock sync
        self.clock_sync_sequence = 0
        smooth_time = 2 * (sps * UPDATE_INTERVAL)
        self.clock_sync = adxl345.ClockSyncRegression(mcu, smooth_time)
    def reset(self, mcu_print_time):
        self.last_sample_time = 0
        self.clock_sync_sequence = 0
        self.clock_sync.reset(mcu_print_time, 0)
    # convert from a sequence number + offset to a sample number
    def _to_sample_count(self, i):
        base = self.clock_sync_sequence * self.samples_per_message
        return base + i
    # When the chip reports time & pending samples
    def updateRegression(self, mcu_clock_32, next_sequence
                         , pending_sample_count):
        mcu_clock = self.mcu.clock32_to_clock64(mcu_clock_32)
        sequence_count = (self.clock_sync_sequence & ~0xffff) | next_sequence
        # detect overflow and increment
        if sequence_count < self.clock_sync_sequence:
            sequence_count += 0x10000
        self.clock_sync_sequence = sequence_count
        sample_count = self._to_sample_count(pending_sample_count)
        self.clock_sync.update(mcu_clock, sample_count)
    # When chip sends messages & sequence counter
    # returns array of predicted message times
    def messages(self, sequence_count_16):
        seq_delta = (self.clock_sync_sequence - sequence_count_16) & 0xffff
        seq_delta -= (seq_delta & 0x8000) << 1
        seq_64 = self.clock_sync_sequence - seq_delta
        time_base, sample_count_base, sample_interval = \
            self.clock_sync.get_time_translation()
        sample_count_diff = (seq_64 * self.samples_per_message) \
                            - sample_count_base
        out = []
        for i in range(self.samples_per_message):
            out.append(time_base + ((sample_count_diff + i) * sample_interval))
        self.last_sample_time = out[len(out) - 1]
        return out
    def get_status(self):
        time_base, sample_count_base, sample_interval = \
            self.clock_sync.get_time_translation()
        sps = round(1.0 / sample_interval, 6)
        return {
            'measured_sps': sps,
            'last_sample_time': self.last_sample_time,
            'last_sequence_number': self.clock_sync_sequence,
            'clock_sync_regression': {
                'time_base': time_base,
                'sample_count_base': sample_count_base,
                'sample_interval': sample_interval,
            }
        }

#
# Constants
#
SAMPLE_WIDTH = 4 # samples are 4 byte wide unsigned integers
SAMPLES_PER_MESSAGE = 12
MIN_MSG_TIME = 0.100
UPDATE_INTERVAL = 0.100  # update printer object at 10Hz
DEFAULT_STATUS_DURATION = .000005

class MultiplexAdcCaptureHelper:
    def __init__(self, printer, oid, mcu, sps):
        self.printer = printer
        self.oid = oid
        self.mcu = mcu
        self.start_clock = 0
        self.message_seq = MessageSequence(mcu, SAMPLES_PER_MESSAGE, sps)
        # Capture message storage (accessed from background thread)
        self.capture_buffer = collections.deque([], None)
        # Measurement conversion
        self._unpack_int32_from = struct.Struct('<i').unpack_from
        # command setup
        mcu.register_response(self._add_samples, "multiplex_adc_data", oid)
        query = "query_multiplex_adc oid=%c clock=%u rest_ticks=%u"
        query_end = "multiplex_adc_end oid=%c sequence=%hu"
        status = "query_multiplex_adc_status oid=%c"
        status_response = "multiplex_adc_status oid=%c clock=%u" \
            " next_sequence=%hu pending=%c"
        self.query_load_cell_cmd = self.mcu.lookup_command(query)
        self.query_load_cell_end_cmd = self.mcu.lookup_query_command(
            query, query_end, oid=oid)
        self.status_cmd = self.mcu.lookup_query_command(status
            , status_response, oid=self.oid)
    def _add_samples(self, new_capture):
        #deque is thread safe, no locking is required
        self.capture_buffer.append(new_capture)
    def now(self):
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        return self.mcu.print_time_to_clock(print_time)
    def reset(self):
        self.start_clock = 0
        self.capture_buffer.clear()
    def start_capture(self, rest_ticks):
        self.reset()
        self.start_clock = reqclock = self.now()
        self.message_seq.reset(reqclock)
        self.sample_ticks = rest_ticks
        self.query_load_cell_cmd.send([self.oid, reqclock, rest_ticks,
                                       0], reqclock=reqclock)
        self.update_clock(minclock=reqclock)
    def stop_capture(self):
        self.query_load_cell_end_cmd.send([self.oid, 0, 0, 0])
        self.reset()
    def is_capturing(self):
        return self.start_clock != 0
    def get_status(self):
        status = self.message_seq.get_status()
        status.update({'is_capturing': self.is_capturing()})
        return status
    def flush(self):
        self.update_clock()
        # local variables to optimize inner loop below
        capture_buffer = self.capture_buffer
        unpack_int32_from = self._unpack_int32_from
        message_seq = self.message_seq
        # Process every message in capture_buffer
        num_messages = len(capture_buffer)
        samples = collections.deque(maxlen=num_messages * SAMPLES_PER_MESSAGE)
        for message_index in range(num_messages):
            # thread safe removal of 1 message from deque
            params = capture_buffer.popleft()
            sequence = params['sequence']
            # the call to bytearray is only required for python2
            data = bytearray(params['data'])
            data_len = len(data)
            # each sample is 4 bytes, so the number of samples is
            sample_count = data_len // SAMPLE_WIDTH
            sample_times = message_seq.messages(sequence)
            try:
                byte_index = 0
                for sample_index in range(sample_count):
                    sample = unpack_int32_from(data, offset=byte_index)[0]
                    byte_index += 4
                    samples.append([sample_times[sample_index], sample])
            except Exception:
                logging.exception("Load Cell Samples threw an exception: " \
                    "%s (%i) i: %i" % \
                    (binascii.hexlify(data), data_len, byte_index))
        return list(samples)
    def update_clock(self, minclock=0):
        # Query current state
        params = self.status_cmd.send([self.oid], minclock=minclock)
        mcu_clock = self.mcu.clock32_to_clock64(params['clock'])
        self.message_seq.updateRegression(mcu_clock, \
            params['next_sequence'], params['pending'])

# Printer class that forms the basis of Multiplex ADC functionality
class MultiplexAdcSensorWrapper():
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.sensor = sensor
        self.name = config.get_name()
        self.load_cell_endstop_oid = 0
        self.mcu = mcu = self.sensor.get_mcu()
        self.mux_adc_oid = mcu.create_oid()
        self.samples = None
        # API server endpoint
        self.api_dump = motion_report.APIDumpHelper(
            self.printer, self._api_update, self._api_startstop
            , UPDATE_INTERVAL)
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
            "query_multiplex_adc oid=%d clock=0 rest_ticks=0"
            % (self.mux_adc_oid,), on_restart=True)
        sps = self.get_samples_per_second()
        self.samples = MultiplexAdcCaptureHelper(self.printer
                                    , self.mux_adc_oid, self.mcu, sps)
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
        web_request.send({'header': ['type', 'time', 'value', 'clock_error']})
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
        sample_period = 1. / float(self.get_samples_per_second())
        return self.mcu.seconds_to_clock(sample_period)
    def start_capture(self):
        if self.is_capturing():
            return False
        logging.debug("Starting multiplex_adc '%s' capture", self.name)
        self.sensor.start_capture()
        # sample at a faster rate than the sensor sample time
        rest_time = (0.7 / self.get_samples_per_second())
        rest_ticks = self.mcu.seconds_to_clock(rest_time)
        self.samples.start_capture(rest_ticks)
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
        capture_status = self.samples.get_status()
        sensor_status.update(capture_status)
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        sensor_status['current_print_time'] = print_time
        return sensor_status
    def subscribe(self, callback):
        client = EventDumpClient(callback)
        self.api_dump.add_client(WebRequestShim(client))
        return client
