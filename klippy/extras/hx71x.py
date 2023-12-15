# HX711/HX717 Support
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import multiplex_adc

class HX71x(multiplex_adc.MultiplexAdcSensor):
    def __init__(self, config,
                 sample_rate_options, default_sample_rate,
                 gain_options, default_gain):
        self.printer = printer = config.get_printer()
        self.name = config.get_name()
        ppins = printer.lookup_object('pins')
        dout_pin_params = ppins.lookup_pin(config.get('dout_pin'))
        sclk_pin_params = ppins.lookup_pin(config.get('sclk_pin'))
        self.mcu = mcu = dout_pin_params['chip']
        self.oid = mcu.create_oid()
        self.dout_pin = dout_pin_params['pin']
        self.sclk_pin = sclk_pin_params['pin']
        # HX711 sps choices
        self.sps = config.getchoice('sample_rate', sample_rate_options,
                                    default=default_sample_rate)
        # HX711 gain/channel choices
        self.gain_channel = int(config.getchoice('gain', gain_options,
                                                 default=default_gain))
        ppins.register_chip(self.name, self)
        mcu.register_config_callback(self._build_config)
    def _build_config(self):
        self.mcu.add_config_cmd("config_hx71x oid=%d dout_pin=%s" \
                                    " sclk_pin=%s gain_channel=%d"
            % (self.oid, self.dout_pin, self.sclk_pin
               , self.gain_channel))
    def get_oid(self):
        return self.oid
    def get_mux_adc_sensor_type(self):
        return 'hx71x'
    def get_mcu(self):
        return self.mcu
    def start_capture(self):
        pass # capture cant be stopped/started on these chips
    def stop_capture(self):
        pass # capture cant be stopped/started on these chips
    def get_samples_per_second(self):
        return self.sps
    def get_bits(self):
        return 24

class HX711(HX71x):
    def __init__(self, config):
        super(HX711, self).__init__(config,
            # HX711 sps options
            {80: 80, 10: 10}, 10,
            # HX711 gain/channel options
            {'A-128': 1, 'B-32': 2, 'A-64': 3}, 'A-128')

class HX717(HX71x):
    def __init__(self, config):
        super(HX717, self).__init__(config,
            # HX717 sps options
            {320: 320, 80: 80, 20: 20, 10: 10}, 320,
            # HX717 gain/channel options
            {'A-128':1, 'B-64': 2, 'A-64': 3, 'B-8': 4}, 'A-128')
