import json
import logging
import os

from . import load_cell_probe

class TapRecorder(load_cell_probe.TapClassifierModule):
    def __init__(self, config):
        self.is_recording = False
        self.tap_file_name = None
        self.printer = config.get_printer()
        self.register_commands()
        self.tap_classifier_module = self.load_module(config,
                'tap_classifier_module', load_cell_probe.TapClassifierModule())

    def classify(self, tap_analysis):
        # allow module chaining
        self.tap_classifier_module.classify(tap_analysis)
        if self.is_recording:
            self.save_tap(tap_analysis)

    def load_module(self, config, name, default):
        module = config.get(name, default=None)
        return default if module is None else self.printer.lookup_object(module)

    def save_tap(self, tap_analysis):
        data = []
        if (os.path.exists(self.tap_file_name)
                and os.path.getsize(self.tap_file_name) > 0):
            try:
                with open(self.tap_file_name, "r", encoding="utf-8") as f:
                    data = json.load(f)  # Load existing JSON data
                    if not isinstance(data, list):
                        raise ValueError("JSON file does not contain an array")
            except Exception as e:
                logging.error("Failed to read file contents", e)
                raise e
        data.append(tap_analysis.to_dict())
        try:
            with open(self.tap_file_name, "w", encoding="utf-8") as f:
                #json.dump(data, file, indent=4)
                f.write(json.dumps(data, indent=4))
        except Exception as e:
            logging.error("Failed to write file", e)
            raise e

    cmd_start_recording_help = "Start recording tap data"
    def cmd_start_recording(self, gcmd):
        full_path = "_"
        try:
            self.tap_file_name = gcmd.get("FILE", default=None)
            full_path = os.path.abspath(self.tap_file_name)
            gcmd.respond_info("Saving tap data to %s" % (full_path,))
            open(self.tap_file_name, "w", encoding="utf-8")
            self.is_recording = True
        except Exception as e:
            error_msg = "Failed to open file %s" % (full_path,)
            raise gcmd.error(error_msg)


    cmd_stop_recording_help = "Stop recording tap data"
    def cmd_stop_recording(self, gcmd):
        self.is_recording = False

    def register_commands(self):
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TAP_RECORDER_START",
                                   self.cmd_start_recording,
                                   desc=self.cmd_start_recording_help)
        gcode.register_command("TAP_RECORDER_STOP",
                                   self.cmd_stop_recording,
                                   desc=self.cmd_stop_recording_help)

def load_config(config):
    return TapRecorder(config)
