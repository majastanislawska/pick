# klippy/extras/camera_ustreamer.py
# UStreamer Camera support for Pick
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import subprocess
import signal
import logging
import re

class UStreamer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.device = config.get('device', '/dev/video0')
        self.resolution = config.get('resolution', '1280x720')
        self.port = config.get('port', '8080')
        self.snapshot_path = f"pnp_snapshot_{self.name}.jpeg"
        self.extra_args = config.get('extra_args', '')  + f" --sink {self.snapshot_path} --sink-rm"
        self.process = None
        self.settings={}
        try: self.controls = self._get_supported_v4l2_controls()
        except subprocess.CalledProcessError as e:
            raise config.error(f"[camera_ustreamer {self.name}] Error: {e.returncode}: {str(e.stderr)}")
        logging.info(f"[camera_ustreamer {self.name}]: Detected V4L2 controls: {self.controls}")
        for ctrl_name, (value, ctrl_type_info, details) in self.controls.items():
            logging.info(f"[camera_ustreamer {self.name}]: Detected V4L2 control: {ctrl_name} = {value}  #({ctrl_type_info}) {details}")
            val = config.get(ctrl_name, None)
            if val is not None:
                self.settings[ctrl_name] = val

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('CAM_RESTART', "CAM", self.name,
                        self.cmd_CAM_RESTART, desc=self.cmd_CAM_RESTART_help)
        gcode.register_mux_command('CAM_SET', 'CAM', self.name, 
                        self.cmd_CAM_SET, desc=self.cmd_CAM_SET_help)
        gcode.register_mux_command('CAM_GET', 'CAM', self.name, 
                        self.cmd_CAM_GET, desc=self.cmd_CAM_GET_help)

        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        try: self._start_ustreamer()
        except subprocess.CalledProcessError as e:
            raise config.error(f"Failed to start ustreamer: {e.returncode}: {str(e.stderr)}")

    def _handle_shutdown(self):
        self._stop_ustreamer()

    def _get_supported_v4l2_controls(self):
        output = subprocess.check_output(["v4l2-ctl", "-d", self.device, "-l"], stderr=subprocess.PIPE, text=True)
        pattern = re.compile(r'^\s*([a-zA-Z0-9_]+)\s+(.+)\s+\((.+)\)\s+:\s+(.*)$')
        controls = {}
        for line in output.splitlines():
            logging.debug(f"[camera_ustreamer {self.name}]: Parsing line: {line}")
            match = pattern.match(line)
            if match:
                ctrl_name = match.group(1)
                ctrl_id_type = match.group(2).strip()
                ctrl_type_info = match.group(3).strip()
                ctrl_details = match.group(4).strip()
                val_match = re.search(r'\bvalue=(\d+)\b', ctrl_details) #remove value=.* from ctrl_details
                if val_match:
                    value = val_match.group(1)
                    details = re.sub(r'\bvalue=\d+\b', '', ctrl_details).strip()
                    details = re.sub(r'\s+', ' ', details) # squash blank spaces
                    controls[ctrl_name] = (value, ctrl_type_info, details)
        return controls
    
    def _apply_v4l2_settings(self,params):
        args = ["v4l2-ctl", "-d", self.device]
        for ctrl_name, val in params.items():
            args.extend(["-c", f"{ctrl_name}={val}"])
        logging.info(f"Setting V4L2 parameters for [{self.name}]: {' '.join(args)}")
        return subprocess.check_output(args, stderr=subprocess.PIPE)

    def _start_ustreamer(self):
        self._stop_ustreamer()
        # Build the execution command
        cmd = [
            "ustreamer",
            "--device", self.device,
            "--resolution", self.resolution,
            "--port", self.port,
            "--host", "0.0.0.0",
            "--device-timeout", "10",
            "--format", "MJPEG",
            "--encoder", "HW",
            "--tcp-nodelay", "-l",
            "-f", "15",
        ]
        
        # Append extra parameters if they exist
        if self.extra_args:
            cmd.extend(self.extra_args.split())
        logging.info(f"[camera_ustreamer {self.name}]: urstreamer: %s" % (cmd,))
        # Spawn the process in the background
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid # Create process group to ensure clean exit
        )
        logging.info("ustreamer started with PID %d" % (self.process.pid,))
        if self.settings:
            reactor = self.printer.get_reactor()
            reactor.pause(reactor.monotonic() + .250) # Wait 250ms for ustreamer to initialize
            return self._apply_v4l2_settings(self.settings)

    def _stop_ustreamer(self):
        if self.process and self.process.poll() is None:
            try:
                # Kill the entire process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    pass
            logging.info("ustreamer stopped")
        self.process = None

    cmd_CAM_RESTART_help = "Restarts the ustreamer service from console"
    def cmd_CAM_RESTART(self, gcmd):
        gcmd.respond_info("Restarting ustreamer...")
        try: self._start_ustreamer()
        except subprocess.CalledProcessError as e:
            raise gcmd.error(f"command returned {e.returncode}: {str(e.stderr)}")

    cmd_CAM_GET_help = "Retrieves current V4L2 camera configuration"
    def cmd_CAM_GET(self, gcmd):
        gcmd.respond_info(f"=== Konfiguracja V4L2 wygenerowana dla [{self.name}] ===")
        #take it from system again
        try: self.controls = self._get_supported_v4l2_controls()
        except subprocess.CalledProcessError as e:
            raise gcmd.error(f"Failed to get V4L2 parameters: {e.returncode}: {str(e.stderr)}")
        gcmd.respond_info("\nyou can copy these values to your camera section in printer.cfg:\n\n")
        for ctrl_name, (value, ctrl_type_info, clean_details) in self.controls.items():
            gcmd.respond_info(f"{ctrl_name}: {value}  #{ctrl_type_info}, {clean_details}")

    
    cmd_CAM_SET_help = "Sets V4L2 camera parameters. Usage: CAM_SET CAM=... PARAM=VALUE"
    def cmd_CAM_SET(self, gcmd):
        params = {k.lower(): gcmd.get(k).lower() for k in gcmd.get_command_parameters() if k != 'CAM'}
        if params:
            try:
                return self._apply_v4l2_settings(params)
            except subprocess.CalledProcessError as e:
                raise gcmd.error(f"Failed to set V4L2 parameters: {e.returncode}: {str(e.stderr)}")
        raise gcmd.error("Missing parameters")

    def get_status(self, eventtime):
        ret= {'pid': self.process.pid if self.process else None, 
              'running': self.process.poll() is None if self.process else False,
              'device': self.device,
              'resolution': self.resolution,
              'port': self.port,
              'camera_controls': self.controls,
              }
        return ret

def load_config_prefix(config):
    return UStreamer(config)