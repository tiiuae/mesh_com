"""
Comms command control NATS node
"""
import json
import subprocess
import base64
from shlex import quote

from .comms_common import STATUS, COMMAND


class LogFiles:  # pylint: disable=too-few-public-methods
    """
    LogFiles class
    """

    # Commands
    WPA = "WPA"
    DMESG = "DMESG"
    # Log files
    WPA_LOG = "/var/log/wpa_supplicant_11s.log"


class Command:  # pylint: disable=too-few-public-methods
    """
    Command class
    """

    def __init__(self, server, port):
        self.nats_server = server
        self.port = port
        self.api_version = 1
        self.command = ""
        self.param = ""
        self.interval = 1

    def handle_command(self, msg: str) -> (str, str, str, str):
        """
        handler for commands
        """
        mesh_status = STATUS.no_status
        data = ""
        try:
            parameters = json.loads(msg)
            print(parameters)
            self.api_version = int(parameters["api_version"])
            self.command = quote(str(parameters["cmd"]))
            if "param" in parameters:
                self.param = quote(str(parameters["param"]))
            if "interval" in parameters:
                self.interval = int(parameters["interval"])
        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            ret, info = "FAIL", "JSON format not correct" + str(error)
            return ret, info, mesh_status

        if self.api_version != 1:
            ret, info = "FAIL", "API version not supported"
        elif self.command == COMMAND.revoke:
            ret, info, mesh_status = self.__activate_default_mesh()
        elif self.command == COMMAND.apply:
            ret, info, mesh_status = self.__apply_mission_config()
        elif self.command == COMMAND.wifi_down:
            ret, info = "FAIL", "Command not implemented"
        elif self.command == COMMAND.wifi_up:
            ret, info = "FAIL", "Command not implemented"
        elif self.command == COMMAND.reboot:
            ret, info = "FAIL", "Command not implemented"
        elif self.command == COMMAND.get_logs:
            ret, info, mesh_status, data = self.__get_logs(self.param)
        elif self.command == COMMAND.enable_visualisation:
            ret, info = self.__enable_visualisation()
        elif self.command == COMMAND.disable_visualisation:
            ret, info = self.__disable_visualisation()
        else:
            ret, info = "FAIL", "Command not supported"

        return ret, info, mesh_status, data

    def __activate_default_mesh(self) -> (str, str, str):
        ret = subprocess.run(["/opt/S9011sMesh", "restart", "default"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            # todo: Default mesh configuration failed. What to do next?
            return "FAIL", "default mesh starting failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr), STATUS.mesh_fail

        return "OK", "Default mesh command applied", STATUS.mesh_default

    def __apply_mission_config(self) -> (str, str, str):
        ret = subprocess.run(["/opt/S9011sMesh", "restart", "mission"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            return "FAIL", "mesh starting failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr), STATUS.mesh_fail

        print('Mission configurations applied')
        return "OK", "Mission configurations applied", \
            STATUS.mesh_mission_not_connected

    def __radio_down(self) -> (str, str, str):
        ret = subprocess.run(["/opt/S9011sMesh", "stop"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            return "FAIL", "Radio deactivation failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr), STATUS.mesh_fail

        print('Radio deactivated')
        # Fixme: What is correct mesh status to report in this case?
        return "OK", "Radio deactivated", STATUS.no_status

    def __radio_up(self) -> (str, str, str):
        # Todo: Should we enable default mesh if it was previously in use?
        ret = subprocess.run(["/opt/S9011sMesh", "start", "mission"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            return "FAIL", "Radio activation failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr), STATUS.mesh_fail
        print("Radio activated")
        # Fixme: What is correct mesh status to report in this case?
        return "OK", "Radio activated", STATUS.mesh_mission_not_connected

    def __enable_visualisation(self) -> (str, str):
        ret = subprocess.run(["/opt/S90comms_visual", "start",
                              str(self.nats_server), str(self.interval)],
                             shell=False, check=True,
                             capture_output=True)
        if ret.returncode != 0:
            return "FAIL", "Enabling visualization failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr)
        print('Visualisation activated')
        return "OK", "Visualisation activated"

    def __disable_visualisation(self) -> (str, str):
        # FIXME: process stopping doesn't work with current script
        ret = subprocess.run(["/opt/S90comms_visual", "stop"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            return "FAIL", "Disabling visualization failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr)
        print('Visualisation disabled')
        return "OK", "Visualisation disabled"

    def __get_logs(self, param) -> (str, str, str, str):
        file = ""
        try:
            files = LogFiles()
            if param == files.WPA:
                file = files.WPA_LOG
                # read as bytes as b64encode expects bytes
                with open(file, "rb") as f:
                    file_log = f.read()
                file_b64 = base64.b64encode(file_log)

            elif param == files.DMESG:
                ret = subprocess.run(["dmesg"],
                                     shell=False, check=True, capture_output=True)
                if ret.returncode != 0:
                    return "FAIL", f"{file} file read failed", STATUS.no_status, None
                file_b64 = base64.b64encode(ret.stdout)
            else:
                return "FAIL", "Log file not supported", STATUS.no_status, None

        except:
            return "FAIL", f"{file} read failed", STATUS.no_status, None

        print('Log file read')
        # Fixme: What is correct mesh status to report in this case?
        return "OK", "wpa_supplicant log", STATUS.no_status, file_b64.decode()
