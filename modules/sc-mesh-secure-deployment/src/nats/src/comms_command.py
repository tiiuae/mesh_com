"""
Comms command control NATS node
"""
import json
import subprocess
import base64
from shlex import quote
import logging

from .comms_common import STATUS, COMMAND
from .comms_status import CommsStatus


class LogFiles:  # pylint: disable=too-few-public-methods
    """
    LogFiles class
    """

    # Commands
    WPA = "WPA"
    HOSTAPD = "HOSTAPD"
    CONTROLLER = "CONTROLLER"
    DMESG = "DMESG"
    # Log files
    CONTROLLER_LOG = "/opt/comms_controller.log"
    WPA_LOG = "/var/log/wpa_supplicant_11s.log"
    HOSTAPD_LOG = "/var/log/hostapd.log"
    DMESG_CMD = "dmesg"


class Command:  # pylint: disable=too-few-public-methods
    """
    Command class
    """

    def __init__(self, server, port, comms_status: CommsStatus, logger):
        self.nats_server = server
        self.port = port
        self.logger = logger
        self.api_version = 1
        self.command = ""
        self.param = ""
        self.interval = 1
        self.comms_status = comms_status

    def handle_command(self, msg: str, cc) -> (str, str, STATUS, str):
        """
        handler for commands

        Args:
            msg: JSON formatted data from NATS message.
			cc: CommsController class

        Returns:
            str: OK/FAIL
            str: Additional info related to command
            STATUS: Status object that contains status string
            for NATS client.
        """
        status = STATUS.invalid_command
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
            self.logger.debug("Command: %s", self.command)
        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            ret, info = "FAIL", "JSON format not correct" + str(error)
            return ret, info, status

        if self.api_version != 1:
            ret, info = "FAIL", "API version not supported"
        elif self.command == COMMAND.revoke:
            ret, info, status = self.__revoke(cc)
        elif self.command == COMMAND.apply:
            ret, info, status = self.__apply_mission_config()
        elif self.command == COMMAND.wifi_down:
            ret, info, status = self.__radio_down()
        elif self.command == COMMAND.wifi_up:
            ret, info, status = self.__radio_up()
        elif self.command == COMMAND.reboot:
            ret, info = "FAIL", "Command not implemented"
        elif self.command == COMMAND.get_logs:
            ret, info, status, data = self.__get_logs(self.param)
        elif self.command == COMMAND.enable_visualisation:
            ret, info, status = self.__enable_visualisation(cc)
        elif self.command == COMMAND.disable_visualisation:
            ret, info, status = self.__disable_visualisation(cc)
        else:
            ret, info = "FAIL", "Command not supported"
        return ret, info, status, data

    def __revoke(self, cc) -> (str, str, STATUS):
        """
        Restores device back to warehouse state without flashing.
        Note! Wi-Fi transmitter gets activates as well even if it
        was commanded off previously.

        Returns:
            tuple: (str, str, STATUS)
        """
        path = "/opt"
        file = "mesh.conf"

        # Delete config files
        subprocess.run(["rm", "-f", f"{path}/{file}"],
                       shell=False, check=True, capture_output=True)

        # Ensure files were deleted
        ret = subprocess.run(["find", f"{path}", "-name", f"{file}"],
                             shell=False, check=True, capture_output=True)
        if ret.stdout:
            return "FAIL", "Failed to delete mesh config", \
                self.comms_status.mesh_cfg_status

        for process in ["/opt/S9011sMesh", "/opt/S90APoint"]:
            # Restart mesh with default settings
            ret = subprocess.run([process, "restart", "default"],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                # todo: Default mesh configuration failed. What to do next?
                return "FAIL", f"default mesh starting failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr), STATUS.mesh_fail

        self.logger.debug('Default mesh command applied')
        self.comms_status.mesh_status = STATUS.mesh_default
        self.comms_status.mesh_cfg_status = STATUS.mesh_default
        self.comms_status.is_mission_cfg = False

        ret, _, _ = self.__disable_visualisation(cc)
        if ret == "FAIL":
            return "FAIL", "Revoke failed partially. Visualisation is still active", \
                STATUS.mesh_default

        # Todo: radio status is inaccurate thus needs rework
        self.comms_status.is_radio_on = True
        return "OK", "Mesh settings revoked", STATUS.mesh_default

    def __apply_mission_config(self) -> (str, str, STATUS):
        """
        Runs S9011sMesh script with mission specific settings.

        Returns:
            tuple: (str, str, STATUS)
        """
        for process in ["/opt/S9011sMesh", "/opt/S90APoint"]:
            ret = subprocess.run([process, "restart", "mission"],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                return "FAIL", f"mesh starting failed {process}" \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr), STATUS.mesh_fail

        self.logger.debug('Mission configurations applied')
        self.comms_status.mesh_status = STATUS.mesh_mission_not_connected
        self.comms_status.is_mission_cfg = True
        return "OK", "Mission configurations applied", \
            STATUS.mesh_mission_not_connected

    def __radio_down(self) -> (str, str, STATUS):

        for process in ["/opt/S9011sMesh", "/opt/S90APoint"]:
            ret = subprocess.run([process, "stop"],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                return "FAIL", f"Radio deactivation failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr), STATUS.mesh_fail

        self.logger.debug('Radio deactivated')
        self.comms_status.is_radio_on = False
        if self.comms_status.is_mission_cfg:
            self.comms_status.mesh_status = STATUS.mesh_mission_not_connected
        else:
            self.comms_status.mesh_status = STATUS.mesh_default

        return "OK", "Radio deactivated", STATUS.wifi_off

    def __radio_up(self) -> (str, str, STATUS):

        for process in ["/opt/S9011sMesh", "/opt/S90APoint"]:
            if self.comms_status.is_mission_cfg:
                ret = subprocess.run([process, "start", "mission"],
                                     shell=False, check=True, capture_output=True)
            else:
                ret = subprocess.run([process, "start", "default"],
                                     shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                return "FAIL", f"Radio activation failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr), STATUS.mesh_fail

        # Update comms_status
        self.comms_status.is_radio_on = True
        if self.comms_status.is_mission_cfg:
            self.comms_status.mesh_status = STATUS.mesh_mission_not_connected
        else:
            self.comms_status.mesh_status = STATUS.mesh_default
        self.logger.debug("Radio activated")
        return "OK", "Radio activated", self.comms_status.mesh_status

    def __enable_visualisation(self, cc) -> (str, str, STATUS):

        try:
            cc.telemetry.run()
        except:
            self.logger.error('Visualisation enabled failed')
            return "FAIL", "Enabling visualization failed", \
                STATUS.visualisation_disabled
        self.logger.debug('Visualisation enabled')
        self.comms_status.is_visualisation_active = True
        return "OK", "Visualisation enabled", STATUS.visualisation_enabled

    def __disable_visualisation(self, cc) -> (str, str, STATUS):
        try:
            cc.telemetry.stop()
            cc.visualisation_enabled = False
        except:
            self.logger.error('Visualisation disable failed')
            return "FAIL", "Disabling visualization failed", \
                STATUS.visualisation_enabled
        self.logger.debug('Visualisation disabled')
        self.comms_status.is_visualisation_active = False
        return "OK", "Visualisation disabled", STATUS.visualisation_disabled

    def __read_log_file(self, filename) -> bytes:
        """
        read file and return the content as bytes and base64 encoded

        param: filename: str
        return: (int, bytes)
        """
        # read as bytes as b64encode expects bytes
        with open(filename, "rb") as f:
            file_log = f.read()
        return base64.b64encode(file_log)

    def __get_logs(self, param) -> (str, str, STATUS, str):
        file = ""
        try:
            files = LogFiles()
            if param == files.WPA:
                file_b64 = self.__read_log_file(files.WPA_LOG)
            elif param == files.HOSTAPD_LOG:
                file_b64 = self.__read_log_file(files.HOSTAPD_LOG)
            elif param == files.CONTROLLER:
                file_b64 = self.__read_log_file(files.CONTROLLER)
            elif param == files.DMESG:
                ret = subprocess.run([files.DMESG_CMD],
                                     shell=False, check=True, capture_output=True)
                if ret.returncode != 0:
                    return "FAIL", f"{file} file read failed", STATUS.no_status, None
                file_b64 = base64.b64encode(ret.stdout)
            else:
                return "FAIL", "Log file not supported", STATUS.no_status, None

        except:
            return "FAIL", f"{file} read failed", STATUS.no_status, None

        self.logger.debug('__getlogs done')
        # Fixme: What is correct mesh status to report in this case?
        return "OK", "wpa_supplicant log", STATUS.no_status, file_b64.decode()
