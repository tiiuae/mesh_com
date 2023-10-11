"""
Comms command control NATS node
"""
import json
import subprocess
import base64
from shlex import quote
import hashlib
import os
import netifaces as ni

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


class ConfigFiles:  # pylint: disable=too-few-public-methods
    """
    ConfigFiles class
    """
    # Commands
    WPA = "WPA_CONFIG"
    HOSTAPD = "HOSTAPD_CONFIG"

    # Config files
    IDENTITY = "/opt/identity"


class Command:  # pylint: disable=too-few-public-methods, too-many-instance-attributes
    """
    Command class
    """

    def __init__(self, server, port, comms_status: [CommsStatus, ...], logger):
        self.nats_server = server
        self.port = port
        self.logger = logger
        self.api_version = 1
        self.command = ""
        self.param = ""
        self.radio_index = ""
        self.interval = 1
        self.comms_status = comms_status

    def handle_command(self, msg: str, cc, csa=False, delay="0") -> (str, str, str):
        """
        handler for commands

        Args:
            msg: JSON formatted data from NATS message.
            cc: CommsController class
            csa: bool: True if CSA is active
            delay: str: delay for channel change in csa case

        Returns:
            str: OK/FAIL
            str: Additional info related to command
            str: Empty string or log data in case of logs query command
        """
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
            if "radio_index" in parameters:
                self.radio_index = parameters["radio_index"]
            self.logger.debug("Command: %s", self.command)
        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            ret, info = "FAIL", "JSON format not correct" + str(error)
            cc.logger.debug("Command failed, %s", info)
            return ret, info, data

        if self.api_version != 1:
            ret, info = "FAIL", "API version not supported"
        elif self.command == COMMAND.revoke:
            ret, info = self.__revoke(cc)
        elif self.command == COMMAND.apply:
            ret, info = self.__apply_mission_config(csa, delay)
        elif self.command == COMMAND.wifi_down:
            ret, info = self.__radio_down()
        elif self.command == COMMAND.wifi_up:
            ret, info = self.__radio_up()
        elif self.command == COMMAND.reboot:
            ret, info = "FAIL", "Command not implemented"
        elif self.command == COMMAND.get_logs:
            ret, info, data = self.__get_logs(self.param)
        elif self.command == COMMAND.enable_visualisation:
            ret, info = self.__enable_visualisation(cc)
        elif self.command == COMMAND.disable_visualisation:
            ret, info = self.__disable_visualisation(cc)
        elif self.command == COMMAND.get_config:
            ret, info, data = self.__get_configs(self.param)
        elif self.command == COMMAND.get_identity:
            ret, info, data = self.get_identity()
        else:
            ret, info = "FAIL", "Command not supported"
        return ret, info, data

    def __revoke(self, cc) -> (str, str):
        """
        Restores device back to warehouse state without flashing.
        Note! Wi-Fi transmitter gets activates as well even if it
        was commanded off previously.

        Returns:
            tuple: (str, str)
        """
        config_file_path = "/opt/mesh.conf"
        hash_file_path = "/opt/mesh.conf_hash"
        pending_config_file_path = "/opt/mesh_stored.conf"

        if os.path.exists(config_file_path):
            try:
                os.remove(config_file_path)
            except:
                self.logger.error("Failed to delete mission config")
                return "FAIL", "Failed to delete mission config"

        if os.path.exists(pending_config_file_path):
            try:
                os.remove(pending_config_file_path)
            except:
                self.logger.error("Failed to delete pending config")

        if os.path.exists(hash_file_path):
            try:
                os.remove(hash_file_path)
            except:
                # Not fatal
                self.logger.debug("Failed to delete hash file")

        for process in ["/opt/S9011sNatsMesh", "/opt/S90APoint", "/opt/S90nats_discovery"]:
            # Restart mesh with default settings
            ret = subprocess.run([process, "restart"],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                self.logger.error("Default mesh starting failed ")
                return "FAIL", f"default mesh starting failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr)

        self.logger.debug("Default mesh command applied")
        if self.comms_status[int(self.radio_index)].is_visualisation_active:
            ret, _, _ = self.__disable_visualisation(cc)
            if ret == "FAIL":
                return "FAIL", "Revoke failed partially." \
                        + " Visualisation is still active"

        return "OK", "Mesh settings revoked"

    def __apply_mission_config(self, csa=False, delay="0") -> (str, str):
        """
        Replaces active mesh configuration file with previously
        stored content and restarts S9011Mesh with new configs.

        Returns:
            tuple: (str, str)
        """
        if self.comms_status[int(self.radio_index)].mesh_cfg_status == STATUS.mesh_cfg_stored:
            try:
                os.replace(f"/opt/{self.radio_index}_mesh_stored.conf", f"/opt/{self.radio_index}_mesh.conf")
            except:
                self.logger.error("Error replacing active config file!")
                return "FAIL", "Error replacing active config file"

            # Create hash file for active config bookkeeping
            try:
                with open(f"/opt/{self.radio_index}_mesh.conf", "rb") as f:
                    data = f.read()
                    hash_obj = hashlib.sha256(data)
                    hash_hex = hash_obj.hexdigest()
                with open(f"/opt/{self.radio_index}_mesh.conf_hash", "w") as f_hash:
                    f_hash.write(hash_hex)
            except:
                self.logger.error("Error writing hash file!")
                # Return failure amd give client a possibility to retry
                # instead of running initd script that could cause loss
                # of connection.
                return "FAIL", "Error writing hash file"

            # Initd script checks hash for mesh.conf config and in case it
            # matches then mission config is applied. Otherwise, default mesh
            # is applied. That logic is based on assumption that some
            # wireless connectivity needs to be ensured after reboot.
            if csa:
                processes = ["/opt/S9011sNatsMesh", "/opt/S90APoint"]
            else:
                processes = ["/opt/S9011sNatsMesh", "/opt/S90APoint", "/opt/S90nats_discovery"]

            for process in processes:
                # delay before restarting mesh using delay
                if delay != "0":
                    ret = subprocess.run(["sleep", delay],
                                            shell=False, check=True,
                                            capture_output=True)

                ret = subprocess.run([process, "restart", "id" + self.radio_index],
                                     shell=False, check=True,
                                     capture_output=True)
                if ret.returncode != 0:
                    return "FAIL", f"mesh starting failed {process}" \
                                   + str(ret.returncode) \
                                   + str(ret.stdout) \
                                   + str(ret.stderr)
            self.logger.debug("Mission configurations applied")
            return "OK", "Mission configurations applied"
        else:
            self.logger.debug("No mission config to apply!")
            return "FAIL", "No setting to apply"

    def __radio_down(self) -> (str, str):

        for process in ["/opt/S9011sNatsMesh", "/opt/S90APoint"]:
            ret = subprocess.run([process, "stop", "id" + self.radio_index],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                self.logger.error("Failed to deactivate radio")
                return "FAIL", f"Radio deactivation failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr)

        self.logger.debug("Radio deactivated")
        return "OK", "Radio deactivated"

    def __radio_up(self) -> (str, str):

        for process in ["/opt/S9011sNatsMesh", "/opt/S90APoint"]:
            ret = subprocess.run([process, "start", "id" + self.radio_index],
                                 shell=False, check=True,
                                 capture_output=True)
            if ret.returncode != 0:
                self.logger.error("Failed to activate radio")
                return "FAIL", f"Radio activation failed {process} " \
                               + str(ret.returncode) \
                               + str(ret.stdout) \
                               + str(ret.stderr)

        self.logger.debug("Radio activated")
        return "OK", "Radio activated"

    def __enable_visualisation(self, cc) -> (str, str):
        try:
            cc.telemetry.run()
        except:
            self.logger.error("Failed to enable visualisation")
            return "FAIL", "Failed to enable visualisation"

        self.logger.debug("Visualisation enabled")
        self.comms_status[int(self.radio_index)].is_visualisation_active = True
        return "OK", "Visualisation enabled"

    def __disable_visualisation(self, cc) -> (str, str):
        try:
            cc.telemetry.stop()
            cc.visualisation_enabled = False
        except:
            self.logger.error("Failed to disable visualisation")
            return "FAIL", "Failed to disable visualisation"

        self.logger.debug("Visualisation disabled")
        self.comms_status[int(self.radio_index)].is_visualisation_active = False
        return "OK", "Visualisation disabled"

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

    def __get_logs(self, param) -> (str, str, str):
        file = ""
        try:
            files = LogFiles()
            if param == files.WPA:
                file_b64 = self.__read_log_file(files.WPA_LOG)
            elif param == files.HOSTAPD:
                file_b64 = self.__read_log_file(files.HOSTAPD_LOG)
            elif param == files.CONTROLLER:
                file_b64 = self.__read_log_file(files.CONTROLLER_LOG)
            elif param == files.DMESG:
                ret = subprocess.run([files.DMESG_CMD],
                                     shell=False, check=True,
                                     capture_output=True)
                if ret.returncode != 0:
                    return "FAIL", f"{file} file read failed", None
                file_b64 = base64.b64encode(ret.stdout)
            else:
                return "FAIL", "Log file not supported", None

        except:
            return "FAIL", f"{param} log reading failed", None

        self.logger.debug("__getlogs done")
        return "OK", "wpa_supplicant log", file_b64.decode()

    def __get_configs(self, param) -> (str, str, str):
        file_b64 = b'None'
        try:
            files = ConfigFiles()
            self.comms_status[int(self.radio_index)].refresh_status()
            if param == files.WPA:
                if_name = self.comms_status[int(self.radio_index)].mesh_interface_name
                if if_name:
                    file_b64 = self.__read_log_file(
                        f"/var/run/wpa_supplicant-11s_{if_name}.conf")
            elif param == files.HOSTAPD:
                if_name = self.comms_status[int(self.radio_index)].ap_interface_name
                if if_name:
                    file_b64 = self.__read_log_file(
                        f"/var/run/hostapd-{if_name}.conf")
            else:
                return "FAIL", "Parameter not supported", None

        except:
            return "FAIL", "Not able to get config file", None

        self.logger.debug("__get_configs done")
        if not if_name:
            return "FAIL", f"{param}, interface not active", None
        else:
            return "OK", f"{param}", file_b64.decode()

    def get_identity(self) -> (str, str, dict):
        identity_dict = {}
        try:
            files = ConfigFiles()
            self.comms_status[0].refresh_status()
            self.comms_status[1].refresh_status()
            self.comms_status[2].refresh_status()

            with open(files.IDENTITY, "rb") as f:
                identity = f.read()
            identity_dict["identity"] = identity.decode().strip()
            # todo hardcoded interface name
            identity_dict["nats_url"] = f"nats://{ni.ifaddresses('br-lan')[ni.AF_INET][0]['addr']}:4222"

        except:
            return "FAIL", "Not able to get identity file", None

        self.logger.debug("get_identity done")
        return "OK", "Identity and NATS URL", identity_dict