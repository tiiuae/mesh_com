"""
Comms command control NATS node
"""
# pylint: disable=invalid-name, broad-except, too-many-branches

import json
import subprocess
import base64
from shlex import quote
import hashlib
import os
import time
from typing import Tuple, Union, List

import netifaces as ni  # type: ignore

from .comms_common import STATUS, COMMAND
from .comms_status import CommsStatus
from .comms_interface_info import CommsInterfaces


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
    WPA_LOG = "/var/log/wpa_supplicant_11s"  # e.g. _id0.log
    HOSTAPD_LOG = "/var/log/hostapd"  # e.g. _id0.log
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


class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class
    """

    def __init__(self, comms_status: List[CommsStatus], logger, nats_mode=False):
        self.logger = logger
        self.api_version = 1
        self.command = ""
        self.param = ""
        self.radio_index = ""
        self.interval = 1
        self.comms_status = comms_status
        self.nats_mode = nats_mode
        self.ctrl_mptcp = False

    def handle_command(self, msg: str, cc) -> Tuple[str, str, str]:
        """
        handler for commands

        Args:
            msg: JSON formatted data command message.
            cc: CommsController class

        Returns:
            str: OK/FAIL
            str: Additional info related to command
            str: Empty string or log data in case of logs query command
        """
        data = ""
        try:
            parameters = json.loads(msg)
            self.api_version = int(parameters["api_version"])
            self.command = quote(str(parameters["cmd"]))
            if "param" in parameters:
                self.param = quote(str(parameters["param"]))
            if "interval" in parameters:
                self.interval = int(parameters["interval"])
            if "radio_index" in parameters:
                self.radio_index = parameters["radio_index"]
            self.logger.debug("Command: %s", self.command)
        except (
            json.decoder.JSONDecodeError,
            KeyError,
            TypeError,
            AttributeError,
        ) as error:
            ret, info = "FAIL", "JSON format not correct" + str(error)
            cc.logger.debug("Command failed, %s", info)
            return ret, info, data

        if self.api_version != 1:
            ret, info = "FAIL", "API version not supported"
        elif self.command == COMMAND.revoke:
            ret, info = self.__revoke(cc)
        elif self.command == COMMAND.apply:
            ret, info = self.__apply_mission_config()
        elif self.command == COMMAND.wifi_down:
            if self.radio_index == "*":
                ret, info = self.__radio_down_all(cc)
            else:
                ret, info = self.__radio_down_single()
        elif self.command == COMMAND.wifi_up:
            if self.radio_index == "*":
                ret, info = self.__radio_up_all(cc)
            else:
                ret, info = self.__radio_up_single()
        # elif self.command == COMMAND.reboot:
        #     ret, info = "FAIL", "Command not implemented"
        # elif self.command == COMMAND.get_logs:
        #     ret, info, data = self.__get_logs(self.param)
        elif self.command == COMMAND.debug:
            ret, info, data = self.__debug(cc, self.param)
        # elif self.command == COMMAND.enable_visualisation:
        #     ret, info = self.__enable_visualisation(cc)
        # elif self.command == COMMAND.disable_visualisation:
        #     ret, info = self.__disable_visualisation(cc)
        # elif self.command == COMMAND.get_config:
        #     ret, info, data = self.__get_configs(self.param)
        # elif self.command == COMMAND.get_identity:
        #     ret, info, data = self.get_identity()  # type: ignore[assignment]
        else:
            ret, info = "FAIL", "Command not supported"
        return ret, info, data

    def __revoke(self, cc) -> Tuple[str, str]:
        """
        Restores device back to warehouse state without flashing.
        Note! Wi-Fi transmitter gets activates as well even if it
        was commanded off previously.

        Returns:
            tuple: (str, str)
        """
        config_file_path = f"/opt/{self.radio_index}_mesh.conf"
        hash_file_path = f"/opt/{self.radio_index}_mesh.conf_hash"
        pending_config_file_path = f"/opt/{self.radio_index}_mesh_stored.conf"

        if os.path.exists(config_file_path):
            try:
                os.remove(config_file_path)
            except Exception as e:
                self.logger.error("Failed to delete mission config, %s", e)
                return "FAIL", "Failed to delete mission config"

        if os.path.exists(pending_config_file_path):
            try:
                os.remove(pending_config_file_path)
            except Exception as e:
                self.logger.error("Failed to delete pending config, %s", e)

        if os.path.exists(hash_file_path):
            try:
                os.remove(hash_file_path)
            except Exception as e:
                # Not fatal
                self.logger.debug("Failed to delete hash file, %s", e)

        processes = [
            "/opt/S9011sNatsMesh",
            "/opt/S90APoint",
        ]
        if self.nats_mode:
            processes.append("/opt/S90nats_discovery")

        if self.ctrl_mptcp:
            # Stop S90mptcp before restarting S9011sNatsMesh
            ret = subprocess.run(
                ["/opt/S90mptcp", "stop"],
                shell=False,
                check=True,
                capture_output=True,
            )
            if ret.returncode != 0:
                return "FAIL", f"Clearing MPTCP configs failed" + str(ret.returncode) + str(
                    ret.stdout
                ) + str(ret.stderr)
            self.logger.debug("Stopped MPTCP service")

        for process in processes:
            # Restart mesh with default settings
            ret = subprocess.run(
                [process, "restart", "id" + self.radio_index],
                shell=False,
                check=True,
                capture_output=True,
            )
            if ret.returncode != 0:
                self.logger.error("Default mesh starting failed ")
                return (
                    "FAIL",
                    f"default mesh starting failed {process} "
                    + str(ret.returncode)
                    + str(ret.stdout)
                    + str(ret.stderr),
                )

        self.logger.debug("Default mesh command applied")

        # if self.comms_status[int(self.radio_index)].is_visualisation_active:
        #     status, _ = self.__disable_visualisation(cc)
        #     if status == "FAIL":
        #         return (
        #             "FAIL",
        #             "Revoke failed partially." + " Visualisation is still active",
        #         )

        return "OK", "Mesh settings revoked"

    def __apply_mission_config(self) -> Tuple[str, str]:
        """
        Replaces active mesh configuration file with previously
        stored content and restarts S9011Mesh with new configs.

        Returns:
            tuple: (str, str)
        """
        if (
            self.comms_status[int(self.radio_index)].mesh_cfg_status
            == STATUS.mesh_cfg_stored
        ):
            try:
                os.replace(
                    f"/opt/{self.radio_index}_mesh_stored.conf",
                    f"/opt/{self.radio_index}_mesh.conf",
                )
            except Exception as e:
                self.logger.error("Error replacing active config file! %s", e)
                return "FAIL", "Error replacing active config file"

            # Create hash file for active config bookkeeping
            try:
                with open(f"/opt/{self.radio_index}_mesh.conf", "rb") as file:
                    data = file.read()
                    hash_obj = hashlib.sha256(data)
                    hash_hex = hash_obj.hexdigest()
                with open(
                    f"/opt/{self.radio_index}_mesh.conf_hash",
                    "w",
                    encoding="utf-8",
                ) as f_hash:
                    f_hash.write(hash_hex)
            except Exception as e:
                self.logger.error("Error writing hash file!, %s", e)
                # Return failure amd give client a possibility to retry
                # instead of running initd script that could cause loss
                # of connection.
                return "FAIL", "Error writing hash file"

            # Initd script checks hash for mesh.conf config and in case it
            # matches then mission config is applied. Otherwise, default mesh
            # is applied. That logic is based on assumption that some
            # wireless connectivity needs to be ensured after reboot.
            processes = [
                "/opt/S9011sNatsMesh",
                "/opt/S90APoint",
            ]
            if self.nats_mode:
                processes.append("/opt/S90nats_discovery")

            if self.ctrl_mptcp:
                # Stop S90mptcp before restarting S9011sNatsMesh
                ret = subprocess.run(
                    ["/opt/S90mptcp", "stop"],
                    shell=False,
                    check=True,
                    capture_output=True,
                )
                if ret.returncode != 0:
                    return "FAIL", f"Clearing MPTCP configs failed" + str(
                        ret.returncode
                    ) + str(ret.stdout) + str(ret.stderr)
                self.logger.debug("Stopped MPTCP service")

            for process in processes:
                # delay before restarting mesh using delay
                ret = subprocess.run(
                    [process, "restart", "id" + self.radio_index],
                    shell=False,
                    check=True,
                    capture_output=True,
                )
                if ret.returncode != 0:
                    return "FAIL", f"mesh starting failed {process}" + str(
                        ret.returncode
                    ) + str(ret.stdout) + str(ret.stderr)
            self.logger.debug("Mission configurations applied")

            if self.ctrl_mptcp:
                time.sleep(5)
                # Start S90mptcp after restarting S9011sNatsMesh
                ret = subprocess.run(
                    ["/opt/S90mptcp", "start"],
                    shell=False,
                    check=True,
                    capture_output=True,
                )
                if ret.returncode != 0:
                    return "FAIL", f"Setting MPTCP configs failed" + str(
                        ret.returncode
                    ) + str(ret.stdout) + str(ret.stderr)
                self.logger.debug("Started MPTCP service")
            return "OK", "Mission configurations applied"
        self.logger.debug("No mission config to apply!")
        return "FAIL", "No setting to apply"

    def __radio_down_single(self) -> Tuple[str, str]:

        for process in ["/opt/S9011sNatsMesh", "/opt/S90APoint"]:
            ret = subprocess.run(
                [process, "stop", "id" + self.radio_index],
                shell=False,
                check=True,
                capture_output=True,
            )
            if ret.returncode != 0:
                self.logger.error("Failed to deactivate radio")
                return "FAIL", f"Radio deactivation failed {process} " + str(
                    ret.returncode
                ) + str(ret.stdout) + str(ret.stderr)

        self.logger.debug("Radio deactivated")
        return "OK", "Radio deactivated"

    def __radio_down_all(self, cc) -> Tuple[str, str]:
        for index in cc.settings.radio_index:
            self.radio_index = str(index)
            ret, info = self.__radio_down_single()
            if ret == "FAIL":
                return ret, info
        self.logger.debug("All radios deactivated")
        return "OK", "All radios deactivated"

    def __radio_up_single(self) -> Tuple[str, str]:

        for process in ["/opt/S9011sNatsMesh", "/opt/S90APoint"]:
            ret = subprocess.run(
                [process, "start", "id" + self.radio_index],
                shell=False,
                check=True,
                capture_output=True,
            )
            if ret.returncode != 0:
                self.logger.error("Failed to activate radio")
                return "FAIL", f"Radio activation failed {process} " + str(
                    ret.returncode
                ) + str(ret.stdout) + str(ret.stderr)

        self.logger.debug("Radio activated")
        return "OK", "Radio activated"

    def __radio_up_all(self, cc) -> Tuple[str, str]:
        for index in cc.settings.radio_index:
            self.radio_index = str(index)
            ret, info = self.__radio_up_single()
            if ret == "FAIL":
                return ret, info
        self.logger.debug("All radios activated")
        return "OK", "All radios activated"

    # def __enable_visualisation(self, cc) -> Tuple[str, str]:
    #     try:
    #         cc.telemetry.run()
    #     except Exception as e:
    #         self.logger.error("Failed to enable visualisation, %s", e)
    #         return "FAIL", "Failed to enable visualisation"
    #
    #     self.logger.debug("Visualisation enabled")
    #     self.comms_status[int(self.radio_index)].is_visualisation_active = True
    #     return "OK", "Visualisation enabled"
    #
    # def __disable_visualisation(self, cc) -> Tuple[str, str]:
    #     try:
    #         cc.telemetry.stop()
    #         cc.visualisation_enabled = False
    #     except Exception as e:
    #         self.logger.error("Failed to disable visualisation, %s", e)
    #         return "FAIL", "Failed to disable visualisation"
    #
    #     self.logger.debug("Visualisation disabled")
    #     self.comms_status[int(self.radio_index)].is_visualisation_active = False
    #     return "OK", "Visualisation disabled"
    #
    # @staticmethod
    # def __read_log_file(filename) -> bytes:
    #     """
    #     read file and return the content as bytes and base64 encoded
    #
    #     param: filename: str
    #     return: (int, bytes)
    #     """
    #     # read as bytes as b64encode expects bytes
    #     with open(filename, "rb") as file:
    #         file_log = file.read()
    #     return base64.b64encode(file_log)

    def __debug(self, cc, param) -> Tuple[str, str, str]:
        file = ""
        try:
            if cc.debug_mode_enabled:
                p = param.replace("'", "").split()
                ret = subprocess.run(
                    p,
                    shell=False,
                    check=True,
                    capture_output=True,
                )
                if ret.returncode != 0:
                    return "FAIL", "'{p}' DEBUG COMMAND failed", ""
                file_b64 = base64.b64encode(ret.stdout)
            else:
                return (
                    "FAIL",
                    f"DEBUG COMMAND disabled: cc.debug_mode_enabled: {cc.debug_mode_enabled}",
                    "",
                )
        except Exception as e:
            self.logger.error("DEBUG COMMAND failed, %s", e)
            return "FAIL", f"'{p}' DEBUG COMMAND failed", ""

        self.logger.debug("__debug done")
        return "OK", f"'{p}' DEBUG COMMAND done", file_b64.decode()

    # def __get_logs(self, param) -> Tuple[str, str, str]:
    #     file = ""
    #     try:
    #         files = LogFiles()
    #         if param == files.WPA:
    #             file_b64 = self.__read_log_file(
    #                 files.WPA_LOG + "_id" + self.radio_index + ".log"
    #             )
    #         elif param == files.HOSTAPD:
    #             file_b64 = self.__read_log_file(
    #                 files.HOSTAPD_LOG + "_id" + self.radio_index + ".log"
    #             )
    #         elif param == files.CONTROLLER:
    #             file_b64 = self.__read_log_file(files.CONTROLLER_LOG)
    #         elif param == files.DMESG:
    #             ret = subprocess.run(
    #                 [files.DMESG_CMD],
    #                 shell=False,
    #                 check=True,
    #                 capture_output=True,
    #             )
    #             if ret.returncode != 0:
    #                 return "FAIL", f"{file} file read failed", ""
    #             file_b64 = base64.b64encode(ret.stdout)
    #         else:
    #             return "FAIL", "Log file not supported", ""
    #
    #     except Exception as e:
    #         self.logger.error("Log reading failed, %s", e)
    #         return "FAIL", f"{param} log reading failed", ""
    #
    #     self.logger.debug("__getlogs done")
    #     return "OK", "wpa_supplicant log", file_b64.decode()
    #
    # def __get_configs(self, param) -> Tuple[str, str, str]:
    #     file_b64 = b"None"
    #     try:
    #         files = ConfigFiles()
    #         self.comms_status[int(self.radio_index)].refresh_status()
    #         if param == files.WPA:
    #             file_b64 = self.__read_log_file(
    #                 f"/var/run/wpa_supplicant-11s_id{self.radio_index}.conf"
    #             )
    #         elif param == files.HOSTAPD:
    #             file_b64 = self.__read_log_file(
    #                 f"/var/run/hostapd-{self.radio_index}.conf"
    #             )
    #         else:
    #             return "FAIL", "Parameter not supported", ""
    #
    #     except Exception as e:
    #         self.logger.error("Not able to get configs, %s", e)
    #         return "FAIL", "Not able to get config file", ""
    #
    #     self.logger.debug("__get_configs done")
    #     return "OK", f"{param}", file_b64.decode()
    #
    # def get_identity(self) -> Tuple[str, str, Union[str, dict]]:
    #     """
    #     Gathers identity, NATS server url and wireless interface
    #     device information and returns that in JSON compatible
    #     dictionary format.
    #
    #     Returns:
    #         tuple: (str, str, str | dict) -- A tuple that contains
    #         always textual status and description elements. 3rd
    #         element is JSON compatible in  normal cases but in
    #         case of failure it is an empty string.
    #     """
    #
    #     identity_dict = {}
    #     nats_ip = "NA"
    #     try:
    #         files = ConfigFiles()
    #         for status in self.comms_status:
    #             status.refresh_status()
    #
    #         with open(files.IDENTITY, "rb") as file:
    #             identity = file.read()
    #         identity_dict["identity"] = identity.decode().strip()
    #
    #         # pylint: disable=c-extension-no-member
    #         ips = ni.ifaddresses("br-lan")
    #         for item in ips[ni.AF_INET6]:
    #             if item["addr"].startswith("fd"):
    #                 nats_ip = item["addr"]
    #                 break
    #
    #         identity_dict["nats_url"] = f"nats://[{nats_ip}]:4222"
    #         identity_dict[
    #             "interfaces"
    #         ] = CommsInterfaces().get_wireless_device_info()  # type: ignore
    #     except Exception as e:
    #         self.logger.error("Not able to get identity, %s", e)
    #         return "FAIL", "Not able to get identity file", ""
    #
    #     self.logger.debug("get_identity done")
    #     return "OK", "Identity and NATS URL", identity_dict
