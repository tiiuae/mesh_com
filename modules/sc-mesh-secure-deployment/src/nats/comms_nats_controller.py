"""
Comms NATS controller
"""
from os import sys, path

if __name__ == "__main__" and __package__ is None:
    PARENT_DIR = path.dirname(path.dirname(path.abspath(__file__)))
    MS20_FEATURES_PATH = path.join(PARENT_DIR, "2_0/features")
    sys.path.append(MS20_FEATURES_PATH)
    # Make it possible to run this module also in other folder (e.g. /opt/nats):
    sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features")

# pylint: disable=wrong-import-position
import argparse
import asyncio
import json
import logging
import signal
import ssl
import subprocess
import threading
import os
import glob
import base64
from enum import Enum
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Optional
from typing import List, Dict

import OpenSSL.crypto
import requests
import yaml
from pyroute2 import IPRoute  # type: ignore[import-not-found, import-untyped]
from cbma import setup_cbma  # type: ignore[import-not-found]
from nats.aio.client import Client as nats

from src import batadvvis
from src import batstat
from src import comms_command
from src import comms_if_monitor
from src import comms_service_discovery
from src import comms_settings
from src import comms_status


# FMO_SUPPORTED_COMMANDS = [
#     "GET_IDENTITY",
#     "ENABLE_VISUALISATION",
#     "DISABLE_VISUALISATION",
# ]


class MeshTelemetry:
    """
    Mesh network telemetry collector
    """

    def __init__(self, loop_interval: int = 1000, logger: logging.Logger = None):
        self.thread_visual = None
        self.thread_stats = None
        # milliseconds to seconds
        self.interval = float(loop_interval / 1000.0)
        self.logger = logger
        self.batman_visual = batadvvis.BatAdvVis(self.interval * 0.2)
        self.batman = batstat.Batman(self.interval * 0.2)
        self.visualisation_enabled = False

    def mesh_visual(self):
        """
        Get mesh visualisation

        :return: mesh visualisation
        """
        return (
            f"[{self.batman_visual.latest_topology},"
            f"{self.batman.latest_stat}]".replace(": ", ":").replace(", ", ",")
        )

    def run(self):
        """
        Run method to start collecting visualisation telemetry

        :return: -
        """
        self.thread_visual = threading.Thread(
            target=self.batman_visual.run
        )  # create thread
        self.thread_visual.start()  # start thread
        self.thread_stats = threading.Thread(target=self.batman.run)  # create thread
        self.thread_stats.start()  # start thread
        self.visualisation_enabled = True  # publisher enabled

    def stop(self):
        """
        Stop method for collecting telemetry

        :return: -
        """
        self.visualisation_enabled = False  # publisher disabled
        if self.batman_visual.thread_running:
            self.batman_visual.thread_running = False  # thread loop disabled
            self.thread_visual.join()  # wait for thread to finish
        if self.batman.thread_running:
            self.batman.thread_running = False  # thread loop disabled
            self.thread_stats.join()  # wait for thread to finish


# pylint: disable=too-many-instance-attributes
class CommsController:  # pylint: disable=too-few-public-methods
    """
    Mesh network
    """

    def __init__(self, interval: int = 1000):
        self.interval = interval
        self.debug_mode_enabled = False

        # base logger for comms and which is used by all other modules
        self.main_logger = logging.getLogger("comms")
        self.main_logger.setLevel(logging.DEBUG)
        log_formatter = logging.Formatter(
            fmt="%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s"
        )
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.main_logger.addHandler(console_handler)

        self.c_status = []
        # TODO how many radios?
        for i in range(0, 3):
            self.c_status.append(
                comms_status.CommsStatus(
                    self.main_logger.getChild(f"status {str(i)}"), i
                )
            )

        self.settings = comms_settings.CommsSettings(
            self.c_status, self.main_logger.getChild("settings")
        )

        for cstat in self.c_status:
            if cstat.index < len(self.settings.mesh_vif):
                cstat.wifi_interface = self.settings.mesh_vif[cstat.index]

        self.command = comms_command.Command(
            self.c_status, self.main_logger.getChild("command")
        )
        self.telemetry = MeshTelemetry(
            self.interval, self.main_logger.getChild("telemetry")
        )

        # logger for this module and derived from main logger
        self.logger = self.main_logger.getChild("controller")


class ConfigType(str, Enum):
    """
    Config type
    """

    MESH_CONFIG = "mesh_conf"
    BIRTH_CERTIFICATE = "birth_certificate"
    FEATURES = "features"
    DEBUG_CONFIG = "debug_conf"


# pylint: disable=too-few-public-methods
class Interface:
    """
    Class to store interface name, operationstatus and MAC address
    """

    def __init__(self, interface_name, operstat, mac_address):
        self.interface_name = interface_name
        self.operstat = operstat
        self.mac_address = mac_address


# pylint: enable=too-few-public-methods


class MdmAgent:
    """
    MDM Agent
    """

    DOWNLOAD_CERTIFICATES = "download_certificates"
    UPLOAD_CERTIFICATES = "upload_certificates"

    GET_CONFIG: str = "public/config"
    GET_DEVICE_CONFIG: str = "public/get_device_config"  # + config_type
    PUT_DEVICE_CONFIG: str = "public/put_device_config"  # + config_type

    GET_DEVICE_CERTIFICATES: str = "public/get_device_certificates"
    PUT_DEVICE_CERTIFICATES: str = "public/put_device_certificates"

    OK_POLLING_TIME_SECONDS: int = 600
    FAIL_POLLING_TIME_SECONDS: int = 1
    YAML_FILE: str = (
        "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"
    )

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        comms_controller,
        keyfile: str = None,
        certificate_file: str = None,
        ca_file: str = None,
        interface: str = None,
    ):
        """
        Constructor
        """
        self.__previous_config_mesh: Optional[str] = self.__read_config_from_file(
            ConfigType.MESH_CONFIG.value
        )
        self.__previous_config_features: Optional[str] = self.__read_config_from_file(
            ConfigType.FEATURES.value
        )
        self.__previous_config_certificates: Optional[
            str
        ] = self.__read_config_from_file(ConfigType.BIRTH_CERTIFICATE.value)
        self.__previous_debug_config: Optional[
            str
        ] = self.__read_debug_config_from_file()
        self.__mesh_conf_request_processed = False
        self.__comms_controller: CommsController = comms_controller
        self.__interval: int = (
            self.FAIL_POLLING_TIME_SECONDS
        )  # possible to update from MDM server?
        self.__debug_config_interval: int = (
            self.FAIL_POLLING_TIME_SECONDS
        )  # possible to update from MDM server?

        self.__url: str = "defaultmdm.local:5000"  # mDNS callback updates this one
        self.__keyfile: str = keyfile
        self.__certificate_file: str = certificate_file
        self.__ca: str = ca_file
        self.__config_version: int = 0
        self.mdm_service_available: bool = False
        self.__if_to_monitor = interface
        self.__interfaces: List[Interface] = []
        self.executor = ThreadPoolExecutor(1)
        self.service_monitor = comms_service_discovery.CommsServiceMonitor(
            service_name="MDM Service",
            service_type="_mdm._tcp.local.",
            service_cb=self.mdm_server_address_cb,
            interface=self.__if_to_monitor,
        )
        self.interface_monitor = comms_if_monitor.CommsInterfaceMonitor(
            self.interface_monitor_cb
        )
        self.__service_monitor_thread = threading.Thread(
            target=self.service_monitor.run
        )

        self.thread_if_mon = threading.Thread(
            target=self.interface_monitor.monitor_interfaces
        )

        self.__cbma_set_up = False  # Indicates whether CBMA has been configured
        try:
            with open("/opt/certs_uploaded", "r", encoding="utf-8") as f:
                self.__certs_uploaded = True
                self.__comms_controller.logger.debug("No need to upload certificates")
        except FileNotFoundError:
            self.__certs_uploaded = False
            self.__comms_controller.logger.debug("Certificate upload needed")

        try:
            with open("/opt/certs_downloaded", "r", encoding="utf-8") as f:
                self.__certs_downloaded = True
                self.__comms_controller.logger.debug("No need to download certificates")
        except FileNotFoundError:
            self.__certs_downloaded = False
            self.__comms_controller.logger.debug("Certificate download needed")

        self.__cbm_certs_path = "/opt/crypto/ecdsa/birth/filebased"
        self.__cbm_certs_downloaded = "/opt"
        self.__cbma_ca_cert_path = "/opt/mspki/ecdsa/certificate_chain.crt"
        self.__cbma_threads: Dict[str, str] = {}
        self.__cbma_shutdown_event = threading.Event()
        self.running = False

        self.__status: dict = {
            ConfigType.MESH_CONFIG.value: "FAIL",
            ConfigType.FEATURES.value: "FAIL",
            self.DOWNLOAD_CERTIFICATES: "OK" if self.__certs_downloaded else "FAIL",
            self.UPLOAD_CERTIFICATES: "OK" if self.__certs_uploaded else "FAIL",
        }

        try:
            with open("/opt/identity", "r", encoding="utf-8") as f:  # todo
                self.__device_id = f.read().strip()
        except FileNotFoundError:
            self.__device_id = "default"

    def mdm_server_address_cb(self, address: str, status: bool) -> None:
        """
        Callback for MDM server address
        :param address: MDM server address
        :param status: MDM connection status
        :return: -
        """
        self.__url = address
        self.mdm_service_available = status

    def interface_monitor_cb(self, interfaces) -> None:
        """
        Callback for network interfaces. Service
        :param interfaces: list of dictionaries that includes
                           interface_name, operstate, mac_address.
        :return: -
        """
        self.__comms_controller.logger.debug("Interface monitor cb: %s", interfaces)
        self.__interfaces.clear()

        for interface_data in interfaces:
            interface = Interface(
                interface_name=interface_data["interface_name"],
                operstat=interface_data["operstate"],
                mac_address=interface_data["mac_address"],
            )
            self.__interfaces.append(interface)

        if self.__if_to_monitor is None:
            self.start_service_discovery()
        else:
            for interface in self.__interfaces:
                if interface.interface_name == self.__if_to_monitor:
                    if interface.operstat == "UP":
                        self.start_service_discovery()
                    elif interface.operstat == "DOWN":
                        self.stop_service_discovery()

    def start_service_discovery(self):
        """
        Starts service monitor thread
        :param: -
        :return: -
        """
        if not self.service_monitor.running:
            self.__comms_controller.logger.debug("Start service monitor thread")

            self.__service_monitor_thread.start()

    def stop_service_discovery(self):
        """
        Stops service monitor thread
        :param: -
        :return: -
        """
        if self.service_monitor.running:
            self.__comms_controller.logger.debug("Stop service monitor thread")
            self.service_monitor.close()
            self.__service_monitor_thread.join()

    def start_interface_monitor(self) -> None:
        """
        Start interface monitoring
        """
        self.thread_if_mon.start()

    async def execute(self) -> None:
        """
        Execute MDM agent
        :return: -
        """
        self.running = True

        while self.running:
            self.__comms_controller.logger.debug("status: %s, mdm_available: %s", self.__status, self.mdm_service_available)

            if (
                self.__status[self.UPLOAD_CERTIFICATES] == "FAIL"
                and self.mdm_service_available ):

                resp = self.upload_certificate_bundle()
                self.__comms_controller.logger.debug(
                    "Certificate upload response: %s", resp
                )
                if resp.status_code == 200:
                    self.__status[self.UPLOAD_CERTIFICATES] = "OK"
                    with open("/opt/certs_uploaded", "w", encoding="utf-8") as f:
                        f.write("certs uploaded")
                else:
                    self.__comms_controller.logger.debug("Certificate upload failed")
            elif (
                self.__status[self.DOWNLOAD_CERTIFICATES] == "FAIL"
                and self.__status[self.UPLOAD_CERTIFICATES] == "OK"
                and self.mdm_service_available ):

                resp = self.download_certificate_bundle()
                self.__comms_controller.logger.debug(
                    "Certificate download response: %s", resp
                )
                if resp.status_code == 200:
                    self.__status[
                        self.DOWNLOAD_CERTIFICATES
                    ] = self.__action_certificates(resp)
                    if self.__status[self.DOWNLOAD_CERTIFICATES] == "OK":
                        with open("/opt/certs_downloaded", "w", encoding="utf-8") as f:
                            f.write("certs downloaded")
                    else:
                        self.__comms_controller.logger.debug(
                            "Certificates action failed"
                        )

                else:
                    self.__comms_controller.logger.debug("Certificate download failed")
            elif self.mdm_service_available:
                await self.__loop_run_executor(self.executor, ConfigType.FEATURES)
                await self.__loop_run_executor(self.executor, ConfigType.MESH_CONFIG)
                if self.__mesh_conf_request_processed:
                    await self.__loop_run_executor(
                        self.executor, ConfigType.DEBUG_CONFIG
                    )
            if self.__is_cbma_feature_enabled() and not self.__cbma_set_up:
                self.__setup_cbma()
            if self.__cbma_threads:
                self.__cbma_thread_alive_check()

            await asyncio.sleep(
                float(min(self.__interval, self.__debug_config_interval))
            )

    def __http_get_device_config(self, config: ConfigType) -> requests.Response:
        """
        HTTP get device config
        :return: HTTP response
        """
        try:
            # "mDNS" discovery callback updates the url and can be None
            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            return requests.get(
                f"https://{__https_url}/{self.GET_DEVICE_CONFIG}/{config.value}",
                params={"device_id": self.__device_id},
                cert=(self.__certificate_file, self.__keyfile),
                verify=self.__ca,
                timeout=2,
            )
        except requests.exceptions.ConnectionError as err:
            self.__comms_controller.logger.error(
                "HTTP request failed with error: %s", err
            )
            return requests.Response()

    def __action_certificates(self, response: requests.Response) -> str:
        """
        Action certificates
        :param response: HTTP response
        :return: -
        """
        try:
            payload_dict = json.loads(response.text)["payload"]
            self.__comms_controller.logger.debug(
                "payload_dict[role]: %s", payload_dict["role"]
            )
            self.__comms_controller.logger.debug(
                "payload_dict[group]: %s", payload_dict["group"]
            )

            certificates_dict = json.loads(response.text)["payload"]["certificates"]

            for certificate_name, certificate in certificates_dict.items():
                with open(
                    f"{self.__cbm_certs_downloaded}/{certificate_name}", "wb"
                ) as cert_file:
                    cert_file.write(base64.b64decode(certificate))

            self.__comms_controller.logger.debug("Certificates saved")
        except Exception as e:
            self.__comms_controller.logger.error(
                "Error saving certificates: %s", str(e)
            )
            return "FAIL"
        return "OK"

    def __action_radio_configuration(self, response: requests.Response) -> str:
        """
        Take radio configuration into use
        :param response: https response
        :return: status
        """
        config: dict = json.loads(response.text)

        try:
            self.__comms_controller.logger.debug(
                f"config: {config} previous: {json.loads(self.__previous_config_mesh)}"
            )
            if json.loads(self.__previous_config_mesh) == config:
                self.__comms_controller.logger.debug(
                    "No changes in mesh config, not updating."
                )
                return "OK"
        except TypeError:
            self.__comms_controller.logger.debug("No previous mesh config")

        ret, info = self.__comms_controller.settings.handle_mesh_settings(
            json.dumps(config["payload"])
        )

        self.__comms_controller.logger.debug("ret: %s info: %s", ret, info)

        if ret == "OK":
            for radio in config["payload"]["radios"]:
                # Extract the radio_index
                index_number = radio["radio_index"]

                # Create the command
                cmd = json.dumps(
                    {
                        "api_version": 1,
                        "cmd": "APPLY",
                        "radio_index": index_number,
                    }
                )

                ret, info, _ = self.__comms_controller.command.handle_command(
                    cmd, self.__comms_controller
                )
                self.__comms_controller.logger.debug("ret: %s info: %s", ret, info)

            if ret == "OK":
                self.__config_version = int(config["version"])
                self.__write_config_to_file(response, ConfigType.MESH_CONFIG.value)

                self.__previous_config_mesh = self.__read_config_from_file(
                    ConfigType.MESH_CONFIG.value
                )

        return ret

    def __action_feature_yaml(self, response: requests.Response) -> str:
        """
        Take feature.yaml into use
        :param config: config dict
        :return: status
        """
        # check if features field exists

        try:
            config: dict = json.loads(response.text)

            try:
                self.__comms_controller.logger.debug(
                    f"config: {config} previous: {json.loads(self.__previous_config_features)}"
                )
                if json.loads(self.__previous_config_features) == config:
                    self.__comms_controller.logger.debug(
                        "No changes in features config, not updating."
                    )
                    return "OK"
            except TypeError:
                self.__comms_controller.logger.debug("No previous features config")

            if config["payload"] is not None:
                features_dict = config["payload"]["features"]

                features_yaml = yaml.dump(features_dict, default_flow_style=False)

                with open(
                    self.YAML_FILE,
                    "w",
                    encoding="utf-8",
                ) as file_handle:
                    file_handle.write(features_yaml)

                self.__write_config_to_file(response, ConfigType.FEATURES.value)
                self.__previous_config_features = self.__read_config_from_file(
                    ConfigType.FEATURES.value
                )
                return "OK"

            self.__comms_controller.logger.error("No features field in config")
        except KeyError:
            self.__comms_controller.logger.error("KeyError features field in config")

        return "FAIL"

    def __action_cert_keys(self, response: requests.Response) -> str:
        """
        Take cert/keys into use
        :param config: config dict
        :return: status
        """
        try:
            config: dict = json.loads(response.text)

            try:
                if json.loads(self.__previous_config_certificates) == config:
                    self.__comms_controller.logger.debug(
                        "No changes in features config, not updating."
                    )
                    return "OK"
            except TypeError:
                self.__comms_controller.logger.debug("No previous certificates config")

            if config["payload"] is not None:
                data = config["payload"]["features"]  # todo implementation
                self.__comms_controller.logger.debug("certs field in config")
                self.__comms_controller.logger.debug(
                    "__action_cert_keys: not implemented"
                )
                self.__write_config_to_file(
                    response, ConfigType.BIRTH_CERTIFICATE.value
                )
                return "OK"
        except KeyError:
            self.__comms_controller.logger.error(
                "KeyError certificates field in config"
            )

        return "FAIL"

    def __handle_received_config(
        self, response: requests.Response, config: ConfigType
    ) -> str:
        """
        Handle received config
        :param response: HTTP response
        :return: -
        """

        self.__comms_controller.logger.debug(
            f"HTTP Request Response: {response.text.strip()} {str(response.status_code).strip()}"
        )

        data = json.loads(response.text)

        if (
            self.__mesh_conf_request_processed
            and config.value == ConfigType.DEBUG_CONFIG.value
        ):
            self.__previous_debug_config = response.text.strip()

            # save to file to use in fmo
            try:
                with open(
                    "/opt/debug_config.json", "w", encoding="utf-8"
                ) as debug_conf_json:
                    debug_conf_json.write(self.__previous_debug_config)
                    self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
                return "OK"
            except:
                self.__comms_controller.logger.debug(
                    "cannot store /opt/debug_config.json"
                )
                self.__debug_config_interval = self.FAIL_POLLING_TIME_SECONDS
                return "FAIL"
        else:
            # radio configuration actions
            if config.value == ConfigType.MESH_CONFIG.value:
                ret = self.__action_radio_configuration(response)
                return ret

            # feature.yaml actions
            if config.value == ConfigType.FEATURES.value:
                ret = self.__action_feature_yaml(response)
                return ret

            # cert/keys actions
            if config.value == ConfigType.BIRTH_CERTIFICATE.value:
                ret = self.__action_cert_keys(response)
                return ret

    @staticmethod
    def __read_config_from_file(config: str) -> Optional[str]:
        """
        Read config from file
        :return: config
        """
        try:
            with open(f"/opt/config_{config}.json", "r", encoding="utf-8") as f_handle:
                return f_handle.read().strip()
        except FileNotFoundError:
            return None

    @staticmethod
    def __read_debug_config_from_file() -> Optional[str]:
        """
        Read config from file
        :return: config
        """
        try:
            with open("/opt/debug_config.json", "r", encoding="utf-8") as f:
                return f.read().strip()
        except FileNotFoundError:
            return None

    @staticmethod
    def __write_config_to_file(response: requests.Response, config: str) -> None:
        """
        Write config to file
        :param response: HTTP response
        :return: -
        """
        with open(f"/opt/config_{config}.json", "w", encoding="utf-8") as f_handle:
            f_handle.write(response.text.strip())

    def download_certificate_bundle(self) -> requests.Response:
        """
        Download certificate bundle
        :return: HTTP response
        """

        try:
            self.__comms_controller.logger.debug("Downloading certificate bundle")

            # Prepare the JSON data for the POST request
            data = {
                "device_id": self.__device_id,
                # "certificate_type": None,
            }

            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            # Make the GET request
            url = f"https://{__https_url}/{self.GET_DEVICE_CERTIFICATES}"
            return requests.get(
                url,
                params=data,
                cert=(self.__certificate_file, self.__keyfile),
                verify=self.__ca,
                timeout=2,
            )
        except FileNotFoundError as e:
            self.__comms_controller.logger.error("Certificate file not found: %s", e)
        except requests.RequestException as e:
            self.__comms_controller.logger.error(
                "Post request failed with error: %s", e
            )
        return requests.Response()

    def upload_certificate_bundle(self) -> requests.Response:
        """
        Upload certificate bundle
        :return: HTTP response
        """
        try:
            certificates_dict = {}

            files = glob.glob("/opt/at_birth.tar.bz2*")

            if len(files) == 0:
                self.__comms_controller.logger.debug("No certificates found")
                return requests.Response()

            for file in files:
                with open(file, "rb") as f:
                    data = f.read()
                    base64_data = base64.b64encode(data)
                    certificates_dict[os.path.basename(file)] = base64_data.decode(
                        "utf-8"
                    )

            self.__comms_controller.logger.debug("Uploading certificate bundle")

            # Prepare the JSON data for the POST request
            data = {
                "device_id": self.__device_id,
                "payload": certificates_dict,
                "format": "text",
            }

            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            # Make the POST request
            url = f"https://{__https_url}/{self.PUT_DEVICE_CERTIFICATES}/{ConfigType.BIRTH_CERTIFICATE.value}"
            return requests.post(
                url,
                json=data,
                cert=(self.__certificate_file, self.__keyfile),
                verify=self.__ca,
                timeout=2,
            )
        except FileNotFoundError as e:
            self.__comms_controller.logger.error("Certificate file not found: %s", e)
        except requests.RequestException as e:
            self.__comms_controller.logger.error(
                "Post request failed with error: %s", e
            )
        return requests.Response()

    @staticmethod
    def __cleanup_default_batman():
        # Create an IPRoute object
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name

        interfaces_to_cleanup = ["bat0"]

        for interface in interfaces_to_cleanup:
            if interface in [link.get_attr("IFLA_IFNAME") for link in ip.get_links()]:
                # If the interface exists, set it down, and then delete it
                ip.link(
                    "set",
                    index=ip.link_lookup(ifname=interface)[0],
                    state="down",
                )
                ip.link("delete", index=ip.link_lookup(ifname=interface)[0])

        # Close the IPRoute object
        ip.close()

    def __remove_all_interfaces_from_bridge(self, bridge_name):
        # Create an IPRoute object
        ip = IPRoute()

        try:
            # Get the index of the bridge
            bridge_index = ip.link_lookup(ifname=bridge_name)[0]

            # Get the indices of all interfaces currently in the bridge
            interfaces_indices = [
                link["index"] for link in ip.get_links(master=bridge_index)
            ]

            # Remove each interface from the bridge
            for interface_index in interfaces_indices:
                ip.link(
                    "set", index=interface_index, master=0
                )  # Set the master to 0 (no bridge)

        except Exception as e:
            self.__comms_controller.logger.debug(
                "Error removing interfaces from bridge %s!",
                e,
            )

        finally:
            # Close the IPRoute object
            ip.close()

    def __add_interface_to_bridge(self, bridge_name, interface_to_add):
        # Create an IPRoute object
        ip = IPRoute()

        try:
            # Get the index of the bridge
            bridge_index = ip.link_lookup(ifname=bridge_name)[0]

            # Get the index of the interface to add
            interface_index = ip.link_lookup(ifname=interface_to_add)[0]

            # Add the interface to the bridge
            ip.link("set", index=interface_index, master=bridge_index)

        except Exception as e:
            self.__comms_controller.logger.debug(
                "Error adding interface to bridge %s!",
                e,
            )

        finally:
            # Close the IPRoute object
            ip.close()

    @staticmethod
    def __delete_macsec_links():
        # Define the command as a list of arguments
        command_macsec = ["ip", "macsec", "show"]
        command_grep = ["grep", ": protect on validate"]
        command_awk1 = ["awk", "-F:", "{print $2}"]
        command_awk2 = ["awk", "{print $1}"]
        command_xargs = ["xargs", "-I", "{}", "ip", "link", "delete", "{}"]

        # Run the commands using subprocess and connect their pipes
        proc_macsec = subprocess.Popen(command_macsec, stdout=subprocess.PIPE)
        proc_grep = subprocess.Popen(
            command_grep, stdin=proc_macsec.stdout, stdout=subprocess.PIPE
        )
        proc_awk1 = subprocess.Popen(
            command_awk1, stdin=proc_grep.stdout, stdout=subprocess.PIPE
        )
        proc_awk2 = subprocess.Popen(
            command_awk2, stdin=proc_awk1.stdout, stdout=subprocess.PIPE
        )
        proc_xargs = subprocess.Popen(command_xargs, stdin=proc_awk2.stdout)

        # Wait for the xargs process to finish
        proc_xargs.wait()

    def __shutdown_interface(self, interface_name):
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name
        try:
            index = ip.link_lookup(ifname=interface_name)[0]
            ip.link("set", index=index, state="down")
        except IndexError:
            self.__comms_controller.logger.debug(
                "Not able to shutdown interface %s! Interface not found!",
                interface_name,
            )
        finally:
            ip.close()

    def __destroy_batman_interface(self, mesh_interface):
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name
        try:
            ip.link("delete", ifname=mesh_interface)
        except Exception as e:
            self.__comms_controller.logger.debug(
                f"Error: Unable to destroy interface {mesh_interface}."
            )
            self.__comms_controller.logger.debug(f"Exception: {str(e)}")
        finally:
            ip.close()

    def __shutdown_and_delete_bridge(self, bridge_name):
        ip = IPRoute()
        try:
            index = ip.link_lookup(ifname=bridge_name)[0]
            ip.link("set", index=index, state="down")
            ip.link("delete", index=index)
        except IndexError:
            self.__comms_controller.logger.debug(
                "Not able to delete bridge %s! Bridge not found!", bridge_name
            )
        finally:
            ip.close()

    def stop_cbma(self):
        try:
            self.__comms_controller.logger.debug("Stopping CBMA...")
            # Set shutdown event to signal CBMA threads to stop
            self.__cbma_shutdown_event.set()
            # Waith for threads to stop
            for thread_name, thread in self.__cbma_threads.items():
                try:
                    if thread.is_alive():
                        thread.join()
                except Exception as e:
                    self.__comms_controller.logger.error(f"Thread join error: {e}")

            self.__comms_controller.logger.debug(
                "CBMA threads stopped, continue cleanup..."
            )
            self.__delete_macsec_links()
            self.__shutdown_interface("bat0")
            self.__shutdown_interface("bat1")
            self.__destroy_batman_interface("bat0")
            self.__destroy_batman_interface("bat1")
            self.__shutdown_and_delete_bridge("br-upper")
            self.__comms_controller.logger.debug("CBMA cleanup finished")

        except Exception as e:
            self.__comms_controller.logger.error(f"Stop CBMA error: {e}")

    def __has_certificate(self, mac) -> bool:
        if not path.exists(f"{self.__cbm_certs_path}/MAC/{mac}.crt"):
            return False

        if not path.exists(f"{self.__cbm_certs_path}/private.key"):
            return False

        return True

    def __is_cbma_feature_enabled(self) -> bool:
        """
        Check if CBMA feature is enabled
        :return: True if enabled, False otherwise
        """
        with open(self.YAML_FILE, "r", encoding="utf-8") as stream:
            try:
                features = yaml.safe_load(stream)
                return bool(features["CBMA"])
            except (yaml.YAMLError, KeyError, FileNotFoundError) as exc:
                self.__comms_controller.logger.error(
                    f"CBMA disabled as feature configuration not found: {exc}"
                )
            return False

    def __setup_cbma(self):
        """
        TODO: This method contains currently proof of content type of functionality
        that mimics cbma example code functionality to enable very basic testing.

        It is already known that next cbma PR is such that we should use multiprocessing
        instead of threading. br-lan/batman control is also changing in next cbma version
        and they should be handled in this module instead.

        """

        # These would tell current settings
        print("mesh_if:", self.__comms_controller.settings.mesh_vif)
        print("radio index:", self.__comms_controller.settings.radio_index)

        # Cleanup and delete bat0
        self.__cleanup_default_batman()
        # Cleanup default bridge
        self.__remove_all_interfaces_from_bridge("br-lan")

        # Fixme: Add usb0 interface to br-lan.
        # This is temporary to allow testing of MDM server via
        # ethernet over usb.
        self.__add_interface_to_bridge("br-lan", "usb0")

        wlan_interfaces = ["wlp1s0", "wlp2s0", "wlp3s0", "halow1"]
        for interface in self.__interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in wlan_interfaces):
                if not self.__has_certificate(interface.mac_address):
                    continue
                try:
                    index = self.__comms_controller.settings.mesh_vif.index(
                        interface_name
                    )
                except ValueError:
                    continue
                thread = threading.Thread(
                    target=setup_cbma.cbma,
                    args=(
                        "lower",
                        interface_name,
                        15001,
                        "bat0",
                        self.__cbm_certs_path,
                        self.__cbma_ca_cert_path,
                        "off",
                        f"/var/run/wpa_supplicant_id{index}/{interface_name}",
                        self.__cbma_shutdown_event,
                    ),
                )
                self.__cbma_threads[interface_name] = thread
                thread.start()

        for interface in self.__interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in ["eth"]) or (
                "lan" in interface_name
                and not interface_name.startswith("br-lan")
                and not interface_name.startswith("wlan")
            ):
                if not self.__has_certificate(interface.mac_address):
                    continue
                thread = threading.Thread(
                    target=setup_cbma.cbma,
                    args=(
                        "lower",
                        interface_name,
                        15001,
                        "bat0",
                        self.__cbm_certs_path,
                        self.__cbma_ca_cert_path,
                        "off",
                        None,
                        self.__cbma_shutdown_event,
                    ),
                )
                self.__cbma_threads[interface_name] = thread
                thread.start()

        thread = threading.Thread(
            target=setup_cbma.cbma,
            args=(
                "upper",
                "bat0",
                15002,
                "bat1",
                self.__cbm_certs_path,
                self.__cbma_ca_cert_path,
                "off",
                None,
                self.__cbma_shutdown_event,
            ),
        )
        self.__cbma_threads[interface_name] = thread
        thread.start()

        # TODO: Perhaps we need to maintain
        # list of interface specific statuses?
        self.__cbma_set_up = True

    def __cbma_thread_alive_check(self):
        # Check if any of launched cbma threads have terminated
        terminated_threads = []
        for thread_name, thread in self.__cbma_threads.items():
            if not thread.is_alive():
                terminated_threads.append(thread_name)

        # Remove terminated threads from the dictionary
        for thread_name in terminated_threads:
            del self.__cbma_threads[thread_name]

    async def __loop_run_executor(self, executor, config: ConfigType) -> None:
        """
        Loop run executor
        :param executor: executor
        """
        # This function makes a synchronous HTTP request using ThreadPoolExecutor
        https_loop = asyncio.get_event_loop()
        response = await https_loop.run_in_executor(
            executor, self.__http_get_device_config, config
        )

        self.__comms_controller.logger.debug(
            "HTTP Request status: %s, config: %s", str(response.status_code), config
        )
        if self.__mesh_conf_request_processed:
            if (
                response.status_code == 200
                and self.__previous_debug_config != response.text.strip()
            ):
                self.__handle_received_config(response, ConfigType.DEBUG_CONFIG)
                self.__mesh_conf_request_processed = False
            elif (
                response.status_code == 200
                and self.__previous_debug_config == response.text.strip()
            ):
                self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
                self.__mesh_conf_request_processed = False
            elif response.text.strip() == "" or response.status_code != 200:
                self.__debug_config_interval = self.FAIL_POLLING_TIME_SECONDS
                if response.status_code == 405:
                    self.__comms_controller.logger.debug(
                        "MDM Server has no support for debug mode"
                    )
                    self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
                    self.__mesh_conf_request_processed = False
        else:
            if response.status_code == 200:
                ret = self.__handle_received_config(response, config)
                self.__comms_controller.logger.debug("config: %s, ret: %s", config, ret)
                if ret == "OK":
                    self.__status[config.value] = "OK"
                if config.value == ConfigType.MESH_CONFIG.value and ret == "OK":
                    self.__mesh_conf_request_processed = True
            elif response.status_code != 200:
                self.__status[config.value] = "FAIL"

            # if all statuses are OK, then we can start the OK polling
            if all(value == "OK" for value in self.__status.values()):
                self.__interval = self.OK_POLLING_TIME_SECONDS
            else:
                self.__interval = self.FAIL_POLLING_TIME_SECONDS

            self.__comms_controller.logger.debug("status: %s", self.__status)


async def main_mdm(keyfile=None, certfile=None, ca_file=None, interface=None) -> None:
    """
    main
    param: keyfile: keyfile
    param: certfile: certfile
    param: ca_file: ca_file
    return: -
    """
    cc = CommsController()

    if keyfile is None or certfile is None or ca_file is None:
        cc.logger.debug("MDM: Closing as no certificates provided")
        return

    # if system clock is reset, set it to the certificate creation time
    if datetime.now() < datetime(2023, 9, 1):
        # Read the certificate file
        with open(certfile, "rb") as file:
            cert_data = file.read()

        # Load the certificate
        cert = OpenSSL.crypto.load_certificate(OpenSSL.crypto.FILETYPE_PEM, cert_data)

        # Get the 'notBefore' time
        creation_time = cert.get_notBefore()

        # The time is in bytes, so decode it to a string
        creation_time = creation_time.decode("utf-8")

        new_time_str = datetime.strptime(creation_time, "%Y%m%d%H%M%SZ")
        # Format the datetime object for the 'date' command
        # Format: MMDDhhmmYYYY.ss
        time_formatted = new_time_str.strftime("%Y-%m-%d %H:%M:%S")
        subprocess.run(["date", "-s", time_formatted], check=True)
        subprocess.run(["hwclock", "--systohc"], check=True)

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()

    def signal_handler():
        cc.logger.debug("Disconnecting MDM agent...")
        mdm.mdm_service_available = False
        mdm.running = False
        mdm.executor.shutdown()
        mdm.stop_cbma()
        if mdm.service_monitor.running:
            mdm.stop_service_discovery()
        if mdm.interface_monitor.running:
            mdm.interface_monitor.running = False
            mdm.thread_if_mon.join()
            cc.logger.debug("Interface monitor stopped")
        asyncio.create_task(stop())

    for sig in ("SIGINT", "SIGTERM"):
        asyncio.get_running_loop().add_signal_handler(
            getattr(signal, sig), signal_handler
        )

    cc.logger.debug("MDM: comms_nats_controller Listening for requests")

    # separate instances needed for FMO/MDM

    mdm = MdmAgent(cc, keyfile, certfile, ca_file, interface)
    mdm.start_interface_monitor()
    await asyncio.gather(mdm.execute())


# pylint: disable=too-many-arguments, too-many-locals, too-many-statements
async def main_fmo(server, port, keyfile=None, certfile=None, interval=1000) -> None:
    """
    main fmo
    param: server: server
    param: port: port
    param: keyfile: keyfile
    param: certfile: certfile
    param: interval: interval
    return: -
    """
    cc = CommsController(interval)

    nats_client = nats()
    status, _, identity_dict = cc.command.get_identity()

    if status == "OK":
        identity = identity_dict["identity"]
        cc.logger.debug("Identity: %s", identity)
    else:
        cc.logger.error("Failed to get identity!")
        return

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()

    def signal_handler():
        if nats_client.is_closed:
            return
        cc.logger.debug("Disconnecting...")
        asyncio.create_task(nats_client.close())
        asyncio.create_task(stop())

    for sig in ("SIGINT", "SIGTERM"):
        asyncio.get_running_loop().add_signal_handler(
            getattr(signal, sig), signal_handler
        )

    async def disconnected_cb():
        cc.logger.debug("Got disconnected...")

    async def reconnected_cb():
        cc.logger.debug("Got reconnected...")

    # Create SSL context if certfile and keyfile are provided
    ssl_context = None
    if certfile and keyfile:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(certfile=certfile, keyfile=keyfile)

    # Connect to NATS server with TLS enabled if ssl_context is provided
    if ssl_context:
        await nats_client.connect(
            f"tls://{server}:{port}",
            tls=ssl_context,
            reconnected_cb=reconnected_cb,
            disconnected_cb=disconnected_cb,
            max_reconnect_attempts=-1,
        )
    else:
        await nats_client.connect(
            f"nats://{server}:{port}",
            reconnected_cb=reconnected_cb,
            disconnected_cb=disconnected_cb,
            max_reconnect_attempts=-1,
        )

    async def fmo_message_handler(message):
        """
        Message handler for FMO
        """
        # reply = message.reply
        subject = message.subject
        data = message.data.decode()
        cc.logger.debug("Received a message on '%s': %s", subject, data)
        ret, info, resp = "FAIL", "Not supported subject", ""

        if subject == f"comms.settings.{identity}":
            ret, info = cc.settings.handle_mesh_settings(data)
        elif subject == f"comms.channel_change.{identity}":
            ret, info, _index = cc.settings.handle_mesh_settings_channel_change(data)
            if ret == "TRIGGER":
                cmd = json.dumps(
                    {"api_version": 1, "cmd": "APPLY", "radio_index": f"{_index}"}
                )
                ret, info, resp = cc.command.handle_command(cmd, cc)
        elif subject in (f"comms.command.{identity}", "comms.identity"):
            # this file was saved by mdm
            try:
                with open(
                    "/opt/debug_config.json", "r", encoding="utf-8"
                ) as debug_conf_json:
                    debug_conf_data = debug_conf_json.read().strip()
                    debug_conf_data_json = json.loads(debug_conf_data)
                    debug_mode = debug_conf_data_json["payload"]["debug_conf"][0][
                        "debug_mode"
                    ]
                    cc.debug_mode_enabled = True if debug_mode == "enabled" else False
            except:
                pass
            ret, info, resp = cc.command.handle_command(data, cc)
        elif subject == f"comms.status.{identity}":
            ret, info = "OK", "Returning current status"

        # Update status info
        _ = [item.refresh_status() for item in cc.c_status]

        response = {
            "status": ret,
            "info": info,
            "mesh_status": [item.mesh_status for item in cc.c_status],
            "mesh_cfg_status": [item.mesh_cfg_status for item in cc.c_status],
            "visualisation_active": [
                item.is_visualisation_active for item in cc.c_status
            ],
            "mesh_radio_on": [item.is_mesh_radio_on for item in cc.c_status],
            "ap_radio_on": [item.is_ap_radio_on for item in cc.c_status],
            "security_status": [item.security_status for item in cc.c_status],
        }

        if resp != "":
            response["data"] = resp

        cc.logger.debug("Sending response: %s", str(response)[:1000])
        await message.respond(json.dumps(response).encode("utf-8"))

    await nats_client.subscribe(f"comms.settings.{identity}", cb=fmo_message_handler)
    await nats_client.subscribe(
        f"comms.channel_change.{identity}", cb=fmo_message_handler
    )
    await nats_client.subscribe(f"comms.status.{identity}", cb=fmo_message_handler)
    await nats_client.subscribe("comms.identity", cb=fmo_message_handler)
    await nats_client.subscribe(f"comms.command.{identity}", cb=fmo_message_handler)

    cc.logger.debug("FMO comms_nats_controller Listening for requests")

    while True:
        await asyncio.sleep(float(cc.interval) / 1000.0)
        try:
            if cc.telemetry.visualisation_enabled:
                msg = cc.telemetry.mesh_visual()
                cc.logger.debug(f"Publishing comms.visual.{identity}: %s", msg)
                await nats_client.publish(f"comms.visual.{identity}", msg.encode())
        except Exception as e:
            cc.logger.error("Error:", e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mesh Settings")
    parser.add_argument("-s", "--server", help="Server IP", required=False)
    parser.add_argument("-p", "--port", help="Server port", required=False)
    parser.add_argument("-k", "--keyfile", help="TLS keyfile", required=False)
    parser.add_argument("-c", "--certfile", help="TLS certfile", required=False)
    parser.add_argument("-r", "--ca", help="ca certfile", required=False)
    parser.add_argument("-i", "--interface", help="e.g. br-lan or bat0", required=False)
    parser.add_argument(
        "-a", "--agent", help="mdm or fmo", required=False, default="fmo"
    )

    args = parser.parse_args()
    loop = asyncio.new_event_loop()

    try:
        if args.agent == "mdm":
            loop.run_until_complete(
                main_mdm(args.keyfile, args.certfile, args.ca, args.interface)
            )
        else:
            loop.run_until_complete(
                main_fmo(args.server, args.port, args.keyfile, args.certfile)
            )
    except Exception as e:
        pass
    loop.close()
