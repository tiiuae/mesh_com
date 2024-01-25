"""
Comms NATS controller
"""
from os import sys, path, umask

if __name__ == "__main__" and __package__ is None:
    PARENT_DIR = path.dirname(path.dirname(path.abspath(__file__)))
    MS20_FEATURES_PATH = path.join(PARENT_DIR, "2_0/features")
    sys.path.append(MS20_FEATURES_PATH)
    # Make it possible to run this module also in other folder (e.g. /opt/nats):
    sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features")
    sys.path.append(
        "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/cbma"
    )

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
import tarfile
import shutil
import time

from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Process
from datetime import datetime
from typing import Optional
from typing import List, Dict
from copy import deepcopy

import OpenSSL.crypto
import requests
import yaml
from pyroute2 import IPRoute  # type: ignore[import-not-found, import-untyped]
from cbma import setup_cbma  # type: ignore[import-not-found]
from nats.aio.client import Client as nats

from cbma.tools.utils import batman  # type: ignore[import-not-found]

from src import batadvvis
from src import batstat
from src import comms_command
from src import comms_if_monitor
from src import comms_service_discovery
from src import comms_settings
from src import comms_status
from src.constants import Constants, ConfigType, StatusType

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
            Constants.FAIL_POLLING_TIME_SECONDS.value
        )  # possible to update from MDM server?
        self.__debug_config_interval: int = (
            Constants.FAIL_POLLING_TIME_SECONDS.value
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
        self.__lock = threading.Lock()

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

        self.__cbma_certs_downloaded = "/opt/mdm/certs"
        self.__cbma_certs_path = "/opt/crypto/ecdsa/birth/filebased"
        self.__cbma_ca_cert_path = "/opt/mspki/ecdsa/certificate_chain.crt"
        self.__upper_cbma_certs_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value + "/crypto/ecdsa/birth/filebased"
        )
        self.__upper_cbma_ca_cert_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value
            + "/mspki/ecdsa/certificate_chain.crt"
        )
        self.__lower_cbma_processes: Dict[str, Process] = {}
        self.__upper_cbma_processes: Dict[str, Process] = {}
        self.__lower_cbma_interfaces: List[Interface] = []
        self.__upper_cbma_interfaces: List[Interface] = []
        self.running = False

        try:
            if not os.listdir(self.__cbma_certs_downloaded):
                self.__certs_downloaded = False
                self.__comms_controller.logger.debug("Certificate download needed")
            else:
                self.__certs_downloaded = True
                self.__comms_controller.logger.debug("No need to download certificates")
        except FileNotFoundError:
            os.makedirs(self.__cbma_certs_downloaded, exist_ok=True)
            self.__certs_downloaded = False
            self.__comms_controller.logger.debug("Certificate download needed")

        self.__status: dict = {
            StatusType.DOWNLOAD_MESH_CONFIG.value: "FAIL",
            StatusType.DOWNLOAD_FEATURES.value: "FAIL",
            StatusType.DOWNLOAD_CERTIFICATES.value: "OK"
            if self.__certs_downloaded
            else "FAIL",
            StatusType.UPLOAD_CERTIFICATES.value: "OK"
            if self.__certs_uploaded
            else "FAIL",
        }

        self.__config_status_mapping = {
            ConfigType.MESH_CONFIG: StatusType.DOWNLOAD_MESH_CONFIG,
            ConfigType.FEATURES: StatusType.DOWNLOAD_FEATURES,
            ConfigType.BIRTH_CERTIFICATE: StatusType.DOWNLOAD_CERTIFICATES,
            ConfigType.LOWER_CERTIFICATE: StatusType.DOWNLOAD_CERTIFICATES,
            ConfigType.UPPER_CERTIFICATE: StatusType.DOWNLOAD_CERTIFICATES,
        }

        try:
            with open("/opt/identity", "r", encoding="utf-8") as f:  # todo
                self.__device_id = f.read().strip()
        except FileNotFoundError:
            self.__device_id = "default"

    def mdm_server_address_cb(self, address: str, status: bool, **kwargs) -> None:
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
        with self.__lock:
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
            self.__comms_controller.logger.debug(
                "status: %s, mdm_available: %s",
                self.__status,
                self.mdm_service_available,
            )

            if (
                self.__status[StatusType.UPLOAD_CERTIFICATES.value] == "FAIL"
                and self.mdm_service_available
            ):
                resp = self.upload_certificate_bundle()
                self.__comms_controller.logger.debug(
                    "Certificate upload response: %s", resp
                )
                if resp.status_code == 200:
                    self.__status[StatusType.UPLOAD_CERTIFICATES.value] = "OK"
                    with open("/opt/certs_uploaded", "w", encoding="utf-8") as f:
                        f.write("certs uploaded")
                else:
                    self.__comms_controller.logger.debug("Certificate upload failed")
            elif (
                self.__status[StatusType.DOWNLOAD_CERTIFICATES.value] == "FAIL"
                and self.__status[StatusType.UPLOAD_CERTIFICATES.value] == "OK"
                and self.mdm_service_available
            ):
                for certificate_type in [
                    # ConfigType.BIRTH_CERTIFICATE,
                    ConfigType.UPPER_CERTIFICATE,
                    # ConfigType.LOWER_CERTIFICATE,
                ]:
                    resp = self.download_certificate_bundle(certificate_type.value)
                    self.__comms_controller.logger.debug(
                        "Certificate download response: %s", resp
                    )
                    if resp.status_code == 200:
                        self.__status[
                            StatusType.DOWNLOAD_CERTIFICATES.value
                        ] = self.__action_certificates(resp, certificate_type.value)
                        if (
                            self.__status[StatusType.DOWNLOAD_CERTIFICATES.value]
                            == "OK"
                        ):
                            # Restart CBMA with new certificates
                            self.stop_cbma()
                            self.setup_cbma()
                        if (
                            self.__status[StatusType.DOWNLOAD_CERTIFICATES.value]
                            == "FAIL"
                        ):
                            self.__comms_controller.logger.debug(
                                "Certificates action failed"
                            )
                    else:
                        self.__comms_controller.logger.debug(
                            "Certificate download failed: %s", certificate_type.value
                        )
            elif self.mdm_service_available:
                await self.__loop_run_executor(self.executor, ConfigType.FEATURES)
                await self.__loop_run_executor(self.executor, ConfigType.MESH_CONFIG)
                if self.__mesh_conf_request_processed:
                    await self.__loop_run_executor(
                        self.executor, ConfigType.DEBUG_CONFIG
                    )
            if self.__is_cbma_feature_enabled() and not self.__cbma_set_up:
                self.setup_cbma()
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
                f"https://{__https_url}/{Constants.GET_DEVICE_CONFIG.value}/{config.value}",
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

    def __action_certificates(
        self, response: requests.Response, certificate_type: str
    ) -> str:
        """
        Action certificates
        :param response: HTTP response
        :return: -
        """
        try:
            if certificate_type == ConfigType.BIRTH_CERTIFICATE.value:
                cert_path = Constants.DOWNLOADED_CBMA_BIRTHCERTS_PATH.value
            elif certificate_type == ConfigType.UPPER_CERTIFICATE.value:
                cert_path = Constants.DOWNLOADED_CBMA_UPPER_PATH.value
            elif certificate_type == ConfigType.LOWER_CERTIFICATE.value:
                cert_path = Constants.DOWNLOADED_CBMA_LOWER_PATH.value
            else:
                self.__comms_controller.logger.error("Unknown certificate type")
                return "FAIL"

            payload_dict = json.loads(response.text)["payload"]
            self.__comms_controller.logger.debug(
                "payload_dict[role]: %s", payload_dict["role"]
            )
            self.__comms_controller.logger.debug(
                "payload_dict[group]: %s", payload_dict["group"]
            )

            certificates_dict = json.loads(response.text)["payload"]["certificates"]

            for certificate_name, certificate in certificates_dict.items():
                certificate_name = os.path.basename(certificate_name)
                if str(certificate_name).endswith(".tar.bz2"):
                    with open(
                        f"{self.__cbma_certs_downloaded}/{certificate_name}", "wb"
                    ) as cert_file:
                        cert_file.write(base64.b64decode(certificate))
                        self.__comms_controller.logger.debug(
                            "Certificate %s saved", certificate_name
                        )
                        try:
                            old_umask = umask(0o377)
                            with tarfile.open(
                                f"{self.__cbma_certs_downloaded}/{certificate_name}",
                                "r:bz2",
                            ) as tar:
                                for member in tar.getmembers():
                                    # Construct the full path where the file would be extracted
                                    file_path = os.path.join(cert_path, member.name)

                                    # Check if the file already exists, overwrite check
                                    if os.path.exists(file_path):
                                        raise FileExistsError(
                                            f"File {file_path} already exists"
                                        )

                                self.__comms_controller.logger.debug(
                                    "Extracting files (not overwriting)"
                                )
                                # todo: add filter="data" once python is updated
                                tar.extractall(path=cert_path)
                                self.__comms_controller.logger.debug(
                                    "Certificate %s extracted to %s",
                                    certificate_name,
                                    cert_path,
                                )
                            umask(old_umask)
                        except (
                            tarfile.TarError,
                            FileNotFoundError,
                            FileExistsError,
                        ) as exc:
                            shutil.rmtree(cert_path)
                            shutil.rmtree(self.__cbma_certs_downloaded)
                            self.__comms_controller.logger.error(
                                "Failed to extract certificate: %s", exc
                            )
                            return "FAIL"
                        finally:
                            umask(old_umask)
                            os.makedirs(cert_path, exist_ok=True)
                            os.makedirs(self.__cbma_certs_downloaded, exist_ok=True)

                elif str(certificate_name).endswith("-sig"):
                    old_umask = umask(0o377)
                    with open(
                        f"{self.__cbma_certs_downloaded}/{certificate_name}", "wb"
                    ) as sig_file:
                        sig_file.write(base64.b64decode(certificate))
                        self.__comms_controller.logger.debug(
                            "Sig %s saved", certificate_name
                        )
                    umask(old_umask)
            self.__comms_controller.logger.debug("Downloaded bundle saved")

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
                    Constants.YAML_FILE.value,
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

    def __handle_received_config(
        self, response: requests.Response, config: ConfigType
    ) -> (str, str):
        """
        Handle received config
        :param response: HTTP response
        :return: -
        """

        self.__comms_controller.logger.debug(
            f"HTTP Request Response: {response.text.strip()} {str(response.status_code).strip()}"
        )

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
                    self.__debug_config_interval = (
                        Constants.OK_POLLING_TIME_SECONDS.value
                    )
                return "OK"
            except:
                self.__comms_controller.logger.debug(
                    "cannot store /opt/debug_config.json"
                )
                self.__debug_config_interval = Constants.FAIL_POLLING_TIME_SECONDS.value
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

    def download_certificate_bundle(
        self, certificate_type: str = ""
    ) -> requests.Response:
        """
        Download certificate bundle
        :return: HTTP response
        """

        try:
            self.__comms_controller.logger.debug("Downloading certificate bundle")

            # Prepare the JSON data for the POST request
            data = {
                "device_id": self.__device_id,
                "certificate_type": certificate_type,
            }

            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            # Make the GET request
            url = f"https://{__https_url}/{Constants.GET_DEVICE_CERTIFICATES.value}"
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
            url = f"https://{__https_url}/{Constants.PUT_DEVICE_CERTIFICATES.value}/{ConfigType.BIRTH_CERTIFICATE.value}"
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
            bridge_indices = ip.link_lookup(ifname=bridge_name)
            if bridge_indices:
                bridge_index = bridge_indices[0]
            else:
                self.__comms_controller.logger.debug(
                    "Cannot remove interfaces from %s as it was not found!",
                    bridge_name,
                )
                return
            # Get the indices of all interfaces currently in the bridge
            interfaces_indices = [
                link["index"] for link in ip.get_links(master=bridge_index)
            ]

            if not interfaces_indices:
                self.__comms_controller.logger.debug(
                    "No interfaces to remove from the bridge %s!",
                    bridge_name,
                )
                return

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
            bridge_indices = ip.link_lookup(ifname=bridge_name)
            if bridge_indices:
                bridge_index = bridge_indices[0]
            else:
                self.__comms_controller.logger.debug(
                    "Cannot remove interfaces from %s as it was not found!",
                    bridge_name,
                )
                return

            # Get the index of the interface to add
            interface_indices = ip.link_lookup(ifname=interface_to_add)

            if not interface_indices:
                self.__comms_controller.logger.debug(
                    "Cannot add interfaces %s to bridge %s!",
                    interface_to_add,
                    bridge_name,
                )
                return

            interface_index = interface_indices[0]
            # Add the interface to the bridge
            ip.link("set", index=interface_index, master=bridge_index)

        except Exception as e:
            self.__comms_controller.logger.debug(
                "Error adding interface %s to bridge %s! Error: %s",
                interface_to_add,
                bridge_name,
                e,
            )

        finally:
            # Close the IPRoute object
            ip.close()

    @staticmethod
    def __delete_ebtables_rules():
        command_ebtables = ["ebtables", "-t", "nat", "-L", "OUTPUT"]
        command_sed = ["sed", "-n", "/OUTPUT/,/^$/{/^--/p}"]
        command_xargs = ["xargs", "ebtables", "-t", "nat", "-D", "OUTPUT"]

        proc_ebtables = subprocess.Popen(command_ebtables, stdout=subprocess.PIPE)
        proc_sed = subprocess.Popen(
            command_sed, stdin=proc_ebtables.stdout, stdout=subprocess.PIPE
        )
        proc_xargs = subprocess.Popen(command_xargs, stdin=proc_sed.stdout)
        proc_xargs.wait()

        subprocess.run(
            [
                "ebtables",
                "--delete",
                "FORWARD",
                "--logical-in",
                "br-upper",
                "--jump",
                "ACCEPT",
            ]
        )
        subprocess.run(
            [
                "ebtables",
                "--delete",
                "FORWARD",
                "--logical-out",
                "br-upper",
                "--jump",
                "ACCEPT",
            ]
        )

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

    def __cleanup_cbma(self):
        self.__shutdown_interface("bat0")
        self.__shutdown_interface("bat1")
        self.__delete_ebtables_rules()
        self.__delete_macsec_links()
        self.__destroy_batman_interface("bat0")
        self.__destroy_batman_interface("bat1")
        self.__shutdown_and_delete_bridge("br-upper")

    def stop_cbma(self):
        if not self.__cbma_set_up:
            return
        try:
            self.__comms_controller.logger.debug("Stopping CBMA...")

            for if_name, process in self.__lower_cbma_processes.items():
                try:
                    self.__comms_controller.logger.debug(
                        "Stopping lower CBMA process %s for interface %s",
                        process,
                        if_name,
                    )
                    if process.is_alive():
                        process.terminate()
                except Exception as e:
                    self.__comms_controller.logger.error(
                        f"Terminating lower CBMA process error: {e}"
                    )

            for process in self.__lower_cbma_processes.values():
                try:
                    if process.is_alive():
                        process.join(timeout=1.0)
                        process.kill()
                except Exception as e:
                    self.__comms_controller.logger.error(
                        f"Killing lower CBMA process error: {e}"
                    )

            for if_name, process in self.__upper_cbma_processes.items():
                try:
                    self.__comms_controller.logger.debug(
                        "Stopping upper CBMA process %s for interface %s",
                        process,
                        if_name,
                    )
                    if process.is_alive():
                        process.terminate()
                except Exception as e:
                    self.__comms_controller.logger.error(
                        f"Terminating upper CBMA process error: {e}"
                    )

            for process in self.__upper_cbma_processes.values():
                try:
                    if process.is_alive():
                        process.join(timeout=1.0)
                        process.kill()
                except Exception as e:
                    self.__comms_controller.logger.error(
                        f"Killing upper CBMA process error: {e}"
                    )
            self.__comms_controller.logger.debug(
                "CBMA processes terminated, continue cleanup..."
            )
            self.__cleanup_cbma()
            self.__comms_controller.logger.debug("CBMA cleanup finished")

        except Exception as e:
            self.__comms_controller.logger.error(f"Stop CBMA error: {e}")
        finally:
            self.__cbma_set_up = False

    def __has_certificate(self, cert_path, mac) -> bool:
        certificate_path = f"{cert_path}/MAC/{mac}.crt"

        if not path.exists(certificate_path):
            self.__comms_controller.logger.debug(
                "Certificate not found: %s", certificate_path
            )
            return False

        self.__comms_controller.logger.debug("Certificate found: %s", certificate_path)
        return True

    def __is_cbma_feature_enabled(self) -> bool:
        """
        Check if CBMA feature is enabled
        :return: True if enabled, False otherwise
        """
        with open(Constants.YAML_FILE.value, "r", encoding="utf-8") as stream:
            try:
                features = yaml.safe_load(stream)
                return bool(features["CBMA"])
            except (yaml.YAMLError, KeyError, FileNotFoundError) as exc:
                self.__comms_controller.logger.error(
                    f"CBMA disabled as feature configuration not found: {exc}"
                )
            return False

    def __update_cbma_interface_lists(self):
        # Initially all the interfaces with certificates should use lower CBMA
        self.__lower_cbma_interfaces.clear()
        with self.__lock:
            self.__lower_cbma_interfaces = deepcopy(self.__interfaces)
        self.__upper_cbma_interfaces.clear()

        # Get list of interfaces that doesn't have lower CBMA certificate
        interfaces_without_certificate = []
        for interface in self.__lower_cbma_interfaces:
            if not self.__has_certificate(
                self.__cbma_certs_path, interface.mac_address
            ):
                interfaces_without_certificate.append(interface)

        # Remove interfaces without certificates lower CBMA interface list
        for interface in interfaces_without_certificate:
            self.__comms_controller.logger.debug(
                "Interface %s doesn't have certificate!", interface.interface_name
            )
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        not_allowed_lower_cbma_interfaces = []
        # FIXME: Hard coded red interfaces for testing purposes
        # Eventually MDM server will define interface colors somehow
        red_interface_prefixes = ["eth", "usb", "wlan", "br-lan", "bat", "lan"]
        for interface in self.__lower_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            if any(
                prefix in interface_name
                for prefix in red_interface_prefixes
            ):
                not_allowed_lower_cbma_interfaces.append(interface)

        # Remove not allowed interfaces from lower CBMA interface list
        for interface in not_allowed_lower_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        # FIXME: Hard coded white (upper CBMA) interfaces for testing purposes
        # Eventually MDM server will define interface colors somehow
        white_interfaces = ["bat0", "halow1", "wlp3s0"]
        for interface in self.__interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in white_interfaces):
                # TODO: Reminder to make sure not to add bat1 ever to upper CBMA
                if interface.interface_name == "bat1":
                    continue
                self.__upper_cbma_interfaces.append(interface)

        # Lower and upper CBMA interfaces are mutually exclusive so remove
        # upper CBMA interfaces from lower CBMA interface list
        for interface in self.__upper_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

    def __wait_for_batman_interfaces(self, timeout=3):
        start_time = time.time()
        while True:
            with self.__lock:
                found = any(
                    interface.interface_name == "bat0"
                    for interface in self.__interfaces
                ) and any(
                    interface.interface_name == "bat1"
                    for interface in self.__interfaces
                )
                if found:
                    return True
            elapsed_time = time.time() - start_time
            if elapsed_time >= timeout:
                return False  # Timeout reached

            time.sleep(1)  # Sleep for 1 second before checking again

    def setup_cbma(self):
        # Cleanup CBMA before starting it
        self.__cleanup_cbma()
        # FIXME: bridge name/control in general
        self.__remove_all_interfaces_from_bridge("br-lan")

        # TODO - Shall this be done differently?
        # Create batman interfaces
        batman("bat0")
        batman("bat1")

        self.__wait_for_batman_interfaces()
        self.__update_cbma_interface_lists()
        self.__setup_lower_cbma()
        self.__setup_upper_cbma()

        # Add interfaces to the bridge
        bridge_setting = self.__comms_controller.settings.bridge[0]
        bridge_setting = bridge_setting.strip('"')
        components = bridge_setting.split()
        bridge_name = components[0]
        interface_names = components[1:]

        not_allowed_interfaces_in_bridge = ["bat0"]
        for not_allowed_interface in not_allowed_interfaces_in_bridge:
            if not_allowed_interface in interface_names:
                interface_names.remove(not_allowed_interface)

        # FIXME: Hard coded red interfaces for testing purposes
        # Eventually MDM server will define interface colors
        mandatory_interfaces = ["bat1", "eth0", "eth1", "usb0"]
        for mandatory_interface in mandatory_interfaces:
            if mandatory_interface not in interface_names:
                interface_names.append(mandatory_interface)

        for interface in interface_names:
            self.__add_interface_to_bridge(bridge_name, interface)
        self.__cbma_set_up = True

    def __setup_lower_cbma(self):
        for interface in self.__lower_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            try:
                index = self.__comms_controller.settings.mesh_vif.index(interface_name)
                wpa_supplicant_ctrl_path = (
                    f"/var/run/wpa_supplicant_id{index}/{interface_name}"
                )
            except ValueError:
                wpa_supplicant_ctrl_path = None

            self.__comms_controller.logger.debug(
                "Setup lower CBMA for interface: %s, wpa_supplicant_ctrl_path: %s",
                interface_name,
                wpa_supplicant_ctrl_path,
            )
            process = setup_cbma.cbma(
                "lower",
                interface_name,
                Constants.CBMA_PORT_LOWER.value,
                "bat0",
                self.__cbma_certs_path,
                self.__cbma_ca_cert_path,
                "off",
                wpa_supplicant_ctrl_path,
            )
            self.__lower_cbma_processes[interface_name] = process

    def __setup_upper_cbma(self):
        for interface in self.__upper_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            try:
                index = self.__comms_controller.settings.mesh_vif.index(interface_name)
                wpa_supplicant_ctrl_path = (
                    f"/var/run/wpa_supplicant_id{index}/{interface_name}"
                )
            except ValueError:
                wpa_supplicant_ctrl_path = None

            if self.__has_certificate(
                self.__upper_cbma_certs_path, interface.mac_address
            ):
                self.__comms_controller.logger.debug(
                    "Using upper cbma certificate for interface %s", interface_name
                )
                cbma_certs_path = self.__upper_cbma_certs_path
                cbma_ca_cert_path = self.__upper_cbma_ca_cert_path
            else:
                # TODO: Temporary backup solution to use lower CBMA certificates
                if self.__has_certificate(
                    self.__cbma_certs_path, interface.mac_address
                ):
                    self.__comms_controller.logger.debug(
                        "Using lower cbma certificate as a backup for interface %s",
                        interface_name,
                    )
                    cbma_certs_path = self.__cbma_certs_path
                    cbma_ca_cert_path = self.__cbma_ca_cert_path
                else:
                    self.__comms_controller.logger.debug(
                        "No cbma certificate for interface %s", interface_name
                    )
                    continue

            self.__comms_controller.logger.debug(
                "Setup upper CBMA for interface: %s, wpa_supplicant_ctrl_path: %s",
                interface_name,
                wpa_supplicant_ctrl_path,
            )

            process = setup_cbma.cbma(
                "upper",
                interface_name,
                Constants.CBMA_PORT_UPPER.value,
                "bat1",
                cbma_certs_path,
                cbma_ca_cert_path,
                "off",
                wpa_supplicant_ctrl_path,
            )
            self.__upper_cbma_processes[interface_name] = process

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

        status_type = self.__config_status_mapping.get(config, None)

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
                self.__debug_config_interval = Constants.OK_POLLING_TIME_SECONDS.value
                self.__mesh_conf_request_processed = False
            elif response.text.strip() == "" or response.status_code != 200:
                self.__debug_config_interval = Constants.FAIL_POLLING_TIME_SECONDS.value
                if response.status_code == 405:
                    self.__comms_controller.logger.debug(
                        "MDM Server has no support for debug mode"
                    )
                    self.__debug_config_interval = (
                        Constants.OK_POLLING_TIME_SECONDS.value
                    )
                    self.__mesh_conf_request_processed = False
        else:
            if response.status_code == 200:
                ret = self.__handle_received_config(response, config)
                self.__comms_controller.logger.debug("config: %s, ret: %s", config, ret)
                if ret == "OK":
                    self.__status[status_type] = "OK"
                if config.value == ConfigType.MESH_CONFIG.value and ret == "OK":
                    self.__mesh_conf_request_processed = True
            elif response.status_code != 200:
                self.__status[status_type] = "FAIL"

            # if all statuses are OK, then we can start the OK polling
            if all(value == "OK" for value in self.__status.values()):
                self.__interval = Constants.OK_POLLING_TIME_SECONDS.value
            else:
                self.__interval = Constants.FAIL_POLLING_TIME_SECONDS.value

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
        mdm.interface_monitor.stop()
        mdm.stop_cbma()
        if mdm.service_monitor.running:
            mdm.stop_service_discovery()
        if mdm.thread_if_mon.is_alive():
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

    try:
        results = await asyncio.gather(mdm.execute())
        cc.logger.debug("Results: %s:", results)
    except Exception as e:
        cc.logger.exception("Exception:")
    finally:
        signal_handler()


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

    if args.agent == "mdm":
        loop.run_until_complete(
            main_mdm(args.keyfile, args.certfile, args.ca, args.interface)
        )
    else:
        loop.run_until_complete(
            main_fmo(args.server, args.port, args.keyfile, args.certfile)
        )
    loop.close()
