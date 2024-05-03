"""
MDM agent
"""
from os import umask
import argparse
import asyncio
import json
import logging
import signal
import threading
import os
import glob
import base64
import tarfile
import shutil
import socket
import ssl

from concurrent.futures import ThreadPoolExecutor
from typing import Optional
from typing import List

from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.asymmetric import rsa, ec, ed25519

import requests
import yaml

from src import comms_if_monitor
from src import comms_controller
from src import comms_service_discovery
from src.constants import Constants, ConfigType, StatusType
from src import comms_config_store
from src import cbma_adaptation
from src.interface import Interface


# pylint: enable=too-few-public-methods
class MdmAgent:
    """
    MDM Agent
    """

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        comms_ctrl: comms_controller.CommsController,
        keyfile: str = None,
        certificate_file: str = None,
        ca_file: str = None,
        interface: str = Constants.LOWER_BATMAN.value,
    ):
        """
        Constructor
        """
        self.__config_store = comms_config_store.ConfigStore("/opt/dl_configs_store.yaml")
        self.__previous_config_mesh: Optional[str] = self.__config_store.read(ConfigType.MESH_CONFIG.value)
        self.__previous_config_features: Optional[str] = self.__config_store.read(ConfigType.FEATURES.value)
        self.__previous_config_certificates: Optional[str] = self.__config_store.read(ConfigType.BIRTH_CERTIFICATE.value)
        self.__previous_debug_config: Optional[str] = self.__config_store.read(ConfigType.DEBUG_CONFIG.value)
        self.__mesh_conf_request_processed = False
        self.__comms_ctrl: comms_controller.CommsController = comms_ctrl
        self.logger: logging.Logger = self.__comms_ctrl.logger.getChild("mdm_agent")
        self.__interval: int = Constants.FAIL_POLLING_TIME_SECONDS.value  # TODO: possible to update from MDM server?
        self.__debug_config_interval: int = Constants.FAIL_POLLING_TIME_SECONDS.value  # TODO: possible to update from MDM server?

        self.__keyfile: str = keyfile
        self.__certificate_file: str = certificate_file
        self.__ca: str = ca_file
        self.__config_version: int = 0
        self.mdm_service_available: bool = False
        self.__if_to_monitor: str = interface
        self.__interfaces: List[Interface] = []
        self.executor = ThreadPoolExecutor(1)
        self.__lock = threading.Lock()
        self.cbma_ctrl = cbma_adaptation.CBMAAdaptation(
            self.__comms_ctrl,
            self.logger,
            self.__lock
        )
        self.__url: str = "defaultmdm.local:5000"  # mDNS callback updates this one
        self.__fallback_url: str = f"[{Constants.IPV6_WHITE_PREFIX.value}::1]:5000"

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
                self.logger.debug("No need to upload certificates")
        except FileNotFoundError:
            self.__certs_uploaded = False
            self.logger.debug("Certificate upload needed")

        self.__cbma_certs_downloaded = "/opt/mdm/certs"
        self.running = False

        try:
            self.__certs_downloaded = bool(os.listdir(self.__cbma_certs_downloaded))
        except FileNotFoundError:
            self.__certs_downloaded = False
            os.makedirs(self.__cbma_certs_downloaded, exist_ok=True)
        finally:
            self.logger.debug(
                f"Certificate download {'not ' if self.__certs_downloaded else ''}needed"
            )

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

    def mdm_server_address_cb(self, address: str, status: bool, **_) -> None:
        """
        Callback for MDM server address
        :param address: MDM server address
        :param status: MDM connection status
        :return: -
        """
        with self.__lock:
            self.__url = address
            self.mdm_service_available = status

            if self.mdm_service_available:
                self.__get_server_cert_type()

    def __get_server_cert_type(self):
        parts = self.__url.split(":")
        if len(parts) == 2:
            hostname = parts[0]
            port = int(parts[1])

            try:
                # Create a socket connection to the server
                sock = socket.create_connection((hostname, port), timeout=20)

                # Create an SSL context and load the certificate, key, and CA files
                context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
                context.load_verify_locations(cafile=self.__ca)

                with context.wrap_socket(sock, server_hostname=hostname) as ssock:
                    peer_cert = ssock.getpeercert(binary_form=True)
                    cert = x509.load_der_x509_certificate(peer_cert, default_backend())
                    # Extract key type and signature algorithm
                    public_key = cert.public_key()

                    # Determine key type
                    if isinstance(public_key, rsa.RSAPublicKey):
                        key_type = "rsa"
                    elif isinstance(public_key, ec.EllipticCurvePublicKey):
                        key_type = "ecdsa"
                    elif isinstance(public_key, ed25519.Ed25519PublicKey):
                        key_type = "eddsa"
                    else:
                        key_type = "unknown"

                    self.logger.debug("Server's public key type: %s", key_type)

                    if not key_type == "unknown":
                        certificate_files = glob.glob(
                            f"/opt/crypto/{key_type}/birth/filebased/DNS/*.local.crt"
                        )
                        if certificate_files:
                            self.__certificate_file = certificate_files[0]
                        else:
                            self.logger.error(
                                "No certificate file found for %s", key_type
                            )
                            return
                        self.__keyfile = (
                            f"/opt/crypto/{key_type}/birth/filebased/private.key"
                        )
                        self.__ca = f"/opt/mspki/{key_type}/certificate_chain.crt"
            except Exception as e:
                self.logger.error(
                    "Error occurred during certificate type detection: %s", e
                )
            finally:
                sock.close()

    def interface_monitor_cb(self, interfaces) -> None:
        """
        Callback for network interfaces. Service
        :param interfaces: list of dictionaries that includes
                           interface_name, operstate, mac_address.
        :return: -
        """
        with self.__lock:
            #self.logger.debug("Interface monitor cb: %s", interfaces)
            self.__interfaces.clear()

            any_interface_up = False
            for interface_data in interfaces:
                interface = Interface(
                    interface_name=interface_data["interface_name"],
                    operstat=interface_data["operstate"],
                    mac_address=interface_data["mac_address"],
                )
                self.__interfaces.append(interface)

                if interface.operstat == "UP":
                    any_interface_up = True

            if any_interface_up:
                # At least one interface is up, start service discovery
                if self.__if_to_monitor is None:
                    self.start_service_discovery()
                else:
                    for interface in self.__interfaces:
                        if interface.interface_name == self.__if_to_monitor:
                            if interface.operstat == "UP":
                                self.start_service_discovery()
                            else:
                                self.mdm_service_available = False
                                self.stop_service_discovery()
            else:
                self.mdm_service_available = False
                self.stop_service_discovery()

    def start_service_discovery(self):
        """
        Starts service monitor thread
        :param: -
        :return: -
        """
        if not self.service_monitor.running:
            self.logger.debug("Start service monitor thread")
            if not self.__service_monitor_thread.is_alive():
                self.__service_monitor_thread = threading.Thread(
                    target=self.service_monitor.run
                )
                self.__service_monitor_thread.start()
            else:
                self.logger.warning("Service monitor thread is already running")

    def stop_service_discovery(self):
        """
        Stops service monitor thread
        :param: -
        :return: -
        """
        if self.service_monitor.running:
            self.logger.debug("Stop service monitor thread")
            self.service_monitor.close()
            self.__service_monitor_thread.join()

    def start_interface_monitor(self) -> None:
        """
        Start interface monitoring
        """
        self.thread_if_mon.start()

    async def execute(self) -> None: # pylint: disable=too-many-branches
        """
        Execute MDM agent
        :return: -
        """
        self.running = True
        previous_status = ""
        while self.running:
            status = f"status: {self.__status}, mdm_available: {self.mdm_service_available}"
            # to avoid flooding log file
            if previous_status != status:
                self.logger.debug(status)
                previous_status = status

            if (
                self.__status[StatusType.UPLOAD_CERTIFICATES.value] == "FAIL"
                and self.mdm_service_available
            ):
                resp = self.upload_certificate_bundle()
                self.logger.debug(
                    "Certificate upload response: %s", resp
                )
                if resp.status_code == 200:
                    self.__status[StatusType.UPLOAD_CERTIFICATES.value] = "OK"
                    with open("/opt/certs_uploaded", "w", encoding="utf-8") as f:
                        f.write("certs uploaded")
                else:
                    self.logger.debug("Certificate upload failed")
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
                    self.logger.debug(
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
                            self.__cbma_set_up = self.cbma_ctrl.stop_cbma()
                            self.__cbma_set_up = self.cbma_ctrl.setup_cbma()
                        if (
                            self.__status[StatusType.DOWNLOAD_CERTIFICATES.value]
                            == "FAIL"
                        ):
                            self.logger.debug(
                                "Certificates action failed"
                            )
                    else:
                        self.logger.debug(
                            "Certificate download failed: %s", certificate_type.value
                        )
            elif self.mdm_service_available:
                await self.__loop_run_executor(self.executor, ConfigType.FEATURES)
                await self.__loop_run_executor(self.executor, ConfigType.MESH_CONFIG)
                if self.__mesh_conf_request_processed:
                    await self.__loop_run_executor(
                        self.executor, ConfigType.DEBUG_CONFIG
                    )
            if not self.__cbma_set_up:
                self.__cbma_set_up = self.cbma_ctrl.setup_cbma()
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
                self.logger.warning("mDNS resolution failed, using fallback")
                __https_url = self.__fallback_url

            return requests.get(
                f"https://{__https_url}/{Constants.GET_DEVICE_CONFIG.value}/{config.value}",
                params={"device_id": self.__device_id},
                cert=(self.__certificate_file, self.__keyfile),
                verify=self.__ca,
                timeout=20,
            )
        except requests.exceptions.ConnectionError as err:
            self.logger.error(
                "HTTP request failed with error: %s", err
            )
            return requests.Response()

    def __action_certificates(
        self, response: requests.Response, certificate_type: str
    ) -> str: # pylint: disable=too-many-branches, too-many-statements
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
                self.logger.error("Unknown certificate type")
                return "FAIL"

            payload_dict = json.loads(response.text)["payload"]
            self.logger.debug(
                "payload_dict[role]: %s", payload_dict["role"]
            )
            self.logger.debug(
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
                        self.logger.debug(
                            "Certificate %s saved", certificate_name
                        )
                        try:
                            old_umask = umask(0o377)
                            with tarfile.open(
                                f"{self.__cbma_certs_downloaded}/{certificate_name}",
                                "r:bz2",
                            ) as tar:
                                for member in tar.getmembers():
                                    # Skip empty folders
                                    if member.isdir() and not ".." in member.name:
                                        continue

                                    # Only extract regular files that don't contain relative paths
                                    if not member.isfile() or ".." in member.name:
                                        raise tarfile.TarError(f"Invalid/Malicious file in archive: {member.name}")

                                    # Construct the full path where the file would be extracted
                                    file_path = os.path.join(cert_path, member.name)

                                    # Check if the file already exists, overwrite check
                                    if os.path.exists(file_path):
                                        raise FileExistsError(
                                            f"File {file_path} already exists"
                                        )
                                    tar.extract(member, cert_path)

                                self.logger.debug(
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
                            self.logger.error(
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
                        self.logger.debug(
                            "Sig %s saved", certificate_name
                        )
                    umask(old_umask)
            self.logger.debug("Downloaded bundle saved")

        except Exception as e:
            self.logger.error(
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

        if self.__previous_config_mesh is not None:
            self.logger.debug(
                f"config: {config} previous: {json.loads(self.__previous_config_mesh)}"
            )

            if json.loads(self.__previous_config_mesh) == config:
                self.logger.debug(
                    "No changes in mesh config, not updating."
                )
                return "OK"

        self.logger.debug("No previous mesh config")

        ret, info = self.__comms_ctrl.settings.handle_mesh_settings(
            json.dumps(config["payload"])
        )

        self.logger.debug("ret: %s info: %s", ret, info)

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

                ret, info, _ = self.__comms_ctrl.command.handle_command(
                    cmd, self.__comms_ctrl
                )
                self.logger.debug("ret: %s info: %s", ret, info)

            if ret == "OK":
                self.__config_version = int(config["version"])
                self.__config_store.store(ConfigType.MESH_CONFIG.value, response.text.strip())
                self.__previous_config_mesh = self.__config_store.read(ConfigType.MESH_CONFIG.value)

        return ret

    def __action_feature_yaml(self, response: requests.Response) -> str:
        """
        Take feature.yaml into use
        :param response: https response
        :return: status
        """
        # check if features field exists

        try:
            config: dict = json.loads(response.text)

            try:
                self.logger.debug(
                    f"config: {config} previous: {json.loads(self.__previous_config_features)}"
                )
                if json.loads(self.__previous_config_features) == config:
                    self.logger.debug(
                        "No changes in features config, not updating."
                    )
                    return "OK"
            except TypeError:
                self.logger.debug("No previous features config")

            if config["payload"] is not None:
                features_dict = config["payload"]["features"]

                features_yaml = yaml.dump(features_dict, default_flow_style=False)

                with open(
                    Constants.YAML_FILE.value,
                    "w",
                    encoding="utf-8",
                ) as file_handle:
                    file_handle.write(features_yaml)

                self.__config_store.store(ConfigType.FEATURES.value, response.text.strip())
                self.__previous_config_features = self.__config_store.read(ConfigType.FEATURES.value)
                return "OK"

            self.logger.error("No features field in config")
        except KeyError:
            self.logger.error("KeyError features field in config")

        return "FAIL"

    def __handle_received_config(
        self, response: requests.Response, config: ConfigType
    ) -> str:
        """
        Handle received config
        :param response: HTTP response
        :return: -
        """

        self.logger.debug(
            f"HTTP Request Response: {response.text.strip()} {str(response.status_code).strip()}"
        )

        ret = "FAIL"

        if (
            self.__mesh_conf_request_processed
            and config.value == ConfigType.DEBUG_CONFIG.value
        ):
            self.__previous_debug_config = response.text.strip()
            self.__config_store.store(ConfigType.DEBUG_CONFIG.value, self.__previous_debug_config)
            self.__debug_config_interval = Constants.OK_POLLING_TIME_SECONDS.value
            ret = "OK"
        else:
            # radio configuration actions
            if config.value == ConfigType.MESH_CONFIG.value:
                ret = self.__action_radio_configuration(response)

            # feature.yaml actions
            elif config.value == ConfigType.FEATURES.value:
                ret = self.__action_feature_yaml(response)

        return ret

    def download_certificate_bundle(
        self, certificate_type: str = ""
    ) -> requests.Response:
        """
        Download certificate bundle
        :return: HTTP response
        """

        try:
            self.logger.debug("Downloading certificate bundle")

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
                timeout=20,
            )
        except FileNotFoundError as e:
            self.logger.error("Certificate file not found: %s", e)
        except requests.RequestException as e:
            self.logger.error(
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
                self.logger.debug("No certificates found")
                return requests.Response()

            for file in files:
                with open(file, "rb") as f:
                    data = f.read()
                    base64_data = base64.b64encode(data)
                    certificates_dict[os.path.basename(file)] = base64_data.decode(
                        "utf-8"
                    )

            self.logger.debug("Uploading certificate bundle")

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
                timeout=20,
            )
        except FileNotFoundError as e:
            self.logger.error("Certificate file not found: %s", e)
        except requests.RequestException as e:
            self.logger.error(
                "Post request failed with error: %s", e
            )
        return requests.Response()

    def __validate_response(
        self, response: requests.Response, config: ConfigType
    ) -> str:
        """
        Validate response
        :param response: HTTP response
        :param config: config
        """
        # validation for response parameters
        self.logger.debug(
            "validating response: %s, Config %s", response.text.strip(), config
        )
        status: str = "FAIL"

        if response.status_code == 200:
            if config == ConfigType.FEATURES:
                try:
                    if json.loads(response.text)["payload"]["features"]:
                        status = "OK"
                except KeyError:
                    self.logger.error(
                        "Features field not found in config"
                    )
            elif config == ConfigType.MESH_CONFIG:
                try:
                    if json.loads(response.text)["payload"]["radios"]:
                        status = "OK"
                except KeyError:
                    self.logger.error(
                        "Radios field not found in config"
                    )
            elif config == ConfigType.DEBUG_CONFIG:
                try:
                    if json.loads(response.text)["payload"]["debug_config"]:
                        status = "OK"
                except KeyError:
                    self.logger.error(
                        "Debug config field not found in config"
                    )
            else:
                self.logger.error("Validation not implemented, unknown config")
        else:
            status = "FAIL"

        return status

    async def __loop_run_executor(self, executor, config: ConfigType) -> None: # pylint: disable=too-many-branches
        """
        Loop run executor
        :param executor: executor
        :param config: configType
        """
        # This function makes a synchronous HTTP request using ThreadPoolExecutor
        https_loop = asyncio.get_event_loop()
        response = await https_loop.run_in_executor(
            executor, self.__http_get_device_config, config
        )

        status_type = self.__config_status_mapping.get(config, None)

        self.logger.debug(
            "HTTP Request status: %s, config: %s", str(response.status_code), config
        )

        # validate response
        if self.__validate_response(response, config) == "FAIL":
            if config == ConfigType.DEBUG_CONFIG.value and response.status_code == 405:
                self.logger.debug("Validation status: FAIL,  DEBUG_CONFIG not supported")
            else:
                self.logger.debug(
                    "Validation status: %s", self.__status[status_type]
                )
            return

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
                    self.logger.debug(
                        "MDM Server has no support for debug mode"
                    )
                    self.__debug_config_interval = (
                        Constants.OK_POLLING_TIME_SECONDS.value
                    )
                    self.__mesh_conf_request_processed = False
        else:
            if response.status_code == 200:
                ret = self.__handle_received_config(response, config)
                self.logger.debug("config: %s, ret: %s", config, ret)
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

            self.logger.debug("status: %s", self.__status)


async def main_mdm(keyfile=None, certfile=None, ca_file=None, interface=None) -> None:
    """
    main
    param: keyfile: keyfile
    param: certfile: certfile
    param: ca_file: ca_file
    return: -
    """
    cc = comms_controller.CommsController()

    if keyfile is None or certfile is None or ca_file is None:
        cc.logger.debug("MDM: Closing as no certificates provided")
        return

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()


    def signal_handler(signum, frame):
        sig_name = signal.Signals(signum).name
        cc.logger.debug(f"signal_handler {sig_name} ({signum}) received.")

        if sig_name in ("SIGUSR1", "SIGINT", "SIGTERM"):
            if not mdm.running:
                # stop sequence already started
                return

        cc.logger.debug("Disconnecting MDM agent...")

        mdm.mdm_service_available = False
        mdm.running = False
        mdm.executor.shutdown()
        mdm.interface_monitor.stop()
        mdm.cbma_ctrl.stop_cbma()
        mdm.cbma_ctrl.stop_radios()
        if mdm.service_monitor.running:
            mdm.stop_service_discovery()
        if mdm.thread_if_mon.is_alive():
            mdm.thread_if_mon.join()
            cc.logger.debug("Interface monitor stopped")

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

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
        signal_handler(signal.SIGUSR1, signal_handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mesh Settings")
    parser.add_argument("-k", "--keyfile", help="TLS keyfile", required=False)
    parser.add_argument("-c", "--certfile", help="TLS certfile", required=False)
    parser.add_argument("-r", "--ca", help="ca certfile", required=False)
    parser.add_argument(
        "-i",
        "--interface",
        help="Interface from where to monitor MDM server. Default is bat0",
        required=False,
    )

    args = parser.parse_args()
    loop = asyncio.new_event_loop()

    if args.interface is None:
        interface = Constants.LOWER_BATMAN.value
    else:
        interface = args.interface
    loop.run_until_complete(main_mdm(args.keyfile, args.certfile, args.ca, interface))
    loop.close()
