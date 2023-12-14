"""
Comms NATS controller
"""
import asyncio
import signal
import ssl
import argparse
import logging
import threading
import json
import re
import base64
from typing import Optional
from datetime import datetime
import subprocess
from concurrent.futures import ThreadPoolExecutor
import OpenSSL.crypto


from nats.aio.client import Client as nats
import requests

from src import comms_settings
from src import comms_command
from src import comms_status

from src import batadvvis
from src import batstat

from src import comms_service_discovery
from src import comms_if_monitor

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
        self.thread_visual = threading.Thread(target=self.batman_visual.run)  # create thread
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


class MdmAgent:
    """
    MDM Agent
    """

    GET_CONFIG: str = "public/config"
    GET_DEVICE_CONFIG: str = "public/get_device_config"  # + config_type
    PUT_DEVICE_CONFIG: str = "public/put_device_config"  # + config_type

    OK_POLLING_TIME_SECONDS: int = 600
    FAIL_POLLING_TIME_SECONDS: int = 1
    YAML_FILE: str = "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features.yaml"

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
        self.__previous_config: Optional[str] = self.__read_config_from_file()
        self.__previous_debug_config: Optional[str] = self.__read_debug_config_from_file()
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
        self.__interfaces = {}
        self.service_monitor = comms_service_discovery.CommsServiceMonitor(
            service_name="MDM Service",
            service_type="_mdm._tcp.local.",
            service_cb=self.mdm_server_address_cb,
            interface=self.__if_to_monitor
        )
        self.interface_monitor = comms_if_monitor.CommsInterfaceMonitor(
            self.interface_monitor_cb
        )
        self.__certs_uploaded = False  # Should be true if CBMA certs exist

        try:
            with open("/opt/identity", "r", encoding="utf-8") as f:  # todo
                self.__device_id = f.read().strip()
        except FileNotFoundError:
            self.__device_id = "default"
        self.__config_type = "mesh_conf"
        self.__debug_config_type = "debug_conf"

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
        :param interfaces: list of dictionaries (ifname, ifstate)
        :return: -
        """
        self.__interfaces = interfaces
        self.__comms_controller.logger.debug(
            "Interface monitor cb: %s", self.__interfaces
            )
        if self.__if_to_monitor is None:
            self.start_service_discovery()
        elif self.__if_to_monitor in self.__interfaces:
            if self.__interfaces[self.__if_to_monitor] == "UP":
                self.start_service_discovery()
            elif self.__interfaces[self.__if_to_monitor] == "DOWN":
                self.stop_service_discovery()

    def start_service_discovery(self):
        """
        Starts service monitor thread
        :param: -
        :return: -
        """
        if not self.service_monitor.running:
            self.__comms_controller.logger.debug(
                "Start service monitor thread"
            )
            self.service_monitor_thread = threading.Thread(
                target=self.service_monitor.run
            )
            self.service_monitor_thread.start()

    def stop_service_discovery(self):
        """
        Stops service monitor thread
        :param: -
        :return: -
        """
        if self.service_monitor.running:
            self.__comms_controller.logger.debug(
                "Stop service monitor thread"
            )
            self.service_monitor.close()
            self.service_monitor_thread.join()

    async def execute(self) -> None:
        """
        Execute MDM agent
        :return: -
        """
        executor = ThreadPoolExecutor(1)

        # Create interface monitoring thread
        self.thread_if_mon = threading.Thread(
            target=self.interface_monitor.monitor_interfaces
            )
        self.thread_if_mon.start()

        while True:
            if self.mdm_service_available:
                await self.__loop_run_executor(executor)
            # if not self.__certs_uploaded:
            #    resp = self.upload_certificate_bundle()
            #    if resp.status_code == 200:
            #        self.__certs_uploaded = True
            await asyncio.sleep(float(min(self.__interval, self.__debug_config_interval)))

    def __http_get_device_config(self) -> requests.Response:
        """
        HTTP get device config
        :return: HTTP response
        """
        try:
            # mDNS discovery callback updates the url and can be None
            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            if self.__mesh_conf_request_processed:
                return requests.get(
                    f"https://{__https_url}/{self.GET_DEVICE_CONFIG}/{self.__debug_config_type}",
                    params={"device_id": self.__device_id},
                    cert=(self.__certificate_file, self.__keyfile),
                    verify=self.__ca,
                    timeout=2,
                )
            else:
                return requests.get(
                    f"https://{__https_url}/{self.GET_DEVICE_CONFIG}/{self.__config_type}",
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

    @staticmethod
    def __is_base64(string_to_check: str) -> bool:
        """
        Check if string is base64 encoded
        :param string_to_check: string to check
        :return: True if base64 encoded
        """
        try:
            # Check if the string is a valid base64 format
            if not re.match("^[A-Za-z0-9+/]+[=]{0,2}$", string_to_check):
                return False

            # Attempt to decode
            _ = base64.b64decode(string_to_check, validate=True)
            return True
        except (re.error, __import__("binascii").Error):
            return False

    def __action_radio_configuration(self, response: requests.Response) -> str:
        """
        Take radio configuration into use
        :param response: https response
        :return: status
        """
        config = json.loads(response.text)

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
                self.__interval = self.OK_POLLING_TIME_SECONDS
                self.__write_config_to_file(response)  # TODO: remove certs from config
        else:
            self.__interval = self.FAIL_POLLING_TIME_SECONDS

        return ret

    def __action_feature_yaml(self, config: dict) -> str:
        """
        Take feature.yaml into use
        :param config: config dict
        :return: status
        """
        # check if features field exists

        try:
            if config["features"] is not None:
                yaml = config["features"]
                # check if features field is base64 encoded
                if self.__is_base64(yaml):
                    # decode base64
                    yaml = base64.b64decode(config["features"]).decode("utf-8")

                with open(
                    self.YAML_FILE,
                    "w",
                    encoding="utf-8",
                ) as f:
                    f.write(yaml)

                self.__interval = self.OK_POLLING_TIME_SECONDS
                self.__comms_controller.logger.debug("No features field in config")
                return "OK"

            self.__comms_controller.logger.error("No features field in config")
            self.__interval = self.FAIL_POLLING_TIME_SECONDS
        except KeyError:
            self.__comms_controller.logger.error("KeyError features field in config")
            self.__interval = self.FAIL_POLLING_TIME_SECONDS

        return "FAIL"

    def __action_cert_keys(self, config: dict) -> str:
        """
        Take cert/keys into use
        :param config: config dict
        :return: status
        """
        try:
            if config["certificates"] is not None:
                self.__comms_controller.logger.debug("certs field in config")
                self.__comms_controller.logger.debug(
                    "__action_cert_keys: not implemented"
                )
                return "OK"
        except KeyError:
            self.__comms_controller.logger.error(
                "KeyError certificates field in config"
            )

        self.__interval = self.FAIL_POLLING_TIME_SECONDS
        return "FAIL"

    def __handle_received_config(self, response: requests.Response) -> None:
        """
        Handle received config
        :param response: HTTP response
        :return: -
        """

        self.__comms_controller.logger.debug(
            f"HTTP Request Response: {response.text.strip()} {str(response.status_code).strip()}"
        )

        data = json.loads(response.text)

        if self.__mesh_conf_request_processed:
            self.__previous_debug_config = response.text.strip()

            # save to file to use in fmo
            try:
                with open("/opt/debug_config.json", "w", encoding="utf-8") as debug_conf_json:
                    debug_conf_json.write(self.__previous_debug_config)
                    self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
            except:
                self.__comms_controller.logger.debug("cannot store /opt/debug_config.json")
                self.__debug_config_interval = self.FAIL_POLLING_TIME_SECONDS
                return

        else:
            self.__previous_config = response.text.strip()

            # radio configuration actions0
            ret = self.__action_radio_configuration(response)
            if ret == "FAIL":
                return

            # feature.yaml actions
            ret = self.__action_feature_yaml(data)
            if ret == "FAIL":
                return

            # cert/keys actions
            ret = self.__action_cert_keys(data)
            if ret == "FAIL":
                return

    @staticmethod
    def __read_config_from_file() -> Optional[str]:
        """
        Read config from file
        :return: config
        """
        try:
            with open("/opt/config.json", "r", encoding="utf-8") as f:
                return f.read().strip()
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
    def __write_config_to_file(response: requests.Response) -> None:
        """
        Write config to file
        :param response: HTTP response
        :return: -
        """
        with open("/opt/config.json", "w", encoding="utf-8") as f:
            f.write(response.text.strip())

    def upload_certificate_bundle(self) -> requests.Response:
        try:
            # TODO: Not sure what kind of bundle we should deliver
            # so bundling just device cert and ca cert for now.
            with open(self.__certificate_file, 'r') as cert_file:
                certificate_content = cert_file.read()

            with open(self.__ca, 'r') as ca_file:
                ca_certificate_content = ca_file.read()

            # Concatenate the certificate and CA certificate
            bundle_content = certificate_content + ca_certificate_content

            # Prepare the JSON data for the POST request
            data = {
                'device_id': self.__device_id,
                'payload': bundle_content,
            }

            __https_url = self.__url
            if "None" in __https_url:
                return requests.Response()  # empty response

            # Make the POST request
            url = f"https://{__https_url}/{self.PUT_DEVICE_CONFIG}/certificates"
            return requests.post(url,
                                 json=data,
                                 cert=(self.__certificate_file, self.__keyfile),
                                 verify=self.__ca,
                                 timeout=2,)
        except FileNotFoundError as e:
            self.__comms_controller.logger.error(
                "Certificate file not found: %s", e
            )
        except requests.RequestException as e:
            self.__comms_controller.logger.error(
                "Post request failed with error: %s", e
            )

    async def __loop_run_executor(self, executor) -> None:
        """
        Loop run executor
        :param executor: executor
        """
        # This function makes a synchronous HTTP request using ThreadPoolExecutor
        https_loop = asyncio.get_event_loop()
        response = await https_loop.run_in_executor(
            executor, self.__http_get_device_config
        )
        self.__comms_controller.logger.debug(
            "HTTP Request status: %s", str(response.status_code)
        )
        if self.__mesh_conf_request_processed:
            if (
                response.status_code == 200
                and self.__previous_debug_config != response.text.strip()
            ):
                self.__handle_received_config(response)
                self.__mesh_conf_request_processed = False
            elif (
                response.status_code == 200
                and self.__previous_debug_config == response.text.strip()
            ):
                self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
                self.__mesh_conf_request_processed = False
            elif (
                response.status_code == 405
            ):
                self.__comms_controller.logger.debug(
                    "MDM Server has no support for debug mode"
                )
                self.__debug_config_interval = self.OK_POLLING_TIME_SECONDS
            elif response.text.strip() == "" or response.status_code != 200:
                self.__debug_config_interval = self.FAIL_POLLING_TIME_SECONDS
        else:
            if (
                response.status_code == 200
                and self.__previous_config != response.text.strip()
            ):
                self.__handle_received_config(response)
                self.__mesh_conf_request_processed = True
            elif (
                response.status_code == 200
                and self.__previous_config == response.text.strip()
            ):
                self.__interval = self.OK_POLLING_TIME_SECONDS
                self.__mesh_conf_request_processed = True
            elif response.text.strip() == "" or response.status_code != 200:
                self.__interval = self.FAIL_POLLING_TIME_SECONDS


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
        cc.logger.debug("Disconnecting...")
        mdm.mdm_service_available = False
        if mdm.service_monitor.running:
            mdm.stop_service_discovery()
        if mdm.interface_monitor.running:
            mdm.interface_monitor.running = False
            mdm.thread_if_mon.join()
        asyncio.create_task(stop())

    for sig in ("SIGINT", "SIGTERM"):
        asyncio.get_running_loop().add_signal_handler(
            getattr(signal, sig), signal_handler
        )

    cc.logger.debug("MDM: comms_nats_controller Listening for requests")

    # separate instances needed for FMO/MDM

    mdm = MdmAgent(cc, keyfile, certfile, ca_file, interface)
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
                with open("/opt/debug_config.json", "r", encoding="utf-8") as debug_conf_json:
                    debug_conf_data = debug_conf_json.read().strip()
                    debug_conf_data_json = json.loads(debug_conf_data)
                    debug_mode = debug_conf_data_json['payload']['debug_conf'][0]['debug_mode']
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

    # separate instances needed for FMO/MDM
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

    # test certificates location
    # if args.ca is None or args.certfile is None or args.keyfile is None:
    #     args.ca = \
    #         "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/ca.crt"
    #     args.certfile = \
    #         "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/issued/csl1.local.crt"
    #     args.keyfile = \
    #         "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/private/csl1.local.key"

    loop = asyncio.new_event_loop()
    if args.agent == "mdm":
        loop.run_until_complete(
            main_mdm(args.keyfile, args.certfile, args.ca, args.interface)
        )
    else:
        loop.run_until_complete(
            main_fmo(args.server, args.port, args.keyfile, args.certfile)
        )
    loop.run_forever()
    loop.close()
