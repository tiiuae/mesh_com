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
from typing import Optional
import OpenSSL.crypto
from datetime import datetime
import subprocess

from nats.aio.client import Client as NATS
from concurrent.futures import ThreadPoolExecutor
import requests

from src import comms_settings
from src import comms_command
from src import comms_status

from src import batadvvis
from src import batstat

from src import comms_service_discovery

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
        self.t1 = None
        self.t2 = None
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
        self.t1 = threading.Thread(target=self.batman_visual.run)  # create thread
        self.t1.start()  # start thread
        self.t2 = threading.Thread(target=self.batman.run)  # create thread
        self.t2.start()  # start thread
        self.visualisation_enabled = True  # publisher enabled

    def stop(self):
        """
        Stop method for collecting telemetry

        :return: -
        """
        self.visualisation_enabled = False  # publisher disabled
        if self.batman_visual.thread_running:
            self.batman_visual.thread_running = False  # thread loop disabled
            self.t1.join()  # wait for thread to finish
        if self.batman.thread_running:
            self.batman.thread_running = False  # thread loop disabled
            self.t2.join()  # wait for thread to finish


# pylint: disable=too-many-instance-attributes
class CommsController:  # pylint: disable=too-few-public-methods
    """
    Mesh network
    """

    def __init__(self, interval: int = 1000):
        self.interval = interval

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

    def __init__(
        self,
        comms_controller,
        keyfile: str = None,
        certificate_file: str = None,
        ca_file: str = None,
    ):
        """
        Constructor
        """
        self.__previous_config: Optional[str] = self.__read_config_from_file()
        self.__comms_controller: CommsController = comms_controller
        self.__interval: int = 1  # possible to update from MDM server?
        self.__url: str = "defaultmdm.local:5000"  # mDNS callback updates this one
        self.__keyfile: str = keyfile
        self.__certificate_file: str = certificate_file
        self.__ca: str = ca_file
        self.__config_version: int = 0
        self.mdm_service_available: bool = False
        try:
            with open("/opt/identity", "r", encoding="utf-8") as f:  # todo
                self.__device_id = f.read().strip()
        except FileNotFoundError:
            self.__device_id = "default"
        self.__config_type = "mesh_conf"

    def mdm_server_address_cb(self, address: str, status: bool) -> None:
        """
        Callback for MDM server address
        :param address: MDM server address
        :param status: MDM connection status
        :return: -
        """
        self.__url = address
        self.mdm_service_available = status

    async def execute(self) -> None:
        """
        Execute MDM agent
        :return: -
        """
        executor = ThreadPoolExecutor(1)

        while True:
            if self.mdm_service_available:
                await self.__loop_run_executor(executor)
            await asyncio.sleep(float(self.__interval))

    def __http_get_device_config(self) -> requests.Response:
        try:
            return requests.get(
                f"https://{self.__url}/{self.GET_DEVICE_CONFIG}/{self.__config_type}",
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

    def __handle_received_config(self, response: requests.Response) -> None:
        """
        Handle received config
        :param response: HTTP response
        :return: -
        """

        self.__comms_controller.logger.debug(
            f"HTTP Request Response: {response.text.strip()} {str(response.status_code).strip()}"
        )
        self.__previous_config = response.text.strip()

        data = json.loads(response.text)
        ret, info = self.__comms_controller.settings.handle_mesh_settings(
            json.dumps(data["payload"])
        )
        self.__comms_controller.logger.debug("ret: %s info: %s", ret, info)
        if ret == "OK":
            for radio in data["payload"]["radios"]:
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
            self.__config_version = int(data["version"])
            self.__interval = 10
            self.__write_config_to_file(response)

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
    def __write_config_to_file(response: requests.Response) -> None:
        """
        Write config to file
        :param response: HTTP response
        :return: -
        """
        with open("/opt/config.json", "w", encoding="utf-8") as f:
            f.write(response.text.strip())

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
        if (
            response.status_code == 200
            and self.__previous_config != response.text.strip()
            and response.text.strip() != ""
        ):
            self.__handle_received_config(response)


async def main_mdm(keyfile=None, certfile=None, ca_file=None):
    """
    main
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
        asyncio.create_task(stop())

    for sig in ("SIGINT", "SIGTERM"):
        asyncio.get_running_loop().add_signal_handler(
            getattr(signal, sig), signal_handler
        )

    cc.logger.debug("MDM: comms_nats_controller Listening for requests")

    # separate instances needed for FMO/MDM

    mdm = MdmAgent(cc, keyfile, certfile, ca_file)
    monitor = comms_service_discovery.CommsServiceMonitor(
        service_name="MDM Service",
        service_type="_mdm._tcp.local.",
        service_cb=mdm.mdm_server_address_cb,
    )
    await asyncio.gather(mdm.execute(), monitor.async_run())


# pylint: disable=too-many-arguments, too-many-locals, too-many-statements
async def main_fmo(server, port, keyfile=None, certfile=None, interval=1000):
    """
    main
    """
    cc = CommsController(interval)

    nats_client = NATS()
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

        command = json.loads(data)["cmd"]

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
    parser.add_argument(
        "-a", "--agent", help="mdm or fmo", required=False, default="fmo"
    )

    args = parser.parse_args()

    # test certificates location
    # if args.ca is None or args.certfile is None or args.keyfile is None:
    #     args.ca = "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/ca.crt"
    #     args.certfile = "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/issued/csl1.local.crt"
    #     args.keyfile = "/home/mika/work/tc_distro/nats-server/mesh_com_dev/mypki/pki/private/csl1.local.key"

    loop = asyncio.new_event_loop()
    if args.agent == "mdm":
        loop.run_until_complete(main_mdm(args.keyfile, args.certfile, args.ca))
    else:
        loop.run_until_complete(
            main_fmo(args.server, args.port, args.keyfile, args.certfile)
        )
    loop.run_forever()
    loop.close()
