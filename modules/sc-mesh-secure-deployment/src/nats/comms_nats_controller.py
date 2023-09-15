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
from nats.aio.client import Client as NATS

from src import comms_settings
from src import comms_command
from src import comms_status

from src import batadvvis
from src import batstat


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
        self.batman_visual = batadvvis.BatAdvVis(self.interval*0.2)
        self.batman = batstat.Batman(self.interval*0.2)
        self.visualisation_enabled = False

    def mesh_visual(self):
        """
        Get mesh visualisation

        :return: mesh visualisation
        """
        return f"[{self.batman_visual.latest_topology}," \
               f"{self.batman.latest_stat}]". \
            replace(": ", ":"). \
            replace(", ", ",")

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
    def __init__(self, server: str, port: str, interval: int = 1000):
        self.nats_server = server
        self.port = port
        self.interval = interval

        # base logger for comms and which is used by all other modules
        self.main_logger = logging.getLogger("comms")
        self.main_logger.setLevel(logging.DEBUG)
        log_formatter = logging.Formatter(
            fmt='%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s')
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.main_logger.addHandler(console_handler)

        self.comms_status = comms_status.CommsStatus(self.main_logger.getChild("status"))
        self.settings = comms_settings.CommsSettings(self.comms_status,
                                                     self.main_logger.getChild("settings"))
        self.command = comms_command.Command(server, port, self.comms_status,
                                             self.main_logger.getChild("command"))
        self.telemetry = MeshTelemetry(self.interval, self.main_logger.getChild("telemetry"))

        # logger for this module and derived from main logger
        self.logger = self.main_logger.getChild("controller")

class CommsCsa: # pylint: disable=too-few-public-methods
    """
    Comms CSA class to storage settings for CSA for a state change
    """
    def __init__(self):
        self.delay = "0"
        self.ack_sent = False


# pylint: disable=too-many-arguments, too-many-locals, too-many-statements
async def main(server, port, keyfile=None, certfile=None, interval=1000):
    """
    main
    """
    cc = CommsController(server, port, interval)
    nats_client = NATS()
    csac = CommsCsa()

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()

    def signal_handler():
        if nats_client.is_closed:
            return
        cc.logger.debug("Disconnecting...")
        asyncio.create_task(nats_client.close())
        asyncio.create_task(stop())

    for sig in ('SIGINT', 'SIGTERM'):
        asyncio.get_running_loop().add_signal_handler(getattr(signal, sig),
                                                      signal_handler)

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
        await nats_client.connect(f"tls://{server}:{port}",
                                  tls=ssl_context,
                                  reconnected_cb=reconnected_cb,
                                  disconnected_cb=disconnected_cb,
                                  max_reconnect_attempts=-1)
    else:
        await nats_client.connect(f"nats://{server}:{port}",
                                  reconnected_cb=reconnected_cb,
                                  disconnected_cb=disconnected_cb,
                                  max_reconnect_attempts=-1)
    async def handle_settings_csa_post(ret):
        if ret == "OK":
            ret = "ACK"
        elif ret == "TRIGGER":
            cmd = json.dumps({"api_version": 1, "cmd": "APPLY"})
            cc.command.handle_command(cmd, cc, True, csac.delay)
        elif ret == "COUNT":
            ret = "COUNT"  # not to send ACK/NACK
        else:
            ret = "NACK"

        if ret in ("ACK", "NACK") and csac.ack_sent is False:
            response = {'status': ret}
            cc.logger.debug("publish response: %s", str(response))
            await nats_client.publish("comms.settings_csa",
                                      json.dumps(response).encode("utf-8"))
            csac.ack_sent = True

    async def message_handler(message):
        # reply = message.reply
        subject = message.subject
        data = message.data.decode()
        cc.logger.debug("Received a message on '%s': %s", subject, data)
        ret, info, resp = "FAIL", "Not supported subject", ""

        if subject == "comms.settings":
            ret, info = cc.settings.handle_mesh_settings(data)
        elif subject == "comms.settings_csa":
            ret, info, delay = cc.settings.handle_mesh_settings_csa(data)
            csac.delay = delay
            csac.ack_sent = "status" in data

        elif subject == "comms.command":
            ret, info, resp = cc.command.handle_command(data, cc)
        elif subject == "comms.status":
            ret, info = "OK", "Returning current status"

        if subject == "comms.settings_csa":
            await handle_settings_csa_post(ret)
        else:
            # Update status info
            cc.comms_status.refresh_status()
            response = {'status': ret, 'info': info,
                        'mesh_status': cc.comms_status.mesh_status,
                        'mesh_cfg_status': cc.comms_status.mesh_cfg_status,
                        'visualisation_active': cc.comms_status.is_visualisation_active,
                        'mesh_radio_on': cc.comms_status.is_mesh_radio_on,
                        'ap_radio_on': cc.comms_status.is_ap_radio_on,
                        'security_status': cc.comms_status.security_status }

            if resp != "":
                response['data'] = resp

            cc.logger.debug("Sending response: %s", str(response)[:1000])
            await message.respond(json.dumps(response).encode("utf-8"))

    await nats_client.subscribe("comms.settings", cb=message_handler)
    await nats_client.subscribe("comms.settings_csa", cb=message_handler)
    await nats_client.subscribe("comms.command", cb=message_handler)
    await nats_client.subscribe("comms.status", cb=message_handler)

    cc.logger.debug("comms_nats_controller Listening for requests")
    while True:
        await asyncio.sleep(float(cc.interval) / 1000.0)
        try:
            if cc.telemetry.visualisation_enabled:
                msg = cc.telemetry.mesh_visual()
                cc.logger.debug("Publishing comms.visual: %s", msg)
                await nats_client.publish("comms.visual", msg.encode())
        except Exception as e:
            cc.logger.error("Error:", e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mesh Settings')
    parser.add_argument('-s', '--server', help='Server IP', required=True)
    parser.add_argument('-p', '--port', help='Server port', required=True)
    parser.add_argument('-k', '--keyfile', help='TLS keyfile', required=False)
    parser.add_argument('-c', '--certfile', help='TLS certfile', required=False)
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(main(args.server, args.port,
                                 args.keyfile, args.certfile))
    loop.run_forever()
    loop.close()
