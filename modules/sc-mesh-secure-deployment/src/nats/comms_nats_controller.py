"""
Comms NATS controller
"""
import asyncio
import signal
import ssl
import argparse
import logging
import threading
from nats.aio.client import Client as NATS

from src import comms_common as comms
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
        Run method for e

        :return: -
        """
        self.batman_visual.thread_running = True # thread loop enabled
        self.batman.thread_running = True # thread loop enabled
        self.t1 = threading.Thread(target=self.batman_visual.run) # create thread
        self.t1.start() # start thread
        self.t2 = threading.Thread(target=self.batman.run) # create thread
        self.t2.start() # start thread
        self.visualisation_enabled = True # publisher enabled

    def stop(self):
        """
        Stop method for collecting telemetry

        :return: -
        """
        self.visualisation_enabled = False # publisher disabled
        self.batman_visual.thread_running = False # thread loop disabled
        self.t1.join() # wait for thread to finish
        self.batman.thread_running = False # thread loop disabled
        self.t2.join() # wait for thread to finish


class CommsController:  # pylint: disable=too-few-public-methods
    """
    Mesh network
    """
    def __init__(self, server: str, port: str, interval: int = 1000):
        self.nats_server = server
        self.port = port
        self.interval = interval
        self.comms_status = comms_status.CommsStatus()

        # base logger for comms and which is used by all other modules
        self.main_logger = logging.getLogger("comms")
        self.main_logger.setLevel(logging.DEBUG)
        log_formatter = logging.Formatter(
            fmt='%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s')
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.main_logger.addHandler(console_handler)

        self.settings = comms_settings.CommsSettings(self.comms_status, self.main_logger.getChild("settings"))
        self.command = comms_command.Command(server, port, self.comms_status, self.main_logger.getChild("command"))
        self.telemetry = MeshTelemetry(self.interval, self.main_logger.getChild("telemetry"))
        self.status = comms.STATUS.no_status

        # logger for this module and derived from main logger
        self.logger = self.main_logger.getChild("controller")

async def main(server, port, keyfile=None, certfile=None, interval=1000):
    """
    main
    """
    cc = CommsController(server, port, interval)
    nats_client = NATS()

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
        ssl_context = ssl.create_default_context()
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

    async def message_handler(msg):
        # reply = msg.reply
        subject = msg.subject
        data = msg.data.decode()
        cc.logger.debug("Received a message on '%s': %s", subject, data)
        ret, info, resp = "FAIL", "Not supported subject", ""
        status = comms.STATUS.no_status

        if subject == "comms.settings":
            ret, info, status = cc.settings.handle_mesh_settings(data)
        elif subject == "comms.command":
            ret, info, status, resp = cc.command.handle_command(data, cc)
        # elif subject == "comms.status":

        if data is None:
            response = f"""{{"status":"{ret}","mesh_status":"{status.value}","info":"{info}"}}"""
        else:
            response = f"""{{"status":"{ret}","mesh_status":"{status.value}","info":"{info}","data":"{resp}"}}"""

        cc.logger.debug("Sending response: %s", response[:1000])
        await msg.respond(response.encode("utf-8"))

    await nats_client.subscribe("comms.settings", cb=message_handler)
    await nats_client.subscribe("comms.command", cb=message_handler)

    cc.logger.debug("comms_nats_controller Listening for requests")
    while True:
        await asyncio.sleep(float(cc.interval) / 1000.0)
        try:
            if cc.telemetry.visualisation_enabled:
                msg = cc.telemetry.mesh_visual()
                cc.logger.debug("Publishing comms.visual: %s", msg)
                await nats_client.publish("comms.visual",
                             msg.encode())
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
