"""
Comms NATS controller
"""
import asyncio
import signal
import ssl
import argparse
import json
from nats.aio.client import Client as NATS

from src import comms_common as comms
from src import comms_settings
from src import comms_command

class CommsController:  # pylint: disable=too-few-public-methods
    """
    Mesh network
    """
    def __init__(self, server, port):
        self.nats_server = server
        self.port = port
        self.settings = comms_settings.CommsSettings()
        self.command = comms_command.Command(server, port)
        self.status = comms.STATUS.no_status

async def main(server, port, keyfile=None, certfile=None):
    """
    main
    """
    cc = CommsController(server, port)

    nats_client = NATS()

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()

    def signal_handler():
        if nats_client.is_closed:
            return
        print("Disconnecting...")
        asyncio.create_task(nats_client.close())
        asyncio.create_task(stop())

    for sig in ('SIGINT', 'SIGTERM'):
        asyncio.get_running_loop().add_signal_handler(getattr(signal, sig),
                                                      signal_handler)

    async def disconnected_cb():
        print("Got disconnected...")

    async def reconnected_cb():
        print("Got reconnected...")

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
        ret, info, resp = "FAIL", "Not supported subject", ""
        mesh_status = comms.STATUS.no_status

        if subject == "comms.settings":
            ret, info, mesh_status = cc.settings.handle_mesh_settings(data)
        elif subject == "comms.command":
            ret, info, mesh_status, resp = cc.command.handle_command(data)
        # elif subject == "comms.status":

        if data is None:
            status = f"""{{"status":"{ret}","mesh_status":"{mesh_status}","info":"{info}"}}"""
        else:
            status = f"""{{"status":"{ret}","mesh_status":"{mesh_status}","info":"{info}","data":"{resp}"}}"""

        await msg.respond(status.encode("utf-8"))

    await nats_client.subscribe("comms.settings", cb=message_handler)
    await nats_client.subscribe("comms.command", cb=message_handler)

    print("Mesh Settings Listening for requests")


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
