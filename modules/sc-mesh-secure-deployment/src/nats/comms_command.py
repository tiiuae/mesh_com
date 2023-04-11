"""
Comms command control NATS node
"""
import json
import subprocess
from shlex import quote
import asyncio
import signal
import ssl
import argparse
from nats.aio.client import Client as NATS

import comms_common as comms


class CommandControl:
    """
    Command  control class
    """

    class Command:  # pylint: disable=too-few-public-methods
        """
        Command class
        """

        def __init__(self):
            self.api_version = 1
            self.command = ""

    def __init__(self):
        self.cmd = self.Command()
        self.status = comms.STATUS.no_status

    def handle_command(self, msg: str) -> (str, str):
        try:
            parameters = json.loads(msg)
            print(parameters)
            self.cmd.api_version = int(parameters["api_version"])
            self.cmd.command = quote(str(parameters["cmd"]))
        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            ret, self.status = "FAIL", comms.STATUS.mesh_fail
            info = "JSON format not correct" + str(error)
            return ret, info

        if self.cmd.api_version != 1:
            ret, info = "FAIL", "API version not supported"
        elif self.cmd.command == comms.COMMAND.revoke:
            ret, info = self.__activate_default_mesh()
        elif self.cmd.command == comms.COMMAND.apply:
            ret, info = self.__apply_mission_config()
        elif self.cmd.command == comms.COMMAND.wifi_down:
            ret, info = "FAIL", "Command not implemented"
        elif self.cmd.command == comms.COMMAND.wifi_up:
            ret, info = "FAIL", "Command not implemented"
        elif self.cmd.command == comms.COMMAND.reboot:
            ret, info = "FAIL", "Command not implemented"
        elif self.cmd.command == comms.COMMAND.get_logs:
            ret, info = "FAIL", "Command not implemented"
        else:
            ret, info = "FAIL", "Command not supported"
        return ret, info

    def __activate_default_mesh(self) -> (str, str):
        ret = subprocess.run(["/opt/S9011sMesh", "restart", "default"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            # todo: Default mesh configuration failed. What to do next?
            self.status = comms.STATUS.mesh_fail
            return "FAIL", "default mesh starting failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr)

        self.status = comms.STATUS.mesh_default
        return "OK", "Default mesh command applied"

    def __apply_mission_config(self) -> (str, str):
        ret = subprocess.run(["/opt/S9011sMesh", "restart", "mission"],
                             shell=False, check=True, capture_output=True)
        if ret.returncode != 0:
            self.status = comms.STATUS.mesh_fail
            return "FAIL", "mesh starting failed " \
                           + str(ret.returncode) \
                           + str(ret.stdout) \
                           + str(ret.stderr)

        print('Mission configurations applied')
        self.status = comms.STATUS.mesh_mission_not_connected
        return "OK", "Mission configurations applied"

async def main(server, port, keyfile=None, certfile=None):
    """
    main
    """
    cmd_controller = CommandControl()

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
        await nats_client.connect("tls://" + str(server) + ":" + str(port),
                                  tls=ssl_context,
                                  reconnected_cb=reconnected_cb,
                                  disconnected_cb=disconnected_cb,
                                  max_reconnect_attempts=-1)
    else:
        await nats_client.connect("nats://" + str(server) + ":" + str(port),
                                  reconnected_cb=reconnected_cb,
                                  disconnected_cb=disconnected_cb,
                                  max_reconnect_attempts=-1)

    async def message_handler(msg):
        # subject = msg.subject
        # reply = msg.reply
        data = msg.data.decode()
        ret, info = cmd_controller.handle_command(data)

        status = f"""{{"status":"{ret}",
                       "mesh_status":"{cmd_controller.status}",
                       "info":"{info}"}}"""

        await msg.respond(status.encode("utf-8"))

    # Subscribe messages
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
