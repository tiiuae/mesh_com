"""
mesh setting nats node
"""
import json
import subprocess
from shlex import quote
import asyncio
import signal
import ssl
import argparse
from nats.aio.client import Client as NATS


class STATUS:  # pylint: disable=too-few-public-methods
    """
    Status values
    """
    no_status = "MESH_NO_STATUS"  # no stat available
    mesh_default = "MESH_DEFAULT"  # mesh default settings
    mesh_default_connected = "MESH_DEFAULT_CONNECTED"  # mesh default connected
    mesh_configuration_stored = "MESH_CONFIGURATION_STORED"  # mesh configuration stored
    mesh_mission_not_connected = "MESH_MISSION_NOT_CONNECTED"  # mesh missiong connected
    mesh_mission_connected = "MESH_MISSION_CONNECTED"  # mesh mission connected
    mesh_fail = "MESH_FAIL"  # fails


class MeshNetwork:
    """
    Mesh network
    """

    class MeshSettings:  # pylint: disable=too-few-public-methods
        """
        Settings class
        """

        def __init__(self):
            self.api_version = 1
            self.ssid = ""
            self.key = ""
            self.ap_mac = ""
            self.country = ""
            self.frequency = ""
            self.ip_address = ""
            self.subnet = ""
            self.tx_power = ""
            self.mode = ""

    def __init__(self):
        self.settings = self.MeshSettings()
        self.mesh_params = ""

    def handle_mesh_settings(self, msg:str) -> (str, str):
        try:
            parameters = json.loads(msg)
            print(parameters)
            self.settings.api_version = int(parameters["api_version"])
            self.settings.ssid = quote(str(parameters["ssid"]))
            self.settings.key = quote(str(parameters["key"]))
            self.settings.ap_mac = quote(str(parameters["ap_mac"]))
            self.settings.country = quote(str(parameters["country"]).lower())
            self.settings.frequency = quote(str(parameters["frequency"]))
            self.settings.ip_address = quote(str(parameters["ip"]))
            self.settings.subnet = quote(str(parameters["subnet"]))
            self.settings.tx_power = quote(str(parameters["tx_power"]))
            self.settings.mode = quote(str(parameters["mode"]))
            ret, ret_msg = self.__change_configuration()

        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            print("JSON format not correct" + str(error))
            ret, ret_msg = STATUS.mesh_fail, "JSON format not correct" + str(error)

        return ret, ret_msg

    def __change_configuration(self) -> (str, str):
        return_code = subprocess.call(["cp", "/opt/mesh.conf", "/opt/mesh.conf_backup"]
                                      , shell=False)
        if return_code != 0:
            print("mesh.conf backup copy failed " + str(return_code))
            return STATUS.mesh_fail, "mesh.conf backup copy failed " + str(return_code)

        # todo parameters check

        try:
            with open("/opt/mesh.conf","w", encoding="utf-8") as mesh_conf:
                mesh_conf.write(f"MODE = {quote(self.settings.mode)}\n")
                mesh_conf.write("IP = 10.20.15.3\n")
                mesh_conf.write("MASK = 255.255.255.0\n")
                mesh_conf.write(f"MAC = {quote(self.settings.ap_mac)}\n")
                mesh_conf.write(f"KEY = {quote(self.settings.key)}\n")
                mesh_conf.write(f"ESSID = {quote(self.settings.ssid)}\n")
                mesh_conf.write(f"FREQ = {quote(self.settings.frequency)}\n")
                mesh_conf.write(f"TXPOWER = {quote(self.settings.tx_power)}\n")
                mesh_conf.write(f"COUNTRY = {quote(self.settings.country).upper()}\n")
                mesh_conf.write("MESH_VIF = wlp1s0\n")
                mesh_conf.write("PHY = phy0\n")
                mesh_conf.write("# CONCURRENCY configuration\n")
                mesh_conf.write("# CONCURRENCY=ap+mesh\n")
                mesh_conf.write("# MCC_CHANNEL=2412\n")
        except:
            print("not able to write new mesh.conf")
            return STATUS.mesh_fail, "not able to write new mesh.conf"

        return_code = subprocess.call(["/opt/S9011sMesh", "stop;", "/opt/S9011sMesh", "start;"],
                                      shell=False)
        if return_code != 0:
            print("mesh process restart failed " + str(return_code))
            return STATUS.mesh_fail, "mesh process restart failed " + str(return_code)

        print('Setting Done')
        return "OK", "OK"


async def main(server, port, keyfile=None, certfile=None):
    """
    main
    """
    mesh_network = MeshNetwork()

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
        asyncio.get_running_loop().add_signal_handler(getattr(signal, sig), signal_handler)

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
                                  ssl=ssl_context,
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
        ret, ret_msg = mesh_network.handle_mesh_settings(data)
        status = f"""{{"status":"{ret}","info":"{ret_msg}"}}"""
        await msg.respond(status.encode("utf-8"))

    await nats_client.subscribe("mesh.settings", "workers", message_handler)

    print("Mesh Settings Listening for requests")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mesh Settings')
    parser.add_argument('-s', '--server', help='Server IP', required=True)
    parser.add_argument('-p', '--port', help='Server port', required=True)
    parser.add_argument('-k', '--keyfile', help='TLS keyfile', required=False)
    parser.add_argument('-c', '--certfile', help='TLS certfile', required=False)
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(main(args.server, args.port, args.keyfile, args.certfile))
    loop.run_forever()
    loop.close()
