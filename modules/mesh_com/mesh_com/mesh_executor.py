import json
import subprocess
from shlex import quote
from time import sleep
import socket


class MeshNetwork:
    class MeshSettings:
        def __init__(self):
            self.api_version = 1
            self.ssid = ""
            self.key = ""
            self.ap_mac = ""
            self.country = ""
            self.frequency = ""
            self.ip = ""
            self.subnet = ""
            self.tx_power = ""
            self.mode = ""
            # self.enc = ""

    def __init__(self):
        self.settings = self.MeshSettings()
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33221        # Port to listen on (non-privileged ports are > 1023)

    def __handle_msg(self, msg):

        # {
        #     "api_version": 1,                 interface version for future purposes
        #     "ssid": "gold",                   0-32 octets, UTF-8, shlex.quote chars limiting
        #     "key": "foobar",                  key for the network
        #     "ap_mac": "00:11:22:33:44:55",    bssid for mesh network
        #     "country": "UA",                  Country code, sets tx power limits and supported
        #                                       channels
        #     "frequency": "5220",              wifi channel frequency, depends on the country
        #                                       code and HW
        #     "ip": "192.168.1.1",              select unique IP
        #     "subnet": "255.255.255.0",
        #     "tx_power": "30",                 select 30dBm, HW and regulations limiting it
        #                                       correct level.
        #                                       Can be used to set lower dBm levels for testing
        #                                       purposes (e.g. 5dBm)
        #     "mode": "mesh"                    mesh=mesh network, ap=debug hotspot
        # }

        try:
            parameters = json.loads(msg)
            self.settings.ssid = parameters["ssid"]
            self.settings.key = parameters["key"]
            self.settings.ap_mac = parameters["ap_mac"]
            self.settings.country = parameters["country"].lower()
            self.settings.frequency = parameters["frequency"]
            self.settings.ip = parameters["ip"]
            self.settings.subnet = parameters["subnet"]
            self.settings.tx_power = parameters["tx_power"]
            self.settings.mode = parameters["mode"]
            # self.settings.enc = parameters["enc"]
            self.__change_configuration()
        except json.decoder.JSONDecodeError or KeyError or Exception:
            self.get_logger().info('Setting Failed')
            pass

    def __change_configuration(self):
        subprocess.call(["/opt/ros/foxy/share/bin/mesh-ibss.sh", quote(self.settings.mode),
                         quote(self.settings.ip),
                         quote(self.settings.subnet),
                         quote(self.settings.ap_mac),
                         quote(self.settings.key),
                         quote(self.settings.ssid),
                         quote(self.settings.frequency),
                         quote(self.settings.tx_power),
                         quote(self.settings.country)])
        self.get_logger().info('Setting Done')

    def run(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind((self.HOST, self.PORT))
                    s.listen()
                    conn, addr = s.accept()
                    with conn:
                        print('Connected by', addr)
                        while True:
                            data = conn.recv(1024)
                            data = data.decode('utf-8')
                            if not data:
                                break
                            self.__handle_msg(data)
            except:
                pass

            sleep(1)


if __name__ == '__main__':
    mesh_network = MeshNetwork()
    mesh_network.run()
