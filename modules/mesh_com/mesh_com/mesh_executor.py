import json
# import subprocess
from shlex import quote
from time import sleep
import socket
import subprocess


# work in progress

# class Batman:
#     """
#     Batman mesh class
#     """
#     def __init__(self):
#         self.topology = {}
#
#     def update_topology_data(self):
#         """
#         Update topology as JSON
#
#         :return: self.topology as str
#         """
#         self.topology = {'my_mac': '', 'devices': ''}
#         device_template = {'a': '',   # active
#                            'o': '',   # originator
#                            'ls': '',  # last-seen
#                            'q': '',   # quality
#                            'nh': ''}  # next-hop
#
#         route = []
#
#         try:
#             proc = subprocess.Popen(['batctl', 'o'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#
#             for line in proc.stdout:
#                 device = dict(device_template)
#                 aux = line.split()
#
#                 if b"B.A.T" in line:
#                     self.topology["my_mac"] = aux[4].decode("utf-8").split("/")[1]
#                 elif b"Originator" not in aux[0]:
#
#                     # active route check
#                     if b"*" in aux[0]:
#                         index = 1
#                     else:
#                         index = 0
#
#                     device['a'] = str(index)
#                     device['o'] = aux[0 + index].decode("utf-8")
#                     device['ls'] = aux[1 + index].decode("utf-8").replace("s", "")
#                     device['q'] = aux[2 + index].decode("utf-8").replace(')', '').replace('(', '')
#                     device['nh'] = aux[3 + index].decode("utf-8")
#                     route.append(device)
#
#             self.topology['devices'] = route
#         except (FileNotFoundError, Exception):
#             self.topology = device_template  # if not succeed, return empty template
#
#         return self.topology
#
#     def get_topology(self):
#         """
#         Get topology as JSON
#
#         :return: self.topology in json compatible str
#         """
#         return "[" + str(self.topology) + "]"


class BatmanVisualisation:
    command = 'batadv-vis'

    @staticmethod
    def remove_interfaces(visual_lines):
        lines = visual_lines.split("\n")
        new_visual = ""
        for line in lines:
            if line and "TT" not in line:
                new_visual += line + "\n"

        while '  ' in new_visual:
            new_visual = new_visual.replace('  ', ' ')

        return new_visual

    def get(self, format_type="dot"):
        if format_type == "dot" or format_type == "jsondoc" or format_type == "json":
            try:
                # returns byte string
                raw_data = subprocess.check_output([self.command, '-f', format_type])
                if raw_data == 255:
                    raw_data = b'NA'
            except (FileNotFoundError, subprocess.CalledProcessError):
                raw_data = b'NA'
        else:
            raw_data = b'NA'
        # return string
        return self.remove_interfaces(raw_data.decode('UTF-8'))


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
        self.PORT = 33221  # Port to listen on (non-privileged ports are > 1023)
        self.batman = BatmanVisualisation()

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
            print('Setting Failed')
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
        print('Setting Done')

    def __handle_report_request(self):
        return self.batman.get()

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
                            elif "report" in data:
                                conn.send((self.__handle_report_request()).encode())
                            else:
                                conn.close()
                                self.__handle_msg(data)
            except:
                pass
            sleep(1)


def main():
    mesh_network = MeshNetwork()
    mesh_network.run()


if __name__ == '__main__':
    main()
