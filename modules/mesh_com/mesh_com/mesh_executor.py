import json
from shlex import quote
import socket
import subprocess
import syslog
import select


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
        self.script_path_1 = "/opt/ros/foxy/share/bin/mesh-ibss.sh"
        self.script_path_2 = "/opt/ros/foxy/share/bin/mesh-11s.sh"

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
            self.settings.api_version = parameters["api_version"]
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
        except json.decoder.JSONDecodeError or KeyError or Exception:
            syslog.syslog('Setting Failed')
            pass
        self.__change_configuration()

    def __change_configuration(self):
        # api 1, ad-hoc
        # api 2, 11s
        if int(self.settings.api_version) == 1:
            path = self.script_path_1
        else:
            path = self.script_path_2

        if self.settings.mode == "ap" or self.settings.mode == "off":
            subprocess.call([path, quote(self.settings.mode)])
        elif self.settings.mode == "mesh":
            subprocess.call([path, quote(self.settings.mode),
                             quote(self.settings.ip),
                             quote(self.settings.subnet),
                             quote(self.settings.ap_mac),
                             quote(self.settings.key),
                             quote(self.settings.ssid),
                             quote(self.settings.frequency),
                             quote(self.settings.tx_power),
                             quote(self.settings.country)])

        syslog.syslog('Setting Done')

    def __handle_report_request(self):
        return self.batman.get()

    def run(self):

        connection_list = []
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(10)
        connection_list.append(s)

        while True:
            read_sockets, write_sockets, error_sockets = select.select(connection_list, [], [], 1)

            for sock in read_sockets:
                if sock == s:
                    # new connection received through server s socket
                    sockfd, address = s.accept()
                    connection_list.append(sockfd)
                    syslog.syslog("Client ({}, {}) connected".format(address[0], address[1]))
                else:
                    try:
                        data = sock.recv(1024)
                        data = data.decode('utf-8')
                        if "report\n" in data:
                            syslog.syslog(str("r:" + data))
                            sock.sendall((self.__handle_report_request()).encode())
                        elif "ssid" in data and \
                                "api_version" in data:
                            syslog.syslog(str("j:" + data))
                            self.__handle_msg(data)
                        else:
                            _address = sock.getpeername()
                            syslog.syslog("Client ({}, {}) empty and removed".format(
                                _address[0], _address[1]))
                            sock.close()
                            connection_list.remove(sock)
                    except (ConnectionRefusedError, socket.timeout, ConnectionResetError, OSError):
                        syslog.syslog("except")
                        sock.close()
                        if sock in connection_list:
                            connection_list.remove(sock)
                        continue


def main():
    mesh_network = MeshNetwork()
    mesh_network.run()


if __name__ == '__main__':
    main()
