"""
mesh executor with root rights
"""
import json
from shlex import quote
import socket
import subprocess
import syslog
import select
import threading

try:
    # in deb import
    from .src.batstat import Batman, STATUS
    from .src.batadvvis import BatAdvVis
    from .src.socket_helper import recv_msg, send_msg
except ImportError:
    # if executed as is .py
    from src.batstat import Batman, STATUS
    from src.batadvvis import BatAdvVis
    from src.socket_helper import recv_msg, send_msg


class MeshNetwork:
    """
    Mesh network executor
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
            # self.enc = ""

    def __init__(self):
        self.settings = self.MeshSettings()
        self.host = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.port = 33221  # Port to listen on (non-privileged ports are > 1023)
        self.script_path_1 = "/opt/ros/galactic/share/bin/mesh-ibss.sh"
        self.script_path_2 = "/opt/ros/galactic/share/bin/mesh-11s.sh"
        self.batman_visual = BatAdvVis()
        self.batman = Batman()
        self.status = STATUS()

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
            self.settings.api_version = int(parameters["api_version"])
            self.settings.ssid = str(parameters["ssid"])
            self.settings.key = str(parameters["key"])
            self.settings.ap_mac = str(parameters["ap_mac"])
            self.settings.country = str(parameters["country"]).lower()
            self.settings.frequency = str(parameters["frequency"])
            self.settings.ip_address = str(parameters["ip"])
            self.settings.subnet = str(parameters["subnet"])
            self.settings.tx_power = str(parameters["tx_power"])
            self.settings.mode = str(parameters["mode"])
            # self.settings.enc = str(parameters["enc"])
            self.__change_configuration()
        except (json.decoder.JSONDecodeError, KeyError,
                TypeError, AttributeError) as error:
            syslog.syslog("JSON format not correct")
            syslog.syslog(str(error))

    def __change_configuration(self):
        # api 1, ad-hoc
        # api 2, 11s

        self.batman.status = self.status.ongoing

        if int(self.settings.api_version) == 1:
            path = self.script_path_1
        else:
            path = self.script_path_2

        if self.settings.mode == "ap":
            subprocess.call([path, quote(self.settings.mode)])
            self.batman.status = self.status.accesspoint
        elif self.settings.mode == "off":
            subprocess.call([path, quote(self.settings.mode)])
            self.batman.status = self.status.off
        elif self.settings.mode == "mesh":
            subprocess.call([path, quote(self.settings.mode),
                             quote(self.settings.ip_address),
                             quote(self.settings.subnet),
                             quote(self.settings.ap_mac),
                             quote(self.settings.key),
                             quote(self.settings.ssid),
                             quote(self.settings.frequency),
                             quote(self.settings.tx_power),
                             quote(self.settings.country)])
            self.batman.status = self.status.mesh

        syslog.syslog('Setting Done')

    def __handle_report_request(self):
        return f"[{self.batman_visual.latest_topology},{self.batman.latest_stat}]". \
            replace(": ", ":"). \
            replace(", ", ",")

    def run(self):
        """
        Run method for executor

        :return: never
        """
        connection_list = []
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        server.bind((self.host, self.port))
        server.listen(10)
        connection_list.append(server)

        self.batman.status = self.status.no_config

        t1 = threading.Thread(target=self.batman_visual.run)
        t1.start()
        t2 = threading.Thread(target=self.batman.run)
        t2.start()

        while True:

            if not t1.is_alive():
                t1 = threading.Thread(target=self.batman_visual.run)
                t1.start()
            if not t2.is_alive():
                t2 = threading.Thread(target=self.batman.run)
                t2.start()

            read_sockets, write_sockets, error_sockets = select.select(connection_list, [], [], 1)

            for sock in read_sockets:
                if sock == server:
                    # new connection received through server s socket
                    sockfd, address = server.accept()
                    connection_list.append(sockfd)
                    syslog.syslog("Client ({}, {}) connected".format(address[0], address[1]))
                else:
                    try:
                        data = recv_msg(sock)
                        if data:
                            data = data.decode('utf-8')
                            if "report\n" in data:
                                # syslog.syslog(str("r:" + data))
                                # send_msg adds 4 byte length prefix
                                send_msg(sock, self.__handle_report_request().encode())
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
    """
    main
    """
    mesh_network = MeshNetwork()
    mesh_network.run()


if __name__ == '__main__':
    main()
