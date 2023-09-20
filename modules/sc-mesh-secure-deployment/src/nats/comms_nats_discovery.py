import socket
import subprocess
import re
import time
import logging
import argparse
import netifaces as netifaces

class NatsDiscovery:  # pylint: disable=too-few-public-methods
    """
    Nats Discovery class.  Utilizes the batctl command to discover devices on the mesh network.
    """
    def __init__(self, role, key, cert):
        self.role = role
        self.key = key
        self.cert = cert
        self.leaf_port = 7422
        self.seed_ip_address = ""

        # base logger for discovery
        self.main_logger = logging.getLogger("nats")
        self.main_logger.setLevel(logging.DEBUG)
        log_formatter = logging.Formatter(
            fmt='%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s')
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        self.main_logger.addHandler(console_handler)

        # logger for this module and derived from main logger
        self.logger = self.main_logger.getChild("discovery")

    def __generate_seed_config(self) -> None:
        """
        Generate the nats-server configuration file for the seed node.
        :return: None
        """

        config = """
listen: 0.0.0.0:4222
leafnodes {
    port: 7422
}
"""
        with open('/var/run/nats.conf', 'w',  encoding='UTF-8') as file_nats_conf:
            file_nats_conf.write(config)

    @staticmethod
    def __generate_leaf_config(_seed_route) -> None:
        """
        Generate the nats-server configuration file for the leaf node.
        :param _seed_route: seed node route
        :return: None
        """
        config = f"""
listen: 0.0.0.0:4222
leafnodes {{
    remotes = [
        {{
            url: "nats://{_seed_route}"
        }},
    ]
}}
"""
        with open('/var/run/nats.conf', 'w',  encoding='UTF-8') as file_nats_conf:
            file_nats_conf.write(config)

    def __reload_nats_server_config(self) -> int:
        """
        nats-server configuration reload.
        :return: 0 if successful, -1 otherwise
        """
        try:
            ret = subprocess.run(["/opt/S90nats_server", "restart"],
                                 shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                self.logger.error("Default mesh starting failed ")
                return -1

        except:
            self.logger.error("Default mesh starting failed ")
            return -1

        return 0

    def __update_configurations_and_restart(self, ip_address) -> None:
        """
        Update the nats-server configuration and restart the nats-server.
        :param ip_address: ip address of the seed node
        :return: None
        """
        self.logger.debug("Updating configurations and reloading nats-server configuration")
        self.__generate_leaf_config(ip_address)

        # reload nats-server configuration
        ret = self.__reload_nats_server_config()
        if ret != 0:
            self.logger.error("Error reloading nats-server configuration")
            return

    @staticmethod
    def __get_mesh_macs() -> list:
        """
        Get the mac addresses of the devices on the mesh network.
        :return: list of mac addresses
        """
        try:
            ret = subprocess.run(["batctl", "o", "-H"], shell=False,
                                 check=True, capture_output=True)
            if ret.returncode != 0:
                return []
            macs = re.findall(r' \* (([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2}))',
                              ret.stdout.decode('utf-8'))
            return [mac[0] for mac in macs]
        except:
            return []

    @staticmethod
    def __mac_to_ip(mac) -> str:
        """
        Convert mac address to ip address.
        :param mac: mac address
        :return: ip address
        """
        ip_br_lan = netifaces.ifaddresses('br-lan')[netifaces.AF_INET][0]['addr'].split(".")[0:-1]
        ip_br_lan = ".".join(ip_br_lan) + "."
        return ip_br_lan + str(int(mac.split(":")[-1],16))

    @staticmethod
    def __scan_port(host, port) -> bool:
        """
        Scan the port of the host.
        :param host: host ip address
        :param port: port to scan
        :return: True if port is open, False otherwise
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.2)  # Adjust the timeout as needed
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0

    def run(self):
        """
        Run the discovery process.
        :return: None
        """
        # amount = 0
        gcs_found = 0
        self.logger.debug("Find devices on mesh network...")

        if self.role == "gcs":
            self.__generate_seed_config()
            self.__reload_nats_server_config()
            return
        # create temporary leaf configuration for nats-server to start
        self.__generate_leaf_config("192.168.1.2")

        while True:
            macs = self.__get_mesh_macs()
            self.logger.debug(f"{macs} len: {len(macs)}")

            for mac in macs:
                ip_address = self.__mac_to_ip(mac)
                if self.seed_ip_address == "":
                    self.logger.debug(f"Scanning {ip_address}, {mac}")
                    if self.__scan_port(ip_address, self.leaf_port):
                        self.seed_ip_address = ip_address
                        gcs_found = 1

            if gcs_found:
                # generate configuration files
                self.logger.debug(f"Found {self.seed_ip_address}")
                self.__update_configurations_and_restart(self.seed_ip_address)
                return

            time.sleep(4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NATS Discovery')
    parser.add_argument('-r', '--role', help='device role', required=True)
    parser.add_argument('-k', '--key', help='key file', required=False)
    parser.add_argument('-c', '--cert', help='cert file', required=False)
    args = parser.parse_args()

    forrest = NatsDiscovery(args.role, args.key, args.cert)
    forrest.run()
