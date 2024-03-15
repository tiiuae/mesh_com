"""
This module is used to discover nats seed nodes and to publish nats seed node service.
"""

import socket
import subprocess
import time
import logging
import fcntl
import struct
import hashlib
from logging import Logger
import argparse
import textwrap
from enum import Enum
import signal
import atexit
import os
import sys
import ipaddress
from pyroute2 import IPRoute
from zeroconf import Zeroconf, IPVersion, ServiceInfo
from src.comms_service_discovery import CommsServiceMonitor

NATS_CONFIG_PATH = "/var/run/nats.conf"


class NatsServerCtrl:
    """
    # Class used to update nats server configuration and restart the server.
    # Shared functionality for seed and leaf nodes.
    #
    """
    SERVICE_TYPE: str = '_natsseed._tcp.local.'
    SERVICE_NAME: str = 'NATSSeed_{identifier}'
    SEED_SERVICE_PORT: int = 7422
    NATS_PORT: int = 4222
    GROUNDSTATION_ROLE: str = "gcs"

    class IPvUsage(Enum):
        """
        Enum for IPv usage
        """
        ANY_IPV = 1
        IPV4_ONLY = 2
        IPV6_ONLY = 3

        def ipv4_enabled(self) -> bool:
            """
            Check if IPv4 is enabled
            :return: True if IPv4 is enabled, False otherwise
            """
            return self in (self.ANY_IPV, self.IPV4_ONLY)

        def ipv6_enabled(self) -> bool:
            """
            Check if IPv6 is enabled
            :return: True if IPv6 is enabled, False otherwise
            """
            return self in (self.ANY_IPV,self.IPV6_ONLY)

    #pylint: disable=too-many-arguments
    def __init__(self, _logger: Logger, key, cert, server_cert,
                 cert_authority, ipv_usage: IPvUsage):
        self.__logger = _logger
        self.__key = key
        self.__cert = cert
        self.__server_cert = server_cert
        self.__cert_authority = cert_authority
        self.__tls_required = False
        self.ipv_usage = ipv_usage

        # set __tls_required flag if all the tls parameters are given
        if self.__key is not None and self.__cert is not None \
           and self.__server_cert is not None and self.__cert_authority is not None:
            self.__tls_required = True

    # restarts nats server
    def reload_config(self) -> int:
        """
        nats-server configuration reload.
        :return: 0 if successful, -1 otherwise
        """
        try:
            ret = subprocess.run(["/opt/S90nats_server", "restart"],
                                    shell=False, check=True, capture_output=True)
            if ret.returncode != 0:
                self.__logger.error("Default mesh starting failed ")
                return -1

        except:
            self.__logger.error("Default mesh starting failed ")
            return -1

        return 0

    def generate_seed_config(self) -> None:
        """
        Generate the nats-server configuration file for the seed node.
        :return: None
        """

        if self.__tls_required:
            config = textwrap.dedent(f"""
                listen: 0.0.0.0:{NatsServerCtrl.NATS_PORT}
                tls {{
                    key_file: {self.__key}
                    cert_file: {self.__server_cert}
                    ca_file: {self.__cert_authority}
                    verify: true
                }}
                leafnodes {{
                    port: {NatsServerCtrl.SEED_SERVICE_PORT}
                    tls {{
                        key_file: {self.__key}
                        cert_file: {self.__server_cert}
                        ca_file: {self.__cert_authority}
                        verify: true
                    }}
                }}
            """)
        else:
            config = textwrap.dedent(f"""
                listen: 0.0.0.0:{NatsServerCtrl.NATS_PORT}
                leafnodes {{
                    port: {NatsServerCtrl.SEED_SERVICE_PORT}
                }}
            """)

        with open(NATS_CONFIG_PATH, 'w',  encoding='UTF-8') as file_nats_conf:
            file_nats_conf.write(config)

    def generate_leaf_config(self, seeds) -> bool:
        """
        Generate the nats-server configuration file for the leaf node.
        :param seeds: seeds to be added dict{seed_name, list[ipaddr_str]}
        :return: None
        """
        if self.__tls_required:
            protocol = "tls"
            remote_cfg = ""
            for seed_urls in seeds.values():
                urls = [seed.replace('$PROTOCOL$', protocol) for seed in seed_urls]
                remote_cfg += f"""
                    {{
                        urls: {urls}
                        tls {{
                            key_file: {self.__key}
                            cert_file: {self.__cert}
                            ca_file: {self.__cert_authority}
                            verify: true
                        }}
                    }},"""

            config = textwrap.dedent(f"""
                listen: 0.0.0.0:{NatsServerCtrl.NATS_PORT}
                tls {{
                    key_file: {self.__key}
                    cert_file: {self.__server_cert}
                    ca_file: {self.__cert_authority}
                    verify: true
                }}
                leafnodes {{
                    remotes = [{remote_cfg}
                    ]
                }}
            """)
        else:
            protocol = "nats"
            remote_cfg = ""
            for seed_urls in seeds.values():
                urls = [seed.replace('$PROTOCOL$', protocol) for seed in seed_urls]
                remote_cfg += f"""
                    {{
                        urls: {urls}
                    }},"""

            config = textwrap.dedent(f"""
                listen: 0.0.0.0:{NatsServerCtrl.NATS_PORT}
                leafnodes {{
                    remotes = [{remote_cfg}
                    ]
                }}
            """)

        # check if config is really modified, to prevent unnecessary nats server restart.
        modified = True
        if os.path.isfile(NATS_CONFIG_PATH):
            modified = open(NATS_CONFIG_PATH, 'r', encoding='UTF-8').read() != config
        # write only if modded.
        if modified:
            self.__logger.debug(f"Write fresh configs to {NATS_CONFIG_PATH}")
            with open(NATS_CONFIG_PATH, 'w',  encoding='UTF-8') as file_nats_conf:
                file_nats_conf.write(config)
        else:
            self.__logger.debug("Config file was not modified, skip.")

        return modified

    def update_configurations_and_restart(self, seeds) -> None:
        """
        Update the nats-server configuration and restart the nats-server.
        :param seeds: seeds to be added dict{seed_name, list[ipaddr_str]}
        :return: None
        """
        self.__logger.debug("Updating configurations and reloading "
                            "NATS-server configuration if needed.")
        ret = self.generate_leaf_config(seeds)

        # reload nats-server configuration
        if ret:
            self.__logger.debug("Restart NATS-server")
            ret = self.reload_config()
            if ret != 0:
                self.__logger.error("Error reloading nats-server configuration")

#pylint: disable=too-few-public-methods
class NatsSeedDiscovery:
    """
    Implements mDNS discovery for nats leaf nodes to actively search for seed nodes.
    """
    def __init__(self, nats_ctrl: NatsServerCtrl, _logger: Logger):
        self.__nats_ctrl = nats_ctrl
        self.__logger = _logger
        self.__seeds = {}

    #pylint: disable=too-many-arguments, unused-argument
    def __seed_service_callback(self, service_name, ipv4_addresses, ipv6_addresses,
                                port, status, **kwargs):
        """
        callback handler called when CommsServiceDiscovery receives seed service update
        """
        self.__logger.debug(f"Seed service callback: name:{service_name}, "
                            f"ip:{ipv4_addresses}, ip6:{ipv6_addresses}, "
                            f"port:{port}, online: {status}")

        # add new leaf
        if status:
            uris = []
            for ip in ipv4_addresses:
                uris.append(f"$PROTOCOL$://{ip}:{port}")
            for ip in ipv6_addresses:
                uris.append(f"$PROTOCOL$://[{ip}]:{port}")

            self.__seeds[service_name] = uris
            self.__nats_ctrl.update_configurations_and_restart(self.__seeds)
        # disconnected leafs are not removed from config as they often are just temporarily out.

    def run(self):
        """
        # starts to monitor services and updates nats.conf when new seed is found
        # generate empty leaf config so nats-server can start-up
        """
        self.__nats_ctrl.generate_leaf_config({})

        self.__logger.debug("Searching seed services.")
        monitor = CommsServiceMonitor(
            service_name=None,
            service_type=self.__nats_ctrl.SERVICE_TYPE,
            service_cb=self.__seed_service_callback
        )
        monitor.run()

#pylint: disable=too-many-instance-attributes
class NatsSeedPublish:
    """
    Implements mDNS service publish for nats seed nodes to publish the service for leaf nodes.
    """
    def __init__(self, nats_ctrl: NatsServerCtrl, _logger: Logger, if_name):
        self.__zeroconf: Zeroconf = None
        self.__service_info: ServiceInfo = None
        self.__seed_ipv4_addr = None
        self.__seed_ipv6_addr = None

        self.__logger = _logger
        self.__nats_ctrl = nats_ctrl
        self.__if_name = if_name

        # fetch ipv4 addr even if ipv6 only, need to get device index from it.
        ipv4 = self.__get_ip4_address(if_name)
        if not ipv4:
            raise ValueError(f"Network interface {if_name} does not found.")

        with IPRoute() as ip:
            link_local_address = None
            # Find the current IPv6 link-local address
            index = ip.link_lookup(ifname=if_name)[0]
            addresses = ip.get_addr(index=index)

            for addr in addresses:
                ip_address = addr.get_attrs('IFA_ADDRESS')[0]
                prefix_length = int(addr['prefixlen'])
                if self.__is_valid_ipv6((ip_address, prefix_length)):
                    link_local_address = ip_address
                    break

            if not link_local_address and self.__nats_ctrl.ipv_usage.ipv6_enabled():
                self.__logger.debug(
                    f"Link-local IPv6 address not found for interface {if_name}")

        # set ipv4 and/or ipv6 according the flags
        if self.__nats_ctrl.ipv_usage.ipv4_enabled():
            self.__seed_ipv4_addr = ipv4.get_attr('IFA_ADDRESS')
        if self.__nats_ctrl.ipv_usage.ipv6_enabled():
            if not ip_address:
                self.__seed_ipv6_addr = None
            else:
                self.__seed_ipv6_addr = ip_address

        # validate that we got the addresses we want to publish
        if self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.IPV4_ONLY:
            if not self.__seed_ipv4_addr:
                raise ValueError(f"bridge {if_name} does not have valid ip4 address.")
        elif self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.IPV6_ONLY:
            if not self.__seed_ipv6_addr:
                raise ValueError(f"bridge {if_name} does not have valid ip6 address.")
        elif self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.ANY_IPV:
            if not self.__seed_ipv4_addr and not self.__seed_ipv6_addr:
                raise ValueError(f"bridge {if_name} does not have valid ip4 or ip6 address.")

        # get MAC address and calculate md5 from it, will use it as service name GUID.
        mac = self.__getHwAddr(if_name)
        self.uid = hashlib.md5(mac.encode()).hexdigest()
        self.__logger.debug(f"IPv4: {self.__seed_ipv4_addr} "
                            f"IPv6: {self.__seed_ipv6_addr} "
                            f"MAC: {mac} UID:{self.uid}")

    def __is_valid_ipv6(self, address: (str, int))-> bool:
        """
        Check if the address is a valid IPv6 address

        :param address: The address to check
        :type address: (str, int)
        :return: True if the address is a valid IPv6 address, False otherwise
        """
        try:
            ip = ipaddress.ip_address(address[0])
            return (ip.version == 6 and
                    (ip.compressed.startswith("fdd8:")) and
                    address[1] == 64)
        except Exception as e:
            self.__logger.error(e)
            return False

    @staticmethod
    def __get_ip4_address(adapter_name):
        """
        Get the IPv4 address of the specified adapter
        :param adapter_name: The name of the adapter
        :type adapter_name: str
        :return: The IPv4 address of the specified adapter
        """
        ip = IPRoute()
        try:
            addr = ip.get_addr(label=adapter_name, family=socket.AF_INET)
            return addr[0]
        except IndexError:
            return None

    @staticmethod
    def __getHwAddr(ifname) -> str:
        """
        Get the MAC address of the specified interface
        :param ifname: The name of the interface
        :type ifname: str
        :return: The MAC address of the specified interface
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', bytes(ifname, 'utf-8')[:15]))
        return ':'.join('%02x' % b for b in info[18:24])

    def __register_seed_service(self) -> None:
        """
        publish service using Zeroconf
        """
        if self.__zeroconf is not None or self.__service_info is not None:
            raise SystemError("Service already registered.")

        service_name = NatsServerCtrl.SERVICE_NAME.format(identifier=self.uid)

        # include ipv4, ipv6 or both addresses.
        addrs = []
        if self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.IPV4_ONLY:
            addrs.append(socket.inet_aton(self.__seed_ipv4_addr))
        elif self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.IPV6_ONLY:
            addrs.append(socket.inet_pton(socket.AF_INET6, self.__seed_ipv6_addr))
        elif self.__nats_ctrl.ipv_usage == NatsServerCtrl.IPvUsage.ANY_IPV:
            addrs.append(socket.inet_aton(self.__seed_ipv4_addr))
            if self.__seed_ipv6_addr:
                addrs.append(socket.inet_pton(socket.AF_INET6, self.__seed_ipv6_addr))

        # publish the service information
        self.__service_info = ServiceInfo(type_=NatsServerCtrl.SERVICE_TYPE,
                                          name=f'{service_name}.{NatsServerCtrl.SERVICE_TYPE}',
                                          addresses=addrs,
                                          port=NatsServerCtrl.SEED_SERVICE_PORT,
                                          properties={'description': service_name})

        self.__zeroconf = Zeroconf(ip_version=IPVersion.All)
        self.__zeroconf.register_service(info=self.__service_info,
                                         cooperating_responders=True)
        
    def __unregister_seed_service(self):
        """
        removes (unpublish) service from Zeroconf
        """
        self.__logger.debug("unregister_seed_service called")
        if self.__zeroconf is None or self.__service_info is None:
            raise SystemError("Service NOT registered.")
        self.__zeroconf.unregister_service(self.__service_info)
        self.__zeroconf.close()

    def __exit_handler(self):
        """
        Exit handler to unregister the service
        """
        self.__unregister_seed_service()

    #pylint: disable=unused-argument
    def __kill_handler(self, *args):
        """
        Kill handler to unregister the service
        """
        sys.exit(0)

    def run(self):
        """
        keeps the service published until script is stopped
        """
        self.__logger.debug("Publish seed service.")
        self.__nats_ctrl.generate_seed_config()
        self.__nats_ctrl.reload_config()

        atexit.register(self.__exit_handler)
        signal.signal(signal.SIGTERM, self.__kill_handler)
        signal.signal(signal.SIGINT, self.__kill_handler)

        self.__register_seed_service()
        while True:
            time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NATS Discovery')
    parser.add_argument('-r', '--role', help='device role', required=True)
    parser.add_argument('-k', '--key', help='key file', required=False)
    parser.add_argument('-c', '--cert', help='client cert file', required=False)
    parser.add_argument('-s', '--servercert', help='server cert file', required=False)
    parser.add_argument('-a', '--ca', help='certificate authority file', required=False)
    parser.add_argument("-i", '--ifname', help='nic adapter name', default='br-lan', required=False)
    parser.add_argument('--ipv4', help='use ipv4 addresses only.', action="store_true")
    parser.add_argument('--ipv6', help='use ipv6 addresses only.', action="store_true")
    args = parser.parse_args()

    # base logger for discovery
    main_logger = logging.getLogger("nats")
    main_logger.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter(
        fmt='%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s')
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(log_formatter)
    main_logger.addHandler(console_handler)

    # logger for this module and derived from main logger
    logger = main_logger.getChild("discovery")

    if args.ipv4 and args.ipv6:
        raise ValueError("Both --ipv4 and --ipv6 can not be defined.")

    # initialize NatsServerCtrl
    ipvUsage = NatsServerCtrl.IPvUsage.ANY_IPV
    if args.ipv4:
        ipvUsage = NatsServerCtrl.IPvUsage.IPV4_ONLY
    if args.ipv6:
        ipvUsage = NatsServerCtrl.IPvUsage.IPV6_ONLY

    natsCtrl = NatsServerCtrl(logger, args.key, args.cert, args.servercert, args.ca, ipvUsage)

    # start seed publish or seed discovery according the role.
    if args.role == NatsServerCtrl.GROUNDSTATION_ROLE:
        publisher = NatsSeedPublish(natsCtrl, logger, args.ifname)
        publisher.run()
    else:
        discovery = NatsSeedDiscovery(natsCtrl, logger)
        discovery.run()
