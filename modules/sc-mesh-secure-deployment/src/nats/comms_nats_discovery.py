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
from pyroute2 import IPRoute
from zeroconf import Zeroconf, IPVersion, ServiceInfo
from src.comms_service_discovery import CommsServiceMonitor
import signal
import atexit
import os


#
# Class used to update nats server configuration and restart the server.
# Shared functionality for seed and leaf nodes.
#
class NatsServerCtrl:

    SERVICE_TYPE: str = '_natsseed._tcp.local.'
    SERVICE_NAME: str = 'NATSSeed_{identifier}'
    SEED_SERVICE_PORT: int = 7422
    NATS_PORT: int = 4222
    GROUNDSTATION_ROLE: str = "gcs"

    class IPvUsage(Enum):
        AnyIPv = 1
        IPv4Only = 2
        IPv6Only = 3

        def ipv4Enabled(self) -> bool:
            return bool(self == self.AnyIPv or self == self.IPv4Only)

        def ipv6Enabled(self) -> bool:
            return bool(self == self.AnyIPv or self == self.IPv6Only)

    def __init__(self, logger: Logger, key, cert, servercert, ca, ipvUsage: IPvUsage):
        self.__logger = logger
        self.__key = key
        self.__cert = cert
        self.__server_cert = servercert
        self.__cert_authority = ca
        self.__tls_required = False
        self.ipvUsage = ipvUsage

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
    
    # generates seed config
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

        with open('/var/run/nats.conf', 'w',  encoding='UTF-8') as file_nats_conf:
            file_nats_conf.write(config)

    # generates leef config with given seeds.
    def generate_leaf_config(self, seeds) -> None:
        """
        Generate the nats-server configuration file for the leaf node.
        :param seeds: seeds to be added dict{seed_name, list[ipaddr_str]}
        :return: None
        """
        if self.__tls_required:
            protocol = "tls"
            remoteCfg = ""
            for seedUrls in seeds.values():
                urls = [seed.replace('$PROTOCOL$', protocol) for seed in seedUrls]
                remoteCfg += f"""
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
                    remotes = [{remoteCfg}
                    ]
                }}
            """)
        else:
            protocol = "nats"
            remoteCfg = ""
            for seedUrls in seeds.values():
                urls = [seed.replace('$PROTOCOL$', protocol) for seed in seedUrls]
                remoteCfg += f"""
                    {{
                        urls: {urls}
                    }},"""
                
            config = textwrap.dedent(f"""
                listen: 0.0.0.0:{NatsServerCtrl.NATS_PORT}
                leafnodes {{
                    remotes = [{remoteCfg}
                    ]
                }}
            """)

        # check if config is really modified, to prevent unnecessary nats server restart.
        modified = True
        configPath = '/var/run/nats.conf'
        if (os.path.isfile(configPath)):
            modified = open(configPath, 'r', encoding='UTF-8').read() != config
        # write only if modded.
        if (modified):
            self.__logger.debug(f"Write fresh configs to {configPath}")
            with open('/var/run/nats.conf', 'w',  encoding='UTF-8') as file_nats_conf:
                file_nats_conf.write(config)
        else:
            self.__logger.debug("Config file was not modified, skip.")

        return modified

    # generate nats config with given seeds and restart the server
    def update_configurations_and_restart(self, seeds) -> None:
        """
        Update the nats-server configuration and restart the nats-server.
        :param ip_address: ip address of the seed node
        :return: None
        """
        self.__logger.debug("Updating configurations and reloading NATS-server configuration if needed.")
        ret = self.generate_leaf_config(seeds)

        # reload nats-server configuration
        if (ret):
            self.__logger.debug("Restart NATS-server")
            ret = self.reload_config()
            if ret != 0:
                self.__logger.error("Error reloading nats-server configuration")


#
# Implements mDNS discovery for nats leaf nodes to actively search for seed nodes.
#
class NatsSeedDiscovery:

    def __init__(self, natsCtrl: NatsServerCtrl, logger: Logger):
        self.__natsCtrl = natsCtrl
        self.__logger = logger
        self.__seeds = {}

    # callback handler called when CommsServiceDiscovery receives seed service update
    def __seed_service_callback(self, service_name, ipv4_addresses, ipv6_addresses, port, status, **kwargs):
        self.__logger.debug(f"Seed service callback: name:{service_name}, ip:{ipv4_addresses}, ip6:{ipv6_addresses}, port:{port}, online: {status}")

        # add new leaf
        if (status):
            uris = []
            for ip in ipv4_addresses:
                uris.append(f"$PROTOCOL$://{ip}:{port}")
            for ip in ipv6_addresses:
                uris.append(f"$PROTOCOL$://[{ip}]:{port}")

            self.__seeds[service_name] = uris
            self.__natsCtrl.update_configurations_and_restart(self.__seeds)
        # disconnected leafs are not removed from config as they often are just temporarily out.

    # starts to monitor services and updates nats.conf when new seed is found
    def run(self):
        # generate empty leaf config so nats-server can start-up
        self.__natsCtrl.generate_leaf_config({})

        self.__logger.debug("Searching seed services.")
        monitor = CommsServiceMonitor(
            service_name=None,
            service_type=self.__natsCtrl.SERVICE_TYPE,
            service_cb=self.__seed_service_callback
        )
        monitor.run()


#
# Implements mDNS service publish for nats seed nodes to publish the service for leaf nodes.
#
class NatsSeedPublish:

    def __init__(self, natsCtrl: NatsServerCtrl, logger: Logger, ifName):
        self.__zeroconf: Zeroconf = None
        self.__serviceInfo: ServiceInfo = None
        self.__seed_ipv4_addr = None
        self.__seed_ipv6_addr = None

        self.__logger = logger
        self.__natsCtrl = natsCtrl
        self.__ifName = ifName
        
        # fetch ipv4 addr even if ipv6 only, need to get device index from it.
        ipv4 = self.__get_ip4_address(ifName)
        if not ipv4:
            raise ValueError(f"Network interface {ifName} does not found.")

        # set ipv4 and/or ipv6 according the flags
        if self.__natsCtrl.ipvUsage.ipv4Enabled():
            self.__seed_ipv4_addr = ipv4.get_attr('IFA_ADDRESS')
        if self.__natsCtrl.ipvUsage.ipv6Enabled():
            ipv6 = self.__get_ip6_address(ipv4['index'])
            if not ipv6:
                self.__seed_ipv6_addr = None
            else:    
                self.__seed_ipv6_addr = ipv6.get_attr('IFA_ADDRESS')

        # validate that we got the addresses we want to publish
        if self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv4Only:
            if not self.__seed_ipv4_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip4 address.")
        elif self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv6Only:
            if not self.__seed_ipv6_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip6 address.")
        elif self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.AnyIPv:
            if not self.__seed_ipv4_addr and not self.__seed_ipv6_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip4 or ip6 address.")
        
        # get MAC address and calculate md5 from it, will use it as service name GUID.
        mac = self.__getHwAddr(ifName)
        self.uid = hashlib.md5(mac.encode()).hexdigest()
        self.__logger.debug(f"IPv4: {self.__seed_ipv4_addr} IPv6: {self.__seed_ipv6_addr} MAC: {mac} UID:{self.uid}")

    @staticmethod
    def __get_ip4_address(adapterName):
        ip = IPRoute()
        try:
            addr = ip.get_addr(label=adapterName, family=socket.AF_INET)
            return addr[0]
        except IndexError:
            return None

    @staticmethod
    def __get_ip6_address(index) -> str:
        ip = IPRoute()
        try:
            addrs = ip.get_addr(family=socket.AF_INET6, index=index)
            for a in addrs:
                if (a['flags'] == 0):
                    return a
            # no global address found
            return None
        except IndexError:
            return None

    @staticmethod
    def __getHwAddr(ifname) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', bytes(ifname, 'utf-8')[:15]))
        return ':'.join('%02x' % b for b in info[18:24])

    # publish service using Zeroconf
    def __register_seed_service(self):

        if self.__zeroconf is not None or self.__serviceInfo is not None:
            raise SystemError("Service already registered.")

        serviceName = NatsServerCtrl.SERVICE_NAME.format(identifier=self.uid)

        # include ipv4, ipv6 or both addresses.
        addrs = []
        if self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv4Only:
            addrs.append(socket.inet_aton(self.__seed_ipv4_addr))
        elif self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv6Only:
            addrs.append(socket.inet_pton(socket.AF_INET6, self.__seed_ipv6_addr))
        elif self.__natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.AnyIPv:
            addrs.append(socket.inet_aton(self.__seed_ipv4_addr))
            if self.__seed_ipv6_addr:
                addrs.append(socket.inet_pton(socket.AF_INET6, self.__seed_ipv6_addr))

        # publish the service information
        self.__serviceInfo = ServiceInfo(type_=NatsServerCtrl.SERVICE_TYPE,
            name=f'{serviceName}.{NatsServerCtrl.SERVICE_TYPE}',
            addresses=addrs,
            port=NatsServerCtrl.SEED_SERVICE_PORT,
            properties={'description': serviceName})

        self.__zeroconf = Zeroconf(ip_version=IPVersion.All)
        self.__zeroconf.register_service(info=self.__serviceInfo,
                                       cooperating_responders=True)
        
    # removes (unpublish) service from Zeroconf
    def __unregister_seed_service(self):
        self.__logger.debug("unregister_seed_service called")
        if self.__zeroconf is None or self.__serviceInfo is None:
            raise SystemError("Service NOT registered.")
        self.__zeroconf.unregister_service(self.__serviceInfo)
        self.__zeroconf.close()

    def __exit_handler(self):
        self.__unregister_seed_service()

    def __kill_handler(self, *args):
        exit(0)

    # keeps the service published until script is stopped
    def run(self):
        self.__logger.debug("Publish seed service.")
        self.__natsCtrl.generate_seed_config()
        self.__natsCtrl.reload_config()

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

    if (args.ipv4 and args.ipv6):
        raise ValueError("Both --ipv4 and --ipv6 can not be defined.")

    # initialize NatsServerCtrl
    ipvUsage = NatsServerCtrl.IPvUsage.AnyIPv
    if (args.ipv4):
        ipvUsage = NatsServerCtrl.IPvUsage.IPv4Only
    if (args.ipv6):
        ipvUsage = NatsServerCtrl.IPvUsage.IPv6Only

    natsCtrl = NatsServerCtrl(logger, args.key, args.cert, args.servercert, args.ca, ipvUsage)

    # start seed publish or seed discovery according the role.
    if (args.role == NatsServerCtrl.GROUNDSTATION_ROLE):
        publisher = NatsSeedPublish(natsCtrl, logger, args.ifname)
        publisher.run()
    else:
        discovery = NatsSeedDiscovery(natsCtrl, logger)
        discovery.run()
