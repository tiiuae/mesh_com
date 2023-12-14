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


#
# Class used to update nats server configuration and restart the server.
# Shared functionality for seed and leaf nodes.
#
class NatsServerCtrl:

    SERVICE_TYPE: str = '_natsseed._tcp.local.'
    SERVICE_NAME: str = 'NATSSeed_{identifier}'
    SERVICE_PORT: int = 7422
    GROUNDSTATION_ROLE: str = "gcs"

    class IPvUsage(Enum):
        AnyIPv = 1
        IPv4Only = 2
        IPv6Only = 3

        def ipv4Enabled(self) -> bool:
            return bool(self == self.AnyIPv or self == self.IPv4Only)

        def ipv6Enabled(self) -> bool:
            return bool(self == self.AnyIPv or self == self.IPv6Only)

    def __init__(self, logger: Logger, role, key, cert, servercert, ca, ipvUsage: IPvUsage):
        self.logger = logger
        self.role = role
        self.key = key
        self.cert = cert
        self.server_cert = servercert
        self.cert_authority = ca
        self.leaf_port = 7422
        self.seed_ip_address = None
        self.tls_required = False
        self.ipvUsage = ipvUsage

        if self.key is not None and self.cert is not None \
           and self.server_cert is not None and self.cert_authority is not None:
            self.tls_required = True

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
                self.logger.error("Default mesh starting failed ")
                return -1

        except:
            self.logger.error("Default mesh starting failed ")
            return -1

        return 0
    
    # generates seed config
    def generate_seed_config(self) -> None:
        """
        Generate the nats-server configuration file for the seed node.
        :return: None
        """

        if self.tls_required:
            config = textwrap.dedent(f"""
                listen: 0.0.0.0:4222
                tls {{
                    key_file: {self.key}
                    cert_file: {self.server_cert}
                    ca_file: {self.cert_authority}
                    verify: true
                }}
                leafnodes {{
                    port: 7422
                    tls {{
                        key_file: {self.key}
                        cert_file: {self.server_cert}
                        ca_file: {self.cert_authority}
                        verify: true
                    }}
                }}
            """)
        else:
            config = textwrap.dedent("""
                listen: 0.0.0.0:4222
                leafnodes {
                    port: 7422
                }
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
        if self.tls_required:
            protocol = "tls"
            remoteCfg = ""
            for seedUrls in seeds.values():
                urls = [seed.replace('$PROTOCOL$', protocol) for seed in seedUrls]
                remoteCfg += f"""
                    {{
                        urls: {urls}
                        tls {{
                            key_file: {self.key}
                            cert_file: {self.cert}
                            ca_file: {self.cert_authority}
                            verify: true
                        }}
                    }},"""

            config = textwrap.dedent(f"""
                listen: 0.0.0.0:4222
                tls {{
                    key_file: {self.key}
                    cert_file: {self.server_cert}
                    ca_file: {self.cert_authority}
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
                listen: 0.0.0.0:4222
                leafnodes {{
                    remotes = [{remoteCfg}
                    ]
                }}
            """)

        print(config)
        with open('/var/run/nats.conf', 'w',  encoding='UTF-8') as file_nats_conf:
            file_nats_conf.write(config)

    # generate nats config with given seeds and restart the server
    def update_configurations_and_restart(self, seeds) -> None:
        """
        Update the nats-server configuration and restart the nats-server.
        :param ip_address: ip address of the seed node
        :return: None
        """
        self.logger.debug("Updating configurations and reloading nats-server configuration")
        self.generate_leaf_config(seeds)

        # reload nats-server configuration
        ret = self.reload_config()
        if ret != 0:
            self.logger.error("Error reloading nats-server configuration")
            return


#
# Implements mDNS discovery for nats leaf nodes to actively search for seed nodes.
#
class NatsSeedDiscovery:

    def __init__(self, natsCtrl: NatsServerCtrl, logger: Logger):
        self.natsCtrl = natsCtrl
        self.logger = logger
        self.seeds = {}

    # callback handler called when CommsServiceDiscovery founds a new seed service
    def __seed_service_found(self, service_name, ipv4_addresses, ipv6_addresses, port, status):
        self.logger.debug(f"Seed service found: name:{service_name}, ip:{ipv4_addresses}, ip6:{ipv6_addresses}, port:{port}, online: {status}")

        # add new leaf
        if (status):
            uris = []
            for ip in ipv4_addresses:
                uris.append(f"$PROTOCOL$://{ip}:{port}")
            for ip in ipv6_addresses:
                uris.append(f"$PROTOCOL$://[{ip}]:{port}")

            self.seeds[service_name] = uris
            self.natsCtrl.update_configurations_and_restart(self.seeds)
        # disconnected leafs are not removed from config as they often are temporarily out of reach.

    # starts to monitor services and updates nats.conf when new seed is found
    def run(self):
        # generate empty leaf config so nats-server can start-up
        self.natsCtrl.generate_leaf_config({})

        self.logger.debug("Searching seed services.")
        monitor = CommsServiceMonitor(
            service_name=None,
            service_type=self.natsCtrl.SERVICE_TYPE,
            service_cb=self.__seed_service_found
        )
        monitor.run()


#
# Implements mDNS service publish for nats seed nodes to publish the service for leaf nodes.
#
class NatsSeedPublish:

    def __init__(self, natsCtrl: NatsServerCtrl, logger: Logger, ifName):
        self.zeroconf: Zeroconf = None
        self.serviceInfo: ServiceInfo = None
        self.seed_ipv4_addr = None
        self.seed_ipv6_addr = None

        self.logger = logger
        self.natsCtrl = natsCtrl
        self.ifName = ifName
        
        ipv4 = self.__get_ip4_address(ifName)
        if not ipv4:
            raise ValueError(f"Network interface {ifName} does not found.")

        if self.natsCtrl.ipvUsage.ipv4Enabled():
            self.seed_ipv4_addr = ipv4.get_attr('IFA_ADDRESS')
        if self.natsCtrl.ipvUsage.ipv6Enabled():
            ipv6 = self.__get_ip6_address(ipv4['index'])
            if not ipv6:
                self.seed_ipv6_addr = None
            else:    
                self.seed_ipv6_addr = ipv6.get_attr('IFA_ADDRESS')

        if self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv4Only:
            if not self.seed_ipv4_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip4 address.")
        elif self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv6Only:
            if not self.seed_ipv6_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip6 address.")
        elif self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.AnyIPv:
            if not self.seed_ipv4_addr and not self.seed_ipv6_addr: 
                raise ValueError(f"bridge {ifName} does not have valid ip4 or ip6 address.")
        
        mac = self.__getHwAddr(ifName)
        self.uid = hashlib.md5(mac.encode()).hexdigest()
        self.logger.debug(f"IPv4: {self.seed_ipv4_addr} IPv6: {self.seed_ipv6_addr} MAC: {mac} UID:{self.uid}")

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

        if self.zeroconf is not None or self.serviceInfo is not None:
            raise SystemError("Service already registered.")

        serviceName = NatsServerCtrl.SERVICE_NAME.format(identifier=self.uid)

        # include ipv4, ipv6 or both addresses.
        addrs = []
        if self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv4Only:
            addrs.append(socket.inet_aton(self.seed_ipv4_addr))
        elif self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.IPv6Only:
            addrs.append(socket.inet_pton(socket.AF_INET6, self.seed_ipv6_addr))
        elif self.natsCtrl.ipvUsage == NatsServerCtrl.IPvUsage.AnyIPv:
            addrs.append(socket.inet_aton(self.seed_ipv4_addr))
            if self.seed_ipv6_addr:
                addrs.append(socket.inet_pton(socket.AF_INET6, self.seed_ipv6_addr))

        # publish the service information
        self.serviceInfo = ServiceInfo(type_=NatsServerCtrl.SERVICE_TYPE,
            name=f'{serviceName}.{NatsServerCtrl.SERVICE_TYPE}',
            addresses=addrs,
            port=NatsServerCtrl.SERVICE_PORT,
            properties={'description': serviceName})

        self.zeroconf = Zeroconf(ip_version=IPVersion.All)
        self.zeroconf.register_service(info=self.serviceInfo,
                                       cooperating_responders=True)
        
    # removes (unpublish) service from Zeroconf
    def __unregister_seed_service(self):
        self.logger.debug("unregister_seed_service called")
        if self.zeroconf is None or self.serviceInfo is None:
            raise SystemError("Service NOT registered.")
        self.zeroconf.unregister_service(self.serviceInfo)
        self.zeroconf.close()

    # keeps the service published until script is stopped
    def run(self):
        self.logger.debug("Publish seed service.")
        self.natsCtrl.generate_seed_config()
        self.natsCtrl.reload_config()

        try:
            self.__register_seed_service()
            while True:
                time.sleep(1)
        finally:
            self.__unregister_seed_service()
            exit(0)


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

    natsCtrl = NatsServerCtrl(logger, args.role, args.key, args.cert, args.servercert, args.ca, ipvUsage)

    # start seed publish or seed discovery according the role.
    if (args.role == NatsServerCtrl.GROUNDSTATION_ROLE):
        publisher = NatsSeedPublish(natsCtrl, logger, args.ifname)
        publisher.run()
    else:
        discovery = NatsSeedDiscovery(natsCtrl, logger)
        discovery.run()
