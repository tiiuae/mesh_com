import unittest
from unittest.mock import MagicMock
from typing import List
from zeroconf import Zeroconf, IPVersion, ServiceInfo
import socket
import psutil

from src.comms_service_discovery import CommsServiceMonitor

SERVICE_TYPE: str = '_mdm._tcp.local.'
SERVICE_NAME: str = 'MDM Service'
HOSTNAME: str = 'defaultmdm.local'

IPV4_ANY_ADDR: str = '127.0.0.1'
IPV6_ANY_ADDR: str = '::'


def get_interface_addresses(interface: str) -> List[bytes]:
    """
    Get interface addresses.
    """
    attributes: List = psutil.net_if_addrs().get(interface, [])
    addresses: List[bytes] = []
    for attr in attributes:
        if socket.AddressFamily.AF_INET == attr.family:
            addresses.append(socket.inet_aton(attr.address))
        if socket.AddressFamily.AF_INET6 == attr.family:
            address: str = str(attr.address).replace(f'%{interface}', '')
            addresses.append(socket.inet_pton(socket.AF_INET6, address))

    return addresses


info: ServiceInfo = None
zeroconf: Zeroconf = None
kwargs_dict: dict = {}

def dns_service_register():
    """
    Register service in DNS-SD.
    """
    global info, zeroconf
    info = ServiceInfo(type_=SERVICE_TYPE,
                       name=f'{SERVICE_NAME}.{SERVICE_TYPE}',
                       addresses=get_interface_addresses("lo"),
                       port=5000,
                       properties={'description': SERVICE_NAME},
                       server=f'{HOSTNAME}.')
    zeroconf = Zeroconf(ip_version=IPVersion.All)
    zeroconf.register_service(info)

def dns_service_unregister():
    """
    Unregister service from DNS-SD.
    """
    global info, zeroconf
    zeroconf.unregister_service(info)
    zeroconf.close()

def service_discovery_cb(address, status, **kwargs):
    """
    Callback function to be called when a service is discovered.
    """
    global kwargs_dict
    kwargs_dict = kwargs


class CommsServiceDiscoveryProperties(unittest.TestCase):

    def test_comms_service_discovery_initialization(self):
        try:
            logger = MagicMock()
            dns_service_register()
            comms_service_discovery = CommsServiceMonitor(
                service_name=SERVICE_NAME,
                service_type=SERVICE_TYPE,
                service_cb=service_discovery_cb,
                # interface="br-lan"
                test=True,
                logger=logger
            )

            comms_service_discovery.run()

            self.assertEqual(kwargs_dict['service_name'], f'{SERVICE_NAME}.{SERVICE_TYPE}')

        except Exception as e:
            self.fail(f"An error occurred: {e}")

        finally:
            dns_service_unregister()
