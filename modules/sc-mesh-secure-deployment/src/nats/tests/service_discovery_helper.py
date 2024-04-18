from typing import List
from zeroconf import Zeroconf, IPVersion, ServiceInfo
import socket
import psutil


SERVICE_TYPE: str = '_mdm._tcp.local.'
SERVICE_NAME: str = 'MDM Service'
HOSTNAME: str = 'defaultmdm.local'

IPV4_ANY_ADDR: str = '127.0.0.1'
IPV6_ANY_ADDR: str = '::'

info: ServiceInfo = None
zeroconf: Zeroconf = None
kwargs_dict: dict = {}


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


def dns_service_register() -> None:
    """
    Register service in DNS-SD.

    params:
        None
    return:
        None
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

def dns_service_unregister() -> None:
    """
    Unregister service from DNS-SD.

    params:
        None
    return:
        None
    """
    global info, zeroconf
    zeroconf.unregister_service(info)
    zeroconf.close()

def service_discovery_cb(address: str, status: bool, **kwargs: dict) -> None:
    """
    Callback function to be called when a service is discovered.

    params:
        address: str
        status: bool
        **kwargs: dict
    """
    global kwargs_dict
    kwargs_dict = kwargs

def get_kwargs_dict() -> dict:
    """
    Get the kwargs dictionary.

    params:
        None
    return:
        dict
    """
    global kwargs_dict
    return kwargs_dict