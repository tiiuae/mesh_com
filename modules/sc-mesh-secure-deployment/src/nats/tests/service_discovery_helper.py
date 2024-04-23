"""
Helper module to register and unregister services in DNS-SD.
"""
from typing import Optional
from zeroconf import Zeroconf, IPVersion, ServiceInfo
from ipaddress import IPv4Address, IPv6Address


SERVICE_TYPE: str = '_mdm._tcp.local.'
SERVICE_NAME: str = 'MDM Service'
HOSTNAME: str = 'defaultmdm.local'

IPV4_ANY_ADDR: bytes = IPv4Address("127.0.0.1").packed
IPV6_ANY_ADDR: bytes = IPv6Address("::1").packed

info: Optional[ServiceInfo] = None
zeroconf: Optional[Zeroconf] = None
kwargs_dict: dict = {}

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
                       addresses=[IPV4_ANY_ADDR, IPV6_ANY_ADDR],
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
    return kwargs_dict
