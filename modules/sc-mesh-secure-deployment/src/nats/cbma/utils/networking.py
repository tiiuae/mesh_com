from struct import Struct
from ipaddress import IPv6Address

from . import common


LLA_PREFIX = 'fe80'

pack_ipv6 = Struct('!16s').pack


def get_interface_mac_address(interface: str) -> str:
    try:
        return open(file=f"/sys/class/net/{interface}/address", mode='r').readline().strip()
    except FileNotFoundError:
        raise Exception(f"No MAC address found for interface {interface}")


def get_interface_link_local_ipv6_address(interface: str) -> str:
    if_inet6 = open('/proc/net/if_inet6', 'r').read()
    for line in if_inet6.splitlines():
        if line.startswith(LLA_PREFIX) and (line_splitted := line.split())[-1] == interface:
            ipv6_addr = IPv6Address(pack_ipv6(bytes.fromhex(line_splitted[0])))
            return ipv6_addr.compressed
    raise Exception(f"{interface} doesn't have a Link Local IPv6 address")


def get_mac_from_ipv6(ipv6: str) -> str:
    ipv6_addr = IPv6Address(ipv6)

    # Extract the EUI-64 identifier from the IPv6 address
    eui64_bytes = bytearray(ipv6_addr.packed[8:])

    # Flip the universal/local bit
    eui64_bytes[0] ^= 0x2

    # Extract the MAC address from the EUI-64 identifier
    mac_bytes = eui64_bytes[:3] + eui64_bytes[5:8]

    return mac_bytes.hex(sep=':', bytes_per_sep=1)


def get_mac_from_ipv6_ndc(ipv6: str) -> str:
    cmd_list = ['ip', '-6', 'neigh']
    if output := common.run_command_output(cmd_list):
        for line in output.splitlines():
            if ipv6 in line:
                return line.rstrip().split()[-2]
    raise ValueError(f"Unable to find MAC address in {ipv6} Neighbor Discovery Cache")
