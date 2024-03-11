import re

from . import logging, common

LLA_PREFIX = "fe80"

logger = logging.get_logger()


def get_interface_mac_address(interface: str) -> str:
    try:
        return open(file=f"/sys/class/net/{interface}/address", mode='r').readline().strip()
    except FileNotFoundError:
        raise Exception(f"No MAC address found for interface {interface}")


def get_interface_link_local_ipv6_address(interface: str) -> str:
    if_inet6 = open("/proc/net/if_inet6", 'r').read()
    for line in if_inet6.splitlines():
        if line.startswith(LLA_PREFIX) and (line_splitted := line.split())[-1] == interface:
            ipv6_raw = line_splitted[0]
            ipv6_ext = re.sub(r'(.{4})(?!$)', r'\1:', ipv6_raw)
            ipv6_min = re.sub(r'(?<=^.{4}:)0000[:0]+', r':', ipv6_ext)
            ipv6 = re.sub(r':0{,3}', r':', ipv6_min)
            return ipv6
    raise Exception(f"{interface} doesn't have a Link Local IPv6 address")


def get_mac_from_ipv6(ipv6: str) -> str:
    ipv6_list = [o for o in ipv6.split(":") if o]

    # Find how many zeroes to add after the prefix
    times_padding = 8 - len(ipv6_list)

    # Expand IPv6
    full_ipv6 = ["0"] * times_padding + ipv6_list[1:]

    # Extract the EUI-64 identifier from the IPv6 address
    eui64_parts = full_ipv6[3:7]

    # Join them together into a single hex string
    eui64_hex = ("{:0>4}" * 4).format(*eui64_parts)

    eui64_bytes = bytearray.fromhex(eui64_hex)

    # Flip the universal/local bit
    eui64_bytes[0] ^= 0x2

    # Extract the MAC address from the EUI-64 identifier
    mac_bytes = eui64_bytes[:3] + eui64_bytes[5:8]

    return mac_bytes.hex(sep=':', bytes_per_sep=1)


def get_mac_from_ipv6_ndc(ipv6: str) -> str:
    cmd_list = ["ip", "-6", "neigh"]
    if output := common.run_command_output(cmd_list):
        for line in output.splitlines():
            if ipv6 in line:
                return line.rstrip().split()[-2]
    return ''
