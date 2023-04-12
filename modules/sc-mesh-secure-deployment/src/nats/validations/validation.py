import re
import socket


def validate_ssid(ssid):
    """
    Validates a given SSID according to the 802.11 specification.
    Returns True if the SSID is valid, False otherwise.
    """
    if len(ssid) > 32:
        return False
    if re.search(r'[^\x20-\x7E]', ssid):
        return False
    return True

def validate_wpa3_psk(psk):
    """
    Validates a given PSK for WPA3.
    Returns True if the PSK is valid, False otherwise.
    """
    if len(psk) < 8 or len(psk) > 63:
        return False
    if not re.match(r'^[!-~]*$', psk):
        return False
    return True

def validate_ip_address(ip):
    """
    Validates a given IP address format.
    Returns True if the IP address is valid, False otherwise.
    """
    try:
        socket.inet_pton(socket.AF_INET, ip)
        return True
    except:
        return False

def validate_netmask(netmask):
    """
    Validates a given netmask format.
    Returns True if the netmask is valid, False otherwise.
    """
    try:
        parts = [int(part) for part in netmask.split('.')]
        if len(parts) != 4:
            return False
        for part in parts:
            if part < 0 or part > 255:
                return False
        bin_str = ''.join([bin(part)[2:].zfill(8) for part in parts])
        if '01' in bin_str[1:]:
            return False
        return True
    except:
        return False