import re
import socket
from iso3166 import countries


def validate_ssid(ssid: str) -> bool:
    """
    Validates a given SSID according to the 802.11 specification.
    Returns True if the SSID is valid, False otherwise.
    """
    if len(ssid) > 32:
        return False
    if re.search(r'[^\x20-\x7E]', ssid):
        return False
    return True


def validate_wpa3_psk(psk: str) -> bool:
    """
    Validates a given PSK for WPA3.
    Returns True if the PSK is valid, False otherwise.
    """
    if len(psk) < 8 or len(psk) > 63:
        return False
    if not re.match(r'^[!-~]*$', psk):
        return False
    return True


def validate_ip_address(ip: str) -> bool:
    """
    Validates a given IP address format.
    Returns True if the IP address is valid, False otherwise.
    """
    try:
        socket.inet_pton(socket.AF_INET, ip)
        return True
    except socket.error:
        return False


def validate_netmask(netmask: str) -> bool:
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
    except (ValueError, TypeError, AttributeError):
        return False


def validate_tx_power(power_in_dbm: int) -> bool:
    """
    Validates a given TX power in dBm.
    Returns True if the power is valid, False otherwise.
    """
    try:
        power = int(power_in_dbm)
        if power < 0 or power > 40:
            return False
        return True
    except (ValueError, NameError, TypeError, AttributeError):
        return False


def validate_country_code(country_code: str) -> bool:
    """
    Validates a given country code. Accepts lower and upper case.
    Returns True if the country alpha2 code is valid, False otherwise.
    """
    try:
        if len(country_code) != 2:
            return False

        country = countries.get(country_code.upper())
        if country is None:
            return False
        if country.alpha2 != country_code.upper():
            return False
        return True
    except (KeyError, AttributeError, TypeError):
        return False


def validate_mode(mode: str) -> bool:
    """
    Validates a given wifi mode.
    Returns True if the mode is valid, False otherwise.
    """
    # todo add correct modes
    if mode in ["mesh"]:
        return True
    return False


def validate_frequency(frequency: int) -> bool:
    """
    Validates a given wi-fi frequency.
    Returns True if the frequency is valid, False otherwise.
    """
    list_of_2ghz_freq = [2412, 2417, 2422, 2427,
                         2432, 2437, 2442, 2447,
                         2452, 2457, 2462, 2467,
                         2472, 2484]
    list_of_5ghz_freq = [5180, 5200, 5220, 5240,
                         5260, 5280, 5300, 5320,
                         5500, 5520, 5540, 5560,
                         5580, 5600, 5620, 5640,
                         5660, 5680, 5700, 5745,
                         5765, 5785, 5805, 5825]
    # list_of_6ghz_freq = [5955, 5975, 5995, 6015,
    #                      6035, 6055, 6075, 6095,
    #                      6115, 6135, 6155, 6175,
    #                      6195, 6215, 6235, 6255,
    #                      6275, 6295, 6315, 6335,
    #                      6355, 6375, 6395, 6415,
    #                      6435, 6455, 6475, 6495,
    #                      6515, 6535, 6555, 6575,
    #                      6595, 6615, 6635, 6655,
    #                      6675, 6695, 6715, 6735,
    #                      6755, 6775, 6795, 6815,
    #                      6835, 6855, 6875, 6895,
    #                      6915, 6935, 6955, 6975,
    #                      6995, 7015, 7035, 7055,
    #                      7075, 7095, 7115]
    if frequency in list_of_2ghz_freq or frequency in list_of_5ghz_freq:
        return True
    return False
