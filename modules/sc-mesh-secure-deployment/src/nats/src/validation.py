"""
Helper functions for validating NATS parameters
"""
import re
import socket

# country list from regulatory domain (wireless-regdb 2021.08.28)
COUNTRY_CODES = [['AD', 'Andorra'], ['AE', 'United Arab Emirates'], ['AF', 'Afghanistan'],
                 ['AI', 'Anguilla'], ['AL', 'Albania'], ['AM', 'Armenia'], ['AR', 'Argentina'],
                 ['AS', 'American Samoa'], ['AT', 'Austria'], ['AU', 'Australia'], ['AW', 'Aruba'],
                 ['AZ', 'Azerbaijan'], ['BA', 'Bosnia and Herzegovina'], ['BB', 'Barbados'],
                 ['BD', 'Bangladesh'], ['BE', 'Belgium'], ['BF', 'Burkina Faso'],
                 ['BG', 'Bulgaria'], ['BH', 'Bahrain'], ['BL', 'Saint Barthélemy'],
                 ['BM', 'Bermuda'], ['BN', 'Brunei Darussalam'],
                 ['BO', 'Bolivia, Plurinational State of'], ['BR', 'Brazil'], ['BS', 'Bahamas'],
                 ['BT', 'Bhutan'], ['BY', 'Belarus'], ['BZ', 'Belize'], ['CA', 'Canada'],
                 ['CF', 'Central African Republic'], ['CH', 'Switzerland'], ['CI', "Côte d'Ivoire"],
                 ['CL', 'Chile'], ['CN', 'China'], ['CO', 'Colombia'], ['CR', 'Costa Rica'],
                 ['CU', 'Cuba'], ['CX', 'Christmas Island'], ['CY', 'Cyprus'], ['CZ', 'Czechia'],
                 ['DE', 'Germany'], ['DK', 'Denmark'], ['DM', 'Dominica'],
                 ['DO', 'Dominican Republic'], ['DZ', 'Algeria'], ['EC', 'Ecuador'],
                 ['EE', 'Estonia'], ['EG', 'Egypt'], ['ES', 'Spain'], ['ET', 'Ethiopia'],
                 ['FI', 'Finland'], ['FM', 'Micronesia, Federated States of'], ['FR', 'France'],
                 ['GB', 'United Kingdom'], ['GD', 'Grenada'], ['GE', 'Georgia'],
                 ['GF', 'French Guiana'], ['GH', 'Ghana'], ['GL', 'Greenland'],
                 ['GP', 'Guadeloupe'], ['GR', 'Greece'], ['GT', 'Guatemala'], ['GU', 'Guam'],
                 ['GY', 'Guyana'], ['HK', 'Hong Kong'], ['HN', 'Honduras'], ['HR', 'Croatia'],
                 ['HT', 'Haiti'], ['HU', 'Hungary'], ['ID', 'Indonesia'], ['IE', 'Ireland'],
                 ['IL', 'Israel'], ['IN', 'India'], ['IR', 'Iran, Islamic Republic of'],
                 ['IS', 'Iceland'], ['IT', 'Italy'], ['JM', 'Jamaica'], ['JO', 'Jordan'],
                 ['JP', 'Japan'], ['KE', 'Kenya'], ['KH', 'Cambodia'],
                 ['KN', 'Saint Kitts and Nevis'], ['KP', "Korea, Democratic People's Republic of"],
                 ['KR', 'Korea, Republic of'], ['KW', 'Kuwait'], ['KY', 'Cayman Islands'],
                 ['KZ', 'Kazakhstan'], ['LB', 'Lebanon'], ['LC', 'Saint Lucia'],
                 ['LI', 'Liechtenstein'], ['LK', 'Sri Lanka'], ['LS', 'Lesotho'],
                 ['LT', 'Lithuania'], ['LU', 'Luxembourg'], ['LV', 'Latvia'], ['MA', 'Morocco'],
                 ['MC', 'Monaco'], ['MD', 'Moldova, Republic of'], ['ME', 'Montenegro'],
                 ['MF', 'Saint Martin (French part)'], ['MH', 'Marshall Islands'],
                 ['MK', 'North Macedonia'], ['MN', 'Mongolia'], ['MO', 'Macao'],
                 ['MP', 'Northern Mariana Islands'], ['MQ', 'Martinique'], ['MR', 'Mauritania'],
                 ['MT', 'Malta'], ['MU', 'Mauritius'], ['MV', 'Maldives'], ['MW', 'Malawi'],
                 ['MX', 'Mexico'], ['MY', 'Malaysia'], ['NG', 'Nigeria'], ['NI', 'Nicaragua'],
                 ['NL', 'Netherlands'], ['NO', 'Norway'], ['NP', 'Nepal'], ['NZ', 'New Zealand'],
                 ['OM', 'Oman'], ['PA', 'Panama'], ['PE', 'Peru'], ['PF', 'French Polynesia'],
                 ['PG', 'Papua New Guinea'], ['PH', 'Philippines'], ['PK', 'Pakistan'],
                 ['PL', 'Poland'], ['PM', 'Saint Pierre and Miquelon'], ['PR', 'Puerto Rico'],
                 ['PT', 'Portugal'], ['PW', 'Palau'], ['PY', 'Paraguay'], ['QA', 'Qatar'],
                 ['RE', 'Réunion'], ['RO', 'Romania'], ['RS', 'Serbia'],
                 ['RU', 'Russian Federation'], ['RW', 'Rwanda'], ['SA', 'Saudi Arabia'],
                 ['SE', 'Sweden'], ['SG', 'Singapore'], ['SI', 'Slovenia'], ['SK', 'Slovakia'],
                 ['SN', 'Senegal'], ['SR', 'Suriname'], ['SV', 'El Salvador'],
                 ['SY', 'Syrian Arab Republic'], ['TC', 'Turks and Caicos Islands'], ['TD', 'Chad'],
                 ['TG', 'Togo'], ['TH', 'Thailand'], ['TN', 'Tunisia'], ['TR', 'Turkey'],
                 ['TT', 'Trinidad and Tobago'], ['TW', 'Taiwan, Province of China'],
                 ['TZ', 'Tanzania, United Republic of'], ['UA', 'Ukraine'], ['UG', 'Uganda'],
                 ['US', 'United States'], ['UY', 'Uruguay'], ['UZ', 'Uzbekistan'],
                 ['VC', 'Saint Vincent and the Grenadines'],
                 ['VE', 'Venezuela, Bolivarian Republic of'], ['VI', 'Virgin Islands, U.S.'],
                 ['VN', 'Viet Nam'], ['VU', 'Vanuatu'], ['WF', 'Wallis and Futuna'],
                 ['WS', 'Samoa'], ['YE', 'Yemen'], ['YT', 'Mayotte'], ['ZA', 'South Africa'],
                 ['ZW', 'Zimbabwe']]


#
def __find_matching_country_code(country_code: str) -> str:
    """
    Returns the matching country code for the given country code.
    """
    for code, _ in COUNTRY_CODES:
        if code == country_code:
            return code
    return "00"


def validate_ssid(ssid: str) -> bool:
    """
    Validates a given SSID according to the 802.11 specification.
    Returns True if the SSID is valid, False otherwise.
    """
    try:
        if isinstance(ssid, int):
            return False
        if len(ssid) > 32:
            return False
        if re.search(r'[^\x20-\x7E]', ssid):
            return False
        return True
    except (ValueError, TypeError, AttributeError):
        return False


def validate_wpa3_psk(psk: str) -> bool:
    """
    Validates a given PSK for WPA3.
    Returns True if the PSK is valid, False otherwise.
    """
    try:
        if len(psk) < 8 or len(psk) > 63:
            return False
        if not re.match(r'^[!-~]*$', psk):
            return False
        return True
    except (ValueError, TypeError, AttributeError):
        return False


def validate_ip_address(ip: str) -> bool:
    """
    Validates a given IP address format.
    Returns True if the IP address is valid, False otherwise.
    """
    try:
        socket.inet_pton(socket.AF_INET, ip)
        return True
    except (socket.error, TypeError, AttributeError):
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
        if country_code.upper() != __find_matching_country_code(country_code.upper()):
            return False
        return True
    except (KeyError, AttributeError, TypeError):
        return False


def validate_mode(mode: str) -> bool:
    """
    Validates a given wifi mode.
    Returns True if the mode is valid, False otherwise.
    """
    if mode in ("mesh", "ap+mesh_scc", "ap+mesh_mcc"):
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
    list_of_5ghz_freq = [5170, 5180, 5190, 5200,
                         5210, 5220, 5230, 5240,
                         5260, 5280, 5300, 5320,
                         5500, 5520, 5540, 5560,
                         5580, 5600, 5620, 5640,
                         5660, 5680, 5700, 5720,
                         5745, 5765, 5785, 5805,
                         5825]

    if frequency in list_of_2ghz_freq or frequency in list_of_5ghz_freq:
        return True
    return False

def validate_routing(routing: str) -> bool:
    """
    Validates a given routing.
    Returns True if the routing is valid, False otherwise.
    """
    if routing in ("olsr", "batman-adv"):
        return True
    return False

def validate_priority(priority: str) -> bool:
    """
    Validates a given priority.
    Returns True if the priority is valid, False otherwise.
    """
    if priority in ("long_range", "high_throughput"):
        return True
    return False

def validate_role(role: str) -> bool:
    """
    Validates a given role.
    Returns True if the role is valid, False otherwise.
    """
    if role in ("drone", "sleeve", "gcs"):
        return True
    return False

def validate_delay(delay: str) -> bool:
    """
    Validates a given delay.
    Returns True if the delay is valid, False otherwise.
    """
    try:
        delay = int(delay)
        if delay < 0 or delay == 0:
            return False
        return True
    except (ValueError, TypeError, AttributeError):
        return False