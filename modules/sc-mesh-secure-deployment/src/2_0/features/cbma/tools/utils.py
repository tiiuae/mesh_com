import subprocess
import configparser

import os
import sys
import time
import shutil

import socket
import ipaddress
import re
from .custom_logger import CustomLogger

# sys.path.insert(0, '../')
path_to_tools_dir = os.path.dirname(
    __file__
)  # Path to dir containing this script

logger_instance = CustomLogger("utils")
logger = logger_instance.get_logger()


######### MOVED FROM CLASSES #############

def batman(batman_interface):
    try:
        subprocess.run(["ip", "link", "add", "name", batman_interface, "type", "batadv"], check=True)

        batman_exec(batman_interface, "batman-adv")
    except Exception as e:
        logger.error(f'Error setting up {batman_interface}: {e}')
        sys.exit(1)


def setup_batman(batman_interface):
    # Wait till a macsec interface is setup and added
    # to batman before setting up batman interface
    # while (
    #     not self.macsec_setup_event.is_set()
    #     and not self.shutdown_event.is_set()
    # ):
    #     time.sleep(1)
    # # Exit in case shutdown event is set
    # if self.shutdown_event.is_set():
    #     return

    # Turn batman interface up if not up already
    if not is_interface_up(batman_interface):
        batman(batman_interface)

        # TODO - This code was unreachable anyways as self.lan_bridge_flag was False by default
        # if self.level == "upper" and self.lan_bridge_flag:
        #     # Add bat1 and ethernet interface to br-lan to connect external devices
        #     bridge_interface = "br-lan"
        #     if not is_interface_up(bridge_interface):
        #         self.setup_bridge_over_batman(batman_interface, bridge_interface)


def setup_bridge_over_batman(batman_interface, bridge_interface):
    # TODO: Need to configure interfaces to add to br-lan
    # (This is just for quick test)
    subprocess.run(["brctl", "addbr", bridge_interface], check=True)
    add_interface_to_bridge(
        interface_to_add=batman_interface,
        bridge_interface=bridge_interface,
    )  # Add bat1 to br-lan
    add_interface_to_bridge(
        interface_to_add="eth1", bridge_interface=bridge_interface
    )  # Add eth1 to br-lan
    logger.info(
        "Setting mac address of %s to be same as %s..",
        bridge_interface,
        batman_interface,
    )
    subprocess.run(
        [
            "ip",
            "link",
            "set",
            "dev",
            bridge_interface,
            "address",
            get_mac_addr(batman_interface),
        ],
        check=True,
    )
    subprocess.run(
        ["ip", "link", "set", bridge_interface, "up"], check=True
    )
    subprocess.run(["ifconfig", bridge_interface], check=True)

##########################################


def is_wpa_supplicant_running():
    try:
        # Running the command and decoding the output
        output = subprocess.check_output(["ps", "ax"]).decode("utf-8")
        # Check for wpa_supplicant in the output
        processes = [
            line for line in output.splitlines() if "wpa_supplicant" in line
        ]
        # Filter out any lines containing 'grep'
        processes = [proc for proc in processes if "grep" not in proc]
        return len(processes) > 0
    except Exception as e:
        logger.error("Error checking wpa_supplicant status: %s", e)
        return False


def run_wpa_supplicant(wifidev):
    """
    maybe this should be executed from the mesh-11s.sh
    but we will need to modify the batmat part
    """
    conf_file = "/var/run/wpa_supplicant-11s.conf"
    log_file = "/tmp/wpa_supplicant_11s.log"
    shutil.copy(
        f"{path_to_tools_dir}/wpa_supplicant-11s.conf", conf_file
    )  # TODO: change in mesh_com, this is only for testing

    # Build the command with all the arguments
    command = [
        "wpa_supplicant",
        "-i",
        wifidev,
        "-c",
        conf_file,
        "-D",
        "nl80211",
        "-C",
        "/var/run/wpa_supplicant/",
        "-B",
        "-f",
        log_file,
    ]

    try:
        # Run the wpa_supplicant command as a subprocess
        result = subprocess.run(command, check=True)
        if result.returncode != 0:
            logger.error(
                "Error executing command: %s. Return code: %s",
                result.args,
                result.returncode,
            )
        else:
            logger.info("wpa_supplicant process started successfully.")
    except subprocess.CalledProcessError as e:
        logger.error("Error starting wpa_supplicant process: %s", e)


def mesh_service():
    # Check if mesh provisioning is done
    # we are assuming that the file exists on /opt/mesh.conf
    if os.path.exists("/opt/S9011sMesh"):
        # Start Mesh service
        try:
            logger.info("starting 11s mesh service")
            subprocess.run(
                ["/opt/S9011sMesh", "start"],
                check=True,
                capture_output=True,
                text=True,
            )
            time.sleep(2)
        except subprocess.CalledProcessError as e:
            logger.error(
                "Error executing command: %s. Return code: %s",
                e.cmd,
                e.returncode,
            )
            logger.error("Error output: %s", e.stderr)


def killall(interface):
    try:
        # Kill wpa_supplicant
        result = subprocess.run(
            ["killall", "wpa_supplicant"], capture_output=True, text=True
        )
        if result.returncode != 0:
            if "no process killed" in result.stderr:
                logger.error("wpa_supplicant process was not running.")
            else:
                logger.error(
                    "Error executing command: %s. Return code: %s",
                    result.args,
                    result.returncode,
                )

        # Bring the interface down
        subprocess.run(["ifconfig", interface, "down"], check=True)

        # Bring the interface back up
        subprocess.run(["ifconfig", interface, "up"], check=True)

    except subprocess.CalledProcessError as e:
        logger.error(
            "Error executing command: %s. Return code: %s", e.cmd, e.returncode
        )


def apply_nft_rules(rules_file="firewall.nft"):
    try:
        # Run the nft command to apply the rules from the specified file
        subprocess.run(["nft", "-f", rules_file], check=True)
        logger.info("nftables rules applied successfully.")
    except subprocess.CalledProcessError as e:
        logger.error("Error applying nftables rules: %s", e)


def modify_conf_file(conf_file_path, new_values):
    config = configparser.ConfigParser()
    config.read(conf_file_path)

    for section, options in new_values.items():
        for option, value in options.items():
            config.set(section, option, value)

    with open(conf_file_path, "w") as configfile:
        config.write(configfile)


def batman_exec(batman_interface, routing_algo):
    if routing_algo != "batman-adv":
        # TODO here should be OLSR
        return
    run_batman(batman_interface)


def run_batman(batman_interface):
    # Hack to support CM2 as well
    interfaces = ["wlp1s0", "wlp2s0", "wlp3s0", "halow1", "eth0"]
    mac = "NaN"
    for interface in interfaces:
        mac = get_mac_addr(interface)
        if mac != "NaN":
            logger.info(
                "Setting mac address of %s to be same as %s.",
                batman_interface,
                interface,
            )
            break
    subprocess.run(
        ["ip", "link", "set", "dev", batman_interface, "address", mac],
        check=True,
    )

    logger.info("Setting %s up..", batman_interface)
    # Run the ifconfig batman_interface up command
    subprocess.run(["ifconfig", batman_interface, "up"], check=True)

    logger.info("Setting %s mtu size", batman_interface)
    # Run the ifconfig batman_interface mtu 1460 command
    subprocess.run(["ifconfig", batman_interface, "mtu", "1460"], check=True)

    # Run the ifconfig batman_interface command to show the
    # interface information
    subprocess.run(["ifconfig", batman_interface], check=True)


"""
def mac_to_ipv6(mac_address):
    # Remove any separators from the MAC address (e.g., colons, hyphens)
    mac_address = mac_address.replace(":", "").replace("-", "").lower()

    # Split the MAC address into two equal halves
    first_half = mac_address[:6]

    # Convert the first octet from hexadecimal to binary
    binary_first_octet = bin(int(first_half[:2], 16))[2:].zfill(8)


    # Invert the seventh bit (change 0 to 1 or 1 to 0)
    inverted_seventh_bit = "1" if binary_first_octet[6] == "0" else "0"


    # Convert the modified binary back to hexadecimal
    modified_first_octet = hex(int(binary_first_octet[:6]
                           + inverted_seventh_bit
                           + binary_first_octet[7:], 2))[2:]

    # Replace the original first octet with the modified one
    modified_mac_address = modified_first_octet + mac_address[2:]


    line = f"{modified_mac_address[:5]}fffe{modified_mac_address[5:]}"

    # Add "ff:fe:" to the middle of the new MAC address
#    mac_with_fffe = ":".join(a + b for a, b in zip(a[::2], a[1::2]))
    mac_with_fffe = ":".join([line[:3], line[3:7], line[7:11], line[11:]])

    return f"fe80::{mac_with_fffe}"
"""


def mac_to_ipv6(mac):
    # Split MAC address and insert ff:fe in the middle
    mac_parts = mac.split(":")
    eui_64 = mac_parts[:3] + ["ff", "fe"] + mac_parts[3:]

    # Modify the 7th bit (Universal/Local bit)
    eui_64[0] = format(int(eui_64[0], 16) ^ 0x02, "02x")

    # Combine parts to form EUI-64 part of IPv6
    eui_64_combined = "".join(eui_64)

# pylint: disable=line-too-long
    return f"fe80::{eui_64_combined[:4]}:{eui_64_combined[4:8]}:{eui_64_combined[8:12]}:{eui_64_combined[12:16]}"  # noqa
# pylint: enable=line-too-long


def get_mac_from_ipv6(ipv6_address, interface):
    try:
        # Send pings to the IPv6 address to prompt an NDP exchange
        # Increase the ping count, if needed, for better reliability
        subprocess.run(
            ["ping", "-c", "1", f"{ipv6_address}%{interface}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )

        # Check the neighbor cache
        output = subprocess.check_output(
            ["ip", "-6", "neigh", "show", ipv6_address], text=True
        )

        # Extract MAC address
        mac_search = re.search(
            r"(([0-9a-f]{2}:){5}[0-9a-f]{2})", output, re.IGNORECASE
        )
        if mac_search:
            return mac_search.group(1)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        logger.error("An error occurred: %s", e)
    except re.error as re_e:
        logger.error("Regex error: %s", re_e)
    return None


def extract_mac_from_ipv6(ipv6_address):  # assuming link local address
    # Parse and expand the IPv6 address to its full form
    full_ipv6_address = ipaddress.IPv6Address(ipv6_address).exploded

    # Extract the EUI-64 identifier from the IPv6 address
    eui_64_parts = full_ipv6_address.split(":")[4:8]

    # Join them together into a single hex string
    eui_64_hex = "".join(eui_64_parts)

    # Separate the bytes that make up the EUI-64 identifier
    eui_64_bytes = bytes.fromhex(eui_64_hex)

    # Extract the MAC address from the EUI-64 identifier
    mac_bytes = bytearray(6)
    mac_bytes[0] = eui_64_bytes[0] ^ 0x02  # Flip the universal/local bit
    mac_bytes[1:3] = eui_64_bytes[1:3]
    mac_bytes[3:5] = eui_64_bytes[5:7]
    mac_bytes[5] = eui_64_bytes[7]

    return ":".join(f"{byte:02x}" for byte in mac_bytes)


def get_mac_addr(EXPECTED_INTERFACE):
    """
    got it from common/tools/field_test_logger/wifi_info.py
    """
    try:
        with open(f"/sys/class/net/{EXPECTED_INTERFACE}/address", "r") as f:
            value = f.readline()
            return value.strip()
    except Exception:
        return "NaN"


def set_ipv6(interface, ipv6):
    command = ["ip", "-6", "addr", "add", f"{ipv6}/64", "dev", interface]
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, check=True
        )
        logger.debug(result.stdout)
    except subprocess.CalledProcessError as e:
        logger.error("Command failed with error: %s", e.stderr)


def is_ipv4(ip):
    try:
        socket.inet_pton(socket.AF_INET, ip)
        return True
    except socket.error:
        return False


def is_ipv6(ip):
    try:
        socket.inet_pton(socket.AF_INET6, ip)
        return True
    except socket.error:
        return False


def generate_random_bytes(byte_size=32):
    return os.urandom(byte_size)


def is_interface_pingable(interface_name, ip_address):
    # Return true if pingable
    try:
        if is_ipv4(ip_address):
            ping_output = subprocess.run(
                ["ping", "-c", "1", "-w", "1", ip_address],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                universal_newlines=True,
            )
            return "1 packets transmitted, 1 received" in ping_output.stdout
        elif is_ipv6(ip_address):
            ping_output = subprocess.run(
                [
                    "ping",
                    "-c", "1",
                    "-w", "1",
                    f"{ip_address}%{interface_name}",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                universal_newlines=True,
            )
            return "1 packets transmitted, 1 received" in ping_output.stdout
        else:
            raise ValueError("Invalid IP address")
    except subprocess.CalledProcessError as e:
        logger.error("%s", e)
        return False


def wait_for_interface_to_be_pingable(
    interface_name, ipv6_address, shutdown_event
):
    waiting_message_printed = False
    while (
        not is_interface_pingable(interface_name, ipv6_address)
        and not shutdown_event.is_set()
    ):
        # Waiting till interface is pingable
        if not waiting_message_printed:
            logger.info("Waiting for %s to be reachable..", interface_name)
            waiting_message_printed = True
        time.sleep(1)


def is_interface_up(interface_name):
    # Check if interface is up
    try:
        # Error suppressed as the command throws an error
        # when interface is not present
        output = subprocess.check_output(
            ["ifconfig", interface_name], stderr=subprocess.DEVNULL
        )
        return "inet" in output.decode()
    except subprocess.CalledProcessError:
        return False


def wait_for_interface_to_be_up(interface_name, shutdown_event):
    waiting_message_printed = False
    while not is_interface_up(interface_name) and not shutdown_event.is_set():
        # Waiting till interface is up
        if not waiting_message_printed:
            logger.info("Waiting for %s to be up..", interface_name)
            waiting_message_printed = True
        time.sleep(1)


def xor_bytes(byte1, byte2, byte_size=32):
    # Trim the bytes if they are longer than byte_size
    if len(byte1) > byte_size:
        byte1 = byte1[:byte_size]
    if len(byte2) > byte_size:
        byte2 = byte2[:byte_size]

    # Pad bytes to the required length
    if len(byte1) < byte_size:
        byte1 = byte1.rjust(byte_size, b"\x00")
    if len(byte2) < byte_size:
        byte2 = byte2.rjust(byte_size, b"\x00")

    # Return bit-wise XOR of byte1 and byte2
    return bytes(a ^ b for a, b in zip(byte1, byte2))


def add_interface_to_batman(interface_to_add, batman_interface):
    # Add interface to batman
    try:
        subprocess.run(
            [
                "batctl",
                "meshif",
                batman_interface,
                "if",
                "add",
                interface_to_add,
            ],
            check=True,
        )
        logger.info(
            "Added interface %s to %s", interface_to_add, batman_interface
        )
    except Exception as e:
        logger.error(
            "Error adding interface %s to %s: %s",
            interface_to_add,
            batman_interface,
            e,
        )


def add_interface_to_bridge(interface_to_add, bridge_interface):
    try:
        subprocess.run(
            ["brctl", "addif", bridge_interface, interface_to_add], check=True
        )
        logger.info(
            "Added interface %s to %s", interface_to_add, bridge_interface
        )
    except Exception as e:
        logger.error(
            "Error adding interface %s to %s: %s",
            interface_to_add,
            bridge_interface,
            e,
        )


def setup_ebtables_macsec(interface, mac):
    try:
        subprocess.run(["ebtables", "--table", "nat", "--append", "OUTPUT",
                        "--out-interface", interface,
                        "--destination", "ff:ff:ff:ff:ff:ff",
                        "--jump", "dnat",
                        "--to-destination", mac], check=True)
        subprocess.run(["ebtables", "--table", "nat", "--append", "PREROUTING",
                        "--in-interface", interface,
                        "--source", mac,
                        "--destination", get_mac_addr(interface),
                        "--jump", "dnat",
                        "--to-destination", "ff:ff:ff:ff:ff:ff"], check=True)
        logger.info(f'Added ebtable rule for {mac} and {interface}')
    except Exception as e:
        logger.info(f'Error adding ebtable rule for {mac} and {interface}: {e}')

def setup_bridge(bridge_interface):
    # Set a bridge interface up
    try:
        subprocess.run(["brctl", "addbr", bridge_interface], check=True)
        subprocess.run(
            ["ip", "link", "set", bridge_interface, "up"], check=True
        )
        logger.info("Setup bridge %s", bridge_interface)
        setup_ebtables_bridge(bridge_interface)
    except Exception as e:
        logger.error("Error setting up bridge %s: %s", bridge_interface, e)

def setup_ebtables_bridge(bridge_interface):
    try:
        subprocess.run(["ebtables", "--append", "FORWARD",
                        "--logical-in", bridge_interface,
                        "--jump", "ACCEPT"],
                       check=True)
        subprocess.run(["ebtables", "--append", "FORWARD",
                        "--logical-out", bridge_interface,
                        "--jump", "ACCEPT"],
                       check=True)
        logger.info(f'Setup ebtables for {bridge_interface}')
    except Exception as e:
        logger.error(f'Error setting up ebtables for {bridge_interface}: {e}')
