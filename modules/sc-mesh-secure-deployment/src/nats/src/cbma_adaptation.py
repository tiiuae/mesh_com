"""
CBMA Adaptation
"""
import os
import logging
import subprocess
import threading
import time
import socket
from typing import List, Union
from copy import deepcopy
import json
import random
import fnmatch
import re
import ipaddress
from pyroute2 import IPRoute  # type: ignore[import-not-found, import-untyped]

from src import cbma_paths
from src.comms_controller import CommsController
from src.constants import Constants
from src.interface import Interface
from src import comms_config_store
from src.bat_ctrl_utils import BatCtrlUtils

from controller import CBMAController
from models.certificates import CBMACertificates


# pylint: disable=broad-except, invalid-name, too-many-instance-attributes
class CBMAAdaptation(object):
    """
    CBMA Control
    """

    def __init__(
        self,
        comms_ctrl: CommsController,
        logger: logging.Logger,
        lock: threading.Lock,
        config_file: str = Constants.MS_CONFIG_FILE.value,
    ):
        """
        Constructor
        """
        self.__comms_ctrl: CommsController = comms_ctrl
        self.logger: logging.Logger = logger.getChild("CBMAAdaptation")
        self.logger.setLevel(logging.INFO)
        self.__interfaces: List[Interface] = []
        self.__lock = lock
        self.__cbma_certs_path = Constants.ECDSA_BIRTH_FILEBASED.value
        self.__upper_cbma_certs_path = Constants.DOWNLOADED_RSA_CERTS_PATH.value
        self.__upper_cbma_ca_cert_path = Constants.DOWNLOADED_UPPER_CBMA_CA_CERT.value
        self.__upper_cbma_key_path = Constants.RSA_BIRTH_KEY
        self.__lower_cbma_controller = None
        self.__upper_cbma_controller = None
        self.__lower_cbma_interfaces: List[Interface] = []
        self.__upper_cbma_interfaces: List[Interface] = []

        self.BR_NAME: str = Constants.BR_NAME.value
        self.LOWER_BATMAN: str = Constants.LOWER_BATMAN.value
        self.UPPER_BATMAN: str = Constants.UPPER_BATMAN.value

        # Set minimum required configuration
        self.__white_interfaces = [self.LOWER_BATMAN]
        self.__red_interfaces = [self.UPPER_BATMAN]
        self.__na_cbma_interfaces = [
            self.LOWER_BATMAN,
            self.UPPER_BATMAN,
            self.BR_NAME,
        ]

        self.__batman = BatCtrlUtils(logger=self.logger, config_file=config_file)
        self.__config = None
        self.__cbma_config = None
        self.__vlan_config = None

        # Read configs from file
        try:
            self.__config = comms_config_store.ConfigStore(config_file)
        except Exception as e:
            self.logger.error(f"Error reading config file: {e}")

        if self.__config is not None:
            self.__cbma_config = self.__config.read("CBMA")
            self.__vlan_config = self.__config.read("VLAN")

        # Extend/update default configs using yaml file content
        if self.__cbma_config:
            white_interfaces = self.__cbma_config.get("white_interfaces")
            red_interfaces = self.__cbma_config.get("red_interfaces")
            exclude_interfaces = self.__cbma_config.get("exclude_interfaces")
            self.logger.info(f"White interfaces config: {white_interfaces}")
            self.logger.info(f"Red interfaces config: {red_interfaces}")
            self.logger.info(f"Exclude interfaces config: {exclude_interfaces}")

            if white_interfaces:
                self.__white_interfaces.extend(white_interfaces)
            if red_interfaces:
                self.__red_interfaces.extend(red_interfaces)
            if exclude_interfaces:
                self.__na_cbma_interfaces.extend(exclude_interfaces)

        # Create VLAN interfaces if configured
        self.__create_vlan_interfaces()

    def __create_vlan_interfaces(self) -> None:
        if not self.__vlan_config:
            self.logger.info("No VLAN configuration found")
            return

        for vlan_name, vlan_data in self.__vlan_config.items():
            self.logger.info(f"{vlan_name}, {vlan_data}")
            parent_interface = vlan_data.get("parent_interface")
            vlan_id = str(vlan_data.get("vlan_id"))
            ipv4_address = vlan_data.get("ipv4_address")
            ipv4_subnet_mask = vlan_data.get("ipv4_subnet_mask")
            ipv6_address = vlan_data.get("ipv6_local_address")
            ipv6_prefix_length = vlan_data.get("ipv6_prefix_length")

            if parent_interface and vlan_id:
                try:
                    # Delete existing VLAN interface if it exists
                    subprocess.run(["ip", "link", "delete", vlan_name], check=False)

                    # Create VLAN interface
                    subprocess.run(
                        [
                            "ip",
                            "link",
                            "add",
                            "link",
                            parent_interface,
                            "name",
                            vlan_name,
                            "type",
                            "vlan",
                            "id",
                            vlan_id,
                        ],
                        check=True,
                    )

                    # Bring VLAN interface up
                    subprocess.run(["ip", "link", "set", vlan_name, "up"], check=True)

                    if ipv4_address and ipv4_subnet_mask:
                        # Set IPv4 address and subnet mask
                        subprocess.run(
                            [
                                "ip",
                                "addr",
                                "add",
                                f"{ipv4_address}/{ipv4_subnet_mask}",
                                "dev",
                                vlan_name,
                            ],
                            check=True,
                        )

                    if ipv6_address and ipv6_prefix_length:
                        # Set IPv6 address and prefix length
                        subprocess.run(
                            [
                                "ip",
                                "addr",
                                "add",
                                f"{ipv6_address}/{ipv6_prefix_length}",
                                "dev",
                                vlan_name,
                            ],
                            check=True,
                        )

                    self.logger.info(
                        f"VLAN interface {vlan_name} created and configured."
                    )
                except subprocess.CalledProcessError as e:
                    self.logger.error(f"Error creating VLAN interface {vlan_name}: {e}")
            else:
                self.logger.error(f"Invalid configuration for VLAN {vlan_name}.")

    def __delete_vlan_interfaces(self) -> None:
        if not self.__vlan_config:
            return

        for vlan_name, _ in self.__vlan_config.items():
            try:
                # Delete existing VLAN interface if it exists
                subprocess.run(["ip", "link", "delete", vlan_name], check=False)
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Error deleting VLAN interface {vlan_name}: {e}")

    def __get_interfaces(self) -> None:
        interfaces = []
        ipr = IPRoute()
        for link in ipr.get_links():
            ifname = link.get_attr("IFLA_IFNAME")
            ifstate = link.get_attr("IFLA_OPERSTATE")
            mac_address = link.get_attr("IFLA_ADDRESS")

            interface_info = {
                "interface_name": ifname,
                "operstate": ifstate,
                "mac_address": mac_address,
            }
            interfaces.append(interface_info)

        self.__interfaces.clear()

        for interface_data in interfaces:
            interface = Interface(
                interface_name=interface_data["interface_name"],
                operstat=interface_data["operstate"],
                mac_address=interface_data["mac_address"],
            )
            self.__interfaces.append(interface)

        self.logger.debug(
            "__get_interfaces: %s",
            interfaces,
        )

    def __create_bridge(self, bridge_name) -> None:
        ip = IPRoute()
        try:
            bridge_indices = ip.link_lookup(ifname=bridge_name)
            if not bridge_indices:
                ip.link("add", ifname=bridge_name, kind="bridge")

        except Exception as e:
            self.logger.exception(
                "Error creating bridge %s! Error: %s",
                bridge_name,
                e,
            )

        finally:
            ip.close()

    def __set_interface_mac(self, interface, new_mac):
        try:
            # Change the MAC address of the bridge interface
            subprocess.check_call(
                ["ip", "link", "set", "dev", interface, "address", new_mac]
            )

        except subprocess.CalledProcessError as e:
            self.logger.error(
                "Error setting MAC address for %s! Error: %s", interface, e
            )

    def __add_interface_to_bridge(
        self, bridge_name: str, interface_to_add: str
    ) -> None:
        # Create an IPRoute object
        ip = IPRoute()

        try:
            bridge_indices = ip.link_lookup(ifname=bridge_name)
            if bridge_indices:
                bridge_index = bridge_indices[0]
            else:
                self.logger.debug(
                    "Cannot add interface to bridge %s as it was not found!",
                    bridge_name,
                )
                return

            # Get the index of the interface to add
            interface_indices = ip.link_lookup(ifname=interface_to_add)

            if not interface_indices:
                self.logger.debug(
                    "Cannot add interface %s to bridge %s!",
                    interface_to_add,
                    bridge_name,
                )
                return

            interface_index = interface_indices[0]
            # Add the interface to the bridge
            ip.link("set", index=interface_index, master=bridge_index)

        except Exception as e:
            self.logger.debug(
                "Error adding interface %s to bridge %s! Error: %s",
                interface_to_add,
                bridge_name,
                e,
            )
        finally:
            # Close the IPRoute object
            ip.close()

    def __set_interface_up(self, interface_name: str) -> None:
        try:
            subprocess.run(
                ["ip", "link", "set", interface_name, "up", "state", "up"], check=True
            )
        except Exception as e:
            self.logger.error(
                "Error bringing up interface %s! Error: %s",
                interface_name,
                e,
            )

    def __shutdown_interface(self, interface_name: str) -> None:
        ip = IPRoute()
        try:
            index = ip.link_lookup(ifname=interface_name)[0]
            ip.link("set", index=index, state="down")
        except IndexError:
            self.logger.debug(
                "Not able to shutdown interface %s! Interface not found!",
                interface_name,
            )
        finally:
            ip.close()

    def __get_mac_addr(self, interface_name: str) -> Union[None, str]:
        for interface in self.__interfaces:
            if interface.interface_name == interface_name:
                return interface.mac_address
        return None  # Interface not found in the list

    def __shutdown_and_delete_bridge(self, bridge_name: str) -> None:
        ip = IPRoute()
        try:
            index = ip.link_lookup(ifname=bridge_name)[0]
            ip.link("set", index=index, state="down")
            ip.link("delete", index=index)
        except IndexError:
            self.logger.debug(
                "Not able to delete bridge %s! Bridge not found!", bridge_name
            )
        finally:
            ip.close()

    def __has_certificate(self, cert_path: str, mac: str) -> bool:
        certificate_path = f"{cert_path}/MAC/{mac}.crt"

        if not os.path.exists(certificate_path):
            self.logger.warning("Certificate not found: %s", certificate_path)
            return False

        self.logger.debug("Certificate found: %s", certificate_path)
        return True

    def __update_cbma_interface_lists(self) -> None:
        # Initially all the interfaces with certificates should use lower CBMA
        self.__lower_cbma_interfaces.clear()
        with self.__lock:
            self.__lower_cbma_interfaces = deepcopy(self.__interfaces)
        self.__upper_cbma_interfaces.clear()

        # Get list of interfaces that doesn't have lower CBMA certificate
        interfaces_without_certificate = []
        for interface in self.__lower_cbma_interfaces:
            if not self.__has_certificate(
                self.__cbma_certs_path, interface.mac_address
            ):
                interfaces_without_certificate.append(interface)

        # Remove interfaces without certificates from lower CBMA interface list
        for interface in interfaces_without_certificate:
            self.logger.debug(
                "Interface %s doesn't have certificate!", interface.interface_name
            )
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        not_allowed_lower_cbma_interfaces = []
        filtering_list = (
            self.__red_interfaces.copy()
            + self.__na_cbma_interfaces.copy()
            + self.__white_interfaces.copy()
        )
        for interface in self.__lower_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in filtering_list):
                not_allowed_lower_cbma_interfaces.append(interface)

        # Remove not allowed interfaces from lower CBMA interface list
        for interface in not_allowed_lower_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        # Add white aka upper CBMA interfaces
        for interface in self.__interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in self.__white_interfaces):
                self.__upper_cbma_interfaces.append(interface)

        # Lower and upper CBMA interfaces are mutually exclusive so remove
        # upper CBMA interfaces from lower CBMA interface list
        for interface in self.__upper_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

    @staticmethod
    def __wait_for_ap(timeout: int = 4) -> bool:
        start_time = time.time()
        while True:
            try:
                result = subprocess.check_output(["iw", "dev", "wlan1", "info"])
                result_str = result.decode("utf-8")
                # Use regular expressions to extract the interface type
                match = re.search(r"type\s+([\w-]+)", result_str)
                if match.group(1) == "AP":
                    return True

                elapsed_time = time.time() - start_time

                if elapsed_time >= timeout:
                    return False  # Timeout reached
                time.sleep(1)
            except subprocess.CalledProcessError:
                return False

    def __wait_for_interface(self, if_name, timeout: int = 3) -> bool:
        self.logger.debug("__wait_for_interface %s", if_name)
        start_time = time.time()
        self.__get_interfaces()
        while True:
            with self.__lock:
                found = any(
                    interface.interface_name == if_name and interface.operstat == "UP"
                    for interface in self.__interfaces
                )
                if found:
                    break
            elapsed_time = time.time() - start_time

            if elapsed_time >= timeout:
                self.logger.warning("__wait_for_interface timeout for %s", if_name)
                return False  # Timeout reached

            time.sleep(1)  # Sleep for 1 second before checking again
            self.__get_interfaces()

        link_local_address = self.__get_link_local_ipv6_address(if_name)
        if not link_local_address:
            self.logger.debug(
                f"Link-local IPv6 address not found for interface {if_name}"
            )
            return False
        index = socket.if_nametoindex(if_name)

        # Now we wait until the interface is ready
        start_time = time.time()
        while True:
            try:
                socket.create_server(
                    (link_local_address, 0, 0, index), family=socket.AF_INET6
                )
                return True
            except Exception:
                elapsed_time = time.time() - start_time
                if elapsed_time >= timeout:
                    return False
                time.sleep(1)

    def __create_mac(self, randomized: bool = False, interface_mac: str = "") -> str:
        """
        Create a random MAC address or flip the locally administered bit of the given
        MAC address.
        :return: The random MAC address
        """
        if randomized or interface_mac is None:
            mac = [
                random.randint(0x00, 0xFF) for _ in range(6)
            ]  # Generate 6 random bytes
            mac[0] &= 0xFC  # Clear multicast and locally administered bits
            mac[0] |= 0x02  # Set the locally administered bit
            return bytes(mac).hex(sep=":", bytes_per_sep=1)
        else:
            # Split MAC address into octets and flip the locally administered bit
            octets = interface_mac.split(":")
            first_octet = int(octets[0], 16)
            flipped_first_octet = first_octet ^ 2

            return f"{flipped_first_octet}:{':'.join(octets[1:])}"

    def __init_batman_and_bridge(self) -> None:
        if_name = self.__comms_ctrl.settings.mesh_vif[0]
        if if_name.startswith("halow"):
            if_pattern = "wlp*s0"
            for interface_name in self.__comms_ctrl.settings.mesh_vif:
                if fnmatch.fnmatch(interface_name, if_pattern):
                    if_name = interface_name
                    break

        self.__get_interfaces()
        # if_name MAC and flip the locally administered bit
        self.__batman.create_batman_interface(
            self.LOWER_BATMAN, self.__create_mac(False, self.__get_mac_addr(if_name))
        )
        self.__batman.configure_batman_interface(self.LOWER_BATMAN)
        self.__set_mtu_size(self.LOWER_BATMAN)
        self.__batman.create_batman_interface(self.UPPER_BATMAN)
        self.__batman.configure_batman_interface(self.UPPER_BATMAN)
        self.__set_mtu_size(self.UPPER_BATMAN)
        self.__set_interface_up(self.LOWER_BATMAN)
        self.__set_interface_up(self.UPPER_BATMAN)
        self.__create_bridge(self.BR_NAME)

        # Set random MAC address for the bridge
        self.__set_interface_mac(self.BR_NAME, self.__create_mac(True))
        self.__wait_for_interface(self.LOWER_BATMAN)
        self.__wait_for_interface(self.UPPER_BATMAN)

    def stop_radios(self) -> bool:
        """
        Stops radios
        """
        # Create command to stop all radios
        cmd = json.dumps(
            {
                "api_version": 1,
                "cmd": "DOWN",
                "radio_index": "*",
            }
        )

        ret, _, _ = self.__comms_ctrl.command.handle_command(cmd, self.__comms_ctrl)

        if ret != "OK":
            self.logger.error("Error: Unable to bring down the radio interfaces!")
            return False

        return True

    def __setup_radios(self) -> bool:
        # Create command to start all radios
        cmd = json.dumps(
            {
                "api_version": 1,
                "cmd": "UP",
                "radio_index": "*",
            }
        )

        ret, _, _ = self.__comms_ctrl.command.handle_command(cmd, self.__comms_ctrl)

        if ret != "OK":
            self.logger.error("Error: Unable to bring up the radio interfaces!")
            return False

        # Radio startup may take long time. Try to ensure
        # interface exists and is ready to be added to bridge.
        for interface_name in self.__comms_ctrl.settings.mesh_vif:
            self.logger.debug("mesh_vif: %s", interface_name)
            timeout = 3
            if interface_name.startswith("halow"):
                timeout = 10
            self.__wait_for_interface(interface_name, timeout)

        for mode in self.__comms_ctrl.settings.mode:
            if mode == "ap+mesh_mcc":
                self.__wait_for_ap()

        return True

    def __set_mtu_size(self, interface_name: str, interface_color: str = None) -> None:
        """
        Set the MTU size for the specified interface color.
        Red, white and black colors are supported.

        if interface_name is lower batman or upper batman:
        Lower batman 1540
        Upper batman 1500

        params:
            interface_name: The name of the interface
            interface_color: The color of the interface
        """
        BASE_MTU_SIZE = 1500
        MACSEC_OVERHEAD = 16
        BATMAN_OVERHEAD = 24

        # if upper or lower batman
        if interface_name == self.UPPER_BATMAN:
            interface_color = Constants.RED_INTERFACE.value
        elif interface_name == self.LOWER_BATMAN:
            interface_color = Constants.WHITE_INTERFACE.value
        elif interface_name.startswith("halow"):
            self.logger.error(
                "Cannot set MTU size for halow interface %s" " as driver limitation",
                interface_name,
            )
            return

        # use color to set mtu size
        if interface_color == Constants.RED_INTERFACE.value:
            mtu_size = str(BASE_MTU_SIZE)
        elif interface_color == Constants.WHITE_INTERFACE.value:
            mtu_size = str(BASE_MTU_SIZE + MACSEC_OVERHEAD + BATMAN_OVERHEAD)
        elif interface_color == Constants.BLACK_INTERFACE.value:
            mtu_size = str(BASE_MTU_SIZE + (2 * (MACSEC_OVERHEAD + BATMAN_OVERHEAD)))
        else:
            self.logger.error("Invalid color %s! MTU size not set.", interface_color)
            return

        self.logger.info("Setting MTU size for %s to %s", interface_name, mtu_size)

        try:
            subprocess.run(
                ["ip", "link", "set", "dev", interface_name, "mtu", mtu_size],
                check=True,
            )
        except subprocess.CalledProcessError as e:
            self.logger.error(
                "Error setting MTU size for %s! Error: %s", interface_name, e
            )

    def setup_cbma(self) -> bool:
        """
        Sets up both upper and lower CBMA.
        :return: True if both lower and upper CBMA was setup
                 successfully. Returns False otherwise.
        """
        self.__init_batman_and_bridge()

        self.__update_cbma_interface_lists()

        # Set radios on
        self.__setup_radios()

        # setup cbma
        lower_cbma_set_up = self.__setup_lower_cbma()
        upper_cbma_set_up = self.__setup_upper_cbma()

        # Add interfaces to the bridge
        for interface in self.__red_interfaces:
            self.__set_mtu_size(interface, Constants.RED_INTERFACE.value)
            self.__add_interface_to_bridge(self.BR_NAME, interface)
        self.__set_interface_up(self.BR_NAME)

        # Wait bridge to be up and add global IPv6 address to the bridge
        self.__wait_for_interface(self.BR_NAME)
        self.__add_global_ipv6_address(self.BR_NAME, Constants.IPV6_RED_PREFIX.value)

        # Add global IPv6 address to the white batman :)
        self.__add_global_ipv6_address(self.LOWER_BATMAN, Constants.IPV6_WHITE_PREFIX.value)

        # Set batman hop penalty
        self.__batman.set_hop_penalty()

        return lower_cbma_set_up and upper_cbma_set_up

    def __is_valid_ipv6_local(self, address: tuple[str, int]) -> bool:
        """
        Check if the address is a valid IPv6 link-local address

        :param address: The address to check
        :type address: (str, int)
        :return: True if the address is a valid IPv6 link-local address, False otherwise
        """
        try:

            ip = ipaddress.ip_address(address[0])
            return (
                ip.version == 6
                and ip.compressed.startswith("fe80:")
                and address[1] == 64
                and ip.is_link_local
            )
        except Exception as e:
            self.logger.error(e)
            return False

    def __get_link_local_ipv6_address(self, interface_name: str) -> str:
        with IPRoute() as ip:
            index = ip.link_lookup(ifname=interface_name)[0]
            addresses = ip.get_addr(index=index)

            for addr in addresses:
                ip_address = addr.get_attrs("IFA_ADDRESS")[0]
                prefix_length = int(addr["prefixlen"])
                if self.__is_valid_ipv6_local((ip_address, prefix_length)):
                    ip.close()
                    return ip_address
            ip.close()
        return ""

    def __add_global_ipv6_address(self, interface_name: str, new_prefix: str) -> None:
        """
        Modify the IPv6 address of the specified interface

        :param interface_name: The name of the interface
        :type interface_name: str
        :param new_prefix: The new prefix to use
        :type new_prefix: str
        """
        try:
            with IPRoute() as ip:
                link_local_address = self.__get_link_local_ipv6_address(interface_name)

                if not link_local_address:
                    self.logger.debug(
                        "Link-local IPv6 address not found for interface %s",
                        interface_name,
                    )
                    return

                # Modify the prefix
                new_address = (
                    new_prefix + link_local_address[link_local_address.find("::") + 1 :]
                )
                self.logger.debug(
                    f"Current {interface_name} Local IPv6 address: {link_local_address}"
                )
                self.logger.debug(
                    f"New {interface_name} Global IPv6 address: {new_address}"
                )
                index = ip.link_lookup(ifname=interface_name)[0]

                # Add the modified IPv6 address to the interface
                ip.addr("add", index=index, address=new_address, prefixlen=64)
                ip.close()

        except Exception as e:
            self.logger.error(f"Error: {e}")

    def __setup_lower_cbma(self) -> bool:
        """
        Sets up lower CBMA.
        :return: True if at least one interface was successfully
                 added to the lower CBMA. Returns False otherwise.
        """
        self.logger.debug("Setting up lower CBMA...")
        cert_dir = f"{self.__cbma_certs_path}/MAC/"
        key = f"{self.__cbma_certs_path}/private.key"
        chain = [
            "/opt/mspki/ecdsa/security_officers/filebased.crt",
            "/opt/mspki/ecdsa/intermediate.crt",
            "/opt/mspki/ecdsa/root.crt",
        ]

        certificates = CBMACertificates(cert_dir, key, chain)

        self.__lower_cbma_controller = CBMAController(
            Constants.CBMA_PORT_LOWER.value, self.LOWER_BATMAN, certificates, False
        )
        intf_added = False
        for interface in self.__lower_cbma_interfaces:
            try:
                self.__set_mtu_size(
                    interface.interface_name, Constants.BLACK_INTERFACE.value
                )

                ret = self.__lower_cbma_controller.add_interface(
                    interface.interface_name
                )
                self.logger.info(
                    f"Lower CBMA interfaces added: {interface.interface_name} "
                    f"status: {ret}"
                )
                if ret:
                    intf_added = True
            except Exception as e:
                self.logger.exception(
                    "Error adding CBMA interface %s! Error: %s",
                    interface.interface_name,
                    e,
                )
        return intf_added

    def __setup_upper_cbma(self) -> bool:
        """
        Sets up upper CBMA.
        :return: True if at least one interface was successfully
                 added to the upper CBMA. Returns False otherwise.
        """

        upper_cbma_interfaces = []
        has_upper_certificate = 1

        self.logger.info("Setting up upper CBMA...")

        for interface in self.__upper_cbma_interfaces:
            if interface.interface_name not in upper_cbma_interfaces:
                upper_cbma_interfaces.append(interface.interface_name)

            if self.__has_certificate(
                self.__upper_cbma_certs_path, interface.mac_address
            ):
                has_upper_certificate = has_upper_certificate and 1
            else:
                if self.__has_certificate(
                    self.__cbma_certs_path, interface.mac_address
                ):
                    has_upper_certificate = has_upper_certificate and 0
                else:
                    self.logger.warning(
                        "No cbma certificate for interface %s", interface.interface_name
                    )
                    continue

        if has_upper_certificate:
            self.logger.info("Using upper cbma certificates for interfaces")
            cert_dir: str = f"{self.__upper_cbma_certs_path}/MAC/"
            key: str = f"{self.__upper_cbma_key_path}/private.key"
            chain: list[str] = [self.__upper_cbma_ca_cert_path]
            ca: str = ""
        else:  # use birth certs
            self.logger.warning(
                "Using lower cbma certificate as a backup for interface"
            )

            cert_dir: str = f"{self.__cbma_certs_path}/MAC/"
            key: str = f"{self.__cbma_certs_path}/private.key"
            chain: list[str] = [
                "/opt/mspki/ecdsa/security_officers/filebased.crt",
                "/opt/mspki/ecdsa/intermediate.crt",
                "/opt/mspki/ecdsa/root.crt",
            ]
            ca: str = ""

        certificates = CBMACertificates(cert_dir, key, chain, ca)
        self.__upper_cbma_controller = CBMAController(
            Constants.CBMA_PORT_UPPER.value, self.UPPER_BATMAN, certificates, True, True
        )
        intf_added = False
        for _interface in self.__upper_cbma_interfaces:
            try:
                # change mtu size
                self.__set_mtu_size(
                    _interface.interface_name, Constants.WHITE_INTERFACE.value
                )

                ret = self.__upper_cbma_controller.add_interface(
                    _interface.interface_name
                )
                self.logger.debug(
                    f"Upper CBMA interfaces added: {_interface.interface_name} "
                    f"status: {ret}"
                )
                if ret:
                    intf_added = True
            except Exception as e:
                self.logger.exception(
                    "Error adding upper CBMA interface %s! Error: %s",
                    _interface.interface_name,
                    e,
                )
        return intf_added

    def __cleanup_cbma(self) -> None:
        self.__shutdown_interface(self.LOWER_BATMAN)
        self.__shutdown_interface(self.UPPER_BATMAN)

        self.__batman.destroy_batman_interface(self.LOWER_BATMAN)
        self.__batman.destroy_batman_interface(self.UPPER_BATMAN)

        self.__delete_vlan_interfaces()
        self.__shutdown_and_delete_bridge(self.BR_NAME)

    def stop_cbma(self) -> bool:
        """
        Stops CBMA by terminating CBMA processes. Also
        destroys batman and bridge interfaces.
        :return: Boolean to indicate success.
        """
        lower_stopped = False
        upper_stopped = False
        try:
            self.logger.debug("Stopping CBMA...")

            lower_stopped = self.__lower_cbma_controller.stop()
            upper_stopped = self.__upper_cbma_controller.stop()

            self.logger.debug("CBMA processes terminated, continue cleanup...")
        except Exception as e:
            self.logger.error(f"Stop CBMA error: {e}")
        finally:
            self.__cleanup_cbma()
            self.logger.debug("CBMA cleanup finished")
        return lower_stopped and upper_stopped
