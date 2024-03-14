"""
CBMA Adaptation
"""
import os
import logging
import subprocess
import threading
import time
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
from mdm_agent import Interface

from controller import CBMAController
from models.certificates import CBMACertificates


# pylint: disable=broad-exception-caught
class CBMAAdaptation(object):
    """
    CBMA Control
    """

    # pylint: disable=too-many-arguments
    def __init__(
        self,
        comms_ctrl: CommsController,
        logger: logging.Logger,
        lock: threading.Lock,
    ):
        """
        Constructor
        """
        self.__comms_ctrl: CommsController = comms_ctrl
        self.logger: logging.Logger = logger.getChild("CBMAAdaptation")
        self.__interfaces: List[Interface] = []
        self.__lock = lock
        self.__cbma_set_up = False  # Indicates whether CBMA has been configured
        self.__cbma_certs_path = "/opt/crypto/ecdsa/birth/filebased"
        self.__cbma_ca_cert_path = "/opt/mspki/ecdsa/certificate_chain.crt"
        self.__upper_cbma_certs_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value + "/crypto/rsa/birth/filebased"
        )
        self.__upper_cbma_ca_cert_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value
            + "/upper_certificates/rootCA.crt"
        )
        self.__upper_cbma_key_path = Constants.GENERATED_CERTS_PATH.value + "/rsa/birth/filebased"
        self.__lower_cbma_controller = None
        self.__upper_cbma_controller = None
        self.__lower_cbma_interfaces: List[Interface] = []
        self.__upper_cbma_interfaces: List[Interface] = []

        self.__white_interfaces = ["bat0", "halow1"]
        self.__red_interfaces = ["bat1", "wlan1"]
        self.__na_cbma_interfaces = [
            "bat0",
            "bat1",
            "br-lan",
            "eth0",
            "eth1",
            "usb0",
            "lan1",
        ]

        self.BR_NAME: str = "br-lan"
        self.BR_WHITE_NAME: str = "br-white"
        self.LOWER_BATMAN: str = "bat0"
        self.UPPER_BATMAN: str = "bat1"
        self.__IPV6_WHITE_PREFIX: str = "fdbb:1ef7:9d6f:e05d"
        self.__IPV6_RED_PREFIX: str = "fdd8:84dc:fe30:7bf2"

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
            subprocess.check_call(['ip', 'link', 'set', 'dev', interface, 'address', new_mac])

        except subprocess.CalledProcessError as e:
            self.logger.debug(
                "Error setting MAC address for %s! Error: %s", interface, e
            )

    def __set_bridge_ip(self, bridge_name: str, mesh_vif_name: str) -> None:
        ip = IPRoute()
        try:
            # Get the MAC address of the mesh interface
            mesh_if_mac = self.__get_mac_addr(mesh_vif_name)

            # Extract the last two bytes of the MAC address
            ip_random = mesh_if_mac[15:17]
            # Calculate the bridge IP address
            bridge_ip = f"192.168.1.{int(ip_random, 16)}"

            # Add the IP address to the bridge
            ip.addr(
                "add",
                index=ip.link_lookup(ifname=bridge_name)[0],
                address=bridge_ip,
                mask=24,
            )

        except Exception as e:
            self.logger.debug(
                "Error setting bridge IP for %s! Error: %s", bridge_name, e
            )

        finally:
            ip.close()

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
                    "Cannot add interfaces %s to bridge %s!",
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
        ip = IPRoute()
        try:
            # Bring the interface ip
            interface_indices = ip.link_lookup(ifname=interface_name)
            if interface_indices:
                ip.link("set", index=interface_indices[0], state="up")
        except Exception as e:
            self.logger.debug(
                "Error bringing up interface %s! Error: %s",
                interface_name,
                e,
            )
        finally:
            ip.close()

    def __shutdown_interface(self, interface_name: str) -> None:
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name
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

    def __create_batman_interface(self, batman_if, mac_addr=None) -> None:
        ip = IPRoute()
        try:
            # Check if the interface already exists
            interface_indices = ip.link_lookup(ifname=batman_if)
            if not interface_indices:
                self.logger.debug(
                    "Create interface %s, mac_addr: %s",
                    batman_if,
                    mac_addr if mac_addr is not None else "random",
                )
                # If the interface doesn't exist, create it as a Batman interface
                if mac_addr is not None:
                    ip.link("add", ifname=batman_if, kind="batadv", address=mac_addr)
                else:
                    ip.link("add", ifname=batman_if, kind="batadv")
        except Exception as e:
            self.logger.debug(
                "Error creating Batman interface %s: %s",
                batman_if,
                e,
            )
        finally:
            ip.close()

    def __configure_batman_interface(self, batman_if: str) -> None:
        is_upper = False
        if batman_if == self.UPPER_BATMAN:
            is_upper = True
        try:
            subprocess.run(
                ["batctl", "routing_algo", "BATMAN_V"], check=True
            )
        except subprocess.CalledProcessError as e:
            logging.error("Error setting routing algo!: %s", e)

        try:
            subprocess.run(
                ["batctl", "meshif", batman_if, "aggregation", "0"], check=True
            )
            subprocess.run(
                ["batctl", "meshif", batman_if, "bridge_loop_avoidance", "1"],
                check=True,
            )

            arp_table_setting = "1" if is_upper else "0"
            subprocess.run(
                [
                    "batctl",
                    "meshif",
                    batman_if,
                    "distributed_arp_table",
                    arp_table_setting,
                ],
                check=True,
            )

            subprocess.run(
                ["batctl", "meshif", batman_if, "fragmentation", "1"], check=True
            )

            subprocess.run(
                ["batctl", "meshif", batman_if, "orig_interval", "5000"], check=True
            )

            mtu_size = "1500" if is_upper else "1546"
            subprocess.run(
                ["ip", "link", "set", "dev", batman_if, "mtu", mtu_size], check=True
            )
        except subprocess.CalledProcessError as e:
            logging.error("Error configuring BATMAN interface: %s", e)

    def __destroy_batman_interface(self, mesh_interface: str) -> None:
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name
        try:
            ip.link("delete", ifname=mesh_interface)
        except Exception as e:
            self.logger.debug(f"Error: Unable to destroy interface {mesh_interface}.")
            self.logger.debug(f"Exception: {str(e)}")
        finally:
            ip.close()

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
            self.logger.debug("Certificate not found: %s", certificate_path)
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
                    return True
            elapsed_time = time.time() - start_time

            if elapsed_time >= timeout:
                return False  # Timeout reached

            time.sleep(1)  # Sleep for 1 second before checking again
            self.__get_interfaces()

    def __create_mac(self, randomized: bool = False, interface_mac: str = "") -> str:
        """
        Create a random MAC address or flip the locally administered bit of the given MAC address
        :return: The random MAC address
        """
        if randomized:
            mac = [random.randint(0x00, 0xff) for _ in range(6)]  # Generate 6 random bytes
            mac[0] &= 0xfc  # Clear multicast and locally administered bits
            mac[0] |= 0x02  # Set the locally administered bit
            return ':'.join(['%02x' % byte for byte in mac])
        else:
            # Split MAC address into octets and flip the locally administered bit
            octets = interface_mac.split(':')
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
        self.__create_batman_interface(
            self.LOWER_BATMAN,
            self.__create_mac(False, self.__get_mac_addr(if_name))
        )
        self.__configure_batman_interface(self.LOWER_BATMAN)
        self.__create_batman_interface(self.UPPER_BATMAN)
        self.__configure_batman_interface(self.UPPER_BATMAN)
        self.__set_interface_up(self.LOWER_BATMAN)
        self.__set_interface_up(self.UPPER_BATMAN)
        self.__create_bridge(self.BR_NAME)

        # Set random MAC address for the bridge
        self.__set_interface_mac(self.BR_NAME, self.__create_mac(True))
        self.__wait_for_interface(self.LOWER_BATMAN)
        self.__wait_for_interface(self.UPPER_BATMAN)

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
            self.logger.debug("Error: Unable to bring up the radio interfaces!")
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

    def setup_cbma(self) -> None:
        """
        Sets up both upper and lower CBMA.
        """
        self.__cleanup_cbma()
        self.__init_batman_and_bridge()

        self.__update_cbma_interface_lists()

        # Set radios on
        self.__setup_radios()

        # setup cbma
        self.__setup_lower_cbma()
        self.__setup_upper_cbma()

        # Add interfaces to the bridge
        for interface in self.__red_interfaces:
            self.__add_interface_to_bridge(self.BR_NAME, interface)
        self.__set_interface_up(self.BR_NAME)

        # Wait bridge to be up and add global IPv6 address to the bridge
        self.__wait_for_interface(self.BR_NAME)
        self.__add_global_ipv6_address(self.BR_NAME, self.__IPV6_RED_PREFIX)

        # Add global IPv6 address to the white batman :)
        self.__add_global_ipv6_address(self.LOWER_BATMAN, self.__IPV6_WHITE_PREFIX)

        self.__cbma_set_up = True
        return True

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
                # Find the current IPv6 link-local address
                link_local_address = None
                index = ip.link_lookup(ifname=interface_name)[0]
                addresses = ip.get_addr(index=index)

                for addr in addresses:
                    ip_address = addr.get_attrs("IFA_ADDRESS")[0]
                    prefix_length = int(addr["prefixlen"])
                    if self.__is_valid_ipv6_local((ip_address, prefix_length)):
                        link_local_address = ip_address
                        break

                if not link_local_address:
                    self.logger.debug(
                        f"Link-local IPv6 address not found for interface {interface_name}"
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

                # Add the modified IPv6 address to the interface
                ip.addr("add", index=index, address=new_address, prefixlen=64)
                ip.close()

        except Exception as e:
            self.logger.error(f"Error: {e}")

    def __setup_lower_cbma(self) -> None:
        """
        Sets up lower CBMA.
        :return: None
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

        for interface in self.__lower_cbma_interfaces:
            try:
                ret = self.__lower_cbma_controller.add_interface(
                    interface.interface_name
                )
                self.logger.debug(
                    f"Lower CBMA interfaces added: {interface.interface_name} "
                    f"status: {ret}"
                )
            except Exception as e:
                self.logger.exception(
                    "Error adding CBMA interface %s! Error: %s",
                    interface.interface_name,
                    e,
                )

    def __setup_upper_cbma(self) -> None:
        """
        Sets up upper CBMA.
        :return: None
        """

        upper_cbma_interfaces = []
        has_upper_certificate = 1

        self.logger.debug("Setting up upper CBMA...")

        for interface in self.__upper_cbma_interfaces:
            if interface.interface_name not in upper_cbma_interfaces:
                upper_cbma_interfaces.append(interface.interface_name)

            if self.__has_certificate(
                self.__upper_cbma_certs_path, interface.mac_address
            ):
                has_upper_certificate = has_upper_certificate and 1
            else:
                # TODO: Temporary backup solution to use lower CBMA certificates
                if self.__has_certificate(
                    self.__cbma_certs_path, interface.mac_address
                ):
                    has_upper_certificate = has_upper_certificate and 0
                else:
                    self.logger.warning(
                        "No cbma certificate for interface %s", interface.name
                    )
                    continue

        if has_upper_certificate:
            self.logger.debug("Using upper cbma certificates for interfaces")
            cert_dir: str = f"{self.__upper_cbma_certs_path}/MAC/"
            key: str = f"{self.__upper_cbma_key_path}/private.key"
            chain: list[str] = [self.__upper_cbma_ca_cert_path]
            ca: str = ""
        else:  # use birth certs
            self.logger.debug("Using lower cbma certificate as a backup for interface")

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

        for _interface in self.__upper_cbma_interfaces:
            try:
                ret = self.__upper_cbma_controller.add_interface(
                    _interface.interface_name
                )
                self.logger.debug(
                    f"Upper CBMA interfaces added: {_interface.interface_name} "
                    f"status: {ret}"
                )
            except Exception as e:
                self.logger.exception(
                    "Error adding upper CBMA interface %s! Error: %s",
                    _interface.interface_name,
                    e,
                )

    def __cleanup_cbma(self) -> None:
        self.__shutdown_interface(self.LOWER_BATMAN)
        self.__shutdown_interface(self.UPPER_BATMAN)

        # HACK: remove
        # self.__delete_ebtables_rules()
        # self.__delete_macsec_links()

        self.__destroy_batman_interface(self.LOWER_BATMAN)
        self.__destroy_batman_interface(self.UPPER_BATMAN)
        # HACK: How can we know that "br-upper" even exists and who owns it?
        self.__shutdown_and_delete_bridge("br-upper")
        self.__shutdown_and_delete_bridge(self.BR_NAME)

    def stop_cbma(self) -> bool:
        """
        Stops CBMA by terminating CBMA processes. Also
        destroys batman and bridge interfaces.
        """
        if not self.__cbma_set_up:
            return
        try:
            self.logger.debug("Stopping CBMA...")

            self.__lower_cbma_controller.stop()
            self.__upper_cbma_controller.stop()

            self.logger.debug("CBMA processes terminated, continue cleanup...")
            self.__cleanup_cbma()
            self.logger.debug("CBMA cleanup finished")

        except Exception as e:
            self.logger.error(f"Stop CBMA error: {e}")
        finally:
            self.__cbma_set_up = False
        return False