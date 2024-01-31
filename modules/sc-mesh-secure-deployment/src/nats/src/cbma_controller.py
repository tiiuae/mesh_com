"""
CBMA Controller
"""
from os import path
import logging
import subprocess
import threading
import time
from multiprocessing import Process
from typing import List, Dict
from copy import deepcopy
from pyroute2 import IPRoute  # type: ignore[import-not-found, import-untyped]

from src import cbma_paths
from src.comms_controller import CommsController
from src.constants import Constants
from mdm_agent import Interface

from cbma import setup_cbma  # type: ignore[import-not-found]
from cbma.tools.utils import batman  # type: ignore[import-not-found]


class CBMAControl:
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
        self.logger: logging.Logger = logger.getChild("CBMAControl")
        self.__interfaces: List[Interface] = []
        self.__lock = lock
        self.__cbma_set_up = False  # Indicates whether CBMA has been configured
        self.__cbma_certs_path = "/opt/crypto/ecdsa/birth/filebased"
        self.__cbma_ca_cert_path = "/opt/mspki/ecdsa/certificate_chain.crt"
        self.__upper_cbma_certs_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value + "/crypto/ecdsa/birth/filebased"
        )
        self.__upper_cbma_ca_cert_path = (
            Constants.DOWNLOADED_CBMA_UPPER_PATH.value
            + "/mspki/ecdsa/certificate_chain.crt"
        )
        self.__lower_cbma_processes: Dict[str, Process] = {}
        self.__upper_cbma_processes: Dict[str, Process] = {}
        self.__lower_cbma_interfaces: List[Interface] = []
        self.__upper_cbma_interfaces: List[Interface] = []

    def __get_interfaces(self):
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

            if interface_info:
                interfaces.append(interface_info)

        self.__interfaces.clear()

        for interface_data in interfaces:
            interface = Interface(
                interface_name=interface_data["interface_name"],
                operstat=interface_data["operstate"],
                mac_address=interface_data["mac_address"],
            )
            self.__interfaces.append(interface)

    @staticmethod
    def __cleanup_batman_interfaces():
        # Create an IPRoute object
        ip = IPRoute()
        interfaces_to_cleanup = ["bat0", "bat1"]

        for interface in interfaces_to_cleanup:
            if interface in [link.get_attr("IFLA_IFNAME") for link in ip.get_links()]:
                # If the interface exists, set it down, and then delete it
                ip.link(
                    "set",
                    index=ip.link_lookup(ifname=interface)[0],
                    state="down",
                )
                ip.link("delete", index=ip.link_lookup(ifname=interface)[0])

        # Close the IPRoute object
        ip.close()

    def __remove_all_interfaces_from_bridge(self, bridge_name):
        # Create an IPRoute object
        ip = IPRoute()

        try:
            # Get the index of the bridge
            bridge_indices = ip.link_lookup(ifname=bridge_name)
            if bridge_indices:
                bridge_index = bridge_indices[0]
            else:
                self.logger.debug(
                    "Cannot remove interfaces from %s as it was not found!",
                    bridge_name,
                )
                return
            # Get the indices of all interfaces currently in the bridge
            interfaces_indices = [
                link["index"] for link in ip.get_links(master=bridge_index)
            ]

            if not interfaces_indices:
                self.logger.debug(
                    "No interfaces to remove from the bridge %s!",
                    bridge_name,
                )
                return

            # Remove each interface from the bridge
            for interface_index in interfaces_indices:
                ip.link(
                    "set", index=interface_index, master=0
                )  # Set the master to 0 (no bridge)

        except Exception as e:
            self.logger.debug(
                "Error removing interfaces from bridge %s!",
                e,
            )

        finally:
            # Close the IPRoute object
            ip.close()

    def __add_interface_to_bridge(self, bridge_name, interface_to_add):
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

    @staticmethod
    def __delete_ebtables_rules():
        command_ebtables = ["ebtables", "-t", "nat", "-L", "OUTPUT"]
        command_sed = ["sed", "-n", "/OUTPUT/,/^$/{/^--/p}"]
        command_xargs = ["xargs", "ebtables", "-t", "nat", "-D", "OUTPUT"]

        proc_ebtables = subprocess.Popen(command_ebtables, stdout=subprocess.PIPE)
        proc_sed = subprocess.Popen(
            command_sed, stdin=proc_ebtables.stdout, stdout=subprocess.PIPE
        )
        proc_xargs = subprocess.Popen(command_xargs, stdin=proc_sed.stdout)
        proc_xargs.wait()

        subprocess.run(
            [
                "ebtables",
                "--delete",
                "FORWARD",
                "--logical-in",
                "br-upper",
                "--jump",
                "ACCEPT",
            ],
            check=False

        )
        subprocess.run(
            [
                "ebtables",
                "--delete",
                "FORWARD",
                "--logical-out",
                "br-upper",
                "--jump",
                "ACCEPT",
            ],
            check=False
        )

    @staticmethod
    def __delete_macsec_links():
        # Define the command as a list of arguments
        command_macsec = ["ip", "macsec", "show"]
        command_grep = ["grep", ": protect on validate"]
        command_awk1 = ["awk", "-F:", "{print $2}"]
        command_awk2 = ["awk", "{print $1}"]
        command_xargs = ["xargs", "-I", "{}", "ip", "link", "delete", "{}"]

        # Run the commands using subprocess and connect their pipes
        proc_macsec = subprocess.Popen(command_macsec, stdout=subprocess.PIPE)
        proc_grep = subprocess.Popen(
            command_grep, stdin=proc_macsec.stdout, stdout=subprocess.PIPE
        )
        proc_awk1 = subprocess.Popen(
            command_awk1, stdin=proc_grep.stdout, stdout=subprocess.PIPE
        )
        proc_awk2 = subprocess.Popen(
            command_awk2, stdin=proc_awk1.stdout, stdout=subprocess.PIPE
        )
        proc_xargs = subprocess.Popen(command_xargs, stdin=proc_awk2.stdout)

        # Wait for the xargs process to finish
        proc_xargs.wait()

    def __shutdown_interface(self, interface_name):
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

    def __destroy_batman_interface(self, mesh_interface):
        # pylint: disable=invalid-name
        ip = IPRoute()
        # pylint: enable=invalid-name
        try:
            ip.link("delete", ifname=mesh_interface)
        except Exception as e:
            self.logger.debug(
                f"Error: Unable to destroy interface {mesh_interface}."
            )
            self.logger.debug(f"Exception: {str(e)}")
        finally:
            ip.close()

    def __shutdown_and_delete_bridge(self, bridge_name):
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

    def __cleanup_cbma(self):
        self.__shutdown_interface("bat0")
        self.__shutdown_interface("bat1")
        self.__delete_ebtables_rules()
        self.__delete_macsec_links()
        self.__destroy_batman_interface("bat0")
        self.__destroy_batman_interface("bat1")
        self.__shutdown_and_delete_bridge("br-upper")

    def stop_cbma(self):
        """
        Stops CBMA by terminating CBMA processes. Also
        destroys batman and bridge interfaces.
        """
        if not self.__cbma_set_up:
            return
        try:
            self.logger.debug("Stopping CBMA...")

            for if_name, process in self.__lower_cbma_processes.items():
                try:
                    self.logger.debug(
                        "Stopping lower CBMA process %s for interface %s",
                        process,
                        if_name,
                    )
                    if process.is_alive():
                        process.terminate()
                except Exception as e:
                    self.logger.error(
                        f"Terminating lower CBMA process error: {e}"
                    )

            for process in self.__lower_cbma_processes.values():
                try:
                    if process.is_alive():
                        process.join(timeout=1.0)
                        process.kill()
                except Exception as e:
                    self.logger.error(
                        f"Killing lower CBMA process error: {e}"
                    )

            for if_name, process in self.__upper_cbma_processes.items():
                try:
                    self.logger.debug(
                        "Stopping upper CBMA process %s for interface %s",
                        process,
                        if_name,
                    )
                    if process.is_alive():
                        process.terminate()
                except Exception as e:
                    self.logger.error(
                        f"Terminating upper CBMA process error: {e}"
                    )

            for process in self.__upper_cbma_processes.values():
                try:
                    if process.is_alive():
                        process.join(timeout=1.0)
                        process.kill()
                except Exception as e:
                    self.logger.error(
                        f"Killing upper CBMA process error: {e}"
                    )
            self.logger.debug(
                "CBMA processes terminated, continue cleanup..."
            )
            self.__cleanup_cbma()
            self.logger.debug("CBMA cleanup finished")

        except Exception as e:
            self.logger.error(f"Stop CBMA error: {e}")
        finally:
            self.__cbma_set_up = False
        return False

    def __has_certificate(self, cert_path, mac) -> bool:
        certificate_path = f"{cert_path}/MAC/{mac}.crt"

        if not path.exists(certificate_path):
            self.logger.debug(
                "Certificate not found: %s", certificate_path
            )
            return False

        self.logger.debug("Certificate found: %s", certificate_path)
        return True

    def __update_cbma_interface_lists(self):
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

        # Remove interfaces without certificates lower CBMA interface list
        for interface in interfaces_without_certificate:
            self.logger.debug(
                "Interface %s doesn't have certificate!", interface.interface_name
            )
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        not_allowed_lower_cbma_interfaces = []
        # FIXME: Hard coded red interfaces for testing purposes
        # Eventually MDM server will define interface colors somehow
        red_interface_prefixes = ["eth", "usb", "wlan", "br-lan", "bat", "lan"]
        for interface in self.__lower_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            if any(
                prefix in interface_name
                for prefix in red_interface_prefixes
            ):
                not_allowed_lower_cbma_interfaces.append(interface)

        # Remove not allowed interfaces from lower CBMA interface list
        for interface in not_allowed_lower_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

        # FIXME: Hard coded white (upper CBMA) interfaces for testing purposes
        # Eventually MDM server will define interface colors somehow
        white_interfaces = ["bat0", "halow1", "wlp3s0"]
        for interface in self.__interfaces:
            interface_name = interface.interface_name.lower()
            if any(prefix in interface_name for prefix in white_interfaces):
                # TODO: Reminder to make sure not to add bat1 ever to upper CBMA
                if interface.interface_name == "bat1":
                    continue
                self.__upper_cbma_interfaces.append(interface)

        # Lower and upper CBMA interfaces are mutually exclusive so remove
        # upper CBMA interfaces from lower CBMA interface list
        for interface in self.__upper_cbma_interfaces:
            if interface in self.__lower_cbma_interfaces:
                self.__lower_cbma_interfaces.remove(interface)

    def __wait_for_batman_interfaces(self, timeout=3):
        start_time = time.time()
        while True:
            with self.__lock:
                found = any(
                    interface.interface_name == "bat0"
                    for interface in self.__interfaces
                ) and any(
                    interface.interface_name == "bat1"
                    for interface in self.__interfaces
                )
                if found:
                    return True
            elapsed_time = time.time() - start_time

            if elapsed_time >= timeout:
                return False  # Timeout reached

            time.sleep(1)  # Sleep for 1 second before checking again
            self.__get_interfaces()

    def setup_cbma(self):
        """
        Sets up both upper and lower CBMA.
        """
        # Cleanup CBMA before starting it
        self.__cleanup_cbma()
        # FIXME: bridge name/control in general
        self.__remove_all_interfaces_from_bridge("br-lan")

        # TODO - Shall this be done differently?
        # Create batman interfaces
        batman("bat0")
        batman("bat1")

        self.__get_interfaces()
        self.__wait_for_batman_interfaces()
        self.__update_cbma_interface_lists()
        self.__setup_lower_cbma()
        self.__setup_upper_cbma()

        # Add interfaces to the bridge
        bridge_setting = self.__comms_ctrl.settings.bridge[0]
        bridge_setting = bridge_setting.strip('"')
        components = bridge_setting.split()
        bridge_name = components[0]
        interface_names = components[1:]

        not_allowed_interfaces_in_bridge = ["bat0"]
        for not_allowed_interface in not_allowed_interfaces_in_bridge:
            if not_allowed_interface in interface_names:
                interface_names.remove(not_allowed_interface)

        # FIXME: Hard coded red interfaces for testing purposes
        # Eventually MDM server will define interface colors
        mandatory_interfaces = ["bat1", "eth0", "eth1", "usb0"]
        for mandatory_interface in mandatory_interfaces:
            if mandatory_interface not in interface_names:
                interface_names.append(mandatory_interface)

        for interface in interface_names:
            self.__add_interface_to_bridge(bridge_name, interface)
        self.__cbma_set_up = True
        return True

    def __setup_lower_cbma(self):
        for interface in self.__lower_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            try:
                index = self.__comms_ctrl.settings.mesh_vif.index(interface_name)
                wpa_supplicant_ctrl_path = (
                    f"/var/run/wpa_supplicant_id{index}/{interface_name}"
                )
            except ValueError:
                wpa_supplicant_ctrl_path = None

            self.logger.debug(
                "Setup lower CBMA for interface: %s, wpa_supplicant_ctrl_path: %s",
                interface_name,
                wpa_supplicant_ctrl_path,
            )
            process = setup_cbma.cbma(
                "lower",
                interface_name,
                Constants.CBMA_PORT_LOWER.value,
                "bat0",
                self.__cbma_certs_path,
                self.__cbma_ca_cert_path,
                "off",
                wpa_supplicant_ctrl_path,
            )
            self.__lower_cbma_processes[interface_name] = process

    def __setup_upper_cbma(self):
        for interface in self.__upper_cbma_interfaces:
            interface_name = interface.interface_name.lower()
            try:
                index = self.__comms_ctrl.settings.mesh_vif.index(interface_name)
                wpa_supplicant_ctrl_path = (
                    f"/var/run/wpa_supplicant_id{index}/{interface_name}"
                )
            except ValueError:
                wpa_supplicant_ctrl_path = None

            if self.__has_certificate(
                self.__upper_cbma_certs_path, interface.mac_address
            ):
                self.logger.debug(
                    "Using upper cbma certificate for interface %s", interface_name
                )
                cbma_certs_path = self.__upper_cbma_certs_path
                cbma_ca_cert_path = self.__upper_cbma_ca_cert_path
            else:
                # TODO: Temporary backup solution to use lower CBMA certificates
                if self.__has_certificate(
                    self.__cbma_certs_path, interface.mac_address
                ):
                    self.logger.debug(
                        "Using lower cbma certificate as a backup for interface %s",
                        interface_name,
                    )
                    cbma_certs_path = self.__cbma_certs_path
                    cbma_ca_cert_path = self.__cbma_ca_cert_path
                else:
                    self.logger.debug(
                        "No cbma certificate for interface %s", interface_name
                    )
                    continue

            self.logger.debug(
                "Setup upper CBMA for interface: %s, wpa_supplicant_ctrl_path: %s",
                interface_name,
                wpa_supplicant_ctrl_path,
            )

            process = setup_cbma.cbma(
                "upper",
                interface_name,
                Constants.CBMA_PORT_UPPER.value,
                "bat1",
                cbma_certs_path,
                cbma_ca_cert_path,
                "off",
                wpa_supplicant_ctrl_path,
            )
            self.__upper_cbma_processes[interface_name] = process
