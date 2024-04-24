"""
Batman control utility module for CBMA Adaptation
"""
import logging
import subprocess
from typing import Optional, Union
import re
from pyroute2 import IPRoute  # type: ignore[import-not-found, import-untyped]

from src.constants import Constants
from src import comms_config_store


# pylint: disable=broad-except
class BatCtrlUtils(object):
    """
    Batman control utilities class
    """

    def __init__(
        self,
        logger: logging.Logger,
        batman_ra: str = "BATMAN_V",
        config_file: str = Constants.MS_CONFIG_FILE.value,
    ):
        """
        Constructor
        """
        self.logger: logging.Logger = logger.getChild("BatCtrlUtils")
        self.logger.setLevel(logging.INFO)
        self.__bat_interfaces = []
        # Default routing algo
        self.__batman_ra = batman_ra
        self.__config = None
        self.__batman_config = None
        self.__hop_penalty = None
        # Read configs from file
        try:
            self.__config = comms_config_store.ConfigStore(config_file)
        except Exception as e:
            self.logger.error(f"Error reading config file: {e}")
        if self.__config is not None:
            self.__batman_config = self.__config.read("BATMAN")
        if self.__batman_config:
            batman_ra = self.__batman_config.get("routing_algo")
            self.__hop_penalty = self.__batman_config.get("hop_penalty")
            self.logger.info(f"Batman routing algo config: {batman_ra}")
            self.logger.info(f"Batman hop penalty config: {self.__hop_penalty}")
            if batman_ra in ["BATMAN_V", "BATMAN_IV"]:
                self.__batman_ra = batman_ra
        # Set batman routing algo is set before any interfaces are created
        self.__set_batman_routing_algo()

    def create_batman_interface(
        self, batman_if: str, mac_addr: Optional[str] = None
    ) -> None:
        """
        Creates a Batman interface with given name and mac.
        If mac is not given then one is genereted.
        """
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
                # Add interface to the book keeping list
                self.__bat_interfaces.append(batman_if)
        except Exception as e:
            self.logger.error(
                "Error creating Batman interface %s: %s",
                batman_if,
                e,
            )
        finally:
            ip.close()

    def __set_batman_routing_algo(self) -> None:
        try:
            subprocess.run(["batctl", "routing_algo", self.__batman_ra], check=True)
        except subprocess.CalledProcessError as e:
            self.logger.error(
                "Error setting batman routing algo to %s: %s", self.__batman_ra, e
            )

    def configure_batman_interface(self, batman_if: str) -> None:
        """
        Configures batman interfaces with pre-defined default settings
        depending on interface name.
        """
        if batman_if not in self.__bat_interfaces:
            self.logger.error("Not allowed to configure %s", batman_if)

        is_upper = False
        if batman_if == Constants.UPPER_BATMAN.value:
            is_upper = True
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
            self.logger.error("Error configuring BATMAN interface: %s", e)

    def set_hop_penalty(self) -> None:
        """
        Sets Batman hop penalty for interfaces defined in config file.
        """
        if self.__hop_penalty is None:
            return
        # Set hop penalty for mesh interfaces
        meshif_hop_penalty = self.__hop_penalty.get("meshif", {})
        self.logger.info(f"meshif_hop_penalty: {meshif_hop_penalty}")
        if meshif_hop_penalty is not None:
            for interface, penalty in meshif_hop_penalty.items():
                if penalty is not None:
                    try:
                        subprocess.run(
                            ["batctl", "meshif", interface, "hop_penalty", str(penalty)],
                            check=True,
                        )
                    except Exception as e:
                        self.logger.info(
                            "Failed to set hop penalty %s for: %s. Error: %s",
                            penalty,
                            interface,
                            e,
                        )

        # Set hop penalty for hard interfaces
        hardif_hop_penalty = self.__hop_penalty.get("hardif", {})
        self.logger.info(f"hardif_hop_penalty: {hardif_hop_penalty}")
        if hardif_hop_penalty is not None:
            for interface, penalty in hardif_hop_penalty.items():
                if penalty is not None:
                    try:
                        # Serch hardif from all my batmans
                        for batman_interface in self.__bat_interfaces:
                            hardif = self.__find_batman_hardif(interface, batman_interface)
                            if hardif:
                                subprocess.run(
                                    [
                                        "batctl",
                                        "hardif",
                                        hardif,
                                        "hop_penalty",
                                        str(penalty),
                                    ],
                                    check=True,
                                )
                                break
                    except Exception as e:
                        self.logger.info(
                            "Failed to set hop penalty %s for: %s. Error: %s",
                            penalty,
                            interface,
                            e,
                        )

    @staticmethod
    def __get_interface_mac(interface: str) -> Union[None, str]:
        try:
            with open(
                file=f"/sys/class/net/{interface}/address",
                mode="r",
                encoding="utf-8",
            ) as file:
                return file.readline().strip()
        except FileNotFoundError:
            return None

    def __find_batman_hardif(self, interface: str, batman_if: str) -> str:
        # Check if the interface exists in the batctl if list
        try:
            batctl_output = subprocess.check_output(["batctl", batman_if, "if"]).decode(
                "utf-8"
            )
            if interface in batctl_output:
                self.logger.info(
                    "Interface %s found in %s interface list.", interface, batman_if
                )
                return interface
        except subprocess.CalledProcessError as e:
            self.logger.warning(
                "Interface %s not found in %s interface list. Error: %s",
                interface,
                batman_if,
                e,
            )

        # If interface is not found, try to find it using MAC address
        mac_address = self.__get_interface_mac(interface).replace(":", "")
        if mac_address:
            try:
                self.logger.debug(
                    f"Find hardif with mac: {mac_address} from {batman_if}"
                )
                batctl_output = subprocess.check_output(
                    ["batctl", batman_if, "if"]
                ).decode("utf-8")
                # batctl_output can contain multpile lines with syntax like:
                # lmb00301a4fc7d5: active
                for line in batctl_output.splitlines():
                    # Get characters before colon characters
                    interface_match = re.search(r"^(.*?):", line)
                    if interface_match:
                        interface_name = interface_match.group(1)
                        # If name contains expected mac?
                        if mac_address in interface_name:
                            self.logger.info(
                                "Interface %s found in %s interface list.",
                                interface_name,
                                batman_if,
                            )
                            return interface_name
            except subprocess.CalledProcessError as e:
                self.logger.error(
                    "Interface %s not found in %s if list with mac %s. Error: %s",
                    interface,
                    batman_if,
                    mac_address,
                    e,
                )

        return None

    def destroy_batman_interface(self, batman_if: str) -> None:
        """
        Deletes given Batman interface.
        """
        if batman_if not in self.__bat_interfaces:
            self.logger.error("Can not delete non-batman interface!")
            return

        ip = IPRoute()
        try:
            ip.link("delete", ifname=batman_if)
            self.__bat_interfaces.remove(batman_if)
        except Exception as e:
            self.logger.debug(f"Error: Unable to destroy interface {batman_if}.")
            self.logger.debug(f"Exception: {str(e)}")
        finally:
            ip.close()
