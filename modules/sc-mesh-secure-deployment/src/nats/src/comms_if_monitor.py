"""
This module contains CommsInterfaceMonitor class which is intended
to be used by MDM agent to get network interface statuss.
"""

# pylint: disable=too-few-public-methods, too-many-nested-blocks
from copy import deepcopy
from pyroute2 import IPRoute


class CommsInterfaceMonitor:
    """
    A class to get network interfaces status.
    """

    def __init__(self, callback) -> None:
        self.__interfaces = []
        self.__previous_interfaces = []
        self.__callback = callback
        self.__running = False
        self.__ipr = IPRoute()

    def __get_initial_interfaces(self):
        for link in self.__ipr.get_links():
            interface_info = self.__get_interface_info(link)
            if interface_info:
                self.__interfaces.append(interface_info)

        if self.__callback:
            self.__callback(self.__interfaces)

    def __get_interface_info(self, link):
        ifname = link.get_attr("IFLA_IFNAME")
        ifstate = link.get_attr("IFLA_OPERSTATE")
        mac_address = link.get_attr("IFLA_ADDRESS")

        interface_info = {
            "interface_name": ifname,
            "operstate": ifstate,
            "mac_address": mac_address,
        }
        return interface_info

    def monitor_interfaces(self):
        """
        Monitors network interfaces using IPRoute.
        Changes are acknowledged to clients via callback
        function.

        Arguments:
            None

        Returns:
            None
        """
        self.__get_initial_interfaces()
        self.__running = True
        self.__ipr.bind()

        while self.__running:
            try:
                # Hox! get() is a blocking call thus stop() doesn't
                # have much affect when execution is blocked within get().
                messages = self.__ipr.get()
                for msg in messages:
                    if msg["event"] == "RTM_NEWLINK" or msg["event"] == "RTM_DELLINK":
                        interface_info = self.__get_interface_info(msg)
                        if interface_info:
                            existing_interface = next(
                                (
                                    iface
                                    for iface in self.__interfaces
                                    if iface["interface_name"]
                                    == interface_info["interface_name"]
                                ),
                                None,
                            )
                            if msg["event"] == "RTM_DELLINK" and existing_interface:
                                self.__interfaces.remove(existing_interface)
                            else:
                                if existing_interface:
                                    existing_interface.update(interface_info)
                                else:
                                    self.__interfaces.append(interface_info)
                            if (
                                self.__callback
                                and self.__interfaces != self.__previous_interfaces
                            ):
                                self.__callback(self.__interfaces)
                                self.__previous_interfaces = deepcopy(self.__interfaces)
            except KeyboardInterrupt:
                break
        self.__ipr.close()

    def stop(self):
        self.__running = False
        self.__ipr.close()
