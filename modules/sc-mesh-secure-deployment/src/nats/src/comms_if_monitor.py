"""
This module contains CommsInterfaceMonitor class which is intended
to be used by MDM agent to get network interface statuss.
"""

# pylint: disable=too-few-public-methods, too-many-nested-blocks
from typing import Callable, List, Dict
import subprocess
import time
from copy import deepcopy
from pyroute2 import IPRoute, NetlinkError

DUMMY_INTERFACE_NAME = 'ifdummy0'

class CommsInterfaceMonitor:
    """
    A class to get network interfaces status.
    """

    def __init__(self, callback: Callable[[List[Dict]], None]) -> None:
        self.__interfaces = []
        self.__previous_interfaces = []
        self.__callback = callback
        self.__running = False
        self.__ipr = IPRoute()

    def __get_initial_interfaces(self):
        ip_links = []
        while True:
            try:
                ip_links = self.__ipr.get_links()
                break
            except NetlinkError:
                time.sleep(1)

        for link in ip_links:
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
                # TODO - Using bufsize=-1 is broken in pyroute2 0.7.12
                # NOTE - bufsize=-1 required to prevent "No buffer space available" error
                messages = self.__ipr.get(bufsize=-1)
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
                                and self.__running
                                and self.__interfaces != self.__previous_interfaces
                            ):
                                self.__callback(self.__interfaces)
                                self.__previous_interfaces = deepcopy(self.__interfaces)
            except KeyboardInterrupt:
                break
        self.__ipr.close()

    def __create_dummy_interface(self, interface_name):
        """
        Create a dummy interface.

        Arguments:
            interface_name: str -- Name of the interface to be created.
        """
        subprocess.run(['ip', 'link', 'add', 'dev', interface_name, 'type', 'dummy'],
                           check=True)

    def __delete_dummy_interface(self, interface_name):
        """
        Delete a dummy interface.

        Arguments:
            interface_name: str -- Name of the interface to be deleted.
        """
        subprocess.run(['ip', 'link', 'delete', 'dev', interface_name], check=True)

    def stop(self):
        """
        Stop monitoring at latest after any waited RTNL message.
        No callbacks are provided after calling this method.

        Arguments:
            None

        Returns:
            None
        """
        self.__running = False
        # workaround to stop the blocking get() call
        self.__create_dummy_interface(DUMMY_INTERFACE_NAME)
        self.__delete_dummy_interface(DUMMY_INTERFACE_NAME)
