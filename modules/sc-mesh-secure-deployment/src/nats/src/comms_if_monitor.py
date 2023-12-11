"""
This module contains CommsInterfaceMonitor class which is intended
to be used by MDM agent to get network interface statuss.
"""

# pylint: disable=too-few-public-methods, too-many-nested-blocks

from pyroute2 import IPRoute


class CommsInterfaceMonitor:
    """
    A class to get network interfaces status.
    """
    def __init__(self, callback) -> None:
        self.__interfaces = {}
        self.__callback = callback
        self.running = False
        self.__ipr = IPRoute()

    def __get_initial_interfaces(self):
        for link in self.__ipr.get_links():
            ifname = link.get_attr('IFLA_IFNAME')
            ifstate = link.get_attr('IFLA_OPERSTATE')
            if ifname and ifstate:
                self.__interfaces[ifname] = ifstate
        if self.__callback:
            self.__callback(self.__interfaces)

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

        self.running = True
        # Subscribe to netlink events for network interfaces
        self.__ipr.bind()

        while self.running:
            try:
                messages = self.__ipr.get()
                for msg in messages:
                    if msg['event'] == 'RTM_NEWLINK' or msg['event'] == 'RTM_DELLINK':
                        # Get interface name and its status
                        ifname = msg.get_attr('IFLA_IFNAME')
                        ifstate = msg.get_attr('IFLA_OPERSTATE')

                        if ifname and ifstate:
                            # Update the initial list with the changed state
                            if msg['event'] == 'RTM_DELLINK' and ifname in self.__interfaces:
                                # Interface is deleted, remove it from the list
                                del self.__interfaces[ifname]
                            else:
                                # Interface is added or modified, update its state
                                self.__interfaces[ifname] = ifstate
                            if self.__callback:
                                self.__callback(self.__interfaces)
            except KeyboardInterrupt:
                break
        self.__ipr.close()


def main():
    """
    Function to test interface monitoring 
    by running module locally.
    """
    def callback(interfaces):
        print("Callback received:", interfaces)

    monitor = CommsInterfaceMonitor(callback)
    monitor.monitor_interfaces()


if __name__ == "__main__":
    main()
