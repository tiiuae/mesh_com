"""
This module contains CommsServiceMonitor class which is intended
to be instantiated/used by both MDM agent and "FMO agent"/CommsController
and to be run in separate thread to get service url and its status changes.
"""

import time
from typing import Optional, Callable

import netifaces
from zeroconf import ServiceStateChange, Zeroconf
from zeroconf import ServiceBrowser, ServiceInfo


class CommsServiceMonitor:
    """
    A class to perform service discovery for given service
    that is available via diven network interface. In case
    interface is not specified then service discovery is using
    all available interfaces. Service status is reported back to client
    via given callabck function.

    Arguments:
        service_name (str) -- For example "My Service"
        service_type (str) -- FQDN for the service to monitor.
                              For example "_myservice._tcp.local."
        service_cb -- Function pointer to a function that takes
                      str and bool values as arguments. String value
                      example: "myservice.local:8000".
        interface (str) -- Network interface name to use in service
                           discovery.
    """
    def __init__(
        self,
        service_name: str,
        service_type: str,
        service_cb: Optional[Callable] = None,
        interface: Optional[str] = None
    ) -> None:
        self.service_name = service_name
        self.service_type = service_type
        self.service_browser: Optional[ServiceBrowser] = None
        self.zeroconf: Optional[Zeroconf] = None
        self.service_callback = service_cb
        self.interface = interface
        self.running = False

    @staticmethod
    def __get_ip_addresses(interface):
        addresses = []

        # Get all addresses for the specified interface
        if interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)

            # Get IPv4 addresses
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    addresses.append(addr_info['addr'])

            # Get IPv6 addresses
            if netifaces.AF_INET6 in addrs:
                for addr_info in addrs[netifaces.AF_INET6]:
                    addresses.append(addr_info['addr'])

        return addresses

    def run(self) -> None:
        """
        Starts service discovery.

        Arguments:
            None

        Returns:
            None
        """
        self.running = True
        if self.interface:
            addresses = self.__get_ip_addresses(self.interface)
            self.zeroconf = Zeroconf(interfaces=addresses)
        else:
            self.zeroconf = Zeroconf()

        self.service_browser = ServiceBrowser(
            self.zeroconf,
            self.service_type,
            handlers=[self.__on_service_state_change]
        )

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.close()

    def close(self) -> None:
        """
        Stops service discovery.

        Arguments:
            None

        Returns:
            None
        """
        self.running = False
        assert self.zeroconf is not None
        assert self.service_browser is not None
        self.service_browser.cancel()
        self.zeroconf.close()

    def __on_service_state_change(
        self,
        zeroconf: Zeroconf,
        service_type: str,
        name: str,
        state_change: ServiceStateChange,
    ) -> None:
        print(
            f"Service {name} of type {service_type} state changed: {state_change}"
        )
        if not self.running:
            return

        if name == f"{self.service_name}.{self.service_type}":
            info = ServiceInfo(service_type, name)
            service_available = True
            if state_change == ServiceStateChange.Removed:
                service_available = False
            info.request(zeroconf, 3000)

            if self.service_callback:
                server = info.server
                if server:
                    server = server.rstrip(".")
                url = f'{server}:{info.port}'
                self.service_callback(url, service_available)


if __name__ == "__main__":

    def service_discovery_cb(url, status):
        """
        An example callback function.

        Arguments:
            url (str) -- String that contains server and port.
                         For example "myservice.local:8000" or None.
            status (bool) -- Boolean to indicate is service available or not.

        Returns:
            None
        """
        print(f"Callback received, service url {url}, online: {status}")

    monitor = CommsServiceMonitor(
        service_name="MDM Service",
        service_type="_mdm._tcp.local.",
        service_cb=service_discovery_cb,
        #interface="br-lan"
    )
    try:
        monitor.run()
    except KeyboardInterrupt:
        monitor.close()
