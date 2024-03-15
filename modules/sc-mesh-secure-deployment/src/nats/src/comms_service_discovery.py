"""
This module contains CommsServiceMonitor class which is intended
to be instantiated/used by both MDM agent and "FMO agent"/CommsController
and to be run in separate thread to get service url and its status changes.
"""

import time
from typing import Optional, Callable
import logging
import inspect

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
        interface: Optional[str] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.service_name = service_name
        self.service_type = service_type
        self.service_browser: Optional[ServiceBrowser] = None
        self.zeroconf: Optional[Zeroconf] = None
        self.service_callback = service_cb
        self.interface = interface
        self.running = False
        if logger is None:
            self.__logger = logging.getLogger("service_monitor")
            self.__logger.setLevel(logging.DEBUG)
            log_formatter = logging.Formatter(
                fmt="%(asctime)s :: %(name)-18s :: %(levelname)-8s :: %(message)s"
            )
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(log_formatter)
            self.__logger.addHandler(console_handler)

        else:
            self.__logger = logger.getChild("service_monitor")

    @staticmethod
    def __get_ip_addresses(interface):
        addresses = []

        # Get all addresses for the specified interface
        if interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)

            # Get IPv4 addresses
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    addresses.append(addr_info["addr"])

            # Get IPv6 addresses
            if netifaces.AF_INET6 in addrs:
                for addr_info in addrs[netifaces.AF_INET6]:
                    addresses.append(addr_info["addr"])

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
            handlers=[self.__on_service_state_change],
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
        if self.zeroconf is not None:
            if self.service_browser is not None:
                self.service_browser.cancel()
            self.zeroconf.close()

    def __on_service_state_change(
        self,
        zeroconf: Zeroconf,
        service_type: str,
        name: str,
        state_change: ServiceStateChange,
    ) -> None:
        self.__logger.debug(
            "Service %s of type %s state changed: %s",
            name,
            service_type,
            state_change,
        )
        if not self.running:
            return

        # if service_name is given, filter using that. otherwise filter with service_type.
        eventMatch = False
        if not self.service_name:
            if service_type == self.service_type:
                eventMatch = True
        else:
            if name == f"{self.service_name}.{self.service_type}":
                eventMatch = True

        if eventMatch:
            info = ServiceInfo(service_type, name)
            service_available = True
            if state_change == ServiceStateChange.Removed:
                service_available = False
            info.request(zeroconf, 3000)

            if self.service_callback:
                server = info.server
                if server:
                    server = server.rstrip(".")

                kwargs = {"address": f'{server}:{info.port}',"service_name": name,
                          "ipv4_addresses": info._ipv4_addresses,
                          "ipv6_addresses": info._ipv6_addresses,
                          "port": info.port,"status": service_available}

                self.service_callback(**kwargs)


if __name__ == "__main__":

    def service_discovery_cb(address, status, **kwargs):
        """
        An example callback function.

        Arguments:
            url (str) -- String that contains server and port.
                         For example "myservice.local:8000" or None.
            status (bool) -- Boolean to indicate is service available or not.

        Arguments kwargs:
            service_name (str) -- Registered service name, 'myservice'
            ipv4_addresses (List(IPv4Address)) -- List of ipv4 addresses published for service.
            ipv6_addresses (List(IPv6Address)) -- List of ipv6 addresses published for service.
            port (int)-- tcp port of service
            status (bool) -- Boolean to indicate is service available or not.

        Returns:
            None
        """
        print(f"Callback received, service url {address}, online: {status}")
        print(f"{kwargs}")

    monitor = CommsServiceMonitor(
        service_name="MDM Service",
        service_type="_mdm._tcp.local.",
        service_cb=service_discovery_cb,
        # interface="br-lan"
    )
    try:
        monitor.run()
    except KeyboardInterrupt:
        monitor.close()
