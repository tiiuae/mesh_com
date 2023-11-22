import asyncio
from typing import Optional, Callable

import netifaces
from zeroconf import ServiceStateChange, Zeroconf
from zeroconf.asyncio import (
    AsyncServiceBrowser,
    AsyncServiceInfo,
    AsyncZeroconf,
)


class CommsServiceMonitor:
    def __init__(
        self,
        service_name: str,
        service_type: str,
        service_cb: Optional[Callable] = None,
        interface: Optional[str] = None
    ) -> None:
        self.service_name = service_name
        self.service_type = service_type
        self.service_browser: Optional[AsyncServiceBrowser] = None
        self.async_zeroconf: Optional[AsyncZeroconf] = None
        self.service_callback = service_cb
        self.interface = interface

    @staticmethod
    def get_ip_addresses(interface):
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

    async def async_run(self) -> None:
        if self.interface:
            addresses = self.get_ip_addresses(self.interface)
            self.async_zeroconf = AsyncZeroconf(interfaces=addresses)
        else:
            self.async_zeroconf = AsyncZeroconf()
        self.service_browser = AsyncServiceBrowser(
            zeroconf=self.async_zeroconf.zeroconf,
            type_=self.service_type,
            handlers=[
                lambda *args, **kwargs: asyncio.ensure_future(
                    self.async_on_service_state_change(*args, **kwargs)
                )
            ],
        )
        while True:
            await asyncio.sleep(1)

    async def async_close(self) -> None:
        assert self.async_zeroconf is not None
        assert self.service_browser is not None
        await self.service_browser.async_cancel()
        await self.async_zeroconf.async_close()

    async def async_on_service_state_change(
        self,
        zeroconf: Zeroconf,
        service_type: str,
        name: str,
        state_change: ServiceStateChange,
    ) -> None:
        print(
            f"Service {name} of type {service_type} state changed: {state_change}"
        )

        if name == f"{self.service_name}.{self.service_type}":
            info = AsyncServiceInfo(service_type, name)
            service_available = True
            if state_change == ServiceStateChange.Removed:
                service_available = False
            await info.async_request(zeroconf, 3000)

            if self.service_callback:
                server = info.server
                if server:
                    server = server.rstrip(".")
                url = f'{server}:{info.port}'
                self.service_callback(url, service_available)


if __name__ == "__main__":

    def service_discovery_cb(url, online_status):
        print(f"Callback received, service url {url}, online: {online_status}")

    loop = asyncio.get_event_loop()
    runner = CommsServiceMonitor(
        service_name="MDM Service",
        service_type="_mdm._tcp.local.",
        service_cb=service_discovery_cb,
        #interface="br-lan"
    )
    try:
        loop.run_until_complete(runner.async_run())
    except KeyboardInterrupt:
        loop.run_until_complete(runner.async_close())
