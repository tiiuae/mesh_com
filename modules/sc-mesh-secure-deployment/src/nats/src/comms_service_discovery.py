import asyncio
from typing import Optional, Callable
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
    ) -> None:
        self.service_name = service_name
        self.service_type = service_type
        self.service_browser: Optional[AsyncServiceBrowser] = None
        self.async_zeroconf: Optional[AsyncZeroconf] = None
        self.service_callback = service_cb

    async def async_run(self) -> None:
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

        info = AsyncServiceInfo(service_type, name)
        service_available = True
        if state_change == ServiceStateChange.Removed:
            service_available = False
        await info.async_request(zeroconf, 3000)

        service_info = self.extract_service_info(info)
        expected_name = name
        if self.service_name:
            expected_name = f"{self.service_name}.{self.service_type}"

        if self.service_callback:
            if service_info["ipv6_address"]:
                url = f'{service_info["ipv6_address"]}:{service_info["port"]}'
            else:
                url = f'{service_info["ipv4_address"]}:{service_info["port"]}'
            if service_info["name"] == expected_name:
                self.service_callback(url, service_available)

    def extract_service_info(self, info: AsyncServiceInfo) -> dict:
        if info:
            ipv6_addresses = [
                addr for addr in info.parsed_scoped_addresses() if ":" in addr
            ]
            ipv6_address = next(iter(ipv6_addresses), None)
            ipv4_addresses = [
                addr for addr in info.parsed_scoped_addresses() if "." in addr
            ]
            ipv4_address = next(iter(ipv4_addresses), None)

            service_info = {
                "name": info.name,
                "ipv6_address": ipv6_address,
                "ipv4_address": ipv4_address,
                "port": info.port,
                "weight": info.weight,
                "priority": info.priority,
                "server": info.server,
                "properties": info.properties,
            }
            return service_info
        else:
            return {
                "name": None,
                "ipv6_address": None,
                "ipv4_address": None,
                "port": None,
                "weight": None,
                "priority": None,
                "server": None,
                "properties": None,
            }


if __name__ == "__main__":

    def service_discovery_cb(url, online_status):
        print(f"Callback received, service url {url}, online: {online_status}")

    loop = asyncio.get_event_loop()
    runner = CommsServiceMonitor(
        service_name="MDM Service",
        service_type="_mdm._tcp.local.",
        service_cb=service_discovery_cb,
    )
    try:
        loop.run_until_complete(runner.async_run())
    except KeyboardInterrupt:
        loop.run_until_complete(runner.async_close())
