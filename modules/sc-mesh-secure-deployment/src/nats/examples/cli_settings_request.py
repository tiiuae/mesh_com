import asyncio
import client
import json
import config


async def main():
    # Connect to NATS!
    nc = await client.connect_nats()
    cmd_dict = {
        "api_version": 1,
        "role": f"{config.MODULE_ROLE}",  # sleeve, drone, gcs
        "radios": [
            {
                "radio_index": "0",
                "ssid": "test_mesh2",
                "key": "1234567890",
                "ap_mac": "00:11:22:33:44:55",
                "country": "US",  # all radios must have the same country
                "frequency": "2412",
                "frequency_mcc": "2412",  # multiradio not supporting
                "routing": "batman-adv",
                "mptcp": "disable",          # MPTCP support feature flag, enable/disable
                "priority": "long_range",
                "ip": "10.20.15.3",
                "subnet": "255.255.255.0",
                "tx_power": "15",
                "mode": "mesh",  # ap+mesh_scc, mesh, halow
                "mesh_vif": "wlp2s0",
                "batman_iface": "bat0",
                "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                "slaac": "usb0 wlp2s0", # SLAAC support for usb0 + mesh_vif
            },
            {
                "radio_index": "1",
                "ssid": "test_mesh",
                "key": "1234567890",
                "ap_mac": "00:11:22:33:44:55",
                "country": "US",  # all radios must have the same country
                "frequency": "5220",
                "frequency_mcc": "2412",  # multiradio not supporting
                "routing": "batman-adv",
                "mptcp": "disable",          # MPTCP support feature flag, enable/disable
                "priority": "long_range",
                "ip": "10.20.15.3",
                "subnet": "255.255.255.0",
                "tx_power": "15",
                "mode": "mesh",  # ap+mesh_scc, mesh, halow
                "mesh_vif": "wlp3s0",  # this needs to be correct
                "batman_iface": "bat0",
                "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                "slaac": "usb0 wlp3s0", # SLAAC support for usb0 + mesh_vif
            },
            {
                "radio_index": "2",
                "ssid": "test_mesh3",
                "key": "1234567890",
                "ap_mac": "00:11:22:33:44:55",
                "country": "US",  # all radios must have the same country
                "frequency": "5190",
                "frequency_mcc": "2412",  # multiradio not supporting
                "routing": "batman-adv",
                "mptcp": "disable",          # MPTCP support feature flag, enable/disable
                "priority": "long_range",
                "ip": "10.20.15.3",
                "subnet": "255.255.255.0",
                "tx_power": "30",
                "mode": "halow",  # ap+mesh_scc, mesh, halow
                "mesh_vif": "halow1",
                "batman_iface": "bat0",
                "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                "slaac": "usb0 halow1", # SLAAC support for usb0 + mesh_vif
            },
        ],
    }

    cmd = json.dumps(cmd_dict)
    rep = await nc.request(
        f"comms.settings.{config.MODULE_IDENTITY}", cmd.encode(), timeout=2
    )
    parameters = json.loads(rep.data)
    print(json.dumps(parameters, indent=2))

    await nc.close()
    exit(0)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
