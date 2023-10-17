import asyncio
import client
import json
import config

async def main():
    # Connect to NATS!
    nc = await client.connect_nats()
    cmd_dict = {
        "api_version": 1,
        "role": f"{config.MODULE_ROLE}",
        "radios": [
            {
                "radio_index": "0",
                "ssid": "test_mesh",
                "key": "1234567890",
                "ap_mac": "00:11:22:33:44:55",
                "country": "US",
                "frequency": "5220",
                "frequency_mcc": "2412",
                "routing": "batman-adv",
                "priority": "long_range",
                "ip": "192.168.1.2",
                "subnet": "255.255.255.0",
                "tx_power": "5",
                "mode": "mesh",
                "mesh_vif": "wlp2s0",
                "phy": "phy0",
                "batman_iface": "bat0",
            },
            {
                "radio_index": "1",
                "ssid": "test_mesh2",
                "key": "1234567890",
                "ap_mac": "00:11:22:33:44:55",
                "country": "US",
                "frequency": "2412",
                "frequency_mcc": "2412",
                "routing": "batman-adv",
                "priority": "long_range",
                "ip": "192.168.1.2",
                "subnet": "255.255.255.0",
                "tx_power": "5",
                "mode": "mesh",
                "mesh_vif": "wlp3s0",
                "phy": "phy2",
                "batman_iface": "bat0",
            },
            # {
            #     "radio_index": "2",
            #     "ssid": "test_mesh3",
            #     "key": "1234567890",
            #     "ap_mac": "00:11:22:33:44:55",
            #     "country": "US",
            #     "frequency": "5190",
            #     "frequency_mcc": "2412",
            #     "routing": "batman-adv",
            #     "priority": "long_range",
            #     "ip": "192.168.1.2",
            #     "subnet": "255.255.255.0",
            #     "tx_power": "5",
            #     "mode": "halow",
            #     "mesh_vif": "halow1",
            #     "phy": "phy2",
            #     "batman_iface": "bat0",
            # },
        ],
        "bridge": "br-lan bat0 eth1 lan1"
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
