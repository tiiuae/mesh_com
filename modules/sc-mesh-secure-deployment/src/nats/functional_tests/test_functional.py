"""
Functional test to test the communication between the controller and the module
"""
import unittest
import json
import base64
import asyncio
from nats.aio.client import Client as NATS

try:
    import config
    import client

    SKIP_TEST = False
except ModuleNotFoundError:
    SKIP_TEST = True


@unittest.skipIf(
    SKIP_TEST, "Functional tests need to be executed from the script directory"
)
class TestFunctional(unittest.IsolatedAsyncioTestCase):
    """
    Functional test to test the communication between the controller and the module
    """
    async def test_identity(self):
        """
        Test cases for comms_settings.py
        """
        received = False
        # settings json is valid
        nc = await client.connect_nats()
        cmd_dict = {"api_version": 1, "cmd": "GET_IDENTITY"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request("comms.identity", cmd.encode(), timeout=2)

        parameters = json.loads(rep.data.decode())

        if "identity" in parameters["data"]:
            with open("identity.py", "w", encoding="utf-8") as f:
                f.write(f"MODULE_IDENTITY=\"{parameters['data']['identity']}\"\n")
            received = True
        else:
            print("No identity received!!!!!!!!!!!")
        await nc.close()

        self.assertTrue(received)

    async def test_status(self):
        """
        Test cases for comms
        """
        received = False
        nc = await client.connect_nats()
        cmd_dict = {"api_version": 1}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.status.{config.MODULE_IDENTITY}", cmd.encode(), timeout=1
        )
        parameters = json.loads(rep.data)
        if parameters["status"] == "OK":
            received = True
        await nc.close()
        self.assertTrue(received)

    async def test_settings_and_apply(self):
        """
        Functional test to test the communication between the controller and the module
        """
        received = False

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
                    "mptcp": "disable",  # MPTCP support feature flag, enable/disable
                    "priority": "long_range",
                    "ip": "10.20.15.3",
                    "slaac": "",
                    "subnet": "255.255.255.0",
                    "tx_power": "15",
                    "mode": "mesh",  # ap+mesh_scc, mesh, halow
                    "mesh_vif": "wlp2s0",
                    "batman_iface": "bat0",
                    "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                },
                # {
                #     "radio_index": "1",
                #     "ssid": "test_mesh",
                #     "key": "1234567890",
                #     "ap_mac": "00:11:22:33:44:55",
                #     "country": "US",  # all radios must have the same country
                #     "frequency": "5220",
                #     "frequency_mcc": "2412",  # multiradio not supporting
                #     "routing": "batman-adv",
                #     "mptcp": "disable",  # MPTCP support feature flag, enable/disable
                #     "priority": "long_range",
                #     "ip": "10.20.15.3",
                #     "subnet": "255.255.255.0",
                #     "slaac": "",
                #     "tx_power": "15",
                #     "mode": "mesh",  # ap+mesh_scc, mesh, halow
                #     "mesh_vif": "wlp3s0",  # this needs to be correct
                #     "batman_iface": "bat0",
                #     "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                # },
                # {
                #     "radio_index": "2",
                #     "ssid": "test_mesh3",
                #     "key": "1234567890",
                #     "ap_mac": "00:11:22:33:44:55",
                #     "country": "US",  # all radios must have the same country
                #     "frequency": "5190",
                #     "frequency_mcc": "2412",  # multiradio not supporting
                #     "routing": "batman-adv",
                #     "mptcp": "disable",  # MPTCP support feature flag, enable/disable
                #     "priority": "long_range",
                #     "ip": "10.20.15.3",
                #     "slaac": "",
                #     "subnet": "255.255.255.0",
                #     "tx_power": "30",
                #     "mode": "halow",  # ap+mesh_scc, mesh, halow
                #     "mesh_vif": "halow1",
                #     "batman_iface": "bat0",
                #     "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
                # },
            ],
        }

        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.settings.{config.MODULE_IDENTITY}", cmd.encode(), timeout=10
        )
        parameters = json.loads(rep.data)

        if (
            parameters["status"] == "OK"
            and parameters["mesh_cfg_status"][0] == "MESH_CONFIGURATION_PENDING"
        ):
            received = True

        self.assertTrue(received)

        for radio in cmd_dict["radios"]:
            # Extract the radio_index
            index_number = radio["radio_index"]

            # Create the command
            cmd = json.dumps(
                {
                    "api_version": 1,
                    "cmd": "APPLY",
                    "radio_index": index_number,
                }
            )
            rep = await nc.request(
                f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=15
            )
            parameters = json.loads(rep.data)

            if parameters["status"] == "OK":
                received = True
                self.assertTrue(received)
            else:
                received = False
                self.assertTrue(received)

        await nc.close()

    async def test_get_configs(self):
        """
        Test cases for comms_settings.py
        """

        received = False
        nc = await client.connect_nats()

        cmd_dict = {
            "api_version": 1,
            "cmd": "GET_CONFIG",
            "param": "WPA_CONFIG",
            "radio_index": "0",
        }
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=4
        )
        parameters = json.loads(rep.data.decode())

        if parameters["status"] == "OK":
            received = True
        elif parameters["status"] == "FAIL":
            print(parameters)
        self.assertTrue(received)

    async def test_visual(self):
        """
        Test cases for comms
        """
        received = False
        nc = await client.connect_nats()
        cmd_dict = {
            "api_version": 1,
            "cmd": "ENABLE_VISUALISATION",
            "interval": "1000",
            "radio_index": "0",
        }
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=2
        )
        parameters = json.loads(rep.data.decode())

        await nc.close()

        data = await asyncio.wait_for(self.visual_helper(), 15)
        d = data[1]
        # todo: check if the data is correct
        if d["status"][0] == "MESH":
            received = True

        self.assertTrue(received)

        nc = await client.connect_nats()
        cmd_dict = {
            "api_version": 1,
            "cmd": "DISABLE_VISUALISATION",
            "interval": "1000",
            "radio_index": "0",
        }
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=2
        )
        parameters = json.loads(rep.data.decode())
        await nc.close()

        if parameters["status"] == "OK":
            received = True
        else:
            received = False

        self.assertTrue(received)

    async def test_logs(self):
        """
        Test cases for comms
        """
        received = False

        nc = await client.connect_nats()

        cmd_dict = {"api_version": 1, "cmd": "LOGS", "param": "CONTROLLER"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            received = True

        self.assertTrue(received)

        cmd_dict = {"api_version": 1, "cmd": "LOGS", "param": "DMESG"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            received = True
        else:
            received = False

        self.assertTrue(received)

        cmd_dict = {"api_version": 1, "cmd": "LOGS", "param": "WPA", "radio_index": "0"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["data"] is None:
            print("No logs available")
            received = False
        else:
            received = True

        self.assertTrue(received)

    async def test_channel_change(self):
        """
        Test cases for comms
        """
        functional = False

        nc = await client.connect_nats()
        cmd_dict = {
            "api_version": 1,
            "cmd": "GET_CONFIG",
            "param": "WPA_CONFIG",
            "radio_index": "0",
        }
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            data = base64.b64decode(parameters["data"].encode()).decode()

            frequency = "2412"
            # find "frequency=" line in data
            for line in data.split("\n"):
                if "frequency=" in line:
                    frequency = line.split("=")[1]
                    break

            if frequency == "2412":
                new_frequency = "2452"
            else:
                new_frequency = "2412"

            cmd_dict = {"frequency": new_frequency, "radio_index": "0"}
            cmd = json.dumps(cmd_dict)
            rep = await nc.request(
                f"comms.channel_change.{config.MODULE_IDENTITY}", cmd.encode(), timeout=10
            )
            parameters = json.loads(rep.data)
            if parameters["status"] == "OK":
                functional = True
            else:
                functional = False
            self.assertTrue(functional)
        else:
            self.assertTrue(functional)
        await nc.close()

    async def visual_helper(self) -> []:
        """
        Helper method to receive the visualisation data
        returns: the data received
        """
        nc = NATS()

        self.data = None
        done = asyncio.Event()

        async def closed_cb():
            print("Connection to NATS is closed.")
            if nc.is_closed:
                return
            await nc.close()
            return self.data

        async def reconnected_cb():
            print("Connected to NATS ...")

        async def subscribe_handler(msg):
            self.data = json.loads(msg.data.decode())
            done.set()  # Signal that we received the message and can return

        try:
            await client.connect(
                nc,
                recon_cb=reconnected_cb,
                closed_cb=closed_cb,
                max_reconnection_attempts=-1,
            )
        except Exception as e:
            print(e)

        print(f"Connected to NATS at: {nc.connected_url.netloc}")

        await nc.subscribe("comms.visual.*", "", cb=subscribe_handler)
        await done.wait()  # Wait for the event to be set
        return self.data  # Return the data


if __name__ == "__main__":
    unittest.main()
