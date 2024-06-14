"""
Debug test to test the DEBUG COMMAND
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
    SKIP_TEST, "Debug tests need to be executed from the script directory"
)
class TestDebug(unittest.IsolatedAsyncioTestCase):
    """
    Debug test to test the DEBUG COMMAND
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

    async def test_debug(self):
        """
        Test cases for comms
        """
        received = False

        nc = await client.connect_nats()

        cmd_dict = {"api_version": 1, "cmd": "DEBUG", "param": "cat /root/build.info"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            received = True
        print(parameters["info"])

        self.assertTrue(received)
        b64_data = base64.b64decode(parameters["data"].encode())
        print(b64_data.decode())

    async def test_slaac(self):
        """
        Test cases for comms
        """
        received = False

        nc = await client.connect_nats()

        cmd_dict = {"api_version": 1, "cmd": "DEBUG", "param": "/opt/slaac.sh wlp1s0"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            received = True
        print(parameters["info"])

        self.assertTrue(received)
        b64_data = base64.b64decode(parameters["data"].encode())
        print(b64_data.decode())

        await asyncio.sleep(float(40))

        cmd_dict = {"api_version": 1, "cmd": "DEBUG", "param": "ifconfig wlp1s0"}
        cmd = json.dumps(cmd_dict)
        rep = await nc.request(
            f"comms.command.{config.MODULE_IDENTITY}", cmd.encode(), timeout=5
        )
        parameters = json.loads(rep.data.decode())
        if parameters["status"] == "OK":
            received = True
        print(parameters["info"])

        self.assertTrue(received)
        b64_data = base64.b64decode(parameters["data"].encode())
        print(b64_data.decode())


if __name__ == "__main__":
    unittest.main()
