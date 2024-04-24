import unittest
from unittest.mock import patch, MagicMock
import subprocess
import os
import yaml

from pyroute2 import IPRoute
from src.bat_ctrl_utils import BatCtrlUtils

@unittest.skipUnless(os.getuid() == 0, "Skipping test, must be run as root")
class TestBatCtrlUtils(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.yaml_content_1 = {
            "BATMAN": {
                # Testing with BATMAN_IV as PC typically doesn't support BATMAN_V
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 5},
                    "hardif": {"ifdummy0": 3, "ifdummy1": 4},
                },
            }
        }
        cls.yaml_content_2 = {
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
            },
        }
        cls.yaml_content_3 = {
            "BATMAN": {
                "routing_algo": "OLSR",
            },
        }
        # Write YAML content to files
        # Not forcing yaml file content in same order as in dictionary in order
        # to ensure yaml file parsing work properly.
        with open("test_config_1.yaml", "w", encoding="utf-8") as file_1:
            yaml.dump(cls.yaml_content_1, file_1)
        with open("test_config_2.yaml", "w", encoding="utf-8") as file_2:
            yaml.dump(cls.yaml_content_2, file_2)
        with open("test_config_3.yaml", "w", encoding="utf-8") as file_2:
            yaml.dump(cls.yaml_content_3, file_2)

    @classmethod
    def tearDownClass(cls):
        # Delete created YAML files
        os.remove("test_config_1.yaml")
        os.remove("test_config_2.yaml")
        os.remove("test_config_3.yaml")

    # @patch('')
    def setUp(self):
        self.logger = MagicMock()
        self.bat_utils = BatCtrlUtils(
            self.logger,
            batman_ra="BATMAN_V",
            config_file="test_config_1.yaml",
        )
    
    def test_config_reading_success(self):
        print("hp:", self.bat_utils._BatCtrlUtils__hop_penalty)
        print("ra:", self.bat_utils._BatCtrlUtils__batman_ra)

        # Ensure batman routing algorithm was applied
        self.assertEqual(self.bat_utils._BatCtrlUtils__batman_ra, "BATMAN_IV")
        # Check hardif ho penalties
        self.assertEqual(
            self.bat_utils._BatCtrlUtils__hop_penalty["hardif"],
            self.yaml_content_1["BATMAN"]["hop_penalty"]["hardif"],
        )
        # Check meshif hop penalties
        self.assertEqual(
            self.bat_utils._BatCtrlUtils__hop_penalty["meshif"],
            self.yaml_content_1["BATMAN"]["hop_penalty"]["meshif"],
        )

    def test_config_reading_success_with_second_yaml(self):
        self.bat_utils2 = BatCtrlUtils(
            self.logger,
            batman_ra="BATMAN_V",
            config_file="test_config_2.yaml",
        )
        # Ensure batman routing algorithm was applied
        self.assertEqual(self.bat_utils2._BatCtrlUtils__batman_ra, "BATMAN_IV")
        self.assertEqual(self.bat_utils2._BatCtrlUtils__hop_penalty, None)

    @patch("src.bat_ctrl_utils.subprocess.run")
    def test_config_reading_success_with_third_yaml(self, mock_subprocess_run):
        self.bat_utils3 = BatCtrlUtils(
            self.logger,
            batman_ra="BATMAN_IV",
            # Config file tries to set routing algo to OLSR
            config_file="test_config_3.yaml",
        )
        # Ensure invalid routing algo was not tried to be applied
        self.assertEqual(self.bat_utils3._BatCtrlUtils__batman_ra, "BATMAN_IV")
        self.assertEqual(self.bat_utils3._BatCtrlUtils__hop_penalty, None)

        mock_subprocess_run.reset_mock()
        self.bat_utils3.set_hop_penalty()
        # Ensure that subprocess.run is not called
        mock_subprocess_run.assert_not_called()

    @patch("src.bat_ctrl_utils.IPRoute")
    def test_create_batman_interface(self, mock_iproute):
        """
        Tests the create_batman_interface method
        """
        mock_iproute_instance = MagicMock()
        mock_iproute.return_value = mock_iproute_instance

        # Simulate that create_batman_interface
        # doesn't find any existing interface
        mock_iproute_instance.link_lookup.return_value = []
        # Try to create bat0
        self.bat_utils.create_batman_interface("bat0")
        mock_iproute_instance.link.assert_called_once_with(
            "add", ifname="bat0", kind="batadv"
        )
        # Ensure bat0 was added to __bat_interfaces list
        self.assertIn("bat0", self.bat_utils._BatCtrlUtils__bat_interfaces)

        # Reset mock_iproute_instance
        mock_iproute_instance.link_lookup.return_value = []
        mock_iproute_instance.link.reset_mock()

        # Test creating bat1 interface with a specified MAC address
        self.bat_utils.create_batman_interface("bat1", "12:34:56:78:90:ab")
        mock_iproute_instance.link.assert_called_once_with(
            "add", ifname="bat1", kind="batadv", address="12:34:56:78:90:ab"
        )
        self.assertIn("bat1", self.bat_utils._BatCtrlUtils__bat_interfaces)

        # Reset mock
        mock_iproute_instance.link.reset_mock()
        # Test creating bat1 interface when simulated that one exists already
        mock_iproute_instance.link_lookup.return_value = 1
        self.bat_utils.create_batman_interface("bat1")
        mock_iproute_instance.link.assert_not_called()

    @patch("src.bat_ctrl_utils.subprocess.run")
    def test_set_batman_routing_algo(self, mock_subprocess_run):
        """
        Tests the __set_batman_routing_algo method
        """
        self.bat_utils._BatCtrlUtils__batman_ra = "BATMAN_V"
        self.bat_utils._BatCtrlUtils__set_batman_routing_algo()
        mock_subprocess_run.assert_called_once_with(
            ["batctl", "routing_algo", "BATMAN_V"], check=True
        )

    @patch("src.bat_ctrl_utils.subprocess.run")
    def test_configure_batman_interface(self, mock_subprocess_run):
        """
        Tests the configure_batman_interface method
        """
        self.bat_utils._BatCtrlUtils__bat_interfaces = ["bat0"]
        self.bat_utils.configure_batman_interface("bat0")
        expected_calls = [
            ["batctl", "meshif", "bat0", "aggregation", "0"],
            ["batctl", "meshif", "bat0", "bridge_loop_avoidance", "1"],
            ["batctl", "meshif", "bat0", "distributed_arp_table", "0"],
            ["batctl", "meshif", "bat0", "fragmentation", "1"],
            ["batctl", "meshif", "bat0", "orig_interval", "5000"],
            ["ip", "link", "set", "dev", "bat0", "mtu", "1546"],
        ]
        for call in expected_calls:
            mock_subprocess_run.assert_any_call(call, check=True)

        self.bat_utils.configure_batman_interface("bat1")
        expected_calls = [
            ["batctl", "meshif", "bat1", "aggregation", "0"],
            ["batctl", "meshif", "bat1", "bridge_loop_avoidance", "1"],
            ["batctl", "meshif", "bat1", "distributed_arp_table", "1"],
            ["batctl", "meshif", "bat1", "fragmentation", "1"],
            ["batctl", "meshif", "bat1", "orig_interval", "5000"],
            ["ip", "link", "set", "dev", "bat1", "mtu", "1500"],
        ]
        for call in expected_calls:
            mock_subprocess_run.assert_any_call(call, check=True)

    # @patch('src.bat_ctrl_utils.subprocess.run')
    def test_set_hop_penalty(self):
        """
        Integration test that tests the set_hop_penalty method
        and dependant __find_batman_hardif and __get_interface_mac methods.
        Test creates first bat0, ifdummy0 and lmb1234567890ab interfaces.
        lmb1234567890ab simulates macsec bridge that CBMA actually adds to bat0
        instead of requested dummyif0.
        Test tries to set hop penalty values for bat0, dummyif0 and dummyif1.
        Under the hood hop penalty should be actually set for lmb1234567890ab instead of
        ifdummy0 so finally test checks hop penalty values from bat0, lmb1234567890ab
        and ifdummy1 and compares do they match with expected values.
        """
        bat0_hp = "15"
        ifdummy0_hp = "0"
        ifdummy1_hp = "10"

        ip = IPRoute()
        try:
            # Simulate CBMA batman interface creation
            self.bat_utils.create_batman_interface("bat0")
            ip.link("add", ifname="ifdummy0", kind="dummy", address="12:34:56:78:90:ab")
            ip.link("add", ifname="ifdummy1", kind="dummy")
            ip.link("add", ifname="lmb1234567890ab", kind="dummy")

            # Add lmb1234567890ab to bat0
            subprocess.run(
                ["batctl", "meshif", "bat0", "if", "add", "lmb1234567890ab"], check=True
            )
            # Add ifdummy1 to bat0
            subprocess.run(
                ["batctl", "meshif", "bat0", "if", "add", "ifdummy1"], check=True
            )

            # Apply hop penalty settings from test_config_1.yaml
            self.bat_utils.set_hop_penalty()
            # Read active hop penalty settings
            bat0_hp = subprocess.run(
                ["batctl", "meshif", "bat0", "hp"],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
            ifdummy0_hp = subprocess.run(
                ["batctl", "hardif", "lmb1234567890ab", "hp"],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
            ifdummy1_hp = subprocess.run(
                ["batctl", "hardif", "ifdummy1", "hp"],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
        except Exception as e:
            print("Test failed! Error:", e)
        finally:
            # Try to delete all the interfaces that we created previously
            # Hox! For some reasason ip.link("delete") command doesn't
            # work if tests run under coverage.
            # Therefore deleting created interfaces using subprocess commands.
            try:
                subprocess.run(["ip", "link", "delete", "ifdummy0"], check=True)
            except Exception as e:
                print("Error deleting ifdummy1:", e)
            try:
                subprocess.run(["ip", "link", "delete", "ifdummy1"], check=True)
            except Exception as e:
                print("Error deleting ifdummy1:", e)
            try:
                subprocess.run(["ip", "link", "delete", "lmb1234567890ab"], check=True)
            except Exception as e:
                print("Error deleting lmb1234567890ab:", e)
            try:
                subprocess.run(["ip", "link", "delete", "bat0"], check=True)
            except Exception as e:
                print("Error deleting bat0:", e)

        ip.close()
        # Check settings were applied as expected
        self.assertEqual("5", bat0_hp)
        self.assertEqual("3", ifdummy0_hp)
        self.assertEqual("4", ifdummy1_hp)

    @patch("src.bat_ctrl_utils.IPRoute")
    def test_destroy_batman_interface(self, mock_iproute):
        """
        Tests the destroy_batman_interface method
        """
        mock_iproute_instance = MagicMock()
        mock_iproute.return_value = mock_iproute_instance

        # Add bat0 to __bat_interfaces list
        self.bat_utils._BatCtrlUtils__bat_interfaces = ["bat0"]
        # Try to destroy an interface that doesn't exist
        self.bat_utils.destroy_batman_interface("bat1")
        # Check that IPRoute.link() was not called
        mock_iproute_instance.link.assert_not_called()
        # Ensure that __bat_interfaces remains unchanged
        self.assertEqual(self.bat_utils._BatCtrlUtils__bat_interfaces, ["bat0"])

        # Try to destroy bat0 interface that is in __bat_interfaces
        self.bat_utils.destroy_batman_interface("bat0")
        # Check that IPRoute.link() was called once
        mock_iproute_instance.link.assert_called_once_with("delete", ifname="bat0")
        # Ensure bat0 got removed from __bat_interfaces
        self.assertNotIn("bat0", self.bat_utils._BatCtrlUtils__bat_interfaces)
