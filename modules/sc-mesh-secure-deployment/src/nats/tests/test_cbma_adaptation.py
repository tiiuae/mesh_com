import subprocess
import os
import logging
import unittest
from unittest.mock import MagicMock, patch
from parameterized import parameterized

from pyroute2 import IPRoute
import yaml
from src.cbma_adaptation import CBMAAdaptation
from src import cbma_paths
from controller import CBMAController


# pylint: disable=broad-except, protected-access
class TestCBMAAdaptation(unittest.TestCase):

    INTERFACE_SETTINGS = [
        {
            "ifname": "ifdummy0_black",
            "kind": "dummy",
            "address": "12:34:56:78:90:ab",
            "state": "up",
        },
        {"ifname": "ifdummy2_white", "kind": "dummy", "state": "up"},
        {
            "ifname": "ifdummy2_white",
            "kind": "dummy",
            "address": "22:33:44:55:66:77",
            "state": "up",
        },
        {
            "ifname": "ifdummy3_black",
            "kind": "dummy",
            "address": "12:33:44:55:66:77",
            "state": "up",
        },
    ]

    FAKE_CERT_EXISTS_FOR_MAC = {
        "12:34:56:78:90:ab",
        "22:33:44:55:66:77",
        "12:33:44:55:66:77",
        "10:34:56:78:90:ab",
    }

    CBMA_FAKE_SUCCESS_FOR_INTERFACES = {"ifdummy0_black", "bat0"}

    @classmethod
    def setUpClass(cls):
        cls.logger = logging.getLogger(__name__)  # Unit test logger
        cls.logger.setLevel(logging.DEBUG)  # Set the logger level as needed
        # Add a StreamHandler to direct log output to stderr (terminal)
        cls.handler = logging.StreamHandler()
        cls.handler.setLevel(logging.DEBUG)  # Set the handler level as needed
        cls.logger.addHandler(cls.handler)

        ip = IPRoute()
        # Create interfaces using settings from the list
        for settings in cls.INTERFACE_SETTINGS:
            try:
                ip.link("add", **settings)
            except Exception as e:
                print(f"Error creating interface {settings.get('ifname')}: {e}")
        ip.close()

        cls.yaml_content_1 = {
            "CBMA": {
                "exclude_interfaces": [
                    "eth0",
                    "eth1",
                    "usb0",
                    "lan1",
                    "lan2",
                    "lan3",
                    "osf0",
                    "vlan_black",
                    "vlan_red",
                ],
                "white_interfaces": ["halow1", "ifdummy2_white"],
                "red_interfaces": ["ifdummy1_red", "vlan_red"],
            },
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 0, "bat1": 0},
                    "hardif": {"halow1": 20},
                },
            },
            "VLAN": {
                "vlan_black": {
                    "parent_interface": "ifdummy0_black",
                    "vlan_id": 100,
                    "ipv4_address": "192.168.1.1",
                    "ipv4_subnet_mask": "255.255.255.0",
                    "ipv6_local_address": "fe80::192.168.1.1",
                    "ipv6_prefix_length": 64,
                },
                "vlan_red": {"parent_interface": "ifdummy0_black", "vlan_id": 200},
            },
        }
        cls.yaml_content_2 = {
            "CBMA": {
                "exclude_interfaces": [],
                "white_interfaces": ["ifdummy2_white"],
                "red_interfaces": ["ifdummy1_red", "vlan_red"],
            },
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 0, "bat1": 0},
                    "hardif": {"halow1": 20},
                },
            },
        }

        cls.yaml_content_3 = """
            CBMA:
            exclude_interfaces:
                - eth0
                - eth1
                - usb0
                - lan1
                - lan2
                - lan3
                - osf0
                - vlan_black
                - "vlan_red
            white_interfaces
                - halow1
            red_interfaces:
                - wlan1
            """
        # No black interfaces left if applied exclude_interfaces
        cls.yaml_content_4 = {
            "CBMA": {
                "exclude_interfaces": ["ifdummy0_black", "ifdummy3_black"],
                "white_interfaces": ["ifdummy2_white"],
                "red_interfaces": ["ifdummy1_red", "vlan_red"],
            },
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 0, "bat1": 0},
                    "hardif": {"halow1": 20},
                },
            },
        }
        # No black interfaces if applied exclude_interfaces + white_interfaces
        cls.yaml_content_5 = {
            "CBMA": {
                "exclude_interfaces": ["ifdummy0_black"],
                "white_interfaces": ["ifdummy3_black", "ifdummy2_white"],
                "red_interfaces": ["ifdummy1_red", "vlan_red"],
            },
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 0, "bat1": 0},
                    "hardif": {"halow1": 20},
                },
            },
        }
        # No black interfaces if applied exclude_interfaces
        # + white_interfaces + red_interfaces
        cls.yaml_content_6 = {
            "CBMA": {
                "exclude_interfaces": [],
                "white_interfaces": ["ifdummy3_black", "ifdummy2_white"],
                "red_interfaces": ["ifdummy1_red", "vlan_red", "ifdummy0_black"],
            },
            "BATMAN": {
                "routing_algo": "BATMAN_IV",
                "hop_penalty": {
                    "meshif": {"bat0": 0, "bat1": 0},
                    "hardif": {"halow1": 20},
                },
            },
        }

        cls.yaml_content_7 = """
            CBMA:
              exclude_interfaces:
                - vlan_black
              white_interfaces: None
              red_interfaces:
                "wlan1"
            BATMAN:
              routing_algo: BATMAN_IV
            """
        # Write YAML content to files
        # Not forcing yaml file content in same order as in dictionary in order
        # to ensure yaml file parsing work properly.
        with open("ms_config.yaml", "w", encoding="utf-8") as file:
            yaml.dump(cls.yaml_content_1, file)
        with open("ms_config2.yaml", "w", encoding="utf-8") as file:
            yaml.dump(cls.yaml_content_2, file)
        with open("invalid_yaml_file.yaml", "w", encoding="utf-8") as file:
            file.write(cls.yaml_content_3)
        with open("ms_config4.yaml", "w", encoding="utf-8") as file:
            yaml.dump(cls.yaml_content_4, file)
        with open("ms_config5.yaml", "w", encoding="utf-8") as file:
            yaml.dump(cls.yaml_content_5, file)
        with open("ms_config6.yaml", "w", encoding="utf-8") as file:
            yaml.dump(cls.yaml_content_6, file)
        with open("ms_config7.yaml", "w", encoding="utf-8") as file:
            file.write(cls.yaml_content_7)

        # Create fakecertificate file
        cls.fake_certificate = "just fake"
        cls.folder = "./MAC"
        if not os.path.exists(cls.folder):
            os.makedirs(cls.folder)
        file_path = os.path.join(cls.folder, "fake_mac.crt")
        with open(file_path, "w", encoding="utf-8") as file_4:
            file_4.write(cls.fake_certificate)

        # Mock the CommsController object and other common mocks
        cls.mock_comms_ctrl = MagicMock()
        cls.mock_lock = MagicMock()
        cls.mock_comms_ctrl.command.handle_command.return_value = ("OK", None, None)

        # Patch CBMAAdaptation.__has_certificate() to return True for
        # MACs in FAKE_CERT_EXISTS_FOR_MAC list
        cls.patcher_has_certificate = patch.object(
            CBMAAdaptation,
            "_CBMAAdaptation__has_certificate",
            side_effect=lambda cert_path, mac: mac in cls.FAKE_CERT_EXISTS_FOR_MAC,
        )
        # Patch CBMAController.add_interface() to return True for
        # interfaces listed in CBMA_FAKE_SUCCESS_FOR_INTERFACES
        cls.patcher_add_interface = patch.object(
            CBMAController,
            "add_interface",
            side_effect=lambda interface: interface
            in cls.CBMA_FAKE_SUCCESS_FOR_INTERFACES,
        )

        # Patch CBMAAdaptation.get_base_mtu_size() to return such a small MTU
        # size that it will work on Ubuntu with default Batman without jumbo
        # frame support
        cls.patcher_get_base_mtu_size = patch.object(
            CBMAAdaptation, "_CBMAAdaptation__get_base_mtu_size", return_value=1400
        )
        # Patch CommsController.command.handle_command() to retur OK
        # for e.g. radio bring up/down commands.
        cls.patcher_comms_ctrl = patch.object(
            cls.mock_comms_ctrl.command,
            "handle_command",
            return_value=("OK", None, None),
        )

        # Start patches
        cls.mock_has_certificate = cls.patcher_has_certificate.start()
        cls.mock_add_interface = cls.patcher_add_interface.start()
        cls.mock_get_base_mtu_size = cls.patcher_get_base_mtu_size.start()
        cls.mock_comms_ctrl_instance = cls.patcher_comms_ctrl.start()

    @classmethod
    def tearDownClass(cls):
        # Delete created yaml/crt files
        for file_name in [
            "ms_config.yaml",
            "ms_config2.yaml",
            "invalid_yaml_file.yaml",
            "ms_config4.yaml",
            "ms_config5.yaml",
            "ms_config6.yaml",
            "ms_config7.yaml",
            "./MAC/fake_mac.crt",
        ]:
            try:
                os.remove(file_name)
            except FileNotFoundError:
                print(f"File {file_name} not found, skipping deletion.")
            except Exception as e:
                print(f"Error deleting file {file_name}: {e}")

        # Delete folder
        try:
            os.rmdir(cls.folder)
        except Exception as e:
            print(f"Error deleting folder {cls.folder}: {e}")

        # Delete interfaces using settings from the list
        for settings in cls.INTERFACE_SETTINGS:
            try:
                ifname = settings.get("ifname")
                subprocess.run(["ip", "link", "delete", ifname], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error deleting interface {ifname}: {e}")
            except Exception as e:
                print(f"Unexpected error deleting interface {ifname}: {e}")

        # Stop patches
        cls.patcher_has_certificate.stop()
        cls.patcher_add_interface.stop()
        cls.patcher_get_base_mtu_size.stop()
        cls.patcher_comms_ctrl.stop()

        # Clean up logging resources
        cls.logger.removeHandler(cls.handler)
        cls.handler.close()

    def setUp(self):
        # Instantiates CBMAAdaptation with ms_config.yaml. Used for most of the tests.
        # Tip: comment following line to get traces from CBMA adaptation. Note that it
        # will break some of the tests that relies on logger outputs.
        self.logger = MagicMock()

        self.cbma_adaptation = CBMAAdaptation(
            self.mock_comms_ctrl, self.logger, self.mock_lock, "ms_config.yaml"
        )
        # Allow "dummy" interfaces during unit testing
        self.cbma_adaptation.ALLOWED_KIND_LIST.add("dummy")

    def tearDown(self):
        self.cbma_adaptation.stop_cbma()

    def test_setup_cbma(self):
        # Patch CBMAAdaptation.__get_mac_addr() to return ifdummy0_black
        # interface's MAC that will be given as a parameter to create MAC
        # for lower batman
        with patch.object(
            CBMAAdaptation,
            "_CBMAAdaptation__get_mac_addr",
            return_value="12:34:56:78:90:ab",
        ):
            result = self.cbma_adaptation.setup_cbma()
            self.assertTrue(result)

    def test_stop_cbma(self):
        result = self.cbma_adaptation.stop_cbma()
        self.assertTrue(result)

    def test_setup_and_stop_cbma2(self):
        # Tries to setup and stop CBMA with such config
        # that doesn't have VLAN definitions.
        self.cbma_adaptation2 = CBMAAdaptation(
            self.mock_comms_ctrl, self.logger, self.mock_lock, "ms_config2.yaml"
        )
        # Allow "dummy" interfaces during unit testing
        self.cbma_adaptation2.ALLOWED_KIND_LIST.add("dummy")
        with patch.object(
            CBMAAdaptation,
            "_CBMAAdaptation__get_mac_addr",
            return_value="12:34:56:78:90:ab",
        ):
            setup_result = self.cbma_adaptation2.setup_cbma()
            self.assertTrue(setup_result)
        stop_result = self.cbma_adaptation2.stop_cbma()
        self.assertTrue(stop_result)

    def test_reading_invalid_config_file(self):
        mock_logger = MagicMock()

        with patch.object(mock_logger, "getChild"):
            # Mock the logger object returned by getLogger
            mock_logger.getChild.return_value.error = MagicMock()

            self.cbma_adaptation3 = CBMAAdaptation(
                self.mock_comms_ctrl,
                mock_logger,
                self.mock_lock,
                "invalid_yaml_file.yaml",
            )
            call_args_list = mock_logger.getChild.return_value.error.mock_calls

            # Assert that the error method of the mocked logger.getChild()
            # was called with the expected message
            expected_message = "Error reading config file"
            self.assertTrue(
                any(
                    expected_message in call_args[1][0] for call_args in call_args_list
                ),
                f"Expected message '{expected_message}' not found in logger calls.",
            )
            self.assertIsNone(self.cbma_adaptation3._CBMAAdaptation__config)
            self.assertIsNone(self.cbma_adaptation3._CBMAAdaptation__cbma_config)
            self.assertIsNone(self.cbma_adaptation3._CBMAAdaptation__vlan_config)

    def test_creating_vlan_interface(self):
        # VLAN interfaces has been created already
        # when self.cbma_adaptation was instantiated.
        # Try to create VLAN interfaces again with same settings. Should fail.
        result = self.cbma_adaptation._CBMAAdaptation__create_vlan_interfaces()
        self.assertFalse(result)

    def test_deleting_vlan_interface(self):
        # VLAN interfaces has been created already
        # when CBMAAdaptation was instantiated for the first time.
        # Try to delete them twice. Second deletion should fail.
        result = self.cbma_adaptation._CBMAAdaptation__delete_vlan_interfaces()
        self.assertTrue(result)
        result = self.cbma_adaptation._CBMAAdaptation__delete_vlan_interfaces()
        self.assertFalse(result)

    def test_creating_and_deleting_bridge_interface(self):
        # First bridge creation should succeed fine without any warning prints
        result = self.cbma_adaptation._CBMAAdaptation__create_bridge("br-lan")
        self.assertTrue(result)
        self.cbma_adaptation.logger.warning.assert_not_called()
        self.cbma_adaptation.logger.warning.reset_mock()

        # Second trial should succeed but warning should be printed.
        result = self.cbma_adaptation._CBMAAdaptation__create_bridge("br-lan")
        self.assertTrue(result)
        self.cbma_adaptation.logger.warning.assert_called_once()
        self.cbma_adaptation.logger.warning.reset_mock()

        # First deletion should work fine without any issues
        result = self.cbma_adaptation._CBMAAdaptation__shutdown_and_delete_bridge(
            "br-lan"
        )
        self.assertTrue(result)
        self.cbma_adaptation.logger.warning.assert_not_called()
        self.cbma_adaptation.logger.warning.reset_mock()

        # Second deletion should succeed but warning should be printed
        result = self.cbma_adaptation._CBMAAdaptation__shutdown_and_delete_bridge(
            "br-lan"
        )
        self.assertTrue(result)
        self.cbma_adaptation.logger.warning.assert_called_once()

    def test_set_interface_mac(self):
        # Try to set MAC for br-lan that should not exist yet
        # but only after setting up CBMA
        self.cbma_adaptation._CBMAAdaptation__set_interface_mac(
            "br-lan", "22:34:44:55:66:77"
        )
        expected_message = "Error setting MAC address"
        self.assertTrue(
            any(
                expected_message in str(call)
                for call in self.cbma_adaptation.logger.error.call_args_list
            ),
            f"Expected message '{expected_message}' not found in logger.error calls.",
        )
        self.cbma_adaptation.logger.error.reset_mock()

        # Try to set MAC for vlan_black that should exist already
        self.cbma_adaptation._CBMAAdaptation__set_interface_mac(
            "vlan_black", "22:34:44:55:66:77"
        )
        self.cbma_adaptation.logger.error.assert_not_called()

    def test_add_interface_to_bridge(self):
        # Bridge not created by default => should fail
        self.cbma_adaptation._CBMAAdaptation__add_interface_to_bridge(
            "br-lan", "vlan_red"
        )

        expected_message = "Cannot add interface to bridge"
        self.assertTrue(
            any(
                expected_message in str(call)
                for call in self.cbma_adaptation.logger.debug.call_args_list
            ),
            f"Expected message '{expected_message}' not found in logger.error calls.",
        )
        # Create bridge
        self.cbma_adaptation._CBMAAdaptation__create_bridge("br-lan")
        # Try to add non-existent interface to bridge
        self.cbma_adaptation._CBMAAdaptation__add_interface_to_bridge(
            "br-lan", "foobar"
        )
        expected_message = "Cannot add interface"
        self.assertTrue(
            any(
                expected_message in str(call)
                for call in self.cbma_adaptation.logger.debug.call_args_list
            ),
            f"Expected message '{expected_message}' not found in logger.error calls.",
        )
        # Reset the mock to clear previous calls
        self.cbma_adaptation.logger.debug.reset_mock()
        self.cbma_adaptation.logger.error.reset_mock()

        # Try adding existing interface to bridge
        self.cbma_adaptation._CBMAAdaptation__add_interface_to_bridge(
            "br-lan", "vlan_red"
        )
        # Success means no debug traces in tested function
        self.cbma_adaptation.logger.debug.assert_not_called()
        self.cbma_adaptation.logger.error.assert_not_called()

    def test_set_interface_up(self):
        self.cbma_adaptation.logger.error.reset_mock()
        # vlan_red_ create in cbma_adaptation instantiation,
        # setting it up should pass
        self.cbma_adaptation._CBMAAdaptation__set_interface_up("vlan_red")
        self.cbma_adaptation.logger.error.assert_not_called()

        # Setting up not existing interface should fail
        self.cbma_adaptation._CBMAAdaptation__set_interface_up("vlan_blue")
        self.cbma_adaptation.logger.error.assert_called_once()

    def test_get_mac_addr(self):
        # Update interfaces list from where to get MAC
        self.cbma_adaptation._CBMAAdaptation__get_interfaces()
        # Get MAC
        result = self.cbma_adaptation._CBMAAdaptation__get_mac_addr("vlan_red")
        self.assertIsNotNone(result)
        result = self.cbma_adaptation._CBMAAdaptation__get_mac_addr("foobar")
        self.assertIsNone(result)

    def test_has_certificate(self):
        self.patcher_has_certificate.stop()
        result = self.cbma_adaptation._CBMAAdaptation__has_certificate(".", "no_mac")
        self.assertFalse(result)
        result = self.cbma_adaptation._CBMAAdaptation__has_certificate(".", "fake_mac")
        self.assertTrue(result)
        self.patcher_has_certificate.start()

    def test_wait_for_ap_no_mock(self):
        # Not expected to have AP interface to wait by default
        result = self.cbma_adaptation._CBMAAdaptation__wait_for_ap()
        self.assertFalse(result)

    @patch("subprocess.check_output")
    @patch("time.sleep")
    def test_wait_for_ap_with_mock(self, mock_sleep, mock_check_output):
        # Fake success
        fake_output = """
        Interface wlan1
                ifindex 6
                wdev 0x1
                addr e4:5f:01:bd:6d:cb
                ssid comms_sleeve#6dcb
                type AP
                wiphy 0
                channel 1 (2412 MHz), width: 20 MHz, center1: 2412 MHz
                txpower 31.00 dBm
        """
        mock_check_output.return_value = fake_output.encode()
        result = self.cbma_adaptation._CBMAAdaptation__wait_for_ap()
        mock_check_output.assert_called_once_with(["iw", "dev", "wlan1", "info"])
        self.assertTrue(result)
        mock_sleep.assert_not_called()
        mock_check_output.reset_mock()

        def side_effect(*args, **kwargs):
            fake_output = """
            Interface wlan1
                    ifindex 6
                    wdev 0x1
                    addr e4:5f:01:bd:6d:cb
                    type managed
                    wiphy 0
                    channel 1 (2412 MHz), width: 20 MHz, center1: 2412 MHz
                    txpower 31.00 dBm
            """
            return fake_output.encode()

        mock_check_output.side_effect = side_effect
        result = self.cbma_adaptation._CBMAAdaptation__wait_for_ap(1)
        self.assertFalse(result)
        mock_sleep.assert_called()
        self.cbma_adaptation.logger.warning.assert_called_with("__wait_for_ap timeout")

    def test_wait_for_and_shutdown_interface(self):
        # Should be up by default
        result = self.cbma_adaptation._CBMAAdaptation__wait_for_interface("vlan_red")
        self.assertTrue(result)
        if result:
            subprocess.run(["ip", "link", "set", "vlan_red", "down"], check=True)
        result = self.cbma_adaptation._CBMAAdaptation__wait_for_interface("vlan_red")
        self.assertFalse(result)
        self.cbma_adaptation.logger.warning.assert_called_with(
            "__wait_for_interface timeout for %s", "vlan_red"
        )

    def _validate_interfaces_config(self, cbma_adaptation):
        cbma_config = getattr(cbma_adaptation, "_CBMAAdaptation__cbma_config", None)
        white_interfaces = cbma_config.get("white_interfaces")
        red_interfaces = cbma_config.get("red_interfaces")
        exclude_interfaces = cbma_config.get("exclude_interfaces")
        return cbma_adaptation._CBMAAdaptation__validate_cbma_config(
            exclude_interfaces, white_interfaces, red_interfaces
        )

    def _test_invalid_config(self, cbma_adaptation, expected_error_msg):
        cbma_adaptation.logger.error.reset_mock()
        result = self._validate_interfaces_config(cbma_adaptation)
        self.assertFalse(result)
        cbma_adaptation.logger.error.assert_called_with(expected_error_msg)

    def test_validate_cbma_config_success(self):
        # Default cbma_adaptation uses valid ms_config.yaml
        result = self._validate_interfaces_config(self.cbma_adaptation)
        self.assertTrue(result)
        self.cbma_adaptation.logger.info.assert_called_with(
            "Black interfaces after validation: ", ["ifdummy0_black", "ifdummy3_black"]
        )

    @parameterized.expand([
        ("ms_config4.yaml", "No black interfaces left if applied exclude_interfaces!"),
        ("ms_config5.yaml", "No black interfaces left if applied white_interfaces!"),
        ("ms_config6.yaml", "No black interfaces left if applied red_interfaces!"),
        ("ms_config7.yaml", "Input params are not lists!"),
    ])
    def test_invalid_cbma_configs(self, config_file, expected_error_msg):
        # Stop default CBMAAdaptation to remove VLAN interfaces
        self.cbma_adaptation.stop_cbma()
        # Instantiate CBMAAdaptation with parametrized config file
        self.cbma_adaptation2 = CBMAAdaptation(
            self.mock_comms_ctrl, self.logger, self.mock_lock, config_file
        )
        # Reset mock as validation fails in isntantiation phase as
        # dummy test interfaces are not allowed in that phase yet.
        self.cbma_adaptation2.logger.error.reset_mock()
        self.cbma_adaptation2.ALLOWED_KIND_LIST.add("dummy")
        self._test_invalid_config(self.cbma_adaptation2, expected_error_msg)

    @parameterized.expand([
        ("No valid black interfaces!"),
    ])
    def test_cbma_configs_without_certificates(self, expected_error_msg):
        self.patcher_has_certificate.stop()
        self.cbma_adaptation.logger.error.reset_mock()
        # Allo dummy type of interfaces in testing phase
        self.cbma_adaptation.ALLOWED_KIND_LIST.add("dummy")
        self._test_invalid_config(self.cbma_adaptation, expected_error_msg)
        self.patcher_has_certificate.start()
