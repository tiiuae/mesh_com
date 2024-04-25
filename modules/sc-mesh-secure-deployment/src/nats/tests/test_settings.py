"""
Unittests for validations.py
"""
# pylint: disable=import-error, wrong-import-position, unused-import, \
# disable=unresolved-reference, undefined-variable, too-long
import unittest
from unittest.mock import mock_open
import os
from copy import deepcopy
from unittest.mock import patch, MagicMock
import json
import warnings
from src.comms_settings import CommsSettings
from src.comms_status import CommsStatus

cmd_dict_org = {
    "api_version": 1,
    "role": "drone",  # sleeve, drone, gcs
    "radios": [
        {
            "radio_index": "0",
            "ssid": "test_mesh2",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "2412",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "tx_power": "15",
            "mptcp": "disable",
            "slaac": "usb0 wlp3s0",
            "mode": "mesh",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "wlp2s0",
        },
        {
            "radio_index": "1",
            "ssid": "test_mesh",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "5220",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "slaac": "usb0 wlp3s0",
            "tx_power": "15",
            "mptcp": "disable",
            "mode": "mesh",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "wlp3s0",  # this needs to be correct
        },
        {
            "radio_index": "2",
            "ssid": "test_mesh3",
            "key": "1234567890",
            "country": "US",  # all radios must have the same country
            "frequency": "5190",
            "frequency_mcc": "2412",  # multiradio not supporting
            "priority": "long_range",
            "tx_power": "30",
            "slaac": "usb0 wlp3s0",
            "mptcp": "disable",
            "mode": "halow",  # ap+mesh_scc, mesh, halow
            "mesh_vif": "halow1",
        },
    ],
}


class TestSettings(unittest.TestCase):
    """
    Test cases for comms_settings.py
    """

    @classmethod
    def tearDownClass(cls):
        current_path = os.getcwd()
        test_files = ["0_mesh.conf", "1_mesh.conf", "2_mesh.conf"]

        # Clean up the test files
        for file_name in test_files:
            file_path = os.path.join(current_path, "tests", file_name)
            if os.path.exists(file_path):
                os.remove(file_path)

    @classmethod
    def setUpClass(cls):
        logger = MagicMock()

        cls.cs: [CommsStatus, ...] = [
            CommsStatus(logger, "0"),
            CommsStatus(logger, "1"),
            CommsStatus(logger, "2"),
        ]
        cls.settings = CommsSettings(cls.cs, logger)

    @patch("src.validation.is_valid_interface")
    def test_handle_mesh_settings(self, mock_is_valid_interface):
        """
        Test cases for handle_mesh_settings()
        """

        with warnings.catch_warnings():
            # interface validation will be tested separately
            mock_is_valid_interface.return_value = True

            warnings.simplefilter("ignore", ResourceWarning)

            cmd_dict = deepcopy(cmd_dict_org)

            jsoned = json.dumps(cmd_dict)
            ret, mesh_status = self.settings.handle_mesh_settings(
                jsoned, "./tests", "mesh.conf"
            )
            self.assertEqual(ret, "OK", msg=f"ret: {ret}, mesh_status: {mesh_status}")

            # settings json is invalid
            ret, mesh_status = self.settings.handle_mesh_settings(
                """{}""", "./tests", "mesh.conf"
            )
            self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
            #
            # # # settings json is invalid

            ret, mesh_status = self.settings.handle_mesh_settings(
                """{"radio_index": "0", "api_version": 1,"ssid": "test;_mesh", "key":"1230","country": "fi","frequency": "5220","tx_power": "5","mode": "mesh","priority":"long_range","role":"gcs"}}""",
                "./tests",
                "test.conf",
            )
            self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")

            del cmd_dict


    def test_handle_mesh_settings_validate_parameters_ssid(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #ssid
        cmd_dict["radios"][0]["ssid"] = "a" * 33
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_key(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #key
        cmd_dict["radios"][0]["key"] = "a"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_mode(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #mode
        cmd_dict["radios"][0]["key"] = "1234567890"
        cmd_dict["radios"][0]["mode"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_frequency(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #frequency
        cmd_dict["radios"][0]["frequency"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

        cmd_dict = deepcopy(cmd_dict_org)
        cmd_dict["radios"][0]["frequency"] = "2444"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_frequency_mcc(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #frequency_mcc
        cmd_dict["radios"][0]["frequency_mcc"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

        cmd_dict = deepcopy(cmd_dict_org)
        cmd_dict["radios"][0]["frequency_mcc"] = "2400"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_cc(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #country
        cmd_dict["radios"][0]["country"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_tx_power(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #tx_power
        cmd_dict["radios"][0]["tx_power"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

        cmd_dict = deepcopy(cmd_dict_org)
        cmd_dict["radios"][0]["tx_power"] = "50"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")

    def test_handle_mesh_settings_validate_parameters_priority(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #priority
        cmd_dict["radios"][0]["priority"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_role(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #role
        cmd_dict["role"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_mptcp(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #mptcp
        cmd_dict["radios"][0]["mptcp"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_slaac(self):
        cmd_dict = deepcopy(cmd_dict_org)

        #slaac
        cmd_dict["radios"][0]["slaac"] = "test"
        jsoned = json.dumps(cmd_dict)
        ret, mesh_status = self.settings.handle_mesh_settings(
            jsoned, "./tests", "mesh.conf"
        )
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del cmd_dict

    def test_handle_mesh_settings_validate_parameters_channel_change(self):

        channel_change_dict = {"frequency": "2452", "radio_index": "1"}
        jsoned = json.dumps(channel_change_dict)
        ret, mesh_status, trigger = self.settings.handle_mesh_settings_channel_change(
            jsoned, "./tests", "mesh.conf"
        )
        #other settings are clean and empty
        self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
        del channel_change_dict

    def test_handle_mesh_settings_load_settings(self):
        # Prepare mock file content (simulate YAML content)
        mock_file_content = """
            ROLE=DRONE
            MSVERSION=nats
            id0_MODE=ap+mesh_mcc
            id0_KEY=1234567890
            id0_ESSID=gold
            id0_FREQ=5805
            id0_FREQ_MCC=2412
            id0_TXPOWER=30
            id0_COUNTRY=US
            id0_PRIORITY=high_throughput
            id0_MESH_VIF=wlp1s0
            id0_MPTCP=disable
            id0_SLAAC="wlan1 usb0"
            """

        with patch("os.path.exists", return_value=True):
            # Patch the open function to return the mock file content
            with patch("builtins.open", mock_open(read_data=mock_file_content)) as mock_file:
                ret, info = self.settings._CommsSettings__load_settings()

                # Verify that open was called with the expected arguments for each file
                mock_file.assert_any_call("/opt/0_mesh.conf", "r", encoding="utf-8")
                mock_file.assert_any_call("/opt/1_mesh.conf", "r", encoding="utf-8")
                mock_file.assert_any_call("/opt/2_mesh.conf", "r", encoding="utf-8")

                # Ensure that the method returns the expected values
                self.assertEqual(ret, "OK", msg=f"ret: {ret}, info: {info}")

        with patch("os.path.exists", return_value=False):
            # Patch the open function to return the mock file content
            with patch("builtins.open", mock_open(read_data=mock_file_content)) as mock_file:
                ret, info = self.settings._CommsSettings__load_settings()
                mock_file.assert_called_once_with("/opt/mesh_default.conf", "r", encoding="utf-8")
                self.assertEqual(ret, "OK", msg=f"ret: {ret}, info: {info}")
