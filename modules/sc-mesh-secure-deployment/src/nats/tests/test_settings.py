"""
Unittests for validations.py
"""
# pylint: disable=import-error, wrong-import-position, unused-import, \
# disable=unresolved-reference, undefined-variable, too-long
import unittest
from unittest.mock import patch, MagicMock
import json
import warnings
from src.comms_settings import CommsSettings
from src.comms_status import CommsStatus

class TestSettings(unittest.TestCase):
    """
    Test cases for comms_settings.py
    """

    @patch("src.validation.is_valid_interface")
    def test_handle_mesh_settings(self, mock_is_valid_interface):
        """
        Test cases for handle_mesh_settings()
        """

        with warnings.catch_warnings():
            # interface validation will be tested separately
            mock_is_valid_interface.return_value = True

            warnings.simplefilter("ignore", ResourceWarning)
            # settings json is valid
            logger = MagicMock()

            cs: [CommsStatus, ...] = [
                CommsStatus(logger, "0"),
                CommsStatus(logger, "1"),
                CommsStatus(logger, "2"),
            ]
            settings = CommsSettings(cs, logger)

            cmd_dict = {
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

            jsoned = json.dumps(cmd_dict)
            ret, mesh_status = settings.handle_mesh_settings(
                jsoned, "./tests", "mesh.conf"
            )
            self.assertEqual(ret, "OK", msg=f"ret: {ret}, mesh_status: {mesh_status}")

            # settings json is invalid
            ret, mesh_status = settings.handle_mesh_settings(
                """{}""", "./tests", "mesh.conf"
            )
            self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
            #
            # # # settings json is invalid

            ret, mesh_status = settings.handle_mesh_settings(
                """{"radio_index": "0", "api_version": 1,"ssid": "test;_mesh", "key":"1230","country": "fi","frequency": "5220","tx_power": "5","mode": "mesh","priority":"long_range","role":"gcs"}}""",
                "./tests",
                "test.conf",
            )
            self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")
