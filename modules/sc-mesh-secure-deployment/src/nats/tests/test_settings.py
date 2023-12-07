"""
Unittests for validations.py
"""
# pylint: disable=import-error, wrong-import-position, unused-import, \
# disable=unresolved-reference, undefined-variable, too-long
import unittest
from unittest.mock import patch
import json
import logging
import warnings
from .context import comms_settings as comms
from .context import comms_status as comms_status

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
            logger = logging.getLogger("test")
            logger.setLevel(logging.CRITICAL)

            cs: [comms_status, ...] = [
                comms_status.CommsStatus(logger, "0"),
                comms_status.CommsStatus(logger, "1"),
                comms_status.CommsStatus(logger, "2"),
            ]
            settings = comms.CommsSettings(cs, logger)

            cmd_dict = {
                "api_version": 1,
                "role": "drone",  # sleeve, drone, gcs
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
                        "priority": "long_range",
                        "ip": "10.20.15.3",
                        "subnet": "255.255.255.0",
                        "tx_power": "15",
                        "mptcp": "disable",
                        "slaac": "usb0 wlp3s0",
                        "mode": "mesh",  # ap+mesh_scc, mesh, halow
                        "mesh_vif": "wlp2s0",
                        "batman_iface": "bat0",
                        "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
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
                        "priority": "long_range",
                        "ip": "10.20.15.3",
                        "slaac": "usb0 wlp3s0",
                        "subnet": "255.255.255.0",
                        "tx_power": "15",
                        "mptcp": "disable",
                        "mode": "mesh",  # ap+mesh_scc, mesh, halow
                        "mesh_vif": "wlp3s0",  # this needs to be correct
                        "batman_iface": "bat0",
                        "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
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
                        "priority": "long_range",
                        "ip": "10.20.15.3",
                        "subnet": "255.255.255.0",
                        "tx_power": "30",
                        "slaac": "usb0 wlp3s0",
                        "mptcp": "disable",
                        "mode": "halow",  # ap+mesh_scc, mesh, halow
                        "mesh_vif": "halow1",
                        "batman_iface": "bat0",
                        "bridge": "br-lan bat0 eth1 lan1 eth0 usb0",
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
                """{"radio_index": "0", "api_version": 1,"ssid": "test;_mesh", "key":"1230","ap_mac": "00:11:22:33:44:55","country": "fi","frequency": "5220","ip": "2522.168.1.2","subnet":  "a.255.255.0","tx_power": "5","mode": "mesh","priority":"long_range","role":"gcs"}}""",
                "./tests",
                "test.conf",
            )
            self.assertEqual(ret, "FAIL", msg=f"ret: {ret}, mesh_status: {mesh_status}")


if __name__ == "__main__":
    unittest.main()
