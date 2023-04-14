"""
Unittests for validations.py
"""
# pylint: disable=import-error, wrong-import-position, unused-import, \
# disable=unresolved-reference, undefined-variable, too-long
import unittest
from .context import comms_settings as comms

class TestSettings(unittest.TestCase):
    """
    Test cases for comms_settings.py
    """
    def test_handle_mesh_settings(self):
        """
        Test cases for handle_mesh_settings()
        """
        # settings json is valid
        settings = comms.CommsSettings()
        ret, info, mesh_status = settings.handle_mesh_settings("""{"api_version": 1,"ssid": "test_mesh", "key":"1234567890","ap_mac": "00:11:22:33:44:55","country": "fi","frequency": "5220","ip": "192.168.1.2","subnet":  "255.255.255.0","tx_power": "5","mode": "mesh"}""", "./tests", "test.conf")
        self.assertEqual(ret, "OK")

        # settings json is invalid
        settings = comms.CommsSettings()
        ret, info, mesh_status = settings.handle_mesh_settings("""{}""", "./tests", "test.conf")
        self.assertEqual(ret, "FAIL")

        # settings json is invalid
        settings = comms.CommsSettings()
        ret, info, mesh_status = settings.handle_mesh_settings("""{"api_version": 1,"ssid": "test;_mesh", "key":"1230","ap_mac": "00:11:22:33:44:55","country": "fi","frequency": "5220","ip": "2522.168.1.2","subnet":  "a.255.255.0","tx_power": "5","mode": "mesh"}""", "./tests", "test.conf")
        self.assertEqual(ret, "FAIL")

if __name__ == '__main__':
    unittest.main()
