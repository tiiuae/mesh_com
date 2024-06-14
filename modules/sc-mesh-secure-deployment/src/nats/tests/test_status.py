import unittest
from unittest.mock import patch
import tempfile
import warnings
import os
from src.constants import Constants
from src.comms_status import CommsStatus
from unittest.mock import MagicMock


class TestCommsStatus(unittest.TestCase):
    def test_comms_status_initialization(self):
        """
        Testing CommsStatus initialization and there is no guarantee that
        the mesh configuration file exists or radios are on.

        Depends on which environment the test is run, the mesh configuration file may not exist.
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            logger = MagicMock()

            # mock the radio status and ap status

            with patch("threading.Lock") as mock_lock:
                comms_status = CommsStatus(logger=logger, index=0)
                if os.path.isfile(f"{Constants.ROOT_PATH.value}/0_mesh.conf"):
                    assert comms_status.mesh_cfg_status == "MESH_CONFIGURATION_APPLIED"
                    assert comms_status.security_status == "SECURITY_NON_PROVISIONED"
                    self.assertFalse(comms_status.is_visualisation_active)
                    self.assertTrue(comms_status.is_mission_cfg)
                    self.assertFalse(comms_status.is_ap_radio_on)
                    self.assertFalse(comms_status.is_mesh_radio_on)
                    assert comms_status.ap_interface_name == ""     # not stored by comms_status
                    assert comms_status.mesh_interface_name == ""   # not stored by comms_status
                else:
                    assert comms_status.mesh_cfg_status == "MESH_CONFIGURATION_NOT_STORED"
                    assert comms_status.security_status == "SECURITY_NON_PROVISIONED"
                    self.assertFalse(comms_status.is_visualisation_active)
                    self.assertFalse(comms_status.is_mission_cfg)
                    self.assertFalse(comms_status.is_ap_radio_on)
                    self.assertFalse(comms_status.is_mesh_radio_on)
                    assert comms_status.ap_interface_name == ""     # not stored by comms_status
                    assert comms_status.mesh_interface_name == ""   # not stored by comms_status

                self.assertFalse(comms_status.is_visualisation_active)

    def test_wpa_status_reset(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)

            logger = MagicMock()
            comms_status = CommsStatus(logger=logger, index=0)
            wpa_status = comms_status.WpaStatus()
            wpa_status.reset()
            assert wpa_status.wpa_state == "INTERFACE_DISABLED"

    def test_hostapd_status_reset(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)

            logger = MagicMock()
            comms_status = CommsStatus(logger=logger, index=0)
            hostapd_status = comms_status.HostapdStatus()
            hostapd_status.reset()
            assert hostapd_status.state == "DISABLED"

    def test_comms_status_properties(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)

            logger = MagicMock()
            comms_status = CommsStatus(logger=logger, index=0)
            assert comms_status.security_status is not None
            assert comms_status.mesh_status is not None
            assert comms_status.mesh_cfg_status is not None
            assert comms_status.is_mission_cfg is not None
            assert comms_status.is_mesh_radio_on is not None
            assert comms_status.is_visualisation_active is not None
            assert comms_status.is_ap_radio_on is not None
            assert comms_status.ap_interface_name is not None
            assert comms_status.mesh_interface_name is not None

    def test_comms_status_mission_config(self):

        logger = MagicMock()
        comms_status = CommsStatus(logger=logger, index=0)

        # Define mock file content
        config_content = b"test_config_content"
        hash_content = b"1a709d4bfad22352b1136fff4061c3c29313e02d06113c18da16c94551a9d62c"
        wrong_hash_content = b"1a1a1a1a"

        # Define file paths
        config_file_path = "/opt/0_mesh.conf"
        hash_file_path = "/opt/0_mesh.conf_hash"

        # Create temporary files with the specified paths
        with open(config_file_path, "wb") as config_file:
            config_file.write(config_content)

        with open(hash_file_path, "wb") as hash_file:
            hash_file.write(hash_content)

        # Call the method under test with correct hash
        comms_status._CommsStatus__get_mission_cfg_status()
        self.assertTrue(comms_status.is_mission_cfg)  # Mission config is expected

        with open(hash_file_path, "wb") as hash_file:
            hash_file.write(wrong_hash_content)

        # Call the method under test with wrong hash
        comms_status._CommsStatus__get_mission_cfg_status()
        self.assertFalse(comms_status.is_mission_cfg)  # Mission config is not expected

        os.unlink(hash_file_path)
        # without hash file causing FileNotFoundError
        comms_status._CommsStatus__get_mission_cfg_status()
        self.assertFalse(comms_status.is_mission_cfg)  # Mission config is not expected

        os.unlink(config_file_path)
        # without files causing FileNotFoundError
        comms_status._CommsStatus__get_mission_cfg_status()
        self.assertFalse(comms_status.is_mission_cfg)  # Mission config is not expected

    @patch('subprocess.Popen')
    def test_get_wpa_cli_status_with_valid_output(self, mock_popen):
        # Mocking the Popen call and setting the return value
        mock_popen.return_value.communicate.return_value = (
            b'Selected interface \'wlan0\'\nbssid=00:00:00:00:00:00\nfreq=2412\nssid=test\nid=0\nmode=station\npairwise_cipher=CCMP\n'
            b'group_cipher=CCMP\nkey_mgmt=WPA2-PSK\nwpa_state=COMPLETED\naddress=00:00:00:00:00:00\nuuid=00000000-0000-0000-0000-000000000000\n',
            None
        )

        comms_status = CommsStatus(logger=MagicMock(), index=0)
        comms_status._CommsStatus__get_wpa_cli_status()

        self.assertEqual(comms_status._CommsStatus__wpa_status.interface, 'wlan0')
        self.assertEqual(comms_status._CommsStatus__wpa_status.bssid, '00:00:00:00:00:00')
        self.assertEqual(comms_status._CommsStatus__wpa_status.freq, '2412')
        self.assertEqual(comms_status._CommsStatus__wpa_status.ssid, 'test')
        self.assertEqual(comms_status._CommsStatus__wpa_status.id, '0')
        self.assertEqual(comms_status._CommsStatus__wpa_status.mode, 'station')
        self.assertEqual(comms_status._CommsStatus__wpa_status.pairwise_cipher, 'CCMP')
        self.assertEqual(comms_status._CommsStatus__wpa_status.group_cipher, 'CCMP')
        self.assertEqual(comms_status._CommsStatus__wpa_status.key_mgmt, 'WPA2-PSK')
        self.assertEqual(comms_status._CommsStatus__wpa_status.wpa_state, 'COMPLETED')
        self.assertEqual(comms_status._CommsStatus__wpa_status.address, '00:00:00:00:00:00')
        self.assertEqual(comms_status._CommsStatus__wpa_status.uuid,
                         '00000000-0000-0000-0000-000000000000')

    @patch('subprocess.Popen')
    def test_get_wpa_cli_status_with_error(self, mock_popen):
        # Mocking the Popen call and setting the return value
        mock_popen.return_value.communicate.return_value = (
            None,
            b'Error from wpa_cli'
        )

        comms_status = CommsStatus(logger=MagicMock(), index=0)
        with self.assertRaises(RuntimeError):
            comms_status._CommsStatus__get_wpa_cli_status()

    @patch('subprocess.Popen')
    def test_hostapd_cli_status_with_valid_output(self, mock_popen):
        mock_popen.return_value.communicate.return_value = (
            b'Selected interface \'wlan0\'\nstate=ENABLED\nphy=phy0\nfreq=2412\nchannel=6\nbeacon_int=100\nssid[0]=test\n',
            None
        )

        comms_status = CommsStatus(logger=MagicMock(), index=0)
        comms_status._CommsStatus__get_hostapd_cli_status()

        self.assertEqual(comms_status._CommsStatus__hostapd_status.interface, 'wlan0')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.state, 'ENABLED')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.phy, 'phy0')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.freq, '2412')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.channel, '6')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.beacon_int, '100')
        self.assertEqual(comms_status._CommsStatus__hostapd_status.ssid, 'test')

    @patch('subprocess.Popen')
    def test_hostapd_cli_status_with_error(self, mock_popen):
        mock_popen.return_value.communicate.return_value = (
            None,
            b'Error from hostapd_cli'
        )

        comms_status = CommsStatus(logger=MagicMock(), index=0)
        with self.assertRaises(RuntimeError):
            comms_status._CommsStatus__get_hostapd_cli_status()

