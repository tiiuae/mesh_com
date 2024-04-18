import unittest
import logging
from unittest.mock import patch
import warnings
from src.comms_status import CommsStatus
from unittest.mock import MagicMock

class TestCommsStatus(unittest.TestCase):
    def test_comms_status_initialization(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            logger = MagicMock()
            with patch("threading.Lock") as mock_lock:
                comms_status = CommsStatus(logger=logger, index=0)
                assert comms_status.mesh_cfg_status == "MESH_CONFIGURATION_NOT_STORED"
                assert comms_status.security_status == "SECURITY_NON_PROVISIONED"
                self.assertFalse(comms_status.is_mission_cfg)
                self.assertFalse(comms_status.is_mesh_radio_on)
                self.assertFalse(comms_status.is_visualisation_active)
                self.assertFalse(comms_status.is_ap_radio_on)
                assert comms_status.ap_interface_name == ""
                assert comms_status.mesh_interface_name == ""

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