import unittest
from unittest.mock import patch, MagicMock
from src.comms_command import Command
from src.comms_settings import CommsSettings


class CommandTests(unittest.TestCase):
    """
    Test the Command class
    """
    def setUp(self):
        """
        Setup the test
        """
        comms_status = MagicMock()
        logger = MagicMock()
        nats_mode = False
        self.command = Command(comms_status, logger, nats_mode)
        self.assertEqual(self.command.comms_status, comms_status)
        self.assertEqual(self.command.logger, logger)
        self.assertEqual(self.command.nats_mode, nats_mode)
        self.settings = CommsSettings([comms_status], logger)

    @patch('src.comms_command.subprocess.run')
    @patch('src.comms_command.os.replace')
    @patch('src.comms_command.open')
    def test_handle_command_for_settings(self, mock_subprocess, mock_replace, mock_open):
        """
        Test the handle_command method
        """
        index = "0"

        cc = MagicMock()
        mock_subprocess.return_value = MagicMock()
        mock_replace.return_value = MagicMock()
        mock_open.return_value = MagicMock()
        msg = '{"api_version": 1, "cmd": "revoke"}'  # lower case revoke not supported
        ret, _, _ = self.command.handle_command(msg, cc)
        self.assertEqual(ret, "FAIL")

        # msg = '{"api_version": 1, "cmd": "REBOOT"}'
        # ret, _, _ = self.command.handle_command(msg, cc)
        # self.assertEqual(ret, "FAIL") # command not supported
        #
        # msg = '{"api_version": 1, "cmd": "APPLY", "radio_index":' + f"{index}" + '}'
        # ret, _, _ = self.command.handle_command(msg, cc)
        # self.assertEqual(ret, "FAIL")
        #
        # msg = ('{ "api_version": 1, "role": "sleeve", "radios": [ {"radio_index": "0", '
        #        '"ssid": "test_mesh2", "key": "1234567890", "country": "US", "frequency": "2412", '
        #        '"frequency_mcc": "2412", "mptcp": "disable", "priority": "high_throughput", '
        #        '"tx_power": "15", "mode": "mesh", "mesh_vif": "wlp2s0", "slaac": ""}]}')
        # ret, _ = self.settings.handle_mesh_settings(msg, "/tmp/")
        # self.assertEqual(ret, "OK")

        # TODO:  APPLY, REVOKE, REBOOT, GET_CONFIG, GET_IDENTITY,
        #  DOWN, UP, DEBUG, LOGS, ENABLE_VISUALISATION, DISABLE_VISUALISATION
