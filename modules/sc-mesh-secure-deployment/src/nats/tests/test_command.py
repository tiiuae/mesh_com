import unittest
from unittest.mock import patch, MagicMock
from src.comms_command import Command

class CommandTests(unittest.TestCase):

    @patch('src.comms_controller.subprocess.run')
    def test_handle_command_calls_correct_sub_function(self, mock_subprocess):
        comms_status = MagicMock()
        logger = MagicMock()
        nats_mode = False
        command = Command(comms_status, logger, nats_mode)
        cc = MagicMock()
        msg = '{"api_version": 1, "cmd": "revoke"}'
        ret, info, data = command.handle_command(msg, cc)
        self.assertEqual(ret, "FAIL") # command not supported