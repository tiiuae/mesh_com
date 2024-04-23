import unittest, warnings
from unittest.mock import patch, MagicMock
import os
from src.comms_controller import CommsController  # Import the CommsController class

class CommsControllerTests(unittest.TestCase):

    @patch('src.comms_controller.logging.getLogger')
    def test_radio_amount_returns_no_error(self, mock_get_logger):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            mock_logger = MagicMock()
            mock_get_logger.return_value = mock_logger
            controller = CommsController()  # Instantiate the CommsController class directly
            assert controller.radio_amount != -1

    @patch('src.comms_controller.logging.getLogger')
    @patch('src.comms_controller.logging.debug')
    def test_radio_amount_returns_zero(self, mock_logger_debug, mock_get_logger):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            with patch(
                    'src.comms_controller.CommsController.'
                    '_CommsController__check_radio_amount', return_value=0):
                mock_logger = MagicMock()
                mock_logger_debug.return_value = None
                mock_get_logger.return_value = mock_logger
                controller = CommsController()
                assert controller.radio_amount == 0

    @patch('src.comms_controller.logging.getLogger')
    @patch('src.comms_controller.subprocess.run')
    def test_check_radio_amount_exception(self, mock_logger, mock_run):
        mock_logger.return_value = MagicMock()

        # Mock the subprocess.run function to raise an exception
        mock_run.side_effect = Exception("Mocked subprocess error")

        # Call the CommsController constructor, which internally calls __check_radio_amount
        with self.assertRaises(Exception) as cm:
            instance = CommsController()

        # Assert that the exception occurred
        self.assertEqual(str(cm.exception), "Mocked subprocess error")

