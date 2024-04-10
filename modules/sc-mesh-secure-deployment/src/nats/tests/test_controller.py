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
            print("\n " + str(controller.radio_amount))
            assert controller.radio_amount != -1

