import unittest
from unittest.mock import MagicMock, patch

from src.comms_service_discovery import CommsServiceMonitor
from tests.service_discovery_helper import (dns_service_register,
                                            dns_service_unregister,
                                            service_discovery_cb,
                                            get_kwargs_dict, SERVICE_NAME, SERVICE_TYPE)

class TestCommsServiceDiscovery(unittest.TestCase):

    @patch('src.comms_controller.logging.getLogger')
    @patch('src.comms_controller.logging.debug')
    def test_comms_service_discovery_initialization(self, mock_logger_debug, mock_get_logger):
        try:
            mock_logger = MagicMock()
            mock_logger_debug.return_value = None
            mock_get_logger.return_value = mock_logger
            dns_service_register()
            comms_service_discovery = CommsServiceMonitor(
                service_name=SERVICE_NAME,
                service_type=SERVICE_TYPE,
                service_cb=service_discovery_cb,
                test=True,
            )

            comms_service_discovery.run()

            kwargs_dict: dict = get_kwargs_dict()
            self.assertEqual(kwargs_dict['service_name'], f'{SERVICE_NAME}.{SERVICE_TYPE}')

        except Exception as e:
            self.fail(f"An error occurred: {e}")

        comms_service_discovery.close()
        dns_service_unregister()

    def test_comms_service_discovery_initialization_with_logger(self):
        try:
            mock_logger = MagicMock()
            dns_service_register()
            comms_service_discovery = CommsServiceMonitor(
                service_name=SERVICE_NAME,
                service_type=SERVICE_TYPE,
                service_cb=service_discovery_cb,
                test=True,
                logger=mock_logger
            )

            comms_service_discovery.run()

            kwargs_dict: dict = get_kwargs_dict()
            self.assertEqual(kwargs_dict['service_name'], f'{SERVICE_NAME}.{SERVICE_TYPE}')

        except Exception as e:
            self.fail(f"An error occurred: {e}")

        comms_service_discovery.close()
        dns_service_unregister()

    def test_comms_service_discovery_initialization_with_interface(self):
        try:
            mock_logger = MagicMock()
            dns_service_register()
            comms_service_discovery = CommsServiceMonitor(
                service_name=SERVICE_NAME,
                service_type=SERVICE_TYPE,
                service_cb=service_discovery_cb,
                test=True,
                logger=mock_logger,
                interface='lo'
            )

            comms_service_discovery.run()

            kwargs_dict: dict = get_kwargs_dict()
            self.assertEqual(kwargs_dict['service_name'], f'{SERVICE_NAME}.{SERVICE_TYPE}')
        except Exception as e:
            self.fail(f"An error occurred: {e}")

        comms_service_discovery.close()
        dns_service_unregister()
