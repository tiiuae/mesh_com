import unittest
from unittest.mock import MagicMock

from src.comms_service_discovery import CommsServiceMonitor
from tests.service_discovery_helper import (dns_service_register,
                                            dns_service_unregister,
                                            service_discovery_cb,
                                            get_kwargs_dict, SERVICE_NAME, SERVICE_TYPE)

class CommsServiceDiscoveryProperties(unittest.TestCase):

    def test_comms_service_discovery_initialization(self):
        try:
            logger = MagicMock()
            dns_service_register()
            comms_service_discovery = CommsServiceMonitor(
                service_name=SERVICE_NAME,
                service_type=SERVICE_TYPE,
                service_cb=service_discovery_cb,
                test=True,
                logger=logger
            )

            comms_service_discovery.run()

            kwargs_dict: dict = get_kwargs_dict()
            self.assertEqual(kwargs_dict['service_name'], f'{SERVICE_NAME}.{SERVICE_TYPE}')

        except Exception as e:
            self.fail(f"An error occurred: {e}")

        finally:
            dns_service_unregister()
