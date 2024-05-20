import unittest
from unittest.mock import patch, MagicMock
import mdm_agent


class TestMdmAgent(unittest.TestCase):
    """
    Test the MdmAgent class
    """

    @patch('src.comms_controller.CommsController')
    @patch('mdm_agent.os.makedirs')  # Mock the os.makedirs call
    @patch('mdm_agent.comms_service_discovery.CommsServiceMonitor')
    @patch('mdm_agent.comms_if_monitor.CommsInterfaceMonitor')
    @patch('mdm_agent.threading.Thread')
    def setUp(self, mock_makedirs, mock_comms_controller, mock_interface_monitor, mock_service_monitor, mock_thread):
        """
        Setup the test
        """
        mock_makedirs.return_value = None
        # mocked interfaces are tested separately
        self.mock_comms_controller = mock_comms_controller
        self.mock_interface_monitor = mock_interface_monitor.return_value
        self.mock_service_monitor = mock_service_monitor.return_value
        self.mock_thread = mock_thread
        self.mock_thread.return_value = MagicMock()
        self.mdm_agent = mdm_agent.MdmAgent(self.mock_comms_controller)

    def test_mdm_server_address_cb_updates_url_and_status(self):
        """
        Test the mdm_server_address_cb method
        """
        self.mdm_agent.mdm_server_address_cb('new_address', True)
        self.assertEqual(self.mdm_agent._MdmAgent__url, 'new_address')
        self.assertTrue(self.mdm_agent.mdm_service_available)

    def test_interface_monitor_cb_clears_interfaces(self):
        """
        Test the interface_monitor_cb method
        """
        self.mdm_agent.service_monitor.running = False
        self.mdm_agent.interface_monitor_cb([])
        self.assertEqual(self.mdm_agent._MdmAgent__interfaces, [])

    def test_interface_monitor_cb_adds_interfaces(self):
        """
        Test the interface_monitor_cb method
        """
        test_data = {"interface_name": "eth0", "operstate": "UP",
                     "mac_address": "00:00:00:00:00:00"}
        self.mdm_agent.interface_monitor_cb([test_data])
        for i in self.mdm_agent._MdmAgent__interfaces:
            self.assertEqual(i.interface_name, test_data['interface_name'])
            self.assertEqual(i.operstat, test_data['operstate'])
            self.assertEqual(i.mac_address, test_data['mac_address'])

    @patch('mdm_agent.requests.get')
    def test_http_get_device_config_makes_request(self, mock_get):
        """
        Test the __http_get_device_config method
        """
        self.mdm_agent._MdmAgent__http_get_device_config(mdm_agent.ConfigType.MESH_CONFIG)
        mock_get.assert_called()

    @patch('mdm_agent.requests.post')
    @patch('mdm_agent.glob.glob')
    def test_upload_certificate_bundle_makes_request(self, mock_post, mock_glob):
        """
        Test the upload_certificate_bundle method
        """
        mock_glob.return_value = ['/opt/at_birth.tar.bz2']
        self.mdm_agent.upload_certificate_bundle()
        mock_post.assert_called()

    @patch('mdm_agent.requests.get')
    def test_download_certificate_bundle_makes_request(self, mock_get):
        """
        Test the download_certificate_bundle method
        """
        self.mdm_agent.download_certificate_bundle()
        mock_get.assert_called()

    def test_validate_response_returns_fail_for_non_200_status(self):
        """
        Test the __validate_response method
        """
        mock_response = MagicMock()
        mock_response.status_code = 404
        result = self.mdm_agent._MdmAgent__validate_response(mock_response,
                                                             mdm_agent.ConfigType.MESH_CONFIG)
        self.assertEqual(result, 'FAIL')

    def test_validate_response_returns_ok_for_200_status_and_empty_payload(self):
        """
        Test the __validate_response method
        """
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.text = '{"payload": {"radios": []}}'

        result = self.mdm_agent._MdmAgent__validate_response(mock_response,
                                                             mdm_agent.ConfigType.MESH_CONFIG)
        self.assertEqual(result, 'FAIL')
