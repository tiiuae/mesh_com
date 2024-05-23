import argparse
import asyncio
import base64
import json
import os
import unittest
from unittest.mock import patch, MagicMock, AsyncMock, mock_open, call
import ssl
import signal
import requests
import yaml

import mdm_agent
from cryptography.hazmat.primitives.asymmetric import rsa, ec, ed25519
from mdm_agent import ConfigType, StatusType, main_mdm
from src.constants import Constants

#pylint: disable=protected-access
class TestMdmAgent(unittest.TestCase):
    """
    Test the MdmAgent class
    """

    # HOX!! setup self.agent is not used in all tests
    @patch('mdm_agent.os.makedirs')  # Mock the os.makedirs call
    @patch('mdm_agent.comms_service_discovery.CommsServiceMonitor')
    @patch('mdm_agent.comms_if_monitor.CommsInterfaceMonitor')
    @patch('mdm_agent.threading.Thread')
    def setUp(self, mock_thread, mock_interface_monitor, mock_service_monitor, mock_makedirs):
        """
        Setup the test
        """
        mock_makedirs.return_value = None
        # Mocking return values of the service and interface monitors
        self.mock_interface_monitor = mock_interface_monitor.return_value
        self.mock_service_monitor = mock_service_monitor.return_value

        # Ensuring the thread mock returns a MagicMock instance
        self.mock_thread = mock_thread
        self.mock_thread.return_value = MagicMock()

        # Assign the monitor_interfaces method to return a mock
        self.mock_interface_monitor.monitor_interfaces = MagicMock()

        self.agent = mdm_agent.MdmAgent(MagicMock())
        self.agent.logger = MagicMock()
        self.agent.upload_certificate_bundle = AsyncMock()
        self.agent.download_certificate_bundle = AsyncMock()
        self.agent._MdmAgent__action_certificates = MagicMock()
        self.agent.cbma_ctrl = MagicMock()
        self.agent._MdmAgent__loop_run_executor = AsyncMock()

        # Set initial statuses
        self.agent._Mdm_agent__status = {
            StatusType.UPLOAD_CERTIFICATES.value: "FAIL",
            StatusType.DOWNLOAD_CERTIFICATES.value: "FAIL",
            StatusType.DOWNLOAD_MESH_CONFIG.value: "FAIL",
            StatusType.DOWNLOAD_FEATURES.value: "FAIL",
        }
        self.agent.mdm_service_available = True
        self.agent._MdmAgent__cbma_set_up = False
        self.agent._MdmAgent__interval = 10
        self.agent._MdmAgent__debug_config_interval = 10
        self.agent._MdmAgent__mesh_conf_request_processed = False

    def tearDown(self):
        """
        Tear down the test
        """
        self.agent.running = False
        self.agent.executor.shutdown(wait=False)

    def test_mdm_server_address_cb_updates_url_and_status(self):
        """
        Test the mdm_server_address_cb method
        """
        self.agent.mdm_server_address_cb('new_address', True)
        self.assertEqual(self.agent._MdmAgent__url, 'new_address')
        self.assertTrue(self.agent.mdm_service_available)

    def test_interface_monitor_cb_clears_interfaces(self):
        """
        Test the interface_monitor_cb method
        """
        self.agent.service_monitor.running = False
        self.agent.interface_monitor_cb([])
        self.assertEqual(self.agent._MdmAgent__interfaces, [])

    def test_interface_monitor_cb_adds_interfaces(self):
        """
        Test the interface_monitor_cb method
        """
        test_data = {"interface_name": "eth0", "operstate": "UP",
                     "mac_address": "00:00:00:00:00:00"}
        self.agent.interface_monitor_cb([test_data])
        for i in self.agent._MdmAgent__interfaces:
            self.assertEqual(i.interface_name, test_data['interface_name'])
            self.assertEqual(i.operstat, test_data['operstate'])
            self.assertEqual(i.mac_address, test_data['mac_address'])

    @patch('mdm_agent.requests.get')
    def test_http_get_device_config_makes_request(self, mock_get):
        """
        Test the __http_get_device_config method
        """
        self.agent._MdmAgent__http_get_device_config(mdm_agent.ConfigType.MESH_CONFIG)
        mock_get.assert_called()

    @patch('mdm_agent.requests.post')
    @patch('mdm_agent.glob.glob')
    def test_upload_certificate_bundle_makes_request(self, mock_glob, mock_post):
        """
        Test the upload_certificate_bundle method
        """
        agent = mdm_agent.MdmAgent(MagicMock())
        agent.logger = MagicMock()

        # Mock the return value of glob.glob
        mock_glob.return_value = ['/opt/at_birth.tar.bz2']

        # pylint: disable=protected-access
        with open('/opt/at_birth.tar.bz2', 'wb') as f:
            f.write(b'test_data')

        # Call the method
        _ = agent.upload_certificate_bundle()

        # Check that glob was called with the correct path
        mock_glob.assert_called_with("/opt/at_birth.tar.bz2*")

        # Check that requests.post was called
        mock_post.assert_called()

        # Prepare the expected base64 data
        expected_base64_data = base64.b64encode(b'test_data').decode('utf-8')

        # Check the arguments passed to requests.post
        mock_post.assert_called_with(
            f"https://defaultmdm.local:5000/{mdm_agent.Constants.PUT_DEVICE_CERTIFICATES.value}/{mdm_agent.ConfigType.BIRTH_CERTIFICATE.value}",
            json={
                "device_id": "default",
                "payload": {'at_birth.tar.bz2': expected_base64_data},
                "format": "text",
            },
            cert=(None, None),
            verify=None,
            timeout=20
        )
        # remove the file
        os.remove('/opt/at_birth.tar.bz2')

    @patch('mdm_agent.requests.get')
    def test_download_certificate_bundle_makes_request(self, mock_get):
        """
        Test the download_certificate_bundle method
        """
        agent = mdm_agent.MdmAgent(MagicMock())

        agent._MdmAgent__url = 'defaultmdm.local:5000'
        agent._MdmAgent__https_url = self.agent._MdmAgent__url
        agent.download_certificate_bundle()
        mock_get.assert_called()

    @patch('builtins.open', new_callable=unittest.mock.mock_open)
    def test_action_feature_yaml_valid_response(self, mock_open):
        """
        Test the __action_feature_yaml method
        """

        self.agent._MdmAgent__previous_config_features = None

        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {
                "features": {
                    "feature1": True,
                    "feature2": False
                }
            }
        })

        self.agent._MdmAgent__config_store.read = MagicMock(return_value=response.text)
        self.agent._MdmAgent__config_store.store = MagicMock()

        status = self.agent._MdmAgent__action_feature_yaml(response)

        self.assertEqual(status, "OK")
        mock_open.assert_called_once_with(Constants.YAML_FILE.value, "w", encoding="utf-8")
        file_handle = mock_open()
        file_handle.write.assert_called_once_with(yaml.dump({
            "feature1": True,
            "feature2": False
        }, default_flow_style=False))
        self.agent._MdmAgent__config_store.store.assert_called_once_with(ConfigType.FEATURES.value,
                                                                         response.text.strip())

    @patch('mdm_agent.MdmAgent._MdmAgent__config_store', create=True)
    def test_action_feature_yaml_no_changes(self, mock_config_store):
        """
        Test the __action_feature_yaml method
        """
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {
                "features": {
                    "feature1": True,
                    "feature2": False
                }
            }
        })
        self.agent._MdmAgent__previous_config_features = response.text
        mock_config_store.read.return_value = response.text

        status = self.agent._MdmAgent__action_feature_yaml(response)

        self.assertEqual(status, "OK")
        # Check for both expected debug calls
        expected_calls = [
            call("config: {'payload': {'features': {'feature1': True, 'feature2': False}}} previous: {'payload': {'features': {'feature1': True, 'feature2': False}}}"),
            call('No changes in features config, not updating.')
        ]
        self.agent.logger.debug.assert_has_calls(expected_calls, any_order=False)

    def test_action_feature_yaml_key_error(self):
        """
        Test the __action_feature_yaml method
        """
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {}
        })

        status = self.agent._MdmAgent__action_feature_yaml(response)

        self.assertEqual(status, "FAIL")
        self.agent.logger.error.assert_called_once_with("KeyError features field in config")

    def test_action_feature_yaml_no_features_field(self):
        """
        Test the __action_feature_yaml method
        """
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({})

        status = self.agent._MdmAgent__action_feature_yaml(response)

        self.assertEqual(status, "FAIL")
        self.agent.logger.error.assert_called_once_with("KeyError features field in config")

    def test_action_radio_configuration_valid_response(self):
        """
        Test the __action_radio_configuration method
        """
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {
                "radios": [
                    {  # foo config.. Nothing to do with the agent
                        "radio_index": 0,
                    }
                ],
            },
            "version": "1",
        })

        self.agent._MdmAgent__previous_config_mesh = None

        self.agent._MdmAgent__comms_ctrl.settings.handle_mesh_settings = MagicMock()
        self.agent._MdmAgent__comms_ctrl.settings.handle_mesh_settings.return_value = "OK", "damn good"

        self.agent._MdmAgent__comms_ctrl.command.handle_command = MagicMock()
        self.agent._MdmAgent__comms_ctrl.command.handle_command.return_value = "OK", "commands done", ""

        self.agent._MdmAgent__config_store.store = MagicMock()
        self.agent._MdmAgent__config_store.read = MagicMock()
        self.agent._MdmAgent__config_store.read.return_value = response.text

        status = self.agent._MdmAgent__action_radio_configuration(response)

        self.assertEqual(status, "OK")
        expected_calls = [
            call("No previous mesh config"),
            call("ret: %s info: %s", "OK", 'damn good'),
            call("ret: %s info: %s", "OK", 'commands done')
        ]
        self.agent.logger.debug.assert_has_calls(expected_calls, any_order=False)
        self.assertEqual(self.agent._MdmAgent__previous_config_mesh, response.text.strip())

    def test_action_radio_configuration_no_changes(self):
        """
        Test the __action_radio_configuration method
        """
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {
                "radios": [
                    {  # foo config.. Nothing to do with the agent
                        "radio_index": 0,
                    }
                ],
            },
            "version": "1",
        })

        self.agent._MdmAgent__previous_config_mesh = response.text
        self.agent._MdmAgent__config_store.read = MagicMock()
        self.agent._MdmAgent__config_store.read.return_value = response.text

        status = self.agent._MdmAgent__action_radio_configuration(response)

        self.assertEqual(status, "OK")
        self.agent.logger.debug.assert_any_call("No changes in mesh config, not updating.")

    @patch("builtins.open", new_callable=mock_open)
    @patch("mdm_agent.tarfile.open")
    @patch("mdm_agent.os.makedirs")
    @patch("mdm_agent.umask", return_value=0)
    def test_action_certificates_birth_certificate(self, mock_umask, mock_makedirs,
                                                   mock_tarfile_open, mock_file_open):
        """
        Test the __action_certificates method
        """
        agent = mdm_agent.MdmAgent(MagicMock())
        response = MagicMock(spec=requests.Response)
        # not testing with actual tar file content
        response.text = json.dumps({
            "payload": {
                "role": "some_role",
                "group": "some_group",
                "certificates": {
                    "cert1.tar.bz2": base64.b64encode(b"dummy_content").decode('utf-8'),
                    "cert1-sig": base64.b64encode(b"dummy_sig").decode('utf-8')
                }
            }
        })

        with patch('mdm_agent.os.path.basename', side_effect=lambda x: x):
            ret = agent._MdmAgent__action_certificates(response,
                                                       ConfigType.BIRTH_CERTIFICATE.value)

        self.assertEqual(ret, "OK")
        mock_file_open.assert_any_call(f"{agent._MdmAgent__cbma_certs_downloaded}/cert1.tar.bz2",
                                       "wb")
        mock_file_open.assert_any_call(f"{agent._MdmAgent__cbma_certs_downloaded}/cert1-sig", "wb")
        self.assertTrue(mock_tarfile_open.called)
        self.assertTrue(mock_makedirs.called)
        self.assertTrue(mock_umask.called)

    def test_action_certificates_fail_unknown_type(self):
        """
        Test the __action_certificates method
        """
        agent = mdm_agent.MdmAgent(MagicMock())
        response = MagicMock(spec=requests.Response)
        response.text = json.dumps({
            "payload": {
                "certificates": {
                    "certificate1": "certificate1_data",
                    "certificate2": "certificate2_data"
                }
            }
        })
        status = agent._MdmAgent__action_certificates(response, "invalid_config")
        agent.logger.error.assert_called_once_with("Unknown certificate type")
        self.assertEqual(status, "FAIL")

    def test_validate_response_returns_fail_for_non_200_status(self):
        """
        Test the __validate_response method
        """
        mock_response = MagicMock()
        mock_response.status_code = 404
        result = self.agent._MdmAgent__validate_response(mock_response,
                                                         mdm_agent.ConfigType.MESH_CONFIG)
        self.assertEqual(result, 'FAIL')

    def test_validate_response_returns_ok_for_200_status_and_empty_payload(self):
        """
        Test the __validate_response method
        """
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.text = '{"payload": {"radios": [{}]}}'

        result = self.agent._MdmAgent__validate_response(mock_response,
                                                         mdm_agent.ConfigType.MESH_CONFIG)
        self.assertEqual(result, 'OK')

        mock_response.text = '{"payload": {"features": [{}]}}'
        result = self.agent._MdmAgent__validate_response(mock_response,
                                                         mdm_agent.ConfigType.FEATURES)
        self.assertEqual(result, 'OK')

        mock_response.text = '{"payload": {"debug_config": [{}]}}'
        result = self.agent._MdmAgent__validate_response(mock_response,
                                                         mdm_agent.ConfigType.DEBUG_CONFIG)
        self.assertEqual(result, 'OK')

        result = self.agent._MdmAgent__validate_response(mock_response,
                                                     "SPECIAL_UNITTEST_CONFIG")
        self.assertEqual(result, 'FAIL')

    @patch('mdm_agent.socket.create_connection')
    @patch('mdm_agent.ssl.create_default_context')
    @patch('mdm_agent.glob.glob')
    @patch('mdm_agent.logging.Logger')
    def test_get_server_cert_type_rsa(self, mock_logger, mock_glob, mock_ssl_context, mock_socket):
        """
        Test the __get_server_cert_type method
        """
        mock_socket_instance = MagicMock()
        mock_socket.return_value = mock_socket_instance

        mock_ssl_context_instance = MagicMock()
        mock_ssl_context.return_value = mock_ssl_context_instance
        mock_ssock_instance = MagicMock()
        mock_ssl_context_instance.wrap_socket.return_value.__enter__.return_value = mock_ssock_instance

        # Mock the return value of getpeercert
        mock_cert = MagicMock()
        mock_ssock_instance.getpeercert.return_value = mock_cert

        # Create a mock certificate with an RSA public key
        mock_rsa_public_key = MagicMock(spec=rsa.RSAPublicKey)
        mock_x509_cert = MagicMock()
        mock_x509_cert.public_key.return_value = mock_rsa_public_key
        mock_load_der_x509_certificate = patch('mdm_agent.x509.load_der_x509_certificate',
                                               return_value=mock_x509_cert)
        mock_load_der_x509_certificate.start()

        # Mock the return value of glob
        mock_glob.return_value = ['/opt/crypto/rsa/birth/filebased/DNS/test.local.crt']

        agent = mdm_agent.MdmAgent(unittest.mock.MagicMock())
        agent._MdmAgent__ca = '/path/to/ca.crt'
        agent._MdmAgent__url = 'defaultmdm.local:5000'
        agent.logger = mock_logger

        #################
        # RSA certificate
        agent._MdmAgent__get_server_cert_type()
        # Assertions
        mock_socket.assert_called_once_with(('defaultmdm.local', 5000), timeout=20)
        mock_ssl_context.assert_called_once_with(ssl.Purpose.CLIENT_AUTH)
        mock_ssl_context_instance.load_verify_locations.assert_called_once_with(
            cafile="/path/to/ca.crt")
        mock_ssl_context_instance.wrap_socket.assert_called_once_with(mock_socket_instance,
                                                                      server_hostname='defaultmdm.local')
        mock_glob.assert_called_once_with('/opt/crypto/rsa/birth/filebased/DNS/*.local.crt')
        mock_logger.debug.assert_any_call("Server's public key type: %s", 'rsa')
        mock_logger.error.assert_not_called()
        self.assertEqual(agent._MdmAgent__certificate_file,
                         '/opt/crypto/rsa/birth/filebased/DNS/test.local.crt')
        self.assertEqual(agent._MdmAgent__keyfile,
                         '/opt/crypto/rsa/birth/filebased/private.key')
        self.assertEqual(agent._MdmAgent__ca, '/opt/mspki/rsa/certificate_chain.crt')
        mock_load_der_x509_certificate.stop()

    @patch('mdm_agent.socket.create_connection')
    @patch('mdm_agent.ssl.create_default_context')
    @patch('mdm_agent.glob.glob')
    @patch('mdm_agent.logging.Logger')
    def test_get_server_cert_type_ecdsa(self, mock_logger, mock_glob, mock_ssl_context,
                                        mock_socket):
        """
        Test the __get_server_cert_type method
        """
        mock_socket_instance = MagicMock()
        mock_socket.return_value = mock_socket_instance

        mock_ssl_context_instance = MagicMock()
        mock_ssl_context.return_value = mock_ssl_context_instance
        mock_ssock_instance = MagicMock()
        mock_ssl_context_instance.wrap_socket.return_value.__enter__.return_value = mock_ssock_instance

        # Mock the return value of getpeercert
        mock_cert = MagicMock()
        mock_ssock_instance.getpeercert.return_value = mock_cert

        # Create a mock certificate with an RSA public key
        mock_ecdsa_public_key = MagicMock(spec=ec.EllipticCurvePublicKey)
        mock_x509_cert = MagicMock()
        mock_x509_cert.public_key.return_value = mock_ecdsa_public_key
        mock_load_der_x509_certificate = patch('mdm_agent.x509.load_der_x509_certificate',
                                               return_value=mock_x509_cert)
        mock_load_der_x509_certificate.start()

        # Mock the return value of glob
        mock_glob.return_value = ['/opt/crypto/ecdsa/birth/filebased/DNS/test.local.crt']

        agent = mdm_agent.MdmAgent(unittest.mock.MagicMock())
        agent._MdmAgent__ca = '/path/to/ca.crt'
        agent._MdmAgent__url = 'defaultmdm.local:5000'
        agent.logger = mock_logger

        #################
        # ecdsa certificate
        agent._MdmAgent__get_server_cert_type()
        # Assertions
        mock_socket.assert_called_once_with(('defaultmdm.local', 5000), timeout=20)
        mock_ssl_context.assert_called_once_with(ssl.Purpose.CLIENT_AUTH)
        mock_ssl_context_instance.load_verify_locations.assert_called_once_with(
            cafile="/path/to/ca.crt")
        mock_ssl_context_instance.wrap_socket.assert_called_once_with(mock_socket_instance,
                                                                      server_hostname='defaultmdm.local')
        mock_glob.assert_called_once_with(
            '/opt/crypto/ecdsa/birth/filebased/DNS/*.local.crt')
        mock_logger.debug.assert_any_call("Server's public key type: %s", 'ecdsa')
        mock_logger.error.assert_not_called()
        self.assertEqual(agent._MdmAgent__certificate_file,
                         '/opt/crypto/ecdsa/birth/filebased/DNS/test.local.crt')
        self.assertEqual(agent._MdmAgent__keyfile,
                         '/opt/crypto/ecdsa/birth/filebased/private.key')
        self.assertEqual(agent._MdmAgent__ca, '/opt/mspki/ecdsa/certificate_chain.crt')
        mock_load_der_x509_certificate.stop()

    @patch('mdm_agent.socket.create_connection')
    @patch('mdm_agent.ssl.create_default_context')
    @patch('mdm_agent.glob.glob')
    @patch('mdm_agent.logging.Logger')
    def test_get_server_cert_type_eddsa(self, mock_logger, mock_glob, mock_ssl_context,
                                        mock_socket):
        """
        Test the __get_server_cert_type method
        """
        mock_socket_instance = MagicMock()
        mock_socket.return_value = mock_socket_instance

        mock_ssl_context_instance = MagicMock()
        mock_ssl_context.return_value = mock_ssl_context_instance
        mock_ssock_instance = MagicMock()
        mock_ssl_context_instance.wrap_socket.return_value.__enter__.return_value = mock_ssock_instance

        # Mock the return value of getpeercert
        mock_cert = MagicMock()
        mock_ssock_instance.getpeercert.return_value = mock_cert

        # Create a mock certificate with an RSA public key
        mock_eddsa_public_key = MagicMock(spec=ed25519.Ed25519PublicKey)
        mock_x509_cert = MagicMock()
        mock_x509_cert.public_key.return_value = mock_eddsa_public_key
        mock_load_der_x509_certificate = patch('mdm_agent.x509.load_der_x509_certificate',
                                               return_value=mock_x509_cert)
        mock_load_der_x509_certificate.start()

        # Mock the return value of glob
        mock_glob.return_value = ['/opt/crypto/eddsa/birth/filebased/DNS/test.local.crt']

        agent = mdm_agent.MdmAgent(unittest.mock.MagicMock())
        agent._MdmAgent__ca = '/path/to/ca.crt'
        agent._MdmAgent__url = 'defaultmdm.local:5000'
        agent.logger = mock_logger

        #################
        # eddsa certificate
        agent._MdmAgent__get_server_cert_type()
        # Assertions
        mock_socket.assert_called_once_with(('defaultmdm.local', 5000), timeout=20)
        mock_ssl_context.assert_called_once_with(ssl.Purpose.CLIENT_AUTH)
        mock_ssl_context_instance.load_verify_locations.assert_called_once_with(
            cafile="/path/to/ca.crt")
        mock_ssl_context_instance.wrap_socket.assert_called_once_with(mock_socket_instance,
                                                                      server_hostname='defaultmdm.local')
        mock_glob.assert_called_once_with('/opt/crypto/eddsa/birth/filebased/DNS/*.local.crt')
        mock_logger.debug.assert_any_call("Server's public key type: %s", 'eddsa')
        mock_logger.error.assert_not_called()
        self.assertEqual(agent._MdmAgent__certificate_file,
                         '/opt/crypto/eddsa/birth/filebased/DNS/test.local.crt')
        self.assertEqual(agent._MdmAgent__keyfile,
                         '/opt/crypto/eddsa/birth/filebased/private.key')
        self.assertEqual(agent._MdmAgent__ca,
                         '/opt/mspki/eddsa/certificate_chain.crt')
        mock_load_der_x509_certificate.stop()

    @patch('mdm_agent.socket.create_connection')
    @patch('mdm_agent.ssl.create_default_context')
    @patch('mdm_agent.glob.glob')
    @patch('mdm_agent.logging.Logger')
    def test_get_server_cert_type_no_certificate_files(self, mock_logger, mock_glob,
                                                       mock_ssl_context, mock_socket):
        """
        Test the __get_server_cert_type method
        """
        mock_socket_instance = MagicMock()
        mock_socket.return_value = mock_socket_instance

        mock_ssl_context_instance = MagicMock()
        mock_ssl_context.return_value = mock_ssl_context_instance
        mock_ssock_instance = MagicMock()
        mock_ssl_context_instance.wrap_socket.return_value.__enter__.return_value = mock_ssock_instance

        # Mock the return value of getpeercert
        mock_cert = MagicMock()
        mock_ssock_instance.getpeercert.return_value = mock_cert

        # # Create a mock certificate with an rsa public key
        mock_rsa_public_key = MagicMock(spec=rsa.RSAPublicKey)
        mock_x509_cert = MagicMock()
        mock_x509_cert.public_key.return_value = mock_rsa_public_key
        mock_load_der_x509_certificate = patch('mdm_agent.x509.load_der_x509_certificate',
                                               return_value=mock_x509_cert)
        mock_load_der_x509_certificate.start()

        # Mock the return value of glob to be empty
        mock_glob.return_value = []

        agent = mdm_agent.MdmAgent(unittest.mock.MagicMock())
        agent._MdmAgent__ca = '/path/to/ca.crt'
        agent._MdmAgent__url = 'defaultmdm.local:5000'
        agent.logger = mock_logger

        agent._MdmAgent__get_server_cert_type()

        # Assertions
        mock_socket.assert_called_once_with(('defaultmdm.local', 5000), timeout=20)
        mock_ssl_context.assert_called_once_with(ssl.Purpose.CLIENT_AUTH)
        mock_ssl_context_instance.load_verify_locations.assert_called_once_with(
            cafile="/path/to/ca.crt")
        mock_ssl_context_instance.wrap_socket.assert_called_once_with(mock_socket_instance,
                                                                      server_hostname='defaultmdm.local')
        mock_glob.assert_called_once_with('/opt/crypto/rsa/birth/filebased/DNS/*.local.crt')
        mock_logger.debug.assert_any_call("Server's public key type: %s", 'rsa')
        mock_logger.error.assert_called_once_with("No certificate file found for %s", 'rsa')

        self.assertIsNone(agent._MdmAgent__certificate_file)
        self.assertIsNone(agent._MdmAgent__keyfile)
        self.assertEqual(agent._MdmAgent__ca, '/path/to/ca.crt')

        mock_load_der_x509_certificate.stop()

    def test_handle_received_config_debug_config(self):
        """
        Test the __handle_received_config method
        """
        self.agent._MdmAgent__mesh_conf_request_processed = True
        self.agent._MdmAgent__config_store.store = MagicMock()
        response = MagicMock(spec=requests.Response)
        response.text = "debug config content"
        response.status_code = 200

        ret = self.agent._MdmAgent__handle_received_config(response, ConfigType.DEBUG_CONFIG)

        self.assertEqual(ret, "OK")
        self.assertEqual(self.agent._MdmAgent__previous_debug_config, response.text.strip())
        self.agent._MdmAgent__config_store.store.assert_called_once_with(
            ConfigType.DEBUG_CONFIG.value,
            self.agent._MdmAgent__previous_debug_config)
        self.assertEqual(self.agent._MdmAgent__debug_config_interval,
                         Constants.OK_POLLING_TIME_SECONDS.value)

    def test_handle_received_config_mesh_config(self):
        """
        Test the __handle_received_config method
        """
        response = MagicMock(spec=requests.Response)
        response.text = "mesh config content"
        response.status_code = 200

        self.agent._MdmAgent__action_radio_configuration = MagicMock()
        self.agent._MdmAgent__action_radio_configuration.return_value = "OK"
        ret = self.agent._MdmAgent__handle_received_config(response, ConfigType.MESH_CONFIG)

        self.assertEqual(ret, "OK")
        self.agent._MdmAgent__action_radio_configuration.assert_called_once_with(response)

    def test_handle_received_config_features(self):
        """
        Test the __handle_received_config method
        """
        response = MagicMock(spec=requests.Response)
        response.text = "features config content"
        response.status_code = 200

        self.agent._MdmAgent__action_feature_yaml = MagicMock()
        self.agent._MdmAgent__action_feature_yaml.return_value = "OK"

        ret = self.agent._MdmAgent__handle_received_config(response, ConfigType.FEATURES)

        self.assertEqual(ret, "OK")
        self.agent._MdmAgent__action_feature_yaml.assert_called_once_with(response)

    def test_handle_received_config_fail(self):
        """
        Test the __handle_received_config method
        """
        response = MagicMock(spec=requests.Response)
        response.text = "invalid config content"
        response.status_code = 400

        self.agent.__mesh_conf_request_processed = False
        ret = self.agent._MdmAgent__handle_received_config(response, ConfigType.DEBUG_CONFIG)

        self.assertEqual(ret, "FAIL")

    @patch('mdm_agent.MdmAgent')
    @patch('mdm_agent.comms_controller.CommsController')
    def test_main_mdm_no_certificates(self, mock_comms_controller, mock_mdm_agent):
        """
        Test the main_mdm function
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        mock_comms_controller.return_value.logger.debug = MagicMock()
        loop.run_until_complete(main_mdm(None, None, None, None))

        mock_comms_controller.return_value.logger.debug.assert_called_with(
            "MDM: Closing as no certificates provided")
        mock_mdm_agent.assert_not_called()

        loop.close()

    @patch('mdm_agent.MdmAgent')
    @patch('mdm_agent.comms_controller.CommsController')
    def test_main_mdm_with_certificates(self, mock_comms_controller, mock_mdm_agent):
        """
        Test the main_mdm function
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        mock_agent_instance = mock_mdm_agent.return_value
        mock_agent_instance.execute = AsyncMock()
        mock_agent_instance.execute.return_value = ["Success"]

        keyfile = 'path/to/keyfile'
        certfile = 'path/to/certfile'
        ca_file = 'path/to/ca_file'
        interface = 'bat0'

        loop.run_until_complete(main_mdm(keyfile, certfile, ca_file, interface))

        mock_mdm_agent.assert_called_once_with(mock_comms_controller.return_value, keyfile, certfile,
                                             ca_file, interface)
        mock_agent_instance.start_interface_monitor.assert_called_once()
        mock_agent_instance.execute.assert_called_once()
        mock_comms_controller.return_value.logger.debug.assert_called_with(
            "Interface monitor stopped")

        loop.close()

    @patch('mdm_agent.MdmAgent')
    @patch('mdm_agent.signal.signal')
    def test_signal_handling(self, mock_signal, mock_mdm_agent):
        """
        Test the signal handling in the main_mdm function
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        mock_agent_instance = mock_mdm_agent.return_value
        mock_agent_instance.execute = AsyncMock()

        keyfile = 'path/to/keyfile'
        certfile = 'path/to/certfile'
        ca_file = 'path/to/ca_file'
        interface = 'bat0'

        loop.run_until_complete(main_mdm(keyfile, certfile, ca_file, interface))

        self.assertEqual(mock_signal.call_count, 2)
        mock_signal.assert_any_call(signal.SIGINT, unittest.mock.ANY)
        mock_signal.assert_any_call(signal.SIGTERM, unittest.mock.ANY)
        loop.close()

    def test_execute_regular_run(self):
        """
        Test the execute method
        """
        self.agent.upload_certificate_bundle.return_value.status_code = 200
        self.agent.download_certificate_bundle.return_value.status_code = 200
        self.agent._MdmAgent__action_certificates.return_value = "OK"
        self.agent._MdmAgent__status[StatusType.UPLOAD_CERTIFICATES.value] = "OK"
        self.agent._MdmAgent__status[StatusType.DOWNLOAD_CERTIFICATES.value] = "OK"
        self.agent.mdm_service_available = True
        self.agent._MdmAgent__cbma_set_up = True
        self.agent._MdmAgent__mesh_conf_request_processed = True

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def run_execute():
            await asyncio.wait_for(self.agent.execute(), timeout=2)

        with self.assertRaises(asyncio.TimeoutError):
            loop.run_until_complete(run_execute())

        self.agent._MdmAgent__loop_run_executor.assert_any_await(self.agent.executor,
                                                                 ConfigType.FEATURES)
        self.agent._MdmAgent__loop_run_executor.assert_any_await(self.agent.executor,
                                                                 ConfigType.MESH_CONFIG)
        self.agent._MdmAgent__loop_run_executor.assert_any_await(self.agent.executor,
                                                                 ConfigType.DEBUG_CONFIG)

    @patch('mdm_agent.asyncio.get_event_loop')
    def test_loop_run_executor(self, mock_get_event_loop):
        """
        Test the __loop_run_executor method
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        mock_logger = MagicMock()
        agent = mdm_agent.MdmAgent(MagicMock())
        agent.logger = mock_logger

        # Setup the mocks
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.text = "some config text"

        agent._MdmAgent__http_get_device_config = MagicMock(return_value=mock_response)
        agent._MdmAgent__validate_response = MagicMock(return_value="OK")
        agent._MdmAgent__handle_received_config = MagicMock(return_value="OK")

        agent._MdmAgent__config_status_mapping = {ConfigType.DEBUG_CONFIG: "status_debug"}
        agent._MdmAgent__status = {"status_debug": "FAIL"}
        agent._MdmAgent__mesh_conf_request_processed = False
        agent._MdmAgent__previous_debug_config = "old config"
        agent._MdmAgent__debug_config_interval = Constants.OK_POLLING_TIME_SECONDS.value

        # Mocking run_in_executor to return the mocked response
        run_in_executor_mock = AsyncMock(return_value=mock_response)
        mock_get_event_loop.return_value.run_in_executor = run_in_executor_mock

        # Run the method
        executor = MagicMock()
        config = ConfigType.DEBUG_CONFIG
        loop.run_until_complete(agent._MdmAgent__loop_run_executor(executor, config))

        # Assertions
        run_in_executor_mock.assert_called_once_with(executor,
                                                     agent._MdmAgent__http_get_device_config,
                                                     config)
        agent._MdmAgent__validate_response.assert_called_once_with(mock_response, config)
        agent._MdmAgent__handle_received_config.assert_called_once_with(mock_response,
                                                                        ConfigType.DEBUG_CONFIG)

        mock_logger.debug.assert_any_call("HTTP Request status: %s, config: %s",
                                          str(mock_response.status_code), config)
        mock_logger.debug.assert_any_call("config: %s, ret: %s", config, "OK")

        self.assertEqual(agent._MdmAgent__status["status_debug"], "OK")
        self.assertEqual(agent._MdmAgent__mesh_conf_request_processed, False)
        self.assertEqual(agent._MdmAgent__debug_config_interval,
                         Constants.OK_POLLING_TIME_SECONDS.value)

        loop.close()

    @patch('mdm_agent.argparse.ArgumentParser.parse_args')
    def test_main_no_certificates(self, mock_parse_args):
        """
        Test the main function
        """
        # Mock the parsed arguments with None for certificates
        mock_parse_args.return_value = argparse.Namespace(
            keyfile=None, certfile=None, ca=None, interface='bat0'
        )

        # Mock the CommsController instance
        with patch('mdm_agent.comms_controller.CommsController') as mock_comms_controller:
            mock_cc_instance = mock_comms_controller.return_value

            # Call the main function
            mdm_agent.main()

            # Assertion to ensure the function exited early
            mock_cc_instance.logger.debug.assert_called_once_with(
                "MDM: Closing as no certificates provided")

    @patch('mdm_agent.MdmAgent')
    @patch('mdm_agent.comms_controller.CommsController')
    @patch('mdm_agent.argparse.ArgumentParser.parse_args')
    def test_main(self, mock_parse_args, mock_comms_controller, mock_mdm_agent):
        """
        Test the main function
        """
        # Mock the parsed arguments
        mock_parse_args.return_value = argparse.Namespace(
            keyfile='keyfile_path', certfile='certfile_path', ca='ca_path', interface='bat0'
        )

        # Mock the CommsController instance
        mock_cc_instance = mock_comms_controller.return_value

        # Mock the MdmAgent instance
        mock_mdm_instance = mock_mdm_agent.return_value
        mock_mdm_instance.execute = MagicMock(return_value='Execution result')

        # Call the main function
        mdm_agent.main()

        # Assertions to ensure the main logic was executed as expected
        mock_comms_controller.assert_called_once()
        mock_mdm_agent.assert_called_once_with(
            mock_cc_instance, 'keyfile_path', 'certfile_path', 'ca_path', 'bat0'
        )
        mock_mdm_instance.start_interface_monitor.assert_called_once()
        mock_mdm_instance.execute.assert_called_once()
        mock_cc_instance.logger.debug.assert_any_call(
            "MDM: comms_nats_controller Listening for requests")
