import pytest
from unittest.mock import patch, MagicMock, call

import add_syspath
from auth.authClient import AuthClient

class MockLogger:
    def __init__(self):
        self.info = MagicMock()
        self.error = MagicMock()
        self.debug = MagicMock()

# Mock Logger
@pytest.fixture
def mock_logger():
    with patch('auth.authClient.logger_instance.get_logger', return_value=MockLogger()) as mock_log:
        yield {
            'mock_log': mock_log
        }

# Mock external module functions
@pytest.fixture
def mock_dependencies():
    with patch('auth.authClient.verify_cert', return_value=True) as mock_verify, \
            patch('auth.authClient.mac_to_ipv6', return_value="::1") as mock_mac_ipv6, \
            patch('auth.authClient.get_mac_addr', return_value="00:00:00:00:00:00") as mock_get_mac:
        yield {
            'mock_verify': mock_verify,
            'mock_mac_ipv6': mock_mac_ipv6,
            'mock_get_mac': mock_get_mac
        }

# Mock ssl and socket related functions
@pytest.fixture
def mock_socket():
    with patch('auth.authClient.socket.socket') as mock_sock, \
            patch('auth.authClient.ssl.SSLContext') as mock_ssl_context:
        mock_sock_instance = MagicMock()
        mock_sock.return_value = mock_sock_instance

        mock_context_instance = MagicMock()
        mock_ssl_context.return_value = mock_context_instance

        mock_sec_sock_instance = MagicMock()
        mock_context_instance.wrap_socket.return_value = mock_sec_sock_instance

        yield {
            'mock_sock': mock_sock,
            'mock_sock_instance': mock_sock_instance,
            'mock_ssl_context': mock_ssl_context,
            'mock_context_instance': mock_context_instance,
            'mock_sec_sock_instance': mock_sec_sock_instance
        }

# Mock glob.glob
@pytest.fixture
def mock_glob(monkeypatch):
    def fake_glob(pattern):
        if 'ca.crt' in pattern:
            return ['dummy_path/ca.crt']
        elif 'crt' in pattern:
            return ['dummy_path/dummy_cert.crt']
        elif 'key' in pattern:
            return ['dummy_path/dummy_key.key']
        return ['dummy_path']

    monkeypatch.setattr("auth.authClient.glob.glob", fake_glob)

# Mock sleep
@pytest.fixture
def mock_time_sleep(monkeypatch):
    def fake_sleep(*args, **kwargs):
        pass  # Do nothing

    monkeypatch.setattr("auth.authClient.time.sleep", fake_sleep)


def test_auth_client_establish_connection(mock_dependencies, mock_socket, mock_glob, mock_logger):
    client = AuthClient("some_interface", "some_server_mac", 15001, "path/to/cert", MagicMock())

    client.establish_connection()

    # Add assertions to verify correct methods were called
    mock_socket['mock_sock'].assert_called_once()
    mock_socket['mock_context_instance'].load_verify_locations.assert_called_once()
    mock_socket['mock_context_instance'].load_cert_chain.assert_called_once()
    mock_socket['mock_sec_sock_instance'].connect.assert_called_once()
    mock_dependencies['mock_verify'].assert_called_once()


def test_auth_client_connection_failure(mock_dependencies, mock_socket, mock_glob, mock_time_sleep, mock_logger):
    client = AuthClient("some_interface", "some_server_mac", 15001, "path/to/cert", MagicMock())

    mock_socket['mock_sec_sock_instance'].connect.side_effect = ConnectionRefusedError()

    client.establish_connection()

    # Add assertions to verify the connection retry behavior
    assert mock_socket['mock_sec_sock_instance'].connect.call_count == 5  # Max retries

def test_auth_client_certificate_verification_pass(mock_dependencies, mock_socket, mock_glob, mock_logger):
    client = AuthClient("some_interface", "some_server_mac", 15001, "path/to/cert", MagicMock())

    mock_dependencies['mock_verify'].return_value = True
    client.establish_connection()
    # Assert mua.auth_pass was called
    client.mua.auth_pass.assert_called_once()
    # Assert mua.auth_fail was NOT called
    client.mua.auth_fail.assert_not_called()

def test_auth_client_certificate_verification_failure(mock_dependencies, mock_socket, mock_glob, mock_logger):
    client = AuthClient("some_interface", "some_server_mac", 15001, "path/to/cert", MagicMock())

    mock_dependencies['mock_verify'].return_value = False
    client.establish_connection()
    # Assert mua.auth_fail was called
    client.mua.auth_fail.assert_called_once()
    # Assert mua.auth_pass was NOT called
    client.mua.auth_pass.assert_not_called()

# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()