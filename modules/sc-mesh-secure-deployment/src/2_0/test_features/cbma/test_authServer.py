import pytest
from unittest.mock import patch, MagicMock, call
import shutil
import os
import socket
import ssl
import threading

import add_syspath
from auth.authServer import AuthServer

class MockLogger:
    def __init__(self):
        self.info = MagicMock()
        self.error = MagicMock()
        self.debug = MagicMock()

# Mock Logger
@pytest.fixture
def mock_logger():
    with patch('auth.authServer.logger', return_value=MockLogger()) as mock_log:
        yield {
            'mock_log': mock_log
        }

# Mock external module functions
@pytest.fixture
def mock_dependencies():
    with patch('auth.authServer.verify_cert', return_value=True) as mock_verify, \
            patch('auth.authServer.get_mac_addr', return_value="00:00:00:00:00:00") as mock_get_mac, \
            patch('auth.authServer.ssl.SSLContext', autospec=True) as mock_ssl_context:

        mock_ssl_instance = MagicMock()
        mock_ssl_context.return_value = mock_ssl_instance
        mock_ssl_instance.load_verify_locations = MagicMock()  # Mocking the method

        yield {
            'mock_verify': mock_verify,
            'mock_get_mac': mock_get_mac,
            'mock_ssl_context': mock_ssl_context,
            'mock_ssl_instance': mock_ssl_instance
        }

# Mock socket related functions
@pytest.fixture
def mock_socket():
    with patch('auth.authServer.socket.socket') as mock_sock:
        mock_sock_instance = MagicMock()
        mock_sock.return_value = mock_sock_instance
        yield {
            'mock_sock': mock_sock,
            'mock_sock_instance': mock_sock_instance
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

    monkeypatch.setattr("auth.authServer.glob.glob", fake_glob)


def test_auth_server_handle_client(mock_dependencies, mock_socket, mock_glob, mock_logger):
    server = AuthServer("some_interface", "127.0.0.1", 15001, "path/to/cert", MagicMock())

    fake_client_connection = MagicMock()
    fake_client_address = ('::1', 12345)
    server.handle_client(fake_client_connection, fake_client_address)

    # Add assertions to verify the correct methods were called
    mock_dependencies['mock_verify'].assert_called_once()


def test_auth_server_start_stop(mock_dependencies, mock_socket, mock_glob, mock_logger):
    server = AuthServer("some_interface", "127.0.0.1", 15001, "path/to/cert", MagicMock())

    # Mock accept to raise socket.timeout just once
    mock_socket['mock_sock_instance'].accept.side_effect = [socket.timeout]

    # Temporarily set server.running to False to stop the loop after one iteration
    with patch.object(server, 'running', new=False):
        # Start the server in a separate thread
        thread = threading.Thread(target=server.start_server)
        thread.start()

        # Join the thread to make sure it's finished before proceeding with assertions
        thread.join()

    # Add assertions to verify the server started and stopped correctly
    mock_socket['mock_sock_instance'].bind.assert_called()
    mock_socket['mock_sock_instance'].listen.assert_called_once()


def test_authenticate_client_verification_pass(mock_dependencies, mock_socket, mock_glob, mock_logger):
    server = AuthServer("some_interface", '::1', 15001, "path/to/cert", MagicMock())
    fake_client_connection = MagicMock()
    fake_client_address = ('::1', 12345)
    # Mock verify_cert to return True
    mock_dependencies['mock_verify'].return_value = True
    # Call authenticate_client
    server.authenticate_client(fake_client_connection, fake_client_address, "fake_client_mac")
    # Assert mua.auth_pass was called
    server.mua.auth_pass.assert_called_once()
    # Assert mua.auth_fail was NOT called
    server.mua.auth_fail.assert_not_called()

def test_authenticate_client_verification_fail(mock_dependencies, mock_socket, mock_glob, mock_logger):
    server = AuthServer("some_interface", '::1', 15001, "path/to/cert", MagicMock())
    fake_client_connection = MagicMock()
    fake_client_address = ('::1', 12345)
    # Mock verify_cert to return False
    mock_dependencies['mock_verify'].return_value = False
    # Call authenticate_client
    server.authenticate_client(fake_client_connection, fake_client_address, "fake_client_mac")
    # Assert mua.auth_fail was called
    server.mua.auth_fail.assert_called_once()
    # Assert mua.auth_pass was NOT called
    server.mua.auth_pass.assert_not_called()

def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()
