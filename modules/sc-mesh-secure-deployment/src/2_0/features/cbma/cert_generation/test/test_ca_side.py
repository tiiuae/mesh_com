import sys
sys.path.insert(0, '../')
from ca_side import monitor_csr_directory, generate_and_send_certificates, handle_client
import pytest
from unittest.mock import MagicMock, patch, mock_open



@pytest.fixture
def mock_socket():
    class MockSocket:
        def __init__(self):
            self.data = None

        def recv(self, num):
            return b'CSR uploaded test.csr'

        def send(self, data):
            self.data = data
            return len(data)

    return MockSocket()


@patch('ca_side.os.path.exists', return_value=True)
@patch('ca_side.generate_and_send_certificates', return_value=None)
@patch('ca_side.os.remove', return_value=None)
@patch('ca_side.logging.error')
def test_handle_client(mock_log_error, mock_remove, mock_gen_send, mock_exists, mock_socket):
    handle_client(mock_socket, "127.0.0.1")

    assert mock_socket.data == b'Filename received'
    mock_gen_send.assert_called_once_with('/tmp/request/test.csr', '127.0.0.1')
    mock_remove.assert_called_once_with('/tmp/request/test.csr')
    mock_log_error.assert_not_called()
