import pytest
from unittest.mock import patch, Mock
import sys
import subprocess
import socket
sys.path.insert(0, '../')
import client_side

# Sample data for the tests
sample_csr_output = "Some random text. CSR generated:  test.csr"

def test_run_command_successful():
    with patch('subprocess.run', return_value=Mock(stdout=sample_csr_output)):
        result = client_side.run_command("ls")
        assert result.stdout == sample_csr_output

def test_run_command_error():
    with patch('subprocess.run', side_effect=subprocess.CalledProcessError(1, 'cmd', stderr="Error")):
        with pytest.raises(SystemExit):
            client_side.run_command("invalid_command")

def test_is_server_reachable():
    with patch('socket.socket') as mock_socket:
        instance = mock_socket.return_value.__enter__.return_value
        instance.settimeout.return_value = None
        instance.connect.return_value = None

        assert client_side.is_server_reachable("127.0.0.1") == True

def test_is_server_unreachable_due_to_timeout():
    with patch('socket.socket') as mock_socket:
        instance = mock_socket.return_value.__enter__.return_value
        instance.connect.side_effect = socket.timeout

        assert client_side.is_server_reachable("192.0.2.0") == False

def test_is_server_unreachable_due_to_refusal():
    with patch('socket.socket') as mock_socket:
        instance = mock_socket.return_value.__enter__.return_value
        instance.connect.side_effect = ConnectionRefusedError

        assert client_side.is_server_reachable("192.0.2.0") == False

def test_upload_file_to_server():
    with patch('client_side.run_command', return_value=None):
        client_side.upload_file_to_server("test.csr", "127.0.0.1", "user")

def test_are_files_received():
    with patch('os.path.exists', side_effect=lambda x: True):
        assert client_side.are_files_received("test.csr")

def test_are_files_not_received():
    with patch('os.path.exists', side_effect=lambda x: False):
        assert not client_side.are_files_received("test.csr")