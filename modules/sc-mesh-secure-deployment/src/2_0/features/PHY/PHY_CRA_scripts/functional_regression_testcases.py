import pytest
import socket
from unittest.mock import patch, MagicMock, mock_open
from SP_CRA_mod import PHYCRA
import numpy as np

@pytest.fixture
def phycra_instance():
    with patch('SP_CRA_mod.socket.socket'):
        instance = PHYCRA()
        instance.acf = MagicMock()
        instance.acf_client = MagicMock()
        yield instance
        # Ensure proper cleanup
        instance.server.close()
        if hasattr(instance, 'shutdown_server'):
            instance.shutdown_server()
        instance.listen_sock.close()

# Test for __init__ method
def test_init(phycra_instance):
    assert phycra_instance.SERVER is not None
    assert phycra_instance.BROADCAST_PORT == 5051
    assert phycra_instance.PORT == 5050

# Test for log_authentication method
def test_log_authentication(phycra_instance):
    with patch('builtins.open', mock_open()) as mock_file:
        phycra_instance.log_authentication('127.0.0.1', '00:00:00:00:00:00', 'Success')
        mock_file.assert_called_with('server_log.txt', 'a')
        mock_file().write.assert_called()

# Test for display_table method
def test_display_table(phycra_instance, capsys):
    with patch('builtins.open', mock_open(read_data='2023-11-03 12:00:00\t127.0.0.1\t00:00:00:00:00:00\tSuccess\n')):
        phycra_instance.display_table()
        captured = capsys.readouterr()
        assert '127.0.0.1' in captured.out
        assert '00:00:00:00:00:00' in captured.out
        assert 'Success' in captured.out

# Test for server_start method
def test_server_start(phycra_instance):
    with patch('SP_CRA_mod.socket.socket') as mock_socket, \
         patch('SP_CRA_mod.threading.Thread') as mock_thread:
        phycra_instance.server_start()
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_thread.assert_called()

# Test for broadcast_status method
def test_broadcast_status(phycra_instance):
    with patch('SP_CRA_mod.socket.socket') as mock_socket:
        phycra_instance.broadcast_status()
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_DGRAM)

# Test for get_mac_address method
def test_get_mac_address(phycra_instance):
    with patch('SP_CRA_mod.get_mac_address') as mock_get_mac:
        mock_get_mac.return_value = '00:00:00:00:00:00'
        assert phycra_instance.get_mac_address() == '00:00:00:00:00:00'

# Test for connect_to_server method
def test_connect_to_server(phycra_instance):
    with patch('SP_CRA_mod.socket.socket') as mock_socket, \
         patch('SP_CRA_mod.pickle.loads') as mock_pickle_loads:
        mock_socket.return_value.recv.return_value = b'pickle_data'
        mock_pickle_loads.return_value = np.array([1, 2, 3])
        phycra_instance.acf_client = np.array([[1, 2, 3]])
        phycra_instance.connect_to_server('127.0.0.1')
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)

# Test for listen_for_broadcast method
def test_listen_for_broadcast(phycra_instance):
    with patch('SP_CRA_mod.socket.socket') as mock_socket, \
         patch('SP_CRA_mod.PHYCRA.connect_to_server') as mock_connect:
        phycra_instance.listen_for_broadcast()
        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_DGRAM)
        mock_connect.assert_not_called()

# Test for get_server_ip method
def test_get_server_ip(phycra_instance):
    with patch('SP_CRA_mod.socket.socket') as mock_socket:
        mock_socket.return_value.getsockname.return_value = ('127.0.0.1', 0)
        assert phycra_instance.get_server_ip() == '127.0.0.1'

@patch('SP_CRA_mod.socket.socket')
def test_handle_client_success(mock_socket, phycra_instance):
    # Set up
    conn = MagicMock()
    addr = ('192.168.1.1', 5050)
    phycra_instance.acf = ['dummy_acf']
    random_index = 0

    with patch('SP_CRA_mod.random.randint', return_value=random_index), \
         patch('SP_CRA_mod.pickle.dumps', return_value=b'encoded_acf'), \
         patch('SP_CRA_mod.PHYCRA.log_authentication') as mock_log_auth:

        conn.recv.side_effect = [
            (2).to_bytes(2, 'big'),  # rx_index_length
            str(random_index).encode(phycra_instance.FORMAT),  # rx_index
            (2).to_bytes(2, 'big'),  # mac_address_length
            'AA:BB:CC:DD:EE:FF'.encode(phycra_instance.FORMAT)  # mac_address
        ]

        # Invoke
        phycra_instance.handle_client(conn, addr)

        # Check
        conn.sendall.assert_called_with(b'encoded_acf')
        mock_log_auth.assert_called_with(addr[0], 'AA:BB:CC:DD:EE:FF', "Success")

@patch('SP_CRA_mod.socket.socket')
def test_handle_client_fail(mock_socket, phycra_instance):
    # Set up
    conn = MagicMock()
    addr = ('192.168.1.1', 5050)
    phycra_instance.acf = ['dummy_acf']
    random_index = 0
    wrong_index = 1  # Different index to simulate failure

    with patch('SP_CRA_mod.random.randint', return_value=random_index), \
         patch('SP_CRA_mod.pickle.dumps', return_value=b'encoded_acf'), \
         patch('SP_CRA_mod.PHYCRA.log_authentication') as mock_log_auth:

        conn.recv.side_effect = [
            (2).to_bytes(2, 'big'),  # rx_index_length
            str(wrong_index).encode(phycra_instance.FORMAT),  # rx_index
            (2).to_bytes(2, 'big'),  # mac_address_length
            'AA:BB:CC:DD:EE:FF'.encode(phycra_instance.FORMAT)  # mac_address
        ]

        # Invoke
        phycra_instance.handle_client(conn, addr)

        # Check
        conn.sendall.assert_called_with(b'encoded_acf')
        mock_log_auth.assert_called_with(addr[0], 'AA:BB:CC:DD:EE:FF', "Access denied")

# Main function to run the tests
if __name__ == '__main__':
    options = ['-v', '-rA']
    pytest.main(options)

