import pytest
import socket
from unittest.mock import MagicMock, patch, call
from SP_CRA_v7 import PHYCRA  

@pytest.fixture
def phycra_instance():
    instance = PHYCRA()
    instance.server_thread = MagicMock()
    instance.listen_thread = MagicMock()
    instance.broadcast_thread = MagicMock()
    instance.stop_event = MagicMock()
    instance.server = MagicMock()
    # Mock the broadcast_sock if it doesn't exist in PHYCRA
    instance.broadcast_sock = MagicMock()
    return instance

# Mocking incorrect index transmission
def test_incorrect_index_transmission(phycra_instance):
    with patch('socket.socket'):
        conn = MagicMock()
        phycra_instance.acf = MagicMock()
        # Simulate receiving a malformed packet
        conn.recv.return_value = b'\x00\x00\x00'
        phycra_instance.handle_client(conn, ('127.0.0.1', 5050))
        conn.send.assert_not_called()  # Assuming it shouldn't send anything on error

# Basic security test cases
def test_replay_attack_prevention(phycra_instance):
    with patch('socket.socket'):
        conn = MagicMock()
        addr = ('127.0.0.1', 5050)
        phycra_instance.acf = MagicMock()
        phycra_instance.acf.check_validity = MagicMock(return_value=False)
        phycra_instance.handle_client(conn, addr)
        conn.send.assert_not_called()  # Assuming it shouldn't send anything if replay attack detected



