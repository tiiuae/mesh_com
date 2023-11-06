import pytest
from unittest.mock import MagicMock
import numpy as np
from SP_CRA_script import PHYCRA  # Replace 'phycra_module' with the actual module name where PHYCRA is defined

@pytest.fixture
def phycra_instance():
    phycra = PHYCRA()
    phycra.acf = np.array([[1, 2, 3], [4, 5, 6]])  # Simplified ACF for testing
    phycra.acf_client = phycra.acf  # Ensure client and server ACF are the same
    return phycra

def test_handle_client_success(phycra_instance):
    conn = MagicMock()
    addr = ("127.0.0.1", 5050)
    phycra_instance.handle_client(conn, addr)
    assert conn.close.called

def test_handle_client_failure(phycra_instance):
    # Simulate a client sending a wrong index
    conn = MagicMock()
    conn.recv.side_effect = [b'\x00\x01', b'1', b'\x00\x0c', b'fake-mac-address']
    addr = ("127.0.0.1", 5050)
    phycra_instance.handle_client(conn, addr)
    assert conn.close.called

def test_handle_client_exception(phycra_instance):
    # Simulate an exception during client handling
    conn = MagicMock()
    conn.recv.side_effect = Exception("Test Exception")
    addr = ("127.0.0.1", 5050)
    phycra_instance.handle_client(conn, addr)
    assert conn.close.called

def test_handle_client_disconnection(phycra_instance):
    # Simulate a client disconnecting unexpectedly
    conn = MagicMock()
    conn.recv.side_effect = [b'\x00\x01', b'0', b'\x00\x0c', ConnectionResetError("Client disconnected")]
    addr = ("127.0.0.1", 5050)
    phycra_instance.handle_client(conn, addr)
    assert conn.close.called

