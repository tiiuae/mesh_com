import pytest
from unittest.mock import MagicMock, patch
import ssl
import json
import queue
import logging
import add_syspath
from secure_channel.secchannel import SecMessageHandler

class MockLogger:
    def __init__(self):
        self.info = MagicMock()
        self.error = MagicMock()
        self.debug = MagicMock()

# Mock Logger
@pytest.fixture
def mock_logger(monkeypatch):
    with patch('secure_channel.secchannel.logger_instance.get_logger', return_value=MockLogger()) as mock_log:
        yield {
            'mock_log': mock_log
        }

@pytest.fixture
def sec_message_handler(mock_logger):
    mock_socket = MagicMock(spec=ssl.SSLSocket)
    handler = SecMessageHandler(mock_socket)
    return handler

def test_set_callback(sec_message_handler, mock_logger):
    mock_callback = MagicMock()
    sec_message_handler.set_callback(mock_callback)
    assert sec_message_handler.callback == mock_callback


def test_is_socket_active_true(sec_message_handler, monkeypatch, mock_logger):
    monkeypatch.setattr(sec_message_handler.socket, 'fileno', lambda: 5)
    assert sec_message_handler._is_socket_active()


def test_is_socket_active_false(sec_message_handler, monkeypatch, mock_logger):
    monkeypatch.setattr(sec_message_handler.socket, 'fileno', lambda: -1)
    assert not sec_message_handler._is_socket_active()


def test_is_ssl_socket(sec_message_handler, monkeypatch, mock_logger):
    assert sec_message_handler._is_ssl_socket()


def test_send_message_no_ssl(sec_message_handler, monkeypatch, mock_logger):
    monkeypatch.setattr(sec_message_handler.socket, '__class__', MagicMock())
    sec_message_handler.send_message("test_message")
    sec_message_handler.socket.send.assert_not_called()


def test_send_message_ssl_inactive(sec_message_handler, monkeypatch, mock_logger):
    monkeypatch.setattr(sec_message_handler.socket, 'fileno', lambda: -1)
    sec_message_handler.send_message("test_message")
    sec_message_handler.socket.send.assert_not_called()


def test_send_message_successful(sec_message_handler, monkeypatch, mock_logger):
    mock_ssl_socket = MagicMock(spec=ssl.SSLSocket)
    mock_ssl_socket.fileno.return_value = 5
    mock_ssl_socket.send = MagicMock()
    monkeypatch.setattr(sec_message_handler, 'socket', mock_ssl_socket)

    sec_message_handler.send_message("test_message")
    mock_ssl_socket.send.assert_called_once_with("test_message".encode())


def test_receive_message_with_macsec_params(sec_message_handler, monkeypatch):
    mock_message = json.dumps({
        'bytes_for_my_key': 'some_value',
        'bytes_for_client_key': 'some_other_value',
        'port': 12345
    })

    mock_ssl_socket = MagicMock(spec=ssl.SSLSocket)
    mock_ssl_socket.getpeername.return_value = ('127.0.0.1', 54321)
    mock_ssl_socket.recv.side_effect = [mock_message.encode('utf-8'), b"GOODBYE"]

    monkeypatch.setattr(sec_message_handler, 'socket', mock_ssl_socket)

    test_queue = queue.Queue()

    sec_message_handler.receive_message(macsec_param_q=test_queue)

    assert not test_queue.empty()
    received_data = test_queue.get_nowait()
    assert received_data == mock_message


def test_receive_message_successful(monkeypatch, caplog):
    # Mocking the necessary methods and attributes
    mock_ssl_socket = MagicMock(spec=ssl.SSLSocket)
    mock_ssl_socket.getpeername.return_value = ('127.0.0.1', 12345)

    # Simulate the reception of three messages and then the GOODBYE message
    mock_ssl_socket.recv.side_effect = [b"test_message_1", b"test_message_2", b"test_message_3", b"GOODBYE"]
    sec_message_handler = SecMessageHandler(mock_ssl_socket)
    #monkeypatch.setattr(sec_message_handler, 'socket', mock_ssl_socket)

    # You can also capture log outputs to check if the expected logs were created.
    with caplog.at_level(logging.INFO):
        sec_message_handler.receive_message()

    # Check that the logger contains the expected logs
    assert "Received: test_message_1 from 127.0.0.1" in caplog.text
    assert "Received: test_message_2 from 127.0.0.1" in caplog.text
    assert "Received: test_message_3 from 127.0.0.1" in caplog.text
    assert "Other end signaled end of communication." in caplog.text

# Remove log directory during teardown
import shutil
import os
def remove_logs_directory():
    logs_directory = 'logs'
    if os.path.exists(logs_directory) and os.path.isdir(logs_directory):
        shutil.rmtree(logs_directory)

def teardown_module():
    remove_logs_directory()