from unittest.mock import patch, MagicMock
import json
import pytest
import queue

import add_syspath
from multicast.multicast import MulticastHandler

class MockLogger:
    def __init__(self):
        self.info = MagicMock()
        self.error = MagicMock()
        self.debug = MagicMock()

# Mock Logger
@pytest.fixture
def mock_logger():
    with patch('multicast.multicast.logger_instance.get_logger', return_value=MockLogger()) as mock_log:
        yield {
            'mock_log': mock_log
        }
@patch('socket.socket')
def test_send_multicast_message(mock_socket, mock_logger):
    # Setup
    mock_queue = MagicMock()
    mock_sock_instance = MagicMock()
    mock_socket.return_value.__enter__.return_value = mock_sock_instance

    # Create a MulticastHandler instance
    handler = MulticastHandler(mock_queue, 'ff02::1', 10000, 'lo')

    # Data to be sent
    data = 'some_mac_address'

    # Expected message
    expected_message = {
        'mac_address': data,
        'message_type': 'mac_announcement'
    }

    # Call send_multicast_message
    handler.send_multicast_message(data)

    # Assertions
    mock_sock_instance.sendto.assert_called_once_with(json.dumps(expected_message).encode('utf-8'), ('ff02::1', 10000))


@patch('socket.socket')
def test_receive_multicast(mock_socket, mock_logger):
    # Setup
    test_queue = queue.Queue()
    mock_sock_instance = MagicMock()
    mock_socket.return_value.__enter__.return_value = mock_sock_instance

    # Mock recvfrom() to return a message and address
    received_message = {
        'mac_address': 'some_mac',
        'message_type': 'mac_announcement'
    }

    # Set recvfrom() to return the message once and then raise an exception to break the loop
    mock_sock_instance.recvfrom.side_effect = [
        (json.dumps(received_message).encode('utf-8'), ('::1', 10000)),
        KeyboardInterrupt
    ]

    # Create a MulticastHandler instance
    # Mock get_mac_addr() to return an excluded MAC address
    with patch('multicast.multicast.get_mac_addr', return_value='excluded_mac'):
        handler = MulticastHandler(test_queue, 'ff02::1', 10000, 'lo')

    with pytest.raises(KeyboardInterrupt):
        handler.multicast_message()

    # Check if the message is added to the queue
    assert not test_queue.empty()
    queue_item = test_queue.get_nowait()
    assert queue_item == ("MULTICAST", 'some_mac')

@patch('socket.socket')
def test_receive_multicast_excluded_mac(mock_socket, mock_logger):
    # Setup
    test_queue = queue.Queue()
    mock_sock_instance = MagicMock()
    mock_socket.return_value.__enter__.return_value = mock_sock_instance

    # Mock recvfrom() to return a message and address
    received_message = {
        'mac_address': 'excluded_mac',
        'message_type': 'mac_announcement'
    }

    # Set recvfrom() to return the message once and then raise an exception to break the loop
    mock_sock_instance.recvfrom.side_effect = [
        (json.dumps(received_message).encode('utf-8'), ('::1', 10000)),
        KeyboardInterrupt
    ]

    # Create a MulticastHandler instance
    # Mock get_mac_addr() to return an excluded MAC address
    with patch('multicast.multicast.get_mac_addr', return_value='excluded_mac'):
        handler = MulticastHandler(test_queue, 'ff02::1', 10000, 'lo')


    with pytest.raises(KeyboardInterrupt):
        handler.multicast_message()

    # Check that excluded mac is not added to queue
    assert test_queue.empty()
