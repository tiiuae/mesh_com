import contextlib
import threading
import time
import os
import pytest
import subprocess
from unittest.mock import patch, MagicMock

import add_syspath
from mba import MBA

path_to_dir = os.path.dirname(__file__)  # Path to dir containing this script

# Hack to support CM2 as well
# Use the first interface which is available in the device to test MBA
interfaces = ["wlp1s0", "wlp2s0", "wlp3s0", "halow1", "eth0"]
for interface in interfaces:
    with contextlib.suppress(subprocess.CalledProcessError):
        output = subprocess.check_output(['ifconfig', interface], stderr=subprocess.DEVNULL)
        if 'inet' in output.decode():
            test_interface = interface
            break

@pytest.fixture
def mba_setup():
    mock_sender_mac = "mac1"
    mock_receiver_mac = "mac2"

    with patch('mba.logger', new_callable=MagicMock) as mock_logger:
        sender = MBA(decision_engine=MagicMock(), multicast_group="ff02::1", port=12345, interface=test_interface, my_cert_dir=f"{path_to_dir}/certificate_samples", peer_cert_dir=f"{path_to_dir}/certificate_samples", stop_event=threading.Event())
        receiver = MBA(decision_engine=MagicMock(), multicast_group="ff02::1", port=12345, interface=test_interface, my_cert_dir=f"{path_to_dir}/certificate_samples", peer_cert_dir=f"{path_to_dir}/certificate_samples", stop_event=threading.Event())

        sender.mymac = mock_sender_mac
        sender.my_cert_dir = f"{path_to_dir}/certificate_samples/{mock_sender_mac}"
        receiver.mymac = mock_receiver_mac
        receiver.my_cert_dir = f"{path_to_dir}/certificate_samples/{mock_receiver_mac}"

        receiver_thread = threading.Thread(target=receiver.receive_mba)
        receiver_thread.start()

        # Wait for the receiver to be ready
        time.sleep(1)

        yield sender, receiver, receiver_thread, mock_logger

        # Cleanup
        receiver.stop_event.set()
        receiver_thread.join()

@patch('mba.get_mac_from_ipv6')
def test_send_receive_mba(mock_get_mac_from_ipv6, mba_setup):
    sender, receiver, _, mock_logger = mba_setup
    mock_get_mac_from_ipv6.return_value = "mac1"  # Mock the function to return "mac1" (sender's mac address)

    with patch.object(receiver, 'notify', new_callable=MagicMock) as mock_notify:
        # Sending a test message
        malicious_mac = "AA:BB:CC:DD:EE:FF"
        malicious_ip = "fe80::abcd"
        sender.send_mba(malicious_mac, malicious_ip)

        # Wait for the message to be processed
        time.sleep(1)  # Adjust as needed


        # Assertions for log calls
        expected_send_log_fragment = "Sending data"
        expected_receive_log_fragment = "Received data"

        send_log_found = any(expected_send_log_fragment in call[0][0] for call in mock_logger.info.call_args_list)
        receive_log_found = any(expected_receive_log_fragment in call[0][0] for call in mock_logger.info.call_args_list)

        assert send_log_found, f"Send log with message fragment '{expected_send_log_fragment}' not found." # Asserts that MBA message was signed and sent
        assert receive_log_found, f"Receive log with message fragment '{expected_receive_log_fragment}' not found." # Asserts that MBA message was received

        # Assertions for notify call
        mock_notify.assert_called() # Asserts that the received signature was verified and therefore decision engine was notified

@pytest.fixture
def mba_invalid_signature_setup():
    mock_sender_mac = "invalid_cert" # Sender does not have valid keys
    mock_receiver_mac = "mac2"

    with patch('mba.logger', new_callable=MagicMock) as mock_logger:
        sender = MBA(decision_engine=MagicMock(), multicast_group="ff02::1", port=12345, interface=test_interface, my_cert_dir=f"{path_to_dir}/certificate_samples", peer_cert_dir=f"{path_to_dir}/certificate_samples", stop_event=threading.Event())
        receiver = MBA(decision_engine=MagicMock(), multicast_group="ff02::1", port=12345, interface=test_interface, my_cert_dir=f"{path_to_dir}/certificate_samples", peer_cert_dir=f"{path_to_dir}/certificate_samples", stop_event=threading.Event())

        sender.mymac = mock_sender_mac
        sender.my_cert_dir = f"{path_to_dir}/certificate_samples/{mock_sender_mac}"
        receiver.mymac = mock_receiver_mac
        receiver.my_cert_dir = f"{path_to_dir}/certificate_samples/{mock_receiver_mac}"

        receiver_thread = threading.Thread(target=receiver.receive_mba)
        receiver_thread.start()

        # Wait for the receiver to be ready
        time.sleep(1)

        yield sender, receiver, receiver_thread, mock_logger

        # Cleanup
        receiver.stop_event.set()
        receiver_thread.join()

@patch('mba.get_mac_from_ipv6')
def test_send_receive_mba_invalid_signature(mock_get_mac_from_ipv6, mba_invalid_signature_setup):
    sender, receiver, _, mock_logger = mba_invalid_signature_setup
    mock_get_mac_from_ipv6.return_value = "invalid_cert"  # Mock the function to return "mac1" (sender's mac address)

    with patch.object(receiver, 'notify', new_callable=MagicMock) as mock_notify:
        # Sending a test message
        malicious_mac = "AA:BB:CC:DD:EE:FF"
        malicious_ip = "fe80::abcd"
        sender.send_mba(malicious_mac, malicious_ip)

        # Wait for the message to be processed
        time.sleep(1)  # Adjust as needed

        # Assertions for log calls
        expected_send_log_fragment = "Sending data"
        expected_receive_log_fragment = "Received data"

        send_log_found = any(expected_send_log_fragment in call[0][0] for call in mock_logger.info.call_args_list)
        receive_log_found = any(expected_receive_log_fragment in call[0][0] for call in mock_logger.info.call_args_list)

        assert send_log_found, f"Send log with message fragment '{expected_send_log_fragment}' not found." # Asserts that MBA message was signed and sent
        assert receive_log_found, f"Receive log with message fragment '{expected_receive_log_fragment}' not found." # Asserts that MBA message was received

        # Assertions for notify call
        mock_notify.assert_not_called() # Asserts that the received signature was invalid and therefore decision engine was not notified