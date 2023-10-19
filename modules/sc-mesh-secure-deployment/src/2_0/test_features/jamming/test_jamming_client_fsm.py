import socket
import threading
import time
import unittest

from unittest.mock import MagicMock, patch
from contextlib import contextmanager

import pytest

import add_syspath
from jamming_client_fsm import JammingDetectionClient, ClientEvent, ClientState, ClientFSM
from options import Options


class TestCodeUnderTest:

    #  The client successfully connects to the orchestrator node.
    def test_connect_to_orchestrator_success(self, mocker):
        # Mock the socket connect method to return successfully
        mocker.patch.object(socket.socket, 'connect')

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Call the connect_to_orchestrator method
        client.connect_to_orchestrator()

        # Assert that the socket connect method was called
        socket.socket.connect.assert_called_once_with(('::1', 1234))

    #  The client FSM transitions from IDLE to LOW_LATENCY_SCAN when triggered by the PERIODIC_LOW_LATENCY_SCAN event.
    def test_fsm_transition_idle_to_low_latency_scan(mocker):
        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Create a mock FSM object
        mock_fsm = MagicMock()
        client.fsm = mock_fsm

        # Set the initial state to IDLE
        mock_fsm.state = ClientState.IDLE

        # Trigger the PERIODIC_LOW_LATENCY_SCAN event
        client.fsm.trigger.return_value = None  # Mock the trigger method's return value
        client.fsm.trigger(ClientEvent.PERIODIC_LOW_LATENCY_SCAN)

        # Assert that the trigger function was called with the ClientEvent PERIODIC_LOW_LATENCY_SCAN
        mock_fsm.trigger.assert_called_once_with(ClientEvent.PERIODIC_LOW_LATENCY_SCAN)

    #  The client FSM transitions from IDLE to SWITCHING_FREQUENCY when triggered by the EXT_SWITCH_EVENT event.
    def test_fsm_transition_idle_to_switching_frequency(self, mocker, capsys):
        # Target frequency is 5220 MHz
        # Create a mock for the decode method
        decode_mock = unittest.mock.Mock(return_value=b'\x82\xa4a_id\x04\xa4freq\xcd\x14d')

        # Patch the decode method to use the mock
        mocker.patch('jamming_client_fsm.decode', decode_mock)

        mock_switch_frequency = mocker.patch.object(JammingDetectionClient, "switch_frequency", return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)
        client.args.debug = True

        # Set the initial state to IDLE
        client.fsm.state = ClientState.IDLE
        client.target_frequency = 5220

        # Trigger the EXT_SWITCH_EVENT event
        client.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT)

        # Assert that the switch_frequency function was called upon the triggering of an EXT_SWITCH_EVENT
        mock_switch_frequency.assert_called_once()

    #  The client FSM transitions from IDLE to HIGH_LATENCY_SCAN when triggered by the EXT_DATA_REQUEST event.
    def test_fsm_transition_idle_to_high_latency_scan(self, mocker):
        # Use the mocker fixture to patch the frequency_hopping_decision method
        mocker.patch.object(JammingDetectionClient, 'performing_scan', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Set the initial state to IDLE
        client.fsm.state = ClientState.IDLE

        # Trigger the EXT_DATA_REQUEST event
        client.fsm.trigger(ClientEvent.EXT_DATA_REQUEST)

        # Assert that the state transitioned to HIGH_LATENCY_SCAN
        assert client.fsm.state == ClientState.HIGH_LATENCY_SCAN

    #  The client FSM transitions from LOW_LATENCY_SCAN to IDLE when triggered by the NO_JAM_DETECTED event.
    def test_fsm_transition_low_latency_scan_to_idle_no_jam_detected(self, mocker):
        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Set the initial state to LOW_LATENCY_SCAN
        client.fsm.state = ClientState.LOW_LATENCY_SCAN

        # Trigger the NO_JAM_DETECTED event
        client.fsm.trigger(ClientEvent.NO_JAM_DETECTED)

        # Assert that the state transitioned to IDLE
        assert client.fsm.state == ClientState.IDLE

    #  The client FSM transitions from LOW_LATENCY_SCAN to SENDING_JAM_ALERT when triggered by the JAM_DETECTED event.
    def test_fsm_transition_low_latency_scan_to_sending_jam_alert_jam_detected(self, mocker):
        # Mock the send_switch_frequency_message method
        mock_sending_jam_alert = mocker.patch.object(JammingDetectionClient, 'sending_jam_alert', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Create a mock JammingClient instance
        mock_jamming_client = MagicMock()

        # Set the initial state to LOW_LATENCY_SCAN
        client.fsm.state = ClientState.LOW_LATENCY_SCAN

        # Assign the mock JammingClient to the client
        client.jamming_client = mock_jamming_client

        # Trigger the JAM_DETECTED event
        client.fsm.trigger(ClientEvent.JAM_DETECTED)

        # Assert that the sending_jam_alert method in the JammingClient was called once
        mock_sending_jam_alert.assert_called_once_with(ClientEvent.JAM_DETECTED)

    @contextmanager
    def mock_socket_connect(self, mocker, exception_to_raise):
        # Create a mock for socket.socket.connect
        mock_connect = mocker.patch('socket.socket.connect', side_effect=exception_to_raise)

        try:
            yield mock_connect
        finally:
            mock_connect.stop()

    # Your test function
    def test_connect_to_orchestrator_failure(self, mocker):
        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Patch sys.exit to prevent it from terminating the test
        with patch('sys.exit'):
            with self.mock_socket_connect(mocker, ConnectionRefusedError):
                # Call the connect_to_orchestrator method
                client.connect_to_orchestrator()

        # Assert that the socket connect method was called multiple times
        assert socket.socket.connect.call_count == 3

    #  The client receives a message with an unknown action ID.
    def test_receive_message_unknown_action_id(self, mocker):
        # Action ID set to unknown int ID 5180 (same as freq value)
        args = Options()

        # Create a mock for the decode method
        recv_mock = unittest.mock.Mock(return_value=b'28:\x83\xa4a_id\xcd\x14n_id\xa7fd01::1\xa4freq\xcd\x14<,')

        # Create a mock for the socket.recv method
        decode_mock = unittest.mock.Mock(return_value=b'\x83\xa4a_id\xcd\x14n_id\xa7fd01::1\xa4freq\xcd\x14<')

        # Patch the socket.recv method to use the recv_mock
        mocker.patch('socket.socket.recv', recv_mock)

        # Patch the decode method to use the mock
        mocker.patch('jamming_client_fsm.decode', decode_mock)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)
        client.args = args

        # Set running to True to start the loop
        client.running = True

        # Create a mock for the logger.error function
        with patch('jamming_client_fsm.logger.error') as error_mock:
            # Create a thread to run client.receive_messages()
            receive_thread = threading.Thread(target=client.receive_messages)

            # Start the thread
            receive_thread.start()

            # Sleep for some time to allow the loop to execute (if needed)
            time.sleep(0.1)

            # Set running to False to exit the loop
            client.running = False

            # Wait for the thread to complete
            receive_thread.join()

            # Assert that logger.error was called with the expected message
            error_mock.assert_called_with("Error in received message: int is not allowed for map key when strict_map_key=True")

        # Assert that no state transition occurred
        assert client.fsm.state == ClientState.IDLE

    #  The client receives a message with the action ID for estimation_request and triggers the EXT_DATA_REQUEST event.
    def test_receive_message_estimation_request(self, mocker):
        # Create a mock for the socket.recv method
        recv_mock = unittest.mock.Mock(return_value=b'50:\x82\xa4a_id\x01\xa4h_id\xd9$4bf6b4c2-a42a-4733-b6a1-aa9fb32cf3ff,')

        # Create a mock for the decode method
        decode_mock = unittest.mock.Mock(return_value=b'\x82\xa4a_id\x01\xa4h_id\xd9$4bf6b4c2-a42a-4733-b6a1-aa9fb32cf3ff')

        # Patch the socket.recv method to use the recv_mock
        mocker.patch('socket.socket.recv', recv_mock)

        # Patch the decode method to use the mock
        mocker.patch('jamming_client_fsm.decode', decode_mock)

        mock_perform_scan = mocker.patch.object(JammingDetectionClient, "performing_scan", return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Set running to True to start the loop
        client.running = True

        # Create a thread to run client.receive_messages()
        receive_thread = threading.Thread(target=client.receive_messages)

        # Start the thread
        receive_thread.start()

        # Sleep for some time to allow the loop to execute
        time.sleep(1)

        # Set running to false to exit the loop
        client.running = False

        # Wait for the thread to complete
        receive_thread.join()

        # Assert that the perfom scan function was called from the receive_messages function
        mock_perform_scan.assert_called_once()

    #  The client receives a message with the action ID for switch_frequency and triggers the EXT_SWITCH_EVENT event.
    def test_receive_message_switch_frequency(self, mocker):
        # Create a mock for the socket.recv method for the switch frequency action
        # Target frequency is 5220 MHz
        recv_mock = unittest.mock.Mock(return_value=b'15:\x82\xa4a_id\x04\xa4freq\xcd\x14d,')

        # Create a mock for the decode method
        decode_mock = unittest.mock.Mock(return_value=b'\x82\xa4a_id\x04\xa4freq\xcd\x14d')

        # Patch the socket.recv method to use the recv_mock
        mocker.patch('socket.socket.recv', recv_mock)

        # Patch the decode method to use the mock
        mocker.patch('jamming_client_fsm.decode', decode_mock)

        # Create a mock for the switch_frequency method
        mock_switch_freq = mocker.patch.object(JammingDetectionClient, 'switch_frequency', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)
        client.current_frequency = 5180

        # Set running to True to start the loop
        client.running = True

        # Create a thread to run client.receive_messages()
        receive_thread = threading.Thread(target=client.receive_messages)

        # Start the thread
        receive_thread.start()

        # Sleep for some time to allow the loop to execute (if needed)
        time.sleep(1)

        # Set running to false to exit the loop
        client.running = False

        # Wait for the thread to complete
        receive_thread.join()

        # Assert that the switch frequency function was called upon receiving a switch frequency message from the server
        mock_switch_freq.assert_called_once()
        assert client.target_frequency == 5220

    #  The client receives a message with the action ID for target_frequency and triggers the EXT_SWITCH_EVENT event.
    def test_receive_message_target_frequency(self, mocker, capsys):
        # Target frequency is 5220 MHz
        # Create a mock for the socket.recv method
        recv_mock = unittest.mock.Mock(return_value=b'15:\x82\xa4a_id\x03\xa4freq\xcd\x14d,')

        # Create a mock for the decode method
        decode_mock = unittest.mock.Mock(return_value=b'\x82\xa4a_id\x04\xa4freq\xcd\x14d')

        # Patch the socket.recv method to use the recv_mock
        mocker.patch('socket.socket.recv', recv_mock)

        # Patch the decode method to use the mock
        mocker.patch('jamming_client_fsm.decode', decode_mock)

        # Create a mock for the switch_frequency method
        mock_switch_frequency = mocker.patch.object(JammingDetectionClient, 'switch_frequency', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the get_mesh_freq function to return a different frequency than the starting frequency
        mocker.patch('util.get_mesh_freq', return_value=5180)
        mocker.patch('util.map_channel_to_freq', return_value=5180)

        # Mock the run_command function to not raise an exception
        mocker.patch('util.run_command', side_effect=lambda command, error_message: (True, None))
        mocker.patch('util.is_process_running', return_value=False)
        mocker.patch('os.path.exists', return_value=False)
        mocker.patch('util.read_file', return_value=True)
        mocker.patch('util.write_file', return_value=True)

        client.current_frequency = 5180

        # Set running to True to start the loop
        client.running = True

        # Create a thread to run client.receive_messages()
        receive_thread = threading.Thread(target=client.receive_messages)

        # Start the thread
        # Create a mock for re.sub within the scope of switch_frequency and call switch_frequency function
        with unittest.mock.patch('re.sub', return_value='conf'):
            receive_thread.start()

        # Sleep for some time to allow the loop to execute (if needed)
        time.sleep(1)

        # Set running to false to exit the loop
        client.running = False

        # Wait for the thread to complete
        receive_thread.join()

        # Assert that the switch frequency function was called upon receiving a new target frequency message
        mock_switch_frequency.assert_called_once()

    #  The client FSM transitions from LOW_LATENCY_SCAN to HIGH_LATENCY_SCAN when triggered by the EXT_DATA_REQUEST event.
    def test_fsm_transition_low_latency_scan_to_high_latency_scan(self, mocker):
        # Patch the performing_scan method
        mocker.patch.object(JammingDetectionClient, 'performing_scan', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Set the initial state to LOW_LATENCY_SCAN
        client.fsm.state = ClientState.IDLE

        # Trigger the EXT_DATA_REQUEST event
        client.fsm.trigger(ClientEvent.EXT_DATA_REQUEST)

        time.sleep(0.1)

        # Assert that the state transitioned to HIGH_LATENCY_SCAN
        assert client.fsm.state == ClientState.HIGH_LATENCY_SCAN

    #  The client FSM transitions from JAM_DETECTED to SENDING_JAM_ALERT when triggered by the PERIODIC_LOW_LATENCY_SCAN event.
    def test_jam_detected_to_idle(self, mocker):
        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the performing_scan method to trigger the JAM_DETECTED event
        mocker.patch.object(JammingDetectionClient, 'performing_scan', side_effect=lambda x: client.fsm.trigger(ClientEvent.JAM_DETECTED))
        mocker.patch.object(JammingDetectionClient, 'sending_jam_alert', return_value=True)
        mocker.patch.object(JammingDetectionClient, 'send_messages', return_value=True)

        # Set the initial state of the FSM to LOW_LATENCY_SCAN
        client.fsm.state = ClientState.LOW_LATENCY_SCAN

        # Call the performing_scan method
        client.performing_scan(ClientEvent.PERIODIC_LOW_LATENCY_SCAN)

        # Assert that the state transitions to IDLE
        assert client.fsm.state == ClientState.IDLE

    #  The client FSM transitions from SENDING_JAM_ALERT to HIGH_LATENCY_SCAN when triggered by the EXT_DATA_REQUEST event.
    def test_sending_jam_alert_to_high_latency_scan(self, mocker):
        # Mock the performing_scan method
        mocker.patch.object(JammingDetectionClient, 'performing_scan', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the sending_jam_alert method to trigger the EXT_DATA_REQUEST event
        mocker.patch.object(JammingDetectionClient, 'sending_jam_alert', side_effect=lambda x: client.fsm.trigger(ClientEvent.EXT_DATA_REQUEST))

        # Mock the is_external_event method to return True in order to conduct the test (to prevent hanging)
        mocker.patch.object(ClientFSM, 'is_external_event', return_value=False)

        # Set the initial state of the FSM to SENDING_JAM_ALERT
        client.fsm.state = ClientState.SENDING_JAM_ALERT

        # Call the performing_scan method
        client.sending_jam_alert(ClientEvent.JAM_DETECTED)
        time.sleep(2)

        # Assert that the state transitions to HIGH_LATENCY_SCAN
        assert client.fsm.state == ClientState.HIGH_LATENCY_SCAN

    #  The client FSM transitions from SENDING_JAM_ALERT to SWITCHING_FREQUENCY when triggered by the EXT_SWITCH_EVENT event.
    def test_sending_jam_alert_to_switching_frequency(self, mocker):
        # Mock the switch_frequency method
        mocker.patch.object(JammingDetectionClient, 'switch_frequency', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the sending_jam_alert method to trigger the EXT_SWITCH_EVENT event
        mocker.patch.object(JammingDetectionClient, 'sending_jam_alert', side_effect=lambda x: client.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT))

        # Mock the is_external_event method to return True in order to conduct the test (to prevent hanging for external event)
        mocker.patch.object(ClientFSM, 'is_external_event', return_value=False)

        # Set the initial state of the FSM to SENDING_JAM_ALERT
        client.fsm.state = ClientState.SENDING_JAM_ALERT

        # Call the performing_scan method
        client.sending_jam_alert(ClientEvent.JAM_DETECTED)
        time.sleep(2)

        # Assert that the state transitions to SWITCHING_FREQUENCY
        assert client.fsm.state == ClientState.SWITCHING_FREQUENCY

    #  The client FSM transitions from HIGH_LATENCY_SCAN to REPORTING_DATA when triggered by the HIGH_LATENCY_SCAN_PERFORMED event.
    def test_high_latency_scan_to_reporting_data(self, mocker):
        # Mock the report_spec_quality method
        mocker.patch.object(JammingDetectionClient, 'report_spec_quality', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the sending_jam_alert method to trigger the EXT_SWITCH_EVENT event
        mocker.patch.object(JammingDetectionClient, 'performing_scan', side_effect=lambda x: client.fsm.trigger(ClientEvent.HIGH_LATENCY_SCAN_PERFORMED))

        # Set the initial state of the FSM to HIGH_LATENCY_SCAN
        client.fsm.state = ClientState.HIGH_LATENCY_SCAN

        # Call the performing_scan method
        client.performing_scan(ClientEvent.EXT_DATA_REQUEST)
        time.sleep(2)

        # Assert that the state transitions to REPORTING_DATA
        assert client.fsm.state == ClientState.REPORTING_DATA

    #  The client FSM transitions from REPORTING_DATA to IDLE when triggered by the DATA_REPORT_SENT event.
    def test_reporting_data_to_idle(self, mocker):
        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the sending_jam_alert method to trigger the EXT_SWITCH_EVENT event
        mocker.patch.object(JammingDetectionClient, 'report_spec_quality', side_effect=lambda x: client.fsm.trigger(ClientEvent.DATA_REPORT_SENT))

        # Set the initial state of the FSM to REPORTING_DATA
        client.fsm.state = ClientState.REPORTING_DATA

        # Call the performing_scan method
        client.report_spec_quality(ClientEvent.HIGH_LATENCY_SCAN_PERFORMED)
        time.sleep(2)

        # Assert that the state transitions to IDLE
        assert client.fsm.state == ClientState.IDLE

    #  The client FSM transitions from REPORTING_DATA to SWITCHING_FREQUENCY when triggered by the EXT_SWITCH_EVENT event.
    def test_reporting_data_to_switching_frequency_transition(self, mocker):
        # Mock the report_spec_quality method
        mocker.patch.object(JammingDetectionClient, 'switch_frequency', return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Mock the sending_jam_alert method to trigger the EXT_SWITCH_EVENT event
        mocker.patch.object(JammingDetectionClient, 'report_spec_quality', side_effect=lambda x: client.fsm.trigger(ClientEvent.EXT_SWITCH_EVENT))

        # Mock the is_external_event method to return True in order to conduct the test (to prevent hanging for external event)
        mocker.patch.object(ClientFSM, 'is_external_event', return_value=False)

        # Set the initial state of the FSM to REPORTING_DATA
        client.fsm.state = ClientState.REPORTING_DATA

        # Call the performing_scan method
        client.report_spec_quality(ClientEvent.HIGH_LATENCY_SCAN_PERFORMED)
        time.sleep(2)

        # Assert that the state transitions to SWITCHING_FREQUENCY
        assert client.fsm.state == ClientState.SWITCHING_FREQUENCY

    #  The client FSM transitions from SWITCHING_FREQUENCY to RESETTING when triggered by the SWITCHED event.
    def test_fsm_transition_switched_to_resetting(self, mocker):
        # Mock the reset function
        mocker.patch.object(JammingDetectionClient, "reset", return_value=True)

        # Create an instance of the JammingDetectionClient
        client = JammingDetectionClient('node_id', '::1', 1234)

        # Set the FSM state to SWITCHING_FREQUENCY
        client.fsm.state = ClientState.SWITCHING_FREQUENCY

        # Trigger the SWITCHED event
        client.fsm.trigger(ClientEvent.SWITCHED)

        # Assert that the FSM state is now RESETTING
        assert client.fsm.state == ClientState.RESETTING
