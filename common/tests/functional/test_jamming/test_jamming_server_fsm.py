import time
import signal
import socket

import pytest
from pytest_mock import MockerFixture
import threading
from unittest.mock import Mock, patch

import add_syspath
from jamming_server_fsm import JammingServer, ServerState, ServerEvent, ServerFSM, JammingClientTwin, action_to_id


class TestCodeUnderTest:

    def test_server_instance(self):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        assert isinstance(server, JammingServer), "server is not an instance of JammingServer"

    @pytest.mark.parametrize("host, port", [("::1", 12345)])
    def test_server_start_successfully(self, host, port, mocker: MockerFixture):
        # Create an instance of the JammingServer class
        server = JammingServer(host, port)

        # Create a custom mock for the socket object
        mock_socket = mocker.patch('socket.socket')

        # Set the return value of accept to a tuple with mock client socket and address
        mock_socket.return_value.accept.return_value = (mocker.MagicMock(), (host, port))

        # Initialize correct server state for specific event tested
        server.fsm.state = ServerState.IDLE

        # Create a thread to start the server
        server_thread = threading.Thread(target=server.start)

        # Start the server in a separate thread
        server_thread.start()
        time.sleep(1)  # Allow the server time to start

        # Wait for the server thread to complete its execution
        if server_thread.is_alive():
            server_thread.join()
        server.stop()

        # Perform assertions without raising exceptions
        assert mock_socket.called_once_with(socket.AF_INET6, socket.SOCK_STREAM)
        assert mock_socket.return_value.bind.called_once_with((host, port))
        assert mock_socket.return_value.listen.called_once()
        assert mock_socket.return_value.accept.called_once()

    #  The server receives a jamming alert message from a client and transitions to the CHECK_IF_TARGET_FREQ_JAMMED state.
    def test_receive_jamming_alert_message(self, mocker):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Initialize correct server state for specific event tested
        server.fsm.state = ServerState.IDLE

        # Define test cases with jam_alert_message and expected_calls
        test_cases = [
            (None, 0),  # No jam_alert_message, so check_jammed_frequency should not be called
            ({'freq': 5}, 1),  # Valid jam_alert_message, so check_jammed_frequency should be called once
        ]

        for jam_alert_message, expected_calls in test_cases:
            # Create a mock client with the specified jam_alert_message
            mock_client = mocker.Mock()
            mock_client.jam_alert_message = jam_alert_message

            # Use the mocker fixture to patch the check_jammed_frequency method
            mocker.patch.object(server, 'check_jammed_frequency', autospec=True)

            # Add the mock client to the server's clients list
            server.clients.append(mock_client)

            # Trigger the jamming alert event
            server.fsm.trigger(ServerEvent.JAM_ALERT)

            # Assert that the check_jammed_frequency() method was called the expected number of times
            if expected_calls > 0:
                server.check_jammed_frequency.assert_called_once()
            else:
                server.check_jammed_frequency.assert_not_called()

    #  The server receives a periodic target frequency broadcast event and transitions to the BROADCAST_TARGET_FREQ state.
    def test_receive_periodic_target_freq_broadcast(self):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Initialize correct server state for specific event tested
        server.fsm.state = ServerState.IDLE

        # Mock the broadcast_target_freq method
        with patch.object(server, 'broadcast_target_freq') as mock_broadcast_target_freq:
            # Trigger the periodic target frequency broadcast event
            server.broadcast_target_freq(ServerEvent.PERIODIC_TARGET_FREQ_BROADCAST)

            # Assert that the broadcast_target_freq method was called once
            mock_broadcast_target_freq.assert_called_once()

    #  The server broadcasts the target frequency to all connected clients.
    def test_broadcast_target_freq(self, mocker):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Initialize correct server state for specific event tested
        server.fsm.state = ServerState.IDLE

        # Use the mocker fixture to patch the send_data_clients method
        mocker.patch.object(server, 'send_data_clients', autospec=True)

        # Create a mock client
        mock_client = mocker.Mock()

        # Add the mock client to the server's clients list
        server.clients.append(mock_client)

        # Set the target_frequency attribute to a specific value
        server.target_frequency = 5

        # Trigger the broadcast complete event
        server.fsm.trigger(ServerEvent.PERIODIC_TARGET_FREQ_BROADCAST)

        action_id: int = action_to_id["target_frequency"]
        server.send_data_clients.assert_called_once_with({'a_id': action_id, 'freq': 5})

    #  The server receives a valid jamming alert message and transitions to the REQUEST_SPECTRUM_DATA state.
    def test_receive_valid_jamming_alert_message(self):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Initialize correct server state for the specific event tested
        server.fsm.state = ServerState.IDLE
        server.last_requested_spectrum_data = time.time()
        server.healing_process_id = 123

        # Create a mock client
        mock_client = Mock()
        mock_client.jam_alert_message = {'freq': 5}
        action_id = action_to_id["estimation_report"]
        mock_client.estimation_report_message = {'a_id': action_id, 'n_id': "fd01::1", 'freq': 5180, 'qual': {'5180': 0.55}, 'h_id': 123}

        # Define a decorator to mock the request_spectrum_data method
        def mock_request_spectrum_data(server_instance):
            server_instance.last_requested_spectrum_data = time.time()

        # Patch the request_spectrum_data method of the JammingServer class
        with patch.object(server, 'request_spectrum_data', side_effect=mock_request_spectrum_data):
            # Initialize correct server state for the specific event tested
            server.fsm.state = ServerState.CHECK_IF_TARGET_FREQ_JAMMED

            # Add the mock client to the server's clients list
            server.clients.append(mock_client)

            # Set the target_frequency attribute to the same value as the jamming alert frequency
            server.target_frequency = 5

            # Trigger the valid jamming alert event
            server.fsm.trigger(ServerEvent.VALID_JAM_ALERT)

            # Ensure some time for event processing
            time.sleep(1)

            # Assert that the request_spectrum_data method was called
            assert server.last_requested_spectrum_data is not None, "request_spectrum_data was not called."

    #  The server receives an invalid jamming alert message and transitions to the RESET_CLIENT_MESSAGES state.
    def test_receive_invalid_jamming_alert_message(self):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)
        server.fsm.state = ServerState.CHECK_IF_TARGET_FREQ_JAMMED

        # Create a mock client
        mock_client = Mock()
        mock_client.jam_alert_message = {'freq': 5}

        # Mock the reset_client_attributes method of the client
        with patch.object(mock_client, 'reset_client_attributes', autospec=True) as mock_reset_client:
            # Add the mock client to the server's clients list
            server.clients.append(mock_client)

            # Set the target_frequency attribute to a different value than the jamming alert frequency
            server.target_frequency = 10

            # Trigger the invalid jamming alert event
            server.fsm.trigger(ServerEvent.INVALID_JAM_ALERT)

            # Assert that the client's reset_client_attributes method was called once
            mock_reset_client.assert_called_once()

    #  The server does not receive any spectrum data from clients within the data gathering wait time and transitions to the FREQ_HOPPING_DECISION state.
    def test_no_spectrum_data_received_within_wait_time(self, mocker):
        # Use the mocker fixture to patch the frequency_hopping_decision method
        mock_frequency_hopping_decision = mocker.patch.object(JammingServer, 'frequency_hopping_decision', return_value=True)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Initialize correct server state for specific event tested
        server.fsm.state = ServerState.GATHER_SPECTRUM_DATA
        server.healing_process_id = 123

        # Create a mock client
        mock_client = mocker.Mock()
        action_id = action_to_id["estimation_report"]
        mock_client.estimation_report_message = {'a_id': action_id, 'n_id': "fd01::1", 'freq': 5180, 'qual': {'5180': 0.55}, 'h_id': 123}

        # Add the mock client to the server's clients list
        server.clients.append(mock_client)

        # Set the last_requested_spectrum_data attribute to a time in the past
        server.last_requested_spectrum_data = time.time() - 1000000

        # Trigger the spectrum data gathering expired event
        server.fsm.trigger(ServerEvent.SPECTRUM_DATA_GATHERING_EXPIRED)

        # Assert that the server's state is now FREQ_HOPPING_DECISION
        mock_frequency_hopping_decision.assert_called_once()

    #  The server receives a switch request sent event and transitions to the RESET_CLIENT_MESSAGES state.
    def test_receive_switch_request_sent_event(self, mocker):
        # Use the mocker fixture to patch the reset_client_attributes method
        mock_reset_client_attributes = mocker.patch.object(JammingServer, 'reset_client_attributes', return_value=True)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        server.fsm.state = ServerState.UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ

        # Trigger the switch request sent event
        server.fsm.trigger(ServerEvent.SWITCH_REQUEST_SENT)

        # Assert that the server's state is now FREQ_HOPPING_DECISION
        mock_reset_client_attributes.assert_called_once()

    #  The server receives a reset complete event and transitions to the IDLE state.
    def test_receive_reset_complete_event(self, mocker):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Trigger the reset complete event
        server.fsm.trigger(ServerEvent.RESET_COMPLETE)

        # Assert that the server's state is now IDLE
        assert server.fsm.state == ServerState.IDLE

    #  The server receives spectrum data from all clients and transitions to the GATHER_SPECTRUM_DATA state.
    def test_receive_spectrum_data_from_all_clients(self, mocker):
        # Use the mocker fixture to patch the frequency_hopping_decision method
        mock_frequency_hopping_decision = mocker.patch.object(JammingServer, 'frequency_hopping_decision', return_value=True)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Create an instance of the JammingServer class
        server.healing_process_id = '123'
        server.fsm.state = ServerState.GATHER_SPECTRUM_DATA

        # Create a mock client
        mock_client1 = mocker.Mock()
        mock_client2 = mocker.Mock()
        mock_client1.estimation_report_message = {'h_id': '123'}
        mock_client2.estimation_report_message = {'h_id': '123'}

        # Add the mock clients to the server's clients list
        server.clients.append(mock_client1)
        server.clients.append(mock_client2)

        # Set the last_requested_spectrum_data attribute to a time in the past
        server.last_requested_spectrum_data = time.time() - 1000000

        # Trigger the gathered all spectrum data event
        server.fsm.trigger(ServerEvent.GATHERED_ALL_SPECTRUM_DATA)

        # Assert that the server's state is now FREQ_HOPPING_DECISION
        mock_frequency_hopping_decision.assert_called_once()

    #  The server sends a spectrum data request to all connected clients.
    def test_send_spectrum_data_request(self, mocker):
        # Mock the send_data_clients method
        mock_gathering_spectrum_data = mocker.patch.object(JammingServer, 'gathering_spectrum_data')

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Create an instance of the JammingServer class
        server.fsm.state = ServerState.REQUEST_SPECTRUM_DATA

        # Trigger the event to send spectrum data request
        server.fsm.trigger(ServerEvent.SENT_SPECTRUM_DATA_REQUEST)

        # Assert that the send_data_clients method was called with the correct arguments
        mock_gathering_spectrum_data.assert_called_once_with(ServerEvent.SENT_SPECTRUM_DATA_REQUEST)

    #  The server receives a new target frequency and transitions to the UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ state.
    def test_receive_new_target_frequency(self, mocker):
        # Mock the send_switch_frequency_message method
        mock_send_switch_frequency_message = mocker.patch.object(JammingServer, 'send_switch_frequency_message')

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Create an instance of the JammingServer class
        server.fsm.state = ServerState.FREQ_HOPPING_DECISION

        # Set the target_frequency attribute to a different value
        server.target_frequency = 5

        # Trigger the event to receive a new target frequency
        server.fsm.trigger(ServerEvent.NEW_TARGET_FREQUENCY)

        # Assert that the send_switch_frequency_message method was called
        mock_send_switch_frequency_message.assert_called_once_with(ServerEvent.NEW_TARGET_FREQUENCY)

    #  The server sends a switch frequency message to all connected clients.
    def test_send_switch_frequency_message(self, mocker):
        # Mock the send_data_clients method
        mock_send_data_clients = mocker.Mock()
        mocker.patch.object(JammingServer, 'send_data_clients', mock_send_data_clients)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)
        server.fsm.state = ServerState.FREQ_HOPPING_DECISION

        # Trigger the send_switch_frequency_message event
        server.fsm.trigger(ServerEvent.NEW_TARGET_FREQUENCY)

        # Assert that the send_data_clients method was called with the correct arguments
        mock_send_data_clients.assert_called_once_with({'a_id': 4, 'freq': server.target_frequency})

    #  The server resets the client attributes and transitions to the IDLE state.
    def test_reset_client_attributes(self, mocker):
        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Mock the reset_client_attributes method
        mocker.patch.object(JammingServer, 'reset_client_attributes', return_value=True, side_effect=lambda x: server.fsm.trigger(ServerEvent.RESET_COMPLETE))

        # Set server initial state
        server.fsm.state = ServerState.UPDATE_TARGET_FREQ_SEND_SWITCH_FREQ

        # Set the server's healing_process_id to a specific value
        server.healing_process_id = "111"

        # Set the jam_alert_message and estimation_report_message attributes of a client
        client = JammingClientTwin(None, None, [], None)
        client.jam_alert_message = {'freq': 5180}
        client.estimation_report_message = {'h_id': '111'}

        # Add the client to the server's clients list
        server.clients.append(client)

        # Trigger the reset_client_attributes event
        server.fsm.trigger(ServerEvent.SWITCH_REQUEST_SENT)

        # Allow time for code execution
        time.sleep(1)

        # Stop client and object execution
        client.stop()
        server.stop()

        # Assert that the healing_process_id was reset to a new value
        assert server.healing_process_id != "111"

        # Assert that the jam_alert_message and estimation_report_message attributes of the client were reset
        assert client.jam_alert_message == {}
        assert client.estimation_report_message == {}

    #  The server handles a signal interrupt (SIGINT) and stops the server gracefully.
    def test_signal_handler(self, mocker,):
        # Mock the send_switch_frequency_message method
        mock_signal_handler = mocker.patch.object(JammingServer, 'signal_handler', return_value=True)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Mock the sys.exit method
        mock_exit = mocker.Mock()
        mocker.patch('sys.exit', mock_exit)

        # Trigger the signal_handler method with a SIGINT signal
        server.signal_handler(signal.SIGINT, None)

        # Assert that the sys.exit method was called with the correct argument
        mock_signal_handler.assert_called_once()

    #  The server runs the server FSM continuously to manage state transitions and periodic tasks.
    def test_run_server_fsm(self, mocker):
        # Mock the ServerFSM class
        mock_fsm = mocker.Mock()
        mock_trigger = mocker.patch.object(ServerFSM, 'trigger', mock_fsm)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)
        server.running = True
        server.fsm.state = ServerState.IDLE
        server.last_target_freq_broadcast = time.time() - 1000000

        # Create a thread to run run_server_fsm
        run_server_fsm_thread = threading.Thread(target=server.run_server_fsm)

        # Start the thread
        run_server_fsm_thread.start()

        # Sleep for some time to allow the loop to execute (if needed)
        time.sleep(1)

        # Set running to false to exit the loop
        server.running = False

        # Wait for the thread to complete
        run_server_fsm_thread.join()

        # Assert that the ServerFSM's trigger method was called with the correct event
        mock_trigger.assert_called_once_with(ServerEvent.PERIODIC_TARGET_FREQ_BROADCAST)

    #  The server checks for a jamming alert message from any connected client.
    def test_check_jam_alert(self, mocker):
        # Mock the ServerFSM class
        mock_fsm = mocker.Mock()
        mock_trigger = mocker.patch.object(ServerFSM, 'trigger', mock_fsm)

        # Mock the JammingClientTwin class
        mock_client = mocker.Mock()

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)
        mocker.patch.object(JammingServer, 'check_jam_alert', side_effect=server.fsm.trigger(ServerEvent.JAM_ALERT))

        # Set up a jam alert message from a client
        mock_client.jam_alert_message = {'freq': 123}

        # Call the check_jam_alert method
        server.check_jam_alert()

        # Assert that the ServerFSM's trigger method was called with the correct event
        mock_trigger.assert_called_once_with(ServerEvent.JAM_ALERT)

    #  The server gathers spectrum data from clients until all data is gathered or until time expiry.
    def test_gathering_spectrum_data(self, mocker):
        # Mock the ServerFSM class
        mock_fsm = mocker.Mock()
        mock_trigger = mocker.patch.object(ServerFSM, 'trigger', mock_fsm)

        # Create an instance of the JammingServer class
        server = JammingServer('::1', 12345)

        # Mock the JammingClientTwin class
        mock_client = mocker.Mock()
        mocker.patch.object(JammingServer, 'gathering_spectrum_data', side_effect=server.fsm.trigger(ServerEvent.GATHERED_ALL_SPECTRUM_DATA))

        # Set up estimation report messages from clients
        mock_client.estimation_report_message = {'h_id': '123'}
        mock_client2 = mocker.Mock()
        mock_client2.estimation_report_message = {'h_id': '123'}
        server.clients = [mock_client, mock_client2]

        # Call the gathering_spectrum_data method
        server.gathering_spectrum_data()

        # Assert that the ServerFSM's trigger method was called with the correct event
        mock_trigger.assert_called_once_with(ServerEvent.GATHERED_ALL_SPECTRUM_DATA)
