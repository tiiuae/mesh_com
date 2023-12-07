import pytest
import subprocess
from unittest import mock

import add_syspath
import util
from jamming_setup import is_interface_up, switch_frequency, check_client_osf_connectivity, check_osf_interface, start_jamming_scripts, start_client, start_server, validate_configuration, main
from options import Options


class TestCodeUnderTest:
    """
    A suite of test cases for the code.
    """

    #  Mesh frequency is switched to starting frequency successfully
    def test_switch_frequency_success(self, mocker):
        # Create an Options object with debug set to True
        args = Options()
        args.starting_channel = 36
        args.debug = True

        # Mock the get_mesh_freq function to return a different frequency than the starting frequency
        mocker.patch('util.get_mesh_freq', return_value=5180)
        mocker.patch('util.map_channel_to_freq', return_value=5180)

        # Mock the run_command function to not raise an exception
        mocker.patch('util.switch_frequency', return_value=True)

        # Run function under test, no exceptions should be returned
        switch_frequency(args)

    #  IPv6 address of tun0 interface is retrieved successfully
    def test_get_ipv6_address_success(self, mocker):
        # Mock the get_ipv6_addr function to return a valid IPv6 address
        mocker.patch('util.get_ipv6_addr', return_value='2001:db8::1')

        # Call the function under test
        ipv6_address = util.get_ipv6_addr(Options().osf_interface)

        # Assert that the returned IPv6 address is correct
        assert ipv6_address == '2001:db8::1'

    #  IPv6 client connectivity with remote server is successful
    def test_check_client_osf_connectivity_success(self, mocker):
        # Mock the subprocess.check_output function to not raise an exception
        mocker.patch('subprocess.check_output')

        # Call the function under test
        check_client_osf_connectivity(Options())

        # Assert that the subprocess.check_output function was called with the correct command
        subprocess.check_output.assert_called_with(['ping6', '-c', '1', Options().jamming_osf_orchestrator], text=True)

    #  Jamming-related scripts are started successfully
    def test_start_jamming_scripts_success(self, mocker):
        args = Options()
        args.jamming_osf_orchestrator = '2000:db1::2'

        # Mock the run_command function to not raise an exception
        mock_run_command = mocker.patch('util.run_command', side_effect=lambda command, error_message: (True, None))
        mocker.patch('util.kill_process_by_pid', return_value=True)

        # Call the function under test
        start_jamming_scripts(args, '2001:db8::1')

        # Assert that the run_command function was called with the correct command
        mock_run_command.assert_called_with(["python", "jamming_client_fsm.py"], 'Failed to run jamming_client_fsm file')


    #  IPv6 connectivity with remote server fails
    def test_check_client_osf_connectivity_failure(self, mocker):
        # Mock the subprocess.check_output function to raise a subprocess.CalledProcessError
        mocker.patch('subprocess.check_output', side_effect=subprocess.CalledProcessError(1, 'ping6'))

        # Call the function under test and assert that it raises a SystemExit
        with pytest.raises(SystemExit):
            check_client_osf_connectivity(Options())

    #  is_interface_up function returns True if interface is up
    def test_is_interface_up_returns_true_if_interface_is_up(self, mocker):
        # Mock the netifaces.interfaces function to return a list of available interfaces
        mocker.patch('netifaces.interfaces', return_value=['eth0', 'wlp1s0', 'tun0'])

        # Call the function under test
        result = is_interface_up('wlp1s0')

        # Assert that the result is True
        assert result is True

    #  Validate configuration fails when channels5 list includes invalid channels
    def test_validate_configuration_fails_channels5_list_includes_invalid_channels(self, mocker):
        args = Options()
        args.channels5 = [36, 40, 44, 48, 149, 153, 157, 1600]
        assert validate_configuration(args) == False

    #  Validate configuration succeeds when channels5 list includes valid channels
    def test_validate_configuration_succeeds_channels5_list_includes_valid_channels(self, mocker):
        args = Options()
        args.channels5 = [36, 40, 44, 48, 149, 153, 157, 161]
        assert validate_configuration(args) == True