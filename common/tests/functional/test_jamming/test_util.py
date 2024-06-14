import math
import pathlib
import subprocess
from unittest import mock

import netifaces
import numpy as np
import pandas
import pytest

import add_syspath
from options import Options
from util import load_sample_data, get_mesh_freq, run_command, map_channel_to_freq, map_freq_to_channel, get_frequency_quality, get_ipv6_addr, read_file, write_file, \
    is_process_running, kill_process_by_pid


class TestCodeUnderTest:

    #  Load sample data successfully
    def test_load_sample_data_successfully(self, mocker):
        # Mock the random.choice function to return a specific sample type and file path
        mocker.patch('random.choice', side_effect=lambda x: x[0] if isinstance(x, list) else x)

        # Mock the pathlib.Path.glob method to return a list of file paths
        def mock_glob(self, pattern):
            return ['sample/floor/sample.csv']

        mocker.patch.object(pathlib.Path, 'glob', mock_glob)

        # Mock the pd.read_csv function to return a specific DataFrame
        mocker.patch('pandas.read_csv', return_value=pandas.DataFrame({'col1': [1, 2, 3], 'col2': [4, 5, 6]}))

        # Call the load_sample_data function
        message, data = load_sample_data()

        # Assert that the correct message and data are returned
        assert message == 'Floor data sample/floor/sample.csv'
        assert data.equals(pandas.DataFrame({'col1': [1, 2, 3], 'col2': [4, 5, 6]}))

    #  Map channel to frequency successfully
    def test_map_channel_to_frequency_successfully(self):
        # Call the map_channel_to_freq function with a valid channel number
        result = map_channel_to_freq(1)

        # Assert that the correct frequency is returned
        assert result == 2412

    #  Map frequency to channel successfully
    def test_map_frequency_to_channel_successfully(self):
        # Call the map_freq_to_channel function with a valid frequency
        result = map_freq_to_channel(2412)

        # Assert that the correct channel number is returned
        assert result == 1

    #  Get frequency quality information successfully
    def test_get_frequency_quality_information_successfully(self):
        # Create mock input arrays
        freq_quality = np.array([0.5, 0.8, 0.6])
        probs = np.array([[0.1, 0.2, 0.3, 0.4], [0.4, 0.3, 0.2, 0.1], [0.2, 0.3, 0.1, 0.4]])
        frequencies = np.array([2412, 2442, 5180])

        # Call the get_frequency_quality function
        jamming_detected, freq_quality_dict = get_frequency_quality(freq_quality, probs, frequencies)

        # Assert that the correct jamming detection and frequency quality dictionary are returned
        assert jamming_detected == True
        assert freq_quality_dict == {'2412': 0.5, '2442': 0.8, '5180': 0.6}

    #  No IPv6 address found for OSF network interface
    def test_no_ipv6_address_found_for_osf_interface(self, mocker):
        # Mock the netifaces.ifaddresses function to return an empty list for AF_INET6
        mocker.patch('netifaces.ifaddresses', return_value={netifaces.AF_INET6: []})

        # Call the get_ipv6_addr function with a mock OSF interface name
        result = get_ipv6_addr('osf_interface')

        # Assert that an empty string is returned
        assert result == None

    #  No mesh frequency found for device
    def test_no_mesh_frequency_found_for_device(self, mocker):
        # Mock the os.popen function to return a specific output
        mocker.patch('subprocess.check_output', return_value='Interface 1: mesh channel 36')

        # Call the get_mesh_freq function
        result = get_mesh_freq()

        # Assert that the result is NaN
        assert np.isnan(result)

    #  Invalid sample_type provided for load_sample_data function
    def test_invalid_sample_type_provided_for_load_sample_data(self, mocker):
        # Declare the args variable
        args = Options()

        # Define invalid return_value to observe raise error behaviour
        mocker.patch('random.choice', return_value='non_existant_sample_type')

        # Set args.debug = False specifically for the load_sample_data() function
        with mock.patch.object(args, "debug", False):
            # Call the load_sample_data function with an invalid sample type
            with pytest.raises(ValueError):
                load_sample_data(args)

    #  Error occurred while executing shell command in run_command function
    def test_error_occurred_while_executing_shell_command(self, mocker):
        # Call the run_command function and assert that it raises an exception
        with pytest.raises(Exception):
            run_command('command', 'Error message')

    #  Error occurred while reading a file in read_file function
    def test_error_occurred_while_reading_file(self, mocker):
        # Mock the open function to raise an exception
        mocker.patch('builtins.open', side_effect=Exception('Error reading file'))

        # Call the read_file function and assert that it returns None
        result = read_file('filename', 'Error message')
        assert result is None

        # Assert that the open function was called with the correct arguments
        open.assert_called_once_with('filename', 'r')

        # Reset the mock
        open.reset_mock()

    #  Get IPv6 address for OSF network interface
    def test_get_ipv6_addr(self, mocker):
        # Mock the netifaces.ifaddresses function to return a specific IPv6 address
        mocker.patch('netifaces.ifaddresses', return_value={netifaces.AF_INET6: [{'addr': 'fd00::1'}]})

        # Call the get_ipv6_addr function
        ipv6_addr = get_ipv6_addr('osf_interface')

        # Assert that the correct IPv6 address is returned
        assert ipv6_addr == 'fd00::1'

    #  Get mesh frequency for device
    def test_get_mesh_freq(self, mocker):
        # Mock the os.popen function to return a specific output
        mocker.patch('subprocess.check_output', return_value="Interface mesh channel 36 (5180 MHz), width: 20 MHz")

        # Call the get_mesh_freq function
        mesh_freq = get_mesh_freq()

        # Assert that the correct mesh frequency is returned
        assert mesh_freq == 5180

    #  Write content to a file successfully
    def test_write_file_successfully(self, mocker):
        # Mock the open function to return a file object
        mock_file = mocker.mock_open()
        mocker.patch('builtins.open', mock_file)

        # Call the write_file function
        write_file('test.txt', 'Hello, world!', 'Error occurred while writing file')

        # Assert that the file was opened and written to correctly
        mock_file.assert_called_once_with('test.txt', 'w')
        mock_file().write.assert_called_once_with('Hello, world!')

    #  Check if a process with a given name is running
    def test_is_process_running(self, mocker):
        # Mock the subprocess.check_output function to return a list of processes
        mocker.patch('subprocess.check_output', return_value=b'root      1234  0.0  0.0   1234   5678 ?        S    00:00:00 process_name')

        # Call the is_process_running function
        result = is_process_running('process_name')

        # Assert that the correct result is returned
        assert result is True

    #  Check if a process with a given name is running
    def test_successfully_kill_process_by_pid(self, mocker):
        # Mock the get_pid_by_process_name function to return a valid PID
        mocker.patch('util.get_pid_by_process_name', return_value=1234)
        # Mock the get_pid_by_process_name function to return a valid PID
        mocker.patch('util.is_process_running', return_value=True)

        # Mock the subprocess.check_output function to avoid actually killing the process
        mocker.patch('subprocess.check_output')

        # Call the kill_process_by_pid function
        kill_process_by_pid('process_name')

        # Assert that the subprocess.check_output function was called to kill running process
        subprocess.check_output.assert_called_once_with(['kill', '1234'])

    #  Get the frequency quality information for all frequencies even if some frequencies have errors
    def test_get_frequency_quality_with_errors(self, mocker):
        # Mock the numpy arrays for frequency quality, interference probabilities, and frequencies
        freq_quality = np.array([0.8, 0.9, np.nan, 0.7])
        probs = np.array([[0.1, 0.2, 0.3, 0.4], [0.4, 0.3, 0.2, 0.1], [0.2, 0.3, 0.4, 0.1], [0.3, 0.2, 0.1, 0.4]])
        frequencies = np.array([1, 2, 3, 4])

        # Call the get_frequency_quality function
        jamming_detected, freq_quality_dict = get_frequency_quality(freq_quality, probs, frequencies)

        # Assert that the jamming detection status is True
        assert jamming_detected == True

        # Assert that the frequency quality dictionary is correct
        expected_freq_quality_dict = {'1': 0.8, '2': 0.9, '3': np.nan, '4': 0.7}
        for key, expected_value in expected_freq_quality_dict.items():
            actual_value = freq_quality_dict[key]
            if math.isnan(expected_value):
                assert math.isnan(actual_value)
            else:
                assert actual_value == expected_value
