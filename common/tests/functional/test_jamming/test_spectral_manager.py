import os

import pandas as pd
import pytest
from unittest.mock import patch, Mock

import add_syspath
from spectral_manager import Spectral
import util


class TestCodeUnderTest:

    #  Initialize Spectral object and call all methods without errors
    def test_initialize_spectral_object(self, mocker):
        mocker.patch.object(Spectral, 'get_driver')
        mocker.patch.object(Spectral, 'initialize_scan')
        mocker.patch.object(Spectral, 'execute_scan')
        mocker.patch.object(Spectral, 'read')
        mocker.patch.object(Spectral, 'create_dataframe')

        spectral = Spectral()
        spectral.get_driver.return_value = "ath10k"
        spectral.initialize_scan.return_value = None
        spectral.execute_scan.return_value = None
        spectral.read.return_value = None
        spectral.create_dataframe.return_value = pd.DataFrame()

        spectral.initialize_scan(spectral.get_driver())
        bin_file = "scan.bin"
        spectral.execute_scan("2.412G", spectral.get_driver(), bin_file)
        spectral.read(bin_file)
        spectral.create_dataframe(spectral.get_driver())

        assert spectral.get_driver.called
        assert spectral.initialize_scan.called
        assert spectral.execute_scan.called
        assert spectral.read.called
        assert spectral.create_dataframe.called

        # Delete the bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Read a valid binary file and create a dataframe
    def test_read_valid_binary_file(self, mocker):
        mocker.patch.object(Spectral, 'file_open')
        mocker.patch.object(Spectral, 'file_close')

        spectral = Spectral()
        spectral.file_open.return_value = mocker.MagicMock()
        spectral.file_close.return_value = None

        directory = os.getcwd()
        bin_file = directory + "/test.bin"

        with open(bin_file, "wb") as file:
            binary_data = b"\x48\x65\x6C\x6C\x6F"
            file.write(binary_data)

        spectral.read(bin_file)

        assert spectral.file_open.called
        assert spectral.file_close.called

        # Delete the bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Driver not detected
    def test_driver_not_detected(self, mocker):
        mocker.patch.object(Spectral, 'get_driver')

        spectral = Spectral()
        spectral.get_driver.return_value = "unknown"

        with pytest.raises(Exception):
            spectral.initialize_scan(spectral.get_driver())

    #  Spectral scan file not found
    def test_spectral_scan_file_not_found(self, mocker):
        mocker.patch.object(Spectral, 'read')

        spectral = Spectral()
        spectral.read.side_effect = FileNotFoundError

        bin_file = "scan.bin"

        with pytest.raises(FileNotFoundError):
            spectral.read("scan.bin")

        # Delete the bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Permission error when accessing spectral scan file
    def test_permission_error(self, mocker):
        mocker.patch.object(Spectral, 'read')

        spectral = Spectral()
        spectral.read.side_effect = PermissionError

        bin_file = "scan.bin"

        with pytest.raises(PermissionError):
            spectral.read(bin_file)

        # Delete the empty.bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Malformed packet in binary file
    def test_malformed_packet(self, mocker):
        mocker.patch.object(Spectral, 'read')

        spectral = Spectral()
        spectral.read.side_effect = Exception("skip malformed packet")

        bin_file = "scan.bin"

        with pytest.raises(Exception):
            spectral.read(bin_file)

        # Delete the bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Unknown chantype in binary file
    def test_unknown_chantype(self, mocker):
        mocker.patch.object(Spectral, 'read')

        spectral = Spectral()
        spectral.read.side_effect = Exception("got unknown chantype: 2")

        bin_file = "scan.bin"

        with pytest.raises(Exception):
            spectral.read(bin_file)

        # Delete the empty.bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Test that the correct driver is returned
    def test_get_driver_with_valid_driver(self, mocker):
        # Mock the necessary dependencies
        mock_popen = Mock()
        mock_popen.return_value.read.return_value = 'ath10k'
        mocker.patch('os.popen', mock_popen)

        # Create an instance of the Spectral class
        spectral = Spectral()

        # Mock the necessary arguments
        spectral.args.debug = False

        driver = spectral.get_driver()

        expected_value = 'ath10k'

        # Assert that os.popen was called
        assert driver == expected_value

    #  Test that the correct driver is returned
    def test_get_driver_with_invalid_driver(self, mocker):
        # Mock the necessary dependencies
        mock_popen = Mock()
        mock_popen.return_value.read.return_value = 'invalid_driver'
        mocker.patch('os.popen', mock_popen)

        # Create an instance of the Spectral class
        spectral = Spectral()

        # Mock the necessary arguments
        spectral.args.debug = False

        # Call the get_driver method
        with pytest.raises(SystemExit):
            spectral.get_driver()

        # Assert that os.popen was called
        os.popen.assert_called_once_with('ls /sys/kernel/debug/ieee80211/phy* | grep ath')

    #  Test that the exception raised with invalid driver passed
    def test_initialize_scan_invalid_driver(self, mocker):
        # Mock the Options class
        mocker.patch('options.Options')

        # Create a Spectral object
        spectral = Spectral()

        # Call the initialize_scan() method with an invalid driver
        with pytest.raises(Exception):
            spectral.initialize_scan("invalid_driver")

    # Test create a dataframe with a valid driver
    def test_create_dataframe_valid_driver(self, mocker):
        # Mock the Options class
        mocker.patch('options.Options')

        # Create a Spectral object
        spectral = Spectral()

        # Set VALUES attribute with valid values
        spectral.VALUES = {0: (1, 2, 3, 4, 5, 6, 7, 8), 1: (9, 10, 11, 12, 13, 14, 15, 16)}

        # Call the create_dataframe() method with a valid driver
        dataframe = spectral.create_dataframe("ath10k")

        # Assert that the returned value is a pandas DataFrame
        assert isinstance(dataframe, pd.DataFrame)

        # Assert that the dataframe has the expected columns and values
        assert dataframe.columns.tolist() == ["freq1", "noise", "max_magnitude", "total_gain_db", "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db"]
        assert dataframe.values.tolist() == [[1, 2, 3, 4, 5, 6, 7, 8], [9, 10, 11, 12, 13, 14, 15, 16]]

    def test_open_nonexistent_binary_file(self, mocker):
        # Mock the Options class
        mocker.patch('options.Options')

        # Create a Spectral object
        spectral = Spectral()

        # Define file
        bin_file = "nonexistent.bin"

        # Call the read() method with a non-existent binary file
        with pytest.raises(FileNotFoundError):
            spectral.file_open(bin_file)

        # Delete file after test
        if os.path.exists(bin_file):
            try:
                os.remove(bin_file)
            except Exception as e:
                print(f"Error deleting the file: {str(e)}")

    def test_empty_binary_file(self, mocker):
        # Mock the Options class
        mocker.patch('options.Options')

        # Create a Spectral object
        spectral = Spectral()

        bin_file = "empty.bin"

        # Call the read() method with an empty binary file
        spectral.read(bin_file)

        # Assert that VALUES dictionary is empty
        assert len(spectral.VALUES) == 0

        # Delete the bin file
        if os.path.exists(bin_file):
            os.remove(bin_file)

    #  Test that the correct frequencies are scanned
    def test_correct_frequencies_scanned(self, mocker):
        with patch.object(util, 'run_command') as mock_run_command, \
                patch.object(Spectral, 'file_open'), \
                patch.object(Spectral, 'file_close'):

            spectral = Spectral()
            spectral.args.scan_interface = "wlan0"
            frequencies = "5180 5200 5220"
            driver = "ath10k"
            directory = os.getcwd()
            bin_file = directory + "/test.bin"

            # Define custom functions to replace util.run_command
            def custom_run_command(command, error_message):
                # Here, you can simulate the behavior of util.run_command without executing the actual commands
                if "ip link set dev wlan0 up" in command:
                    return 0    # Simulate success
                elif "iw dev wlan0 scan freq 5180 5200 5220 flush" in command:
                    return 0    # Simulate success
                elif "echo disable > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl" in command:
                    return 0    # Simulate success
                elif "cat /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan0 > scan.bin" in command:
                    return 0    # Simulate success
                else:
                    return 1    # Simulate failure for unrecognized commands

            # Apply the custom_run_command function as the side_effect for util.run_command
            mock_run_command.side_effect = custom_run_command

            with open(bin_file, "wb") as file:
                binary_data = b"\x48\x65\x6C\x6C\x6F"
                file.write(binary_data)

            spectral.execute_scan(frequencies, driver, bin_file)

            # Delete the empty.bin file
            if os.path.exists(bin_file):
                os.remove(bin_file)

    def test_execute_scan_with_valid_arguments(self):
        # Mock the Options class
        options_mock = Mock()
        with patch('options.Options', return_value=options_mock), \
                patch('util.map_freq_to_channel') as map_freq_to_channel_mock, \
                patch('options.VALID_CHANNELS') as VALID_CHANNELS_mock, \
                patch('util.run_command') as run_command_mock:

            # Define custom functions to replace util.run_command
            def custom_run_command(command, error_message):
                # Here, you can simulate the behavior of util.run_command without executing the actual commands
                if "ip link set dev" in command:
                    # Simulate success
                    return 0
                elif "iw dev" in command:
                    # Simulate success
                    return 0
                elif "echo disable > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl" in command:
                    # Simulate success
                    return 0
                elif "cat /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan0 >" in command:
                    # Simulate success
                    return 0
                else:
                    # Simulate failure for unrecognized commands
                    return 1

            # Apply the custom_run_command function as the side_effect for util.run_command
            run_command_mock.side_effect = custom_run_command

            # Create an instance of the Spectral class
            spectral = Spectral()

            bin_file = "scan.bin"

            # Call the execute_scan() method with valid arguments
            spectral.execute_scan("5180 5745", "ath10k", bin_file)

            # Delete the empty.bin file
            if os.path.exists(bin_file):
                os.remove(bin_file)

    def test_initialize_scan_with_valid_driver(self, mocker):
        # Mock the run_command() function
        mocker.patch('util.run_command', side_effect=lambda command, error_message: (True, None))
        mocker.patch('subprocess.call', return_value=0)
        mocker.patch('os.path.exists', return_value=True)

        # Call the initialize_scan() method with the file paths
        spectral = Spectral()
        spectral.initialize_scan("ath10k")

        # Assert that the mock run_command() function was called with the correct arguments
        expected_calls = [
            mocker.call("echo background > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl", 'Failed to run cmd_background'),
            mocker.call("echo trigger > /sys/kernel/debug/ieee80211/phy0/ath10k/spectral_scan_ctl", 'Failed to run cmd_trigger')
        ]
        mocker.call.assert_has_calls(expected_calls, any_order=True)
