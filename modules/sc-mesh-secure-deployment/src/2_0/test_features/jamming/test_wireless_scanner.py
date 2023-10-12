import pandas as pd
from unittest.mock import Mock

import add_syspath
from options import Options
from spectral_manager import Spectral
from wireless_scanner import WirelessScanner


class TestCodeUnderTest:

    #  Get all available frequencies for the device for 5.0GHz band
    def test_get_band_frequencies_5ghz(self, mocker):
        current_freq: int = 5240
        scan_band = "5.0GHz"
        expected_result = "5180 5200 5220"

        wireless_scanner = WirelessScanner()
        mocker.patch.object(wireless_scanner.args, 'channels5', [36, 40, 44])
        frequencies = wireless_scanner.get_band_frequencies(current_freq, scan_band)

        assert frequencies == expected_result

    #  Read a random CSV file from the specified folder and filter rows with a specific 'freq1' value
    def test_get_current_freq_sample_data(self, mocker):
        current_freq: int = 5180
        expected_result = pd.DataFrame({'freq1': [5180, 5180, 5180]})

        wireless_scanner = WirelessScanner()
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch('util.load_sample_data', return_value=("Jamming_5180MHz.csv", expected_result))

        curr_freq_data = wireless_scanner.get_current_freq_sample_data()

        pd.testing.assert_frame_equal(curr_freq_data, expected_result)

    #  Run a low latency spectral scan for the current frequency
    def test_scan_current_frequency(self, mocker):
        current_freq: int = 5180
        expected_result = pd.DataFrame({'freq1': [5180, 5180, 5180]})

        wireless_scanner = WirelessScanner()
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch.object(wireless_scanner, 'scan', return_value=expected_result)

        scan = wireless_scanner.scan_current_frequency()

        pd.testing.assert_frame_equal(scan, expected_result)

    #  Run a high-latency spectral scan on specified bands
    def test_high_latency_scan(self, mocker):
        current_freq: int = 5180
        data = [5180, 5200, 5220, 5240]
        expected_result = pd.DataFrame(data, columns=['freq1'])

        wireless_scanner = WirelessScanner()
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch.object(wireless_scanner, 'get_band_frequencies', return_value="5200 5220 5240")
        mocker.patch.object(wireless_scanner, 'scan', return_value=expected_result)

        scan = wireless_scanner.scan_all_frequencies()

        pd.testing.assert_frame_equal(scan, expected_result)

    #  Scan a list of frequencies and return a pandas DataFrame
    def test_scan(self, mocker):
        current_freq: int = 5200
        frequencies = "5200 5220 5200"
        # freq1,noise,max_magnitude,total_gain_db,base_pwr_db,rssi,relpwr_db,avgpwr_db
        expected_result = pd.DataFrame(
            {'freq1': [5200, 5200, 5220], 'noise': [1, 2, 3], 'max_magnitude': [1, 2, 3], 'total_gain_db': [1, 2, 3], 'base_pwr_db': [1, 2, 3], 'rssi': [1, 2, 3],
             'relpwr_db': [1, 2, 3], 'avgpwr_db': [1, 2, 3]})

        wireless_scanner = WirelessScanner()
        wireless_scanner.args.debug = False

        # Mock the necessary dependencies
        mock_popen = Mock()
        mock_popen.return_value.read.return_value = 'ath10k'
        mocker.patch('os.popen', mock_popen)
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch('util.run_command', side_effect=lambda command, error_message: (True, None))
        mocker.patch.object(wireless_scanner.spec, 'get_driver', return_value="ath10k")
        mocker.patch.object(wireless_scanner.spec, 'initialize_scan')
        mocker.patch.object(wireless_scanner.spec, 'execute_scan')
        mocker.patch.object(wireless_scanner.spec, 'read')
        mocker.patch.object(wireless_scanner.spec, 'create_dataframe', return_value=expected_result)
        mocker.patch.object(wireless_scanner, 'is_valid_scan', return_value=True)
        mocker.patch('pandas.concat', return_value=expected_result)

        scan = wireless_scanner.scan(frequencies)
        print("scan returned: ", scan)

        pd.testing.assert_frame_equal(scan, expected_result)

    #  Check if the given scan is valid based on the minimum number of rows per frequency
    def test_is_valid_scan(self):
        data = [5200, 5200, 5220, 5220]
        scan = pd.DataFrame(data, columns=['freq1'])

        wireless_scanner = WirelessScanner()

        is_valid = wireless_scanner.is_valid_scan(scan)

        assert is_valid == True

    #  Error message when scan_band is not 5.0GHz
    def test_get_band_frequencies_invalid_scan_band(self, capsys, mocker):
        current_freq: int = 5180
        scan_band = "3GHz"

        wireless_scanner = WirelessScanner()
        mocker.patch.object(wireless_scanner.args, 'debug', True)

        frequencies = wireless_scanner.get_band_frequencies(current_freq, scan_band)

        captured = capsys.readouterr()
        assert "Error: Scan band can should be 5.0GHz" in captured.out

        assert frequencies == ''

    #  Frequency not found in the file path
    def test_get_current_freq_sample_data_frequency_not_found(self, capsys, mocker):
        current_freq: int = 5200
        message = "Jamming data sample/jamming/jamming_freq5200.csv"
        scan = pd.DataFrame({'freq1': [5200, 5200, 5200], 'noise': [1, 2, 3], 'max_magnitude': [1, 2, 3], 'total_gain_db': [1, 2, 3], 'base_pwr_db': [1, 2, 3], 'rssi': [1, 2, 3],
                             'relpwr_db': [1, 2, 3], 'avgpwr_db': [1, 2, 3]})

        # Mock util functions
        mocker.patch('util.get_mesh_freq', autospec=True, return_value=current_freq)
        mocker.patch('random.choice', autospec=True, side_effect=lambda x: 'jamming' if 'jamming' in x else x[0] if isinstance(x, list) else x)
        mocker.patch('util.load_sample_data', autospec=True, return_value=(message, scan))

        # Create wireless scanner instance and mock attributes
        wireless_scanner = WirelessScanner()
        mocker.patch.object(wireless_scanner.args, 'debug', True)

        # Call get_current_freq_sample_data function
        curr_freq_data = wireless_scanner.get_current_freq_sample_data()

        captured = capsys.readouterr()
        assert "Frequency not found in the file path." in captured.out
        assert curr_freq_data.empty == True

    #  When jamming file selected, filter for the jammed frequency rows, replace their freq1 value with current mesh frequency value
    def test_get_current_freq_sample_data_jamming_file(self, mocker):
        current_freq: int = 5180
        message = "Jamming_5200MHz.csv"
        scan = pd.DataFrame({'freq1': [5200, 5200]})
        expected_result = pd.DataFrame({'freq1': [5180, 5180]})

        wireless_scanner = WirelessScanner()
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch('util.load_sample_data', return_value=(message, scan))

        curr_freq_data = wireless_scanner.get_current_freq_sample_data()

        pd.testing.assert_frame_equal(curr_freq_data, expected_result)

    #  When scan is invalid, return False
    def test_is_valid_scan_invalid_scan(self):
        scan = pd.DataFrame()

        wireless_scanner = WirelessScanner()

        is_valid = wireless_scanner.is_valid_scan(scan)

        assert is_valid == False

    #  Concatenate current frequency data with filtered scan dataframe
    def test_low_latency_scan(self, mocker):
        current_freq: int = 5180

        wireless_scanner = WirelessScanner()
        mocker.patch('util.get_mesh_freq', return_value=current_freq)
        mocker.patch.object(wireless_scanner, 'args')
        mocker.patch.object(wireless_scanner, 'scan')
        mocker.patch.object(wireless_scanner, 'is_valid_scan')

        wireless_scanner.args.debug = False
        wireless_scanner.scan.return_value = pd.DataFrame({'freq1': [5180, 5180], 'data': [3, 4]})
        wireless_scanner.is_valid_scan.return_value = True

        expected_result = pd.DataFrame({'freq1': [5180, 5180], 'data': [3, 4]})

        result = wireless_scanner.scan_current_frequency()

        assert result.equals(expected_result)

    #  Initialize the WirelessScanner object with Options and Spectral
    def test_initialize_wireless_scanner(self):
        wireless_scanner = WirelessScanner()
        assert isinstance(wireless_scanner.args, Options)
        assert isinstance(wireless_scanner.spec, Spectral)
