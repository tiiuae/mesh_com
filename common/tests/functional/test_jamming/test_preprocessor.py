import json

import numpy as np
import pandas as pd
import pytest

import add_syspath
from preprocessor import Preprocessor
from options import Options


class TestCodeUnderTest:

    #  Preprocessor object is initialized with default parameters
    def test_preprocessor_initialized_with_default_parameters(self):
        args = Options()
        with open(args.col_mean_std_path, 'r') as file:
            json_data = json.load(file)
        expected_cols_mean_std = json_data

        preprocessor = Preprocessor()
        assert preprocessor.input_length == 128
        assert preprocessor.cols_mean_std == expected_cols_mean_std

    #  feature_engineer method is called with a valid pd.DataFrame input
    def test_feature_engineer_with_valid_input(self):
        preprocessor = Preprocessor()
        data = {
            'freq1': [5180, 5200, 5220], 'max_magnitude': [1, 2, 3], 'noise': [0.1, 0.2, 0.3],
            'rssi': [10, 20, 30], 'relpwr_db': [40, 50, 60], 'base_pwr_db': [10, 20, 30],
            'avgpwr_db': [25, 35, 45], 'total_gain_db': [1, 2, 3], 'snr': [-10.457575, -7.212464, -5.376020],
            'cnr': [9.9, 19.8, 29.7], 'pn': [-156.535598, -156.535598, -156.535598], 'ssi': [-30, -30, -30],
            'pd': [-15, -15, -15], 'sinr': [-30, -30, -30], 'sir': [49, 68, 87], 'mr': [0.040000, 0.057143, 0.066667],
            'pr': [2.50, 1.75, 1.50]
        }
        expected_result = pd.DataFrame(data)

        df = pd.DataFrame(
            {'freq1': [5180, 5200, 5220], 'max_magnitude': [1, 2, 3], 'noise': [0.1, 0.2, 0.3],
             'rssi': [10, 20, 30], 'relpwr_db': [40, 50, 60], 'base_pwr_db': [10, 20, 30],
             'avgpwr_db': [25, 35, 45], 'total_gain_db': [1, 2, 3]})
        result = preprocessor.feature_engineer(df)

        # Assert that feature engineering performed as expected on initial df
        pd.testing.assert_frame_equal(result, expected_result)

    #  resize method is called with a valid pd.DataFrame input
    def test_resize_with_valid_input(self):
        preprocessor = Preprocessor()
        preprocessor.input_length = 6
        df = pd.DataFrame({'col1': [1, 2, 3], 'col2': [4, 5, 6], 'freq1': [1, 1, 1]})

        # Set expected_result
        expected_result = pd.DataFrame({'col1': [1, 1, 1, 2, 2, 3], 'col2': [4, 4, 4, 5, 5, 6], 'freq1': [1, 1, 1, 1, 1, 1]})
        # Iterate through columns and convert values to float for columns not in exclude_columns
        for col in expected_result.columns:
            if col not in ['freq1', 'freq2', 'tsf']:
                expected_result[col] = expected_result[col].astype(float)

        # Call function under test (resize)
        result = preprocessor.resize(df)

        # Assert that resized dataframe is returned as expected
        pd.testing.assert_frame_equal(result, expected_result)

    #  normalize method is called with a valid pd.DataFrame input
    def test_normalize_with_valid_input(self):
        # Set expected data
        expected_data = {
            'freq1': [5180], 'max_magnitude': [-2.699096], 'rssi': [0.386689], 'relpwr_db': [5.723355],
            'base_pwr_db': [-54.127222], 'avgpwr_db': [-7.553099], 'total_gain_db': [-8.580278], 'snr': [-4.467233],
            'cnr': [-19.34198], 'pn': [-22.509996], 'ssi': [-4.161893], 'pd': [-33.130156], 'sinr': [-4.161893],
            'sir': [7.230196], 'mr': [-2.481305], 'pr': [187.510565]
        }
        expected_result = pd.DataFrame(expected_data)

        # Create preprocessor instance
        preprocessor = Preprocessor()

        # Create sample dataframe to pass to normalize function
        data = {
            'freq1': [5180], 'max_magnitude': [1], 'rssi': [10], 'relpwr_db': [40], 'base_pwr_db': [10],
            'avgpwr_db': [25], 'total_gain_db': [1], 'snr': [-10.457575], 'cnr': [9.9], 'pn': [-156.535598],
            'ssi': [-30], 'pd': [-15], 'sinr': [-30], 'sir': [49], 'mr': [0.040000], 'pr': [2.50]
        }
        df = pd.DataFrame(data)

        # Call function under test
        result = preprocessor.normalize(df)

        # Assert result of normalize function is equal to the expected result
        pd.testing.assert_frame_equal(result, expected_result)

    #  input_length parameter is set to 0
    def test_input_length_zero(self):
        preprocessor = Preprocessor(input_length=0)
        assert preprocessor.input_length == 0

    #  col_mean_std_path parameter is set to an invalid file path
    def test_invalid_col_mean_std_path(self):
        with pytest.raises(FileNotFoundError):
            Preprocessor(col_mean_std_path='invalid_path/cols_mean_std.json')

    #  feature_engineer method is called with an empty pd.DataFrame input
    def test_feature_engineer_with_empty_input(self):
        preprocessor = Preprocessor()
        df = pd.DataFrame()
        expected_result = pd.DataFrame()
        result = preprocessor.feature_engineer(df)
        pd.testing.assert_frame_equal(result, expected_result)

    #  resize method is called with an empty pd.DataFrame input
    def test_resize_with_empty_input(self):
        preprocessor = Preprocessor()
        df = pd.DataFrame()
        expected_result = pd.DataFrame()
        result = preprocessor.resize(df)
        pd.testing.assert_frame_equal(result, expected_result)

    #  normalize method is called with an empty pd.DataFrame input
    def test_normalize_with_empty_input(self):
        preprocessor = Preprocessor()
        df = pd.DataFrame()
        expected_result = pd.DataFrame()
        result = preprocessor.normalize(df)
        pd.testing.assert_frame_equal(result, expected_result)

    #  preprocess method is called with an empty pd.DataFrame input
    def test_preprocess_with_empty_input(self):
        preprocessor = Preprocessor()
        df = pd.DataFrame()
        expected_feat_array = np.array([])
        expected_unique_freqs = np.array([])
        feat_array, unique_freqs = preprocessor.preprocess(df)
        np.testing.assert_array_equal(feat_array, expected_feat_array)
        np.testing.assert_array_equal(unique_freqs, expected_unique_freqs)

    #  preprocess method returns a tuple containing a NumPy array and a NumPy array of unique frequencies
    def test_preprocess_returns_tuple_with_numpy_array_and_unique_frequencies(self):
        # Create a mock DataFrame with one row
        df = pd.DataFrame({'freq1': [1], 'max_magnitude': [2], 'total_gain_db': [3], 'base_pwr_db': [4], 'rssi': [5], 'relpwr_db': [6], 'avgpwr_db': [7]})

        # Create an instance of the Preprocessor class
        preprocessor = Preprocessor()

        # Call the preprocess method
        result = preprocessor.preprocess(df)

        # Assert that the result is a tuple
        assert isinstance(result, tuple)

        # Assert that the first element of the tuple is a NumPy array
        assert isinstance(result[0], np.ndarray)

        # Assert that the second element of the tuple is a NumPy array
        assert isinstance(result[1], np.ndarray)

    #  preprocess method is called with a pd.DataFrame input containing only one row
    def test_preprocess_called_with_dataframe_with_one_row(self, mocker):
        # Create a mock DataFrame with one row
        df = pd.DataFrame({'freq1': [1,1,1], 'max_magnitude': [2,2,2], 'total_gain_db': [3,3,3], 'base_pwr_db': [4,4,4], 'rssi': [5,5,5], 'relpwr_db': [6,6,6], 'avgpwr_db': [7,7,7]})

        # Create an instance of the Preprocessor class
        preprocessor = Preprocessor()

        # Mock methods not to be tested
        mocker.patch('pandas.concat')
        mocker.patch('numpy.array')
        mocker.patch('numpy.reshape')
        mocker.patch.object(preprocessor, 'resize')
        mocker.patch.object(preprocessor, 'feature_engineer')
        mocker.patch.object(preprocessor, 'normalize')

        # Call the preprocess method
        preprocessor.preprocess(df)

        # Assert that the resize method was called once
        assert preprocessor.resize.call_count == 1

    #  preprocess method is called with a pd.DataFrame input containing multiple rows and one unique frequency
    def test_preprocess_with_valid_input(self, mocker):
        # Create a mock DataFrame with one row and one unique frequency
        data = {'freq1': [1, 1, 1], 'max_magnitude': [2, 2, 2], 'total_gain_db': [3, 3, 3],
                'base_pwr_db': [4, 4, 4], 'rssi': [5, 5, 5], 'relpwr_db': [6, 6, 6],
                'avgpwr_db': [7, 7, 7], 'snr': [8, 8, 8], 'cnr': [9, 9, 9],
                'pn': [10, 10, 10], 'ssi': [11, 11, 11], 'pd': [12, 12, 12],
                'sinr': [13, 13, 13], 'sir': [14, 14, 14], 'mr': [15, 15, 15], 'pr': [16, 16, 16]}
        df_mock = pd.DataFrame(data)

        # Create a mock Preprocessor object
        preprocessor = Preprocessor()

        # Mock the resize method to return a DataFrame with the same length as the input DataFrame
        mocker.patch.object(preprocessor, 'resize', return_value=df_mock)

        # Mock the feature_engineer method to return a DataFrame with the same length as the input DataFrame
        mocker.patch.object(preprocessor, 'feature_engineer', return_value=df_mock)

        # Mock the normalize method to return a DataFrame with the same length as the input DataFrame
        mocker.patch.object(preprocessor, 'normalize', return_value=df_mock)

        # Mock the reshape and transpose operations to return a correctly shaped array
        mocker.patch('numpy.reshape', return_value=np.zeros((3, 16, 1)))  # Replace with the desired shape

        # Call the preprocess method with the mock DataFrame
        result = preprocessor.preprocess(df_mock)

        # Assert that the result is a tuple containing the processed DataFrame as a NumPy array and the unique frequency
        assert isinstance(result, tuple)
        assert isinstance(result[0], np.ndarray)
        assert isinstance(result[1], np.ndarray)

        # Assert that the processed DataFrame has the expected shape (post transpose operation)
        assert result[0].shape == (3, 1, 16)

        # Assert that the unique frequency is as expected (replace with the desired value)
        assert np.array_equal(result[1], np.array([1]))

    # Test that an exception in the preprocess method triggers empty array returns
    def test_exception_in_preprocess_returns_empty_array(self, mocker):
        # Create a mock DataFrame with one row and one unique frequency
        df_mock = pd.DataFrame({'freq1': [1, 1, 1], 'max_magnitude': [2, 2, 2], 'total_gain_db': [3, 3, 3],
                                'base_pwr_db': [4, 4, 4], 'rssi': [5, 5, 5], 'relpwr_db': [6, 6, 6], 'avgpwr_db': [7, 7, 7]})

        # Create a mock Preprocessor object
        preprocessor = Preprocessor()

        # Mock the resize method to return a DataFrame with a different length than expected
        mocker.patch.object(preprocessor, 'resize', return_value=pd.DataFrame(
            {'freq1': [1, 2], 'max_magnitude': [2, 3], 'total_gain_db': [3, 4], 'base_pwr_db': [4, 5], 'rssi': [5, 6], 'relpwr_db': [6, 7], 'avgpwr_db': [7, 8], 'noise': [8, 9]}))

        # Call function under test
        feat_array, unique_freqs = preprocessor.preprocess(df_mock)

        # Assert that empty arrays were returned
        expected_feat_array = np.array([])
        expected_unique_freqs = np.array([])
        np.testing.assert_array_equal(feat_array, expected_feat_array)
        np.testing.assert_array_equal(unique_freqs, expected_unique_freqs)

    #  The cols_mean_std JSON file is empty
    def test_preprocessor_object_with_non_existent_file(self, mocker):
        # Create a mock Preprocessor object with an empty cols_mean_std JSON file
        with pytest.raises(FileNotFoundError):
            Preprocessor(col_mean_std_path='empty_cols_mean_std.json')
