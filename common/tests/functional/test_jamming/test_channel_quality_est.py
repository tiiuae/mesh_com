import numpy as np
import pytest
from unittest.mock import patch

import add_syspath
from channel_quality_est import ChannelQualityEstimator


class TestCodeUnderTest:

    #  Estimating channel quality with valid input features and frequencies
    def test_valid_input_features_and_frequencies(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Mock _compute_channel_quality functions
        mocker.patch('numpy.dot', return_value=np.array([90, 70]))
        mocker.patch('numpy.sum', return_value=np.array([20, 30]))

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the mocked return value for _forward
        mocked_probs = np.array([[5180, 5180, 5180, 5200, 5200, 5200],
                                [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]])

        # Define the input features and frequencies for estimate function
        feat_array = np.array([[1, 2, 3], [4, 5, 6]])
        frequencies = np.array([5180, 5200])

        # Use patch to mock the _forward function and return the mocked_probs
        with patch.object(cqe, '_forward', return_value=mocked_probs):
            # Call the estimate method
            channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (2,)
        assert probs.shape == (2, 6)
        assert quality_normalized.shape == (2,)

    #  Estimating channel quality with minimum input feature values
    def test_minimum_input_feature_values(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Mock _compute_channel_quality functions
        mocker.patch('numpy.dot', return_value=np.array([90]))
        mocker.patch('numpy.sum', return_value=np.array([20]))

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the mocked return value for _forward
        mocked_probs = np.array([[5180], [0.1]])

        # Define the input features and frequencies for estimate function
        feat_array = np.array([1])
        frequencies = np.array([5180])

        # Use patch to mock the _forward function and return the mocked_probs
        with patch.object(cqe, '_forward', return_value=mocked_probs):
            # Call the estimate method
            channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (1,)
        assert probs.shape == (2, 1)
        assert quality_normalized.shape == (1,)

    #  Estimating channel quality with empty input feature array
    def test_empty_input_feature_array(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=False)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([])
        frequencies = np.array([5180])

        # Call the estimate method
        channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (0,)
        assert probs.shape == (0, 0)
        assert quality_normalized.shape == (0,)

    #  Estimating channel quality with empty frequency array
    def test_empty_frequency_array(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([[1, 2, 3]])
        frequencies = np.array([])

        # Call the estimate method
        channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (0,)
        assert probs.shape == (0, 0)
        assert quality_normalized.shape == (0,)

    #  Estimating channel quality with input feature array having NaN values
    def test_input_feature_array_with_nan_values(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=False)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([[1, np.nan, 3]])
        frequencies = np.array([5180])

        # Call the estimate method
        channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (0,)
        assert probs.shape == (0, 0)
        assert quality_normalized.shape == (0,)

    #  Estimating channel quality with input feature array having infinite values
    def test_input_feature_array_with_infinite_values(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Mock _compute_channel_quality functions
        mocker.patch('numpy.dot', return_value=np.array([90]))
        mocker.patch('numpy.sum', return_value=np.array([20]))

        # Define the mocked return value for _forward
        mocked_probs = np.array([[5180], [0.1]])

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([[1, np.inf, 3]])
        frequencies = np.array([5180])

        # Use patch to mock the _forward function and return the mocked_probs
        with patch.object(cqe, '_forward', return_value=mocked_probs):
            # Call the estimate method
            channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (1,)
        assert probs.shape == (2, 1)
        assert quality_normalized.shape == (1,)

    #  Estimating channel quality with input feature array having negative values
    def test_negative_input_features(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Mock _compute_channel_quality functions
        mocker.patch('numpy.dot', return_value=np.array([-90]))
        mocker.patch('numpy.sum', return_value=np.array([-20]))

        # Define the mocked return value for _forward
        mocked_probs = np.array([[5180], [-0.1]])

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([[-1, -2, -3]])
        frequencies = np.array([5180])

        # Use patch to mock the _forward function and return the mocked_probs
        with patch.object(cqe, '_forward', return_value=mocked_probs):
            # Call the estimate method
            channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (1,)
        assert probs.shape == (2, 1)
        assert quality_normalized.shape == (1,)

    #  Estimating channel quality with frequencies outside the 5 GHz range
    def test_invalid_frequencies(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load')
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Mock _compute_channel_quality functions
        mocker.patch('numpy.dot', return_value=np.array([-90]))
        mocker.patch('numpy.sum', return_value=np.array([-20]))

        # Define the mocked return value for _forward
        mocked_probs = np.array([[5180], [0.1]])

        # Create an instance of the ChannelQualityEstimator class
        cqe = ChannelQualityEstimator()

        # Define the input features and frequencies
        feat_array = np.array([[1, 2, 3]])
        frequencies = np.array([5000, 2000])

        # Use patch to mock the _forward function and return the mocked_probs
        with patch.object(cqe, '_forward', return_value=mocked_probs):
            # Call the estimate method
            channel_quality, probs, quality_normalized = cqe.estimate(feat_array, frequencies)

        # Assert the expected outputs
        assert channel_quality.shape == (1,)
        assert probs.shape == (2, 1)
        assert quality_normalized.shape == (0,)

    #  Estimating channel quality with a pre-trained model that is not loaded
    def test_pretrained_model_not_loaded(self, mocker):
        # Mock the necessary dependencies
        mocker.patch('torch.cuda.is_available', return_value=True)
        mocker.patch('torch.jit.load', side_effect=FileNotFoundError)
        mocker.patch('torch.from_numpy')
        mocker.patch('torch.Tensor.to')
        mocker.patch('torch.Tensor.cpu')
        mocker.patch('torch.Tensor.numpy')

        # Create an instance of the ChannelQualityEstimator class
        with pytest.raises(FileNotFoundError):
            cqe = ChannelQualityEstimator()

