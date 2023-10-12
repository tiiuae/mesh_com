from typing import Tuple

import numpy as np
import torch
import torch.nn.functional as F

from options import Options, VALID_CHANNELS
from util import map_freq_to_channel


class ChannelQualityEstimator:
    """
    A class for estimating the quality of communication channels using a pre-trained ResCNN model.

    This class utilizes a pre-trained ResCNN model to estimate the channel quality based on input features.
    The channel quality is computed as a score that reflects the difference between good and jamming states.

    Positive channel quality values indicate a better channel quality, while negative values suggest
    potential jamming or lower channel quality.

    Attributes:
        device (str): A PyTorch device ('cuda' or 'cpu') to run the model on.
        model (ResCNN): The pre-trained ResCNN model for channel quality estimation.
    """

    def __init__(self) -> None:
        """
        Initializes the ChannelQualityEstimator object.

        Loads the pre-trained traced ResCNN model and sets the device to 'cuda' if available, 'cpu' otherwise.
        """
        self.args = Options()
        # Model related attributes
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        try:
            self.model = torch.jit.load(self.args.traced_model_path)
            self.model = self.model.to(self.device)
            self.model.eval()
        except FileNotFoundError:
            raise FileNotFoundError("Model file not found")
        except Exception as e:
            print(f"Exception error in loading pretrained model: {e}") if self.args.debug else None

    def _forward(self, feat_array: np.ndarray) -> np.ndarray:
        """
        Computes the class probabilities for a given feature array.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :return: A 2D NumPy array of shape (n_samples, n_classes) containing the model's class probabilities.
        """
        with torch.no_grad():
            # Create tensors
            inputs = torch.from_numpy(feat_array).float()
            inputs = inputs.to(self.device, non_blocking=True)
            # Feed inputs and compute jamming probability for each frequency
            out = self.model(inputs)
            # Compute the probabilities
            probs = F.softmax(out, dim=1)
        return probs.cpu().numpy()

    def _compute_channel_quality(self, probs: np.ndarray) -> np.ndarray:
        """
        Computes the channel quality scores based on the model's predictions.

        The channel quality is calculated as the difference between the weighted sum of good state probabilities
        and the average probability of jamming states. Positive channel quality values indicate a better channel
        quality, while negative values suggest potential jamming or lower channel quality.

        :param probs: A 2D NumPy array of shape (n_channels, n_classes) containing the model's class probabilities.
        :return: A 1D NumPy array of shape (n_channels,) representing the channel quality scores for each channel.
        """
        # Weights for good states (communication, floor, inter_mid, inter_high)
        good_weights = np.array([1.0, 0.6, 0.4])
        # Compute good state probabilities
        good_probs = probs[:, :3]  # Get the probabilities for the first 3 states
        # Compute a score based on the good state probabilities and their weights
        good_scores = np.dot(good_probs, good_weights)
        # Compute jamming state probabilities
        jamming_probs = probs[:, 3:]  # Get the probabilities for the remaining jamming states
        # Compute a jamming score based on the sum of the jamming probabilities
        jamming_scores = np.sum(jamming_probs, axis=1)
        # Compute channel quality as the difference between good scores and jamming scores
        channel_quality = good_scores - jamming_scores

        return channel_quality

    def check_arrays(self, feat_array: np.ndarray, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Check if numpy arrays contain NaN values or contain inf values.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :param frequencies: A 1D NumPy array of shape (n_channels,) containing the frequencies of the channels.
        """
        # Check if frequencies contains NaN values
        if np.isnan(frequencies).any():
            frequencies_nan_mask = np.isnan(frequencies)
            frequencies = frequencies[~frequencies_nan_mask]
            # Reshape feat_array to match the new size of frequencies
            feat_array = feat_array[~frequencies_nan_mask]

        # Check if feat_array contains NaN values
        elif np.isnan(feat_array).any():
            feat_array_nan_mask = np.isnan(feat_array)
            feat_array = feat_array[~feat_array_nan_mask]

        # Check if frequencies contains infinite values
        if np.isinf(frequencies).any():
            frequencies_inf_mask = np.isinf(frequencies)
            frequencies = frequencies[~frequencies_inf_mask]
            # Reshape feat_array to match the new size of frequencies
            feat_array = feat_array[~frequencies_inf_mask]

        # Check if feat_array contains infinite values
        elif np.isinf(feat_array).any():
            feat_array_inf_mask = np.isinf(feat_array)
            feat_array = feat_array[~feat_array_inf_mask]

        return feat_array, frequencies

    def estimate(self, feat_array: np.ndarray, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Estimates the channel quality for a given feature array.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :param frequencies: A 1D NumPy array of shape (n_channels,) containing the frequencies of the channels.
        :return: A tuple containing the estimated channel quality scores (1D NumPy array) and the class probabilities
        (2D NumPy array).
        """
        feat_array, frequencies = self.check_arrays(feat_array, frequencies)

        # Check if feat_array and frequencies arrays are empty
        if feat_array.size == 0 or frequencies.size == 0:
            return np.array([]), np.empty((0, 0)), np.array([])

        try:
            # Compute class probabilities for the given features
            probs = self._forward(feat_array)
            # Compute the channel quality
            channel_quality = self._compute_channel_quality(probs)
            try:
                freq_list = [int(freq) for freq in frequencies.tolist()]
                channel_list = [map_freq_to_channel(freq) for freq in freq_list]
                mask = np.isin(channel_list, VALID_CHANNELS)
                # Normalize the quality values
                quality_normalized = (channel_quality[mask] - (-1)) / (1 - (-1))
            except Exception:
                quality_normalized = np.array([])

            return channel_quality, probs, quality_normalized
        except Exception:
            return np.array([]), np.empty((0, 0)), np.array([])
