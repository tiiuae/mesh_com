"""
channel_quality_estimator.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License: 

Repository:
"""
from typing import Tuple

import numpy as np
import torch
import torch.nn.functional as F


class ChannelQualityEstimator:
    """
    A class for estimating the quality of communication channels using a pre-trained ResCNN model.

    This class utilizes a pre-trained ResCNN model to estimate the channel quality based on input features.
    The channel quality is computed as a score that combines the probabilities of good and jamming states,
    providing a relative measure of quality for each channel.

    Positive channel quality values indicate a better channel quality, while negative values suggest
    potential jamming or lower channel quality.

    Attributes:
        device (str): A PyTorch device ('cuda' or 'cpu') to run the model on.
        model (ResCNN): The pre-trained ResCNN model for channel quality estimation.
    """

    def __init__(self):
        """
        Initializes the ChannelQualityEstimator object.

        Loads the pre-trained ResCNN model and sets the device to 'cuda' if available, 'cpu' otherwise.
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # self.model = ResCNN(15, 26, separable=True).to(self.device)
        # self.model.load_state_dict(torch.load('pretrained_model/best.pth'))
        # self.model.eval()
        self.model = torch.jit.load("my_traced_model.pt")
        self.model = self.model.to(self.device)
        self.model.eval()

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
        good_weights = np.array([0.75, 1.0, 0.5, 0.25])
        # Compute good state probabilities
        good_probs = probs[:, :4]  # Get the probabilities for the first 4 states
        # Compute a score based on the good state probabilities and their weights
        good_scores = np.dot(good_probs, good_weights)
        # Compute jamming state probabilities
        jamming_probs = probs[:, 4:]  # Get the probabilities for the remaining jamming states
        # Compute a jamming score based on the average probability of jamming states
        jamming_scores = np.mean(jamming_probs, axis=1)
        # Compute channel quality as the difference between good scores and jamming scores
        channel_quality = good_scores - jamming_scores

        return channel_quality

    def estimate(self, feat_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimates the channel quality for a given feature array.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :return: A 1D NumPy array of shape (n_channels,) representing the channel quality scores for each channel.
        """
        # Compute class probabilities for the given features
        probs = self._forward(feat_array)
        # Compute the channel quality
        channel_quality = self._compute_channel_quality(probs)

        return channel_quality, probs
