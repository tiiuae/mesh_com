"""
channel_quality_estimator.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License: 

Repository:
"""

import json
import socket
from typing import Tuple

import numpy as np
import torch
import torch.nn.functional as F

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

    def __init__(self, host: str = 'localhost', port: int = 8000) -> None:
        """
        Initializes the ChannelQualityEstimator object.

        Loads the pre-trained traced ResCNN model and sets the device to 'cuda' if available, 'cpu' otherwise.
        Creates a socket object to be used for sending data.
        """
        self.host = host
        self.port = port

        # Model related attributes
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.jit.load("my_traced_model.pt")
        self.model = self.model.to(self.device)
        self.model.eval()

        # Socket object for sending data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

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
        good_weights = np.array([0.8, 1.0, 0.6, 0.4])
        # Compute good state probabilities
        good_probs = probs[:, :4]  # Get the probabilities for the first 4 states
        # Compute a score based on the good state probabilities and their weights
        good_scores = np.dot(good_probs, good_weights)
        # Compute jamming state probabilities
        jamming_probs = probs[:, 4:]  # Get the probabilities for the remaining jamming states
        # Compute a jamming score based on the sum of the jamming probabilities
        jamming_scores = np.sum(jamming_probs, axis=1)
        # Compute channel quality as the difference between good scores and jamming scores
        channel_quality = good_scores - jamming_scores

        return channel_quality

    def send_data(self, channel_quality: np.ndarray, frequencies: np.ndarray) -> None:
        """
        Sends the estimated channel quality data to a remote server.

        :param channel_quality: A 1D NumPy array of shape (n_channels,) containing the estimated channel quality scores.
        :param frequencies: A 1D NumPy array of shape (n_channels,) containing the frequencies of the channels.
        """
        # Filter frequencies to include only the 5 GHz channels
        mask = (frequencies >= 5180) & (frequencies <= 5825)
        freq_5ghz = frequencies[mask]
        channels = [map_freq_to_channel(freq) for freq in freq_5ghz]

        # Normalize the quality values
        quality_normalized = (channel_quality[mask] - (-1)) / (1 - (-1))

        if not self.socket:
            print("Error: Socket is not initialized.")
            return

        try:
            data = {'action': 'new_estimation', 'channels': channels, 'channel_quality': quality_normalized.tolist()}
            json_str = json.dumps(data)
            self.socket.send(json_str.encode())
        except ConnectionRefusedError:
            print("Error: Connection refused by the server. Check if the server is running and reachable.")
        except socket.error as e:
            print(f"Error: Socket error occurred: {e}")

    def estimate(self, feat_array: np.ndarray, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimates the channel quality for a given feature array.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :param frequencies: A 1D NumPy array of shape (n_channels,) containing the frequencies of the channels.
        :return: A tuple containing the estimated channel quality scores (1D NumPy array) and the class probabilities
        (2D NumPy array).
        """
        # Compute class probabilities for the given features
        probs = self._forward(feat_array)
        # Compute the channel quality
        channel_quality = self._compute_channel_quality(probs)

        # Send channel quality data to the server
        self.send_data(channel_quality, frequencies)

        return channel_quality, probs
