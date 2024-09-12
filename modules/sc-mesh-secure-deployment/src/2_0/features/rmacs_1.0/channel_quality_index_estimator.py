import numpy as np
from typing import Tuple
import re

class ChannelQualityIndexEstimator:
    def __init__(self):
        pass

    def _check_arrays(self, feat_array: np.ndarray, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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

    def channel_estimate(self, feat_array: np.ndarray, frequencies: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Estimates the channel quality for a given feature array.

        :param feat_array: A 2D NumPy array of shape (n_samples, n_features) containing the input features.
        :param frequencies: A 1D NumPy array of shape (n_channels,) containing the frequencies of the channels.
        :return: A tuple containing the estimated channel quality scores (1D NumPy array) and the class probabilities
        (2D NumPy array).
        """
        feat_array, frequencies = self._check_arrays(feat_array, frequencies)

        # Check if feat_array and frequencies arrays are empty
        if feat_array.size == 0 or frequencies.size == 0:
            return np.array([]), np.empty((0, 0)), np.array([])
        
        try:
            # Extract the rssi value from the feat array 
            rssi_array = np.array([int(re.search(r'rssi (\d+)', entry).group(1)) for entry in feat_array])
            # Average the rssi value
            avg_rssi = np.mean(rssi_array)
            print(f'The avg_rssi value is :{avg_rssi}')
            

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
        
    import numpy as np

    def detect_interference_statistically(rssi_data, max_mag_data, relpwr_data, window_size=10):
        # Calculate rolling mean and variance for RSSI
        rssi_mean = np.mean(rssi_data[-window_size:])
        rssi_variance = np.var(rssi_data[-window_size:])
        
        # Calculate rolling mean and variance for Max Magnitude
        max_mag_mean = np.mean(max_mag_data[-window_size:])
        max_mag_variance = np.var(max_mag_data[-window_size:])
        
        # Calculate rolling mean and variance for Relative Power
        relpwr_mean = np.mean(relpwr_data[-window_size:])
        relpwr_variance = np.var(relpwr_data[-window_size:])
        
        # Define thresholds based on observation and analysis
        rssi_mean_threshold = 20
        rssi_var_threshold = 10
        
        max_mag_mean_threshold = 40
        max_mag_var_threshold = 15
        
        relpwr_mean_threshold = 5
        relpwr_var_threshold = 3
        
        # If both mean and variance exceed thresholds, classify as interference
        if (rssi_mean > rssi_mean_threshold and rssi_variance > rssi_var_threshold and
            max_mag_mean > max_mag_mean_threshold and max_mag_variance > max_mag_var_threshold and
            relpwr_mean > relpwr_mean_threshold and relpwr_variance > relpwr_var_threshold):
            return "Interference Detected"
        else:
            return "Idle"
    
    
    