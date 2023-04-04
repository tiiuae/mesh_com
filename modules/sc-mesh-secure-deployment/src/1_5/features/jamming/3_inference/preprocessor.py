"""
preprocessor.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License: 

Repository:  
"""

import json
import sys
from typing import Optional, Tuple

import numpy as np
import pandas as pd

from transforms import resize
from util import FEATS_ATH10K


class Preprocessor:
    def __init__(self, input_length: int = 128, col_mean_std_path: Optional[str] = 'normalization_data/cols_mean_std.json') -> None:
        """
        Initializes the Preprocessor object.

        :param input_length: The desired length of the spectral scan data after resizing.
        :param col_mean_std_path: The path to the JSON file containing the mean and standard deviation information for
                                  the columns to be normalized. Defaults to 'cols_mean_std.json'.
        """
        self.input_length = input_length

        # Load the mean and standard deviation information from the JSON file
        with open(col_mean_std_path, 'r') as file:
            json_data = json.load(file)
        self.cols_mean_std = json_data

    def feature_engineer(self, df: pd.DataFrame, eps: float = sys.float_info.epsilon) -> pd.DataFrame:
        """
        Computes additional features based on the given spectral scan data.

        :param df: The spectral scan data to compute features for.
        :param eps: A small value added to the denominator of some computations to avoid division by zero.
        :return: The spectral scan data with additional computed features.
        """
        # Compute SNR
        df['snr'] = 10 * np.log10(np.maximum((df['max_magnitude'] ** 2 - df['noise']) / (df['rssi'] + eps), eps))

        # Carrier-to-noise ratio (CNR)
        df['cnr'] = df['rssi'] - df['noise']

        # Phase noise (PN)
        df['pn'] = 10 * np.log10(np.maximum(df['max_magnitude'] ** 2 / (2 * df['freq1'] * df['snr'] + eps), eps))

        # Signal strength indicator (SSI)
        df['ssi'] = df['rssi'] - df['relpwr_db']

        # Power Difference (PD)
        df['pd'] = df['base_pwr_db'] - df['avgpwr_db']

        # Signal-to-interference-plus-noise ratio (SINR):
        df['sinr'] = df['rssi'] - df['relpwr_db']

        # Signal-to-interference ratio (SIR):
        df['sir'] = df['rssi'] - (df['total_gain_db'] - df['relpwr_db'])

        # Magnitude ratio (MR) between maximum magnitude and average power:
        df['mr'] = df['max_magnitude'] / (df['avgpwr_db'] + eps)

        # Power ratio (PR) between average power and base power:
        df['pr'] = df['avgpwr_db'] / (df['base_pwr_db'] + eps)

        return df

    def resize(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Resizes the given spectral scan data to a fixed input length.

        :param df: The spectral scan data to resize.
        :return: The resized spectral scan data.
        """
        df = df.copy()
        df_down = pd.DataFrame()
        freq1 = df['freq1'].iloc[0]
        for col in df.columns:
            if col not in ['freq1', 'freq2', 'tsf']:
                df_down[col] = resize(df[col].to_numpy(), self.input_length)

        df_down['freq1'] = freq1
        assert len(df_down) == self.input_length, 'wrong length'
        return df_down

    def normalize(self, df: pd.DataFrame) -> np.ndarray:
        """
        Normalizes the given spectral scan data to have a mean of 0 and a standard deviation of 1 using the
        mean and standard deviation information from the col_mean_std.json file.

        :param df: The spectral scan data to normalize.
        :return: The normalized spectral scan data as a NumPy array.
        """
        for col in df.columns:
            if col in ['freq1']:
                continue
            df[col] = (df[col] - self.cols_mean_std[col]['mean']) / self.cols_mean_std[col]['std']

        return df

    def preprocess(self, df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
        """
        Preprocesses the given spectral scan data by computing features, resizing, and normalizing it.

        :param df: The spectral scan data to preprocess.
        :return: The preprocessed spectral scan data as a NumPy array.
        """
        # Ignore first and last rows, sometimes corrupted
        df = df.iloc[1:-1].copy()

        all_series = []
        # Get unique frequencies
        unique_freqs = df['freq1'].unique()

        # Resize time series to fixed length
        for freq in unique_freqs:
            df_channel = df.loc[df['freq1'] == freq]
            resized_channel = self.resize(df_channel)
            all_series.append(resized_channel)
        df = pd.concat(all_series, ignore_index=True)

        # Compute additional features
        df = self.feature_engineer(df)

        # Drop irrelevant columns
        cols = ['freq1', 'max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db', 'snr', 'cnr', 'pn', 'ssi', 'pd', 'sinr', 'sir', 'mr', 'pr']
        df = df[cols]

        # Normalize so cols have mean 0 and std 1
        df_norm = self.normalize(df)

        # Transform df into np.array
        num_series = len(df_norm['freq1'].unique())
        feat_array = np.array(df_norm[FEATS_ATH10K])
        feat_array = np.reshape(feat_array, [num_series, self.input_length, len(FEATS_ATH10K)])

        # Finally, transpose the data to we get [samples, features, time] array
        feat_array = feat_array.transpose(0, 2, 1)

        return feat_array, unique_freqs
