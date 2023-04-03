import copy
import glob
import os

import numpy as np
import pandas as pd
from sklearn.utils import shuffle

from util import FEATURES, STAT_FEATURES, CHANNELS, TRUNCATE2, TRUNCATE5, LABELS


# Split the data into three classes: jammed, not jammed, interference
def preprocess_raw_files(dataset_path='raw_dataset', storing_folder='preprocessed'):
    # Check if preprocessed_dataset data is created
    if not os.path.exists(storing_folder):
        os.mkdir(storing_folder)

        # Dict to store preprocessed data (7 statistical over the extracted values over time of a given channel)
        empty_dict = {}
        for feature in FEATURES:
            for stats_name in STAT_FEATURES:
                empty_dict[f'{feature}_{stats_name}'] = []
        empty_dict['bin_label'] = -1
        empty_dict['label'] = -1

        # Loop through jammed files and extract features
        for folder in ['jamming/2.4', 'jamming/5.0']:
            features = copy.deepcopy(empty_dict)
            for file_name in glob.glob(f'{dataset_path}/{folder}/*.csv'):
                print(file_name)
                df = pd.read_csv(file_name)
                # Ignore sometimes corrupted first row
                df = df.iloc[1:, :]
                # Define jammed channel given the file name
                target_channel = int(file_name.split('samples_chamber_')[1][:4])
                # Compute statistics for each channel
                df_channel = df.loc[df['freq1'] == target_channel]
                # Define truncate value given band
                trunc_val = TRUNCATE2 if target_channel < 3000 else TRUNCATE5
                # Truncate spectral scan sample window to a fixed size
                trunc_start = len(df_channel) - trunc_val
                if trunc_start > 0:
                    df_channel = df.loc[df['freq1'] == target_channel].iloc[trunc_start:]
                    # Extract statistical features for each feature
                    for feature in FEATURES:
                        stats = df_channel[feature].describe(include='all')
                        stats['mad'] = df_channel[feature].mad()
                        for stats_name in STAT_FEATURES:
                            features[f'{feature}_{stats_name}'].append(stats[stats_name])
            features['bin_label'] = 1
            features['label'] = LABELS['jamming']
            final_df = pd.DataFrame(data=features)
            file_name = '24_features.csv' if folder == 'jamming/2.4' else '50_features.csv'
            final_df.to_csv(f'{storing_folder}/jamming_{file_name}', index=False)

        # Loop through noise floor and normal files and extract features
        for folder in ['floor', 'inter_mid', 'inter_high']:
            features2, features5 = copy.deepcopy(empty_dict), copy.deepcopy(empty_dict)
            for file_name in glob.glob(f'{dataset_path}/{folder}/*.csv'):
                print(file_name)
                df = pd.read_csv(file_name)
                # Ignore sometimes corrupted first row
                df = df.iloc[1:, :]
                # Loop through each channel
                for channel in CHANNELS:
                    features = features2 if channel < 3000 else features5
                    # Compute statistics for each channel
                    df_channel = df.loc[df['freq1'] == channel]
                    # Define truncate value given band
                    trunc_val = TRUNCATE2 if channel < 3000 else TRUNCATE5
                    # Truncate spectral scan sample window to a fixed size
                    trunc_start = len(df_channel) - trunc_val
                    if trunc_start > 0:
                        df_channel = df.loc[df['freq1'] == channel].iloc[trunc_start:]
                        # Extract statistical features for each feature
                        for feature in FEATURES:
                            stats = df_channel[feature].describe(include='all')
                            stats['mad'] = df_channel[feature].mad()
                            for stats_name in STAT_FEATURES:
                                features[f'{feature}_{stats_name}'].append(stats[stats_name])
            features2['bin_label'] = 0
            features2['label'] = LABELS[folder]
            final_df = pd.DataFrame(data=features2)
            final_df.to_csv(f'{storing_folder}/{folder}_24_features.csv', index=False)
            features5['bin_label'] = 0
            features5['label'] = LABELS[folder]
            final_df = pd.DataFrame(data=features5)
            final_df.to_csv(f'{storing_folder}/{folder}_50_features.csv', index=False)


def train_test_split(storing_folder='preprocessed'):
    # Check if preprocessed_dataset data is created
    if not os.path.exists('train_test_split'):
        os.mkdir('train_test_split')
        # Split data into train and test
        train_df, test_df = None, None
        for file_name in glob.glob(f'{storing_folder}/*.csv'):
            df = pd.read_csv(file_name)
            msk = np.random.rand(len(df)) < 0.8
            train_df = df[msk] if train_df is None else pd.concat([train_df, df[msk]])
            test_df = df[~msk] if train_df is None else pd.concat([test_df, df[~msk]])
        train_df, test_df = shuffle(train_df), shuffle(test_df)

        train_df.to_csv('train_test_split/train.csv', index=False)
        test_df.to_csv('train_test_split/test.csv', index=False)


def main():
    np.random.seed(0)
    preprocess_raw_files()
    train_test_split()


if __name__ == '__main__':
    main()
