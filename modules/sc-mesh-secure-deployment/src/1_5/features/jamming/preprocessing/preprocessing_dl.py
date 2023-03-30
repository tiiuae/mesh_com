import glob
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.express as px
import tsaug
from scipy.signal import butter, filtfilt
from sklearn.model_selection import train_test_split

from util import SEED, NUM_MEASUREMENT, ORDERED_COLS, CHANNELS

normal_folders = ['floor', 'inter_mid', 'inter_high']
# normal_folders = ['inter_mid']


def plot_timeserie(original: np.ndarray, filtered: np.ndarray, resized: np.ndarray):
    colors = px.colors.qualitative.G10

    fig, ax = plt.subplots(3, 1, figsize=(10, 15))

    ax[0].plot(original, color=colors[0], label='Original')
    ax[1].plot(filtered, color=colors[0], label='Original')
    ax[2].plot(resized, color=colors[1], label='After filter')

    # Show the plot
    plt.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.25))
    plt.tight_layout()
    plt.show()
    # plt.savefig('plots/augmented.pdf', dpi=300)
    quit()


def resize_to_length(df: pd.DataFrame, filter=False):
    df = df.copy()
    df_down = pd.DataFrame()
    freq1 = df['freq1'].iloc[0]
    for col in df.columns:
        if col not in ['freq1', 'freq2', 'tsf']:
            if filter:
                # filtered = savgol_filter(df[col].to_numpy(), 10, 3)

                # # Define the sampling frequency and cutoff frequency for the filter
                fs = 100  # Sampling frequency (Hz)
                fc = 10  # Cutoff frequency (Hz)

                # Define the order of the filter
                order = 3

                # Define the coefficients of the low-pass filter using a Butterworth filter
                nyq = 0.5 * fs
                cutoff = fc / nyq
                b, a = butter(order, cutoff, btype='low', analog=False)

                # Apply the low-pass filter to the time series data
                filtered_data = filtfilt(b, a, df[col].to_numpy())

                df_down[col] = tsaug.Resize(size=NUM_MEASUREMENT).augment(filtered_data)
            else:
                df_down[col] = tsaug.Resize(size=NUM_MEASUREMENT).augment(df[col].to_numpy())

    df_down['freq1'] = freq1
    assert len(df_down) == NUM_MEASUREMENT, 'wrong length'
    return df_down


# Split the data into three classes: jammed, not jammed, interference
def preprocess_raw_files(dataset_path='raw_dataset', storing_folder='preprocessed'):
    series_id = 0

    # Check if preprocessed_dataset data is created
    if not os.path.exists(storing_folder):
        os.mkdir(storing_folder)

    # Loop through noise floor and normal files and extract features
    for folder in normal_folders:
        all_series = []
        for file_name in glob.glob(f'{dataset_path}/{folder}/*.csv'):
            print(file_name)
            df = pd.read_csv(file_name)
            # Ignore sometimes corrupted first row
            df = df.iloc[1:-1]
            # Loop through each channel
            for channel in CHANNELS:
                # Filter measurements for a particular channel within file
                df_channel = df.loc[df['freq1'] == channel]

                # Ignore timeseries with low number of measurements
                if len(df_channel) < 64:
                    continue

                # Down sample by interpolating mean
                df_serie = resize_to_length(df_channel)

                # Set serie id
                df_serie['series_id'] = series_id
                df_serie['bin_label'] = 0
                df_serie['label'] = folder

                # Concatenate
                all_series.append(df_serie)
                series_id += 1

        all_series = pd.concat(all_series, ignore_index=True)
        all_series = all_series[ORDERED_COLS]
        all_series.to_csv(f'{storing_folder}/{folder}.csv', index=False)

    all_series = []
    # Loop through jammed files and extract features
    for folder in ['jamming/2.4', 'jamming/5.0']:
        for file_name in glob.glob(f'{dataset_path}/{folder}/*.csv'):
            print(f'Processing file {file_name}')
            df = pd.read_csv(file_name)

            # Ignore sometimes corrupted first row
            df = df.iloc[1:, :]

            # Define jammed channel given the file name
            target_channel = int(file_name.split('samples_chamber_')[1][:4])

            # Filter measurements for a particular channel within file
            df_channel = df.loc[df['freq1'] == target_channel]

            # Ignore timeseries with low number of measurements
            if len(df_channel) < 64:
                continue

            # Down sample by interpolating mean
            df_serie = resize_to_length(df_channel)

            # Set series id
            df_serie['series_id'] = series_id
            series_id += 1

            # Label it
            df_serie['bin_label'] = 1
            split_filename = file_name.split("_")
            frequency = int(split_filename[3].split("MHz")[0])
            frequency = '2' if frequency < 3000 else '5'
            distance = split_filename[4].split("cm")[0]
            power_level = split_filename[5].split("dBm")[0]
            # df_serie['label'] = labels[frequency][distance][power_level]
            df_serie['label'] = f'jam_{frequency}GHz_{distance}cm_{power_level}dBm'

            # Concatenate
            all_series.append(df_serie)

    all_series = pd.concat(all_series, ignore_index=True)
    all_series = all_series[ORDERED_COLS]
    all_series.to_csv(f'{storing_folder}/jamming.csv', index=False)


def store_all_data(storing_folder='preprocessed'):
    all_data = None
    for file_name in normal_folders + ['jamming']:
        print(f'Loading {storing_folder}/{file_name}.csv')
        df = pd.read_csv(f'{storing_folder}/{file_name}.csv')
        all_data = df if all_data is None else pd.concat([all_data, df])
    all_data.to_csv(f'{storing_folder}/all_series.csv', index=False)


def split_data(storing_folder='preprocessed'):
    # Check if preprocessed_dataset data is created
    if not os.path.exists('train_test_split'):
        os.mkdir('train_test_split')

    # Split data into train and test
    dataset = None
    for file_name in normal_folders + ['jamming']:
        print(f'Loading {storing_folder}/{file_name}.csv')
        df = pd.read_csv(f'{storing_folder}/{file_name}.csv')
        dataset = df if dataset is None else pd.concat([dataset, df])

    # Get all unique series_id values
    series_ids = dataset['series_id'].unique()
    # Split the series_ids into training and testing sets
    training_series_ids, testing_series_ids = train_test_split(series_ids, test_size=0.2, random_state=SEED)
    # Create the training and testing dataframes
    training = dataset[dataset['series_id'].isin(training_series_ids)]
    testing = dataset[dataset['series_id'].isin(testing_series_ids)]
    # Save splits
    training.to_csv('train_test_split/train.csv', index=False)
    testing.to_csv('train_test_split/test.csv', index=False)


def main():
    np.random.seed(0)
    preprocess_raw_files()
    store_all_data()
    # split_data()


if __name__ == '__main__':
    main()
