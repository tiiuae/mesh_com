import pandas
import pandas as pd
import torch.nn.functional
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from torch.utils.data import DataLoader, TensorDataset, Dataset
from torch.utils.data import WeightedRandomSampler

import sys

from util import *


class CustomDataset(Dataset):
    def __init__(self, raw, fft, bin_target, target):
        self.raw = raw
        self.fft = fft
        self.bin_target = bin_target
        self.target = target
        self.n_samples = raw.shape[0]

    def __getitem__(self, index):
        serie = self.raw[index]
        shape = serie.shape

        # Plot augmentations
        # plot_timeserie_augment(serie)

        # Perform augmentation
        aug = serie.view(1, shape[0], shape[1])
        aug = random_crop_resize(aug, size=shape[1], scale=(0.1, 1.0))
        if np.random.rand() < 0.5: aug = horizontal_flip(aug)
        # if np.random.rand() < 0.5: aug = cut_out(aug) # Test accuracy multiclass: 0.9979959919839679, bin accuracy: 1.0

        # if np.random.rand() < 0.5:
        #     if np.random.rand() < 0.5:
        #         aug = gaussian_noise(aug, magnitude=0.03, additive=True)
        #     else:
        #         aug = gaussian_noise(aug, magnitude=0.03, additive=False)

        # aug = time_warp(aug, magnitude=0.1, order=6)
        # aug = window_warp(aug, magnitude=0.1)

        # Bring back shape
        aug = aug.view(shape[0], shape[1])

        return aug, self.bin_target[index], self.target[index]

    def __len__(self):
        return self.n_samples


def normalize_df(df: pandas.DataFrame):
    cols_mean_std = {}
    df_norm = df.copy()
    for col in df_norm.columns:
        if col in ['series_id', 'freq1', 'bin_label', 'label']:
            continue
        cols_mean_std[col] = {'mean': 0, 'std': 0}

        cols_mean_std[col]['mean'] = df_norm[col].mean()
        cols_mean_std[col]['std'] = df_norm[col].std()
        df_norm[col] = (df_norm[col] - cols_mean_std[col]['mean']) / cols_mean_std[col]['std']
    print('Normalization variables', cols_mean_std)
    return df_norm


def feature_engineer(df, eps=sys.float_info.epsilon):
    # Compute SNR
    # df['snr'] = 10 * np.log10((df['max_magnitude'] ** 2 / (df['rssi'] + eps)))
    # df['snr1'] = df['rssi'] - df['avgpwr_db']
    df['snr'] = 10 * np.log10(np.maximum((df['max_magnitude'] ** 2 - df['noise']) / (df['rssi'] + eps), eps))

    # Carrier-to-noise ratio (CNR)
    df['cnr'] = df['rssi'] - df['noise']

    # Phase noise (PN)
    df['pn'] = 10 * np.log10(1e-10 + df['max_magnitude'] ** 2 / (2 * df['freq1'] * df['snr'] + eps))

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

    #  Power ratio (PR) between average power and base power:
    df['pr'] = df['avgpwr_db'] / (df['base_pwr_db'] + eps)

    return df


# bounds_jamm_window denote lb and ub for the size of jam window that will be interpolated to generated jammed sampled
def impute_series(serie_type_1: pd.DataFrame, serie_type_2: pd.DataFrame, bounds_jamm_window: tuple = (0.25, 0.75)):
    seq_len = len(serie_type_1)
    lambd = np.random.uniform(bounds_jamm_window[0], bounds_jamm_window[1])
    win_len = int(round(seq_len * lambd))

    # Find start of window
    start = np.random.randint(0, seq_len - win_len)

    # Extract window from serie_type_2
    window = serie_type_2[start:start + win_len].copy()

    # Replace corresponding segment in serie_type_1 with window
    new_serie = serie_type_1.copy()
    new_serie.iloc[start:start + win_len] = window

    return new_serie


def generate_data_imputation(df, amount=2000, label='simulated', plot=False):
    num_series = df['series_id'].nunique()

    normal2 = df[(df['bin_label'] == 0) & (pd.to_numeric(df['freq1']) < 3000) & (~df['label'].str.contains('inter_high'))]
    jamming2 = df[(df['bin_label'] == 1) & (pd.to_numeric(df['freq1']) < 3000) & (~df['label'].str.contains('_0dBm'))]
    normal5 = df[(df['bin_label'] == 0) & (pd.to_numeric(df['freq1']) > 3000) & (~df['label'].str.contains('inter_high'))]
    jamming5 = df[(df['bin_label'] == 1) & (pd.to_numeric(df['freq1']) > 3000) & (~df['label'].str.contains('_0dBm'))]

    # ignore jam 5 60cm 0dbm
    for i in range(amount):
        r = np.random.rand()
        normal = normal2 if r < 0.5 else normal5
        jamming = jamming2 if r < 0.5 else jamming5

        norm_serie_id = normal.sample(n=1, random_state=np.random.randint(0, 10000))['series_id'].values[0]
        norm_serie = normal[normal['series_id'] == norm_serie_id]
        freq1 = norm_serie['freq1'].iloc[0]

        jamming_freq = jamming[jamming['freq1'] == freq1]
        jam_serie_id = jamming_freq.sample(n=1, random_state=np.random.randint(0, 10000))['series_id'].values[0]
        jam_serie = jamming_freq[jamming_freq['series_id'] == jam_serie_id]

        new_serie = impute_series(norm_serie, jam_serie)
        new_serie['series_id'] = num_series
        new_serie['freq1'] = freq1
        new_serie['label'] = label
        new_serie['bin_label'] = 1

        df = pd.concat([df, new_serie], ignore_index=True)
        num_series += 1

        if plot:
            colors = px.colors.qualitative.G10
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))

            ax.plot(norm_serie['rssi'].to_numpy(), color=colors[0], label=norm_serie['label'].to_numpy()[0])
            ax.plot(jam_serie['rssi'].to_numpy(), color=colors[1], label=jam_serie['label'].to_numpy()[0])
            ax.plot(new_serie['rssi'].to_numpy(), color=colors[2], label=new_serie['label'].to_numpy()[0])
            ax.set_xlim(0, 128)

            plt.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.25))
            plt.tight_layout()
            plt.show()

    return df


def load_data(plot=False):
    if not os.path.exists('../data/preprocessed/enhanced.csv'):
        # Load data and compute number of series within it
        df = pd.read_csv(f'../data/preprocessed/all_series.csv')

        # Generate additional jamming samples by imputation of normal and jamming time series
        df = generate_data_imputation(df)
        df.to_csv('../data/preprocessed/enhanced.csv')

    # Load enhanced data with simulated data
    df = pd.read_csv('../data/preprocessed/enhanced.csv')

    # Expand amount of features by engineering some
    df = feature_engineer(df)

    # Reorder columns
    reorder_cols = ['series_id', 'freq1', 'max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db', 'snr', 'cnr', 'pn', 'ssi', 'pd', 'sinr', 'sir',
                    'mr', 'pr', 'bin_label', 'label']
    df = df[reorder_cols]

    # Encode labels
    label_encoder = LabelEncoder()
    df['label'] = label_encoder.fit_transform(df['label'])

    # simulated_label = label_encoder.transform(['simulated'])
    # num_classes = len(label_encoder.classes_)
    # print('simulated_label', simulated_label)
    # print('num_classes', num_classes)

    # Normalize data
    df_norm = normalize_df(df)

    # Columns that will be used as features
    feat_10k = ['max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db', 'snr', 'cnr', 'pn', 'ssi', 'pd', 'sinr', 'sir', 'mr', 'pr']
    print(len(feat_10k))
    feat_cols = feat_10k

    # Transform df into np.array
    num_series = df['series_id'].nunique()
    feat_array = np.array(df_norm[feat_cols])
    feat_array = np.reshape(feat_array, [num_series, NUM_MEASUREMENT, len(feat_cols)])
    bin_target_array = np.array(df_norm['bin_label'])
    bin_target_array = np.reshape(bin_target_array, [num_series, NUM_MEASUREMENT])
    bin_target_array = bin_target_array[:, 0]
    target_array = np.array(df_norm['label'])
    target_array = np.reshape(target_array, [num_series, NUM_MEASUREMENT])
    target_array = target_array[:, 0]

    # Finally, transpose the data to we get [samples, features, time] array
    feat_array = feat_array.transpose(0, 2, 1)

    # Compute FFT for each serie
    feat_fft_array = np.abs(np.fft.rfft(np.copy(feat_array)))

    print('Raw shape:', feat_array.shape)
    print('FFT shape:', feat_array.shape)
    print('Number of classes:', len(np.unique(target_array)))

    if plot:
        plot_timeseries(feat_array, target_array, label_encoder, feat_cols)
        # plot_fft(feat_array, feat_fft_array)
        # plot_timeseries_augment(feat_array, target_array, label_encoder)
        quit()

    return (feat_array, feat_fft_array, bin_target_array, target_array), label_encoder


def create_datasets(data, valid_pct=0.1, test_pct=0.2):
    raw, fft, bin_target, target = data
    assert len(raw) == len(fft), 'raw not same length as fft arrays'

    idx = np.arange(raw.shape[0])
    train_idx, test_idx = train_test_split(idx, test_size=test_pct, random_state=SEED)
    train_ds = TensorDataset(
        torch.tensor(raw[:][train_idx]).float(),
        torch.tensor(fft[:][train_idx]).float(),
        torch.tensor(bin_target[:][train_idx]).long(),
        torch.tensor(target[:][train_idx]).long()
    )
    test_ds = TensorDataset(
        torch.tensor(raw[:][test_idx]).float(),
        torch.tensor(fft[:][test_idx]).float(),
        torch.tensor(bin_target[:][test_idx]).long(),
        torch.tensor(target[:][test_idx]).long()
    )

    idx = np.arange(len(train_ds))
    train_idx, valid_idx = train_test_split(idx, test_size=valid_pct, random_state=SEED)
    train_ds = CustomDataset(
        torch.tensor(raw[:][train_idx]).float(),
        torch.tensor(fft[:][train_idx]).float(),
        torch.tensor(bin_target[:][train_idx]).long(),
        torch.tensor(target[:][train_idx]).long()
    )
    val_ds = TensorDataset(
        torch.tensor(raw[:][valid_idx]).float(),
        torch.tensor(fft[:][valid_idx]).float(),
        torch.tensor(bin_target[:][valid_idx]).long(),
        torch.tensor(target[:][valid_idx]).long()
    )

    print(f'Train dataset length: {len(train_ds)}')
    print(f'Val. dataset length: {len(val_ds)}')
    print(f'Test dataset length: {len(test_ds)}')

    return train_ds, val_ds, test_ds


def create_resampler(label_encoder, train_ds):
    # Calculate the frequency of each class in the dataset:
    class_freq = [0] * len(label_encoder.classes_)
    for x_raw, y_bin_batch, y_batch in train_ds:
        class_freq[y_batch.item()] += 1

    # Calculate the weight of each class, which is the inverse of the frequency:
    weights = 1. / torch.tensor(class_freq, dtype=torch.float)
    samples_weights = weights[train_ds.target]
    sampler = WeightedRandomSampler(weights=samples_weights, num_samples=len(train_ds), replacement=True)

    return sampler


def create_loaders(data, label_encoder, batch_size, jobs=0):
    train_ds, val_ds, test_ds = data

    sampler = create_resampler(label_encoder, train_ds)
    train_dl = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=jobs, drop_last=True)
    # train_dl = DataLoader(train_ds, batch_size=batch_size, num_workers=jobs, drop_last=True, sampler=sampler)
    val_dl = DataLoader(val_ds, batch_size=batch_size, shuffle=False, num_workers=jobs)
    test_dl = DataLoader(test_ds, batch_size=batch_size, shuffle=False, num_workers=jobs)
    return train_dl, val_dl, test_dl


def get_classification_data():
    data, label_encoder = load_data()
    datasets = create_datasets(data)
    len_train_data = len(datasets[0])
    loaders = create_loaders(datasets, label_encoder, batch_size=BATCH_SIZE)

    return loaders, len_train_data, label_encoder
