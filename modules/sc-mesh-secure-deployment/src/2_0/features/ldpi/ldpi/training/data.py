import os
import random
from typing import List, Optional
from typing import Tuple

import numpy as np
import pandas as pd
import torch
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler

import transforms

# Type aliases
ArrayFloat = np.ndarray
ArrayInt = np.ndarray
DataFrame = pd.DataFrame
DataLoaderType = DataLoader


class CustomDataset(Dataset):
    """
    A custom dataset class for handling samples and targets.

    Attributes:
        samples (ArrayFloat): A numpy array of samples.
        targets (ArrayInt): A numpy array of target values.
        bin_targets (ArrayInt): A numpy array of binary target values.
        n_samples (int): Number of samples in the dataset.
    """

    def __init__(self, samples: ArrayFloat, targets: ArrayInt, bin_targets: ArrayInt):
        """
        Initializes the CustomDataset with samples and targets.

        Args:
            samples (ArrayFloat): A numpy array of samples.
            targets (ArrayInt): A numpy array of target values.
            bin_targets (ArrayInt): A numpy array of binary target values.
        """
        self.samples = samples
        self.targets = targets
        self.bin_targets = bin_targets
        self.n_samples = samples.shape[0]

    def __getitem__(self, index: int) -> Tuple[ArrayFloat, ArrayFloat, ArrayFloat]:
        """
        Returns the sample and target values at the specified index.

        Args:
            index (int): Index of the desired sample.

        Returns:
            Tuple[ArrayFloat, int, int]: A tuple containing the sample, target, and binary target at the given index.
        """
        return self.samples[index], self.targets[index], self.bin_targets[index]

    def __len__(self) -> int:
        """
        Returns the total number of samples in the dataset.

        Returns:
            int: Total number of samples.
        """
        return self.n_samples


class OneClassContrastiveDataset(Dataset):
    """
    A dataset class for one-class contrastive learning, with synthetic outliers and data augmentation techniques.

    Attributes:
        samples (ArrayFloat): A numpy array of samples.
        targets (ArrayInt): A numpy array of target values.
        bin_targets (ArrayInt): A numpy array of binary target values.
        n_samples (int): Number of samples in the dataset.
        outlier_noisy (ArrayFloat): Synthetic outliers with injected random noise.
        outlier_reversed (ArrayFloat): Synthetic outliers with reversed sequences.
        outlier_shuffled (ArrayFloat): Synthetic outliers with shuffled sequences.
        outlier_inverted (ArrayFloat): Synthetic outliers with inverted values.
    """

    def __init__(self, samples: np.ndarray, targets: np.ndarray, bin_targets: np.ndarray):
        """
        Initializes the OneClassContrastiveDataset with samples, targets, and synthetic outliers.

        Args:
            samples (np.ndarray): A numpy array of samples.
            targets (np.ndarray): A numpy array of target values.
            bin_targets (np.ndarray): A numpy array of binary target values.
        """
        self.samples = samples
        self.targets = targets
        self.bin_targets = bin_targets
        self.n_samples = samples.shape[0]

        # Create fake outliers
        self.outlier_noisy = transforms.inject_random_noise(samples)
        self.outlier_reversed = transforms.reverse_sequences(samples)
        self.outlier_shuffled = transforms.shuffle_sequences(samples)
        self.outlier_inverted = transforms.invert_values(samples)

    def transform(self, sample: ArrayFloat) -> ArrayFloat:
        """
        Applies random data augmentation techniques to a sample.

        Args:
            sample (ArrayFloat): A numpy array of a sample.

        Returns:
            ArrayFloat: Transformed sample.
        """
        # Randomly select an augmentation to apply
        augs = [transforms.temporal_scaling, transforms.jittering, transforms.random_segment_permutation]
        augmented = np.random.choice(augs)(sample)
        augmented = transforms.crop_and_resize(augmented)
        return augmented

    def __getitem__(self, index: int) -> Tuple:
        """
        Returns a pair of views (two transformed sample) for contrastive learning.

        Args:
            index (int): Index of the desired sample pair.

        Returns:
            Tuple: A tuple containing two transformed samples.
        """
        if random.random() < 0.5:
            sample = self.samples[index]
        else:
            outlier_choice = random.choice([self.outlier_noisy, self.outlier_reversed, self.outlier_shuffled, self.outlier_inverted])
            sample = outlier_choice[index]
        v1, v2 = self.transform(sample), self.transform(sample)
        v1, v2 = torch.from_numpy(v1).float(), torch.from_numpy(v2).float()
        return v1, v2

    def __len__(self) -> int:
        """
        Returns the total number of samples in the dataset.

        Returns:
            int: Total number of samples.
        """
        return self.n_samples


def make_weights_for_balanced_classes(targets: ArrayInt) -> List[float]:
    """
    Calculates weights for balanced classes based on target values.

    Args:
        targets (ArrayInt): A numpy array of target values.

    Returns:
        List[float]: A list of weights for each class to balance the classes.
    """
    count = {1: 0, -1: 0}
    for item in targets:
        count[item] += 1

    N = float(sum(count.values()))
    weight_per_class = {1: N / float(count[1]) if count[1] > 0 else 0,
                        -1: N / float(count[-1]) if count[-1] > 0 else 0}

    return [weight_per_class[val] for val in targets]


def load_data_from_folder(dataset_name: str, category: str, counter: int) -> Tuple[int, List[ArrayFloat], List[ArrayInt]]:
    """
    Load data from a folder based on dataset name and category.

    Args:
        dataset_name (str): Name of the dataset.
        category (str): Category of data ('benign' or 'malicious').
        counter (int): Counter to track label values.

    Returns:
        Tuple[int, List[ArrayFloat], List[ArrayInt]]: Updated counter, list of samples, and list of labels.
    """
    samples: List[ArrayFloat] = []
    targets: List[ArrayInt] = []

    parent_path = os.path.join('samples', dataset_name, 'pcap', category)

    # Loop through subdirectories
    for traffic_type in os.listdir(parent_path):
        traffic_path = os.path.join(parent_path, traffic_type)

        for sub_traffic_type in os.listdir(traffic_path):
            sub_traffic_path = os.path.join(traffic_path, sub_traffic_type)
            print(sub_traffic_path, counter)

            folder_data: Optional[ArrayFloat] = None
            for file_name in os.listdir(sub_traffic_path):
                if file_name.endswith('.npy'):
                    data = np.load(os.path.join(sub_traffic_path, file_name)).reshape(1, -1)
                    folder_data = data if folder_data is None else np.concatenate((folder_data, data))

            if folder_data is None:
                continue

            folder_data = folder_data.astype(np.float32) / 255.0
            samples.append(folder_data)
            labels = np.ones(folder_data.shape[0]) * counter

            targets.append(labels)
            counter += 1

    return counter, samples, targets


def load_data(dataset: str, test_size: float = 0.20, only_normal: bool = False) -> Tuple[ArrayFloat, ArrayInt, ArrayInt, ArrayFloat, ArrayInt, ArrayInt]:
    """
    Load data for training and testing.

    Args:
        dataset (str): Name of the dataset.
        test_size (float, optional): Size of the test set as a proportion of the dataset. Defaults to 0.20.
        only_normal (bool, optional): Whether to load only normal data. Defaults to False.

    Returns:
        Tuple[ArrayFloat, ArrayInt, ArrayInt, ArrayFloat, ArrayInt, ArrayInt]: Training and testing data and labels.
    """
    # Assuming load_data_from_folder is a function that loads the data
    # Replace with the actual data loading logic as necessary
    label_counter, benign_data, benign_labels = load_data_from_folder(dataset, 'benign', counter=0)

    if not only_normal:
        label_counter, malware_data, malware_labels = load_data_from_folder(dataset, 'malicious', counter=label_counter)
        anomaly = np.concatenate(malware_data).astype(np.float32)
        anomaly_targets = np.concatenate(malware_labels).astype(int)
    else:
        anomaly = np.array([]).astype(np.float32)
        anomaly_targets = np.array([]).astype(int)

    normal = np.concatenate(benign_data).astype(np.float32)
    normal_targets = np.concatenate(benign_labels).astype(int)

    data = np.concatenate((normal, anomaly)) if not only_normal else normal
    targets = np.concatenate((normal_targets, anomaly_targets)) if not only_normal else normal_targets
    bin_targets = np.concatenate((np.ones(normal.shape[0]), -np.ones(anomaly.shape[0]))) if not only_normal else np.ones(normal.shape[0])

    df = pd.DataFrame({'sample': data.tolist(), 'target': targets, 'bin_target': bin_targets})

    if only_normal:
        df = df[df['bin_target'] == 1]

    if test_size > 0:
        train, test = train_test_split(df, test_size=test_size, random_state=42)
    else:
        # When test_size is 0, use all data for training and create empty test sets
        train = df
        test = pd.DataFrame(columns=['sample', 'target', 'bin_target'])

    train_samples = np.array(train['sample'].tolist()).astype(np.float32)
    train_targets = np.array(train['target'].tolist()).astype(int)
    train_bin_targets = np.array(train['bin_target'].tolist()).astype(int)

    test_samples = np.array(test['sample'].tolist()).astype(np.float32)
    test_targets = np.array(test['target'].tolist()).astype(int)
    test_bin_targets = np.array(test['bin_target'].tolist()).astype(int)

    return train_samples, train_targets, train_bin_targets, test_samples, test_targets, test_bin_targets


def get_training_dataloader(dataset: str, batch_size: int = 64) -> Tuple[DataLoaderType, DataLoaderType]:
    """
    Get training and testing data loaders for the specified dataset.

    Args:
        dataset (str): Name of the dataset.
        batch_size (int, optional): Batch size for training. Defaults to 64.

    Returns:
        Tuple[DataLoaderType, DataLoaderType]: Training and testing data loaders.
    """
    train_samples, train_targets, train_bin_targets, test_samples, test_targets, test_bin_targets = load_data(dataset, only_normal=False)
    print(f'{train_samples.shape[0]} training samples')
    print(train_samples.shape, test_samples.shape)

    train_ds = CustomDataset(train_samples, train_targets, train_bin_targets)
    test_ds = CustomDataset(test_samples, test_targets, test_bin_targets)

    weights = make_weights_for_balanced_classes(train_bin_targets)
    sampler = WeightedRandomSampler(torch.DoubleTensor(weights), len(weights))

    train_loader = DataLoader(dataset=train_ds, batch_size=batch_size, sampler=sampler, drop_last=True, num_workers=0)
    test_loader = DataLoader(dataset=test_ds, batch_size=batch_size, shuffle=False, drop_last=False, pin_memory=True)

    return train_loader, test_loader


def get_pretrain_dataloader(dataset: str, batch_size: int, contrastive: bool = False, shuffle: bool = True, drop_last: bool = True) -> DataLoaderType:
    """
    Get a pretraining data loader for the specified dataset.

    Args:
        dataset (str): Name of the dataset.
        batch_size (int): Batch size for pretraining.
        contrastive (bool, optional): Whether to use contrastive data augmentation. Defaults to False.
        shuffle (bool, optional): Whether to shuffle the data. Defaults to True.
        drop_last (bool, optional): Whether to drop the last batch if it's smaller than batch_size. Defaults to True.

    Returns:
        DataLoaderType: Pretraining data loader.
    """
    train_samples, train_targets, train_bin_targets, _, _, _ = load_data(dataset, test_size=0.0, only_normal=True)
    print(f'Pretraining with {train_samples.shape[0]} normal samples')

    if contrastive:
        train_ds = OneClassContrastiveDataset(train_samples, train_targets, train_bin_targets)
    else:
        train_ds = CustomDataset(train_samples, train_targets, train_bin_targets)

    pretrain_loader = DataLoader(dataset=train_ds, batch_size=batch_size, shuffle=shuffle, drop_last=drop_last, pin_memory=True)

    return pretrain_loader
