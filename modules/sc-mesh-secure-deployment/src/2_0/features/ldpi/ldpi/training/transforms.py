import numpy as np
from scipy.interpolate import interp1d

# Type aliases for clarity (compatible with Python 3.8)
ArrayFloat = np.ndarray


def inject_random_noise(samples: ArrayFloat, noise_level: float = 0.05) -> ArrayFloat:
    """
    Injects random noise into samples to create noisy outliers.

    Args:
        samples (ArrayFloat): A numpy array of samples.
        noise_level (float, optional): Level of noise to inject. Defaults to 0.05.

    Returns:
        ArrayFloat: Noisy samples.
    """
    noise = np.random.uniform(-noise_level, noise_level, samples.shape)
    noisy_samples = np.clip(samples + noise, 0, 1)
    return noisy_samples


def reverse_sequences(samples: ArrayFloat) -> ArrayFloat:
    """
    Reverses the order of sequences in samples to create reversed outliers.

    Args:
        samples (ArrayFloat): A numpy array of samples.

    Returns:
        ArrayFloat: Reversed samples.
    """
    reversed_samples = np.flip(samples, axis=1)
    return reversed_samples


def shuffle_sequences(samples: ArrayFloat) -> ArrayFloat:
    """
    Shuffles the sequences in samples to create shuffled outliers.

    Args:
        samples (ArrayFloat): A numpy array of samples.

    Returns:
        ArrayFloat: Shuffled samples.
    """
    shuffled_samples = np.copy(samples)
    for s in shuffled_samples:
        np.random.shuffle(s)
    return shuffled_samples


def invert_values(samples: ArrayFloat) -> ArrayFloat:
    """
    Inverts the values of samples to create inverted outliers.

    Args:
        samples (ArrayFloat): A numpy array of samples.

    Returns:
        ArrayFloat: Inverted samples.
    """
    inverted_samples = 1 - samples
    return inverted_samples


def crop_and_resize(sample: ArrayFloat) -> ArrayFloat:
    """
    Crops and resizes a sample.

    Args:
        sample (ArrayFloat): A numpy array of a sample.

    Returns:
        ArrayFloat: Cropped and resized sample.
    """
    original_length = len(sample)
    scale = np.random.uniform(0.1, 1.0)
    crop_size = int(original_length * scale)
    start = np.random.randint(0, len(sample) - crop_size)
    cropped_sample = sample[start:start + crop_size]
    # Linear interpolation to resize back to original sequence length
    return interp1d(np.linspace(0, crop_size - 1, num=crop_size), cropped_sample, kind='linear', fill_value='extrapolate')(np.arange(original_length))


def temporal_scaling(sample: ArrayFloat) -> ArrayFloat:
    """
    Applies temporal scaling to a sample.

    Args:
        sample (ArrayFloat): A numpy array of a sample.

    Returns:
        ArrayFloat: Sample with temporal scaling applied.
    """
    original_length = len(sample)
    scale = np.random.uniform(0.8, 1.2)  # Adjust scaling factors as needed
    scaled_length = int(original_length * scale)
    indices = np.linspace(0, original_length - 1, num=scaled_length)
    scaled_sample = interp1d(np.arange(original_length), sample, kind='linear')(indices)

    # Adjust the length to match the original length
    if scaled_length < original_length:
        # Extend the sequence to the original length
        additional_indices = np.linspace(scaled_length, original_length - 1, num=original_length - scaled_length)
        extended_sample = interp1d(np.arange(scaled_length), scaled_sample, kind='linear', fill_value='extrapolate')(additional_indices)
        return np.concatenate([scaled_sample, extended_sample])
    else:
        # Trim the sequence to the original length
        return scaled_sample[:original_length]


def jittering(sample: ArrayFloat, noise_level: float = 0.02) -> ArrayFloat:
    """
    Applies jittering to a sample by adding noise.

    Args:
        sample (ArrayFloat): A numpy array of a sample.
        noise_level (float, optional): Level of noise to add. Defaults to 0.02.

    Returns:
        ArrayFloat: Sample with jittering applied.
    """
    noise = np.random.normal(0, noise_level, len(sample))
    return np.clip(sample + noise, 0, 1)


def random_segment_permutation(sample: ArrayFloat, num_segments: int = 5) -> ArrayFloat:
    """
    Applies random segment permutation to a sample.

    Args:
        sample (ArrayFloat): A numpy array of a sample.
        num_segments (int, optional): Number of segments to create. Defaults to 5.

    Returns:
        ArrayFloat: Sample with random segment permutation applied.
    """
    segment_length = len(sample) // num_segments
    segments = [sample[i * segment_length:(i + 1) * segment_length] for i in range(num_segments)]
    np.random.shuffle(segments)
    return np.concatenate(segments)
