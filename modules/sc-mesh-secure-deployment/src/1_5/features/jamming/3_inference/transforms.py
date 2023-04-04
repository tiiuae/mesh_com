"""
transforms.py
Description: 

Author: Willian T. Lunardi
Contact: wtlunar@gmail.com
License:

Repository:
"""

import numpy as np


def resize(X: np.ndarray, size: int) -> np.ndarray:
    """
    Resizes the given 1D NumPy array to a new size.

    If the new size is equal to the original size of the array, a copy of the original array is returned.

    :param X: A 1D NumPy array to be resized.
    :param size: An integer specifying the new size of the array.
    :return: A 1D NumPy array of the specified size that is a resized version of the original array.
    """
    T = X.shape[0]
    if size == T:
        return X.copy()
    ind = np.linspace(0, T - 1, size, dtype=int)
    return np.interp(ind, np.arange(T), X)