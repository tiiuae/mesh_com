import numpy as np


def resize(X: np.ndarray, size: int) -> np.ndarray:
    T = X.shape[0]
    if size == T:
        return X.copy()
    ind = np.linspace(0, T - 1, size, dtype=int)
    return np.interp(ind, np.arange(T), X)
