import numpy as np
import torch
import torch.nn.functional as F
from scipy.interpolate import CubicSpline


def random_curve_generator(o, magnitude=0.1, order=4, noise=None):
    seq_len = o.shape[-1]
    f = CubicSpline(np.linspace(-seq_len, 2 * seq_len - 1, 3 * (order - 1) + 1, dtype=int),
                    np.random.normal(loc=1.0, scale=magnitude, size=3 * (order - 1) + 1), axis=-1)
    return f(np.arange(seq_len))


def random_cum_curve_generator(o, magnitude=0.1, order=4, noise=None):
    x = random_curve_generator(o, magnitude=magnitude, order=order, noise=noise).cumsum()
    x -= x[0]
    x /= x[-1]
    x = np.clip(x, 0, 1)
    return x * (o.shape[-1] - 1)


def random_cum_linear_generator(o, magnitude=0.1):
    seq_len = o.shape[-1]
    win_len = int(round(seq_len * np.random.rand() * magnitude))
    if win_len == seq_len: return np.arange(o.shape[-1])
    start = np.random.randint(0, seq_len - win_len)
    # mult between .5 and 2
    rand = np.random.rand()
    mult = 1 + rand
    if np.random.randint(2): mult = 1 - rand / 2
    x = np.ones(seq_len)
    x[start: start + win_len] = mult
    x = x.cumsum()
    x -= x[0]
    x /= x[-1]
    return np.clip(x, 0, 1) * (seq_len - 1)


def horizontal_flip(o: torch.tensor):
    output = torch.flip(o, [-1])
    return output


def gaussian_noise(o: torch.tensor, magnitude: float = 0.1, additive: bool = False):
    if magnitude <= 0:
        return o
    noise = magnitude * torch.randn_like(o)
    if additive:
        return o + noise
    else:
        return o * (1 + noise)


def scale(o: torch.tensor, scale: float = 0.1):
    return o * np.random.uniform(1.0 - scale, 1.0 + scale)


def cut_out(o: torch.tensor, magnitude: tuple = 0.05):
    if not magnitude or magnitude <= 0: return o
    seq_len = o.shape[-1]
    lambd = np.random.beta(magnitude, magnitude)
    lambd = min(lambd, 1 - lambd)
    win_len = int(round(seq_len * lambd))
    start = np.random.randint(-win_len + 1, seq_len)
    end = start + win_len
    start = max(0, start)
    end = min(end, seq_len)
    output = o.clone()
    output[..., start:end] = 0
    return output


def random_crop_resize(o: torch.tensor, size: int, scale: tuple = (0.1, 1.0), mode: str = 'linear'):
    seq_len = o.shape[-1]
    lambd = np.random.uniform(scale[0], scale[1])
    win_len = int(round(seq_len * lambd))
    if win_len == seq_len:
        if size == seq_len: return o
        _slice = slice(None)
    else:
        start = np.random.randint(0, seq_len - win_len)
        _slice = slice(start, start + win_len)
    return F.interpolate(o[..., _slice], size=size, mode=mode, align_corners=False)


def window_slicing(o: torch.tensor, magnitude: float = 0.1, mode: str = 'linear'):
    if not magnitude or magnitude <= 0 or magnitude >= 1: return o
    seq_len = o.shape[-1]
    win_len = int(round(seq_len * (1 - magnitude)))
    if win_len == seq_len: return o
    start = np.random.randint(0, seq_len - win_len)
    return F.interpolate(o[..., start: start + win_len], size=seq_len, mode=mode, align_corners=False)


def time_warp(o: torch.tensor, magnitude: float = 0.1, order: int = 6):
    if not magnitude or magnitude <= 0: return o
    f = CubicSpline(np.arange(o.shape[-1]), o.cpu(), axis=-1)
    output = o.new(f(random_cum_curve_generator(o, magnitude=magnitude, order=order)))
    return output


def window_warp(o: torch.tensor, magnitude: float = 0.1):
    if not magnitude or magnitude <= 0 or magnitude >= 1: return o
    f = CubicSpline(np.arange(o.shape[-1]), o.cpu(), axis=-1)
    output = o.new(f(random_cum_linear_generator(o, magnitude=magnitude)))
    return output
