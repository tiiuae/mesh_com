import torch
from torch import nn


class _SepConv1d(nn.Module):
    """A simple separable convolution implementation.

    The separable convlution is a method to reduce number of the parameters
    in the deep learning network for slight decrease in predictions quality.
    """

    def __init__(self, ni, no, kernel, stride, pad):
        super().__init__()
        self.depthwise = nn.Conv1d(ni, ni, kernel, stride, padding=pad, groups=ni)
        self.pointwise = nn.Conv1d(ni, no, kernel_size=1)

    def forward(self, x):
        return self.pointwise(self.depthwise(x))


class SepConv1d(nn.Module):
    """Implementes a 1-d convolution with 'batteries included'.

    The module adds (optionally) activation function and dropout
    layers right after a separable convolution layer.
    """

    def __init__(self, ni, no, kernel, stride, pad,
                 drop=None, bn=True,
                 activ=lambda: nn.PReLU()):

        super().__init__()
        assert drop is None or (0.0 < drop < 1.0)
        layers = [_SepConv1d(ni, no, kernel, stride, pad)]
        if activ:
            layers.append(activ())
        if bn:
            layers.append(nn.BatchNorm1d(no))
        if drop is not None:
            layers.append(nn.Dropout(drop))
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)


class Flatten(nn.Module):
    """Converts N-dimensional tensor into 'flat' one."""

    def __init__(self, keep_batch_dim=True):
        super().__init__()
        self.keep_batch_dim = keep_batch_dim

    def forward(self, x):
        if self.keep_batch_dim:
            return x.view(x.size(0), -1)
        return x.view(-1)


class Classifier(nn.Module):
    def __init__(self, raw_ni, fft_ni, no, drop=.5):
        super().__init__()

        self.raw = nn.Sequential(
            SepConv1d(raw_ni, 32, 8, 2, 3, drop=drop),
            SepConv1d(32, 32, 3, 1, 1, drop=drop),
            SepConv1d(32, 64, 8, 4, 2, drop=drop),
            SepConv1d(64, 64, 3, 1, 1, drop=drop),
            SepConv1d(64, 128, 8, 4, 2, drop=drop),
            SepConv1d(128, 128, 3, 1, 1, drop=drop),
            SepConv1d(128, 256, 8, 4, 2),
            Flatten(),
            nn.Dropout(drop), nn.Linear(256, 64), nn.PReLU(), nn.BatchNorm1d(64),
            nn.Dropout(drop), nn.Linear(64, 64), nn.PReLU(), nn.BatchNorm1d(64))

        # self.fft = nn.Sequential(
        #     SepConv1d(fft_ni, 32, 8, 2, 4, drop=drop),
        #     SepConv1d(32, 32, 3, 1, 1, drop=drop),
        #     SepConv1d(32, 64, 8, 2, 4, drop=drop),
        #     SepConv1d(64, 64, 3, 1, 1, drop=drop),
        #     SepConv1d(64, 128, 8, 4, 4, drop=drop),
        #     SepConv1d(128, 128, 8, 4, 4, drop=drop),
        #     SepConv1d(128, 256, 8, 2, 3),
        #     Flatten(),
        #     nn.Dropout(drop), nn.Linear(256, 64), nn.PReLU(), nn.BatchNorm1d(64),
        #     nn.Dropout(drop), nn.Linear(64, 64), nn.PReLU(), nn.BatchNorm1d(64))

        self.out = nn.Sequential(
            nn.Linear(64, 64), nn.ReLU(inplace=True), nn.Linear(64, no))

        self.init_weights(nn.init.kaiming_normal_)

    def init_weights(self, init_fn):
        def init(m):
            for child in m.children():
                if isinstance(child, nn.Conv1d):
                    init_fn(child.weights)

        init(self)

    def forward(self, t_raw):
        raw_out = self.raw(t_raw)
        # fft_out = self.fft(t_fft)
        # t_in = torch.cat([raw_out, fft_out], dim=1)
        out = self.out(raw_out)
        return out


# Simple MLP, decoder is used for pre-training
class MLP(nn.Module):
    def __init__(self, input_size, num_features, rep_dim, bias=True):
        super().__init__()
        self.input_size = input_size
        self.num_features = num_features
        self.rep_dim = rep_dim
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.encoder = nn.Sequential(
            nn.Linear(input_size, num_features, bias=bias),
            nn.ReLU(),
            nn.Linear(num_features, num_features // 2, bias=bias),
            nn.ReLU(),
            nn.Linear(num_features // 2, rep_dim, bias=bias),
        ).to(self.device)

        self.decoder = nn.Sequential(
            nn.Linear(rep_dim, num_features // 2, bias=bias),
            nn.ReLU(),
            nn.Linear(num_features // 2, num_features, bias=bias),
            nn.ReLU(),
            nn.Linear(num_features, input_size, bias=bias)
        ).to(self.device)

        # util.init_weights(self.encoder, init_type='normal')
        # util.init_weights(self.decoder, init_type='normal')

    def encode(self, x):
        return self.encoder(x)

    def decode(self, x):
        return self.decoder(x)

    def forward(self, x):
        return self.decode(self.encode(x))
