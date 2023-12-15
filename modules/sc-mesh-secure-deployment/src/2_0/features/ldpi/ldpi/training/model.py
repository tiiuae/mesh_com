import torch
import torch.nn.functional as F
from torch import nn
from tsai.models.all import ResCNN


class ResCNNContrastive(nn.Module):
    """
    A contrastive learning model built on the ResCNN architecture.

    This class creates a model for contrastive learning using Residual CNNs.
    It includes methods to initialize the model, create the encoder and head,
    and perform forward pass operations.

    Attributes:
        dim_mid (int): Dimensionality of the intermediate layers in the head.
        feat_dim (int): Dimensionality of the output feature vector.
        device (torch.device): The device (CPU or GPU) on which the model runs.
        encoder (nn.Module): The encoder module based on ResCNN.
        head (nn.Sequential): The head module consisting of linear and activation layers.
        dim_in (int): Input dimensionality for the head module.

    Methods:
        forward(x): Computes the forward pass of the model.
        encode(x): Encodes the input using the model without normalization.
    """

    def __init__(self, dim_mid: int = 128, feat_dim: int = 128, verbose: bool = True):
        """
        Initializes the ResCNNContrastive model.

        Args:
            dim_mid (int): Intermediate feature dimension in the head. Defaults to 128.
            feat_dim (int): Feature dimension of the final output. Defaults to 128.
            verbose (bool): If True, prints model and size information.

        """
        super().__init__()
        self.dim_mid = dim_mid
        self.feat_dim = feat_dim
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Instantiate encoder and head
        self._create_encoder()
        self._create_head()
        self._initialize_weights()

        # Print model and size information if verbose is True
        if verbose:
            print(self)
            print(f'Model size: {sum([param.nelement() for param in self.parameters()]) / 1000000} (M)')

    def _create_encoder(self):
        """Creates the encoder module based on ResCNN."""
        self.encoder = ResCNN(1, 10, separable=True).to(self.device)
        self.dim_in = self.encoder.lin.in_features
        self.encoder.lin = nn.Identity()

    def _create_head(self, depth: int = 4):
        """
        Creates the head module for the model.

        Args:
            depth (int): Number of layers in the head module.
        """
        layers = []
        layers.extend([
            nn.Linear(self.dim_in, self.dim_mid, bias=False),
            nn.BatchNorm1d(self.dim_mid),
            nn.ReLU(inplace=True)
        ])
        for _ in range(depth - 1):
            layers.extend([
                nn.Linear(self.dim_mid, self.dim_mid, bias=False),
                nn.BatchNorm1d(self.dim_mid),
                nn.ReLU(inplace=True)
            ])
        layers.append(nn.Linear(self.dim_mid, self.feat_dim, bias=True))
        self.head = nn.Sequential(*layers)

    def _initialize_weights(self):
        """
        Initializes weights of the model using He and Glorot initialization.
        """
        # Apply He (Kaiming) initialization to the encoder layers
        for layer in self.encoder.modules():
            if isinstance(layer, (nn.Conv2d, nn.Linear)):
                nn.init.kaiming_normal_(layer.weight, mode='fan_out', nonlinearity='relu')
                if layer.bias is not None:
                    nn.init.constant_(layer.bias, 0)
            elif isinstance(layer, nn.BatchNorm2d):
                nn.init.constant_(layer.weight, 1)
                nn.init.constant_(layer.bias, 0)

        # Apply Glorot (Xavier) initialization to the head layers
        for layer in self.head.modules():
            if isinstance(layer, nn.Linear):
                nn.init.xavier_uniform_(layer.weight)
                if layer.bias is not None:
                    nn.init.constant_(layer.bias, 0)
            elif isinstance(layer, nn.BatchNorm1d):
                nn.init.constant_(layer.weight, 1)
                nn.init.constant_(layer.bias, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Performs the forward pass of the model.

        Args:
            x (torch.Tensor): Input tensor.

        Returns:
            torch.Tensor: Normalized feature vector.
        """
        feat = self.encoder(x)
        feat = F.normalize(self.head(feat), dim=1)
        return feat

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        """
        Encodes the input using the model without normalization.

        Args:
            x (torch.Tensor): Input tensor.

        Returns:
            torch.Tensor: Feature vector.
        """
        feat = self.encoder(x)
        feat = self.head(feat)
        return feat
