import torch
import torch.nn as nn
import torch.nn.functional as F


class OneClassContrastiveLoss(nn.Module):
    """Implements a one-class contrastive loss function for self-supervised learning.

    This loss function calculates the contrastive loss between two different views of the same data point. It utilizes cosine similarity and cross-entropy loss to measure the similarity between these views.

    Attributes:
        tau (float): The temperature scaling parameter used to scale the cosine similarities.
        reduction (str): Specifies the reduction to apply to the output, 'mean' or 'sum'.

    Example:
        >>> loss_fn = OneClassContrastiveLoss(tau=0.07, reduction='mean')
        >>> features = torch.randn(10, 2, 5)  # Simulated batch of 10 samples with 2 views each, and 5 features
        >>> loss = loss_fn(features)
    """

    def __init__(self, tau=0.07, reduction='mean'):
        super(OneClassContrastiveLoss, self).__init__()
        self.tau = tau
        self.reduction = reduction

    # Parameter `features` should be L2-normalized during forward pass
    def forward(self, features: torch.Tensor) -> torch.Tensor:
        """Calculates the one-class contrastive loss for a batch of features.

        Args:
            features (torch.Tensor): A tensor of shape (batch_size, 2, feature_dim). Each sample in the batch contains two views (e.g., augmented versions) of the same data point.

        Returns:
            torch.Tensor: The calculated loss for the batch.

        Example:
            >>> features = torch.randn(10, 2, 5)
            >>> loss = loss_fn(features)
        """
        # Split the features into two views
        f1, f2 = features[:, 0, :], features[:, 1, :]

        # Compute the cosine similarity (given that f1 and f2 are L2-normalized)
        cos_similarity = torch.mm(f1, f2.t())

        # Scale the cosine similarities by the temperature tau
        logits = cos_similarity / self.tau

        # Labels for each entry in a batch are the indices themselves since the diagonal corresponds to the positive examples (each entry with its own augmented version)
        labels = torch.arange(logits.size(0), device=logits.device)

        # Calculate the cross-entropy loss, which automatically applies softmax
        loss = F.cross_entropy(logits, labels, reduction=self.reduction)

        return loss
