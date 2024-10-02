import numpy as np
import torch


def percentage_pixels(img, threshold=0.0):
    """
    Compute the percentage of pixels at or bellow a threshold.
    """
    if isinstance(img, np.ndarray):
        return np.mean(img <= threshold)
    elif isinstance(img, torch.Tensor):
        return torch.mean(img <= threshold).item()
    else:
        raise ValueError("Unsupported type for img: {}".format(type(img)))
