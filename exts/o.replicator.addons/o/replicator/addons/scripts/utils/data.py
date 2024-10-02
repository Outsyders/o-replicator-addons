import torch
from typing import Union
import numpy as np
import warp as wp


def wp_to_torch(data: Union[np.ndarray, dict], requires_grad=True) -> torch.Tensor:
    """Converts a numpy array or weakref to a torch tensor"""
    if isinstance(data, np.ndarray):
        image = torch.from_numpy(data)
    elif isinstance(data, dict):
        image = torch.from_numpy(data["data"])
    else:
        if data.dtype not in [wp.float16, wp.float32, wp.float64] and requires_grad:
            requires_grad = False

        image = wp.to_torch(data, requires_grad=requires_grad)

    return image


def clip_max(x: torch.Tensor) -> torch.Tensor:
    """Clip inf to max value of the tensor"""
    if x.is_leaf:
        # Cannot perform inplace operation on leaf tensor
        x = x.clone()
    fmax = torch.finfo(x.dtype).max
    x[x == torch.inf] = fmax
    return x
