import sys
from typing import Any, Callable, List, Optional, Tuple, Union

import carb
import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt
from omni.usd._impl.utils import get_prim_at_path
from pxr import Sdf, UsdGeom

from omni.replicator.core.distribution import choice, sequence, uniform
from omni.replicator.core.utils import ReplicatorItem, ReplicatorWrapper, create_node, sequential, set_target_prims, utils

import omni.replicator.core as rep

from .utils import _set_node_input


@ReplicatorWrapper
def focus(
    focus_on: Union[ReplicatorItem, str, Sdf.Path, usdrt.Sdf.Path, List[Union[str, Sdf.Path, usdrt.Sdf.Path]]],
    zoom: Union[ReplicatorItem, float] = 2.0,
    use_horizontal_fov: bool = True,
    conform: Union[int, str] = None,
    input_prims: Union[ReplicatorItem, List[str]] = None,
) -> ReplicatorItem:
    """Modify the focal length of the camera specified in ``input_prims`` to focus at the specified target.

    Args:
        target: The target to orient towards. If multiple prims are set, the target point will be the mean of their
            positions.
        input_prims: The prims to be modified. If using ``with`` syntax, this argument can be omitted.

    Example:
        >>> import omni.replicator.core as rep
        >>> target = rep.create.sphere()
        >>> with rep.create.camera():
        ...     rep.modify.focus(
        ...         focus_on=target,
        ...         zoom=rep.distribution.uniform(2, 4)
        ...     )
        omni.replicator.core.modify._look_at
    """
    with sequential():
        calc_node = _focus_on(
            target=focus_on,
            zoom=zoom,
            use_horizontal_fov=use_horizontal_fov,
            set_focal_length=False,
            conform=conform,
            input_prims=input_prims
            )
        
        write_node = rep.modify.attribute(
            name="focalLength",
            value=calc_node,
            attribute_type="float",
            input_prims=input_prims,
        )


@ReplicatorWrapper
def _focus_on(
    target: Union[
        ReplicatorItem,
        str,
        Sdf.Path,
        usdrt.Sdf.Path,
        List[Union[str, Sdf.Path, usdrt.Sdf.Path]],
    ],
    zoom: Union[ReplicatorItem, float] = 2.0,
    set_focal_length: bool = True,
    use_horizontal_fov: bool = True,
    conform: Union[int, str] = None,
    input_prims: Union[ReplicatorItem, List[str]] = None,
) -> ReplicatorItem:
    node = create_node("omni.replicator.addons.CalculateFocalLength")

    if isinstance(zoom, ReplicatorItem):
        if zoom.node.get_attribute_exists("inputs:numSamples"):
            og.AttributeValueHelper(zoom.node.get_attribute("inputs:numSamples")).set(1, update_usd=True)
        utils.auto_connect(zoom.node, node, mapping=[utils.AttrMap("outputs:samples", "inputs:zoom")])
    else:
        if isinstance(zoom, (int, float)):
            og.AttributeValueHelper(node.get_attribute("inputs:zoom")).set(zoom, update_usd=True)
        elif zoom is None:
            pass
        else:
            raise ValueError(f"The type of `zoom` must be either float or int, but got {type(zoom)}.")

    if isinstance(set_focal_length, ReplicatorItem):
        if set_focal_length.node.get_attribute_exists("inputs:numSamples"):
            og.AttributeValueHelper(set_focal_length.node.get_attribute("inputs:numSamples")).set(1, update_usd=True)
        utils.auto_connect(set_focal_length.node, node, mapping=[utils.AttrMap("outputs:samples", "inputs:setFocalLength")])
    else:
        if isinstance(set_focal_length, (int, bool)):
            og.AttributeValueHelper(node.get_attribute("inputs:setFocalLength")).set(set_focal_length, update_usd=True)
        elif set_focal_length is None:
            pass
        else:
            raise ValueError(f"The type of `set focal length` must be bool, but got {type(set_focal_length)}.")

    if isinstance(use_horizontal_fov, ReplicatorItem):
        if use_horizontal_fov.node.get_attribute_exists("inputs:numSamples"):
            og.AttributeValueHelper(use_horizontal_fov.node.get_attribute("inputs:numSamples")).set(1, update_usd=True)
        utils.auto_connect(use_horizontal_fov.node, node, mapping=[utils.AttrMap("outputs:samples", "inputs:useHorizontalFov")])
    else:
        if isinstance(use_horizontal_fov, (int, bool)):
            og.AttributeValueHelper(node.get_attribute("inputs:useHorizontalFov")).set(use_horizontal_fov, update_usd=True)
        elif use_horizontal_fov is None:
            pass
        else:
            raise ValueError(f"The type of `use_horizontal_fov` must be bool, but got {type(use_horizontal_fov)}.")

    _set_node_input(node, "inputs:targetPrim", target)

    if conform:
        _set_node_input(node, "inputs:conform", conform)

    # Target is the prim(s) to focus on
    # input_prims is the camera to modify

    if input_prims:
        set_target_prims(node, "inputs:prims", input_prims)

    return node
