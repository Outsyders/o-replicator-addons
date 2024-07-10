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

from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

def _set_node_inputs(node: og.Node, inputs: Dict[str, Any]):
    for input_name, value in inputs.items():
        _set_node_input(node, input_name, value)


def _set_node_input(
        node: og.Node,
        input_name: str,
        value: Any,
):
    if isinstance(value, ReplicatorItem):
        if any([value.node.get_attribute_exists(attr) for attr in ["outputs:prims", "outputs_prims", "outputs_primsBundle"]]):
            utils._connect_prims(node, input_name, value)
        elif value.node.get_attribute_exists("inputs:choices") or value.node.get_attribute_exists("inputs:items"):
            utils.auto_connect(
                value.node, node, mapping=[utils.AttrMap("outputs:samples", input_name)]
            )
        else:
            if value.node.get_attribute_exists("inputs:numSamples"):
                og.AttributeValueHelper(value.node.get_attribute("inputs:numSamples")).set(1, update_usd=True)
            utils.auto_connect(value.node, node, mapping=[utils.AttrMap("outputs:samples", input_name)])
    elif hasattr(value, "__iter__"):
        if isinstance(value, str) or isinstance(value[0], str) or isinstance(value[0], (Sdf.Path, usdrt.Sdf.Path)):
            set_target_prims(node, input_name, value)
        elif isinstance(value[0], ReplicatorItem):
            set_target_prims(node, input_name, value)
        else:
            og.AttributeValueHelper(node.get_attribute(input_name)).set(value, update_usd=True)
    else:
        raise ValueError(f"Unable to set input for {node} {input_name} of type {type(value)}")