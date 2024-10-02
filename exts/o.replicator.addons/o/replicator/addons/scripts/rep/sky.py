from typing import Any, Callable, List, Optional, Tuple, Union

import omni.replicator.core as rep
from omni.replicator.core.utils import (
    ReplicatorItem,
    ReplicatorWrapper,
    create_node,
    sequential,
    set_target_prims,
    utils,
)
import omni.graph.core as og


@ReplicatorWrapper
def randomize_sky(
    # time_of_day: ReplicatorItem = [0.0, 23.0],
    elevation: Union[ReplicatorItem, float] = [10.0, 170.0],
    input_prims: Union[ReplicatorItem, List[str]] = None,
) -> ReplicatorItem:
    """
    Randomize the sky time of day.
    """

    if isinstance(elevation, ReplicatorItem):
        pass
    elif isinstance(elevation, float):
        elevation = rep.distribution.uniform(elevation, elevation)
    elif isinstance(elevation, list):
        elevation = rep.distribution.uniform(elevation[0], elevation[1])
    else:
        raise ValueError(f"Invalid time of day value: {elevation}")

    node = create_node(
        "omni.replicator.core.OgnWritePrimAttribute", attribute="inputs:Elevation", attributeType="float"
    )
    elevation.node.get_attribute("outputs:samples").connect(node.get_attribute("inputs:values"), True)

    if input_prims:
        set_target_prims(node, "inputs:prims", input_prims)

    return node
