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


def send_og_event(event_name: str) -> ReplicatorItem:
    event_node = ReplicatorItem(create_node, "omni.graph.action.SendCustomEvent")
    event_node.node.get_attribute("inputs:eventName").set(event_name)
    return event_node


@ReplicatorWrapper
def toggle_visibility(input_prims: Union[ReplicatorItem, List[str]] = None, name: str = None) -> ReplicatorItem:
    """
    Create a node that toggles the visibility of the input prims.
    """

    with sequential():
        get_vis = ReplicatorItem(create_node, "o.replicator.addons.OgnGetVisibility", node_name=name)
        set_vis = ReplicatorItem(create_node, "omni.replicator.core.OgnSetVisibility")

    bool_not = create_node("omni.graph.nodes.BooleanNot")

    get_vis.node.get_attribute("outputs:values").connect(bool_not.get_attribute("inputs:valueIn"), True)
    bool_not.get_attribute("outputs:valueOut").connect(set_vis.node.get_attribute("inputs:values"), True)

    if input_prims:
        set_target_prims(set_vis, "inputs:prims", input_prims)

    return get_vis


# @ReplicatorWrapper
# def toggle_visibility(flip: bool = False, input_prims: Union[ReplicatorItem, List[str]] = None) -> ReplicatorItem:
#     """
#     Create a node that toggles the visibility of the input prims.
#     """

#     vis_node = create_node("omni.replicator.core.OgnSetVisibility", node_name="Set Visibility")

#     if input_prims:
#         set_target_prims(vis_node, "inputs:prims", input_prims)

#     toggle_node = ReplicatorItem(create_node, "omni.graph.action.FlipFlop")

#     make_array_node = create_node("omni.graph.nodes.ConstructArray")
#     toggle_node.node.get_attribute("outputs:isA").connect(make_array_node.get_attribute("inputs:input0"), True)

#     toggle_node.node.get_attribute("outputs:a").connect(vis_node.get_attribute("inputs:execIn"), True)
#     toggle_node.node.get_attribute("outputs:b").connect(vis_node.get_attribute("inputs:execIn"), True)

#     if flip:
#         negate_node = create_node("omni.graph.nodes.BooleanNot")
#         make_array_node.get_attribute("outputs:array").connect(negate_node.get_attribute("inputs:valueIn"), True)
#         negate_node.get_attribute("outputs:valueOut").connect(vis_node.get_attribute("inputs:values"), True)
#     else:
#         make_array_node.get_attribute("outputs:array").connect(vis_node.get_attribute("inputs:values"), True)

#     return toggle_node


@ReplicatorWrapper
def random_select(n: int = 1, input_prims: Union[ReplicatorItem, List[str]] = None) -> List[ReplicatorItem]:
    """
    Randomly toggle visibility of prims.
    """

    counter = ReplicatorItem(create_node, "omni.replicator.core.OgnCount")
    # input_prims.node.get_attribute("outputs:prims").connect(count_node.get_attribute("inputs:prims"), True)

    if input_prims:
        set_target_prims(counter.node, "inputs:prims", input_prims)

    array_node = create_node("omni.graph.nodes.ConstructArray")
    array_node.get_attribute("inputs:arraySize").set(1)
    array_node.get_attribute("inputs:arrayType").set("bool[]")
    array_node.get_attribute("inputs:input0").set(False)

    resize_node = create_node("omni.graph.nodes.ArrayResize")
    array_node.get_attribute("outputs:array").connect(resize_node.get_attribute("inputs:array"), True)
    counter.node.get_attribute("outputs:count").connect(resize_node.get_attribute("inputs:newSize"), True)

    set_idx_node = create_node("omni.graph.nodes.ArraySetIndex")
    resize_node.get_attribute("outputs:array").connect(set_idx_node.get_attribute("inputs:array"), True)
    set_idx_node.get_attribute("inputs:value").set(True)

    rand_idx_node = rep.distribution.choice(list(range(n)))  # TODO: get input size
    rand_idx_node.node.get_attribute("outputs:samples").connect(set_idx_node.get_attribute("inputs:index"), True)

    set_visibility_node = create_node("omni.replicator.core.OgnSetVisibility")
    set_idx_node.get_attribute("outputs:array").connect(set_visibility_node.get_attribute("inputs:values"), True)
    # input_prims.node.get_attribute("outputs:prims").connect(set_visibility_node.get_attribute("inputs:prims"), True)

    return set_visibility_node
