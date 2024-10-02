from typing import Any, List, Dict
import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt
import importlib
import carb

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.replicator.core as rep
import omni.replicator.core.ogn as ogn


def _init_db_state(db: og.Database) -> None:
    state = db.shared_state
    is_seed_valid = db.inputs.seed is not None
    is_seed_changed = state.rng is None or db.inputs.seed != state.rng.seed

    if is_seed_valid and is_seed_changed:
        node_id = db.inputs.nodeId if db.node.get_attribute_exists("inputs:nodeId") else 0
        state.rng.initialize(db.inputs.seed, db.node, node_id)


def _set_database_state(db: og.Database, new_state: Dict, state_attr: str = "shared_state") -> None:
    """
    Set the state of a node if it has one

    Args:
        db (og.Node): The node to set the state of
        new_state (Dict): The state to set
    """
    if not new_state:
        return False

    if not hasattr(db, "shared_state"):
        carb.log_warning(f"Shared state is `None` for {db.node.get_prim_path()}")
        return False

    # state = db.shared_state
    state = getattr(db, state_attr)

    if not state:
        carb.log_warning(f"State is `None` for {db.node.get_prim_path()}")
        return False

    if not hasattr(state, "rng"):
        carb.log_warning(f"State does not have an RNG for {db.node.get_prim_path()}")
        return False

    _init_db_state(db)

    rng: rep.rng.ReplicatorRNG = state.rng
    gen = rng.generator

    if not gen:
        carb.log_warning(f"Generator is `None` for {db.node.get_prim_path()}")
        return False

    bit = gen.bit_generator

    if not bit:
        carb.log_warning(f"Bit generator is `None` for {db.node.get_prim_path()}")
        return False

    bit.state = new_state

    return True


def _get_database_state(db: og.Database, state_attr: str = "shared_state") -> Any:
    """
    Get the state of a node if it has one

    Args:
        state (og.Node): The node to get the state of

    Returns:
        Any: The state of the node
    """
    if not hasattr(db, "shared_state"):
        carb.log_warning(f"Shared state is `None` for {db}")
        return None

    # state = db.shared_state
    state = getattr(db, state_attr)

    if not state:
        return None

    if not hasattr(state, "rng"):
        return None

    _init_db_state(db)

    gen = state.rng.generator

    if not gen:
        return None

    bit = gen.bit_generator

    if not bit:
        return None

    return bit.state


def _all_replicator_nodes() -> List[og.Node]:
    usdrt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    for prim_path in usdrt_stage.GetPrimsWithTypeName("OmniGraphNode"):
        node = og.Controller().node(str(prim_path))
        yield node


def _get_node_db(node: og.Node) -> Any:
    type_name = node.get_type_name()
    type_name = type_name.split(".")[-1]

    class_name = f"{type_name}Database"
    module_name = f"omni.replicator.core.ogn.{class_name}"

    try:
        module = importlib.import_module(module_name)
        db = getattr(module, class_name)
    except ModuleNotFoundError:
        return None

    node_db = db(node)

    return node_db


def get_replicator_state(skip_empty: bool = False) -> dict:
    # Find all RNG nodes in the graph and get their states
    states = {}
    for node in _all_replicator_nodes():
        node_db = _get_node_db(node)

        if not node_db:
            continue

        shared_state = _get_database_state(node_db, "shared_state")
        per_instance_state = _get_database_state(node_db, "per_instance_state")

        if skip_empty and not shared_state and not per_instance_state:
            continue

        states[node.get_prim_path()] = {
            "shared_state": shared_state,
            "per_instance_state": per_instance_state,
        }

    return states


def set_replicator_state(states: dict):
    # TODO: Rep.orchestrator.step() needs to be called before the
    # state can be set. This is a workaround for now. This should be
    # fixed in the future.

    # Set the states of all RNG nodes in the graph
    for node in _all_replicator_nodes():
        node_db = _get_node_db(node)

        if not node_db:
            continue

        node_path = node.get_prim_path()
        if node_path not in states:
            carb.log_warn(f"Node {node_path} not found in state dict... skipping")
            continue

        _state_dict = states[node_path]

        for state_attr, state in _state_dict.items():
            _set_database_state(node_db, state, state_attr)
