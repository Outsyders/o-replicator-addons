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
from typing import Any, Callable, List, Optional, Tuple, Union
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import set_stage_up_axis
import omni.isaac.core.utils.stage as stage_utils


@ReplicatorWrapper
def _density(density: ReplicatorItem, input_prims: Union[ReplicatorItem, List[str]] = None) -> ReplicatorItem:
    if isinstance(density, ReplicatorItem):
        pass
    elif isinstance(density, float):
        density = rep.distribution.uniform(density, density)
    elif isinstance(density, list):
        density = rep.distribution.uniform(density[0], density[1])
    else:
        raise ValueError(f"Invalid density value: {density}")

    node = create_node(
        "omni.replicator.core.OgnWritePrimAttribute", attribute="inputs:volume_density_scale", attributeType="float"
    )
    # utils._setup_random_attribute(write_node=node, attribute_value=density, prim_path=input_prims)

    density.node.get_attribute("outputs:samples").connect(node.get_attribute("inputs:values"), True)

    if input_prims:
        set_target_prims(node, "inputs:prims", input_prims)

    return node


@ReplicatorWrapper
def _texture(texture: ReplicatorItem, input_prims: Union[ReplicatorItem, List[str]] = None) -> ReplicatorItem:
    if isinstance(texture, ReplicatorItem):
        pass
    elif isinstance(texture, str):
        texture = rep.distribution.uniform(texture, texture)
    elif isinstance(texture, list):
        texture = rep.distribution.choice(texture)
    else:
        raise ValueError(f"Invalid texture value: {texture}")

    node = create_node(
        "omni.replicator.core.OgnWritePrimAttribute",
        attribute="inputs:volume_density_texture",  # , attributeType="token"
    )
    # utils._setup_random_attribute(write_node=node, attribute_value=texture, prim_path=input_prims)

    texture.node.get_attribute("outputs:samples").connect(node.get_attribute("inputs:values"), True)

    if input_prims:
        set_target_prims(node, "inputs:prims", input_prims)

    return node


@ReplicatorWrapper
def randomize_volume(
    volume_texture: List[str] = None,
    volume_density: List[float] = None,
    # volume_color: List[List[float]] = None,
    # volume_scattering_phase: List[float] = None,
    input_prims: Union[ReplicatorItem, List[str]] = None,
):
    """
    Randomize the volume density and color of the given prims.

    ```
    import omni

    from omni.isaac.core.utils.prims import get_all_matching_child_prims

    prim_path = "/"

    vol_mat_path = (
        get_all_matching_child_prims(prim_path, lambda p: p.endswith("OmniVolumeDensity"))[0].GetPath().pathString
    )

    # vol_path = "omniverse://localhost/Projects/blu/volumes/smoke/jet_smoke/Houdini/geo/SMOKE_CACHE_0/v1"
    # result, volumes = omni.client.list(vol_path)
    # # result, volumes = await asyncio.wait_for(omni.client.list_async(vol_path), timeout=30)
    # print(result == omni.client.Result.OK)
    # print(result, volumes)

    vols = [
        paths to .vdb files
    ]

    shader = rep.get.prim_at_path(f"{vol_mat_path}/Shader")

    with shader:
        randomize_volume(volume_texture=vols)
    ```

    """

    with sequential():
        if volume_texture:
            _texture(volume_texture, input_prims=input_prims)

        if volume_density:
            _density(volume_density, input_prims=input_prims)

        # if volume_color:
        #     _color(volume_color, input_prims=input_prims)

        # if volume_scattering_phase:
        #     _scattering_phase(volume_scattering_phase, input_prims=input_prims)
