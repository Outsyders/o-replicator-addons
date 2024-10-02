"""
Utility functions for working with volumes in USD.
"""

from pxr import UsdGeom, UsdShade, Sdf, UsdLux, Usd, Gf, Vt

import omni
import carb
from omni.replicator.core.utils import ReplicatorItem, get_non_xform_prims
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid, move_prim
from omni.kit.material.library import CreateAndBindMdlMaterialFromLibrary

# Enable volume rendering | path tracing settings | "Non Uniform Volumes"

# Set the primvars:isVolume to True

# Assign a volume density shader to the prim

# *May need to correct for up axis*


volumeRenderSettings = {
    "/rtx/pathtracing/ptvol/enabled": True,  # Non-Uniform Volumes
    "/rtx/flow/enabled": True,
    "/rtx/flow/rayTracedShadowsEnabled": True,
    "/rtx/flow/rayTracedReflectionsEnabled": True,
    "/rtx/flow/rayTracedTranslucencyEnabled": True,
    "/rtx/flow/pathTracingEnabled": True,
    "/rtx/flow/pathTracingShadowsEnabled": True,
    "/rtx/flow/compositeEnabled": True,
    "/rtx/flow/useFlowLibrarySelfShadow": True,
    "/rtx/flow/maxBlocks": 100,
}


def setup_flow():
    """
    Enable the FlowUSD extension if it is not already enabled.

    Bad things will happen if this is not enabled before trying to use volume rendering.
    """
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled("omni.flowusd"):
        manager.set_extension_enabled_immediate("omni.flowusd", True)


def enable_volume_rendering(max_bounces=2, max_light_collision_count=32, max_collision_count=1024):
    """
    Enable volume rendering with the given settings.

    Args:
        max_bounces (int, optional): Maximum number of bounces. Defaults to 2.
        max_light_collision_count (int, optional): Maximum number of light collision counts. Defaults to 32.
        max_collision_count (int, optional): Maximum number of collision counts. Defaults to 1024.

    If there is stepping in the volume, increase the max_collision_count to around 4096.
    """

    setup_flow()

    settings = carb.settings.get_settings()

    for key, value in volumeRenderSettings.items():
        settings.set(key, value)

    settings.set("/rtx/pathtracing/ptvol/maxCollisionCount", max_collision_count)
    settings.set("/rtx/pathtracing/ptvol/maxLightCollisionCount", max_light_collision_count)
    settings.set("/rtx/pathtracing/ptvol/maxBounces", max_bounces)


def _set_is_volume(path):
    _usd_context = omni.usd.get_context("")
    stage = _usd_context.get_stage()

    _prim_name = "isVolume"
    primvars_api = UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(path))
    value = primvars_api.GetPrimvar(_prim_name)

    if value:
        if value.GetTypeName() != Sdf.ValueTypeNames.Bool:
            carb.log_error(f"TogglePrimVarCommand: cannot set value as {value.GetTypeName()} isn't a {self._prim_type}")
        else:
            value.Set(not value.Get())
    else:
        primvars_api.CreatePrimvar(_prim_name, Sdf.ValueTypeNames.Bool).Set(True)


def make_prim_volume(prim_path, shader_path=None, vdb_path=None, density_scale=1.0):
    """
    Create a volume prim with the given path and density shader.
    """

    # Get a list of paths
    # Input can be a ReplicatorItem, Usd.Prim, or a string, or a list of these
    # Out put is a list of strings

    def _get_prim_paths(prim_path):
        if isinstance(prim_path, ReplicatorItem):
            prim_paths = get_non_xform_prims(prim_path.get_output_prims()["prims"])
            return [path.GetPath().pathString for path in prim_paths]
        if isinstance(prim_path, Usd.Prim):
            return [prim_path.GetPath().pathString]
        if isinstance(prim_path, str):
            return [prim_path]

        raise ValueError(f"Invalid prim path: {prim_path}")

    if isinstance(prim_path, list):
        prim_paths = []
        for path in prim_path:
            prim_paths.extend(_get_prim_paths(path))
    else:
        prim_paths = _get_prim_paths(prim_path)

    # if not is_prim_path_valid(prim_path):
    #     raise ValueError(f"Invalid prim path: {prim_path}")

    # Add a volume shader
    if is_prim_path_valid(prim_path=shader_path if shader_path else ""):
        material = UsdShade.Material(get_prim_at_path(shader_path))
    else:
        mtl_created_list = []
        CreateAndBindMdlMaterialFromLibrary(
            mdl_name="OmniVolumeDensity.mdl", mtl_name="OmniVolumeDensity", mtl_created_list=mtl_created_list
        ).do()

        material = UsdShade.Material(get_prim_at_path(mtl_created_list[0]))

    material_path = material.GetPath().pathString

    # print(f"Bound material {material_path} to prim {prim_path}")
    shader_prim = get_prim_at_path(f"{material_path}/Shader")
    shader = UsdShade.Shader(shader_prim)

    if shader.GetInput("volume_density_scale").Get() is None:
        shader.CreateInput("volume_density_scale", Sdf.ValueTypeNames.Float).Set(density_scale)

    shader.GetInput("volume_density_scale").Set(density_scale)

    if shader.GetInput("volume_density_texture").Get() is None:
        shader.CreateInput("volume_density_texture", Sdf.ValueTypeNames.Asset)

    if vdb_path is not None:
        shader.GetInput("volume_density_texture").Set(vdb_path)

    for path in prim_paths:
        _set_is_volume(path)

        omni.kit.commands.execute(
            "BindMaterialCommand",
            prim_path=path,
            material_path=material_path,
        )

    return prim_path, material_path
