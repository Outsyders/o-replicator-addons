# Code copied from: \omni.replicator.core-1.11.14+106.0.1.wx64.r.cp310\omni\replicator\core\ogn\python\_impl\nodes\OgnGetSkeletonData.py

from typing import Dict, Tuple
import json

import carb
import numpy as np
import omni.graph.core as og
import omni.kit
import omni.syntheticdata as sd
import omni.timeline
import omni.usd
from pxr import Semantics, UsdGeom, UsdSkel, Vt, Sdf

from omni.isaac.core.utils.prims import get_prim_attribute_value


def _ftheta_distortion(ftheta, x):
    """F-Theta distortion."""
    return ftheta["poly_a"] + x * (
        ftheta["poly_b"] + x * (ftheta["poly_c"] + x * (ftheta["poly_d"] + x * ftheta["poly_e"]))
    )


def get_camera_params(camera_path: str, size: Tuple[int, int] = None) -> Dict:
    stage = omni.usd.get_context().get_stage()
    camera = stage.GetPrimAtPath(camera_path)
    current_time = omni.timeline.get_timeline_interface().get_current_time()

    view_to_world = UsdGeom.Imageable(camera).ComputeLocalToWorldTransform(current_time)
    world_to_view = view_to_world.GetInverse()

    if size is None:
        render_product = camera.GetAttribute("renderProduct").Get(current_time)
        render_product_prim = stage.GetPrimAtPath(Sdf.Path(render_product))
        width, height = render_product_prim.GetAttribute("resolution").Get()
    elif isinstance(size, tuple) or isinstance(size, list):
        width, height = size
    else:
        carb.log_error(f"Invalid size type: {type(size)}")
        return None

    projection_type = camera.GetAttribute("cameraProjectionType").Get()

    if "fisheye" in projection_type:
        ftheta = {
            "width": camera.GetAttribute("fthetaWidth").Get(),
            "height": camera.GetAttribute("fthetaHeight").Get(),
            "cx": camera.GetAttribute("fthetaCx").Get(),
            "cy": camera.GetAttribute("fthetaCy").Get(),
            "poly_a": camera.GetAttribute("fthetaPolyA").Get(),
            "poly_b": camera.GetAttribute("fthetaPolyB").Get(),
            "poly_c": camera.GetAttribute("fthetaPolyC").Get(),
            "poly_d": camera.GetAttribute("fthetaPolyD").Get(),
            "poly_e": camera.GetAttribute("fthetaPolyE").Get(),
            "max_fov": camera.GetAttribute("fthetaMaxFov").Get(),
        }
        ftheta["edge_fov"] = _ftheta_distortion(ftheta, ftheta["width"] / 2)
        ftheta["c_ndc"] = np.array(
            [
                (ftheta["cx"] - ftheta["width"] / 2) / ftheta["width"],
                (ftheta["height"] / 2 - ftheta["cy"]) / ftheta["width"],
            ]
        )
    else:
        ftheta = None

    return {
        "view_to_world": np.array(view_to_world).reshape(4, 4),
        "world_to_view": np.array(world_to_view).reshape(4, 4),
        "projection_type": projection_type,
        "ftheta": ftheta,
        "width": width,
        "height": height,
        "aspect_ratio": width / height,
        "clipping_range": np.array(camera.GetAttribute("clippingRange").Get()),
        "horizontal_aperture": camera.GetAttribute("horizontalAperture").Get(current_time),
        "vertical_aperture": camera.GetAttribute("verticalAperture").Get(current_time),
        "focal_length": camera.GetAttribute("focalLength").Get(current_time),
    }


def _get_parent_indices(joints):
    joint_parent_map = {}

    for joint in joints:
        joint_array = joint.split("/")

        if len(joint_array) == 1:
            joint_parent_map[joint_array[0]] = None
        else:
            joint_parent_map[joint_array[-1]] = joint_array[-2]

    new_joints = list(joint_parent_map.keys())
    parent_indices = [new_joints.index(j) for j in (list(joint_parent_map.values())[1:])]
    parent_indices = [-1] + parent_indices
    return parent_indices


def get_asset_path_from_prim_path(prim_path):
    stage = omni.usd.get_context().get_stage()
    if prim_path:
        prim = stage.GetPrimAtPath(str(prim_path))
    else:
        raise ValueError("Prim path can not be empty!")

    if prim.IsValid():
        asset_path = omni.usd.get_composed_references_from_prim(prim)[0][0].assetPath
    else:
        raise ValueError(f"Prim with path `{prim_path}` is invalid!")

    return asset_path


def get_anim_variant_from_prim_path(char_anim_dup_path):
    """Get animation variant path"""
    if not char_anim_dup_path:
        raise ValueError(f"Path `{char_anim_dup_path}` is invalid")
    stage = omni.usd.get_context().get_stage()
    dup_prim = stage.GetPrimAtPath(str(char_anim_dup_path))
    var_select = None
    # search chilid node has variatntSets name animationVariant
    for child in dup_prim.GetChildren():
        if child.GetPrimTypeInfo().GetTypeName() == "Xform":
            if child.HasVariantSets:
                var_set = child.GetVariantSet("animationVariant")
                var_select = var_set.GetVariantSelection()
                if var_select:
                    break
    return var_select


def compute_2d_translations(points, view_params, skel_data):
    # Compute world-to-image transforms given translation points
    if np.linalg.det(view_params["world_to_view"]) == 0.0:
        carb.log_warn("View matrix determinant is 0.0, can't calculate 2D joint translations!")
        skel_data["translations_2d"] = []
        skel_data["in_view"] = False
    else:
        joint_pos2d = sd.helpers.world_to_image(points, None, view_params)[:, :2]

        joint_pos2d *= np.array([view_params["width"], view_params["height"]])
        skel_data["translations_2d"] = joint_pos2d.tolist()

        # Check if the current skeleton is in view of the viewport
        skel_data["in_view"] = not np.any(
            np.any(joint_pos2d[:, 0] < 0)
            and np.any(joint_pos2d[:, 0] > view_params["width"])
            and np.any(joint_pos2d[:, 1] < 0)
            and np.any(joint_pos2d[:, 1] > view_params["height"])
        )


def _get_skel_joints_global_translations(skel_prim_path: str, skelJnt_attrs: list):
    global_translations = []
    skelJnt_prims = get_skel_joints_prim_path(skel_prim_path, skelJnt_attrs)
    for prim in skelJnt_prims:
        xform = UsdGeom.Xformable(prim)
        global_mtrx = xform.ComputeLocalToWorldTransform(0)
        translation = global_mtrx.ExtractTranslation()
        global_translations.append(translation)

    return np.array(global_translations)


def _get_skel_joints_local_rotations(skel_prim_path: str, skelJnt_attrs: list):
    local_rotations = []
    skelJnt_prims = get_skel_joints_prim_path(skel_prim_path, skelJnt_attrs)
    for prim in skelJnt_prims:
        xform = UsdGeom.Xformable(prim)
        local_mtrx = xform.GetLocalTransformation()
        local_quat = local_mtrx.ExtractRotation().GetQuaternion()
        rotation = np.concatenate(([local_quat.GetReal()], np.array(local_quat.GetImaginary())))
        local_rotations.append(rotation)

    return np.array(local_rotations)


def get_global_translations(prims):
    global_translations = []
    for prim in prims:
        xform = UsdGeom.Xformable(prim)
        global_mtrx = xform.ComputeLocalToWorldTransform(0)
        translation = global_mtrx.ExtractTranslation()
        global_translations.append(translation)

    return global_translations


def get_local_rotations(prims):
    local_rotations = []
    for prim in prims:
        xform = UsdGeom.Xformable(prim)
        local_mtrx = xform.GetLocalTransformation()
        local_quat = local_mtrx.ExtractRotation().GetQuaternion()
        rotation = np.concatenate(([local_quat.GetReal()], np.array(local_quat.GetImaginary())))
        local_rotations.append(rotation)

    return local_rotations


def get_skel_joints_prim_path(skel_prim_path: str, skelJnt_attrs: list):
    skelJnt_prims = []
    stage = omni.usd.get_context().get_stage()

    skelJnt_prim_paths = [f"{skel_prim_path}/{jnt}" for jnt in skelJnt_attrs]
    for path in skelJnt_prim_paths:
        if is_valid_prim_path(path):
            skelJnt_prims.append(stage.GetPrimAtPath(path))
        else:
            carb.log_error(f"Invalid prim path: {path}")

    return skelJnt_prims


def is_valid_prim_path(prim_path, stage=None):
    if not isinstance(prim_path, str):
        return False

    if stage is None:
        context = omni.usd.get_context()
        stage = context.get_stage()

    prim = stage.GetPrimAtPath(prim_path)

    return prim.IsValid()
