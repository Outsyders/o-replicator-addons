"""
This is the implementation of the OGN node defined in OgnFocusAt.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy as np
import math

import carb
import omni.graph.core as og
import omni.kit
import omni.timeline
import omni.usd
from omni.replicator.core import utils

from pxr import (
    OmniAudioSchema,
    Gf,
    Tf,
    Kind,
    Sdf,
    Sdr,
    Trace,
    Usd,
    UsdGeom,
    UsdLux,
    UsdShade,
)


from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union


def _get_camera_prim(camera_prim_path: str):
    stage = omni.usd.get_context().get_stage()
    camera = stage.GetPrimAtPath(str(camera_prim_path))

    if camera.HasAttribute("replicatorXform"):
        camera = camera.GetChildren()[0]

    return camera


def _get_time():
    timeline_iface = omni.timeline.get_timeline_interface()
    return timeline_iface.get_current_time() * timeline_iface.get_time_codes_per_seconds()


def calculate_focal_length_from_radius(
    camera_path: str,
    distance: float,
    radius: float,
    use_horizontal_fov: Optional[bool] = None,
    aspect_ratio: float = 1.0,
    conform: Optional[Union[int, str]] = None,
):
    """
    Calculate the focal length of a camera given a radius to fit the bounding sphere of a set of prims.

    Args:
        camera_path (str): The path to the camera prim.
        radius (float): The radius of the bounding sphere to fit.
        use_horizontal_fov (bool, optional): Whether to use the horizontal field of view. Defaults to None.
        aspect_ratio (float, optional): The aspect ratio of the camera. Defaults to 1.0.
        conform (Union[int, str], optional): The conform setting. Defaults to None. Options are:
            - 0 or "vertical": Conform to vertical aperture.
            - 1 or "horizontal": Conform to horizontal aperture.
            - 2 or "fit": Fit the aperture to the aspect ratio.
            - 3 or "crop": Crop the aperture to the aspect ratio.

    Returns:
        float: The calculated focal length.
    """
    camera = _get_camera_prim(camera_path)

    time = _get_time()

    # h_fov_rad, v_fov_rad = self.__horizontal_fov, self.__horizontal_fov

    if camera:
        h_aperture = camera.GetAttribute("horizontalAperture")
        v_aperture = camera.GetAttribute("verticalAperture")
        projection = camera.GetAttribute("projection")

        if h_aperture or v_aperture:
            if h_aperture and not v_aperture:
                v_aperture = h_aperture
            elif v_aperture and not h_aperture:
                h_aperture = v_aperture
            h_aperture = h_aperture.Get(time)
            v_aperture = v_aperture.Get(time)

            if projection.Get(time) == "orthographic":
                new_horz_ap = (max(0.001, radius) / Gf.Camera.APERTURE_UNIT) * 2.0
                new_vert_ap = v_aperture * ((new_horz_ap / h_aperture) if h_aperture else new_horz_ap)
                return (new_horz_ap, new_vert_ap)

            h_fov_rad = math.atan((h_aperture * Gf.Camera.APERTURE_UNIT) / (2.0 * Gf.Camera.FOCAL_LENGTH_UNIT))
            v_fov_rad = math.atan((v_aperture * Gf.Camera.APERTURE_UNIT) / (2.0 * Gf.Camera.FOCAL_LENGTH_UNIT))

    def fit_horizontal():
        if use_horizontal_fov is not None:
            return use_horizontal_fov

        if conform is None:
            conform = carb.settings.get_settings().get("/app/hydra/aperture/conform")

        if conform == 0 or conform == "vertical":
            return False

        is_fit = conform == 2 or conform == "fit"
        if is_fit or (conform == 3 or conform == "crop"):
            fov_aspect = h_fov_rad / v_fov_rad
            return not (is_fit ^ (fov_aspect > aspect_ratio))

        return True

    if fit_horizontal():
        v_fov_rad = h_fov_rad / aspect_ratio
    else:
        h_fov_rad = v_fov_rad * aspect_ratio

    sensor_size = min(h_fov_rad, v_fov_rad)

    if distance == 0 or radius == 0:
        return 0

    focal_length = sensor_size * (distance / radius)

    return focal_length


def calculate_focal_length_from_distance(
    camera_path: str,
    distance: float,
    horizontal_fov=0.2,
    use_horizontal_fov=False,
    aspect_ratio=1,
):
    camera = _get_camera_prim(camera_path)

    if not camera:
        return None

    if horizontal_fov <= 0:
        carb.log_warn(f"Invalid horizontal_fov: {horizontal_fov}")
        return None

    time = _get_time()

    h_aperture = camera.GetAttribute("horizontalAperture").Get(time)
    v_aperture = camera.GetAttribute("verticalAperture").Get(time)

    if use_horizontal_fov is None:
        use_horizontal_fov = aspect_ratio >= 1
    else:
        use_horizontal_fov = horizontal_fov

    if use_horizontal_fov:
        h_aperture = h_aperture if h_aperture else v_aperture * aspect_ratio
        focal_length = (h_aperture * Gf.Camera.APERTURE_UNIT) / (2.0 * math.tan(horizontal_fov / 2.0))
    else:
        v_aperture = v_aperture if v_aperture else h_aperture / aspect_ratio
        focal_length = (v_aperture * Gf.Camera.APERTURE_UNIT) / (2.0 * math.tan(horizontal_fov / 2.0))

    focal_length = distance / (2.0 * math.tan(horizontal_fov / 2.0))

    return focal_length


def compute_local_transform(camera_path: str):
    # stage = omni.usd.get_context().get_stage()
    # prim = stage.GetPrimAtPath(str(camera_path))
    prim = _get_camera_prim(camera_path)

    if not prim:
        carb.log_warn(f"Framing of UsdPrims failed, {camera_path} doesn't exist")
        return None, None, None

    time = _get_time()

    local_xform, world_xform = None, None
    xformable = UsdGeom.Xformable(prim)
    if xformable:
        local_xform = xformable.GetLocalTransformation(time)

    imageable = UsdGeom.Imageable(prim)

    if imageable:
        parent_xform = imageable.ComputeParentToWorldTransform(time)
        if not local_xform:
            world_xform = imageable.ComputeLocalToWorldTransform(time)
            local_xform = world_xform * parent_xform.GetInverse()
        if not world_xform:
            world_xform = parent_xform * local_xform
        return local_xform, parent_xform, world_xform

    carb.log_warn(f"Framing of UsdPrims failed, {camera_path} isn't UsdGeom.Xformable or UsdGeom.Imageable")
    return None, None, None


def compute_bounds(camera_path: str, target_paths: List[str]):
    aabbox = Gf.Range3d()
    usd_context = omni.usd.get_context()

    def add_to_range(prim_path):
        aab_min, aab_max = usd_context.compute_path_world_bounding_box(str(prim_path))

        in_range = Gf.Range3d(Gf.Vec3d(*aab_min), Gf.Vec3d(*aab_max))

        if in_range.IsEmpty():
            aa_range = Gf.Range3d(Gf.Vec3d(-20, -20, -20), Gf.Vec3d(20, 20, 20))
            matrix = Gf.Matrix4d(*usd_context.compute_path_world_transform(str(prim_path)))
            bbox = Gf.BBox3d(aa_range, matrix)
            in_range = bbox.ComputeAlignedRange()
            if in_range.IsEmpty():
                pos = matrix.ExtractTranslation()
                in_range.SetMin(pos - aa_range.GetMin())
                in_range.SetMax(pos + aa_range.GetMax())

        aabbox.UnionWith(in_range)

    for prim_path in target_paths:
        if prim_path != camera_path:
            add_to_range(prim_path)

    return aabbox


class OgnCalculateFocalLength:
    """
    Set prim rotation and focal length to look at the target coordinates.
    This is a modified version of the focus command.
    """

    @staticmethod
    def compute(db) -> bool:
        camera_prim_path: Sequence[Union[str, Sdf.Path]] = db.inputs.prims
        target_prim_paths: Union[str, Sdf.Path] = db.inputs.targetPrim
        zoom: float = db.inputs.zoom
        set_focal_length: bool = db.inputs.setFocalLength
        use_horizontal_fov: bool = db.inputs.useHorizontalFov
        conform: Union[int, str] = db.inputs.conform

        def failed():
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        if len(camera_prim_path) == 0 or camera_prim_path is None:
            return failed()

        if len(target_prim_paths) == 0:
            return failed()

        camera_prim_path = camera_prim_path[0]

        try:
            local_xform, parent_xform, world_xform = compute_local_transform(camera_prim_path)

            aabbox = compute_bounds(camera_prim_path, target_prim_paths)

            if aabbox.IsEmpty():
                carb.log_warn(f"Framing of UsdPrims {target_prim_paths} resulted in an empty bounding-box")
                return failed()

            if True:
                # Orient the aabox to the camera
                target = aabbox.GetMidpoint()
                tr0 = Gf.Matrix4d().SetTranslate(-target)
                local_rot = Gf.Matrix4d().SetRotate(local_xform.GetOrthonormalized().ExtractRotationQuat())
                tr1 = Gf.Matrix4d().SetTranslate(target)

                # And compute the new range
                aabbox = Gf.BBox3d(aabbox, tr0 * local_rot * tr1).ComputeAlignedRange()

            # Compute where to move in the parent space
            aabbox = Gf.BBox3d(aabbox, parent_xform.GetInverse()).ComputeAlignedRange()

            # Target is in parent-space (just like the camera / object we're moving)
            target = aabbox.GetMidpoint()

            # Compute the distance to the target
            camera_position = local_xform.ExtractTranslation()
            distance = (camera_position - target).GetLength()

            # Frame against the aabox's bounding sphere
            radius = aabbox.GetSize().GetLength() * zoom  # * distance

            # carb.log_info(aabbox)
            # carb.log_info(f"Distance: {distance}, Radius: {radius}")

            focal_length = calculate_focal_length_from_radius(
                camera_prim_path, distance, radius, use_horizontal_fov, conform=conform
            )

        except Exception as error:
            db.log_error(f"FocusAt Error: {error}")
            return failed()

        if focal_length is None:
            return failed()

        if set_focal_length:
            with Sdf.ChangeBlock():
                camera = _get_camera_prim(camera_prim_path)

                camera.GetAttribute("focalLength").Set(focal_length)

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        db.outputs.values = [focal_length]
        return True
