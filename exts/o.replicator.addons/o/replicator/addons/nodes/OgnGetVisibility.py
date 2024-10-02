import carb
import omni.graph.core as og
from omni.replicator.core import utils
from pxr import Sdf, UsdGeom


def compute_visibility(prim):
    """Computes the visibility state of a given prim.

    This function checks the visibility attribute of the given prim and its ancestors
    in the USD scene hierarchy. If any ancestor's visibility is set to invisible,
    the function will return 'invisible'. Otherwise, it will return 'inherited'.

    Args:
        prim (usdrt.Usd.Prim): The prim to compute visibility for.

    Returns:
        str: The visibility state of the prim, either 'invisible' or 'inherited'."""
    imageable = UsdGeom.Imageable(prim)
    if imageable:
        visibility_attr = imageable.GetVisibilityAttr()
        if visibility_attr.IsValid():
            if visibility_attr.Get() == UsdGeom.Tokens.invisible:
                return UsdGeom.Tokens.invisible

    parent = prim.GetParent()
    if parent:
        # Check parent visibility
        return compute_visibility(prim.GetParent())

    return UsdGeom.Tokens.inherited


class OgnGetVisibility:
    @staticmethod
    def compute(db) -> bool:
        targets = db.inputs.prims

        meshes = utils.find_prims(targets, "prims")

        samples = []
        with Sdf.ChangeBlock():
            for idx, mesh in enumerate(meshes):
                # UsdGeom.PrimvarsAPI(mesh).GetPrimvar("hideForCamera").Set(False)
                # UsdGeom.PrimvarsAPI(mesh).GetPrimvar("doNotCastShadows").Set(False)
                # if mesh.HasAttribute("visibility"):
                #     sample = mesh.GetAttribute("visibility").Get()
                #     samples.append(sample)
                sample = compute_visibility(mesh)
                if sample == UsdGeom.Tokens.invisible:
                    samples.append(False)
                else:
                    samples.append(True)
                # else:
                #     carb.log_warn(f"{mesh} has no visibility attribute. Skipping...")

        # TODO validation
        if len(samples) != len(meshes):
            db.log_error(
                f"Number of input values is different from number of input prims: {len(samples)} != {len(meshes)}"
            )
            return False

        db.outputs.values = samples
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
