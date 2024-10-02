import omni
import pxr


def find_materials_by_name(name: str):
    """
    Find all materials in the scene with the given name
    :return: List of transparent materials
    """
    # ToDO: search by material transparency

    stage = omni.usd.get_context().get_stage()
    found_prims = []
    for prim in stage.Traverse():
        # Check the material for glass
        if omni.usd.is_prim_material_supported(prim):
            mat, rel = pxr.UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()

            if not mat:  # ignore inherited materials
                continue

            mat_path = mat.GetPath().pathString

            if name not in mat_path.lower():
                continue

            print(f"Found glass material at {prim.GetPath().pathString}")

            found_prims.append(pxr.UsdGeom.Imageable(prim))

    return found_prims
