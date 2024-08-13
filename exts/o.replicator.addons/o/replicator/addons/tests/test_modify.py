from omni.replicator.core.utils import (
    viewport_manager,
    ReplicatorItem,
    ReplicatorWrapper,
    create_node,
    # _set_node_input,
    set_target_prims,
    utils,
)
import omni.graph.core as og
import omni.replicator.core as rep
import omni
from pxr import Gf, Sdf, Semantics, Tf, Usd, UsdGeom, UsdShade, UsdRender

await omni.usd.get_context().new_stage_async()

rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512))

print(type(rp))
print(rp.path)
print(rp.hydra_texture.camera_path)


def send_og_event(event_name: str) -> None:
    event_node = ReplicatorItem(create_node, "omni.graph.action.SendCustomEvent")
    event_node.node.get_attribute("inputs:eventName").set(event_name)
    return event_node


# with rep.new_layer():
cams = []
for i in range(3):
    cam = rep.create.camera()
    cams.append(cam)

with rep.trigger.on_custom_event("set_camera"):
    choices = rep.create.choice(cams)
    rep.modify.render_product(rp, choices)

with rep.trigger.on_frame():
    # rep.utils.send_og_event("set_camera")
    send_og_event("set_camera")


# import omni.graph.core as og
# import omni.usd
# import usdrt
# from omni.syntheticdata.scripts.SyntheticData import SyntheticData
# from pxr import Gf, Sdf, Semantics, Tf, Usd, UsdGeom, UsdShade, UsdRender

# REPLICATOR_SCOPE = "/Replicator"
# GRAPH_PATH = f"{REPLICATOR_SCOPE}/SDGPipeline"


# def get_render_product_at_path(path: str) -> viewport_manager.HydraTexture:
#     stage = omni.usd.get_context().get_stage()
#     if not stage:
#         return None
#     render_prim = stage.GetPrimAtPath(path)
#     if not render_prim.IsValid():
#         return None
#     return render_prim
#     # return viewport_manager.HydraTexture(render_prim)


# _manager = viewport_manager.ViewportManager()
# _render_product = _manager._hydra_textures.get(_manager._context, str(rp.path))

# print(_render_product.path)

stage = omni.usd.get_context().get_stage()
render_prim = UsdRender.Product(stage.GetPrimAtPath(rp.path))
print(dir(render_prim))

print(render_prim.GetPath())

print(render_prim.GetCameraRel().GetTargets()[0])

print(render_prim.GetOrderedVarsRel().GetForwardedTargets())
