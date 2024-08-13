import carb
import omni.graph.core as og
import omni.usd
import usdrt
from typing import List, Union, Tuple, Dict, Any
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdShade, UsdRender

import asyncio

import omni.replicator.core as rep
from omni.replicator.core.utils import get_non_xform_prims, viewport_manager, ReplicatorItem

from omni.isaac.core_nodes import BaseResetNode


async def _get_replicator_item_output_paths_async(node: ReplicatorItem) -> List[str]:
    """
    Get the output paths from the node asynchronously
    """
    while not node.get_outputs().get("prims") and node.node.get_compute_count() == 0:
        await omni.kit.app.get_app().next_update_async()

    paths = [str(p) for p in node.get_output("prims")]

    if not paths:
        raise ValueError(f"Unable to get output path from {node}")

    return paths


def _get_replicator_item_output_paths(item: ReplicatorItem) -> List[str]:
    """
    Get the output paths from the `ReplicatorItem` synchronously.

    Args:
        item (List[ReplicatorItem]): The replicator item to get the output paths from.

    Returns:
        List[str]: The output paths from the replicator item.
    """
    if not item.get_outputs().get("prims"):
        paths = asyncio.ensure_future(_get_replicator_item_output_paths_async(item))
    else:
        paths = [str(p) for p in item.get_outputs().get("prims")]

    return paths


def _get_camera_path(camera: Union[ReplicatorItem, str, Sdf.Path]) -> str:
    if isinstance(camera, list):
        camera = camera[0]

    if isinstance(camera, ReplicatorItem):
        camera = _get_replicator_item_output_paths(camera)[0]

    if isinstance(camera, (str, Sdf.Path, usdrt.Sdf.Path)):
        camera = get_non_xform_prims([str(camera)])
    else:
        raise ValueError(f"Unable to get camera path from {camera}")

    if isinstance(camera, list):
        camera = camera[0]

    return camera


def _get_render_product(render_product: Union[ReplicatorItem, str, Sdf.Path, UsdRender.Product]) -> UsdRender.Product:
    if isinstance(render_product, list):
        render_product = render_product[0]

    if isinstance(render_product, ReplicatorItem):
        render_product = _get_replicator_item_output_paths(render_product)[0]

    if isinstance(render_product, (str, Sdf.Path, usdrt.Sdf.Path)):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(str(render_product))
        render_product = UsdRender.Product(prim)

    if not isinstance(render_product, UsdRender.Product):
        raise ValueError(f"Unable to get render product from {render_product}")

    return render_product


class OgnSetRenderProductInternalState(BaseResetNode):
    def __init__(self):
        self.manager = viewport_manager.ViewportManager()
        self.handle = None
        self.render_product_path = None
        self.factory = None
        self.resolution = [0, 0]
        self.camera_path = ""
        super().__init__(initialize=False)

    def on_stage_event(self, event: carb.events.IEvent):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if self.handle:
                self.handle.hydra_texture.set_updates_enabled(False)
            self.initialized = False
        elif event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.handle:
                self.handle.hydra_texture.set_updates_enabled(True)


class OgnSetRenderProduct:
    """
    Set the camera for the render product
    """

    @staticmethod
    def internal_state():
        return OgnSetRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        render_product: Union[ReplicatorItem, str, Sdf.Path, UsdRender.Product] = db.inputs.renderProduct
        camera: Union[ReplicatorItem, str, Sdf.Path] = db.inputs.prims
        state = db.per_instance_state

        def failed():
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        if not camera or not render_product:
            db.log_error(f"Camera and render product must be provided. Got {camera} and {render_product}.")
            return failed()

        camera_path = _get_camera_path(camera)
        render_prod_prim = _get_render_product(render_product)

        if not camera_path:
            db.log_error(f"Camera path must be provided. Got {camera_path}.")
            return failed()

        if not render_prod_prim:
            db.log_error(f"Render product must be provided. Got {render_product}.")
            return failed()

        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if state.handle is None:
                state.render_product_path = str(render_prod_prim.GetPath())
                state.handle = state.manager._hydra_textures.get(state.manager._context, state.render_product_path)
                state.camera_path = render_prod_prim.GetCameraRel().GetTargets()[0]  # get from prim
                state.resolution = render_prod_prim.GetResolutionAttr().Get()

            if db.inputs.width != 0 and db.inputs.height != 0:
                if state.resolution[0] != db.inputs.width or state.resolution[1] != db.inputs.height:
                    render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(db.inputs.width, db.inputs.height))
                    state.resolution = (db.inputs.width, db.inputs.height)

            if state.camera_path != camera_path:
                render_prod_prim.GetCameraRel().SetTargets([camera_path])
                state.camera_path = camera_path

            if state.render_product_path is not None:
                db.outputs.renderProductPath = state.render_product_path
                db.outputs.cameraPath = state.camera_path

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            from omni.replicator.core.ogn.OgnSetRenderProductDatabase import OgnSetRenderProductDatabase

            state = OgnSetRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            if state.handle:
                state.handle.destroy()
            state.handle = None
            state.rp_sub = None
