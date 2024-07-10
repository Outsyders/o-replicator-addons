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


class OgnSetCameraParams:
    """
    Set parameters on a camera prim.
    """

    @staticmethod
    def compute(db) -> bool:
        camera_prim_path: Sequence[Union[str, Sdf.Path]] = db.inputs.cameraPrim
        params: Sequence[Any] = db.inputs.params
        values: Sequence[Any] = db.inputs.values

        def failed():
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        stage = omni.usd.get_context().get_stage()
        camera = stage.GetPrimAtPath(str(camera_prim_path))
        if camera.HasAttribute("replicatorXform"):
            camera = camera.GetChildren()[0]

        current_time = omni.timeline.get_timeline_interface().get_current_time()

        try:
            attribute_names = [attr.GetName() for attr in camera.GetAttributes()]
            ret = {}
            for param, value in zip(params, values):
                if param not in attribute_names and f"inputs:{param}" in attribute_names:
                        # fallback to inputs prefix
                        param = f"inputs:{param}"

                if param not in attribute_names:
                    carb.log_warning(f"Camera does not have attribute: {param}")
                    continue

                camera.GetAttribute(param).Set(value, current_time)
                ret[param] = value
        except Exception as e:
            carb.log_error(f"Failed to set camera parameter: {e}")
            return failed()
        
        # https://openusd.org/dev/api/class_sdf_change_block.html
        # with Sdf.ChangeBlock():
        #     camera.

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        db.outputs.values = ret
        return True
