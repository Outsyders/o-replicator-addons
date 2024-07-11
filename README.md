# OmniGraph Extension [o.replicator.addons]
Extension with implementation of some OmniGraph nodes

## Calculate Focal Length
*Calculates the focal length of a camera based on the distance between the camera and the target primitives.*

**Inputs**:
- Camera Path: Path to the camera
- Target Prims: Primitives to calculate the focal length to
- Zoom: Zoom factor

**Outputs**:
- Focal Length: Focal length of the camera

### Example Usage
*Note that this extension monkey patches the `omni.replicator.core` module. The module can also be accessed from `o.replicator.addons`.*
```
import omni.replicator.core as rep

# Import addons will patch the core module
# You can also use functions directly from the addon module
import o.replicator.addons 

camera =rep.create.camera()
target = target = rep.create.sphere()

with rep.trigger.on_frame():
	with camera:
		rep.modify.pose(look_at=target, position=rep.distribution.uniform([-1000,0,-1000], [1000,0,1000]))
		rep.modify.focus(focus_on=target, zoom=rep.distribution.uniform(1, 4))
```

## More to come...