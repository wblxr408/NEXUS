"""CPU-only online visual front-end primitives used by the ROS2 adapter.

The package deliberately has no ROS imports.  ROS nodes can feed images and
detector results into these deterministic components, which also makes the
front-end replayable in the offline experiment harness.
"""

from .patch_tracker import PatchTracker, TrackedTarget
from .detect_arbiter import Detection, DetectionArbiter
from .vio_bridge import ImuSample, UvioPose, VioBridge, scaled_imu_to_si

__all__ = [
    "Detection", "DetectionArbiter", "ImuSample", "PatchTracker",
    "TrackedTarget", "UvioPose", "VioBridge", "scaled_imu_to_si",
]
