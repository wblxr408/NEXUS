"""Offline fault audit against production platform and pose timeline."""
import json
from pathlib import Path
import numpy as np
from nexus_fusion_localization.platform_window import PlatformWindow, PlatformConfig
from nexus_fusion_localization.platform_state import BodyState
from nexus_fusion_localization.imu_preintegration import ImuReading
from nexus_fusion_localization.platform_measurements import PositionMeasurement
from nexus_vision_localization.pose_timeline import PoseTimeline, TimedPose

window = PlatformWindow(PlatformConfig(window_size=4, kernel="linear", irls_iterations=1))
window.initialize(BodyState(1_000_000_000, [0,0,1], [0,0,0], np.eye(3), np.zeros(3), np.zeros(3)), np.eye(15)*.01)
for ns in range(1_000_000_000, 1_201_000_000, 5_000_000):
    window.add_imu(ImuReading(ns, [0,0,9.80665], [0,0,0]))
first=window.step(1_100_000_000,[PositionMeasurement(1_100_000_000,[0,0,1],np.eye(3)*.01)])
assert first.valid
for ns in range(1_700_000_000,3_001_000_000,5_000_000):
    window.add_imu(ImuReading(ns,[0,0,9.80665],[0,0,0]))
recovery=[]
for ns in [1_800_000_000,2_000_000_000,2_500_000_000,3_000_000_000]:
    result=window.step(ns,[PositionMeasurement(ns,[0,0,1],np.eye(3)*.01)])
    recovery.append({"stamp_ns":ns,"valid":result.valid,"reason":result.reason})
timeline=PoseTimeline()
for ns in [1_000_000_000,2_000_000_000]:
    timeline.add(TimedPose(ns,np.eye(4),np.eye(6)*.01))
try:
    timeline.at(1_100_000_000)
    timing="accepted"
except ValueError as error:
    timing=str(error)
result={"synthetic":True,"initial_normal_step_valid":first.valid,
        "after_500ms_imu_gap":recovery,"camera_frame_between_1hz_platform_poses":timing}
Path(__file__).with_suffix(".json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
