import json,time,collections,sys
from pathlib import Path
import rclpy
from std_msgs.msg import String
from nexus_msgs.msg import TargetKinematicState
from rosidl_runtime_py.convert import message_to_ordereddict
rclpy.init();n=rclpy.create_node("e047_target_observer");events={}
def cb(t,convert):
 def receive(m):events.setdefault(t,[]).append(convert(m))
 return receive
for t in ["/nexus/vision/motion_status","/nexus/vision/target_tracks","/nexus/vision/target_geometry_status","/nexus/viz/localization_state","/nexus/platform/status","/nexus/platform/initialization_status"]:n.create_subscription(String,t,cb(t,lambda m:json.loads(m.data)),100)
for t in ["/nexus/vision/target_kinematics","/nexus/target/kinematics"]:n.create_subscription(TargetKinematicState,t,cb(t,message_to_ordereddict),100)
start=time.monotonic()
while time.monotonic()-start<float(sys.argv[1]):rclpy.spin_once(n,timeout_sec=.02)
raw=Path("data/raw/2026-09-08_E047_recovery_and_pose_stream")/("target_events_"+str(time.time_ns())+".json");raw.write_text(json.dumps(events))
r=dict(counts={k:len(v) for k,v in events.items()},reasons={k:dict(collections.Counter(x.get("reason","") for x in v)) for k,v in events.items()},latest={k:v[-1] for k,v in events.items()},raw=str(raw));Path(sys.argv[2]).write_text(json.dumps(r,indent=2));print(json.dumps(r)[-20000:]);n.destroy_node();rclpy.shutdown()
