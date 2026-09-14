"""Register an operator-selected actual reference frame and collect real chain outputs."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nexus_msgs.msg import TargetKinematicState
from rosidl_runtime_py.convert import message_to_ordereddict
p=argparse.ArgumentParser();p.add_argument('--reference',required=True);p.add_argument('--output',required=True);p.add_argument('--request',required=True);p.add_argument('--target',default='traffic_light_001');p.add_argument('--box',nargs=4,type=float,required=True);p.add_argument('--anchor',nargs=2,type=float);p.add_argument('--seconds',type=float,default=35);a=p.parse_args()
rclpy.init();n=rclpy.create_node('e047_live_target_registration');events={}
def collect(topic,convert):
 def cb(m):events.setdefault(topic,[]).append(convert(m))
 return cb
for topic in ['/nexus/vision/target_request_status','/nexus/vision/target_tracks','/nexus/vision/target_geometry_status','/nexus/viz/localization_state','/nexus/platform/status']:
 n.create_subscription(String,topic,collect(topic,lambda m:json.loads(m.data)),20)
for topic in ['/nexus/vision/target_kinematics','/nexus/target/kinematics']:
 n.create_subscription(TargetKinematicState,topic,collect(topic,message_to_ordereddict),20)
image_pub=n.create_publisher(Image,'/nexus/vision/target_reference_image',20);request_pub=n.create_publisher(String,'/nexus/vision/target_requests',20)
start=time.monotonic()
while time.monotonic()-start<5:
 rclpy.spin_once(n,timeout_sec=.01)
 if image_pub.get_subscription_count() and request_pub.get_subscription_count() and time.monotonic()-start>1:break
frame=deserialize_message(Path(a.reference).read_bytes(),Image)
request=dict(schema_version=1,request_id=a.request,target_id=a.target,sample_timestamp_ns=frame.header.stamp.sec*10**9+frame.header.stamp.nanosec,frame_id=frame.header.frame_id,bbox_xywh_px=a.box,motion_model='static')
if a.anchor:request['anchor_reference_px']=a.anchor
image_pub.publish(frame);request_pub.publish(String(data=json.dumps(request)));print('Registration sent',flush=True)
start=time.monotonic()
while time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.01)
result=dict(request=request,elapsed_s=time.monotonic()-start,counts={k:len(v) for k,v in events.items()},latest={k:v[-6:] for k,v in events.items()})
raw=Path('data/raw/2026-09-08_E047_recovery_and_pose_stream')/(a.request+'_events.json');raw.write_text(json.dumps(events));result['raw_events']=str(raw)
Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,ensure_ascii=False)[-14000:]);n.destroy_node();rclpy.shutdown()
