import collections,json,time,sys
import rclpy
from sensor_msgs.msg import Imu,Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('e047_live_chain_probe')
counts=collections.Counter(); latest={}; stamps={}; reasons=collections.Counter(); poses=[]
def cb(topic):
 def receive(m):
  counts[topic]+=1
  if hasattr(m,'data') and isinstance(m.data,str):
   try:
    latest[topic]=json.loads(m.data)
    if 'reason' in latest[topic]: reasons[latest[topic]['reason']]+=1
   except ValueError: pass
  if isinstance(m,Odometry):
   poses.append(dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,xyz=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]))
  if hasattr(m,'header'): stamps.setdefault(topic,[]).append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
 return receive
for topic,typ in [('/imu_global_001',Imu),('/nexus/fcu/imu',Imu),('/nexus/camera/imx219/image_raw',Image),('/nexus/platform/odom',Odometry),('/nexus/platform/status',String),('/nexus/pi/fcu_clock_status',String),('/nexus/pi/camera_health',String),('/nexus/platform/initialization_status',String),('/nexus/uwb/startup_statistics',String),('/nexus/uwb/ranges_smoothed',String)]:
 n.create_subscription(typ,topic,cb(topic),QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT) if typ is Imu else qos_profile_sensor_data)
start=time.monotonic()
while time.monotonic()-start<float(sys.argv[1]): n.executor if False else rclpy.spin_once(n,timeout_sec=.1)
result=dict(elapsed_s=time.monotonic()-start,counts=dict(counts),latest=latest,reasons=dict(reasons),timing={k:dict(hz=(len(v)-1)*1e9/(v[-1]-v[0]) if len(v)>1 and v[-1]>v[0] else 0,max_gap_s=max((b-a)/1e9 for a,b in zip(v,v[1:])) if len(v)>1 else None) for k,v in stamps.items()})
if poses:
 from pathlib import Path
 import hashlib,math
 raw=Path('data/raw/2026-09-08_E047_recovery_and_pose_stream')/('poses_'+str(time.time_ns())+'.jsonl');raw.parent.mkdir(parents=True,exist_ok=True);raw.write_text(''.join(json.dumps(p)+'\n' for p in poses))
 result['pose_summary']=dict(first=poses[0],last=poses[-1],max_step_m=max((math.dist(a['xyz'],b['xyz']) for a,b in zip(poses,poses[1:])),default=0),raw_path=str(raw),sha256=hashlib.sha256(raw.read_bytes()).hexdigest())
print(json.dumps(result,indent=2)); n.destroy_node();rclpy.shutdown()
