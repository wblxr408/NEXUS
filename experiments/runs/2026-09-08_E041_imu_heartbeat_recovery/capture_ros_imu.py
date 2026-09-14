import collections,json,math,time,statistics
from pathlib import Path
import rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n=rclpy.create_node('imu_recovery_validation')
rows=[];counts=collections.Counter()
def imu(m):
 counts['imu']+=1
 rows.append(dict(receive_unix_ns=time.time_ns(),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,acc=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],gyro=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]))
def count(name):
 def cb(m):counts[name]+=1
 return cb
subs=[n.create_subscription(Imu,'/imu_global_001',imu,qos_profile_sensor_data),n.create_subscription(String,'/nexus/uwb/ranges',count('uwb'),qos_profile_sensor_data),n.create_subscription(Odometry,'/odom_global_001',count('odom'),qos_profile_sensor_data)]
start=time.monotonic()
while time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.1)
Path('/root/fcu_core_onboard/log/imu_recovery_ros.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
summary=dict(counts=dict(counts),elapsed_s=time.monotonic()-start)
if rows:
 d=[(b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(rows,rows[1:])]
 summary.update(hz=(len(rows)-1)/((rows[-1]['receive_unix_ns']-rows[0]['receive_unix_ns'])/1e9),non_increasing_stamps=sum(x<=0 for x in d),max_stamp_gap_s=max(d),frames=dict(collections.Counter(r['frame'] for r in rows)),acc_norm_mean=statistics.mean(math.sqrt(sum(x*x for x in r['acc'])) for r in rows),finite=all(math.isfinite(x) for r in rows for x in r['acc']+r['gyro']))
Path('/root/fcu_core_onboard/log/imu_recovery_ros_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
n.destroy_node();rclpy.shutdown()
assert counts['imu']>=100,'Actual IMU stream missing'
