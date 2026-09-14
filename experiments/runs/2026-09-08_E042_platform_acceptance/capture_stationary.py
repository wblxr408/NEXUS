import json,time,subprocess,shlex,os
from pathlib import Path
from collections import Counter
from pymavlink import mavutil
out=Path('/home/pi/nexus_readonly_runs/2026-09-08_platform_acceptance')
pids=subprocess.check_output(['pgrep','-x','fcu_bridge_001'],text=True).split()
assert len(pids)==1,pids
pid=pids[0]
args=Path('/proc/'+pid+'/cmdline').read_bytes().decode().strip('\0').split('\0')
(out/'original_bridge_args.json').write_text(json.dumps(args))
tcp=mavutil.mavlink_connection('tcp:192.168.1.126:333',source_system=253)
hb=tcp.recv_match(type='HEARTBEAT',blocking=True,timeout=8)
tcp.close()
assert hb is not None and not hb.base_mode & 128,'No current disarmed heartbeat'
print('Verified disarmed heartbeat',flush=True)
restore='source /opt/ros/humble/setup.bash; source /root/fcu_core_onboard/install/setup.bash; export ROS_DOMAIN_ID=20; exec '+shlex.join(args)
subprocess.run(['docker','exec','fcu_ros2','pkill','-INT','-x','fcu_bridge_001'],check=True)
master=None
try:
 for _ in range(50):
  if not Path('/proc/'+pid).exists():break
  time.sleep(.1)
 assert not Path('/proc/'+pid).exists(),'Bridge did not release serial'
 master=mavutil.mavlink_connection('/dev/ttyAMA0',baud=115200,source_system=254,source_component=190)
 with (out/'stationary_v02.jsonl').open('x') as f:
  for role,duration in [('stationary',180)]:
   start=time.monotonic();next_hb=0;counts=Counter()
   while time.monotonic()-start<duration:
    elapsed=time.monotonic()-start
    if elapsed>=next_hb:
     master.mav.heartbeat_send(18,8,0,0,4)
     next_hb=elapsed+1
    msg=master.recv_match(blocking=True,timeout=.05)
    if msg is None:continue
    if msg.get_type()=='HEARTBEAT' and msg.base_mode & 128:raise RuntimeError('Armed heartbeat; stopping probe')
    counts[msg.get_type()]+=1
    f.write(json.dumps(dict(role=role,elapsed_s=elapsed,receive_unix_ns=time.time_ns(),message=msg.to_dict()), default=lambda value: {'bytes_hex': bytes(value).hex()})+'\n')
   print(role,dict(counts),flush=True)
finally:
 if master is not None:master.close()
 subprocess.run(['docker','exec','-d','fcu_ros2','bash','-lc',restore],check=True)
 print('Original bridge restarted',flush=True)
