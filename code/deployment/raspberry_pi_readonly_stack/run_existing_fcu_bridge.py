"""Restore the captured FC bridge argv, with durable parameters and no mission node."""
import json
import os
import shlex
from pathlib import Path

root = Path('/home/pi/nexus_readonly_stack/config')
args = json.loads((root / 'fcu_bridge_args.json').read_text())
command = 'source /opt/ros/humble/setup.bash && source /root/fcu_core_onboard/install/setup.bash && exec ' + shlex.join(args)
os.execvp('docker', ['docker', 'exec', '-e', 'ROS_DOMAIN_ID=20', 'fcu_ros2', 'bash', '-lc', command])
