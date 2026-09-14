# 机载串口 IMU 心跳修复

目标是树莓派 `/home/pi/fcu_core_onboard/src/fcu_core_ros2`，不是本仓库 ROS1 第三方源码。
2026-09-08 已部署并验证，见 `experiments/runs/2026-09-08_E041_imu_heartbeat_recovery/run.md`。

当前 `offboard=false` 会让旧桥接发送 GCS 心跳；Mcontroller 串口收到 ONBOARD_CONTROLLER 心跳后才开始输出本轮实际观测到的 SCALED_IMU。修复使 `MAVLINK_COMM_0` 使用机载身份，保留 offboard 的控制行为、网络身份规则及其他心跳字段。没有修改 IMU 单位或数据转换。

补丁同时增加四种 channel/offboard 组合的报文测试，并修正未跟随既有遥测请求更新的旧测试断言；飞行指令字段测试保持。

在目标目录先检查（已经部署的设备不要重复应用）：

```bash
patch --dry-run -l -p1 -i /path/to/imu_heartbeat.patch
patch -l -p1 -i /path/to/imu_heartbeat.patch
```

适用的修复前 SHA-256：

- `src/fcu_bridge.h`: `d10c867180aeeed4821bf5df0c28b5feac1980d042fbdd75d371dd98dba487d6`
- `test/test_control_packets.cpp`: `cfd8ec539faec5067351af24dc4b6657cd0812725cac9706c865134c40b71017`

部署时保留了原文件备份，位于树莓派 `/home/pi/nexus_readonly_runs/2026-09-08_imu_recovery/` 的 `.before` 文件。回滚需恢复这两个文件、重新构建 fcu_core，再于未解锁状态重启桥接。构建与测试必须使用机载 ROS2 环境；实机串口只能由一个进程读取。
