# E031：树莓派至 Mcontroller 的 UWB/IMU 短时遥测联调

## 状态

BLOCKED。树莓派可经 TCP 连接 Mcontroller，但飞控在本次记录期间没有输出任何
MAVLink 数据。因此未得到有效 UWB 测距或 IMU 样本，不能计算频率、跳变、残差、
GDOP、IMU 噪声或 UWB-IMU 时间偏差。

## 设备与安全边界

- 树莓派：`192.168.1.143`；实际 UWB 标签为 `2`，不等同于 ROS domain `20`。
- 读取端点：`192.168.1.126:333`。
- 适配器：`uav_readonly_adapter_v2.py`，仅建立 TCP 遥测连接并按厂商要求发送
  被动 GCS `HEARTBEAT`；未发送解锁、模式、航点、速度、高度或参数写入命令。
- 使用树莓派既有 `/home/pi/mavlink_env` 中的 `pymavlink`，未安装依赖。

## 记录

采集命令在树莓派执行 75 秒，输出随后复制到本地：

```text
data/raw/2026-09-07_E031_uwb_imu_short/telemetry_v02.jsonl
```

记录为 633 行。逐行检查结果：

| 字段 | 结果 |
| --- | --- |
| `online` | 0 / 633 为真 |
| `uwb.anchor_ranges_m` | 0 / 633 存在有效距离 |
| `imu.acceleration_mps2` | 0 / 633 存在三轴数值 |
| `imu.angular_velocity_rps` | 0 / 633 存在三轴数值 |

适配器日志反复显示“TCP 已连接”，随后“连续 5.0 秒未收到 MAVLink 数据”。633 行是
适配器在重连期间输出的离线状态快照，不能作为传感器数据回放或统计输入。

## 已完成与未完成

已完成：Pi 网络与 SSH 可达；Mcontroller TCP 端口可连接；标签 ID、四锚点 map
坐标及范围槽位登记可读；只读采集通路可启动。

未完成：飞控遥测输出启用/桥接；四路量程槽位实际对应验证；LOS/NLOS 采集；UWB
残差与 GDOP；IMU 静置噪声、轴向、时间戳；静止/直线/转动回放；UWB-IMU 融合。

## 下一步

使 Mcontroller 向 `192.168.1.126:333` 的 TCP 客户端输出 MAVLink 流，或启动已有的
`fanciswarm_bridge.py` 并提供其只读 UDP 输出。确认收到 `BATTERY_STATUS`（四路
范围）和 `SCALED_IMU` 后，先做 60 秒静置采集，再做 LOS/NLOS 与手持直线/转动
轨迹采集；全程无需起飞或转动螺旋桨。
