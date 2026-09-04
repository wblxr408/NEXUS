# 树莓派只读接入完成清单（2026-09-04）

| 项目 | 状态 | 证据 |
|---|---|---|
| 飞控只读遥测入口 | VALIDATED（无真实输入） | 新入口不启动 `fcu_mission`/控制转发；树莓派单测通过 |
| IMU 单位修正版 | VALIDATED（合成值） | 三轴 `/1000`，`z=-9870` 得 `-9.87 m/s²` |
| 标签 2、基站、9° 配置 | IMPLEMENTED | v01 配置；基站状态仍为用户提供、未 survey |
| UWB 到 `/nexus/uwb/ranges` | VALIDATED（跨设备合成） | 17 条初测及 E024 录包 1152 条 |
| 飞控时间到 ROS 时间 | VALIDATED（合成） | boot delta 保持、reset 测试、跨设备 ROS 时间戳 |
| IMX219 Image/时间戳 | VALIDATED（实机） | 1640×1232 RGB、SensorTimestamp、跨设备 ROS topic |
| 树莓派只读记录 | PARTIALLY VALIDATED | 相机 68 帧；飞控断电时遥测正确标记 offline |
| 链路健康检查 | VALIDATED | 遥测速率/丢样/延迟与相机帧率/错误统计均有 topic |
| 标签 2、Domain 0 部署 | VALIDATED | `/home/pi/nexus_readonly_stack` 单测通过；旧文件校验不变 |
| 树莓派到本机接收 | VALIDATED | TCP 遥测、真实相机、7 话题实机相机录包及 8 话题最新版软件契约均通过 |

实机飞控数据、IMU 静置噪声、UWB 已知点、外参以及飞行精度不在上述
VALIDATED 范围内。
