# E032 飞控遥测流与 IMU 可用性探测

日期：2026-09-07  
状态：PARTIALLY COMPLETE（原始 UWB 测距已恢复；IMU 原始流仍未输出）

## 目的

验证树莓派机载 ROS2 桥接端能否从已上电飞控接收心跳、位置、原始 IMU 与原始 UWB 锚点测距；不执行解锁、起飞、模式切换或航点指令。

## 配置

- 树莓派机载容器：`fcu_ros2`，host 网络，`ROS_DOMAIN_ID=20`。
- UWB 物理标签 ID：`2`（与 ROS 域 ID 无关）。
- 串口：`/dev/ttyAMA0`，115200 baud，8N1。
- 桥接参数：`channel=0`、`offboard=false`、`set_goal=false`、`use_uwb=true`。
- 为请求标准 IMU，桥接器在首个飞控心跳后一次性发送：
  - `MAV_CMD_SET_MESSAGE_INTERVAL`，消息 `SCALED_IMU (26)`，20,000 µs（50 Hz）；
  - 兼容回退 `REQUEST_DATA_STREAM(RAW_SENSORS, 50 Hz)`。

## 执行与证据

1. 飞控上电后，桥接器发现 `system=1, component=1`，持续接收心跳；串口初始化成功。
2. `/odom_global_001` 和 `/gnss_global_001` 实测发布频率约 10 Hz。
3. `/imu_global_001` 在两轮标准请求后均为 0 Hz，未收到 `COMMAND_ACK`。
4. 临时只读协议探针观测到的输入 MAVLink 消息 ID 为：
   `0 HEARTBEAT`、`2 SYSTEM_TIME`、`33 GLOBAL_POSITION_INT`、
   `48 SET_GPS_GLOBAL_ORIGIN`、`101 GLOBAL_VISION_POSITION_ESTIMATE`、
   `111 TIMESYNC`、`147 BATTERY_STATUS`。
5. 未观测到 `SCALED_IMU` 或 `ATTITUDE_QUATERNION`。但此处此前的“没有原始 UWB”结论不正确：厂商复用了 `BATTERY_STATUS.voltages[2:6]`，四个槽位分别是基站 1--4 的原始距离，单位 cm。
6. 动力系统重启后，机载桥接器将该既有厂商映射接入 `/nexus/uwb/ranges`：标签 `2`、锚点顺序 `["1","2","3","4"]`、厘米转米、`0/65535` 无效值拒绝。ROS 实测完整消息为 `[5.19, 2.30, 4.00, 5.65] m`，发布频率为 `1.000 Hz`；飞控界面同一时段显示 `[5.16, 2.28, 3.98, 5.66] m`，属于 1 Hz 更新中的相邻采样。
7. 临时协议探针已移除。当前机载桥接器已重新构建并运行，且仅发布遥测；未发送解锁、起飞、模式切换或航点指令。
8. 设备侧构建所用文件校验：`fcu_bridge_001.cpp` 为 `b6e218b818700629f66e700a67e364d6921438854e1a72ea628498e541e8bbc7`，`fcu_bridge.h` 为 `d10c867180aeeed4821bf5df0c28b5feac1980d042fbdd75d371dd98dba487d6`。`ros2 node info /fcu_bridge_001` 已列出 `/nexus/uwb/ranges (std_msgs/msg/String)` 发布端。

## 结论

- 已验证：飞控—串口—树莓派—ROS2 的心跳、位置遥测及标签 2 的四路原始 UWB 测距链路。
- 未验证：UWB 锚点槽位与测量坐标的几何一致性、LOS/NLOS、跳变/残差、GDOP；以及 IMU 噪声、IMU 时间戳和 UWB-IMU 融合。
- `odom_global_001` 由 `GLOBAL_VISION_POSITION_ESTIMATE` 映射，不能替代原始 UWB 测距；`gnss_global_001` 由 `GLOBAL_POSITION_INT` 映射，且实测心跳中 `sat=0`，不能作为真实 GNSS 验证数据。

## 后续阻塞条件

1. 飞控固件或厂商协议提供 `SCALED_IMU` 或等价 IMU 消息流；
2. 对四路 UWB 距离补充质量/NLOS 状态与传感器时间戳；
3. 用已知位置的静态样本确认 `voltage[2:6]` 与锚点 1--4 的顺序及 `map` 坐标系一致。

因此可以表述为“原始 UWB 距离已实机接入并记录”，但不得表述为“原始 IMU 或 UWB--IMU 融合已完成实机验证”。
