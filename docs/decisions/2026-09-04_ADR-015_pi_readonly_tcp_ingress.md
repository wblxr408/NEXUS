# ADR-015：树莓派只读 TCP 接入与双时钟对齐

日期：2026-09-04  
状态：Accepted（软件链路与实机相机已验证；真实飞控输入待供电）

## 背景

旧 `fcu_core` 启动会同时包含任务与 UDP 转发节点，而且旧 C++ 桥接的
`SCALED_IMU` Y/Z 换算存在 `/10000` 路径。项目要求保留此前可运行版本，仅增加
只读定位入口。实测还发现树莓派发往 Windows/WSL 的 UDP 入站被防火墙/NAT
阻断；添加限定 UDP 规则需要管理员权限。

## 决策

1. 不修改或替换旧 `fcu_core`、Domain 20 文件及其启动入口。
2. 新增 `raspberry_pi_readonly_stack`，通过厂商 MAVLink TCP 输入读取数据，唯一
   出站 MAVLink 消息沿用被动 GCS heartbeat；不提供解锁、起飞、航线、速度或高度 API。
3. 三轴厂商加速度统一按 `mm/s² / 1000` 转为 `m/s²`；每条
   `SCALED_IMU` 到达即转发，状态快照频率不再下采样 IMU。
4. 树莓派在 14551 提供 JSONL TCP 遥测流，在 14552 提供带长度前缀的 JPEG+元数据
   相机流；边缘设备主动连接树莓派，无需放宽 Windows 入站防火墙。
5. 飞控 boot time 通过首包到达时刻对齐到 ROS Unix 时间，并保留设备时间间隔；
   UWB 的 MAVLink 载体没有采样时间，使用树莓派接收时间，并再次对齐树莓派与边缘设备时钟。
6. 相机 ROS header 使用边缘接收时间；libcamera `SensorTimestamp`、树莓派接收时间及
   时间域继续发布在 `/nexus/camera/imx219/metadata`，不得伪称为已完成硬件同步。
7. 用户提供的标签 2、四基站坐标和 9° 偏航写入 v01 配置；坐标状态为
   `user_provided_not_surveyed`，不能作为厘米级精度证据。

## 输出接口

- `/nexus/fcu/imu`：`sensor_msgs/Imu`
- `/nexus/uwb/ranges`：JSON `std_msgs/String`，map、m、标签 2、基站 1–4
- `/nexus/pi/telemetry_health`：频率、估算丢样、非单调样本、延迟与时钟重置
- `/nexus/camera/imx219/image_raw`、`image_raw/compressed`、`camera_info`
- `/nexus/camera/imx219/metadata`、`/nexus/pi/camera_health`

## 限制

真实飞控未供电时只能验证合成 IMU/UWB 跨设备链路。相机内参、外参和两机时钟同步
尚未测量；CameraInfo 的零内参明确表示未标定。
