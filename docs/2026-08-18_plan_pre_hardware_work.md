# 硬件到货前工作计划（前四阶段）

- 日期：2026-08-18
- 状态：accepted
- 适用范围：硬件到货前的软件、接口和文档准备
- 关联决定：`docs/decisions/2026-08-18_ADR-002_target-localization-scope.md`

## 范围

硬件到货前只完成：

1. 文档与接口契约；
2. ROS2 软件骨架；
3. 离线分析与评估工具；
4. ROS1/ROS2 信息通道契约。

不做硬件验收、真实传感器接入、真实标定、现场实验、精度结论或演示视频录制。单元测试可以使用固定几何样例验证公式，但测试样例不属于实验数据、输出物或答辩证据。

## 阶段一：文档与接口契约

### 已固定边界

- 被定位对象是无人机之外的目标；无人机平台位姿、目标观测和目标世界位姿不得混用。
- ROS1 + 幻思官方 `fcu_core` 只负责硬件兼容；ROS2 负责视觉、目标定位、坐标转换、算法比较和评估。
- `motion_001` 不属于首版目标定位主链路。
- 视觉输入、目标形态、UWB 标签安装方式、map 原点、真实外参和真值方案保持 `TBD`。
- 精度目标只能作为规划目标，不能写成实测结果。

### 官方软件基线登记

当前公开 `fcu_core` 基线：

```text
commit: 14e228c63cfe2bbdecebda8c4f1fc259f295e0aa
environment: Ubuntu 20.04 + ROS Noetic + catkin_make
dependencies: serial, Eigen, quadrotor_msgs
```

官方源码中发现的厘米转米、Z 轴取反和 ROS 接收时间行为只允许存在于 ROS1 适配层；ROS2 算法层必须接收已经标明单位、frame 和时间语义的数据。

### ROS2 接口包

新增 `code/ros2_ws/src/nexus_msgs/`，使用 `ament_cmake`，只存放消息定义。

`TargetObservation.msg`：

```text
uint8 SOURCE_DIRECT_UWB=1
uint8 SOURCE_DIRECT_VISION=2
uint8 SOURCE_PLATFORM_RELATIVE=3
uint8 SOURCE_FUSED=4

std_msgs/Header header
string target_id
uint8 source_mode
geometry_msgs/Pose pose
float64[36] covariance
float32 confidence
```

统一 topic：

```text
/nexus/fcu/odom
/nexus/fcu/imu
/nexus/vision/target_observation
/nexus/target/pose
```

每条目标消息必须包含采样时间、frame、目标 ID、来源模式、协方差、置信度和米制单位。

## 阶段二：ROS2 软件骨架

ROS2 自研节点首版使用 Python、`rclpy`、OpenCV 和 NumPy。每个包必须包含 `package.xml`、`config/`、`launch/`、`src/` 和 `test/`。

### `nexus_vision_localization`

- 订阅 `sensor_msgs/Image` 和 `sensor_msgs/CameraInfo`；
- 提供 AprilTag/ArUco 检测和 PnP 观测接口；
- 输出 `/nexus/vision/target_observation`；
- 明确处理无图像、无目标、缺少时间戳和缺少相机参数；
- 不绑定具体相机、数量、标签尺寸、真实内参或机载相机型号。

### `nexus_coord_transform`

- 维护 `map`、`camera_i`、`base_link`、`target_link` 和 `uwb_anchor_i`；
- 使用 ROS2 `tf2`；
- 实现 NED/ENU、米/厘米、四元数/旋转矩阵和 SVD/Umeyama 变换；
- 缺少真实外参时明确失败，不填入真实坐标；
- 同时保留“目标直接测量”和“平台间接测量”两条适配入口。

### `nexus_fusion_localization`

- 单源观测直接输出；
- 双源观测按协方差或置信度加权；
- 拒绝过期观测和 frame 不一致观测；
- 没有有效观测时输出无效状态；
- 保留 `source_mode` 并输出 `/nexus/target/pose`；
- 不实现 EKF 调参、复杂图优化或 `motion_001` 回传。

### `nexus_bringup` 与 `nexus_viz_dashboard`

- 提供 ROS2 launch、参数加载、topic remap、tf2 配置和 rosbag 录制入口；
- 提供 RViz2 中的平台位姿、目标观测和最终目标位姿显示；
- 显示目标 ID、来源模式和置信度；
- 没有硬件输入时给出明确提示，不创建假传感器数据源，不填精度数字。

ROS2 验收命令：

```bash
colcon list
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## 阶段三：离线分析与评估工具

工具放在 `code/analysis/` 对应子目录，全部提供 `--help`，不读取未登记的本机路径或真值。

### 标定与几何

- SVD/Umeyama 刚体变换；
- NED/ENU 转换；
- 单位转换；
- 外参 YAML/JSON 校验；
- tf2 变换检查；
- UWB 基站 GDOP 计算；
- 标定参数版本检查。

### 评估与回放

统一预留 CSV、JSON、ROS1 rosbag 导出、ROS2 rosbag 导出和飞控 SD 日志转换结果输入，计算：

- RMSE、CEP50、CEP95、最大误差；
- 有效样本数、更新频率和延迟；
- 丢帧数、异常值数量和异常值规则。

输出必须包含单位、样本数、统计口径、坐标系和“测试样例/实测数据”标记。固定几何测试样例不得写入 `experiments/runs/`、`outputs/` 或 `defense/`。

### 视觉与融合分析

- 解析标签检测结果；
- 计算 PnP 和重投影误差；
- 统计目标观测质量；
- 实现时间对齐、过期判断、单源比较、协方差加权和来源模式统计；
- 不进行真实参数寻优。

## 阶段四：ROS1/ROS2 信息通道契约

传输方式暂不定型，只固定传输无关的 envelope 和映射规则。

```text
schema_version
message_type
sequence
sample_timestamp_ns
frame_id
source_mode
payload
```

必须定义消息版本、序号、采样时间、frame、单位、目标 ID、来源、不确定度、过期规则和异常规则。

初始映射：

```text
ROS1 /odom_global_001 → ROS2 /nexus/fcu/odom
ROS1 /imu_global_001  → ROS2 /nexus/fcu/imu
```

具体 bridge 程序、TCP/UDP/标准 `ros1_bridge`、部署主机、网络地址、频率和真实 UWB 目标字段保持 `TBD`。

ROS1 适配层负责读取官方 topic、保留官方消息时间、记录 ROS 接收时间、执行明确的单位/轴向转换并输出契约字段；ROS2 算法层不解析 MAVLink、不解析幻思串口、不猜测 frame/单位，也不把平台位姿直接当作目标位姿。

阶段四只测试 envelope 字段完整性、序号、时间戳、单位、frame、target ID、source mode 和 schema 版本拒绝规则，不测试真实网络、ROS1/ROS2 互通、UWB、视觉或飞控回传。

实现位置：`code/tools/nexus_channel_contract.py`。目标观测 payload 固定要求 `target_id`、米制 `unit`、位置协方差和 0–1 置信度；序号连续性通过 `validate_sequence` 检查。平台状态 envelope 不会因字段相似而被转换为目标位姿。

## 完成标准

- `nexus_msgs` 能编译；
- 所有 ROS2 包能 `colcon build` 和 `colcon test`；
- 目标与无人机有不同消息语义；
- 两种测量模型有独立入口；
- `/nexus/target/pose` 接口固定；
- 坐标、单位和外参没有真实值硬编码；
- 离线工具支持 `--help`；
- 指标定义、ROS1→ROS2 映射和 envelope 契约完成；
- 没有虚构精度数据进入实验、输出或答辩目录。

本计划不包含硬件安装、真实接入、标定、实验、bridge 传输方式定型、EKF 调参、`motion_001` 回传或演示视频。
