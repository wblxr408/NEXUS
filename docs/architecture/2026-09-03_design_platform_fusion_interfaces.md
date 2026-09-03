# 平台滑窗的状态、时间与观测接口

状态：ADR-011授权下的实现接口。适用无人机 `base_link`，不是机外目标。所有采样时间均须事先映射到同一ROS时钟；节点不猜测飞控启动时间与Unix时间的偏移。

## 数值接口

`BodyState`：`stamp_ns`、`position_m`、`velocity_mps`、`rotation_map_body`、`accel_bias_mps2`、`gyro_bias_rps`。
局部15维增量与协方差顺序为 `[p_map, v_map, theta_body, b_a, b_g]`；旋转按右乘 `R_new = R_ref Exp(theta)` 更新。

IMU必须是 `base_link` 下的比力（含静止重力响应，m/s²）和角速度（rad/s）。`imu_link` 输入必须先使用已标定的 `R_body_imu` 旋转；位置杆臂的旋转加速度效应不被默认视为零，首版要求IMU原点与估计机体参考点重合或上游已补偿。静止且R=I的测试比力为 `[0,0,9.80665]`，重力向量在map中为 `[0,0,-9.80665]`；它不代表厂商实际轴向已验证。

预积分区间必须由真实IMU样本覆盖。可对区间边界进行有括号的插值，不允许外推缺失样本；时间倒退、重复、间隔超过配置阈值、非有限数据均拒绝。偏置一阶修正同时作用于旋转、速度和位置预积分量。

外部观测分为：

- `PositionMeasurement`：map中的机载标签位置，3×3协方差，以及已标定的标签机体杆臂。
- `PoseMeasurement`：map中的机体位姿；数值6×6协方差顺序为 `[p_map, theta_body]`。
- `RangeMeasurement`：已测量map锚点、米制距离、方差和机体标签杆臂，要求完整配置的最少基站数。
- `RelativeMotionMeasurement`：旋转为两采样时刻的 `R_body_i_body_j`；平移观测为传感器原点的位移，表达在body_i中。默认`sensor_body_m=[0,0,0]`时等于机体原点位移；有相机杆臂时预测为`R_i^T(p_j-p_i)+R_i^T R_j lever-lever`。`metric_translation=false`时只约束该向量的方向，不将单目方向当米制基线；其协方差平移块是无量纲方向协方差。不能先猜一个单目长度再减去杆臂。

每条外部观测先以基础协方差做创新检验，再施加有界可靠性放大。IMU/偏置随机游走/边缘化先验不经过视觉离群核。每个被接纳的原始观测仅进入窗口一次。

边缘化只线性化涉及待删除状态的因子及旧先验，通过Schur消元生成新先验；保留的观测不再计入先验，避免重复累积信息。

窗口内的迟到观测可插入其原采样时间，并拆分跨越该时刻的IMU预积分边；早于最旧状态的观测拒绝。节点保留有界IMU缓存，长时间断链后需要新的绝对位姿参考初始化，不能只凭UWB位置猜测姿态。

相对视觉的两个相机时刻都可按上述机制插入，不要求与UWB采样恰好重合。非零视觉杆臂需在平台标定`vision.camera_body_m`中登记；ROS观测的`sensor_body_m`必须与之相符。缺此配置只允许旧的零杆臂观测语义。

状态增量使用局部右乘SO(3)导数。解析因子Jacobian用于求解、创新协方差和边缘化；优化器的旋转向量增量还需右Jacobian链式转换，输出协方差重新在当前状态切空间计算。数值差分保留为独立测试参照。鲁棒IRLS权重在一次最小二乘迭代期间固定，IMU白化矩阵按预积分边缓存。

## ROS2接口

| 方向 | Topic | 类型 | 语义 |
|---|---|---|---|
| 输入 | `/nexus/fcu/imu` | `sensor_msgs/Imu` | 原采样时间、SI单位、显式IMU frame |
| 输入 | `/nexus/fcu/odom` | `nav_msgs/Odometry` | 只取通过frame/3D/covariance检查的平台位置，不使用缺失的厂商姿态 |
| 输入 | `/nexus/platform/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 显式初始平台map位姿及不确定性 |
| 输入 | `/nexus/vision/platform_pose` | 同上 | 固定场地参考得到的map机体位姿，不能用目标Tag位置替代 |
| 输入 | `/nexus/uwb/ranges` | `std_msgs/String` JSON | schema_version=1、sample_timestamp_ns、frame_id=map、unit=m、anchor_ids、ranges_m、variances_m2；锚点坐标和杆臂来自标定文件 |
| 输入 | `/nexus/vision/body_motion` | `std_msgs/String` JSON | schema_version=1、previous_stamp_ns、sample_timestamp_ns、frame_id=base_link、rotation_i_j、translation_i_j、metric_translation、covariance(6×6) |
| 输入 | `/nexus/observations/quality` | `std_msgs/String` JSON | schema_version=1、subject=platform、source、sample_timestamp_ns、features；source为imu/uwb_position/uwb_ranges/fixed_tag/superpoint |
| 输出 | `/nexus/platform/odom` | `nav_msgs/Odometry` | 本机估计的map→base_link状态；与厂商fcu/odom分开 |
| 输出 | `/nexus/platform/status` | `std_msgs/String` JSON | 原采样时间、估计有效性、source状态、拒绝原因、窗口长度、求解耗时、bias、可用性 |

ROS pose协方差为固定map轴的 `[p,angle]`；数值边界会显式转换角度块及交叉块。无新有效约束或IMU断流时只发诊断，不以新时间戳重发旧位置。

固定参考另输出 `/nexus/vision/reference_status`。平台计算完成后再次检查观测年龄；超过预算则输出 `solution_stale`，不会通过提高时间戳伪装新结果。`publish_tf`默认关闭，完整启动中必须明确唯一map→base_link发布者。

## 校准与验证边界

启动节点需要标定配置路径，包含IMU轴向矩阵和版本、噪声密度、初始状态不确定性、标签杆臂；原始测距还需要本场地锚点survey。测试配置必须标明 `synthetic_test`，不得作为实机标定。

验证包括静止重力、恒加速度、恒角速度、偏置修正、异步区间边界、单目尺度不可观测、UWB跳变、零杆臂/非零杆臂、Schur线性等价、窗口不重复计数、超时/失败回滚、ROS消息链。合成验证不证明实机精度或实时预算已达标。
