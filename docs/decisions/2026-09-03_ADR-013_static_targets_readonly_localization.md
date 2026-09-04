# ADR-013：静态目标、条件误差与只读定位链路

- 日期：2026-09-03。
- 授权：用户明确目标固定、相关位姿/外参误差可忽略、无人机由原有方式操控，并要求完成六项修正。
- 本决定覆盖 ADR-012 中目标运动预测的要求；原文作为历史保留。

## 模型与边界

1. 项目目标在 map 中静止。注册、身份参考、在线多帧几何仅接受 static；保留图像像素运动跟踪，因为无人机运动仍使投影移动。
2. 平台 p/v/q/IMU bias 估计及其噪声模型不变。速度是估计结果；新定位启动不包含飞行控制节点，也不发布速度、高度或航点指令。旧 Gazebo 巡航只用于独立历史仿真。
3. 目标几何使用实际时刻的平台位姿和实际相机安装变换，但按用户建模假设忽略它们的误差传播。返回的目标协方差是以这些位姿为已知条件的协方差，仍包含像素噪声和鲁棒权重；平台自身协方差不被清零。重叠图像仍有关联，目标融合继续使用 CI。
4. 目标融合不做速度外推，不添加目标速度/加速度过程噪声。目标速度未被测量，保持 NaN 与 velocity_observed=false；不能把静止假设伪装为零速度测量。
5. TargetKinematicState 新增 historical。历史输出 valid=false、historical=true、三个 observed=false，有限坐标/协方差是原估计；last_valid_sample_timestamp_ns 保持原观测时刻，header.stamp 为状态事件时刻。身份不确定时也只能展示为旧参考点的历史记录。
6. 静态地图点在节点运行期间保留，数量受 maximum_tracks 限制，不因短时遮挡或输入缓存超时被删除。长时丢失后重新确认需要连续一致特征与位置；同一物理参考点不能靠连续错误观测跳到另一位置。
7. 新 RViz/Web 消费平台 Odometry 与目标新消息，展示当前、历史、丢失与重新识别。无新输入时不刷新观测时间，不把历史点列为当前测量。
8. 新统一启动只接入采集状态、平台估计、固定参考、视觉、目标融合、展示和显式启用的记录。录包目录不得覆盖已有数据；原始采样时间、配置与消息语义必须可追溯。

## 验证边界

按六项修正逐项记录实际构建、测试及未验证项。真实模型、硬件标定、飞行精度不由合成测试证明。当前实施记录见[静态目标修正清单](../2026-09-03_checklist_static_target_corrections.md)；本决定本身不表示全部六项已完成。

## 新增展示与启动合同

- `TargetKinematicState` 添加字段后需重建所有发布/订阅方；不能混用旧消息生成物。`reassociation_gap_ms` 替代新静态融合入口中的预测时长参数。
- `/nexus/viz/localization_state` 为 `std_msgs/String` JSON，`schema_version=1`，固定 `frame_id=map` / `unit=m`。顶层包含 `session_id`、`generated_timestamp_ns`、`input_mode`、`run_id`、`covariance_assumption`、`platform`、`targets`。纳秒时间使用十进制字符串，避免浏览器整数精度损失。
- platform 包含 display_state、position、orientation、speed_mps、age_s、sample_timestamp_ns；无有效输入时位置/姿态/速度为 null。目标包含 target_id、source、position_reference、state、reason、display_state、sample_timestamp_ns、last_valid_sample_timestamp_ns、age_s、position、historical_position、sigma_m、position_covariance、orientation。当前 position 与 historical_position 互斥，未知值为 null；历史数据不继续刷新 last_valid 时间。
- `/nexus/viz/localization_markers` 为 `visualization_msgs/MarkerArray`，RViz 在 map 显示实际 UAV、当前目标及显式 HISTORY 标记。消息失去刷新后 marker 生命周期到期；不生成没有观测来源的相机射线、目标姿态或沙盘轮廓。
- `dual_localization.launch.py` 为新的只读主入口。录包显式启用并保存配置与运行溯源；Web 只订阅，桥接限制 publish/action/服务入口（安装版 rosbridge 自动保留 rosapi 名称空间，本入口不启动 rosapi）。可更改 Web/桥接端口以避开现有服务。
- 旧 launch、旧 TargetObservation 展示与仿真控制脚本保留为历史独立分支；新 Web 接收双对象快照后，旧目标/平台状态不能覆盖新状态。
