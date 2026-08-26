# ADR-006：目标观测契约 V2 与实时坐标链

- 日期：2026-08-21
- 状态：accepted
- 负责人：项目组
- 关联任务/运行：硬件到货前软件闭环；仅为功能验证，不是精度实验
- 取代范围：细化并取代 ADR-005 第 5、6 条的消息字段和同步表达；ADR-005 的目标/平台、单位、坐标和拒绝边界继续有效

## 背景

ADR-005 已要求采样时间、接收时间和有效性，但原 `TargetObservation` 只有 `Header`，实时视觉观测也会直接进入只接受 `map` 的融合节点。代码无法表达失败原因、最后有效时刻或当前层接收时间，传感器 frame 到 `map` 的职责不清晰。

硬件尚未到货，不能在修复软件链时同时假定沙盘原点、轴向或相机/UWB 外参。

## 决定

1. `TargetObservation.header.stamp` 表示采样时间；通道 envelope 的同义字段为 `sample_timestamp_ns`。两者均不得用接收时间替代。
2. ROS 消息和 envelope 都包含 `receive_timestamp_ns`，且不得早于采样时间。
3. ROS 消息增加 `validity`、`invalid_reason`、`last_valid_sample_timestamp_ns` 和 `unit`。有效消息原因必须为空；无效消息必须给出原因，以 NaN 作为坐标/协方差占位。最后有效时刻只用于诊断，禁止复用旧坐标。
4. 实时 topic 分成传感器 frame 与 `map` 两层：
   - `/nexus/vision/target_observation → /nexus/vision/map_target_observation`；
   - `/nexus/uwb/raw_target_observation → /nexus/uwb/target_observation`。
   融合只消费两个 `map` 观测。
5. 坐标节点在消息采样时刻查询 tf2，并旋转位置协方差。找不到已验证变换时发布 `transform_unavailable`，不发布猜测坐标。
6. 双源融合要求目标、frame、单位、有效性、时效均一致，且采样时间差不超过 `max_pair_delta_ms`。超限时降级为最新单源并保留其来源模式，不标记为融合。
7. envelope 的时效由 `validate_freshness` 根据调用方冻结的 `now_timestamp_ns` 和 `max_age_ns` 显式检查；通道格式本身不写死实验阈值。
8. 到货前外参配置必须为空模板。沙盘 `map` 物理原点、`+x/+y` 基准和相机/UWB 外参只在硬件到货后的标定运行中填写，并关联运行编号。

## 验证

- 消息和 envelope 单测检查接收时间、有效性与原因规则；
- ROS2 launch 测试启动正式 `target_localization.launch.py`，用测试专用 frame/TF 验证 AprilTag/PnP、`camera → map`、融合与 `/nexus/target/pose`；
- 同一测试验证缺少 TF 的无效状态和双源超时降级；
- 测试专用 TF 不代表实体沙盘，不进入外参模板或实验目录。

## 影响

所有后续 ROS1/ROS2 适配器必须填充 V2 字段。旧包或旧录包缺少字段时必须通过显式转换器补充来源和接收时间，不能静默假定为有效数据。
