# ADR-008：无标签纯视觉优先与接口单一事实源

- 日期：2026-08-25
- 状态：accepted
- 负责人：项目组
- 取代：ADR-004 的“AprilTag 首版主链路”排序；不取代 AprilTag 代码、依赖或可解释基线
- 关联：ADR-006、ADR-007、`docs/无人机高精度目标定位项目规划 (2).md`、`docs/module-map (1).html`

## 决定

1. 算法复现顺序改为：**无标签纯视觉目标定位 → 坐标链验证 → 可选平台位姿融合 → 有时间再做 AprilTag**。无标签路线先在公开数据或项目录制帧上完成可重复复现，再接入机载相机和 `map` 坐标；不能用 AprilTag 结果替代无标签路线的验证。
2. 无标签路线的候选按可观测性选择：有目标 CAD/几何和 RGB（或深度）时优先复现 FoundationPose、PVNet/GDR-Net 等现成方法；目标固定且有纹理但没有 CAD 时，先做固定沙盘建图、局部特征匹配和 PnP；没有几何、深度或稳定纹理时，记录为可观测性阻塞，不通过更换网络假装解决。
3. AprilTag 36h11、`apriltag_ros` 和 IPPE/PnP 保留为低风险工程基线和后续扩展。只有无标签路线被接口/数据条件阻塞，或主线完成后仍有时间，才启动该基线；其 `target_id`、标签尺寸和外参仍以 ADR-004 的实测规则为准。
4. 规划书和模块图仍是项目范围、硬件分层和候选模块的事实来源，但其中“AprilTag/ArUco 视觉主线”只保留为候选说明，执行排序以本 ADR 为准。规划中的 5 cm、30 cm 等是目标值，不是实测结果。
5. 数据接口以代码为单一事实源：
   - ROS2 消息字段和常量以 `code/ros2_ws/src/nexus_msgs/msg/TargetObservation.msg` 为准；
   - 传输 envelope、字段校验、时间新鲜度和 ROS1→ROS2 话题映射以 `code/tools/nexus_channel_contract.py` 为准；
   - 话题、参数默认值和 frame 以各 ROS2 包的 `config/*.yaml`、launch 和节点代码为准；
   - 本仓库接口文档只解释语义，不另行创造字段、单位或默认值。
6. `TargetObservation` 永远表示机外目标，不能承载无人机自身平台位姿。当前已实现的 AprilTag 节点仍可发布 `/nexus/vision/target_observation`，无标签适配器必须复用同一契约；当前没有无标签实现或精度结论。

## 当前主链路

```text
机载图像 ──→ 无标签纯视觉目标观测（当前待复现） ──→ camera → map 坐标变换
飞控/UWB 平台位姿 ────────────────────────────────→ base_link → camera 外参
                                                └─→ /nexus/target/pose
```

UWB 只解释为携带标签的无人机自身位置；不得把 `/odom_global_001`、`/nexus/fcu/odom` 或 UWB 原始距离自动标成机外目标位置。融合必须在同一目标、frame、单位、有效性和时间栅格上进行。

## 与现有规划的冲突处理

| 来源 | 仍然有效 | 冲突项 | 当前处理 |
|---|---|---|---|
| `docs/无人机高精度目标定位项目规划 (2).md` | 双 ROS 分层、FanciSwarm/UWB、机载视觉、目标值与验收边界 | AprilTag/ArUco 被描述为视觉主线 | 本 ADR 只调整执行优先级，不改历史规划文本 |
| `docs/module-map (1).html` | 开源模块候选、ROS 分层、硬件接口假设 | AprilTag/ArUco 视觉层仍写成主线 | 作为候选模块图阅读，实际启用顺序以本 ADR 为准 |
| ADR-004 | AprilTag 36h11 参数、标签外参与验收规则 | AprilTag 是首版主链路 | 排序被本 ADR supersede；参数规则仍可用于后续基线 |
| ADR-006 与代码 | `TargetObservation` V2、时间/有效性/坐标链 | 旧文档中的字段或示例可能不同 | 以代码和本 ADR 的接口表为准 |

## 复现验收顺序

1. 固定无标签算法、代码提交、权重、环境、相机模型和输入样例，完成离线可重复运行。
2. 用机载相机原始 `Image`、`CameraInfo`、采样时间和 `camera_i` frame 做回放；缺少任一项时只记功能阻塞。
3. 完成 `base_link → camera_i` 与 `camera_i → map` 标定和独立验证点，再将相对目标结果变为 `map → target_link`。
4. 在同一运行中分别记录目标视觉误差、平台 UWB 误差、时间配对误差和端到端延迟；没有 `experiments/runs/` 运行编号不得写精度结论。
5. 主线完成或被证据阻塞后，按需启用 AprilTag 基线；再决定是否做融合或 UVINS/GTSAM 等后续方案。

## 影响

- 旧的重复计划、日记录、检索清单和步骤文档不再作为执行入口；整合内容见 `docs/2026-08-25_plan_current_execution_baseline.md`。
- AprilTag 源码、submodule、ROS2 launch 和测试保留，避免破坏已有功能链；它们的文档改称“延后基线”。
- 任何未来改变算法优先级、消息字段、topic、frame 或单位的修改，都必须新增 ADR 并同步代码测试。
