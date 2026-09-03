# nexus_vision_localization（ROS2）

职责：保留一个延后的 AprilTag 36h11/PnP 工程基线，接入 `apriltag_ros` 检测结果和同一相机的 `CameraInfo`，输出外部目标 `target_0` 的 `/nexus/vision/target_observation`。当前算法复现主线是无标签纯视觉；无标签适配器必须输出同一 `TargetObservation` 契约。输出必须注明相机/世界 frame、采样时间、目标标识和不确定度，不能与无人机自身位姿混用。

`marker_size_m` 是到货后用量具复测的实体 AprilTag 边长，默认 `0.0` 表示未配置，节点会拒绝输出；不得用上游示例尺寸代替。`target_link` 是刚性安装板外向平面的几何中心，标签中心到该点的外参必须在标定运行中登记。相机输入、`CameraInfo`、frame 和时间戳确认后，使用 `apriltag_localization.launch.py` 同时启动 `apriltag_ros` 与本适配节点；适配节点拒绝相机参数与检测时间戳或 frame 不一致的数据。完整决定见 `ADR-004`。

## 2026-09-03新增主链（ADR-011）

以上保留历史单Tag基线说明；最新设计使用少量固定Tag辅助平台定位，并增加SuperPoint背景运动与指定目标身份跟踪。各入口如下：

| 入口 | 输出与用途 |
|---|---|
| `fixed_reference.launch.py` | surveyed多Tag联合PnP，输出map平台绝对位姿及质量 |
| `static_visual.launch.py` | 类别检测/动态框、SuperPoint背景相对运动、参考目标像素轨迹 |
| `superpoint_motion_node` | 可独立接同帧框或分割掩膜；`enable_target_tracking`控制目标身份分支 |
| `solve_keypoint_pose` / `compose_target_pose` | 目标已知几何PnP与完整不确定性传播，目前是数值接口 |

启动图像链需提供实际本地ONNX资产描述和相机标定文件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nexus_vision_localization static_visual.launch.py \
  superpoint_manifest:=/absolute/path/superpoint.json \
  detector_manifest:=/absolute/path/detector.json \
  calibration_file:=/absolute/path/visual_calibration.yaml
```

路径为接口占位，不是已交付权重。资产描述包含SHA256、明确输出布局和来源/许可；缺模型、模型不符或标定未测量时拒绝启动。合成标定仅在独立测试中显式允许。模型格式、标定字段与检测契约见仓库 `docs/architecture/2026-09-03_design_superpoint_frontend_interfaces.md`。

参考图/首帧ROI通过 `/nexus/vision/target_reference_image` 与 `/nexus/vision/target_requests` 注册，结果见 `/nexus/vision/target_request_status`；具体字段、已有轨迹命名绑定和状态语义见 `docs/architecture/2026-09-03_design_target_identity_interfaces.md`。

`/nexus/vision/target_tracks` 当前是像素身份轨迹，明确标记世界位置和姿态不可观测。不能把框中心、像素速度或单目单位平移方向解释为米制目标坐标。目标三维前端与融合仍在补全；数值接口及协方差约定见 `docs/architecture/2026-09-03_design_target_metric_geometry_interfaces.md`。

验证使用 `colcon build --packages-up-to nexus_vision_localization --symlink-install`，然后 `colcon test --packages-select nexus_vision_localization` 与 `colcon test-result --test-result-base build/nexus_vision_localization --verbose`。DDS链路测试明确替换网络边界为合成特征/检测结果，不证明真实模型效果、实时帧率或实机精度。

## 多帧米制目标入口

后续已增加 `target_metric_node`，`static_visual.launch.py` 默认同时启动它。校准文件还需包含真实 `extrinsic_covariance` 和 `pixel_sigma_px`；字段与算法参数参考 `config/target_geometry_template.yaml`（unmeasured，必须实测填写）。纯像素分支检查可传 `enable_target_geometry:=false`。

平台 `/nexus/platform/odom` 是另一条必需输入。节点按图像时刻查询或有限插值，不用接收时刻的最新平台位置替代。参考请求可指定 `motion_model=static/constant_velocity` 和 `anchor_reference_px`，默认匀速模型；无尺度、视差或运动可观测性不足时保持无效状态。

新输出 `/nexus/vision/target_kinematics` 使用 `TargetKinematicState`：map位置、可观测时的速度、各分量观测标志及9×9协方差。该多帧点分支的位置定义是固定参考特征点，姿态为未知；不能把它当作几何中心或完整6D姿态。旧目标融合与展示尚需接入新消息。完整接口和测试边界见 `docs/architecture/2026-09-03_design_target_multiview_pipeline.md`。

## 2026-09-03 目标融合接入更新

`static_visual.launch.py`默认启动`nexus_fusion_localization/target_kinematic_fusion_node`，将目标米制输出接到`/nexus/target/kinematics`。新增enable_target_fusion、target_reliability_mode、target_reliability_model启动参数。目标几何质量包增加source和position_reference，供新融合节点与原采样时间一起精确配对；特征字段和旧source_mode保留。

该路径保留未知速度/姿态标记和p/v交叉协方差，并按相关估计处理相邻多帧结果。完整接口见`docs/architecture/2026-09-03_design_target_kinematic_fusion_interfaces.md`；新消息到RViz/Web和完整统一bringup仍需继续接入。
