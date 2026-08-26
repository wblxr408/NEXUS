# 可复用定位算法与论文检索

- 日期：2026-08-24
- 类型：research
- 状态：screened（外部资料初筛；尚无本项目实测结论）
- 范围：四基站 UWB 测距/解算定位无人机自身、固定沙盘上的机外目标视觉定位，以及两者的坐标链与融合
- 当前决定：`ADR-007`；UWB 定位无人机自身，机载相机定位无人机外目标

## 结论先行

首版不应为了绕开验证而自研检测器或 SLAM；但四基站 UWB 距离是无人机自身的主测量输入，必须明确其多边测量/厂商解算语义。应先复用以下现有实现并在同一真值、同一时间栅格下比较：

1. **机外目标标签基线：AprilTag 3 / `apriltag_ros` + IPPE_SQUARE PnP。** 这是低风险工程基线，不再代表老师要求的视觉算法优化主线；无标签候选见 `docs/2026-08-24_research_markerless_visual_target_localization.md`。
2. **静态/多标签增强：TagSLAM。** 用因子图联合优化相机轨迹与 AprilTag 位姿；适合对静态标记布局或刚性带标签目标作独立对照。
3. **无人机自身位姿对照：VINS-Mono/VINS-Fusion 单目模式。** 使用机载单目图像与 IMU 的 VIO，与官方 UWB 自身位姿在相同时间栅格比较；它不直接输出机外目标位置。VINS-Fusion 同时支持单目+IMU、双目+IMU 和纯双目，当前只选择单目+IMU配置。
4. **后续紧耦合：UVINS 或同类 UWB-VIO。** 当前项目已将四基站距离作为无人机自身定位的测量来源；接入 UVINS 仍须核对每条距离的时间戳、基站坐标、标签–IMU 外参和报文单位，不能用厂商已解算的单一位姿伪装成原始距离输入。

论文中的数值不能直接当作本项目预期精度：相机、标签大小、距离、基站几何、同步、真值和指标不同。它们只用于筛选算法；精度结论只能来自 `experiments/runs/` 中的正式比较。

各算法需要的相机字段、IMU/UWB输入、时间同步、外参及首次验收步骤见 `docs/2026-08-24_checklist_algorithm_reproduction_hardware_interfaces.md`。

## 候选实现与论文

| 优先级 | 可直接复用的实现 | 本项目用途与输出 | 接入前提 / 当前限制 | 论文与资料 |
|---|---|---|---|---|
| P0 | [AprilTag](https://github.com/AprilRobotics/apriltag) + [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) | 检测 `36h11` 目标标签；通过相机内参和 PnP 得到 `camera → tag_target`，再由 `map → base_link → camera` 得到机外 `target_link`。仓库已登记 `apriltag_ros` 源码。 | 必须先验收机载 `Image`、`CameraInfo`、时间戳与 camera frame；标签边长及 `tag_target → target_link` 外参必须实测。 | [AprilTag 2: Efficient and Robust Fiducial Detection (IROS 2016)](https://doi.org/10.1109/IROS.2016.7759617)；[AprilTag 状态估计分析与改进（开放获取，2019）](https://doi.org/10.3390/s19245480) |
| P0 | [OpenCV `solvePnPGeneric` / `SOLVEPNP_IPPE_SQUARE`](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) | AprilTag 平面正方形标记的双解 PnP；可与现有检测器组合，按重投影误差、正深度和连续性选择有效解。 | 仅对已校准的针孔/对应模型有效；不能把错误相机内参、标签尺寸或时间错配靠算法修正。 | Collins & Bartoli, [Infinitesimal Plane-Based Pose Estimation (IJCV 2014)](https://doi.org/10.1007/s11263-014-0725-5) |
| P1 | [TagSLAM](https://github.com/berndpfrommer/tagslam)（[安装入口](https://github.com/berndpfrommer/tagslam_root)） | 基于 AprilTag 的因子图优化；用于静态标记布局、重复观测和相机/标记位姿的离线或 ROS 对照。 | 需单独评估其 ROS 版本、相机接口和动态目标支持；不能直接替换项目 `target_link` 语义。Apache-2.0。 | [TagSLAM: Robust SLAM with Fiducial Markers (arXiv)](https://arxiv.org/abs/1910.00679) |
| P1 | [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | **单目+IMU模式**的优化式 VIO；作为无人机 `base_link` 自身视觉惯性位姿基线，与官方 UWB 位姿比较或融合。 | 不需要深度相机或双目相机，但需要机载图像、可用 IMU、相机-IMU 外参和时间同步；单目 VIO 的尺度和长期漂移需由 UWB/其他绝对来源约束。上游面向 ROS Kinetic/Melodic，接入 Noetic 前需独立构建验证；GPLv3，不直接并入项目源码。 | Qin et al., [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator (T-RO 2018)](https://doi.org/10.1109/TRO.2018.2853729)；[在线时间标定（IROS 2018）](https://doi.org/10.1109/IROS.2018.8593603) |
| P1 | [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | **单目或单目+IMU模式**的视觉/视觉惯性 SLAM 独立无人机自身位姿对照，可在纹理充足的沙盘环境测试。 | 不需要深度相机或双目相机；单目模式存在尺度不可观/漂移问题，单目+IMU仍需绝对来源或初始化约束。它不是机外目标语义定位器；需特征纹理、相机/IMU 标定，且为 GPLv3。只做隔离评估，不作为首版必选依赖。 | Campos et al., [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multi-Map SLAM (T-RO 2021)](https://doi.org/10.1109/TRO.2021.3075644) |
| P2 | [UVINS：UWB-assisted VIO Correction](https://github.com/SKBT-lab/UVINS-Ultra-Wideband-assisted-VIO-Correction-System) | 在 VINS-Fusion 基础上将多基站 UWB 距离与 VIO 融合，以 UWB 抑制 VIO 漂移。仓库提供 ROS 构建、数据集和配置示例。 | 其输入明确为原始 `/UWB_module/distance`、基站坐标和 tag–IMU 外参；当前 FanciSwarm 是否提供等价原始距离尚未确认。该仓库 README 声明其自身论文尚未发布。 | 可作为方法依据阅读：[Range-Focused Fusion of Camera-IMU-UWB for Accurate and Drift-Reduced Localization (RA-L 2021)](https://doi.org/10.1109/LRA.2021.3057838)；[Low Drift Visual-Inertial Odometry with UWB Aided for Indoor Localization（开放获取，2022）](https://doi.org/10.1049/cmu2.12359) |
| P2 | [GTSAM](https://github.com/borglab/gtsam) | 若 P0/P1 单源比较完成后需要统一优化，可用成熟因子图库表示 UWB range、IMU、相机 PnP 和外参因子。 | 这是框架而不是即插即用定位节点；需要自建因子和实验数据，属于后续自研，不能先于现成基线。BSD-3-Clause。 | Dellaert, [Factor Graphs and GTSAM: A Hands-on Introduction](https://doi.org/10.1561/2200000024) |

## 与当前硬件边界的对应关系

```text
四基站 UWB 距离（无人机自身） ─→ 多边测量/厂商解算 ─→ map → base_link
                                      │
机载相机 + AprilTag/IPPE ─→ camera → tag_target → target_link
                                      │
                         时间对齐、外参变换、可选融合
```

- 上图的 P0 主链路不是“UWB 直接定位机外目标”。UWB 只提供 `map → base_link`；机外目标位置由机载相机的标签观测经坐标链转换得到。
- VINS-Fusion、ORB-SLAM3 和 UWB-VIO 都首先改善或对照 `base_link` 的自身位姿；其中 VINS-Fusion/ORB-SLAM3 均可使用当前单目相机，但需要 IMU（VINS）或接受单目尺度限制。它们有价值，但不能代替目标检测和 `target_link` 输出。
- LS、加权 LS、鲁棒 LS、Chan/Taylor 等 UWB 解算应与四基站距离一起作为无人机自身定位基线；若最终接口只给厂商已解算位姿，则记录为黑盒基线，不能把它伪装成自研原始测距解算。

## 建议的复用与比较顺序

1. **先建立无人机自身 UWB 基线。** 记录四基站坐标、每条距离、时间戳、解算方法和 `map → base_link` 误差；若厂商同时提供位置，单列为黑盒对照。
2. **再建立机外目标视觉基线。** AprilTag 只作为工程对照；无标签主线使用固定沙盘建图/目标 6D 方法，记录检测率、重投影误差、3D RMSE、P95、可用率和端到端延迟。
3. **再比较整链路。** 对同一外部目标真值点、同一时间栅格比较“视觉相对观测 + UWB 多边测量平台位姿”与可选 VIO 平台位姿。不能将无人机自身误差混入机外目标误差而不单列。
4. **最后决定是否紧耦合。** 只有 UWB 距离和视觉失败证据、时间同步及独立真值都齐全时，才引入 UVINS/GTSAM；否则保留可解释的 UWB 平台位姿变换。

## 精度摘录规则

整理论文精度时，每一行必须同时记录：传感器型号、相机分辨率/快门、标签尺寸和距离、基站数量与几何、场地/NLOS、真值设备、静态或动态、指标定义、样本量。缺少这些条件的“厘米级”叙述不得用于算法排序，更不得作为项目实测结果。

## 仍待确认的接口

- 机载摄像头是否可提供原始 `Image`、`CameraInfo`、采样时间戳和光学 frame；
- 飞控/ROS 是否提供可用于 VIO 的 IMU 频率、时间基准与相机–IMU 同步；
- FanciSwarm 是否开放每个基站的原始 TWR/TDOA 距离；
- UWB 标签、IMU、相机的物理安装外参与每次预约时段的基站坐标是否可复测。

在这些事实确认前，所有候选只可用公开数据集或项目测试样例做功能验证，不产生本项目精度结论。
