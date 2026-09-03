# NEXUS 项目索引

## 任务类别与最小阅读路径

| 任务 | 先读 | 再读 | 产物位置 |
|---|---|---|---|
| ROS/fcu_core 代码 | 本文件、`code/README.md` | `docs/architecture/README.md`、硬件接口基准 | `code/ros1_ws/`、`code/ros2_ws/` |
| 传感器/算法 | 本文件、规划书 | `docs/experiments/README.md`、相关实验记录 | `code/analysis/`、`experiments/` |
| 标定/坐标 | 本文件、规划书任务3 | `docs/architecture/coordinate-frames.md` | `code/ros2_ws/src/nexus_coord_transform/` |
| 图表/统计 | `experiments/README.md` | 数据集元数据与运行记录 | `outputs/figures/`、`outputs/tables/` |
| 答辩/报告 | 本文件、`defense/README.md` | 已验证运行与结论索引 | `defense/` |
| 协作/规则 | `AGENTS.md`、`CLAUDE.md` | `docs/records/README.md` | `docs/records/` |

## 不可混淆的边界

- `docs/decisions/2026-08-24_ADR-007_dual_subject_iterative_localization.md`：当前定位对象与测量模型边界；UWB 定位无人机自身，机载视觉定位无人机之外的目标，两条链路在统一坐标系中迭代。该决定对 ADR-002 的对象范围与待选模型部分具有优先级。
- `docs/decisions/2026-08-25_ADR-008_markerless_visual_first_and_interface_authority.md`：当前算法优先级和接口单一事实源；无标签纯视觉优先，AprilTag 延后。
- `docs/decisions/2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md`：无标记定位的深度观测通道、十目标对称群与 BOP `symmetries_*` 键名、相机高度、UWB 锚点坐标与 UWB 角色、`2 cm` 标签口径的单一裁定。该决定使 `simulation/target_catalog_v01.yaml` v01 的 `symmetry: none` 和 E003/E005 的对称口径失效（历史运行结论不改写，重算落在新运行）；深度不再由任何网络回归；UWB 只做段级基准与兜底，不进逐帧深度回路；`2 cm` 是 TARGET 且仅对仿真成立。
- `docs/decisions/2026-09-01_ADR-010_rrd_sandbox_and_hardware_uwb_range_availability.md`：当前沙盘场景与锚点/UWB 数据来源的边界。沙盘口径由参数化沙盘 / CARLA `sandbox-v29` 改为 CARLA **RRD** 地图（`/Game/CustomMaps/roadrunne3/RRD`，当前 3 个目标、非 BOP 的 jsonl 布局）；`simulation/target_catalog_v01.yaml` 的十个参数化目标、`simulation/sandbox_scene.yaml` 的 `4.0 m × 4.7 m` 外包络与其 `uwb_anchors`、`simulation/carla/target_catalog_v03_carla.yaml` 的 sandbox-v29 十类 prop 降级为**历史资料**（历史运行结论不改写，只声明其场景口径失效）。锚点分三套且**禁止跨环境套用**：参数化沙盘四点（ADR-009，仿真设计值）/ RRD 采样器四点（相对 spawn，仿真设计值）/ 实机四基站（坐标未实测）；ADR-009 第 5 项据此收窄为只对参数化沙盘成立。实机原始 UWB 测距**可用**（厂商复用 `BATTERY_STATUS.voltages[2:6]`，cm），ADR-001 的对应警告已解除——但可用的是「距离」不是「坐标」，锚点 survey 仍是前置条件。厂商位置只有 x/y 且属非标准复用语义，只能作 F7 弱先验，不得当三维平台真值。机载适配器只读、只发 GCS `HEARTBEAT`。「UWB 已校验」目前只是控制台演示，补出运行记录前不得表述为「UWB 已验证」。
- `docs/无人机高精度目标定位项目规划 (2).md`：按幻思资料修订后的规划，不等于完成情况。
- `docs/module-map (1).html`：开源模块选型参考，使用前仍需记录版本、许可证和实际验证结果。
- `docs/figures/`：沙盘参考图片，不是测量真值。
- `data/raw/`：原始采集；`data/processed/`：可追溯处理结果；`outputs/`：面向汇报的导出物。
- `defense/` 只接收有来源运行编号的材料；未验证的图表不得进入最终 PPT/报告。

## 文档阅读顺序

1. 目标、范围和里程碑：规划书。
2. 模块边界和开源依赖：模块选型图。
3. 本仓库实现边界：`docs/architecture/README.md`。
4. 每次实验的输入、命令、结果：`experiments/runs/`。
5. 可提交结论：`docs/records/evidence-index.md` 和 `defense/`。

当前执行基线：`docs/2026-08-25_plan_current_execution_baseline.md`。它整合算法复现顺序、验收矩阵和代码接口；无标签纯视觉优先，AprilTag 仅为延后基线。

实机验证前的仿真入口：`docs/2026-08-25_plan_markerless_visual_simulation_preparation.md`。该文档规定沙盘 3D 建模、UWB/GNSS/单目视觉仿真边界、接口字段填充、实验步骤与实机迁移准入；仿真结果不等同于实测精度。

无标记定位优化主线：`docs/2026-08-29_design_markerless_target_localization_optimization_v01.md`。它规定深度由运动基线交会与支撑面几何求解（不再由网络回归）、统一稀疏最小二乘的八个因子、三条可单独运行并列对比的链路（A 精度 / B 对抗与退化鲁棒 / C 自适应结合）以及 E013–E015、E018–E022 实验序列（原方案的 E013–E020 连号与已占用的 E016/E017 冲突）；口径裁定见 ADR-009。文中 E012 相关数字来源为 `docs/优化方案.md` 的记载，本仓库内无对应运行目录与表格、不可复算，引入答辩材料前须先补回运行记录。

**沙盘口径已于 2026-09-01 变更**：实验对象由参数化沙盘 / CARLA `sandbox-v29` 改为 CARLA **RRD** 地图，可用数据是 500 帧、3 个目标、只有 bbox（无实例掩膜、无深度、无目标网格），因此 GDR-Net 稠密对应无法在这批数据上训练、目标旋转不可观测。受限条件下的因子集与误差预算重算见该设计文档附二，新增的在线增量定位见附三，未实现清单见附四。附二.3 的量化结论：当前 RRD 采集配置（`640×480`、`fov 90°`、约 `14.6 m`）的地面采样距离是 `45.62 mm/px`，是参数化沙盘的 `25.4` 倍，侧向 sigma 约 `64.5 mm`，`2 cm` 与已记录的 `≤5 cm` 都不可达，必须先改采集配置重采数据。该口径变更的决策记录（ADR-010）仍待建。

**在线定位框架（2026-09-02 定稿，见该设计文档附五）**：平台端整体采用现成的 UWB-aided VIO（实机实测飞控推 `SCALED_IMU 188.9 Hz`，但**不推 `ATTITUDE`、不推 `OPTICAL_FLOW`**，且加速度单位是厂商自定的 `mm/s²` 而非 MAVLink 标准 mG，按规范解会差约 9.8 倍）；目标端沿用滑窗捆绑，两端以 F5/F6/F7 因子接口解耦，平台端换实现不影响目标端。§12 误差预算已勘误：漏算的基线不确定性项在逐帧用 UWB 定基线时单独就是 `53 mm`，因此**必须有 VIO/VO 提供段内相对几何**，段级 UWB 尺度才有东西可乘。相机模式裁定为 IMX219 的 `1640×1232 @ 41.85 fps`（全幅 2×2 合并，HFOV `79.3°`）。`/dev/ttyAMA0` 是独占资源，在线链路接 `fanciswarm_bridge.py` 的 UDP 输出，不得再抢串口。

在线框架的独立成文版是 `docs/2026-09-02_design_online_target_localization_framework_v01.md`：它给出坐标系定义与全部转换公式（CARLA 左手系到 `map`、换轴矩阵、完整位姿链、针孔投影、三角化与支撑面求交、UWB 残差与 DOP、段级尺度）、输入如何获取、输出如何得到、八个因子的残差式、所用开源件与许可，以及 2D 水平 `5.68 mm` / 3D `16.30 mm` 的蒙特卡洛实算。两条结论要特别注意：(1) 视线噪声在约 `1 px` 就饱和，早前把亚像素跟踪当关键杠杆的判断已更正；(2) 真正的三个精度前置是支撑面高度、锚点 survey、环绕半径，都不在算法里。

### 历史资料与 2026-08-24 已确认事实（保留入口）

- [算法复现硬件与接口验收清单（2026-08-24）](2026-08-24_checklist_algorithm_reproduction_hardware_interfaces.md)：保留单目相机、树莓派 5 8GB、Mcontroller V7、STM32H743、四基站 UWB、UM982、网络/串口和已知参数；未确认字段单独列出。
- [可复用定位算法、论文与开源库（2026-08-24）](2026-08-24_research_reusable_localization_algorithms.md)：保留论文 DOI、源码仓库、许可证和接入限制。
- [无标签视觉目标定位论文与开源实现（2026-08-24）](2026-08-24_research_markerless_visual_target_localization.md)：保留 FoundationPose、PVNet、GDR-Net、hloc、LightGlue、COLMAP、ORB-SLAM3、DROID-SLAM 等链接和单目边界。
- [项目执行步骤总表（2026-08-19）](2026-08-19_plan_project_execution_steps.md)、[详细执行指导（2026-08-19）](2026-08-19_guide_project_execution_steps_detailed.md)、[步骤文档](project_steps/)：保留原有实施顺序、验收条件和交付物。
- [硬件到货前实施计划（2026-08-18）](2026-08-18_plan_pre_hardware_work.md)、[沙盘世界坐标标定方案（2026-08-19）](2026-08-19_plan_sandbox_world_frame_calibration.md)：保留接口、坐标和标定原方案。
- [双 Ubuntu/ROS 环境配置（2026-08-20）](2026-08-20_guide_dual_ubuntu_ros_environment_setup.md)、[硬件首日验收](2026-08-20_checklist_hardware_first_day_acceptance.md)、[视觉输入准备](2026-08-20_checklist_vision_input_and_marker_readiness.md)：保留安装、验收和输入检查命令。
- [硬件验收与每日记录](2026-08-22_guide_hardware_acceptance_evidence_capture.md) / [日记录目录](records/daily/)：保留过程证据，不因当前执行基线更新而删除。

## 已确定的项目边界

- 无人机既是传感器、通信和计算平台，也是 UWB 定位对象；机载视觉定位无人机之外的目标。两类定位在统一坐标系中迭代，且不得混用 `base_link` 与 `target_link`。
- ROS1 Noetic + 官方 `fcu_core` 是幻思硬件兼容层；ROS2 Humble 是赛题要求的二次自研层。
- 两侧必须建立显式信息通道，但 ROS2 结果回传飞控不是首版必选项。
- 视觉输入、目标世界坐标和真值测量由项目自定义。
- 最终验收以 PPT 和有实验来源的演示视频为主，不要求现场实时飞行。

完整决定见 `docs/decisions/2026-08-24_ADR-007_dual_subject_iterative_localization.md`；ADR-002 未被其覆盖的 ROS 分层、输出语义和证据边界继续有效。

## 当前未决项

已确认的硬件事实包括：Mcontroller V7（STM32H743）、FanciSwarm 四基站 UWB 与机载标签、树莓派 5 8GB、**单目 800 万像素机载相机**、UM982 FGNSS、Ubuntu 20.04/ROS1 Noetic `fcu_core` 和 Ubuntu 22.04/ROS2 Humble。相机实际分辨率、编码、帧率、时间戳、内参和外参等接口字段仍需实机验收，不能把这些未决字段误写成“没有相机”或“硬件未知”。

未决硬件事实直接记录在对应 `experiments/runs/` 的配置和结论中；改变技术边界、字段、topic、frame 或单位必须新增 ADR。历史计划、8 月 24 日研究清单和日记录是事实与依据，必须保留；当前执行基线只是新增入口，不替代或删除历史资料。

## 2026-09-03 双对象实现的最新入口

最新用户方案与历史优先级差异见[ADR-011](decisions/2026-09-03_ADR-011_robust_dual_localization.md)，当前完成与未完成状态以[逐项实现清单](2026-09-03_checklist_dual_localization_implementation.md)为准。平台IMU/UWB/固定参考、SuperPoint静态/目标前端、目标多帧几何与目标窗口边缘化的实现记录均由该清单路由；保留上文历史规划，不将历史目标值表述为实测结果。

目标新消息的融合与预测接口见[ADR-012](decisions/2026-09-03_ADR-012_correlated_target_kinematic_fusion.md)及[接口说明](architecture/2026-09-03_design_target_kinematic_fusion_interfaces.md)。该节点已接入静态视觉启动链，但完整联合后端、RViz/Web、统一平台启动及真实数据验收仍须按清单继续完成。
