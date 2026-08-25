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
