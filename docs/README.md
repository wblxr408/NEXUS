本题目面向无人机系统定位算法，采用**UWB和视觉方式**，对目标进行**高精度定位**。主要内容包括：

（1）安装 Ubuntu 22.04 + ROS2 Humble 作为项目自研主线；同时准备 Ubuntu 20.04 + ROS1 Noetic 和幻思官方 `fcu_core` 兼容环境；

（2）对已有定位算法进行精度比较，确定精度为厘米级的定位算法；

（3）对获得的目标位置坐标进行转换；

（4）完成演示应用。

## 已确认的赛题对象

- 被定位对象是**无人机之外的目标和无人机本身**；无人机作为传感器、通信和计算平台。

- ROS1 与 ROS2 之间建立显式信息通道；除非演示需要目标跟踪控制，否则 ROS2 结果不必回传飞控。
- 已确认硬件：Mcontroller V7（STM32H743）、FanciSwarm 四基站 UWB + 机载标签、树莓派 5 8GB、**单目 800 万像素机载相机**、UM982 FGNSS；ROS 侧为 Ubuntu 20.04 + ROS1 Noetic/`fcu_core` 和 Ubuntu 22.04 + ROS2 Humble。相机实际分辨率、编码、帧率、采样时间戳、内参/外参和标签安装关系仍需验收。
- 最终验收以 PPT 和有实验来源的演示视频为主，不要求现场实时飞行。

范围决定见 [ADR-007](decisions/2026-08-24_ADR-007_dual_subject_iterative_localization.md)；当前算法顺序和接口权威见 [ADR-008](decisions/2026-08-25_ADR-008_markerless_visual_first_and_interface_authority.md)。无标签纯视觉是复现主线，AprilTag 延后。

## 仓库导航

- [项目索引](PROJECT_INDEX.md)：按任务选择最小阅读路径，区分规划、实现和证据。
- [实施规划书](<无人机高精度目标定位项目规划 (2).md>)：已按幻思硬件资料修订的方案与四周计划。
- [当前执行基线](2026-08-25_plan_current_execution_baseline.md)：整合无标签纯视觉优先的复现顺序、验收矩阵和最新代码接口。
- [无标签纯视觉仿真准备、方法与实施步骤](2026-08-25_plan_markerless_visual_simulation_preparation.md)：实机验证前的沙盘 3D 建模、UWB/GNSS/单目视觉仿真、字段映射、实验矩阵和迁移准入。
- [参数化沙盘模型](../simulation/README.md)：基于 `4.0 m × 4.7 m` 外包络和 `2.0 m × 2.3 m` 中央工业区参数生成 Gazebo SDF、OBJ 与 Web 场景 JSON。
- [完整执行步骤](2026-08-19_plan_project_execution_steps.md)：从范围定义、软件骨架、硬件接入、标定、实验到答辩交付的实际顺序。
- [执行步骤详细指导](2026-08-19_guide_project_execution_steps_detailed.md)：每阶段准备、决策、复用模块、自研边界、产出和验收条件。
- [Ubuntu 与 ROS 双环境配置教程](2026-08-20_guide_dual_ubuntu_ros_environment_setup.md)：Ubuntu 22.04 + ROS2 Humble 与 Ubuntu 20.04 + ROS1 Noetic 的安装和验证。
- [12 份独立步骤指导](project_steps/)：每个项目步骤单独成文，便于分工和逐项验收。
- [实体沙盘世界坐标系标定方案](2026-08-19_plan_sandbox_world_frame_calibration.md)：`map`、基准点、相机/UWB 外参和独立验证方法。
- [模块选型图](<module-map (1).html>)：开源模块和“需要自己写”的边界，使用前需自行验证版本与许可证。
- [系统架构](architecture/README.md)：ROS1 硬件层、ROS1-ROS2 bridge、ROS2 算法层、坐标系和接口草案。
- [幻思硬件接口基准](architecture/hardware-integration.md)：官网核对后的硬件、通信和验收边界。
- [2026-08-24 硬件与接口验收清单](2026-08-24_checklist_algorithm_reproduction_hardware_interfaces.md)：完整保留已知参数、单目相机边界和待验收字段。
- [2026-08-24 算法/论文/开源库检索](2026-08-24_research_reusable_localization_algorithms.md)：论文 DOI、源码链接、许可证和复现条件。
- [2026-08-24 无标签视觉检索](2026-08-24_research_markerless_visual_target_localization.md)：单目无标签路线、论文和开源实现。
- [历史执行计划与步骤](2026-08-19_plan_project_execution_steps.md)：8 月 19–24 日形成的计划、清单和记录均为保留资料。
- [待确认问题清单：赛事方与硬件厂商](2026-08-23_checklist_open_questions_advisor_and_vendor.md)：保留问题、答复和阻塞关系。
- [项目记录](records/README.md)：日记录、会议、问题和证据索引。
- [答辩材料规则](../defense/README.md)：PPT、报告、演示和 Q&A 的归档位置。

设备型号有FanciSwarm UWB基站(选配，用于高精度UWB定位)

| col1       | col2                      |
| ---------- | ------------------------- |
| 飞控名称   | Mcontroller V7跨模态飞控  |
| 飞控处理器 | STM32H743                 |
| 轴距       | 约113mm                   |
| 伴随计算机 | 树莓派5 8G（轻量计算）/外部主机（视觉与分析） |
| 摄像头像素 | 800W像素                  |
| 光流定位   | 支持                      |
| UWB定位    | FanciSwarm UWB基站        |
| GNSS       | UM982高精度GNSS模组       |
| 尺寸       | 约80x80x180mm             |


代码、数据、实验和输出物的目录约定见仓库根目录 [README.md](../README.md)。
