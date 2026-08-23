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

- `docs/decisions/2026-08-18_ADR-002_target-localization-scope.md`：当前赛题对象与验收边界；被定位对象是无人机之外的目标，优先级高于旧规划中把目标位姿等同于无人机位姿或默认回传飞控的表述。
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

硬件到货前实施计划：`docs/2026-08-18_plan_pre_hardware_work.md`。该计划只覆盖文档/接口、ROS2 骨架、离线工具和传输契约，不产生硬件或精度结论。

完整执行顺序：`docs/2026-08-19_plan_project_execution_steps.md`。该文档把硬件到货前准备、硬件验收、单传感器基线、标定、E001/E002/E003 实验、融合、演示和交付串成一条主线；其中规划目标仍不等于实测结果。

执行步骤详细指导：`docs/2026-08-19_guide_project_execution_steps_detailed.md`。该文档按每个步骤说明前置输入、推进顺序、模块图中的可复用候选、必须自研的边界、产出和停止条件，不涉及具体代码实现。

独立步骤文档目录：`docs/project_steps/`。步骤 01～12 分别成文，可按人员分工或阶段评审单独阅读。

实体沙盘坐标标定：`docs/2026-08-19_plan_sandbox_world_frame_calibration.md`。该方案把沙盘物理基准点建立为 `map`，再分别求相机、UWB 和目标外参，并使用独立验证点检查结果。

## 已确定的项目边界

- 无人机是传感器、通信和计算平台，被定位对象是无人机之外的目标。
- ROS1 Noetic + 官方 `fcu_core` 是幻思硬件兼容层；ROS2 Humble 是赛题要求的二次自研层。
- 两侧必须建立显式信息通道，但 ROS2 结果回传飞控不是首版必选项。
- 视觉输入、目标世界坐标和真值测量由项目自定义。
- 最终验收以 PPT 和有实验来源的演示视频为主，不要求现场实时飞行。

完整决定见 `docs/decisions/2026-08-18_ADR-002_target-localization-scope.md`。

## 当前未决项

目标的物理形态与运动状态、UWB 标签安装对象、相机视角与数量、视觉输入接口、时间同步方式、跨 ROS 信息通道、真值测量方法和最终融合实现仍需通过决策记录确认；Mcontroller V7、FanciSwarm UWB 四基站、UM982、ROS Noetic/fcu_core 与 ROS2 Humble 的分层角色已确定。在确认前使用 `TBD`，不得在代码中写死。

上述未决项对应的具体提问、提问对象和阻塞关系见 `docs/2026-08-23_checklist_open_questions_advisor_and_vendor.md`。该清单只登记问题，不预设答案；答复后按需新增决策记录。
