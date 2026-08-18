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

## 当前未决项

相机数量、视觉输入接口、时间同步方式、bridge 部署方式和最终融合实现仍需通过决策记录确认；Mcontroller V7、FanciSwarm UWB 四基站、UM982、ROS Noetic/fcu_core 与 ROS2 Humble 的分层角色已确定。在确认前使用 `TBD`，不得在代码中写死。
