# 开发工具

放置 rosbag 转换、环境检查、回放和结果索引工具。工具必须支持 `--help`，并在 `docs/records/` 或运行记录中留下调用命令。

`nexus_channel_contract.py` 和 `channel_contract_cli.py` 只定义并校验传输无关的 ROS1/ROS2 envelope；当前不打开网络、不解析 MAVLink，也不决定最终 bridge 传输方式。

目标观测 envelope payload 的最小字段是 `target_id`、米制 `unit`、位置协方差和 0–1 置信度；序号连续性通过 `validate_sequence` 检查，时效通过调用方冻结阈值后的 `validate_freshness` 检查；采样/接收时间、frame、schema 版本、有效性和来源模式非法时拒绝。

`measurement_export_cli.py` 将 ROS1 bag、ROS2 bag 或飞控 SD CSV 导出为统一 CSV。ROS1 bag 转换必须在有 `rostopic` 的 Noetic 主机运行；ROS2 bag 使用 `rosbag2_py`。输入列映射仍需在硬件首日按厂商实际 SD 表头确认，未知 frame 保持 `TBD`，不能猜测为 `map`。

`hardware_acceptance_capture.py` 用于到货首日的只读证据采集：保存 `rostopic list/type/echo -n 1`、ROS/Git 版本、主机时间同步状态、SD CSV 表头/大小/SHA256，以及明确保留 `TBD` 的 `metadata_draft.json`。它不复制 rosbag、SD 文件正文、设备序列号或网络凭据。完整命令和产物说明见 `docs/2026-08-22_guide_hardware_acceptance_evidence_capture.md`。
