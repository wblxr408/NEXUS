# 开发工具

放置 rosbag 转换、环境检查、回放和结果索引工具。工具必须支持 `--help`，并在 `docs/records/` 或运行记录中留下调用命令。

`nexus_channel_contract.py` 和 `channel_contract_cli.py` 只定义并校验传输无关的 ROS1/ROS2 envelope；当前不打开网络、不解析 MAVLink，也不决定最终 bridge 传输方式。

目标观测 envelope payload 的最小字段是 `target_id`、米制 `unit`、位置协方差和 0–1 置信度；序号连续性通过 `validate_sequence` 检查，采样时间、frame、schema 版本和来源模式非法时拒绝。
