# 输出物

这里只放由实验运行生成的图表、表格、报告和演示导出物。文件名必须包含主题、场景和版本，并在旁边的索引中写明来源运行编号。

建议：`fig_<topic>_<condition>_v01.png`、`tbl_<topic>_v01.csv`、`report_<topic>_v01.pdf`。

2026-09-03 本机软件联调证据：来源为 `synthetic_record_roundtrip` 与 `synthetic_live_display_qa`，仅用于录包、回归和展示验证，不是实机精度实验。外部归档位于 `C:/Users/wblxr/.codex/visualizations/2026/09/03/01a065c8-bb9e-7752-93d1-4ef1f2ba16d1/local_integration_v01/`；其中 `sha256.json` 逐文件登记 SHA256，`validation.json` 登记环境、代码版本和验证结果。rosbag、截图、日志均留在外部目录，不加入 Git。复现步骤与限制见 [本机联调记录](../docs/2026-09-03_record_local_integration_validation.md)。

2026-09-04 RRD 三目标检测微调：原数据由包内 manifest 明确标为 CARLA 仿真。候选权重、转换数据、训练/测试日志与图表位于 `C:/Users/wblxr/.codex/visualizations/2026/09/03/01a065c8-bb9e-7752-93d1-4ef1f2ba16d1/rrd_detector_training_v01/`；实验口径见 [`E023`](../experiments/runs/2026-09-04_E023_rrd_detector_finetune/run.md)。大文件未加入 Git。
