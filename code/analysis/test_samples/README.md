# 硬件到货前测试样例

本目录只存放 `data_type: test_sample` 的确定性功能样例，用于验证消息语义、坐标 frame、单位、时间配对与融合降级规则。它们不是传感器仿真、真实采集或真值数据，不能放入 `experiments/runs/`、`outputs/` 或答辩材料。

运行：

```bash
python3 code/analysis/replay_cli.py \
  --input code/analysis/test_samples/pre_hardware_replay_cases.json \
  --output /tmp/nexus_pre_hardware_replay_report.json
```

当前覆盖：目标观测经 `nexus_coord_transform` 转至 `map` 后的单源输出、单源与固定协方差融合、视觉丢失时的 UWB 单源输出、过期拒绝、frame 不一致拒绝、单位错误拒绝、异步配对拒绝及两源均无效。报告还调用评估指标模块生成**合成**位置/延迟统计，只说明功能检查是否通过，明确不包含精度结论。
