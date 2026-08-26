# E001 静态目标定位运行模板

首次正式 E001 运行时，将本目录复制为 `experiments/runs/YYYY-MM-DD_E001_static_target_localization_baseline/`，不要在模板中填写实测数据或保存原始文件。完整实验协议见 `experiments/designs/E001_static_target_localization_baseline.md`。

开始前必须满足：测量模型已冻结、`map`/外参已独立标定、真值方法及不确定度已登记、UWB 标签对象和视觉输入已确认。缺少任一项时，创建的运行应标记 `blocked`，不能填写精度结果。

目录用途：

```text
run.md                    运行环境、命令、限制和结论边界
config/                   冻结的配置副本、sha256、test_points.csv
artifacts/                小型导出；大文件只登记外部路径和 sha256
metrics/                  每个基线方案的 JSON 与 summary.csv
```

将 `config/test_points.csv` 中的占位行替换为正式测量点。不得把该模板或任何 `test_sample` 当作正式运行。
