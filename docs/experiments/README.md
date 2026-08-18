# 实验设计与可复现运行

实验定义写在 `experiments/designs/`，实际运行写在 `experiments/runs/`。一个运行目录只对应一次明确配置，不覆盖旧结果。

运行目录命名：`YYYY-MM-DD_E<三位编号>_<slug>/`，至少包含：

```text
run.md            运行记录（环境、命令、输入、结论）
config/           参数与标定版本
artifacts/        本次生成物（大文件留外部路径和 sha256）
metrics/          CSV/JSON 指标
```

实验结论必须区分实测、模型预测和外部资料；精度指标至少注明样本数、单位、统计口径和异常值处理。
