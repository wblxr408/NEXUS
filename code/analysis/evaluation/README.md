# 精度评估

放置 RMSE、CEP50/CEP95、最大误差、频率和延迟统计脚本。

`observation_quality_cli.py` 读取 `measurement_export_cli.py` 生成的统一 CSV，输出可用率、无效原因、来源占比、更新频率、最大间隙、估计丢样、延迟、置信度和重投影误差。该报告描述观测质量，不自动产生精度结论。
