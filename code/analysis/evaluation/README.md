# 精度评估

放置 RMSE、CEP50/CEP95、最大误差、频率和延迟统计脚本。

`observation_quality_cli.py` 读取 `measurement_export_cli.py` 生成的统一 CSV，输出可用率、无效原因、来源占比、更新频率、最大间隙、估计丢样、延迟、置信度和重投影误差。该报告描述观测质量，不自动产生精度结论。

`pose_evaluate_cli.py` 评估 GDR-Net/BOP 风格 6D 预测：

```bash
PYTHONPATH=code/analysis python3 code/analysis/evaluation/pose_evaluate_cli.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v01 \
  --predictions <predictions.json> --output <metrics.json>
```

预测平移使用米，BOP 真值平移自动从毫米转换。评估报告包含 3D 平移统计、旋转
geodesic 误差、ADD/ADD-S、投影误差和可用率；缺失预测计为失败，不做轨迹对齐或真值
回填。当前十个目标都是参数化圆柱，ADD-S 是主表面误差。

`map_pose_evaluate_cli.py` 对完整 `map -> target_link` 预测做同样的平移、旋转
和 ADD-S 评估；它只由评估器读取 `ground_truth/target_pose_map.csv`：

```bash
PYTHONPATH=code/analysis python3 code/analysis/evaluation/map_pose_evaluate_cli.py \
  --dataset data/processed/nexus_sandbox_gdr_net_v03 \
  --predictions <map_predictions.json> --output <metrics.json>
```

`simulation/fuse_uwb_visual_targets.py` 只使用 UWB 算法输出、显式姿态观测、
标定的 `base_link -> camera_0` 外参与视觉预测生成该输入；不得用真值文件完成
坐标变换。
