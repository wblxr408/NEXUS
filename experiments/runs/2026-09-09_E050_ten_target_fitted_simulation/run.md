# E050：十目标真值拟合仿真与创新支撑图

日期：2026-09-09。状态：**COMPUTED_FITTED_SIMULATION_NOT_LIVE_VALIDATED**。

## 目的与边界

本运行用 E030 用户接受的实体沙盘真值表，选取 10 个有明确 XY 与高度的交通灯实例，生成带 `target_id` 的多帧模拟识别/定位观测，并用仓库已有 E049 蒙特卡洛误差分布与 E048 三帧近似坐标残差拟合噪声。生成数据用于答辩中的误差分析和创新机制可视化，**不是新的真实识别、不是实机测量，也不是在线系统验收**。

## 输入与方法

- 真值：`experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/physical_sandbox_ground_truth_v01.json`；实例为 J01-NW/NE/SE/SW-HIGH、J02-NW/NE/SE/SW-HIGH、J03-NW/NE-HIGH。
- 误差拟合：E049 `tbl_dual_object_monte_carlo_v01.csv` 的 500 行 `target_bias_corrected_error_xy_m`，按二维各向同性分量拟合噪声尺度；E048 `result.json` 的 3 帧高灯结果拟合 XY 偏差。
- 仿真：固定随机种子 `20260909`；每目标、每方法 60 帧；`baseline` 使用完整拟合残差与偏差，`proposed` 使用声明的 0.65 残差尺度和 0.35 偏差尺度。
- ID 保持曲线和端到端时延取声明的场景概率/场景点，仅用于展示设计权衡，不是从真实日志测得的 IDF1 或 latency。

## 产物

- `tbl_ten_target_simulated_observations_v01.csv`：1200 行（10 目标 × 60 帧 × 2 方法），包含真值坐标、模拟估计、XY 误差、可见状态。
- `fig_ten_target_error_box_v01.png`：误差箱线 + 全部散点。
- `fig_ten_target_trajectory_overlay_v01.png`：十目标真值星标与 proposed 模拟轨迹叠加。
- `fig_id_retention_curve_v01.png`：遮挡帧数—正确 ID 保持概率场景曲线。
- `fig_accuracy_latency_pareto_v01.png`：轻量/平衡/鲁棒三种场景的 P95 精度—时延点。
- `metrics_v01.json`：参数、来源、统计摘要与限制。

## 结果（仿真）

| 方法 | n | 中位 XY 误差 | P95 XY 误差 | RMSE |
|---|---:|---:|---:|---:|
| baseline | 600 | 17.996 cm | 24.846 cm | 18.574 cm |
| proposed（sim.） | 600 | 6.588 cm | 11.304 cm | 7.119 cm |

结果仅表示在当前拟合噪声和声明缩放情景下的模拟差异；不能外推为真实算法增益。

## 复现与验证

```powershell
python -X utf8 experiments/runs/2026-09-09_E050_ten_target_fitted_simulation/run_ten_target_fitted_simulation.py
```

脚本执行成功，生成 1200 行表和 4 张 PNG。图片尺寸检查通过（2240×1536、2304×1504、2624×1536、2240×1920）；四张图均完成视觉抽查，标题、单位、图例和仿真免责声明可读。

## 限制

1. E030 坐标虽为用户接受的沙盘登记真值，但本运行没有重新采集十目标图像。
2. E049 噪声来自参数化蒙特卡洛，E048 偏差仅来自 3 帧；拟合结果对来源运行和假设敏感。
3. `proposed` 的缩放参数、ID 保持概率和 Pareto 时延点是仿真设定，不是实测创新收益。
4. 不能由本运行声称真实识别率、IDF1、三维精度、端到端时延或飞行稳定性。

未扩大任务范围。
