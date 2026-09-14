# E051：创新中间结果拟合仿真

日期：2026-09-09。状态：COMPUTED_INTERMEDIATE_FITTED_SIMULATION_NOT_LIVE_VALIDATED。

## 目的

生成支撑五项创新的中间结果图：视觉质量—误差校准、鲁棒残差权重、观测几何质量场、异步链路延迟预算和协方差覆盖校准。

## 数据与边界

- 真值与目标布局：E030 `physical_sandbox_ground_truth_v01.json`。
- 误差形状：E050 十目标拟合仿真（其来源为 E049 500 次蒙特卡洛与 E048 三帧近似坐标残差）。
- qv、几何信息代理、阶段时延和覆盖率均为仿真/场景量；不能表述为实测质量标定、GDOP、端到端 latency 或真实协方差校准。

## 产物

- `fig_quality_error_calibration_v01.png`：质量分数与中位/P90误差单调关系。
- `fig_robust_weight_response_v01.png`：Huber/IRLS 残差权重响应。
- `fig_observation_geometry_field_v01.png`：锚点几何代理场与 48 个接受真值点。
- `fig_async_latency_waterfall_v01.png`：采集→检测→特征→几何→融合→显示的场景延迟预算。
- `fig_covariance_calibration_v01.png`：1σ/2σ 经验覆盖率与二维高斯参考。

## 复现与验证

```powershell
python -X utf8 experiments/runs/2026-09-09_E051_innovation_intermediate_simulation/run_intermediate_figures.py
```

脚本执行成功，生成 5 张高分辨率 PNG；图片尺寸检查通过，视觉抽查通过，中文字体警告已消失，`git diff --check` 通过。

未修改算法、真值表或历史运行；未扩大任务范围。
