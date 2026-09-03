# 离线分析

按 `uwb/`、`vision/`、`calibration/`、`fusion/`、`evaluation/`、`localization/` 分类。脚本输入必须来自 `data/metadata/` 登记的数据集，输出写入对应实验运行目录。

`localization/` 是无标记目标定位的稀疏最小二乘捆绑（F1–F8），口径见 [设计文档](../../docs/2026-08-29_design_markerless_target_localization_optimization_v01.md) 与 [ADR-009](../../docs/decisions/2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md)：深度来自多视图交会与支撑面几何，任何网络回归的 z 都不得进入求解器。三条链路的因子集与阈值在仓库根 `config/chain_{a_precision,b_robust,c_adaptive}.yaml`，单独运行入口是 `localization/run_chain_cli.py --chain <名字>`。

`test_samples/` 是例外：只放明确标注为 `test_sample` 的功能验证输入，运行结果应写入临时目录，不能作为实验数据或精度证据。其回放入口为 `replay_cli.py`。

可切换算法的统一注册表位于 `algorithms/router.py`。仿真和离线回放通过算法全名选择实现，算法核心不得读取仿真真值；外部算法适配位置和来源见 `algorithms/third_party/`。

测试入口：`PYTHONPATH=code/analysis python3 -m pytest code/analysis/test -q`（在仓库根执行）。

在线静态目标窗口 `localization/online_solver.py` 已将弱 F8 历史近似替换为平方根边缘化：仅压缩离开窗口的观测与旧历史因子，固定 F3/F8 保留一次，保留目标/相机/段级交叉信息和不可观测零空间。最多求解 N+1 帧后淘汰一帧，失败不提交状态；`carry_prior=False` 为无历史消融，`prior_inflation=1` 为默认不遗忘。接口与数值边界见[目标窗口边缘化说明](../../docs/architecture/2026-09-03_design_target_window_marginalization.md)。它仍是静态目标分析链；新 ROS 米制目标消息的融合、学习质量和展示接入不能据此视为完成。
