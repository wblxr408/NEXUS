# E008 ORB-SLAM2 + UWB + GDR-Net 同步端到端实验

## 目的与数据

本次实验使用一份同步生成的数据：同一组 180 个 RGB 帧同时作为 ORB-SLAM2 的
`rgb.txt` 输入和 GDR-Net 的 BOP `test/000001/rgb` 输入；每帧包含固定沙盘十个目标，
并生成独立带噪 UWB 测距。平台真值、目标 `map → target_link` 真值只存在于
`ground_truth/`，算法运行不读取这些文件。

生成命令：

```bash
python3 simulation/generate_orb_gdrn_dataset.py \
  --out data/processed/nexus_sandbox_orb_gdrn_v01 \
  --frames 180 --post-init-speed 0.08
```

坐标约定：`map → base_link` 表示无人机平台；GDR-Net 输出
`camera_0 → target_link`；最终输出为 `map → target_link`。本次相机到 base 外参为
YAML 中的零平移、单位旋转，后续实体标定后只需替换配置。

## 运行链路

1. Windows Conda 中运行上游兼容构建的 `mono_tum.exe`，输出逐帧
   `FrameTrajectoryTcw_windows.csv`。
2. 使用 `simulation/evaluate_uwb_dataset.py` 对同一数据的 UWB 测距运行多边定位。
3. 使用 `simulation/fuse_orb_slam2_uwb.py` 将 ORB 任意尺度/原点与 UWB 位置做
   Umeyama 恢复，生成算法输出 `orb_uwb_camera_poses.csv`。
4. 使用已有 Windows `gdr_net` checkpoint 对同一 RGB/BOP 帧推理，生成
   `predictions_camera_to_target.json`。
5. 使用 `simulation/fuse_orb_visual_targets.py` 组合两个算法输出，生成
   `predictions_map_to_target.json`；使用 `map_pose_evaluate_cli.py` 评估最终目标位姿。

上述第 2–5 步也可由统一入口重跑：

```bash
python3 simulation/run_orb_gdrn_uwb_pipeline.py \
  --dataset data/processed/nexus_sandbox_orb_gdrn_v01 \
  --orb data/processed/nexus_sandbox_orb_gdrn_v01/orb_output/FrameTrajectoryTcw_windows.csv \
  --uwb data/processed/nexus_sandbox_orb_gdrn_v01/orb_output/uwb_metrics_estimates.csv \
  --camera-predictions experiments/runs/2026-08-27_E008_orb_gdrn_uwb_end_to_end/artifacts/predictions_camera_to_target.json \
  --output-dir experiments/runs/2026-08-27_E008_orb_gdrn_uwb_end_to_end/artifacts/rerun
```

## 实测结果

### 平台自身定位

| 输出 | 有效/应输出 | 3D RMSE | P95 | 最大误差 | 更新率 |
|---|---:|---:|---:|---:|---:|
| ORB 单目原始（任意尺度） | 174/180 | 5.1075 m | 5.5465 m | 5.6218 m | 10 Hz |
| ORB Sim(3) GT 诊断对齐 | 174/180 | 0.02163 m | 0.03518 m | 0.11921 m | 10 Hz |
| ORB + UWB 松耦合恢复 | 174/180 | 0.02614 m | 0.04134 m | 0.10466 m | 10 Hz |
| UWB 单独多边定位 | 180/180 | 0.11421 m | 0.21095 m | 0.54795 m | 10 Hz |

Sim(3) GT 对齐只用于诊断，不能当成固定地图下的绝对 ATE。最终平台结果应看
ORB+UWB 的外部坐标恢复结果；该实现是后处理对齐，不是 ORB-SLAM2 内部紧耦合滤波。

### 目标定位

| 输出 | 有效/应输出 | 平移 RMSE | 平移 P95 | 旋转 RMSE | ADD-S 均值 | ADD-S recall@10% |
|---|---:|---:|---:|---:|---:|---:|
| GDR-Net `camera → target` | 1800/1800 | 1.2961 m | 1.6415 m | 166.97° | 1.1653 m | 0 |
| 组合 `map → target` | 1740/1800 | 1.0041 m | 1.3013 m | 168.67° | 0.8941 m | 0 |

完整指标 JSON 同时包含 MAE、P50、最大误差、x/y/z bias、可用率、失败率、运行时间、
延迟和恢复时间字段。关键补充值如下：GDR-Net 相机平移 MAE/P50/最大值为
`1.2450/1.3256/1.7639 m`，组合地图平移 MAE/P50/最大值为
`0.9781/1.0136/1.4072 m`；两者延迟和恢复时间均为 `null`（离线数据无传输时钟，且
ORB 丢帧后没有独立恢复事件日志）。GDR-Net 平均/P95 推理耗时为 `4.893/5.375 ms`。

GDR-Net 本次使用 BOP `bbox_visib` 真值 ROI，因此可用率是“目标姿态输出可用率”，
不是整图检测召回率。当前 checkpoint 在该同步序列上的精度未达标，不能把结果写成
厘米级性能；十个目标的连续 z 轴对称性已按 ADD-S 评估。

## 结果文件

运行产物位于被 `.gitignore` 忽略的 `data/processed/nexus_sandbox_orb_gdrn_v01/`，
实验记录只登记配置、命令和指标，避免提交图像、模型权重、DLL 或词袋文件。

## 校验与限制

已执行 `python3 -m compileall -q simulation code/analysis/evaluation` 与
`git diff --check`。当前 Linux 容器未安装 `pytest`，单元测试未执行。Windows ORB
二进制可完整运行并输出轨迹；训练/推理耗时和目标指标来自真实程序输出，但目标检测
仍为 GT ROI 条件，且 ORB+UWB 为松耦合 Umeyama 后处理。
