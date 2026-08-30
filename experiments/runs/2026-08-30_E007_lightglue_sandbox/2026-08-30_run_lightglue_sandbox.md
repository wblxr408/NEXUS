# 实验运行：2026-08-30_E007_lightglue_sandbox

- 目的：在沙盘 RGB 图上复现 LightGlue（SuperPoint 特征提取 + LightGlue 匹配），按组长统一指标模板（docs/算法比较定义参数.md）报告匹配质量与耗时。
- 设计文件：`docs/算法比较定义参数.md`（组长给定的 13 项指标模板 + 位置误差定义）
- 代码提交：`code/analysis/algorithms/vision/lightglue/`（本次提交）
- 环境/设备：WSL2 Ubuntu-22.04，CPU（无 GPU 图形加速），torch 2.13.0+cpu；venv 位于仓库外 `/home/e0h/nexus_workspace/lightglue_venv`
- 输入数据与 sha256：`data/processed/nexus_sandbox_gdr_net_v01/train_pbr/000001/rgb/` 5 张沙盘 RGB 图（000000~000004.png）
- 配置目录：`simulation/gdr_net_dataset.yaml`（width 960 / height 720 / hfov 79.3°）、`simulation/sandbox_scene.yaml`（sandbox_4000x4700_v03）
- 执行命令：
  ```bash
  python code/analysis/algorithms/vision/lightglue/report_lightglue.py \
    --rgb_dir data/processed/nexus_sandbox_gdr_net_v01/train_pbr/000001/rgb \
    --out_dir experiments/runs/2026-08-30_E007_lightglue_sandbox
  ```

## 指标定义

- 样本数：5 张图两两配对 = 10 对。
- Inlier Count：匹配对中通过 RANSAC 基础矩阵（`cv2.findFundamentalMat`，阈值 3.0px、置信度 0.999）验证的内点数量。
- Matching Inlier Ratio：内点数 ÷ 匹配数。
- Availability / Success Rate：内点数 ≥ 8（够 RANSAC/最小 PnP 求解）的图对占比。
- Failure Rate = 1 − Availability。
- Update Rate = 1000 ÷ Runtime(ms)（离线两两匹配，Latency ≈ Runtime）。
- 单位：Inlier Count 为点对数（无量纲）、Ratio 为比例、Runtime/Latency 为 ms、Memory 为 MB。

## 结果

| 指标 | 数值 |
|---|---|
| Inlier Count（均值） | 55.0 |
| Matching Inlier Ratio（均值） | 0.574 |
| Runtime（均值 / P95） | 3185.6 / 3290.5 ms |
| Update Rate | 0.314 Hz |
| Availability / Success Rate | 90.0%（9/10） |
| Failure Rate | 10.0%（1/10） |
| Peak Memory | 1461 MB（CPU） |

位置误差类 6 项（3D Position RMSE/MAE、P50/P95、Maximum Error、Axis Bias）：**N/A**，LightGlue 不输出三维位置，归 PnP/PVNet 环节。

## 证据文件

```text
lightglue_report.md     89a3247ea2085b944225ddfb6f05b9a226bef4263fc765ba45b6dd730e770aae
matching_metrics.csv    ee68dfb97f717feba64aa73a1eb808d39c89b61959262fb2c42e629b3931db63
match_000001_000003.png 1ef0458b343ab79d92c2433d3ab42615a713228ae15d36524cf8a314952cebfe
demo_match_result.png   ea78409f0430099ef221f95274c5af58cec7daea1e0abd3d82f07077c2ab481c
```

输入沙盘图 sha256：

```text
000000.png 1f093cdf3608071a9809b5ed11d0b60c2cf5ce4af159785295336b98a3fca796
000001.png e015eee6a21dcd0d88c2734a5001d746dd665e3123aad8b515b27caba625256c
000002.png 697a48f767a6b6be150b5c9d1a11243a2abc471af3000bc3698a547afaeb0735
000003.png 38eed0e4eec4037bfa0caf2545740a375110a2ea14adff8a08f98a253fb4df4e
000004.png 89fbda00d83996336ccf6a71cfb9c08dd03d0a2b4418fd5e771385cf48b36569
```

## 结论与限制

- 实测结论：LightGlue 在沙盘图上能稳定匹配，平均内点比 0.574、平均内点数 55；10 对中 9 对匹配成功（内点 ≥ 8）。唯一失败对（图 0↔1，仅 7 匹配 0 内点）对应视角差异大/纹理少的相邻帧，属正常。
- 位置误差类指标为 N/A（非本环节产出），非"未测好"；由 PnP/PVNet 位姿环节输出后填写。
- Runtime 为 CPU 软跑实测（约 3.2s/次），GPU 环境会显著下降。
- 沙盘图为 OpenCV 几何渲染、无实测纹理/光照/传感器噪声，结果用于验证数据链路与指标口径，不代表真实机载图像精度。
