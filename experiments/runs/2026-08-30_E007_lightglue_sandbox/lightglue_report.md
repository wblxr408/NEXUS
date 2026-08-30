# LightGlue 沙盘特征匹配复现报告

- 数据：5 张沙盘 RGB 图，两两配对共 10 对
- 特征提取：SuperPoint（max_keypoints=2048）
- 匹配器：LightGlue（features=superpoint）
- 运行设备：cpu
- 几何验证：RANSAC 基础矩阵（阈值 3.0px，置信度 0.999）

## 位置误差定义（组长给定）

```text
e_i = p_bar_i - p_i^gt
d_i = ||e_i||_2
RMSE = sqrt( (1/N) * sum_i d_i^2 )
```

> 位置误差类指标针对「输出三维位置的算法」（PnP、PVNet、融合）。
> LightGlue 是特征匹配器，只输出点对应关系、不输出位置，故此类指标标记 N/A，由位姿环节填写。

## 核心指标（组长 13 项模板）

| 指标 | LightGlue 数值 | 适用性 |
|---|---|---|
| 3D Position RMSE | N/A | 位置类，属 PnP/PVNet |
| 3D Position MAE | N/A | 位置类，属 PnP/PVNet |
| P50 / Median Error | N/A | 位置类，属 PnP/PVNet |
| P95 Error | N/A | 位置类，属 PnP/PVNet |
| Maximum Error | N/A | 位置类，属 PnP/PVNet |
| Axis Bias | N/A | 位置类，属 PnP/PVNet |
| Availability / Success Rate | 90.0%（9/10 对内点≥8） | ✓ |
| Failure Rate | 10.0%（1/10） | ✓ |
| Update Rate | 0.314 Hz | ✓ |
| Latency（mean / P95） | 3185.6 / 3290.5 ms | ✓（离线=解算耗时） |
| Recovery Time | N/A | 离线两两匹配，无失锁概念 |
| Runtime | 3185.6 ms | ✓ |
| CPU/GPU/Memory | cpu / 峰值 1461 MB | ✓ |

## 特征匹配专属诊断指标（文档第四部分）

| 指标 | 数值 |
|---|---|
| Inlier Count（均值） | 55.0 |
| Matching Inlier Ratio（均值） | 0.574 |

## 两两匹配明细

| 图i | 图j | 匹配数 | 内点数 | 内点比 | 耗时(ms) | 成功 |
|---|---|---|---|---|---|---|
| 0 | 1 | 7 | 0 | 0.000 | 3219.9 | ✗ |
| 0 | 2 | 17 | 14 | 0.824 | 3267.2 | ✓ |
| 0 | 3 | 205 | 122 | 0.595 | 3169.8 | ✓ |
| 0 | 4 | 46 | 28 | 0.609 | 3210.6 | ✓ |
| 1 | 2 | 39 | 20 | 0.513 | 3055.8 | ✓ |
| 1 | 3 | 251 | 185 | 0.737 | 3309.5 | ✓ |
| 1 | 4 | 45 | 20 | 0.444 | 3075.4 | ✓ |
| 2 | 3 | 38 | 27 | 0.711 | 3181.8 | ✓ |
| 2 | 4 | 150 | 110 | 0.733 | 3177.9 | ✓ |
| 3 | 4 | 42 | 24 | 0.571 | 3188.4 | ✓ |
