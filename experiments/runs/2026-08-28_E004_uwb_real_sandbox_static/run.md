# 实验运行：2026-08-28_E004_uwb_real_sandbox_static

- 状态：blocked（实体数据、现场坐标和真值尚未登记）
- 目的：使用同一批实体沙盘 UWB 原始测距数据，统一评估五个 Python 二维定位算法。
- 测量模型：direct_target（目标直接携带 UWB 标签；安装方式尚待现场确认）
- 范围：`map` 坐标系下的二维水平 `x/y`；不由二维结果补写 `z=0`，不作三维精度结论。
- 数据集元数据：`data/metadata/2026-08-28_uwb_sandbox_static_v01.yaml`
- 设计/协议：实体沙盘静态 UWB 算法实验计划；本目录只记录本次运行，不修改历史规划结论。
- 代码提交：TBD（运行时填写）
- Python/依赖版本：TBD（运行时填写）
- 原始数据外部位置、大小、SHA256：TBD
- 归一化数据 SHA256：TBD
- 基站坐标来源：`config/anchors.yaml`；当前为现场实测占位符，不能使用仿真坐标。
- 目标标签安装、参考点和偏移：TBD
- 真值方法与不确定度：`config/test_points.csv`；当前为占位符。

## 准入检查

- [ ] 四个基站已固定，`map` 原点、轴向、单位和右手系已记录。
- [ ] 四个基站的实测 `x/y/z` 已写入 `config/anchors.yaml`，并有坐标版本和 SHA256。
- [ ] 标签确实安装在被定位目标上；标签中心与目标参考点偏移已测量。
- [ ] 八个真值点（核心 3、过渡 3、UWB 覆盖 2）已填写，真值独立于算法标定。
- [ ] 每点三轮、每轮 10 s 稳定期和 60 s 评估期均已采集，且未覆盖旧轮次。
- [ ] 原始采样时间戳、接收时间戳、标签 ID 和基站 ID 可追溯。
- [ ] 距离单位已由设备接口说明和抽样人工测量确认，统一为米。
- [ ] EKF/UKF 初值与过程/测量噪声已在采集前冻结，且每点每轮前重置状态。

## 被测算法

所有算法接收完全相同、按时间顺序排列的 `UwbObservationFrame`，不接收真值，也不读取其他算法结果。内部路由名保留代码来源标识，报告中使用简洁名称：

| 报告名称 | 内部 ID |
|---|---|
| Trilateration | `uwb.matlab.trilateration` |
| Multilateration | `uwb.matlab.multilateration` |
| Taylor | `uwb.matlab.taylor` |
| EKF | `uwb.matlab.ekf` |
| UKF | `uwb.matlab.ukf` |

## 采集和评估口径

- 目标放置并复核后，稳定采集 10 s；随后连续记录 60 s 评估数据。
- 公共评估时间栅格为 10 Hz（100 ms）；只匹配同一算法最近且时间差不超过 50 ms 的有效输出。
- 无匹配记为不可用，不以前一帧或其他算法结果填补；可用率分母保留全部栅格。
- 使用二维 `x/y` 欧氏误差计算 RMSE、MAE、P50、P95、最大误差和轴向偏差，并报告有效率、失败原因、更新频率、运行时间和接收延迟。
- P50/P95 是二维水平欧氏误差分位数，不是严格统计意义上的 CEP。
- `error > 0.30 m` 的样本计为异常但保留在主统计中；非法或缺失输入只计为不可用。

## 计划执行命令

数据和配置完成后执行；当前不得把该命令的预期输出写成实测结果：

```bash
PYTHONPATH=code/analysis:simulation \
python3 code/analysis/algorithms/uwb/run_real_data.py \
  --input data/processed/2026-08-28_uwb_sandbox_static_v01/observations.csv \
  --truth data/processed/2026-08-28_uwb_sandbox_static_v01/truth_points.csv \
  --config experiments/runs/2026-08-28_E004_uwb_real_sandbox_static/config/algorithm_config.yaml \
  --output experiments/runs/2026-08-28_E004_uwb_real_sandbox_static/metrics/
```

## 结果与证据

当前没有归一化观测、真值或算法指标；`metrics/summary.csv` 仅保留输出字段表头。没有完整真值、坐标系、单位和时间戳时，本目录只能支持功能/数据质量检查，不能进入正式精度统计。

运行完成后，结果必须关联：

- `run_id: 2026-08-28_E004_uwb_real_sandbox_static`
- `data_type: measured`
- `is_measured_result: true`
- 实际代码提交、配置 SHA256、原始和归一化数据 SHA256

## 结论边界和未验证项

在状态改为 `completed` 前，不得填写精度结论，也不得声称达到任何厘米级目标。当前未验证：实体标签语义、基站坐标、真值不确定度、时钟同步、原始距离单位、五算法完整离线运行、指标配对和现场 LOS/NLOS 条件。
