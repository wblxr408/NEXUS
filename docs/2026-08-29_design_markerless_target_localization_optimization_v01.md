# NEXUS 无标记目标定位优化方案（三链路 / 低算力）v01

- 日期：2026-08-29
- 类型：design
- 状态：draft
- 关联：`docs/2026-08-24_research_markerless_visual_target_localization.md`（screened 检索）、
  `experiments/runs/2026-08-29_E012_target_catalog_clean_retrain/`、ADR-005/006/007/008、
  [ADR-009](decisions/2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md)
- 标签口径：本文档所有数字按
  `defense/slides/2026-08-19_plan_defense_slide_content.md:22-30` 的
  `REQUIREMENT / IMPLEMENTED / MEASURED / TARGET / REFERENCE / TBD` 标注。
  `MEASURED` 必须关联 `experiments/runs/` 运行编号。
- 精度目标：`2 cm` 是 **TARGET**，不是已记录需求。已记录需求为
  `核心演示区 ≤5cm（视觉可见时）/ 全场景兜底 <30cm`，分档
  `视觉主导 ≤5cm / 过渡 8~15cm / UWB 兜底 15~25cm`，任务书写「厘米级」。
- 边界：本方案不修改论文原文、不修改 GDR-Net / ORB-SLAM2 上游 submodule、
  不新增需要训练的大网络。

> **修订提示（2026-09-01，先读这条再读正文）**
>
> 下面 0–16 节成文时的实验对象是**参数化沙盘**（`4.0 m × 4.7 m`、十个参数化代理目标、
> `3280×2464`、相机高 `3.05~3.65 m`）。**该前提已作废**：沙盘已换成 CARLA 的 **RRD**
> 地图，可用数据是 500 帧、**3 个目标**、**只有 bbox**（无掩膜/无深度/无目标网格）。
> 方法本身不变（深度仍由几何求解），但**因子集、误差预算和可报指标都要按本文档
> 「附二：沙盘变更为 CARLA RRD 后的受限条件优化」重算**；
> 本次新增的**在线增量定位**见「附三：在线增量定位（机载 C6）」；
> **尚未实现的部分逐项列在「附四：未实现清单」**。
> **在线算法框架的定稿设计（含实机硬件实测、相机模式裁定、修正后的误差预算）见
> 「附五：在线视觉-惯性-测距融合定位框架」，它取代附三的分层方案。**
> 口径变更由 ADR-010 裁定（该 ADR 本身仍待建，见附四 U7）。
>
> 附二.3 的量化结论要特别注意：RRD 现状配置的地面采样距离是 `45.62 mm/px`，
> 是参数化沙盘的 `25.4` 倍，侧向 sigma 约 `64.5 mm`
> ⇒ **`2 cm` 与已记录的 `≤5 cm` 在当前采集配置下都不可达，必须先改采集配置重采数据。**

## 0 一句话方案

**把深度误差从"网络回归量"搬到"几何求解量"上**：弃用视像大小反解深度，
改由运动基线交会与支撑面几何给出深度，用一个 ~300 参数的稀疏最小二乘把
多视图视线、区域对应线、支撑面、控制点、ORB-SLAM 相对位姿和段级 UWB 一起解出来；
在此之上并列交付三条可独立运行的配置：A 精度、B 对抗与退化鲁棒、C 二者自适应结合。
全案净新增 0 个需要训练的大网络，推理主链路目标是在树莓派 5 的 CPU 上跑通。

## 1 问题与现状诊断

### 1.1 现状（MEASURED，E012）

`experiments/runs/2026-08-29_E012_target_catalog_clean_retrain/`，测试集为独立
`test_perimeter_arc` 轨迹 360 实例，按 `frame_id + obj_id` 与 BOP `camera → target`
真值逐样本比较，不做坐标对齐、不滤波、不用真值 bbox 顶替：

| 指标 | GT ROI 条件 | 外部 YOLO ROI 条件 |
|---|---:|---:|
| 平移 RMSE | `0.102543 m` | `0.112422 m` |
| 平移 MAE | `0.083676 m` | `0.089018 m` |
| 平移 P50 | `0.072536 m` | `0.077519 m` |
| 平移 P95 | `0.192314 m` | `0.203335 m` |
| 旋转 RMSE | `34.7546 deg` | `35.9240 deg` |
| 旋转平均 | `28.5237 deg` | `29.2559 deg` |
| 旋转 P50 | `22.7394 deg` | `23.2435 deg` |
| 旋转 P95 | `65.5498 deg` | `67.6973 deg` |
| ADD@10% 直径召回 | `3.89%` | `2.22%` |
| 平移 ≤2 cm 比例 | `13.89%` | `13.33%` |
| 旋转 ≤2 deg 比例 | `0%` | `0%` |
| 平均推理时间 | `12.29 ms` | `7.48 ms` |

**数据可追溯性说明（搬运时核对，不可省略）**：本仓库当前工作树中 `experiments/runs/` 下
**没有** `2026-08-29_E012_target_catalog_clean_retrain` 运行目录（现有运行为 E001–E011、
`2026-08-30_E007_lightglue_sandbox`、`2026-08-31_E016_carla_sandbox_v29_full_gdrn`、
`2026-08-31_E017_carla_sandbox_v29_camera_localization`），
`outputs/tables/tbl_e012_pose_errors_v01.csv` 与 `outputs/tables/tbl_e012_pose_by_target_v01.csv`
也**不存在**（`outputs/tables/` 目前仅有 `.gitkeep`）。因此 1.1、1.2、10.1、10.2 的全部数字
**来源是 `docs/优化方案.md` 的记载，在本仓库内不可复算**。按 `AGENTS.md:7`
「规划中的"目标值"不是实测结果。只有 `experiments/runs/` 中有数据、配置、代码版本和结论的运行，
才可进入答辩材料」，这些数字虽沿用原文的 `MEASURED` 标注，但在引入答辩材料之前
**必须先补回可追溯的运行目录与表格**；在补回之前，对外一律按 `TBD` 处理，
不得作为已实测结果引用。

检测器已饱和：precision `0.997230`、recall `1.000000`、F1 `0.998613`、
mAP50 `1.000000`，TP=360 / FP=1 / FN=0。两个条件只差约 1 cm
⇒ **瓶颈不在检测，在位姿级**。

### 1.2 误差方向分解（MEASURED，从 `outputs/tables/tbl_e012_pose_errors_v01.csv` 720 行重算）

| 量 | 值 |
|---|---:|
| 侧向（垂直光轴）RMS | `24.3 mm` |
| 深度（沿光轴）RMS | `99.6 mm` |
| 深度占平移平方误差比例 | `94.4%` |
| 深度/侧向 比 | `4.11` |
| 深度均值偏差 | `−71.3 mm`（十目标**全为负**，一律估得偏近） |
| 逐目标深度偏差范围 | `−17.3 mm`（obj5）～ `−120.6 mm`（obj4） |
| 仅扣逐目标均值偏差后 | 深度 RMS `99.6 → 63.3 mm`；总平移 `102.5 → 67.8 mm` |
| `corr(dz, Z)` | `−0.29`（与距离弱相关） |

结论：主误差是**沿光轴的系统性偏差**，不是随机噪声，也不是检测抖动。

### 1.3 两层原因

**(a) 本地训练配置欠配**（`code/analysis/vision/train_gdr_net_synthetic.py`，IMPLEMENTED 事实）：

```text
cfg.MODEL.CDPN.ROT_HEAD.NUM_CLASSES = 1      # 十个尺寸各异的形状共用一个头
cfg.MODEL.CDPN.PNP_NET.PM_LW      = 0.0      # GDR-Net 自带解耦点匹配损失被关闭
cfg.MODEL.CDPN.PNP_NET.REGION_LW  = 0.0
Z_TYPE      = "REL"                          # 相对 z
Z_LOSS_TYPE = "L1"                           # L1 的最优解是条件中位数
```

共享头 + 相对 z + L1 ⇒ 回归到"形状边缘分布的中位深度"，恰好解释 1.2 里
"十目标深度偏差全为负"。单位约定已核对无 bug：
`translation_ratio[2] = z / (64/scale)` 与上游
`pose_from_pred_centroid_z.py:84-86` 一致。

**(b) 观测通道本身选错了。** 视像大小反解深度：

```text
Z = fx · S / s              # S 物理尺寸，s 像素尺寸
dZ = Z² / (fx · S) · ds
```

测试集真实相机距离均值 `3.550 m`（min `2.936`、max `4.077`，因为
`camera_height_m: [3.05, 3.65]`）⇒ 灵敏度 **`33.5 mm/px`**；要拿到 1 cm 需要
`0.30 px` 的轮廓尺度精度，不可达。而且该通道对**目标模型尺寸误差 1:1 敏感**，
而 `simulation/target_catalog_v01.yaml` 是
`source: designed_parameterized_proxy_without_cad`、`dimension_confidence: C`
——这是不可约的计量依赖。

### 1.4 替代通道量化（TARGET；闭式 + 用 36 个真实测试相机位姿蒙特卡洛）

| 深度通道 | 1 px 视线噪声下深度 σ | 对模型尺寸误差敏感 | 算力 |
|---|---:|---|---|
| 视像大小（E012 现状） | `33.5 mm/px` | 是（1:1） | 网络前传 |
| 运动基线三角化 N=3 | `6.85 mm` | 否 | 可忽略 |
| 运动基线三角化 N=8 | `2.49 mm` | 否 | 可忽略 |
| 运动基线三角化 N=36 | `1.10 mm` | 否 | 可忽略 |
| 支撑面射线求交（单视图） | `2.59 mm` | 否 | 可忽略 |

视线方向仍有 **~27× 余量**（现在等效用掉 `13.5 px`，物理地板约 `0.5 px`）。
**这就是"高精度"与"低算力"不矛盾的根本原因**：几何求解几乎不花算力。
符合 ADR-008 `不通过更换网络假装解决`、ADR-003（学习件只能输出质量权重／
残差修正／有效性／不确定度，不得直接输出目标坐标）。

## 2 逐篇取长补短（不以任何单篇为蓝本）

算力档位定义：`CPU-实时` / `CPU-可跑` / `小GPU-离线` / `大GPU-必需`。
许可一列为 2026-08-29 直接拉取仓库 LICENSE 核对的结果（**IMPLEMENTED 核对动作**，
版本会变，接入前需复核）。

| 方法 | 我们取什么 | 我们不取什么 | 档位 | 许可 |
|---|---|---|---|---|
| **SRT3D**（IJCV 2022）/ **ICG**（CVPR 2022）/ **M3T**，`DLR-RM/3DObjectTracking` | 稀疏区域法「对应线」：**只需物体网格**、专治**无纹理**物体；ICG 论文报 `1.3 ms/帧 单 CPU 核`（REFERENCE）。仓库 readme 明确针对 `partial occlusions, appearance changes, motion blur, background clutter, object ambiguity, and real-time requirements`——正好是链路 B 的退化清单。我们的十个目标就是无纹理参数化体 ⇒ **低算力精化的核心构件** | 它是 tracker，需要初值，不当全局初始化用 | CPU-实时 | **MIT**（© 2023 Manuel Stoiber, DLR）✅ |
| **XFeat**（CVPR 2024），`verlab/accelerated_features` | CPU 实时稀疏特征，仅依赖 torch，无需 GPU（readme: `Real-time sparse inference on CPU for VGA images (tested on laptop with an i5 CPU and vanilla pytorch)`）；替掉 SuperPoint/SuperGlue/LoFTR 那一档，用于场景锚定与视线 | 不上 GPU 级匹配器 | CPU-实时 | **Apache-2.0** ✅ |
| **EPro-PnP**（CVPR 2022 Oral / Best Student Paper），`tjiiv-cprg/EPro-PnP` | 概率 PnP 层：输出 SE(3) 上的**位姿分布**⇒ 链路 B 需要的协方差几乎零额外算力就有了，且天然表达多峰（对称性歧义） | 不做端到端重训整条骨干 | CPU-可跑 | **Apache-2.0**（© Alibaba Group）✅ |
| **GDR-Net**（CVPR 2021，已有 submodule） | 几何引导稠密 2D–3D 对应；**打开它自带的 `PM_LW` / `REGION_LW` / `PM_LOSS_SYM`** | **不再用它回归的 z**；不再共享单头 | 小GPU-离线（仅训练） | 见 submodule，接入前复核 |
| **PVNet**（CVPR 2019） | 像素投票给出关键点不确定度分布，可直接作 PnP 权重 | 全分辨率投票的开销 | CPU-可跑 | TBD |
| **CosyPose**（ECCV 2020） | **多视图对象级联合优化的问题表述**（object-level scene refinement）——这正是链路 A 的骨架 | render-compare 迭代精化的算力 | 大GPU-必需（原版） | TBD |
| **FoundationPose**（CVPR 2024） | 只作 REFERENCE 上界，不接入 | 需 RGB-D + 大模型；LICENSE 为 NVIDIA 自有条款（非 Apache/MIT）；研究文档已写 `树莓派级端侧不能假定实时` | 大GPU-必需 | NVIDIA 专有条款 ❌ |
| **hloc + COLMAP** / **ACE**（CVPR 2023） | 场景锚定定位思想 + 控制点固定尺度与规范 | 沙盘近平面是经典 SfM 退化情形；ACE 仍需 GPU 训练 | 小GPU-离线 | TBD |
| **ORB-SLAM2/3**（已有 submodule） | 帧间相对位姿先验（F5/F7）；`umeyama` 恢复米制尺度 | 不把它的绝对轨迹当真值 | CPU-实时 | GPLv3，注意分发边界 ⚠ |
| **QuadricSLAM**（RA-L 2019）/ **CubeSLAM**（T-RO 2019） | **仅用检测框做对象级 bearing-only 三角化**的最小观测模型 ⇒ 近零算力兜底通道 | 二次曲面参数化精度有限，不作主通道 | CPU-实时 | TBD |
| **DROID-SLAM**（NeurIPS 2021） | 无 | 算力不合适 | 大GPU-必需 | — |
| **LoFTR / LightGlue / SuperPoint+SuperGlue** | 作为精度上界对照；`LighterGlue`（XFeat 仓库自带）比原版 LightGlue 约快 3× 可作备选 | 主链路不用 GPU 级匹配器 | 小GPU-离线 | LightGlue Apache-2.0（复核） |
| **对抗**：`arXiv 2203.00302` Adversarial samples for deep monocular 6D pose；`arXiv 2512.19058` Backdoor Attacks in 6DoF Pose；`arXiv 2108.07229` How Sensitive are Patch Attacks to 3D Pose | 威胁模型：推移分割注意力即可破坏位姿，**且能绕过 RANSAC** ⇒ 链路 B 的压力测试设计依据 | 不发布攻击工具本身 | CPU-可跑 | 论文，REFERENCE |
| **Kendall & Gal** aleatoric 不确定度 | `s = ln σ²` 的 2 标量输出头，用于视图选择与外点剔除 | 不用它替代融合（ADR-003） | 近零 | 论文，REFERENCE |

### 2.1 合成结论（这才是「方案」，而不是「选型」）

取 **CosyPose 的多视图对象级联合优化表述** ⊕ **GDR-Net 的稠密对应与对称感知损失**
⊕ **EPro-PnP 的位姿分布/协方差** ⊕ **SRT3D/ICG 的无纹理区域对应线**
⊕ **XFeat 的 CPU 级场景锚定** ⊕ **QuadricSLAM 的框级 bearing-only 兜底**，
统一进**一个稀疏最小二乘捆绑**；深度不再由任何网络回归，
改由运动基线与支撑面几何给出。三条链路共用这一个求解器，只是因子集与门控策略不同。

### 2.2 工程注意（诚实记录，不许省略）

- XFeat readme 写 `does not need a GPU for real-time sparse inference ... unless you
  run it on high-res images`。本项目图像是 `3280×2464`，**属于高分辨率**，
  必须先下采样或分块，否则 CPU 实时前提不成立。这一点要在 E018 里实测，不许照抄论文。
- XFeat 仓库把 `TensorRT / ONNX` 导出列为 **TODO**，即官方尚未提供导出脚本。
  本方案的 ONNX-int8 路径要自己写，属新增工作量。
- `DLR-RM/3DObjectTracking` 是 C++ 工程，接入需要绑定层；若不接入其代码，
  则只用其方法论（沿投影轮廓法向采样对应线）自研实现，MIT 许可不构成障碍但仍需署名。

## 3 算力预算与「低配置」的定义

### 3.1 现有硬件（IMPLEMENTED 事实，来自 `docs/` 环境记录）

| 位置 | 硬件 | 现有角色 |
|---|---|---|
| 机载 | 树莓派 5 8 GB，Raspberry Pi OS，**无 ROS、无 GPU、无 NPU** | 采集 / 推流 / 通信 |
| 地面 | Windows 笔记本 + RTX 4070 Laptop 8 GiB，PyTorch 2.6.0+cu124，Python 3.10.19 | 训练与离线评估 |
| 分析 | WSL Ubuntu，`scipy 1.8.0`、`numpy`、`opencv` 已在位 | 数据集生成、指标、绘图 |

仓库中**没有任何** ONNX / TensorRT / NCNN / OpenVINO / 量化工作——这是缺口，不是约束。

### 3.2 「低配置」的可验收定义（TARGET）

| 编号 | 条目 | 目标 |
|---|---|---|
| C1 | 主链路推理不需要 GPU | 树莓派 5 CPU 4 线程可跑通 |
| C2 | 不新增需要训练的大网络 | 净新增可训练网络 = **0** |
| C3 | 新增求解器只用已装依赖 | `scipy.optimize.least_squares`，无 Ceres / gtsam / pytorch3d |
| C4 | 单目标单次捆绑求解参数量 | `≈300`（十目标 × 6 DoF + 少量标定/尺度项） |
| C5 | 地面离线批处理时长 | 360 实例全解 `<60 s`（笔记本 CPU） |
| C6 | 机载增量模式（可选） | `≤10 Hz` 更新，`≤1` CPU 核 |

`不要求现场实时飞行` 已在基线文档记录 ⇒ **允许滑窗/批处理**。这是「高精度 + 低算力」
能同时成立的第二个原因（第一个是 1.4 的几何求解几乎不花算力）。

### 3.3 为什么砍掉的东西正是最贵的东西

| 砍掉 | 原本算力 | 替代 | 替代算力 |
|---|---|---|---|
| render-and-compare 迭代精化（CosyPose/FoundationPose） | 大GPU，每轮一次渲染 | 稀疏区域对应线（SRT3D 思路） | 单 CPU 核毫秒级 |
| 网络回归深度 | 每帧一次骨干前传 | 运动基线交会 + 支撑面求交 | 闭式，微秒级 |
| GPU 级稠密匹配器（LoFTR） | 小GPU | XFeat（下采样后） | CPU |
| EKF/UKF/因子图库 | 新依赖 | 信息矩阵加权 + `least_squares` | 已装 |

## 4 坐标系、真值点、估计坐标与转换关系

### 4.1 坐标系清单

| 帧 | 定义 | 谁实现它 | 状态 |
|---|---|---|---|
| `map` | 沙盘固定世界系，原点在沙盘一角，`+Z` 向上，单位 m | **UWB 锚点坐标**（不是 UWB 测距本身） | 锚点未实测 ⇒ TBD |
| `base_link` | 无人机机体 | FCU / UWB 解算 | IMPLEMENTED（仿真） |
| `nexus_uav_camera_optical_frame` | 相机光学系，`+Z` 沿光轴向前，`+X` 右，`+Y` 下 | `base_to_camera` 外参 | **理想化单位阵**，未标定 |
| `target_link`（×10） | 每个目标的几何参考中心，`origin_definition: target_link at each object's geometric reference centre` | `simulation/target_catalog_v01.yaml` | 参数化 proxy，`dimension_confidence: C` |

### 4.2 先把 UWB 的作用说清楚（回答此前的疑问）

UWB **只测距**，这是对的。但一组距离要变成坐标，必须知道**锚点在哪**；
而"锚点在哪"这句话是写在某个坐标系里的，**那个坐标系就是 `map`**。所以：

```text
UWB 测距噪声          → 随机误差，多帧可平均（仿真 range_noise_std_m: 0.015）
锚点坐标测量误差       → 系统误差，不随帧数减小，直接平移/旋转整个 map
```

`code/analysis/uwb/multilateration.py:10` `solve_multilateration(anchors_m, ranges_m)`
的第一个入参就是锚点坐标；它是**基准量（gauge）**，不是观测量。本方案因此把
UWB 放在**段级尺度/基准因子（F6）**和**兜底通道**两个位置，不放进目标深度的逐帧回路。

**必须记录的现状缺口**：仓库里存在三组互不一致、且都未实测的锚点坐标，
且 `simulation/sandbox_scene.yaml:138` 的 `uwb_anchors: []` 是空的。
本运行使用 `simulation/gdr_net_dataset_v03.yaml:36-44` 的一组：

| 锚点 | 坐标 (m) |
|---|---|
| `anchor_0` | `[0.15, 0.15, 0.40]` |
| `anchor_1` | `[3.85, 0.15, 2.40]` |
| `anchor_2` | `[3.85, 4.55, 4.40]` |
| `anchor_3` | `[0.15, 4.55, 1.40]` |

`range_noise_std_m: 0.015`，`range_bias_m: 0.0`，`tag_to_base_translation_m: [0,0,0]`。
锚点坐标口径统一必须走决策记录，不得在代码里各自取值。

### 4.3 十个真值点（GROUND TRUTH，`map` 帧，单位 mm，源 `simulation/target_catalog_v01.yaml`）

| obj | class_name | 源编号 | 形状 | 位置 `x,y,z` | 尺寸 `l,w,h` | yaw° | 真实对称群（本方案主张） | 目录当前写法 |
|---:|---|---|---|---|---|---:|---|---|
| 1 | `vertical_striped_tank` | NW-T01 | cylinder | `1320, 3080, 115` | `190,190,130` | 10 | 绕 z 连续 | `none` ❌ |
| 2 | `horizontal_capsule_tank` | NW-T02 | horizontal_cylinder | `1610, 3100, 115` | `260,140,140` | 35 | 绕物体 x 连续 + z 180° | `none` ❌ |
| 3 | `conical_roof_silo` | NW-T03 | silo_roof | `1760, 2780, 105` | `145,145,150` | 0 | 绕 z 连续 | `none` ❌ |
| 4 | `triangular_hopper` | NW-T04 | triangular_prism | `1570, 2730, 95` | `180,135,110` | −20 | 无 | `none` ✅ |
| 5 | `hexagonal_column` | NE-T01 | hex_prism | `2300, 3080, 120` | `170,170,150` | 15 | z 六重离散 | `none` ❌ |
| 6 | `stacked_box_tower` | NE-T02 | stacked_box | `2650, 3080, 120` | `190,150,170` | −25 | z 二重离散 | `none` ❌ |
| 7 | `l_profile_factory` | NE-T03 | l_prism | `2770, 2770, 100` | `200,170,120` | 10 | 无 | `none` ✅ |
| 8 | `cross_valve_block` | NE-T04 | cross_prism | `2560, 2740, 100` | `200,200,110` | 45 | z 四重离散 | `none` ❌ |
| 9 | `tapered_frustum` | SW-T01 | frustum | `1280, 1910, 110` | `180,180,145` | 0 | 绕 z 连续 | `none` ❌ |
| 10 | `twin_cylinder_unit` | SW-T02 | twin_cylinder | `1580, 1910, 105` | `230,150,130` | −15 | z 二重离散 | `none` ❌ |

**十项中八项对称群写错。** 后果：obj1/3/9 的 yaw 在几何上**不可观测**
（且 `generate_target_detection_dataset.py:_target_mesh()` 从不读 `decoration`，
所以条纹、端带、顶环没渲染，连纹理线索也没有），它们在 E012 里的
`22.29 / 22.88 / 45.67 deg` 旋转 RMSE 是**把噪声当误差在计分**。
这必须由 ADR-009 定口径，不得静默改历史结论（`AGENTS.md:12`）。

另一处：九个目标底面沉入台面 5–20 mm（`base_z` 对 `deck_height: 50`：
obj1 50.0、obj2 45.0、obj3 30.0、obj4 40.0、obj5 45.0、obj6 35.0、obj7 40.0、
obj8 45.0、obj9 37.5、obj10 40.0）⇒ 支撑面因子不能假设 `z_base = deck`，
每目标 `base_z` 要么进状态，要么从目录固定。

### 4.4 转换关系（IMPLEMENTED，`simulation/fuse_uwb_visual_targets.py:57-62`）

```python
R_base_target = R_base_camera @ R_camera_target
t_base_target = R_base_camera @ (t_camera_target - t_camera_base)
R = R_map_base @ R_base_target
t = base_position_map + R_map_base @ t_base_target
```

写成链式：

```text
map --[UWB 多边定位 + FCU 姿态]--> base_link
    --[base_to_camera 外参, 现为理想单位阵]--> camera_optical
    --[视觉位姿估计, 本方案要改的就是这一段]--> target_link
```

误差是**串联**的：目标在 `map` 中的误差 = 平台位姿误差 ⊕ 外参误差 ⊕ 相机→目标误差。
所以"UWB 把无人机定准了"不等于"沙盘目标定准了"，两者不可互相替代。

尺度与基准：`code/ros2_ws/src/nexus_coord_transform/.../geometry.py:100`
`umeyama_alignment` 与 `simulation/fuse_orb_slam2_uwb.py:18` `umeyama`
用于把单目/SLAM 轨迹对到 `map` 的米制。**评估时不得自由 SE(3) 对齐后再报指标**
（`docs/算法比较定义参数.md:128`）。

### 4.5 估计坐标的输出契约

`code/ros2_ws/src/nexus_msgs/msg/TargetObservation.msg`（IMPLEMENTED）已有：

```text
SOURCE_DIRECT_UWB=1  SOURCE_DIRECT_VISION=2  SOURCE_PLATFORM_RELATIVE=3  SOURCE_FUSED=4
VALIDITY_UNKNOWN=0   VALIDITY_VALID=1        VALIDITY_INVALID=2
float64[36] covariance
```

本方案**不新增消息类型**：三条链路都用 `SOURCE_PLATFORM_RELATIVE` / `SOURCE_FUSED`
写出，链路差异记在 `covariance` 和 validity 上。
按 ADR-006：缺少已验证的变换 ⇒ 发 `transform_unavailable`，不发猜测位姿；
无效 ⇒ NaN 占位，不复用上一帧。

每目标每次求解落盘一条记录（新增 CSV/JSON，不改消息）：

```text
frame_id, obj_id, class_name, chain(A|B|C),
x_map_m, y_map_m, z_map_m, qx, qy, qz, qw,
sigma_x_m, sigma_y_m, sigma_z_m, sigma_rot_deg,
n_views, baseline_m, depth_source(triangulation|support_plane|uwb_fallback),
validity, degraded_reason, runtime_ms
```

## 5 L0 前置修复（三条链路共同前提，先做，零新网络）

这一层不引入任何新方法，只是把 E012 里被关掉、写错、缺失的东西补上。
**不做 L0，后面三条链路的对比全部不可信。**

| 编号 | 修复 | 位置 | 依据 |
|---|---|---|---|
| L0-1 | 对称群写进目录 + 导出 BOP `symmetries_discrete` / `symmetries_continuous` | `simulation/target_catalog_v01.yaml`、`generate_target_detection_dataset.py` | 4.3；上游 `lib/pysixd/misc.get_symmetry_transformations` **已能解析**，无需改 submodule |
| L0-2 | 打开 GDR-Net 自带损失 | `train_gdr_net_synthetic.py`：`PM_LW`、`REGION_LW` 非零，`PM_LOSS_SYM=True` | `GDRN.py:411` 已接好 `PM_LOSS_SYM` |
| L0-3 | `ROT_HEAD.NUM_CLASSES = 10` | 同上 | 十个尺寸/形状差异大的目标不能共享一个头（1.3a） |
| L0-4 | `Z_TYPE`/`Z_LOSS_TYPE` 口径改正，或**直接不用网络 z** | 同上 | L1 最优解是条件中位数 ⇒ 系统性偏近（1.2） |
| L0-5 | 指标脚本对称键名对齐 | `code/analysis/evaluation/pose_metrics.py`（现只认 `symmetry == "continuous_z"`） | 目录从不发这个键 ⇒ ADD-S 分支实际从未触发 |
| L0-6 | 渲染 `decoration` | `_target_mesh()` | 条纹/端带/顶环不渲染 ⇒ 连续对称目标 yaw 无任何线索 |
| L0-7 | 数据集姿态多样性 | `_look_at()` 锁定相机 roll 到 map `+X`；`rotation_map_target = _rotation_z(yaw_deg)` 每目标每划分只有一个朝向 | 相对旋转只覆盖 SO(3) 里约 1-D 曲线 ⇒ 现数据集**支撑不了通用 6-DoF 估计器** |
| L0-8 | 相机高度口径 | `imx219_gazebo_camera.yaml` `spawn_pose_m: [2.0,2.35,2.5]` vs `camera_height_m: [3.05,3.65]` | 两者不一致，需 ADR 定一个 |

L0-1、L0-5、L0-8 属**口径变更**，按 `AGENTS.md:12` 必须写进 ADR-009，
不得直接改动历史结论；E003/E005 曾写 `continuous rotation about z; use ADD-S`，
与 v01 目录的 `symmetry: none` 直接矛盾，也在 ADR-009 里一并裁定。

**L0 的预期效果（TARGET，不是承诺）**：仅扣除逐目标深度均值偏差就把平移
RMSE 从 `102.5 mm` 降到 `67.8 mm`（1.2 实算）。L0 让网络自己学到这部分，
是"不换网络"的第一笔收益；但它**到不了 2 cm**，因为 1.3b 的通道问题还在。

## 6 算法流程（统一求解器 + 八个因子）

### 6.1 总流程图

```text
                     ┌──────────────── 离线一次性（地面）────────────────┐
                     │  目录 → 网格/直径/对称群  |  控制点survey  |  锚点survey │
                     └───────────────────────┬─────────────────────────┘
                                             │(常量)
 图像序列 3280×2464 @10Hz
   │
   ├─▶ [S1] YOLO 检测        → class_id, bbox_xywh, conf         (已 IMPLEMENTED, mAP50=1.0)
   │                                │
   ├─▶ [S2] ROI 裁剪 + 稠密对应  → 2D↔3D 对应 + 每点权重/不确定度   (GDR-Net 头, 只取对应, 不取 z)
   │                                │
   ├─▶ [S3] 轮廓区域对应线      → 沿投影轮廓法向的一维对应          (SRT3D/ICG 思路, 单CPU核)
   │                                │
   ├─▶ [S4] 场景锚定特征        → 图像↔控制点匹配                  (XFeat, 需先下采样)
   │                                │
   └─▶ [S5] 帧间相对位姿        → T_{c_i c_j} 先验                 (ORB-SLAM2, 已有 submodule)
                                    │
        UWB 距离 + 锚点坐标 ─▶ [S6] 多边定位 → base_link 位姿 + 协方差 (已 IMPLEMENTED)
                                    │
                                    ▼
        ┌───────────────────────────────────────────────────────────┐
        │ [S7] 稀疏最小二乘捆绑  scipy.optimize.least_squares         │
        │      method='trf', loss='soft_l1', jac_sparsity=...        │
        │      状态 ≈300 参数：10×6DoF 目标位姿 + 段级尺度 + 少量标定项  │
        │      因子 F1..F8（见 6.3），全部用信息矩阵加权                │
        └───────────────────────────┬───────────────────────────────┘
                                    │
                     ┌──────────────┴──────────────┐
                     ▼                             ▼
        map→target_link 位姿 + 协方差        validity / degraded_reason
                     │                             │
                     └──────────────┬──────────────┘
                                    ▼
                      [S8] 输出与可视化（第 11 节）
```

### 6.2 每步输入输出与算力

| 步 | 输入 | 输出 | 档位 | 备注 |
|---|---|---|---|---|
| S1 | RGB | `class_id, bbox, conf` | CPU-实时（yolo11n, imgsz 960） | 已饱和，不动 |
| S2 | ROI 256×256 | 稠密 2D–3D 对应 + 权重 | 小GPU-离线训练 / CPU-可跑推理 | **丢弃 z 回归输出** |
| S3 | ROI + 网格 + 位姿初值 | 对应线残差 | CPU-实时（ICG 报 `1.3 ms`/核） | 只需网格，专治无纹理 |
| S4 | 下采样图 + 控制点 | 2D–3D 锚定对应 | CPU-实时（VGA 尺度） | 高分辨率必须先降采样 |
| S5 | 图像序列 | 帧间相对位姿 | CPU-实时 | 尺度未定，靠 F6 定 |
| S6 | 4×距离 + 锚点坐标 | `base_link` 位姿 + 协方差 | 近零 | 基准量，非目标观测 |
| S7 | 全部因子 | 10 个 `map→target` + 协方差 | CPU，360 实例目标 `<60 s` | 无新依赖 |
| S8 | 位姿 + 协方差 | 话题 / CSV / 前端 | 近零 | 见第 11 节 |

### 6.3 因子集

| 因子 | 含义 | 残差 | 谁提供 | 关键作用 |
|---|---|---|---|---|
| **F1** | 多视图视线（bearing-only） | 归一化像面重投影 | S2/S1 | **深度的主来源**（运动基线交会） |
| **F2** | 轮廓区域对应线 | 沿轮廓法向 1-D 距离 | S3 | 无纹理下的姿态与侧向精化 |
| **F3** | 支撑面约束 | `z_base − (deck + Δ_obj)` | 目录 | 单视图深度可观测性（`2.59 mm`） |
| **F4** | 场景控制点锚定 | 控制点重投影 | S4 | 固定 `map` 规范与尺度 |
| **F5** | 帧间相对位姿先验 | 相对位姿对数映射 | S5 | 稳基线，抑制漂移 |
| **F6** | **段级** UWB 基准/尺度 | 段内轨迹尺度与位置偏置 | S6 | 米制基准；**不进逐帧深度回路** |
| **F7** | 逐帧平台位姿先验（带协方差） | 平台位姿残差 | S6 + FCU | 弱约束，权重由协方差定 |
| **F8** | 目录弱先验 | 直立性 / `base_z` / 尺寸 | 目录 | 正则化，避免退化解 |

对称性处理：F1/F2 的残差在对称群上取最小值（BOP `symmetries_*` 展开），
与 `PM_LOSS_SYM` 同一口径；报指标时对称目标用 ADD-S 或对称最小化旋转误差
（`docs/算法比较定义参数.md:184`）。

### 6.4 为什么这不是"暴力加码"

八个因子里 **F1/F3/F6/F8 是纯几何/常量，零网络**；F2 只要网格；
F5/F6 用已有 submodule 与已有解算器；只有 F2 的初值和 F1 的对应权重来自学习件，
而学习件**只输出权重/不确定度/对应**，不输出目标坐标——严格符合 ADR-003。
净新增可训练网络 = 0（C2 达成）。

## 7 链路 A：精度优先（`chain_a_precision`）

**目标**：在视觉可见、无退化、无攻击的条件下把 `map→target_link` 平移误差压到
`2 cm` 量级（TARGET）。

配置：

| 项 | 取值 |
|---|---|
| 因子 | F1 + F2 + F3 + F4 + F5 + F6 + F8（F7 弱权重） |
| 视图数 | 段内全部可见视图（测试集每目标 36） |
| 求解 | 批处理，`least_squares(method='trf', loss='soft_l1')`，两轮：先只 F1+F3+F6 定位，再加 F2 精化姿态 |
| 深度来源 | `triangulation`（主）+ `support_plane`（单视图/短基线时） |
| 拒绝策略 | 残差 3σ 外点剔除；基线 `< b_min` 的视图对不参与三角化 |
| 不做 | 不用网络回归 z；不用 UWB 逐帧深度；不 render-compare |

预算（TARGET，基于 1.4 的闭式 + 36 个真实测试相机位姿蒙特卡洛）：

| 项 | 值 |
|---|---|
| 视线噪声假设 | `1 px` |
| N=36 三角化深度 σ | `1.10 mm` |
| 支撑面单视图深度 σ | `2.59 mm` |
| 侧向（现状即已达到） | `24.3 mm` RMS ⇒ **侧向反而成为新瓶颈**，靠 F2/F4 压 |
| 平台位姿注入（若逐帧用 UWB） | `29.59 mm`（E004 46 mm 平台噪声）／`75.89 mm`（E008 114 mm） |

⇒ **这就是 F6 只能做段级的原因**：逐帧引入 UWB 平台噪声，`2 cm` 立刻不可能。
另外系统项不随帧数平均：`0.5%` 轨迹尺度误差 → `17.1 mm`；`5 px` 恒定视线偏置 → `8.8 mm`。
所以 A 的验收必须同时报 P50/P95/availability，不允许只报均值
（`docs/算法比较定义参数.md:350-357`）。

## 8 链路 B：对抗与退化鲁棒（`chain_b_robust`）

**目标**：在遮挡 / 光照 / 运动模糊 / UWB NLOS **以及**对抗样本与对抗补丁下，
不输出错误位姿——宁可标 `INVALID`，也不给猜测值（ADR-006）。

### 8.1 威胁与退化清单

| 类别 | 项 | 注入方式（仿真） |
|---|---|---|
| 退化 | 部分遮挡 30/50/70% | ROI 内矩形/多边形遮挡 |
| 退化 | 光照：过曝、欠曝、强阴影 | gamma / 增益 / 方向光 |
| 退化 | 运动模糊 | 沿轨迹方向线性核 |
| 退化 | 背景杂乱 | 提高 `texture_landmark_count` / 加干扰体 |
| 退化 | UWB NLOS | 对部分锚点加正偏置（不是零均值噪声） |
| 对抗 | 对抗补丁贴在目标表面 | `arXiv 2108.07229` 的补丁-位姿敏感性设计 |
| 对抗 | 分割/注意力推移攻击 | `arXiv 2203.00302`：**可绕过 RANSAC** |
| 对抗 | 后门（仅作威胁分析） | `arXiv 2512.19058`，**不实现攻击工具** |

### 8.2 B 的机制（与 A 的差别）

| 机制 | 说明 |
|---|---|
| 冗余通道并联 | F1（稠密对应）‖ F2（轮廓）‖ QuadricSLAM 式**框级 bearing-only** ‖ UWB 兜底 |
| 通道一致性检验 | 三通道位姿两两距离超阈值 ⇒ 该帧 `INVALID`，写 `degraded_reason` |
| 概率 PnP | EPro-PnP 输出位姿分布 ⇒ 协方差与**多峰**（对称歧义）显式表达 |
| 不确定度门控 | Kendall & Gal `s = ln σ²` 头，只用于视图选择与外点剔除，不替代融合（ADR-003） |
| 鲁棒核 | `soft_l1` / Huber；对抗攻击能绕过 RANSAC ⇒ **不能只靠 RANSAC** |
| 几何不可伪造量 | F3 支撑面与 F4 控制点是**图像补丁攻击不到**的约束 ⇒ B 的最后防线 |

**关键论证**：对抗攻击针对的是**网络注意力**。本方案把深度从网络搬到几何后，
攻击能污染的只剩 F1/F2 的对应，而 F3/F4/F6 仍然成立 ⇒ 结构性抗攻击，
不是靠数据增强堆出来的。

### 8.3 B 的验收口径

不报"精度更高"，报**降级正确率**：

| 指标 | 定义 |
|---|---|
| `availability` | 有效帧占比（无效帧计入分母，`:350-357`） |
| 误报率 | 输出 `VALID` 但误差超阈值的比例 ⇒ **这是 B 的第一指标** |
| 漏报率 | 标 `INVALID` 但实际可用的比例 |
| `recovery_time_ms` | 退化解除后恢复有效输出的时间 |
| 分场景统计 | 每种退化/攻击单独一张表，不许混算 |

## 9 链路 C：自适应结合（`chain_c_adaptive`）

**目标**：同一套求解器，按当前观测质量在 A 与 B 的因子集/权重之间连续切换。

门控输入（全部已可计算，无新网络）：

```text
n_views_valid, baseline_m, bbox_area_px, conf,
sigma_from_epro_pnp, channel_disagreement_m,
uwb_residual_m, platform_cov_trace, blur_metric, exposure_metric
```

门控输出（对应仓库已有的 `SOURCE_*` 与分档口径）：

| 档 | 条件（TARGET，阈值待 E021 定） | 因子集 | 期望精度档 |
|---|---|---|---|
| `precision` | 多视图充足、基线够、通道一致 | A 全因子 | 视觉主导 `≤5 cm`，冲 `2 cm` |
| `transition` | 视图少 / 基线短 / 轻度退化 | F1+F3+F4+F8，F2 降权 | `8~15 cm` |
| `fallback` | 视觉不可用或通道打架 | F6+F7（UWB 兜底）+ 框级 bearing | `15~25 cm` |
| `unavailable` | 缺已验证变换 | — | 发 `transform_unavailable`（ADR-006） |

分档口径沿用已记录需求：`核心演示区 ≤5cm（视觉可见时）/ 全场景兜底 <30cm`。
**`2 cm` 全程是 TARGET，不是需求，也不是实测。**

C 的实现不是第三套代码：它就是 A/B 共用求解器 + 一个门控函数，
三条链路以配置文件区分，可单独运行、可并列对比（这是用户选定的形态）。

## 10 误差表与误差分布

### 10.1 现状逐目标误差（MEASURED，E012，源 `outputs/tables/tbl_e012_pose_by_target_v01.csv`）

**可追溯性同 1.1 的说明**：`outputs/tables/tbl_e012_pose_by_target_v01.csv` 在本仓库工作树中
不存在（`outputs/tables/` 仅有 `.gitkeep`），E012 运行目录亦不存在，下表数字来源为
`docs/优化方案.md` 的记载、**在本仓库内不可复算**；引入答辩材料前必须先补回运行目录与表格。

GT ROI 条件：

| obj | class | 平移 RMSE (m) | 平移 P50 | 平移 P95 | 旋转 RMSE (°) | 旋转 P50 | 旋转 P95 |
|---:|---|---:|---:|---:|---:|---:|---:|
| 1 | Striped tank | `0.0759` | `0.0513` | `0.1355` | `22.29` | `19.40` | `32.30` |
| 2 | Capsule tank | `0.0658` | `0.0503` | `0.1123` | `15.27` | `11.69` | `26.21` |
| 3 | Roof silo | `0.1070` | `0.0656` | `0.2282` | `22.88` | `19.40` | `33.96` |
| 4 | Hopper | **`0.1533`** | `0.1117` | `0.2781` | `31.56` | `26.45` | `52.63` |
| 5 | Hex column | **`0.0537`** | `0.0409` | `0.0961` | `33.14` | `22.86` | `59.97` |
| 6 | Box tower | `0.1023` | `0.0591` | `0.1939` | `29.91` | `18.41` | `50.98` |
| 7 | L factory | `0.0937` | `0.0788` | `0.1615` | `28.60` | `20.19` | `60.79` |
| 8 | Valve block | `0.1196` | `0.0854` | `0.2247` | `31.74` | `23.93` | `56.00` |
| 9 | Frustum | `0.1228` | `0.1199` | `0.1720` | **`45.67`** | `35.97` | `67.86` |
| 10 | Twin unit | `0.0928` | `0.0888` | `0.1481` | **`62.65`** | `31.98` | **`118.14`** |

YOLO ROI 条件与之逐项相差约 1 cm 以内（最大差异 obj2 `0.0658 → 0.1520`），
完整数据在同一 CSV 的 `YOLO ROI` 段。

**读法（重要）**：obj9/obj10 的旋转 RMSE 高不是"网络差"，而是 4.3 的对称群写错——
obj9 绕 z 连续对称、obj10 二重对称，却按 `symmetry: none` 计分。
obj10 的 P95 `118.14°` 接近 180°，是二重对称被当成错误的典型特征。

### 10.2 误差分布（MEASURED，从 `tbl_e012_pose_errors_v01.csv` 720 行重算）

**可追溯性同 1.1 的说明**：`tbl_e012_pose_errors_v01.csv` 与其 720 行逐样本数据在本仓库
工作树中不存在，下表来源为 `docs/优化方案.md` 的记载、**在本仓库内不可复算**；
按 `AGENTS.md:7`，补回可追溯运行目录之前不得作为已实测结果进入答辩材料。

| 量 | 值 |
|---|---:|
| 侧向 RMS | `24.3 mm` |
| 深度 RMS | `99.6 mm` |
| 深度占平移平方误差 | `94.4%` |
| 深度/侧向 | `4.11` |
| 深度均值偏差 | `−71.3 mm`（十目标全为负） |
| 逐目标深度偏差 | `−17.3 mm`(obj5) ～ `−120.6 mm`(obj4) |
| 扣逐目标均值偏差后 | 深度 `63.3 mm`，总平移 `67.8 mm` |
| `corr(dz, Z)` | `−0.29` |
| 相机距离 | 均值 `3.550 m`，min `2.936`，max `4.077` |

分布形状要在文档配图里给出（现有 `outputs/figures/fig_e012_pose_error_distribution_v01.png`
已画总体分布，**缺**深度/侧向分解直方图与逐目标偏差条形图 ⇒ E013 补）。

### 10.3 三链路对比表模板（TARGET，实测后填入并标运行号）

同一测试集 `test_perimeter_arc` 360 实例、同一真值、同一评价脚本；
不做跨算法补点，不自由 SE(3) 对齐；无效帧计入 `availability` 分母。

| 指标 | E012 基线 | L0 后 | 链路 A | 链路 B | 链路 C |
|---|---:|---:|---:|---:|---:|
| `translation_rmse_3d_m` | `0.1124` | TBD | TBD | TBD | TBD |
| `translation_p50_3d_m` | `0.0775` | TBD | TBD | TBD | TBD |
| `translation_p95_3d_m` | `0.2033` | TBD | TBD | TBD | TBD |
| 深度 RMS / 侧向 RMS | `99.6 / 24.3 mm` | TBD | TBD | TBD | TBD |
| `rotation_rmse_deg`（对称最小化） | `35.92`（口径错） | TBD | TBD | TBD | TBD |
| `rotation_p95_deg` | `67.70` | TBD | TBD | TBD | TBD |
| `add_m` / `add_s_m` | — | TBD | TBD | TBD | TBD |
| `add_s_recall`@10%d | `2.22%`（ADD，非 ADD-S） | TBD | TBD | TBD | TBD |
| `reprojection_error_p95_px` | 未记录 | TBD | TBD | TBD | TBD |
| ≤2 cm 比例 | `13.33%` | TBD | TBD | TBD | TBD |
| `availability` | 未记录 | TBD | TBD | TBD | TBD |
| `update_rate_hz` | — | TBD | TBD | TBD | TBD |
| `latency_mean_ms` / `latency_p95_ms` | `7.48` / 未记录 | TBD | TBD | TBD | TBD |
| `recovery_time_ms` | — | — | — | TBD | TBD |
| 误报率（B 第一指标） | — | — | — | TBD | TBD |

指标名严格取自 `docs/算法比较定义参数.md`，不新造名字。

### 10.4 退化/对抗分场景表模板（链路 B/C，TARGET）

| 场景 | `translation_rmse_3d_m` | `translation_p95` | `availability` | 误报率 | `recovery_time_ms` |
|---|---:|---:|---:|---:|---:|
| 无退化 | TBD | TBD | TBD | TBD | — |
| 遮挡 30 / 50 / 70% | TBD | TBD | TBD | TBD | TBD |
| 过曝 / 欠曝 / 强阴影 | TBD | TBD | TBD | TBD | TBD |
| 运动模糊 | TBD | TBD | TBD | TBD | TBD |
| 背景杂乱 | TBD | TBD | TBD | TBD | TBD |
| UWB NLOS 正偏置 | TBD | TBD | TBD | TBD | TBD |
| 对抗补丁 | TBD | TBD | TBD | TBD | TBD |
| 注意力推移攻击 | TBD | TBD | TBD | TBD | TBD |

## 11 运行画面与输出流程

### 11.1 现有输出链（IMPLEMENTED，实测代码）

```text
[求解器 / 节点]                      [话题]                                    [消费者]
vision_node ─────────▶ /nexus/vision/target_observation (camera 帧)
                              │
coord_transform_node ──▶ /nexus/vision/map_target_observation (map 帧)
   (tf2 lookup_transform 按采样时刻查询;
    协方差只旋转 6×6 的 [:3,:3] 块;
    查不到 ⇒ invalid_reason="transform_unavailable")
                              │
fusion_node ───────────▶ /nexus/target/pose  (SOURCE_FUSED=4)
   (fusion.py 逆协方差/信息形式加权;
    max_age_ms=250, max_pair_delta_ms=50;
    超时 ⇒ degraded_reason="unsynchronized_sources";
    只保留位置 3×3 块, 索引 (0,1,2,6,7,8,12,13,14))
                              │
       ┌──────────────────────┼───────────────────────────┐
       ▼                      ▼                           ▼
dashboard_node          rosbridge_websocket:9090     record_topics.launch.py
 Marker×2 @1Hz           web/src/services/            ros2 bag（7 个话题）
 /nexus/viz/target_observation (橙 1.0,0.7,0.0)
 /nexus/viz/target_pose        (绿 0.0,1.0,0.2, sphere 0.15)
       │                      │
       ▼                      ▼
 rviz2 nexus_target.rviz   浏览器 index.html → main.js → dashboard_store.js
 Fixed Frame=map           左：CARLA 数据流卡片  中：相机成像 640×480
 Grid/Odometry/Marker×2/TF 右：当前目标位置 / 链路健康 / 已登记实验指标
```

前端订阅（`web/src/services/rosbridge_client.js`）：
`/nexus/target/pose`、`/nexus/vision/map_target_observation`、
`/nexus/uwb/target_observation`（均 `nexus_msgs/msg/TargetObservation`）、
`/nexus/gazebo/uav/odom`（节流 100 ms）、
`/nexus/camera/imx219/image_raw/compressed`（节流 1000 ms + 二次门 900 ms ⇒ **≈1.1 Hz**）；
`validity !== 1` 的消息直接丢弃；算法路由通过 `/nexus/algorithm_route`（`std_msgs/String`）下发。
状态流另有 `ws://<host>:8766/ws/carla_status`（`simulation/carla/carla_status_bridge.py`，
实况 10 Hz / mock 20 Hz，缩略图 0.4 s 节流、宽 480）。

### 11.2 本方案对输出链的改动（最小化）

**不新增消息类型、不改 fusion 信息形式加权、不改 tf2 查询语义。** 只做四件事：

| 编号 | 改动 | 位置 | 理由 |
|---|---|---|---|
| V1 | 在 `/nexus/target/pose` 写出**求解器给的完整位置协方差**（不再是各向同性对角） | 新求解节点填 `covariance[0..2,6..8,12..14]` | 链路 B/C 的门控与前端椭圆都需要真协方差 |
| V2 | 新增 RViz 可视化：位姿协方差椭球 + 视线束 + 支撑面 | `dashboard_node` 增 Marker，不改话题名 | 让"深度靠交会得到"在画面上可见——答辩要的就是这个 |
| V3 | 前端右侧"当前目标位置"面板加 `σx/σy/σz`、`n_views`、`baseline_m`、`depth_source`、`chain` | `dashboard_store.js` + `dashboard_layout.js` | 已有面板扩字段，不新建面板 |
| V4 | 相机预览恢复到 IA 规定的 `5–10 FPS`（现为 ≈1.1 Hz） | `rosbridge_client.js:135` 的二次门 | IA 文档 §3.3 写的是 640×480 @5–10 FPS，实现比规格慢了一个量级 |

### 11.3 静态证据画面（离线，进 PPT 的那几张）

**搬运时核对（与原方案不同）**：原方案假定的两个产图脚本
`code/analysis/evaluation/plot_e012_pose_results.py` 与
`code/analysis/evaluation/export_e008_simulation_evidence.py` 在本仓库中**不存在**
（`code/analysis/evaluation/` 下只有 `evaluate_cli.py`、`map_pose_evaluate_cli.py`、
`metrics.py`、`observation_quality.py`、`observation_quality_cli.py`、
`pose_evaluate_cli.py`、`pose_metrics.py`），因此"复用已有 `plot_transform`
（`:189-208`）"和"扩 `plot_runtime_frame`（`:211-237`）"这两条路径都不成立。
改为：下列六张图全部由 `code/analysis/evaluation/plot_pose_results.py` **统一产出**
（这是本次实现新建的唯一产图入口）；图的清单与内容描述保留原方案不变：

| 图 | 内容 | 来源 |
|---|---|---|
| 坐标链关系图 | `map → (map→camera) → (camera→target) → target`，标注 `T_map_target = T_map_camera × T_camera_target` | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |
| 运行帧叠加图 | RGB 帧上绿圈=真值中心、红叉=估计中心，扩成三链路三色 | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |
| 深度/侧向分解直方图 | 新增（10.2 缺这张） | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |
| 逐目标深度偏差条形图 | 新增，显示十目标全负偏 | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |
| 三链路 P50/P95 箱线对比 | 新增 | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |
| 协方差椭球 vs 实际误差校准图 | 新增，验证 B/C 的协方差不是装饰 | `plot_pose_results.py`（新建，原方案假定的两个脚本在本仓库不存在） |

输出目录沿用 `outputs/figures/`、`outputs/tables/`、`outputs/demos/`、`outputs/reports/`。

### 11.4 演示录屏流程（当前**完全缺失**，本方案补）

`defense/demo/README.md` 只有三行占位（"保存演示脚本、现场检查清单和录屏索引"），
仓库里没有任何录屏脚本、检查清单或索引；唯一已实现的"记录"是
`record_topics.launch.py` 的 `ros2 bag`（那是 `REPLAY` 素材，不是录屏）。
`defense/slides/2026-08-19_plan_defense_slide_content.md:242` 的
`[演示视频链接/文件]` / `[rosbag/运行编号]` / `[RViz 配置版本]` / `[录屏日期]` 四个占位仍空。

补齐的固定流程（写入 `defense/demo/`）：

```text
1. 起服务（顺序照 simulation/2026-08-28_runbook_carla_dashboard_dual_window.md §9）
   CARLA(Windows, RPC 127.0.0.1:2000) → carla_status_bridge(:8766)
   → rosbridge_websocket --port 9090 → http.server 8765
   → ros2 launch nexus_bringup target_localization.launch.py
2. ros2 bag record（record_topics.launch.py），记下 bag 路径与 sha256
3. 三链路依次通过 /nexus/algorithm_route 切换，每档录 60 s
4. 屏幕录制 RViz + 浏览器双窗；文件不入 Git，只在 defense/demo/ 登记外部路径 + sha256
5. 回填 slide 的四个占位；数字必须能追到 experiments/runs/ 的运行号（ADR-002）
```

### 11.5 运行画面前必须先解决的三个硬缺口（IMPLEMENTED 事实）

| 缺口 | 证据 | 影响 |
|---|---|---|
| **TF 表为空** | `nexus_bringup/config/pre_hardware_transforms.yaml` `transforms: []`，`data_status: tbd_until_hardware_calibration`，`map_definition: "TBD"` | `coord_transform_node` 必然发 `transform_unavailable` ⇒ `/nexus/target/pose` 出不来，RViz 与前端全空 |
| **没有 URDF / xacro / SDF，也没有 `static_transform_publisher`** | 全仓库 `find` 无命中；唯一静态 TF 广播器读的就是上面那张空表 | 同上；`base_link → camera` 只在数据集 YAML 里以理想单位阵存在，从未进过 TF 树 |
| **`gazebo_sandbox.launch.py` 引用的世界文件不存在** | `:11-12` 指向 `nexus_bringup/simulation/nexus_sandbox_imx219.world`，仓库无任何 `.world`（且 git status 显示 `simulation/nexus_sandbox_imx219.world` 已被删除） | 该 launch 在 `gzserver` 启动即失败 |

⇒ **运行画面的第一优先级不是加图元，是把 TF 树填上。** 这三项列入 E013 的前置。

### 11.6 安全提醒（顺带发现，需处理）

`code/carla_bridge/server.py` 的 FastAPI 网关 `allow_origins=["*"]` 且
`/ws/carla`、`/ws`、`/ws/carla_status`、`/ws/nexus` **四个 WebSocket 端点全部无认证**，
而 README 指导以 `--host 0.0.0.0` 绑定。同网段任何人都能读遥测流，并能向
`/ws/nexus` 注入 `target` 指令。答辩现场若走共享网络，至少要绑回 `127.0.0.1`
或加一个共享 token。这属于本方案范围外的独立问题，但必须记录。

## 12 误差预算（TARGET，链路 A，`map→target_link` 平移）

按独立项平方和合成。所有数值为设计预算，不是实测。

| 项 | 来源 | 预算 | 可否随帧数平均 |
|---|---|---:|---|
| 视线噪声 → 深度（N=36 交会） | F1 | `1.1 mm` | 是 |
| 视线噪声 → 侧向 | F1/F2 | `6.0 mm` | 是 |
| 支撑面高度不确定（`base_z` 已知到 ±2 mm） | F3 | `2.6 mm` | 部分 |
| 目标网格几何误差（参数化 proxy，`dimension_confidence: C`） | 目录 | `8.0 mm` | **否** |
| 控制点 survey 误差 | F4 | `5.0 mm` | **否** |
| `base_to_camera` 外参（现为理想单位阵，未标定） | 标定 | `5.0 mm` | **否** |
| 段级 UWB 尺度/基准残差 | F6 | `10.0 mm` | **否** |
| 时间同步残差（10 Hz，`max_pair_delta_ms=50`） | 系统 | `4.0 mm` | 部分 |
| **合成（RSS）** | | **`≈17 mm`** | |

> **勘误（2026-09-02）**：上表**漏了基线不确定性项** `Z·δB/B`。逐帧用 UWB 定基线时，
> 该项单独就是 `53 mm`（`Z=5 m`、`B=4 m`、UWB 单帧位置 σ=30 mm），比表里所有项之和还大。
> 修正后的完整预算见附五.2 与附五.7；结论是**必须有 VIO/VO 提供段内相对几何**，
> 段级 UWB 尺度才有东西可乘。

**读法**：预算刚好落在 `2 cm` 以内，但**四项系统误差贡献了绝大部分**
（网格 8 + 控制点 5 + 外参 5 + UWB 基准 10，仅这四项 RSS 已是 `14.1 mm`）。
结论有两层：

1. 在**仿真**里（网格即真值、外参即单位阵、锚点即真值），系统项几乎为零，
   `2 cm` 是可达的 —— 这也是用户选定的 `暂不实测，先在仿真里把算法做到 2 cm`。
2. 在**实物**上，`2 cm` 的前提是控制点与外参必须实测标定，且目标要有实测 CAD。
   只要 `simulation/target_catalog_v01.yaml` 仍是
   `source: designed_parameterized_proxy_without_cad`，实物 `2 cm` **不可承诺**。

这一层区分必须写进文档正文和 ADR-009，不允许在答辩材料里混为一谈
（`AGENTS.md:35`：不能把"代码已写"表述为"精度已达标"）。

## 13 实验序列（E013–E015、E018–E022）

**编号勘误（2026-09-01 核对 `experiments/runs/`）**：`docs/优化方案.md` 原文写的是
`E013–E020` 连号，但 `2026-08-31_E016_carla_sandbox_v29_full_gdrn` 与
`2026-08-31_E017_carla_sandbox_v29_camera_localization` 已占用 E016/E017，
因此原 E016–E020 顺延为 E018–E022。E013/E014/E015 未被占用，保持原编号，
全文其他章节对它们的引用不变。另外 `experiments/runs/` 里存在两个 E007
（`2026-08-27_E007_orb_slam2_mono_simulation` 与 `2026-08-30_E007_lightglue_sandbox`），
属既有编号重复，本方案不改写历史目录，只登记。

每个运行按 `AGENTS.md:27` 命名 `YYYY-MM-DD_E<三位编号>_<slug>`，
并在 `experiments/runs/` 下留数据、配置、代码版本、结论。

| 编号 | 内容 | 产出 | 前置 |
|---|---|---|---|
| **E013** | L0 前置修复 + 数据集重生成（对称群、decoration、姿态多样性、`base_z`）+ 补齐 TF 表与坐标链画面缺口（11.5） | 新数据集 + 深度/侧向分解图 + 逐目标偏差图 | — |
| **E014** | L0 后重训 GDR-Net（`NUM_CLASSES=10`、`PM_LW/REGION_LW` 开、`PM_LOSS_SYM`），只取对应不取 z | 对照基线，验证"不换网络"的收益 | E013 |
| **E015** | F1+F3 最小捆绑（`scipy.least_squares`，≈300 参数）—— **这是全案的关键验证** | 深度 RMS 是否从 `99.6 mm` 降到 `<10 mm` | E014 |
| **E018** | XFeat 在 `3280×2464` 上的实测算力与下采样策略；F4 控制点锚定 | CPU 时延实测表（不许照抄论文） | E013 |
| **E019** | F2 轮廓区域对应线（SRT3D/ICG 方法论自研实现或 C++ 绑定） | 侧向与姿态精化收益 | E015 |
| **E020** | 链路 A 完整（F1–F6+F8）+ EPro-PnP 协方差；协方差校准图 | 链路 A 全指标表 | E015, E019 |
| **E021** | 链路 B：退化 + 对抗注入，标定门控阈值 | 分场景表（10.4）+ 误报/漏报率 | E020 |
| **E022** | 链路 C：门控自适应；三链路并列对比 | 10.3 对比表填满 | E020, E021 |

树莓派 5 上的 CPU 端侧验证（C1/C6）挂在 E018 与 E020 之后单独一次运行；
ONNX-int8 导出为新增工作量（XFeat 官方仅列 TODO），若时间不够则明确标 TBD，
不得写成"已部署"。

## 14 验证方式

| 层 | 怎么验 |
|---|---|
| 观测通道 | E015 单独看深度 RMS：若 F1+F3 不能把深度从 `99.6 mm` 压到 `<10 mm`，1.4 的推导有错，立即停下重推 |
| 求解器 | 用 BOP 真值构造合成观测（加已知噪声），检查解出的位姿与协方差是否统计一致（χ² 检验） |
| 指标 | 全部走 `code/analysis/evaluation/pose_metrics.py`；对称目标必须走 ADD-S 分支（L0-5 修好后确认分支真的进了） |
| 坐标链 | `simulation/fuse_uwb_visual_targets.py` 链式与本方案求解器输出对同一帧应一致（差 `<1 mm`），否则外参/帧约定有 bug |
| 端到端 | `target_localization.launch.py` 起全链，RViz + 浏览器出图；`ros2 bag` 记 7 个话题；数字回填 `docs/records/evidence-index.md`（现仅占位行） |
| 算力 | 树莓派 5 上实测单帧与整段耗时，报 `latency_mean_ms` / `latency_p95_ms` |

### 14.1 已完成的求解器层验证（2026-09-01，**合成观测，不是数据集运行**）

`code/analysis/test/test_localization_bundle.py`，`PYTHONPATH=code/analysis python3 -m pytest code/analysis/test -q`
⇒ `48 passed`。场景是按 4.3 的 obj1 几何（`190×190×130 mm`，`base_z 50 mm`）
构造的合成观测：8 个下视相机位姿、`1 px` 高斯视线噪声、只开 F1+F3、`loss='linear'`、
相机位姿固定为真值。12 次独立试验：

| 量 | 值 |
|---|---:|
| 平移误差 RMS | `0.89 mm`（max `1.92 mm`） |
| 沿 map z（本几何下即深度）RMS | `0.82 mm`（max `1.91 mm`） |
| 协方差 χ² 一致性 `mean(e²/σ²)`（40 次试验） | `1.093`（理想 `1.0`） |

**这三行只证明"求解器与协方差实现本身是自洽的、1.4 的量级推导没有算错"。**
它**不是** E015，不是 `test_perimeter_arc` 上的实测，也不代表任何链路精度：
合成观测里没有网格几何误差、没有外参误差、没有控制点误差、没有检测器抖动，
而 12 节的误差预算说这四项系统项才是 `≈17 mm` 里的大头。
按 `AGENTS.md`，只有 `experiments/runs/` 中有数据、配置、代码版本和结论的运行
才可进入答辩材料；上表不构成 MEASURED。

## 15 风险、停止条件与许可核对

### 15.1 风险

| 风险 | 影响 | 处置 |
|---|---|---|
| 现数据集姿态多样性只有 ~1-D（L0-7） | 任何 6-DoF 估计器都学不出来，A/B/C 对比失去意义 | E013 必须先修，否则整个序列不启动 |
| 目标无实测 CAD（`dimension_confidence: C`） | 实物 `2 cm` 不可达（12 节） | 只在仿真声明 `2 cm`；实物结论另开运行 |
| 沙盘近平面 ⇒ COLMAP/hloc 经典退化 | F4 控制点锚定可能建不起来 | F4 改为**人工测量控制点**而非 SfM；从未为 SfM 拍过照，需先试 |
| FanciSwarm 可能不暴露原始 UWB 距离（ADR-001 已警告） | F6 拿不到距离，只能拿厂家解算位置 | 退化为位置级基准因子，协方差放大 |
| XFeat 高分辨率下 CPU 不实时（2.2） | C1 不成立 | 下采样/分块，E018 实测决定 |
| ORB-SLAM2 GPLv3 | 分发边界 | 保持 submodule 形式，不静态链接进本项目发布物 |
| TF 表为空 + 无 URDF（11.5） | 端到端画面出不来 | 列为 E013 前置 |

### 15.2 停止条件（触发即停，写进运行记录，不许绕）

- 独立真值验证未完成前，**不得写"达到厘米级"**（研究文档停止条件原文）。
- E015 若深度未显著改善 ⇒ 停止，重新推导 1.4，不得靠换网络掩盖（ADR-008
  `不通过更换网络假装解决`）。
- 任何仿真结果不得表述为实物精度；不得以"UWB 无人机位置准确"替代
  "沙盘目标 `map → target_link` 准确"。
- 学习件若被改成直接输出目标坐标 ⇒ 违反 ADR-003，回退。

### 15.3 许可核对表（2026-08-29 直接拉 LICENSE 核对）

| 仓库 | 许可 | 可否接入 |
|---|---|---|
| `DLR-RM/3DObjectTracking`（SRT3D/ICG/M3T） | **MIT**，© 2023 Manuel Stoiber, DLR | ✅ 需署名；C++ 工程，接入需绑定层 |
| `verlab/accelerated_features`（XFeat） | **Apache-2.0** | ✅ ONNX/TensorRT 导出官方仅 TODO |
| `tjiiv-cprg/EPro-PnP` | **Apache-2.0**，© Alibaba Group | ✅ |
| `NVlabs/FoundationPose` | NVIDIA 自有非商业条款 | ❌ 只作 REFERENCE，不接入 |
| GDR-Net（已有 submodule） | 见 submodule，接入前复核 | 已在用，不修改上游 |
| ORB-SLAM2（已有 submodule） | GPLv3 | ⚠ 注意分发边界 |
| PVNet / CosyPose / QuadricSLAM / CubeSLAM / hloc / ACE | TBD | 未核对前不写入依赖 |

## 16 参考文献

- Wang et al., **GDR-Net: Geometry-Guided Direct Regression Network for Monocular 6D Object Pose Estimation**, CVPR 2021.
- Stoiber et al., **SRT3D: A Sparse Region-Based 3D Object Tracking Approach for the Real World**, IJCV 2022.
- Stoiber et al., **Iterative Corresponding Geometry (ICG)**, CVPR 2022（`arXiv:2203.05334`，报 `1.3 ms/帧 单 CPU 核`）。
- Potje et al., **XFeat: Accelerated Features for Lightweight Image Matching**, CVPR 2024.
- Chen et al., **EPro-PnP: Generalized End-to-End Probabilistic Perspective-n-Points for Monocular Object Pose Estimation**, CVPR 2022（Oral / Best Student Paper）。
- Labbé et al., **CosyPose: Consistent Multi-View Multi-Object 6D Pose Estimation**, ECCV 2020.
- Peng et al., **PVNet: Pixel-wise Voting Network for 6DoF Pose Estimation**, CVPR 2019.
- Wen et al., **FoundationPose**, CVPR 2024（REFERENCE 上界，不接入）。
- Nicholson et al., **QuadricSLAM**, RA-L 2019；Yang & Scherer, **CubeSLAM**, T-RO 2019.
- Mur-Artal & Tardós, **ORB-SLAM2**, T-RO 2017.
- Brachmann et al., **ACE: Accelerated Coordinate Encoding**, CVPR 2023；Sarlin et al., **hloc / SuperGlue**, CVPR 2019/2020.
- Kendall & Gal, **What Uncertainties Do We Need in Bayesian Deep Learning for Computer Vision?**, NeurIPS 2017.
- **Adversarial samples for deep monocular 6D object pose estimation**，`arXiv:2203.00302`。
- **Backdoor Attacks in 6DoF Object Pose Estimation**，`arXiv:2512.19058`。
- **How Sensitive are Patch Attacks to 3D Pose?**，`arXiv:2108.07229`。
- Hodaň et al., **BOP** 数据集与指标规范（ADD / ADD-S / `symmetries_*`）。
- 本项目内部：`docs/2026-08-24_research_markerless_visual_target_localization.md`、
  `docs/算法比较定义参数.md`、ADR-001/002/003/005/006/007/008、
  `experiments/runs/2026-08-29_E012_target_catalog_clean_retrain/`。

---

## 附：落地清单（实现阶段要碰的文件）

**新建**

| 文件 | 内容 |
|---|---|
| `docs/2026-08-29_design_markerless_target_localization_optimization_v01.md` | 上面第 0–16 节全文 |
| `docs/decisions/2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md` | 由 `templates/adr.md` 复制，裁定头部六项 |
| `code/analysis/localization/bundle_solver.py` | F1–F8 稀疏最小二乘（`scipy.optimize.least_squares`，`method='trf'`、`loss='soft_l1'`、`jac_sparsity`） |
| `code/analysis/localization/factors.py` | 八个因子的残差与雅可比稀疏结构 |
| `code/analysis/localization/gating.py` | 链路 C 的门控函数（输入见第 9 节） |
| `config/chain_a_precision.yaml` / `chain_b_robust.yaml` / `chain_c_adaptive.yaml` | 三条链路的因子集与阈值，可单独运行 |
| `simulation/inject_degradations.py` | 遮挡/光照/模糊/杂乱/NLOS/对抗补丁注入（第 8.1 节清单） |
| `defense/demo/2026-08-29_checklist_demo_recording.md` | 11.4 的录屏流程与检查清单 |

**修改（复用现有实现，不重写）**

| 文件 | 改动 |
|---|---|
| `simulation/target_catalog_v01.yaml` | 十目标 `symmetry` 改正 + 每目标 `base_z` |
| `simulation/generate_target_detection_dataset.py` | 导出 BOP `symmetries_*`；`_target_mesh()` 渲染 `decoration`；`_look_at()` 放开 roll、目标朝向多样化（L0-6/L0-7） |
| `code/analysis/vision/train_gdr_net_synthetic.py` | `ROT_HEAD.NUM_CLASSES=10`、`PM_LW`/`REGION_LW` 非零、`PM_LOSS_SYM=True`；不再使用回归 z（L0-2/3/4） |
| `code/analysis/evaluation/pose_metrics.py` | 对称键名对齐 BOP `symmetries_*`（L0-5），确认 ADD-S 分支真的进入 |
| `code/analysis/evaluation/plot_pose_results.py` | **新建**：11.3 的六张图统一入口（坐标链、运行帧叠加、深度/侧向分解、逐目标偏差、三链路箱线、协方差校准）¹ |
| `code/analysis/algorithms/router.py` | 注册三条链路为新 `AlgorithmSpec`（`vision.bundle_a/b/c`），沿用 `AlgorithmResult` 契约 |
| `code/ros2_ws/src/nexus_bringup/config/pre_hardware_transforms.yaml` | 填入 `map`/`base_link`/`camera` 的可用变换（11.5 第一优先级） |
| `code/ros2_ws/src/nexus_viz_dashboard/.../dashboard_node.py` | V2：协方差椭球 + 视线束 + 支撑面 Marker（不改话题名） |
| `web/src/features/telemetry/dashboard_store.js`、`components/dashboard_layout.js` | V3：右侧面板加 `σx/σy/σz`、`n_views`、`baseline_m`、`depth_source`、`chain` |
| `web/src/services/rosbridge_client.js` | V4：相机预览二次门（`:135` 的 900 ms）放宽到 IA 规定的 5–10 FPS |
| `docs/README.md`、`docs/PROJECT_INDEX.md`、`docs/decisions/README.md` | 登记新文档与 ADR-009 |

¹ 原方案此处列 `code/analysis/evaluation/plot_e012_pose_results.py` 与
`code/analysis/evaluation/export_e008_simulation_evidence.py` 两行「修改」，但这两个文件
在本仓库中**不存在**（理由同 11.3：`code/analysis/evaluation/` 下无任何调用 `savefig` 的脚本），
没有可修改的对象，故合并为一行新建入口 `plot_pose_results.py`。

**不碰**：GDR-Net / ORB-SLAM2 上游 submodule、`nexus_msgs` 消息定义、
`fusion.py` 的信息形式加权、`coord_transform_node.py` 的 tf2 查询语义、
论文原文与 `docs/` 中已有规划和图片。

**实现阶段实际新增的、原清单未列的文件**（2026-09-01 落地时补齐，全部服务于本方案已写明的口径）

| 文件 | 为什么需要 |
|---|---|
| `code/analysis/localization/observations.py` | S2/S1 输出 → F1 因子的转换；含 `factor_symmetries()` 把 BOP `symmetries_*` 交给 F1/F2 做对称最小化（6.3 要求，连续轴按 12 步展开，指标脚本仍用 72 步） |
| `code/analysis/localization/contour_lines.py` | S3/F2 沿投影轮廓法向的一维对应线搜索（SRT3D/ICG 方法论自研实现，未接入 C++ 工程） |
| `code/analysis/localization/xfeat_anchor.py` | S4/F4：XFeat 加载 + **显式下采样**（2.2 的高分辨率前提）+ 控制点匹配转 F4 |
| `code/analysis/localization/pose_distribution.py` | EPro-PnP 位姿样本 → 协方差与多峰标记（8.2 概率 PnP 一栏） |
| `code/analysis/localization/channel_consistency.py` | 8.2 的通道一致性检验：可用通道 <2 时 disagreement 记 `inf`，不可测的交叉校验不许当成通过的交叉校验 |
| `code/analysis/localization/records.py` | 4.5 的逐次求解落盘契约（列名与 4.5 一字不差）；σ 非有限或非正时写 `None`，不写假的 `0` |
| `code/analysis/localization/bundle_io.py` | 因子/问题的 JSON 序列化，让一次运行可复现重跑；参照文档为 `simulation/markerless_bundle_example_v01.json` |
| `code/analysis/localization/chain_runner.py` | 三条链路到 `AlgorithmResult` 契约的适配（router 注册的就是它） |
| `code/analysis/localization/run_chain_cli.py` | 「三条链路可单独运行、可并列对比」的实际入口：`--chain chain_a_precision\|chain_b_robust\|chain_c_adaptive` |
| `simulation/target_detection_dataset_v02.yaml` | E013 数据集配置。v01 保持原样以便旧运行可复现；姿态多样性（L0-7）只在 v02 打开 |
| `code/analysis/test/test_localization_bundle.py` | 14.1 的求解器层验证（深度、χ²、对称吸收、阶段调度、退化理由、记录契约） |

同时落地了两处原清单未列但 ADR-009 裁定必须统一的口径文件：
`simulation/sandbox_scene.yaml` 的 `uwb_anchors`（原为 `[]`，按 4.2 的四点填入并带
`survey_status: simulation_only_not_surveyed`）、`simulation/gdr_net_dataset_v03.yaml`
（`uwb` 段加 `survey_status`；`targets` 段十个目标一律 `continuous_z` 的第三套对称口径
按 4.3 改正）。`simulation/generate_gdr_net_dataset.py` 只改了 `_draw_mesh()`，
加一个可选 `face_colors` 形参供 decoration 按面着色使用，默认行为不变。
`simulation/nexus_sandbox_imx219.world` 把 planar_move 的 `publish_odom_tf` 从
`false` 改为 `true`：这是 11.5 第一条缺口的真正阻塞点，仓库里没有任何其他
`map → base_link` 广播者，不打开则填好 TF 表也出不来画面。

**留给后续的已登记问题（不在本次范围内改）**

| 问题 | 证据 | 为什么这次不动 |
|---|---|---|
| `ground_truth/camera_pose_map.csv` 的列名 `R_map_camera_<r><c>` 与内容相反：存的是 `map → camera`（即 `R_camera_map`） | 四个生成器（`generate_target_detection_dataset.py`、`generate_orb_slam2_dataset.py`、`generate_orb_gdrn_dataset.py`）与 `fuse_orb_slam2_uwb.py` 用的是同一套（错的）名字，所以数值上**一致、没有活的 bug** | 改名会同时波及四个生成器、三个消费者和已冻结的 E007 记录，属 `AGENTS.md:11` 禁止的顺手重构。本次只在生成器写入处、`dataset_manifest.json` 的 `camera_rotation_convention` 键、以及 `plot_pose_results.py` / `inject_degradations.py` 的 docstring 里写清实际约定 |
| `experiments/runs/` 有两个 E007 | `2026-08-27_E007_orb_slam2_mono_simulation` 与 `2026-08-30_E007_lightglue_sandbox` | 历史目录不改写（`AGENTS.md:12`），只登记 |
| `code/carla_bridge/server.py` 四个 WebSocket 端点无鉴权、`allow_origins=["*"]` | 见 11.6；`code/carla_bridge/.venv/` 还是未跟踪的虚拟环境目录，按 `AGENTS.md:14` 不应进 Git | 11.6 已声明这是本方案范围外的独立问题；处置步骤写在 `defense/demo/2026-08-29_checklist_demo_recording.md` 的安全提醒一节 |
| `/nexus/algorithm_route` 只在文档里存在，无发布者/订阅者，`simulation/algorithm_routes.yaml` 的 `default` 全为 `null` | 11.1 把它写成 IMPLEMENTED，实际不是 | 11.4 第 3 步（三链路各录 60 s）因此暂不可执行，已在录屏清单里标为必须先补的缺口 |

---

## 附二：沙盘变更为 CARLA RRD 后的受限条件优化（2026-09-01 修订）

本方案 0–16 节成文时的实验对象是参数化沙盘（`4.0 m × 4.7 m`、十个参数化代理目标、
`3280×2464`、相机高 `3.05~3.65 m`）。**该前提已作废。** 当前沙盘是 CARLA 的 RRD 地图
（RoadRunner Dataprep，启动参数 `/Game/CustomMaps/roadrunne3/RRD`），
可用数据是 500 帧同步采集。口径变更由 ADR-010 裁定，本节只做方案层面的重算。

### 附二.1 新条件与旧条件的差别（IMPLEMENTED 事实）

| 项 | 参数化沙盘（0–16 节假设） | RRD 现状 |
|---|---|---|
| 目标数 | 10 个参数化代理体 | **3 个**（CARLA vehicle actor，物理关闭） |
| 目标几何 | 有网格与 `models/*.ply`、有 `base_z_mm` | **无网格、无 CAD、无标称高度** |
| 逐像素监督 | 实例掩膜 + 稠密 XYZ + 深度 | **只有图像 bbox** |
| 数据布局 | BOP（`scene_gt.json` / `mask_visib` / `xyz_crop`） | jsonl（`vision_target_observations.jsonl` 等） |
| 图像 | `3280×2464`，`fx = 1978.89` | `640×480`，`fov 90°` ⇒ `fx = 320.0` |
| 相机距离 | `3.550 m`（均值） | 约 `14.6 m`（平台 `+15 m`，目标 `+0.4 m`） |
| UWB | 4 锚点，`sigma 0.015 m`，作段级基准 | 4 锚点，`sigma 0.030 m`，**用于无人机自身定位** |
| 数据位置 | 本机可生成 | 云机（`/root/autodl-tmp/...`），本机不可访问 |

**直接后果：GDR-Net 的稠密对应无法在这批数据上训练**——`PM_LW`/`REGION_LW`/`PM_LOSS_SYM`
（L0-2）需要掩膜、稠密 XYZ 与模型点，三者都不存在。E014 在 RRD 数据上不成立。

### 附二.2 因子集在受限条件下的存活情况

这正是 0 节"把深度从网络回归量搬到几何求解量"的回报：**bbox 中心本身就是一条视线**，
所以深度主通道不依赖掩膜和网格，换沙盘后不需要换方法。

| 因子 | 是否存活 | 依据 |
|---|---|---|
| **F1** 多视图视线 | ✅ 存活，退化为 bbox 中心视线（QuadricSLAM 式，2 节已列为兜底通道） | 只需 bbox + 相机位姿 |
| **F2** 轮廓区域对应线 | ❌ **不存活** | 需要目标网格 + 掩膜，RRD 两者都无 |
| **F3** 支撑面 | ✅ 存活，但支撑高度与目标标称高度降级为**声明的设计先验**（`config/rrd_target_priors_v01.yaml`），不是实测 | 目标停在路面上 |
| **F4** 场景控制点 | ✅ 存活（XFeat 只需图像与控制点，不需目标网格） | 未实测，见附四 |
| **F5** 帧间相对位姿 | ✅ 存活（ORB-SLAM2 只需图像序列） | 未接入，见附四 |
| **F6/F7** UWB 基准与平台先验 | ✅ 存活（RRD 采样器给原始测距） | 锚点是仿真设计值 |
| **F8** 弱先验 | ✅ 存活，先验来源由目录改为 RRD 目标先验文件 | — |
| F1 的**稠密权重** | ❌ 不存活 | 需要 GDR-Net 稠密对应，见附二.1 |

**必须同时承认的一条**：bbox 中心只给一条视线，**不含姿态信息**。因此在 RRD 数据上
**目标旋转不可观测**，必须写 NaN + `degraded_reason`，不得报 `rotation_rmse_deg` /
ADD-S 旋转项（ADR-006 的"无效即 NaN、不复用上一帧"同一口径）。10.3 对比表在 RRD
条件下只填位置类指标。

### 附二.3 采集几何重算：`2 cm` 在**当前** RRD 配置下不可达（本节是本次修订最重要的结论）

12 节的误差预算建立在 `1.79 mm/px` 的地面采样距离上。RRD 现状配置差 25 倍：

| 量 | 参数化沙盘 | RRD 现状 |
|---|---:|---:|
| `fx` | `1978.9 px` | `320.0 px` |
| 相机距离 | `3.550 m` | `14.6 m` |
| 地面采样距离（GSD） | `1.79 mm/px` | **`45.62 mm/px`** |
| 倍数 | 1× | **25.4×** |

bbox 中心的视线噪声按 `4 px` 估（框中心不是三维包围盒中心的投影，还有透视偏置），
N=8 视图平均后等效 `1.41 px`：

```text
侧向 sigma ≈ 1.41 px × 45.62 mm/px ≈ 64.5 mm
```

**这已经超过已记录需求的"视觉主导 ≤5 cm"，更不用说 `2 cm` 的 TARGET。**
瓶颈不是算法，是采集配置：`640×480 + fov 90° + 14.6 m` 这一组合本身不支持厘米级。

候选配置（TARGET，闭式，按上式重算；深度项需按 RRD 实际基线重推，暂记 TBD）：

| 配置 | 宽 | fov | 高度 | `fx` | GSD | 侧向 sigma |
|---|---:|---:|---:|---:|---:|---:|
| **A 现状** | `640` | `90°` | `14.6 m` | `320.0` | `45.62 mm/px` | `64.5 mm` ❌ |
| B | `1280` | `60°` | `14.6 m` | `1108.5` | `13.17 mm/px` | `18.6 mm` ❌ |
| C | `1920` | `60°` | `8.0 m` | `1662.8` | `4.81 mm/px` | `6.8 mm` ⚠ |
| **D 建议** | `1920` | `60°` | `5.0 m` | `1662.8` | `3.01 mm/px` | `4.3 mm` ✅ |
| E | `3280` | `60°` | `14.6 m` | `2840.6` | `5.14 mm/px` | `7.3 mm` ⚠ |

结论三条，缺一不可：

1. **重采一次数据**：按配置 D（或 E，若必须保持 `15 m` 巡飞高度则接受 `7.3 mm` 侧向）
   重新采集，`simulation/carla/rrd_uav_uwb_dataset_v01.yaml` 的 `width/height/fov` 与
   平台高度必须一起改。500 帧的**帧数**够用，**分辨率与高度不够**。
2. **在现状 A 配置的数据上，只能验证链路能跑通与相对改善**，不得报绝对精度达标；
   任何 `≤5 cm` / `2 cm` 的表述在配置 A 上都不成立。
3. 12 节的四项系统误差（网格 `8 mm` / 控制点 `5 mm` / 外参 `5 mm` / UWB 基准 `10 mm`）
   在 RRD 上**更差**：目标无网格（标称高度只能声明）、控制点未 survey、
   UWB `sigma` 由 `0.015` 变 `0.030 m`。所以即便换到配置 D，`2 cm` 仍只是 TARGET。

### 附二.4 受限条件下的链路配置（不新增求解器）

三条链路仍共用同一个求解器，只是 RRD 条件下的因子集按附二.2 收缩：

| 链路 | RRD 条件下的因子集 | 与 7/8/9 节的差别 |
|---|---|---|
| A 精度 | F1(bbox) + F3 + F4 + F5 + F6 + F8，F7 弱权重 | 去掉 F2；两轮调度的第二轮不再加 F2，改为加 F4/F5 |
| B 鲁棒 | 同 A，另加通道一致性（F1 视线 ‖ F3 支撑面 ‖ UWB 兜底三通道） | 冗余通道由"稠密‖轮廓‖框级‖UWB"减为"框级‖支撑面‖UWB"，误报率阈值需重标 |
| C 自适应 | 同一门控函数，`n_views/baseline/bbox_area/uwb_residual` 仍可算 | `sigma_from_epro_pnp` 与 `blur/exposure` 在 RRD jsonl 里没有来源，门控需退化为可算子集 |

---

## 附三：在线增量定位（机载 C6，2026-09-01 新增需求）

> **本节的分层方案已由附五取代（2026-09-02）**：附三写于实机硬件实测之前，把平台端
> 假设成 UWB + 飞控姿态；实测证明飞控给的是 `188.9 Hz` 原始 IMU 且**不推融合姿态**，
> 平台端应整体采用现成的 UWB-aided VIO。本节的因子接口、100 ms 预算纪律、
> 「只有 valid 才更新携带状态」等约束**仍然有效**。

0–16 节只交付批处理求解（`3.2` 的 C5：360 实例全解 `<60 s`）。本次新增要求：
**数据由无人机实时采集，定位在线出结果**，即把 `3.2` 的 C6
（`≤10 Hz` 更新、`≤1` CPU 核）从"可选"提为必做。

### 附三.1 设计（不新增因子类型、不新增网络、不新增依赖）

| 项 | 取值 |
|---|---|
| 形态 | 固定滑窗增量求解，窗口 `W` 帧（默认 8） |
| 状态量 | `6 × 目标数 + 6 × W + 4`（段级尺度与偏置），仍在 C4 的 `≈300` 参数量级 |
| 窗口外信息 | **按 F8 弱先验携带**：用上一窗口的目标位姿与其边缘 sigma（乘一个膨胀系数）构成先验因子，不新增因子种类，也不把旧证据直接丢掉 |
| 每帧预算 | `100 ms`（10 Hz），超时记 `within_budget=false`，不静默拖慢 |
| 迭代上限 | 硬上限 `max_nfev`（默认 30），且**关闭**批处理的两轮阶段调度与外点二次求解——否则单帧代价无界 |
| 状态更新纪律 | **只有 `valid` 的解才更新携带状态**，被判无效的帧不得悄悄污染下一窗口的先验 |
| 输出 | 仍走 4.5 的记录契约，`n_views` 记窗口内视图数、`baseline_m` 记窗口基线 |
| 门控 | 复用 9 节的 `select_chain`；`transform_available=false` ⇒ 发 `transform_unavailable`，不发猜测位姿（ADR-006） |

**为什么滑窗而不是 EKF/因子图库**：3.3 已裁定不引入新依赖（C3）。滑窗 +
`scipy.optimize.least_squares` 复用的是同一份 F1–F8 代码，A/B/C 三条链路的配置文件
也不用改，只是把 `stages` 与 `second_pass` 关掉。

### 附三.2 与批处理的关系（必须并列报，不能只报一个）

| 指标 | 批处理（C5） | 在线（C6） |
|---|---|---|
| `translation_rmse_3d_m` | TBD | TBD（预期略差：窗口内视图少、基线短） |
| `update_rate_hz` | 不适用 | TARGET `≥10` |
| `latency_mean_ms` / `latency_p95_ms` | 段级耗时 | 逐帧耗时，`p95` 必须 `≤100 ms` |
| `availability` | 无效帧计入分母 | 同口径，另记 `within_budget` 比例 |

在线结果**不得**用批处理的数字代替，反之亦然；两者是不同的可用性口径
（`docs/算法比较定义参数.md:350-357`）。

### 附三.3 实时数据来源

两条来源都要支持，且必须在记录里标明用了哪条：

1. **仿真在线**：CARLA RRD 采样器边采边喂（jsonl 追加），平台位姿由 UWB 多边定位给出，
   **不读 CARLA 真值**；真值只在评估时用。
2. **实机在线**：树莓派只读适配器（`code/deployment/raspberry_pi_uav_readonly_adapter/`）
   输出的 JSONL。ADR-010 已确认厂商通过 `BATTERY_STATUS.voltages[2:6]` 暴露四个基站
   原始测距（单位 cm，`0`/`65535` 为不可用哨兵），所以 F6 在实机上有真实输入；
   但厂商位置只有 `x/y`、没有 `z`，只能作 F7 弱先验，高度必须另有来源。

---

## 附四：未实现清单（核对时间 2026-09-01，逐项在工作树里核对过）

### 附四.1 已落地

0–16 节的落地清单（含附的两张表）已全部有对应实现：
`code/analysis` 54 个测试通过，ROS2 `colcon test` 51 个通过、0 失败（6 skipped 为
`apriltag_ros` 的 cppcheck，与本方案无关）。附二/附三新增部分的落地情况：

| 文件 | 行数 | 状态 |
|---|---:|---|
| `code/analysis/localization/online_solver.py` | 163 | ✅ 附三.1 的滑窗求解器已实现（含 F8 先验携带、100 ms 预算标记、只有 valid 才更新状态） |
| `code/analysis/localization/rrd_episode.py` | 237 | ✅ RRD episode → 因子适配器已实现（含 CARLA 左手系 → `map` 右手系换算） |
| `simulation/carla/rrd_uav_uwb_dataset_v01.yaml` | 84 | ✅ 采样参数已从脚本里抽成配置 |
| `code/deployment/raspberry_pi_uav_readonly_adapter/` | 362 + 测试 + README | ✅ 实机只读适配器已进 Git |

### 附四.2 破损项（必须先修，当前工作树里是坏的）

| 项 | 证据 | 影响 |
|---|---|---|
| `simulation/carla/uav_uwb_mono_sampler.py` 在第 519 行被截断 | `python3 -m py_compile` 报 `SyntaxError: expected 'except' or 'finally' block` | RRD 数据采集脚本**当前无法运行**。原始可运行副本仍在 `/mnt/d/CARLA_0.9.16/uav_uwb_mono_sampler.py`（未修 bug 的版本） |
| ADR-010 悬空链接 | `docs/decisions/README.md` 与 `docs/PROJECT_INDEX.md` 已各写一条指向 `2026-09-01_ADR-010_*.md` 的链接，但该文件不存在 | 两处文档链接指向空文件 |

### 附四.3 未实现清单

| 编号 | 未实现项 | 为什么要紧 | 前置 |
|---|---|---|---|
| U1 | 补完并修好 `simulation/carla/uav_uwb_mono_sampler.py` | 见附四.2；同时要修掉已定位的 8 条 bug（下表） | — |
| U2 | `simulation/carla/validate_uav_uwb_dataset.py` | 数据集校验器。现有 `/mnt/d` 版本的图像检查只查 `std > 1.0`，抓不住"相机拍到自身车体网格" | U1 |
| U3 | `simulation/carla/README_rrd_uav_uwb_dataset.md` | RRD 数据集契约文档，必须写清"只有 bbox、不能训练 GDR-Net 稠密对应" | U1 |
| U4 | `config/rrd_target_priors_v01.yaml` | F3 支撑面需要声明的目标标称高度与支撑高度。**没有它 `rrd_episode.py` 跑不起来** | — |
| U5 | `code/analysis/localization/run_online_cli.py` | 在线定位只有类、没有入口，无法接实时流（附三.3 的两条来源都没接） | U4 |
| U6 | `code/analysis/test/test_rrd_uav_uwb_dataset.py` 与在线/RRD 适配器的测试 | 坐标系换算、近平面裁剪、DOP、滑窗预算目前**一条回归测试都没有**；坐标系搞反和定位失败在数字上分不出来 | U1, U4 |
| U7 | `docs/decisions/2026-09-01_ADR-010_*.md` | 沙盘换 RRD、三套锚点分环境、原始 UWB 测距在实机可用（推翻 ADR-001 警告）、厂商位置只有 `x/y`、ADR-009 第 5 项裁定范围收窄——这些口径变更按 `AGENTS.md:12` 必须走决策记录 | — |
| U8 | 按附二.3 的配置 D/E 重采一次 RRD 数据 | 现状 `640×480 + fov 90° + 14.6 m` 的 GSD 是 `45.62 mm/px`，侧向 sigma `64.5 mm`，`2 cm` 与 `≤5 cm` 都不可达 | U1, U2 |
| U9 | `carla_status_client.js` 的端口冲突 | 状态流被改到 `8765`，与静态页服务器同端口（runbook §9 是静态 `8765` / 状态桥 `8766`） | — |
| U10 | 锚点 survey + 用适配器录一段实机 JSONL 建立运行记录 | 有原始测距不等于能出坐标；且"UWB 已校验"目前只是控制台演示，`experiments/runs/` 无记录，按 `AGENTS.md:35` 不得表述为已验证 | — |
| U11 | 附五.10 的 S0：从 `fanciswarm_bridge.py` 的 UDP 流被动抓包，确认 `GLOBAL_VISION_POSITION_ESTIMATE` 的 `z`/姿态是否有效 + 静置 IMU 噪声实测 | 决定要不要自研 AHRS；VIO 噪声参数必须实测填，照抄默认值不收敛 | — |
| U12 | S1：单位换算层（加速度 `mm/s²`）+ 时间同步；适配器扩输出 `SCALED_IMU` | 现有适配器只处理 4 类消息，**不含 IMU**，所以在线 VIO 现在拿不到数据 | U11 |
| U13 | S3：拉 `uvio` / `VIR-SLAM`，核 LICENSE + aarch64 编译；否则退回 OpenCV 单应平面 VO | L1 平台端唯一的段内相对几何来源 | U12 |
| U14 | **S4 关键否证**：VIO 接进 F5，跑附五.9 的消融 | 深度误差应从 `≈50 mm` 掉到 `10~15 mm`；掉不下来必须停下重推（ADR-008） | U13 |
| U15 | S5：`code/analysis/vision/online_frontend/`（亚像素块跟踪 + 检测/跟踪仲裁），检测器降到 1–2 Hz | 把 `σ_px` 从 4 px 压到 `<1 px`；仓库里一行都没有 | U12 |
| U16 | `config/chain_d_online_visual.yaml` + `router.py` 注册 `vision.bundle_d` | 第四条链路的入口，可与 A/B/C 并列对比 | U14 |
| U17 | 相机切 `1640×1232 @ 41.85 fps` 重采（附五.3）；C3/C6 的放宽走一条 ADR | RRD 现状 `640×480` 是裁切模式，同时吃低分辨率与窄视野两个亏 | — |

### 附四.4 U1 要修的 8 条 bug（已在 `/mnt/d/CARLA_0.9.16/uav_uwb_mono_sampler.py` 上定位，行号指原文件）

| 编号 | bug | 严重性 |
|---|---|---|
| B1 | `:311` UWB 解算的初值就是无人机**真实位置** `[loc.x, loc.y, loc.z]` ⇒ 违反"算法核心不得读取仿真真值"，`position_error_m` 变成乐观的最好情况 | 高 |
| B2 | `:267` 下视相机装在 vehicle actor 的**车体原点**（`Location(z=0.0)`，`pitch=-90`）⇒ 相机在自身网格内部朝下，图像很可能是废的 | 高 |
| B3 | `:275` 注释写 `non-coplanar-in-height anchors enable full 3-D`，但**从未核算过几何**。真正的 bug 是「断言几何而不测量」，不是「共面就一定差」——修完加上 `dop_metrics()` 实测后，结论与原先的直觉相反：`(2,2,3,2)` 共面 @±20 m 的 `PDOP=1.638 / cond=1.57`，而把 z 拉开成 `(1.5,20,6,13)` 反而变差到 `PDOP=2.184 / cond=2.72`（把某个 anchor 抬到接近 tag 高度时它的视线 z 分量趋零，对高度不提供信息）。最终布局按实测选为 `(±13,±13,1.5)`，`PDOP=1.503 / cond=1.07`，`σ_range=0.03 m` ⇒ 位置 `σ≈45 mm`。**这 45 mm 正是附五.2「逐帧 UWB 不能定基线」的来源。** | 高 |
| B4 | `:306` UWB 真距用命令的 `loc`、`:300` 位姿真值用 `actor.get_transform()` ⇒ 同一帧两套"真值" | 中 |
| B5 | `:249-253` 目标 spawn 失败就静默退到任意 `spawns[1:]`，"三个目标都在下视视野内"的前提被悄悄破坏 | 中 |
| B6 | `project_vertices` 丢弃相机后方顶点而不做近平面裁剪；且先把坐标夹到图像边界再判 `visible` ⇒ 出画目标可能得到退化但"可见"的框 | 中 |
| B7 | `write_jsonl` 每行重开文件 | 低 |
| B8 | 校验器只查 `image.std() > 1.0`，且把 `500/1500/2000` 写死（用户已明确"输出编号不要很在意"） | 中 |

### 附四.5 本机无法验证的项（不得表述为已验证）

| 项 | 原因 |
|---|---|
| RRD 采样器端到端 | 本机无 `carla` 模块、无 CARLA 服务端 |
| 500 帧数据集本体 | 数据在云机 `/root/autodl-tmp/NEXUS/outputs/uav_uwb_mono_rrd/`，WSL 与 D 盘都没有 `episodes` 目录；用户表示之后放到本机或树莓派 |
| GDR-Net 训练侧改动（L0-2/3/4） | 本机无 `torch`、无 `mmcv`，只过了 `py_compile` |
| 端到端画面（RViz 三个新 Marker、前端帧率） | 本机无 Gazebo/CARLA 运行环境 |
| 在线定位的 `≤10 Hz` / `≤1` CPU 核（C6） | 需要在树莓派 5 上实测，未做 |
| 实机 UWB 定位精度 | 锚点未 survey，无运行记录 |

---

## 附五：在线视觉-惯性-测距融合定位框架（`chain_d`，2026-09-02 定稿）

> **独立成文版见 [`2026-09-02_design_online_target_localization_framework_v01.md`](2026-09-02_design_online_target_localization_framework_v01.md)**：那份文档给出完整的坐标转换公式（LaTeX）、输入获取方式、输出契约、八个因子的残差式、开源件清单，以及 2D/3D 的蒙特卡洛实算与灵敏度分析。本附五保留为优化方案内部的摘要与勘误记录。

本节是"在线算法框架"的定稿设计，**取代附三的分层方案**：附三写于实机硬件参数实测之前，
把平台端假设成 UWB + 飞控姿态；实测证明飞控提供的是 `188.9 Hz` 的原始 IMU、
且**不提供**融合姿态，因此平台端应整体采用现成的 UWB-aided VIO，而不是自研递推层。
附三的因子接口、100 ms 预算纪律、"只有 valid 才更新携带状态"等约束仍然有效。

设计原则（用户 2026-09-02 明确要求）：**优先拼装成熟开源件，不自研新算法。**
自研范围压到"胶水"：时间同步、单位换算、线程仲裁、因子构建、记录落盘。

### 附五.1 实机硬件基线（MEASURED，2026-09-02，`ssh pi@192.168.1.143` 实测）

机载计算：

| 项 | 实测值 |
|---|---|
| 机型 | Raspberry Pi 5 Model B **Rev 1.1** |
| OS / 内核 | Debian 12 bookworm / `6.12.25+rpt-rpi-2712`（aarch64） |
| CPU | **4× Cortex-A76**，max `2400 MHz`，min `1500 MHz`，scaling 100% |
| 内存 | `7.9 GiB` 总 / **`5.3 GiB` 空闲** / swap `511 MiB` |
| 温度 / 限频 | `63.7 °C`，**`throttled=0x0`（无限频）** |
| Python | `3.11.2`，`~/mavlink_env` 内 pymavlink `2.4.49` |
| 串口 / I2C | `/dev/ttyAMA0`、`/dev/ttyAMA10`；`i2c-6/10/13/14` |

**内存不是瓶颈**：几何求解状态量约 73 个参数（KB 级）。约束是 4 核 CPU、无 GPU 无 NPU。

相机 IMX219 可用模式（`rpicam-hello --list-cameras` 实测），`fx` 由仓库确认的
`3280` 宽全幅 `fx = 1978.892939`（HFOV `79.3°`）按裁切/合并推得：

| 模式 | 裁切 | `fx` | HFOV | fps | GSD@8m | 地面宽@8m |
|---|---|---:|---:|---:|---:|---:|
| `3280×2464` | 全幅 | `1978.9` | `79.3°` | `21.19` | `4.04 mm/px` | `13.26 m` |
| `1920×1080` | 裁切 | `1978.9` | `51.8°` | `47.57` | `4.04 mm/px` | `7.76 m` |
| **`1640×1232`** | **全幅 2×2 合并** | `989.4` | **`79.3°`** | **`41.85`** | `8.09 mm/px` | **`13.26 m`** |
| `640×480` | 裁切 2×2 | `989.4` | `35.8°` | `103.33` | `8.09 mm/px` | `5.17 m` |

**只有两种角分辨率**：不合并 `fx≈1979`、2×2 合并 `fx≈989`。注意 `640×480` 不是
"低配全幅"，它是**裁切**模式，HFOV 只有 `35.8°`——RRD 采样器用的就是 `640×480`，
这解释了附二.3 里那个 `45.62 mm/px`：它同时吃了低分辨率和窄视野两个亏。

飞控实际推送的 MAVLink 消息（`/dev/ttyAMA0` @115200，被动监听 20 s，只发 GCS `HEARTBEAT`）：

| 消息 | 计数 | 速率 | 说明 |
|---|---:|---:|---|
| **`SCALED_IMU`** | 3779 | **`188.9 Hz`** | **IMU 可用**：加速度 + 陀螺 + 磁 |
| `GLOBAL_VISION_POSITION_ESTIMATE` | 200 | `10.0 Hz` | 厂商融合位置；消息本身带 `z` 与 `roll/pitch/yaw` |
| `GLOBAL_POSITION_INT` | 200 | `10.0 Hz` | 速度（cm/s） |
| `BATTERY_STATUS` | 20 | `1.0 Hz` | UWB 四路测距在 `voltages[2:6]`（ADR-010） |
| `HEARTBEAT` | 20 | `1.0 Hz` | — |
| `TIMESYNC` / `SYSTEM_TIME` | 1 / 1 | 偶发 | — |
| **`ATTITUDE`** | 0 | — | **飞控不推融合姿态** |
| **`OPTICAL_FLOW` / `OPTICAL_FLOW_RAD`** | 0 | — | **不推光流**，尽管硬件表写"光流定位 支持" |

样本：`xacc=-582, yacc=44, zacc=-9873, xgyro=-6, ygyro=3, zgyro=11, xmag=20685`

**四条必须写进接口文档的语义修正：**

1. **加速度单位不是 MAVLink 标准的 mG。** 模长 `≈9890`，`÷1000 = 9.89 m/s²` ≈ 重力
   ⇒ 厂商用 **`mm/s²`**。按规范当 mG 解会差约 **9.8 倍**，在 VIO 里会直接发散。
2. 陀螺与规范一致（`mrad/s`）：静止 `zgyro=11` → `0.63 °/s`，是合理零偏噪声。
3. 磁力计 `xmag=20685` 与 mgauss 对不上（会得 `2068 µT`，地磁只有 `~50 µT`），
   **语义未确认，暂不使用**。
4. `GLOBAL_VISION_POSITION_ESTIMATE` 在标准 MAVLink 里是伴随机**发给**飞控的视觉位姿输入，
   这里是飞控回传 ⇒ 厂商复用，语义待厂商确认（ADR-010 已裁定只作 F7 弱先验）。

**串口是独占资源（2026-09-02 实测踩到）**：排查期间 `/dev/ttyAMA0` 被
`fanciswarm_bridge.py --serial /dev/ttyAMA0 --udp-host 192.168.1.140 --udp-port 14550`
占用，导致第二次读取报 `multiple access on port`。因此定一条硬约束：

> **由一个进程独占串口，解析后经 UDP 分发给下游。在线定位适配器接 UDP，不得再抢串口。**

现有的 `fanciswarm_bridge.py` 已经在做这件事，在线链路应接它的 UDP 输出。

### 附五.2 §12 误差预算勘误：主误差是**基线**，不是像素

§12 只计了像素噪声项，漏了**基线本身的不确定性**。深度误差有两项：
`Z²σ_px/(fx·B·√N)`（像素）与 `Z·δB/B`（基线）。按配置 `Z=5 m`、`B=4 m` 重算：

| 基线来源 | 基线误差 | 深度误差 |
|---|---:|---:|
| **逐帧 UWB 位置差**（σ=30 mm） | `42.4 mm` | **`53.0 mm`** ❌ |
| 逐帧 UWB（σ=45 mm） | `63.6 mm` | `79.5 mm` ❌ |
| 段级尺度，段内 10 组测距 | — | `25.2 mm` ❌ |
| 段级尺度，段内 40 组测距 | — | `12.6 mm` ⚠ |
| 段级尺度，段内 100 组测距 | — | `8.0 mm` ✅ |
| 对照：像素项 `σ_px=4.0` | — | `5.3 mm` |
| 对照：像素项 `σ_px=0.3` | — | `0.4 mm` |

§7 那句"逐帧引入 UWB 平台噪声，`2 cm` 立刻不可能"由此有了精确数字。而 ADR-009 裁定
F6 只做**段级**——段级尺度是一个标量，**必须乘在某个来源提供的段内相对几何上**。
仓库里那个来源是 F5（帧间相对位姿），`RelativePoseFactor` 已实现但**没有生产者**。

> **这是一个结构性缺口，不是待优化项：没有 VIO/VO，段级 UWB 无处可乘。**
> 这就是"在线视觉算法"在本方案中是承重件而非可选件的原因。

### 附五.3 相机模式裁定：`1640×1232 @ 41.85 fps`（全幅 2×2 合并）

附二.3 曾建议"提高分辨率"，那是在只有 bbox（`σ_px≈4 px`）的假设下算的。一旦引入
**亚像素块跟踪**，分辨率就不再是约束（`Z=8 m`，`B=4 m`，`N=8`）：

| 模式 | 只有 bbox（`σ_px=4.0`） | **亚像素跟踪（`σ_px=0.3`）** |
|---|---:|---:|
| `1640×1232`（`fx 989.4`） | `22.9 mm` ❌ | **`1.72 mm`** ✅ |
| `3280×2464`（`fx 1978.9`） | `11.4 mm` ⚠ | `0.86 mm` ✅ |

两种角分辨率在跟踪条件下都是**亚毫米到毫米级**，像素项彻底退出主误差。
所以该优化的不是分辨率，**是视野和帧率**：

| 判据 | 为什么选 `1640×1232` |
|---|---|
| 全幅 `79.3°` HFOV | 8 m 高覆盖地面 `13.26 m`。RRD 三个目标间距约 12 m，**只有全幅模式能同时看到三个**；`1920×1080` 裁切只有 `7.76 m`，装不下 |
| `41.85 fps` | VIO 与块跟踪要高帧率；可抽帧到 10 Hz 喂捆绑 |
| `fx=989.4` 够用 | 见上表 |
| 数据量 | 2×2 合并后是全幅的 1/4，回传与 CPU 都轻（`≈7 Mbit/s` JPEG@10 Hz 量级，需实测） |

`3280×2464 @ 21.19 fps` 留作**地面离线高精度模式**与精度上界对照。
若必须保持更高巡飞高度或目标更分散，则改为拉开高度而不是换裁切模式——
裁切会同时损失视野，是最差的组合（RRD 现状 `640×480` 就是这个坑）。

### 附五.4 框架分层：只有一层进 10 Hz 硬预算

```text
L0  离线一次（地面）
    相机内参标定 · 锚点 survey / 在线自标定初值 · base_to_camera 外参 · 控制点 survey
    IMU 噪声参数（静置实测）· 厂商单位换算表（加速度 mm/s²！）

L1  高速率平台端（整体采用现成 UWB-aided VIO）
    SCALED_IMU 188.9 Hz ┐
    UWB 四路测距 1 Hz    ├─► 现成 UWB-VIO ─► 平台位姿 + 段内相对几何 + 尺度
    厂商位置/速度 10 Hz  ┘   (uvio / VIR-SLAM)   └─► F5(相对位姿) / F6(段级尺度) / F7(弱先验)

L2  10 Hz（唯一受 100 ms 约束的一层）
    图像 41.85 fps ─► 亚像素目标块跟踪 ─► F1 方位 ┐
                     F3 支撑面先验 ───────────────┼─► 滑窗捆绑 ─► 目标位置 + 协方差 + validity
                     L1 的 F5/F6/F7 ─────────────┘   (现有 online_solver.py)

L3  1–2 Hz 异步（可在地面）
    检测器：目标 ID / 重初始化 / 否决跟踪漂移
    LightGlue 或 XFeat + 控制点 ─► F4 场景锚定（独立于 UWB 的第二条绝对基准）
    锚点在线精化
```

**速率解耦是这个框架成立的关键**：把最贵的检测器从 10 Hz 挪到 1–2 Hz，
把精度担子交给便宜的几何组件（VIO + 块跟踪）。结果是**算力下降、精度上升**。

**角色反转（本框架的核心主张）**：

```text
错的分工： UWB 给平台位姿（逐帧）  →  视觉只贡献 4 个 bbox 数字
对的分工： 视觉给段内相对几何 + 亚像素方位  →  UWB 只给尺度和 map 基准（段级）
```

### 附五.5 组件清单：全部现成件（许可须按 15.3 接入前复核）

| 槽位 | 现成件 | 许可 | 仓库现状 |
|---|---|---|---|
| L1 平台端（主选） | `aau-cns/uvio`（UWB-aided VIO，OpenVINS 生态） | **待核** | 无 |
| L1 平台端（备选） | `MISTLab/VIR-SLAM`（Visual-Inertial-**Ranging** SLAM） | **待核** | 无 |
| L1 对照基线 | VINS-Fusion（含 ROS2 移植）/ OpenVINS | GPLv3 / 待核 ⚠ | 无 |
| L1 对照基线（已有） | ORB-SLAM2 | **GPLv3 ⚠ 分发边界** | **已是 submodule** |
| L2 亚像素跟踪 | OpenCV `TrackerCSRT` / `TrackerVit` + `findTransformECC` + `calcOpticalFlowPyrLK` | Apache-2.0 | **已装** |
| L2 后端滑窗 | 现有 `online_solver.py`（`scipy.optimize.least_squares`） | BSD | **已实现** |
| L2 后端（升级选项） | GTSAM `IncrementalFixedLagSmoother`（正规 Schur 边缘化） | BSD-3 | 无，**与 C3 冲突** |
| L3 检测器 | Ultralytics YOLO + NCNN 导出 | **AGPL-3.0 ⚠** | **已在用** |
| L3 场景锚定 | **LightGlue**（E007 已复现沙盘匹配）/ XFeat | Apache-2.0 | **已是 submodule** |
| UWB 解算 | 仓库自有 `uwb/multilateration.py`（闭式 + 精化，无真值入参） | — | **已实现** |
| 锚点自标定 | `INRoL/inrol_uwb_localization` / `TIERS/uwb-cooperative-mrs-localization` | 待核 | 无 |
| 机载遥测 | `code/deployment/raspberry_pi_uav_readonly_adapter/` | — | **已进 Git** |

**因子接口是可插拔边界**：`RelativePoseFactor`(F5) / `SegmentUwbFactor`(F6) /
`PlatformPoseFactor`(F7) 已经把平台端定义成一组因子。**平台端换实现不影响目标端**——
今天用 uvio，明天换别的，只要往这三个因子里灌数据。目标端（纯方位目标定位）
没有开箱即用的库，但我们已经写好了。

### 附五.6 100 ms 预算分配（TARGET，须在 Pi 5 上实测替换）

| 环节 | 核 | 预算 |
|---|---:|---:|
| 取帧 `1640×1232` + 缩放 | 0.3 | `8 ms` |
| VIO（IMU 预积分 + 特征跟踪 + 滤波） | 1.2 | `35 ms` |
| 亚像素目标块跟踪（3 目标） | 0.3 | `6 ms` |
| 滑窗捆绑（≈73 参数） | 1.0 | `25 ms` |
| 落盘 + 话题发布 | 0.2 | `5 ms` |
| **余量** | 1.0 | **`21 ms`** |

**3.2 的 C6「`≤1` CPU 核」必须放宽为 `≤3` 核**：那条是在假设机载不跑检测器与 VIO 时写的。
若采用分体部署（图像回传、地面跑检测器与 VIO），机载才可能回到 1 核。
实测 `63.7 °C` 且 `throttled=0x0`，散热有余量。

### 附五.7 修正后的误差预算（`1640×1232` @ 8 m，亚像素跟踪，TARGET）

| 项 | 来源 | 预算 | 可否随帧平均 |
|---|---|---:|---|
| 像素方位（`0.3 px`，N=8） | L2/F1 | `0.86 mm` | 是 |
| 深度（同上，B=4 m） | L2/F1 | `1.72 mm` | 是 |
| 段内相对几何（VIO 漂移） | L1/F5 | `~6 mm` | 部分 |
| 段级尺度（UWB，100 组测距） | L1/F6 | `8.0 mm` | 否 |
| 控制点 survey | L3/F4 | `5.0 mm` | 否 |
| `base_to_camera` 外参（未标定） | L0 | `5.0 mm` | 否 |
| 支撑面高度先验（RRD 无实测网格） | F3 | `TBD` | 部分 |
| **合成（RSS，不含 TBD）** | | **`≈12 mm`** | |

`2 cm` 有余量，但**余量全部来自 VIO 与亚像素跟踪**：只用 bbox ⇒ `22.9 mm`；
逐帧用 UWB 定基线 ⇒ `53 mm`。两者任一缺失都会爆预算。

### 附五.8 可观测性硬约束（写进实现，不许绕）

1. **旋转不可观测**（若只跟单个框中心）：只输出位置，旋转写 NaN + `degraded_reason`。
   跟踪目标上**多个**可区分块可使姿态部分可观测——这是不需要目标网格就能改善的唯一途径。
2. **尺度必须来自测距或 IMU 激励**：单目无尺度。UWB 段级尺度是主通道，IMU 激励是备份。
3. **`map` 基准 = 锚点坐标**：未 survey 就没有米制基准，是系统误差、不随帧数平均。
   要么 survey，要么纳入状态在线自标定。
4. **深度需要基线 ⇒ 轨迹是算法的一部分**：悬停时基线为零、深度不可观测。
   在线系统必须把"保持环绕"作为约束，不能停在目标正上方。

### 附五.9 消融验收：每个组件都要证明自己值得存在

不许"接上就算好"。逐条消融，同一测试集、同一真值、同一评价脚本：

| 实验 | 对照 | 判据 |
|---|---|---|
| 只有 F1(bbox) + F3 + F6 逐帧 | 基线 | 预期深度误差 `≈50 mm` 量级，用来验证附五.2 的推算 |
| **+ L1（VIO → F5）** | 上一行 | **深度误差应掉到 `10~15 mm`。掉不下来就停下来查，不要继续加东西**（ADR-008 `不通过更换网络假装解决`） |
| + L2 亚像素跟踪 | 上一行 | 侧向与方位残差明显下降；**实测 `σ_px` 必须报出来**，不许用 `0.3 px` 这个假设值交差 |
| + L3（F4 场景锚定） | 上一行 | 段间一致性、绝对基准误差 |
| 在线 vs 批处理 | — | `latency_p95_ms ≤ 100`、`update_rate_hz ≥ 10`、协方差 χ² 校准图（14 节） |

在线结果**不得**用批处理数字代替，反之亦然：两者是不同的可用性口径
（`docs/算法比较定义参数.md:350-357`）。在线另需报 `within_budget` 比例。

### 附五.10 落地顺序（S0–S7）

| 步 | 内容 | 判据 / 否证点 |
|---|---|---|
| **S0** | 补齐硬件查询：从 `fanciswarm_bridge.py` 的 UDP 流**被动抓包**（不抢串口）确认 `GLOBAL_VISION_POSITION_ESTIMATE` 的 `z`/`roll`/`pitch`/`yaw` 是否有效；静置 60 s 测 IMU 噪声 | 姿态字段有效 ⇒ 省掉自研 AHRS；噪声参数进 L0 配置 |
| **S1** | 单位换算 + 时间同步层；扩适配器输出 `SCALED_IMU`（现在只输出 4 类消息、没有 IMU） | JSONL 里加速度 `÷1000` 后模长 ≈ `9.81` |
| **S2** | 相机切 `1640×1232 @ 41.85 fps`，录一段 IMU + UWB + 图像同步数据 | 三个目标同时在画面内；时间戳权威在采集时刻 |
| **S3** | 拉 uvio / VIR-SLAM，核 LICENSE + aarch64 编译 | 编得动且许可可接受；否则退回 OpenCV 单应平面 VO（下视近平面场景**不能用 5 点法**，必须单应分解） |
| **S4** | **关键否证**：VIO 接进 F5，跑附五.9 第二行消融 | 深度误差 `≈50 mm → 10~15 mm`。这一步花时间最少、否证力最强，**必须最先做** |
| S5 | OpenCV 亚像素跟踪 + 检测器降到 1–2 Hz + 跟踪/检测仲裁 | 实测 `σ_px` 能否 `<1 px` |
| S6 | LightGlue 接 F4；锚点在线精化 | 段间一致性 |
| S7 | 视 S4/S5 结果决定是否引入 GTSAM 替换手写的 F8 先验携带 | 边缘化近似是否为瓶颈 |

**锚点 survey 与采集配置（附二.3 / 附五.3）是 S0 之前的前置**，跳过它们只能得到
"能跑但精度不达标"的系统。

### 附五.11 命名

既然是系统集成而非自研新算法，命名要如实：

| 层 | 名字 |
|---|---|
| 配置 | `config/chain_d_online_visual.yaml` |
| 注册表（`router.py`） | `vision.bundle_d`（与 `bundle_a/b/c` 同族，可并列对比） |
| 后端模块 | `code/analysis/localization/online_solver.py`（已存在） |
| 前端模块（待建） | `code/analysis/vision/online_frontend/`：`patch_tracker.py`、`detect_arbiter.py`、`vio_bridge.py` |
| 论文层表述 | **"UWB 段级定标的在线视觉-惯性-测距融合目标定位"**；方法学名写 **bearing-only target localization from a moving observer**，实现逐个列明所用现成件 |

**不要声称自研新方法。** 答辩表述为"系统集成 + 口径设计 + 可观测性分析"更稳，
也符合 2 节"逐篇取长补短、不以任何单篇为蓝本"的定位。

### 附五.12 待核与冲突（未核对前不得写入依赖，`AGENTS.md` + 15.3）

| 项 | 状态 |
|---|---|
| `uvio` / `VIR-SLAM` / `INRoL` 的许可与 aarch64 可编译性 | **待核**，未拉 LICENSE 前不写进依赖 |
| Ultralytics **AGPL-3.0** 对 WebSocket 网关的影响 | 学术答辩无碍；闭源交付有碍，需换检测器或买授权 |
| 厂商磁力计单位语义 | **待厂商确认**，暂不使用 |
| `GLOBAL_VISION_POSITION_ESTIMATE` 厂商语义 | 待确认（ADR-010 已限定只作 F7 弱先验） |
| 锚点坐标 | **仍未实测**。有原始测距 ≠ 有坐标 |
| **C3「不新增依赖」与引入 uvio / GTSAM 冲突** | 需一条 ADR 裁定。建议：**先用现有 scipy 版跑通拿数字，被证明是瓶颈再引入 GTSAM**，避免一上手就背 aarch64 编译风险 |
| C6「`≤1` CPU 核」 | 与机载跑 VIO + 检测器冲突，须放宽为 `≤3` 核或改分体部署（附五.6） |
| 在线链路的可用性口径 | 分体部署时链路进 `availability` 分母；丢链发 `INVALID`，不发猜测位姿（ADR-006） |

