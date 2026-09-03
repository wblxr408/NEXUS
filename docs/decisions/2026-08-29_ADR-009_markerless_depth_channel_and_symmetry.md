# ADR-009：无标记定位的深度观测通道与十目标对称群口径

- 日期：2026-08-29
- 状态：accepted
- 负责人：NEXUS 无标记定位主线
- 关联任务/运行：
  - 设计文档 [`docs/2026-08-29_design_markerless_target_localization_optimization_v01.md`](../2026-08-29_design_markerless_target_localization_optimization_v01.md)
  - 计划运行 E013–E020（序列见设计文档第 13 节）
  - 被本决定的口径影响的历史运行：E003、E005、E012
  - 上位决定：ADR-003（学习件边界）、ADR-006（观测契约与 `transform_unavailable`）、
    ADR-008（无标签纯视觉优先、不通过更换网络假装解决）

## 背景

E012（`2026-08-29_E012_target_catalog_clean_retrain`）是无标记位姿链路目前最好的一次运行，
其结论仍是"不是厘米级定位，也不应表述为精度达标"：外部 YOLO 检测框条件下平移 RMSE
`0.1124 m`、旋转 RMSE `35.92 deg`、ADD@10% 召回 `2.22%`；同时检测器已饱和
（mAP50 `1.000000`、recall `1.000000`），GT ROI 与 YOLO ROI 只差约 1 cm
⇒ **瓶颈在位姿级，不在检测**。逐样本重算显示：深度（沿光轴）占平移平方误差的
`94.4%`，十目标深度偏差**全为负**（均值 `−71.3 mm`），是系统性偏差而非噪声。

**可追溯性声明（诚实记录）**：本仓库当前工作树中 `experiments/runs/` 下没有
`2026-08-29_E012_target_catalog_clean_retrain` 目录，`outputs/tables/tbl_e012_pose_errors_v01.csv`
与 `tbl_e012_pose_by_target_v01.csv` 也不存在（`outputs/tables/` 仅有 `.gitkeep`）。
上述数字的来源是 `docs/优化方案.md` 的记载，在本仓库内不可复算。本 ADR 裁定的是**口径**，
口径的正确性不依赖这些数字；但这些数字在进入答辩材料前必须先补回可追溯的运行目录
（`AGENTS.md:7`）。

用户要求把 UWB + 视觉的目标定位精度做到 `2 cm` 量级，同时要求算力配置很低，并要三条
可单独运行、可并列对比的链路（精度 / 对抗与退化鲁棒 / 二者结合）。要走这条路，
有六项**口径**必须先裁定——它们都会改变数据集字段、指标含义或结论表述方式，
按 `AGENTS.md:12` 不能在代码里静默改：

1. 深度的观测通道到底是什么。
2. 十个目标的真实对称群（目录现全写 `symmetry: none`，其中八项有误）。
3. `code/analysis/evaluation/pose_metrics.py` 认哪个对称键名（旧实现只认
   `symmetry == "continuous_z"`，而目录从不发这个键 ⇒ ADD-S 分支从未触发）。
4. 相机高度以哪个文件为准（`imx219_gazebo_camera.yaml` 的 `spawn_pose_m: [2.0, 2.35, 2.5]`
   与 `target_detection_dataset_v01.yaml` 的 `camera_height_m: [3.05, 3.65]` 不一致）。
5. UWB 锚点坐标的统一口径（仓库存在多组互不一致且都未实测的锚点，
   `simulation/sandbox_scene.yaml:138` 的 `uwb_anchors: []` 还是空的），以及 UWB 在链路中的角色。
6. `2 cm` 这个数字的标签口径（是需求还是目标？对仿真还是对实物？）。

## 候选方案

| 项 | 候选 A | 候选 B | 候选 C |
|---|---|---|---|
| 深度通道 | 继续用视像大小反解（网络回归 z），把训练配置调好 | 换更强的位姿网络 / render-and-compare | **运动基线交会（F1）+ 支撑面几何（F3）**，网络不再输出 z |
| 对称群 | 保持 `symmetry: none`，只在评估脚本里特判 | 只写人可读字符串（`continuous_z` 等） | **目录写结构化对称块，生成器展开为 BOP `symmetries_*`** |
| 指标对称键名 | 继续认旧 `symmetry` 字符串 | 同时认两套、以旧字符串优先 | **BOP `symmetries_*` 为唯一权威，旧字符串仅作缺失时退化判据** |
| 相机高度 | 以 `imx219_gazebo_camera.yaml` 的 `spawn_pose_m` 为准 | 两个文件各自成立、不统一 | **以 `target_detection_dataset_v01.yaml` 的 `camera_height_m` 为唯一口径** |
| UWB 锚点 | 每个脚本各取一组 | 统一坐标并把 UWB 接进逐帧目标深度 | **统一为 `gdr_net_dataset_v03.yaml` 一组 + 标注未实测；UWB 只做段级基准 F6 与兜底** |
| `2 cm` | 写成需求，答辩直接引用 | 只写"厘米级"、不给数字 | **标为 TARGET 且仅对仿真成立，实物不承诺** |

各候选的否决理由集中在「为什么这样选」。

## 决定

### 1 深度观测通道：由「视像大小反解」改为「运动基线交会（F1）+ 支撑面几何（F3）」

- 深度不再由任何网络回归。GDR-Net（及后续任何学习件）只提供 **2D–3D 稠密对应与逐点权重/
  不确定度**，其 z 回归输出在本方案的主链路中**不输出、不使用**。
- 深度的主来源是 F1 多视图视线交会（bearing-only），单视图或短基线时由 F3 支撑面
  射线求交补足；两者都是闭式几何，算力可忽略。
- 依据见设计文档 1.3(b) 与 1.4：`3.55 m` 处视像大小通道灵敏度是 `33.5 mm/px`，
  要拿到 1 cm 需要 `0.30 px` 的轮廓尺度精度，**不可达**；且该通道对目标模型尺寸误差
  **1:1 敏感**，而目标目录是 `designed_parameterized_proxy_without_cad` /
  `dimension_confidence: C`，这是不可约的计量依赖。替代通道在 1 px 视线噪声下为
  三角化 N=36 `1.10 mm`、N=8 `2.49 mm`、支撑面单视图 `2.59 mm`。
- 该裁定符合 ADR-003（学习件不得直接输出目标坐标，只能输出权重/残差修正/有效性/
  不确定度）与 ADR-008（`不通过更换网络假装解决`）：本决定不引入任何需要训练的新大网络，
  净新增可训练网络 = 0。

### 2 十目标对称群改正

按设计文档 4.3 表裁定十个目标的真实对称群：

| obj | class_name | 裁定的对称群 | `label` 取值 |
|---:|---|---|---|
| 1 | `vertical_striped_tank` | 绕 z 连续 | `continuous_z` |
| 2 | `horizontal_capsule_tank` | 绕物体 x 连续 + 绕 z 二重离散 | `continuous_x_and_z2` |
| 3 | `conical_roof_silo` | 绕 z 连续 | `continuous_z` |
| 4 | `triangular_hopper` | 无 | `none` |
| 5 | `hexagonal_column` | 绕 z 六重离散 | `discrete_z6` |
| 6 | `stacked_box_tower` | 绕 z 二重离散 | `discrete_z2` |
| 7 | `l_profile_factory` | 无 | `none` |
| 8 | `cross_valve_block` | 绕 z 四重离散 | `discrete_z4` |
| 9 | `tapered_frustum` | 绕 z 连续 | `continuous_z` |
| 10 | `twin_cylinder_unit` | 绕 z 二重离散 | `discrete_z2` |

字段格式：`simulation/target_catalog_v01.yaml` 的 `symmetry` 由字符串 `none`
改为结构化块，字段名固定为

```yaml
symmetry:
  label: <字符串>                                  # 人可读旧式写法，供旧消费者兼容
  continuous_axes: [[x, y, z], ...]                # 目标自身坐标系下的连续对称轴
  discrete_folds: [{axis: [x, y, z], fold: n}, ...] # 绕该轴的 n 重离散对称
```

- `label` 保留旧式人可读字符串（`none` / `continuous_z` / `continuous_x_and_z2` /
  `discrete_z2` / `discrete_z4` / `discrete_z6`），只作兼容与阅读用途，**不是权威**。
- 数据集生成器负责把该块展开为 BOP 标准字段写入 `models/models_info.json`：
  `symmetries_continuous` 为 `{axis, offset}` 列表；`symmetries_discrete` 为
  **16 元展平的 4×4** 齐次矩阵列表（BOP 规范），离散折数 n 展开 n−1 个非单位元。
- 上游 `lib/pysixd/misc.get_symmetry_transformations` 已能解析这两个字段，**不需要改
  GDR-Net submodule**。

对历史运行的裁定（`AGENTS.md:12`，不改写历史结论）：

- E003 / E005 记录里的 `continuous rotation about z; use ADD-S` 对 **obj1 / obj3 / obj9
  成立**，对其余七个目标**不成立**（obj2 的连续轴是物体 x 而不是 z；obj5/6/8/10 是离散对称；
  obj4/7 无对称）。
- `simulation/target_catalog_v01.yaml` v01 的 `symmetry: none` **全面失效**（十项中八项写错）。
- `simulation/gdr_net_dataset_v03.yaml` 把十个目标一律写成 `symmetry: continuous_z`，
  同样失效。
- **历史运行的结论文本不改写**；本 ADR 只声明其对称口径失效。E003/E005/E012 的旋转类指标
  与 ADD/ADD-S 召回在新口径下**必须重算**，重算结果记在新的运行目录（E013 起），
  不回填旧目录。

### 3 `pose_metrics.py` 的对称键名与指标分列

- **BOP `symmetries_discrete` / `symmetries_continuous` 是唯一权威**；旧 `symmetry`
  字符串仅在两个 BOP 字段都缺失时作为"是否对称"的退化判据（用于冻结的历史数据集）。
- 该口径已在 `code/analysis/evaluation/pose_metrics.py` 的 `symmetry_rotations()` 与
  `model_is_symmetric()` 中生效：`symmetries_discrete` 的 16 元项 reshape 成 4×4 取旋转块；
  `symmetries_continuous` 的每个轴按 **72 步**离散展开（`continuous_steps: int = 72`）；
  `model_is_symmetric()` 先看两个 BOP 字段，再退化到旧字符串。
- 指标分列裁定：**对称目标报 ADD-S 与对称最小化旋转误差；非对称目标报 ADD 与测地旋转误差。
  两者不得混入同一列**，也不得在同一张表里对不同目标混用两种定义而不标注。
  同一次评估必须能说明每个目标走的是哪一支。

### 4 相机高度口径

- **唯一口径**：`simulation/target_detection_dataset_v01.yaml` 的
  `platform.camera_height_m: [3.05, 3.65]`。定位数据集生成与**全部精度推导**都以它为准，
  包括设计文档 1.3(b)/1.4 的 `3.550 m` 距离均值（min `2.936`、max `4.077`）、
  `33.5 mm/px` 灵敏度、以及基于 36 个真实测试相机位姿的蒙特卡洛。
- `simulation/imx219_gazebo_camera.yaml` 的 `uav.spawn_pose_m: [2.0, 2.35, 2.5]`
  只是 **Gazebo 单次静态起飞点**，不是轨迹高度，**不得用于任何精度推导、误差预算或灵敏度计算**。
- 该文件**不改数值**（避免影响已有 Gazebo 起飞流程），只在本 ADR 中限定其用途；
  两个数字并存不再算矛盾，因为它们语义不同（起飞点 vs 采集高度区间）。

### 5 UWB 锚点坐标统一口径与 UWB 的角色

统一采用 `simulation/gdr_net_dataset_v03.yaml` 的 `uwb.anchors_m` 四点：

| 锚点 | 坐标 (m) |
|---|---|
| `anchor_0` | `[0.15, 0.15, 0.40]` |
| `anchor_1` | `[3.85, 0.15, 2.40]` |
| `anchor_2` | `[3.85, 4.55, 4.40]` |
| `anchor_3` | `[0.15, 4.55, 1.40]` |

同时统一 `range_noise_std_m: 0.015`、`range_bias_m: 0.0`、
`tag_to_base_translation_m: [0.0, 0.0, 0.0]`。

- 这组坐标是**仿真设计值、未实测**。凡传播这组坐标的文件与产物都必须带标注
  `survey_status: simulation_only_not_surveyed`，不得以任何形式表述为实测控制点。
- `simulation/sandbox_scene.yaml:138` 现为空的 `uwb_anchors: []` 按同一组坐标填入，
  并带上同一标注。此后**不允许任何脚本自取一组锚点**；改锚点必须新增 ADR。
- **锚点坐标是基准量（gauge），不是观测量**：UWB 只测距，一组距离要变成坐标必须先知道锚点
  在 `map` 中的位置；因此锚点误差是系统误差，不随帧数减小，会整体平移/旋转 `map`。
- **UWB 的角色被限定为两处**：段级尺度/基准因子 **F6**，以及视觉不可用时的**兜底通道**。
  **UWB 不进逐帧目标深度回路。** 依据设计文档第 7 节：逐帧注入平台噪声会带来
  `29.59 mm`（E004 46 mm 平台噪声）／`75.89 mm`（E008 114 mm）量级的误差，
  `2 cm` 立刻不可能。

### 6 `2 cm` 的口径

- `2 cm` 是 **TARGET**，**且仅对仿真成立**。它不是需求，也不是实测。
- 已记录需求仍然是：`核心演示区 ≤5cm（视觉可见时）/ 全场景兜底 <30cm`，
  分档 `视觉主导 ≤5cm / 过渡 8~15cm / UWB 兜底 15~25cm`，任务书表述为「厘米级」。
  本 ADR **不修改**这套需求。
- 实物 `2 cm` **不可承诺**，前提缺三项：目标无实测 CAD
  （`source: designed_parameterized_proxy_without_cad`、`dimension_confidence: C`）、
  控制点未 survey、`base_to_camera` 外参未标定（现为理想单位阵）。
  设计文档第 12 节的误差预算显示仅网格 8 mm + 控制点 5 mm + 外参 5 mm + UWB 基准 10 mm
  四项系统误差的 RSS 已是 `14.1 mm`，且这四项**不随帧数平均**。
- **禁止把仿真结果表述为实物精度**，禁止以"UWB 把无人机定准了"替代
  "沙盘目标 `map → target_link` 定准了"（`AGENTS.md:35`）。

## 为什么这样选

**深度通道（第 1 项）**：候选 A（调好训练配置继续回归 z）能拿到的收益是可算的——
仅扣除逐目标深度均值偏差就把平移 RMSE 从 `102.5 mm` 降到 `67.8 mm`，但**到不了 2 cm**，
因为 `33.5 mm/px` 的灵敏度和 1:1 的模型尺寸依赖是通道本身的性质，不是训练问题。
候选 B（换更强网络 / render-and-compare）直接违反 ADR-008，且算力档位跳到"大GPU-必需"，
与"低算力"要求冲突。候选 C 反而同时解决两个约束：几何求解几乎不花算力，
而被砍掉的 render-and-compare、GPU 级稠密匹配、每帧骨干前传恰好是最贵的部分。
这就是"高精度"与"低算力"不矛盾的根本原因。

**对称群与键名（第 2、3 项）**：不裁定的代价是指标本身没有意义——obj1/3/9 的 yaw
在几何上不可观测（且生成器从不读 `decoration`，连纹理线索都没有），
它们在 E012 里 `22.29 / 22.88 / 45.67 deg` 的旋转 RMSE 是**把噪声当误差在计分**；
obj10 的旋转 P95 `118.14°` 接近 180°，是二重对称被当成错误的典型特征。
只在评估脚本里特判（候选 A）会让"对称口径"散落在多个脚本里，且训练侧
`PM_LOSS_SYM` 与评估侧口径无法保证一致；只写人可读字符串（候选 B）无法表达
"绕物体 x 连续 + 绕 z 二重"这类复合群，也不被上游解析。选 BOP 标准字段是唯一
能让**训练损失、评估指标、求解器对称最小化残差**三处共用同一份定义的方案；
保留 `label` 是为了不破坏已按旧字符串消费该字段的既有代码。

**相机高度（第 4 项）**：精度推导必须锚在**采集高度分布**上，而不是一个起飞点。
把 `spawn_pose_m` 当高度会让 1.4 的灵敏度算错（2.5 m vs 3.55 m 差约 2 倍量级），
进而让整份误差预算不可信。不改 `imx219_gazebo_camera.yaml` 的数值是为了不动已有
Gazebo 起飞流程——矛盾的根源是语义没写清，而不是数字错。

**UWB（第 5 项）**：把 UWB 接进逐帧深度（候选 B）在数值上直接否掉 `2 cm`
（`29.59 mm` / `75.89 mm`）。而锚点是基准量：多组不一致的锚点意味着不同脚本其实工作在
不同的 `map` 里，任何跨脚本一致性检查都失去意义，所以必须统一到一组并集中管理。
统一后仍必须带 `survey_status` 标注，否则未来很容易把仿真设计值误当实测控制点使用，
那会直接污染实物结论。

**`2 cm`（第 6 项）**：把目标值写成需求是答辩材料里最容易犯、也最难挽回的错误。
标 TARGET 并限定"仅仿真"既保住了技术目标，也守住了 `AGENTS.md:7`
（规划中的目标值不是实测结果）和 `AGENTS.md:35`（不能把"代码已写"表述为"精度已达标"）。

## 验证计划与结果

验证按设计文档第 13 节的 E013–E020 序列执行，每项裁定都有对应的可判定验证：

| 裁定 | 怎么验 | 判定门槛 | 运行 |
|---|---|---|---|
| 1 深度通道 | 单独统计深度 RMS（沿光轴），只开 F1+F3 | 深度 RMS 从 `99.6 mm` 降到 `<10 mm`；否则 1.4 的推导有错，按停止条件立即停下重推 | E015 |
| 2 对称群 | 重生成数据集后检查 `models/models_info.json` 的 `symmetries_*`；用上游 `get_symmetry_transformations` 解析出的元素个数与裁定折数一致 | obj1/3/9 连续、obj2 连续+2 重、obj5 6 重、obj6/10 2 重、obj8 4 重、obj4/7 空 | E013 |
| 3 键名与指标分列 | 对已知对称目标构造"仅差一个对称元"的假估计，ADD-S 应≈0 而 ADD 明显非零；确认 ADD-S 分支真的进入（旧实现从未触发） | 对称目标全部走 ADD-S 分支；ADD 与 ADD-S 分列输出 | E013 / E014 |
| 4 相机高度 | 用数据集实际相机位姿反算距离分布，与 `camera_height_m` 区间一致 | 距离均值 `≈3.55 m`、min/max 落在 `2.9~4.1 m` | E013 |
| 5 UWB 口径 | 全仓库检索锚点坐标出现处，确认只有统一的一组且都带 `survey_status`；F6 只在段级出现 | 无第二组锚点；逐帧深度回路中不出现 UWB 项 | E013 / E018 |
| 6 `2 cm` 口径 | 检查文档与答辩材料中每处 `2 cm` 都带 TARGET 标注与"仅仿真"限定 | 无一处把 `2 cm` 写成需求或实测 | 全程 |
| 求解器整体 | 用 BOP 真值构造合成观测（加已知噪声），检查解出位姿与协方差统计一致（χ² 检验） | 通过 χ² 检验 | E015 / E018 |
| 坐标链 | `simulation/fuse_uwb_visual_targets.py` 的链式结果与新求解器对同一帧比对 | 差 `<1 mm`，否则外参/帧约定有 bug | E013 |

**当前结果（2026-09-01 对工作树逐项核对，诚实记录；实现仍在推进，状态以运行记录为准）**：

| 裁定 | 工作树中的落地状态 |
|---|---|
| 1 深度通道 | **求解器已在位**：`code/analysis/localization/` 下 `bundle_solver.py`、`factors.py`、`gating.py`、`chain_runner.py`、`contour_lines.py`、`observations.py`、`pose_distribution.py`、`xfeat_anchor.py`；**精度结果未测（TBD）** |
| 2 对称群写进目录 | **已落地**：`simulation/target_catalog_v01.yaml` 十行的 `symmetry` 已是 `{label, continuous_axes, discrete_folds}` 结构块并新增 `base_z_mm` / `support_deck_z_mm` / `symmetry_authority`；`generate_target_detection_dataset.py` 已有把该块展开为 BOP `symmetries_discrete`（展平 4×4，剔除单位元）/ `symmetries_continuous` 的代码，并按面着色渲染 `decoration`、把 `base_z_mm` 传给支撑面 |
| 3 对称键名 | **已生效**：`pose_metrics.py` 的 `symmetry_rotations()` / `model_is_symmetric()` 以 BOP 字段为权威，连续对称按 72 步展开，旧字符串仅作退化判据 |
| 4 相机高度 | 口径由本 ADR 确立；两个 YAML 的数值均未改动（按裁定不需要改，只限定用途） |
| 5 UWB 锚点 | **已落地**（2026-09-01）：`gdr_net_dataset_v03.yaml` 的 `uwb` 段加了 `survey_status: simulation_only_not_surveyed`；`simulation/sandbox_scene.yaml` 的 `uwb_anchors` 由 `[]` 改为同一组四点（mm 制，带 `survey_status` / `source` / `authority`）；该文件 `targets` 段十个目标一律 `continuous_z` 的第三套旧口径已按第 2 项改正。UWB 仍只出现在 F6 段级与兜底通道，不进逐帧深度回路 |
| 6 `2 cm` 口径 | 已在设计文档与本 ADR 中全程按 TARGET + 仅仿真表述 |
| TF 表（影响项） | **已落地**：`pre_hardware_transforms.yaml` 由 `transforms: []` 改为 `calibration_version: "1.1"`、`data_status: "simulation_only_pending_hardware_calibration"`、结构化 `map_definition`（带 `survey_status: simulation_only_not_surveyed`）、一条 `base_link → nexus_uav_camera_optical_frame` 静态变换（值来自 `nexus_sandbox_imx219.world`，标 `simulation_design_value_not_measured`），并把 `map → base_link` 单列为 `dynamic_transforms`（运行期动态量，不作静态外参发布） |
| 产图入口（影响项） | **已落地**：`code/analysis/evaluation/plot_pose_results.py` 已新建，六张图齐备 |
| `map → base_link` 广播（影响项） | **已落地**（2026-09-01）：`simulation/nexus_sandbox_imx219.world` 的 planar_move 插件 `publish_odom_tf` 由 `false` 改为 `true`（`odometry_frame: map`、`robot_base_frame: base_link`）。这是仓库里该边的唯一广播者；`pre_hardware_transforms.yaml` 的静态表刻意不含它，两者不会争同一条边。由 `nexus_bringup/test/test_gazebo_contract.py::test_simulation_broadcasts_the_single_map_to_base_link_edge` 锁定 |

**所有精度类结论一律 TBD**，本 ADR 不声称任何精度已达成
（`AGENTS.md:35`）。未落地项列为下面的后续动作。

## 影响、回滚和后续动作

### 附带裁定的两条补充口径

本次实现顺带确立以下两条口径，同样纳入本 ADR 的裁定范围：

**(i) `decoration` 只作为渲染外观。** `decoration`（条纹 `vertical_stripes`、端带
`end_bands`、顶环 `top_ring`、`roof_cap`、`light_cap` 等）**只按面着色实现，复用既有几何，
不加入模型网格**。因此 `models/*.ply` 的顶点集与 BOP 对称群保持一致——若把条纹压成几何，
obj1 的"绕 z 连续"就会退化成离散甚至无对称，`symmetries_continuous` 与网格自相矛盾。
代价是外观线索只能帮到基于图像的通道（F1/F2），不改变几何可观测性；连续对称目标的 yaw
仍然依赖 `decoration` 的**着色**提供纹理线索（设计文档 L0-6）。

**(ii) `base_z_mm` 进入目标目录。** 每目标底面高度进目录：obj1 `50.0`、obj2 `45.0`、
obj3 `30.0`、obj4 `40.0`、obj5 `45.0`、obj6 `35.0`、obj7 `40.0`、obj8 `45.0`、
obj9 `37.5`、obj10 `40.0`（单位 mm）。相对 `deck_height: 50` 有 `5–20 mm` 的下沉
（九个目标下沉，obj1 恰好齐平）。因此 **F3 支撑面因子必须读目录的 `base_z_mm`，
不得假设 `z_base = deck`**；否则单视图深度会被引入最大 20 mm 的常量偏差，
直接吃掉 `2 cm` 预算的一半。

### 影响

| 影响面 | 内容 |
|---|---|
| 数据集字段 | `simulation/target_catalog_v01.yaml` 的 `symmetry` 由字符串变结构化块并新增 `base_z_mm`；`models/models_info.json` 新增 BOP `symmetries_discrete` / `symmetries_continuous` |
| 指标含义 | 对称目标改走 ADD-S 与对称最小化旋转误差 ⇒ **E003/E005/E012 的旋转类与 ADD 类数字在新口径下不可与新运行直接比较**，跨口径对比必须标注 |
| 历史运行 | 结论文本不改写，只声明对称口径失效；重算结果落在 E013 起的新运行目录 |
| 网络角色 | GDR-Net 的 z 回归输出退出主链路；训练侧仍打开 `PM_LW` / `REGION_LW` / `PM_LOSS_SYM` 并用 `NUM_CLASSES=10`，但只为拿到更好的稠密对应 |
| UWB | 锚点坐标集中管理并带 `survey_status`；UWB 退出逐帧深度回路，只留 F6 段级基准与兜底通道 |
| 接口 | **不新增消息类型**：三条链路仍用 `SOURCE_PLATFORM_RELATIVE` / `SOURCE_FUSED`，差异记在 `covariance` 与 validity 上（ADR-006 不变） |
| TF | `code/ros2_ws/src/nexus_bringup/config/pre_hardware_transforms.yaml` 由空表（`transforms: []`、`data_status: tbd_until_hardware_calibration`、`map_definition: "TBD"`）改为填入**仅仿真**的 `map` / `base_link` / `camera` 变换；`data_status` 改为标注**仿真来源**，并明确**硬件标定仍待做**。填入的值只用于打通端到端画面与坐标链自检，**不得作为实物标定结果引用**（ADR-005/ADR-006 的"缺已验证变换就发 `transform_unavailable`"语义不变） |

### 回滚方式

若本 ADR 被推翻：恢复 `simulation/target_catalog_v01.yaml` 的 `symmetry: none`
与网络 z 回归（`Z_TYPE`/`Z_LOSS_TYPE` 原配置），撤回 BOP `symmetries_*` 导出，
并**重开一条运行记录**说明推翻的证据与新口径；**不改写任何历史运行**
（`AGENTS.md:12`）。`pre_hardware_transforms.yaml` 回滚为空表即可，无遗留状态。

### 后续动作

1. 第 2 项已落到 `simulation/target_catalog_v01.yaml` 与
   `simulation/generate_target_detection_dataset.py`；**仍需在 E013 里验证**导出的
   `symmetries_*` 元素个数与裁定折数一致，并确认 `_look_at()` 已放开 roll、
   目标朝向已多样化（L0-6/L0-7）。
2. 第 5 项已于 2026-09-01 补齐（见上表），剩下的是**实测**：锚点坐标至今是仿真设计值，
   实物阶段必须 survey 后替换，替换前不得出现任何实物精度表述。
3. `map → base_link` 已由 world 的 `publish_odom_tf=true` 广播，但**端到端画面尚未验证**：
   本机没有 Gazebo/CARLA 运行环境，`gzserver` 起链、RViz 三个新 Marker 的实际画面、
   浏览器帧率都只做了配置与代码层核对。另有一个更靠前的阻塞点：`/nexus/algorithm_route`
   在仓库里没有任何发布者/订阅者，`simulation/algorithm_routes.yaml` 的 `default` 全为
   `null`，所以设计文档 11.4 第 3 步「三链路各录 60 s」暂不可执行。
4. `code/analysis/evaluation/plot_pose_results.py` 的六张图已实现，但**从未跑过真实数据**：
   `outputs/tables/` 只有 `.gitkeep`，loader 目前只在合成 CSV 上验证过。
5. 按 E013–E015、E018–E022 逐项执行并回填设计文档 10.3 / 10.4 的表格，数字必须能追到运行号
   （原方案的 E013–E020 连号与已占用的 E016/E017 冲突，顺延理由见设计文档 13 节的编号勘误）；
   E012 的运行目录与 `outputs/tables/tbl_e012_*.csv` 必须补回，否则 1.1/1.2/10.1/10.2
   的数字不得进入答辩材料。
6. 设计文档 11.6 记录的 `code/carla_bridge/server.py` 无认证 WebSocket 网关
   （`allow_origins=["*"]`，四个端点无鉴权，README 指导 `--host 0.0.0.0`）属本 ADR 范围外的
   独立安全问题，需单独处理，不在本决定内裁定。

### 停止条件

引用设计文档 15.2，全文有效，触发即停并写进运行记录：独立真值验证未完成前不得写
"达到厘米级"；E015 若深度未显著改善则停止并重新推导 1.4，不得靠换网络掩盖（ADR-008）；
任何仿真结果不得表述为实物精度；学习件若被改成直接输出目标坐标即违反 ADR-003，回退。

