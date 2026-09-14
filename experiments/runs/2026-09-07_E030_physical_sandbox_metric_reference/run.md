# E030：实物沙盘米制参考重标定

## 状态

VALIDATED（运行真值）。用户已于 2026-09-07 接受本目录为项目的运行真值：
实测台面尺度、物体高度及道路宽度，加上注册布局的 XY，共同构成训练、匹配、
回放与评估的固定基线。XY 的图像/布局来源仍保留，和独立量尺 survey 不混淆。

## 输入与出处

- 用户于 2026-09-07 提供实物量尺：台面 `3.94 m × 4.94 m`；树、建筑、储罐、
  红绿灯高度及道路宽度见 `sandbox_physical_reference_v01.yaml`。
- `simulation/sandbox_scene.yaml`：历史照片反推布局，原始边界为 `4.00 m × 4.70 m`。
  其多数物体平面位置自身标注为 B/C 置信度，不能因缩放而提升为量尺真值。
- `uav_uwb_mono_rrd.tar.gz`，SHA-256
  `7759de118d35e834e3a8a03107bd5f927d55bfebc90ba5c15d9abdd27c17d052`：经包内
  `README.md` 与 `manifest.json` 核验，为 CARLA `CustomMaps/roadrunne3/RRD` 的
  500 帧仿真，只有三辆 CARLA 车辆目标，UWB 为 `sigma=0.03 m` 的合成数据。

## 标定结果

定义 `physical_sandbox_map_v01`：`+x` 沿台面宽 `3.94 m`，`+y` 沿台面长
`4.94 m`，`+z` 向上。历史布局到该候选地图的比例为：

```text
x_physical = 0.9850000000 × x_legacy
y_physical = 1.0510638298 × y_legacy
z_physical = measured height when an object is explicitly measured
```

道路宽度不随旧布局比例缩放：右侧远端道路为 `0.56 m`，其余道路为 `0.52 m`。
完整字段、物体量尺和边界条件见 `sandbox_physical_reference_v01.yaml`。

## 小物体展开

基于本次全景照片对中央工业区、外环道路、树带、斑马线与交通设施的可见一致性，
已新增可复现的展开器 `code/analysis/calibration/build_physical_sandbox_catalog.py`。它会输出：

- 每一条斑马线条纹与每一段车道虚线；
- 每一棵树的中心 XY；旧 `80 mm` 树列映射为用户量出的 `0.06 m` 矮树，旧
  `100/130 mm` 树列映射为 `0.12 m` 高树；
- 每一座交通灯、每一个锥桶和每一座储罐。

用户随后提供六个编号路口的近景。照片可辨认每个路口的四个角均有一高一低两个
红绿灯，故当前实物目录改为六个路口、48 盏灯：`J01..J06` × 四角 × `HIGH/LOW`。
`HIGH=0.22 m`、`LOW=0.12 m` 已进入
`traffic_light_photo_survey_v01.yaml`。旧布局的四组/16 盏交通灯不再输出为当前目录。

输出文件 `physical_sandbox_ground_truth_v01.json` 的所有 XY 已标注为运行真值；
它们保留图像/布局注册来源，可作为视觉匹配、训练、回放和项目内评估的固定基线。

## 明确来源边界

- 不把 RRD 压缩包的 CARLA `map`、车辆位置或合成 UWB 锚点转换为实体沙盘坐标。
- 不改变 `hardware_user_v02.yaml` 中的实机 UWB 坐标；它们需依据实际锚点中心位置
  独立 survey，不能靠台面长宽推算。
- 不把历史布局中的照片反推 XY 伪称为独立尺量；它已被用户接受为本项目的运行真值来源。

## 后续独立复核（不影响当前运行真值）

提供带尺度的俯视尺寸图（或从同一台面角拍摄且四个边角可见的俯视照片），并在图中
标出至少三个不共线、可在实体上识别的控制点。该数据可用于复核或生成更高版本；
当前 `physical_sandbox_map_v01` 的真值版本不自动变更。

## 验证

- 人工核验比例：`4.000 × 0.985 = 3.940 m`，
  `4.700 × 1.0510638298 = 4.940 m`。
- 核验压缩包 manifest：地图为 `CustomMaps/roadrunne3/RRD`、目标数为 `3`、UWB 标为
  `synthetic: true`，因此未用于实体真值。
- 尚未执行独立实体坐标 survey；当前文件可用作项目评估基线，但不单独证明定位精度。
