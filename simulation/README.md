# 沙盘参数化模型

本目录的权威布局表使用毫米坐标：Bounding Box 为 `X=0~4000 mm、Y=0~4700 mm`，原点为左下角，`+x` 向右，`+y` 向后，`+z` 向上。当前视觉布局以 `docs/figures/沙盘实物图1.jpg` 和尺寸图为参考：外围为连续双向双车道环路，四角道路延伸至沙盘边界并形成四个带人行横道的十字路口；出界道路端也有横道。中心工业核心为 `X=1000~3000 mm、Y=1200~3500 mm`，工业岛边界直接贴住核心边界道路，没有中间绿化带；仅由一条横向和一条纵向单车道分割，内部道路不绘制车道虚线，纵向内部道路不设置斑马线。

`sandbox_scene.yaml` 保留 CAD/Blender 使用的 mm 原值；生成器输出的 OBJ、SDF 和 Web JSON 自动转换为 m，运行时 JSON 同时记录 `source_unit: mm`。

## 生成

```bash
cd simulation
python3 generate_sandbox.py
```

生成器会在输出前检查建筑、罐体、工业岛和人行道是否与可行驶道路重叠；出现重叠直接失败。斑马线允许覆盖道路，并单独检查不同组之间不得重叠。

生成：

- `sandbox.obj`/`sandbox.mtl`：通用几何模型；
- `sandbox.sdf`：Gazebo Classic 11 可加载的静态场景；
- `sandbox_scene.json`：Web 前端或仿真回放读取的运行时描述。

## 证据等级

外框尺寸来自尺寸图，标记为 `confidence: A`。道路、工业岛、建筑、绿化及交通标线均为从实物照片反推的视觉先验，标记为 `confidence: C`；它们可用于当前仿真和前端可视化，但在实体沙盘测量前不能作为厘米级真值。

不规则外边界的完整顶点尚未提供，因此当前 SDF/OBJ 使用 `4.0 × 4.7 m` 包络矩形；补充边界点后只需修改 `outer_boundary`。

目标尚未指定，见 `target_spec.yaml`。填入 `source_object_id` 后，再由视觉仿真节点决定目标对象和 `target_link`。

## 算法切换

仿真不通过读取预先生成的结果来“切换算法”。统一路由位于
`code/analysis/algorithms/router.py`，同一份输入数据会交给所选算法重新计算。
算法核心和外部算法预留位置见 `code/analysis/algorithms/README.md`。

算法实现由各自负责人员放入 `code/analysis/algorithms/vision/` 或
`code/analysis/algorithms/uwb/` 下的对应目录，再调用
`register_algorithm("<family>.<name>", family, runner, source)` 注册。
仿真/回放只负责把同一份输入交给路由选中的 runner；不会读取预先保存的结果。
将来前端传递完整算法名称即可切换，前端不实现算法。未注册的算法会明确报错，
不会用仿真真值冒充算法结果。

## GDR-Net 合成数据集

`gdr_net_dataset.yaml` 定义十个固定中央储罐和五个确定性随机的无人机悬停位姿。
使用以下命令生成参数化简化 CAD（PLY）、RGB、实例掩码、相机/物体真值、BOP 风格
标注和带噪 UWB 测距：

```bash
python3 simulation/generate_gdr_net_dataset.py \
  --out data/processed/nexus_sandbox_gdr_net_v01
```

输出数据是 `metric_simulation` 的几何渲染基线：模型尺寸/位置来自 YAML，而不是
实体扫描 CAD；RGB 没有照片纹理、逼真光照或遮挡。因此它能用于验证 GDR-Net 的数据
格式、相机坐标、相对位姿和 UWB+视觉坐标链，不能作为真实沙盘性能结论。所有选中的
圆柱目标均标记为连续绕 z 轴对称，最终评估须使用 ADD-S/对称性处理而不是普通旋转
误差或 ADD。

`trajectory.csv` 仅包含算法可见的带噪 UWB 测距；平台真值单独写入
`ground_truth/platform_pose.csv`，十个目标的 `map` 真值写入
`ground_truth/target_pose_map.csv`。评估器读取真值目录，UWB/视觉算法不得读取它。

注意：上游发布的 GDR-Net 权重对应 LINEMOD/LM-O/YCB-V 等目标，不会直接识别本
项目的十个自定义沙盘物体。要做这些目标的真实算法实验，需用本项目目标模型和
训练/验证轨迹制作目标专属配置并训练或微调 checkpoint；本数据集当前只有 5 帧，
只够检查格式、坐标和融合链路，不能单独作为训练集或精度证据。

`gdr_net_dataset_v02.yaml` 是评测数据版本：它生成到 BOP 标准的 `test/000001/`，
`scene_camera.json` 只含相机内参与深度比例；`map -> base_link` 与
`map -> target_link` 完整真值（含旋转）只在 `ground_truth/` 中，供评估器使用。
这样视觉推理从 BOP 元数据无法读取平台的地图真值。可用以下命令生成：

```bash
python3 simulation/generate_gdr_net_dataset.py \
  --config simulation/gdr_net_dataset_v02.yaml \
  --out data/processed/nexus_sandbox_gdr_net_v02
```

`gdr_net_dataset_train_v01.yaml` 是与 v02 独立的训练划分：随机种子为
`20260827`，生成 40 个无人机视角和 400 个目标实例到 `train_pbr/000001/`。
它只可用于训练；最终指标始终在 v02 的五个未见视角上计算。

`gdr_net_dataset_v03.yaml` 固定沿用 v02 的五个评测视角及 RGB/BOP 标注，另增加
算法可见的 `platform_attitude.csv`。它模拟机载 IMU/飞控姿态观测（当前额外噪声为
零）；UWB 仍只提供 `trajectory.csv` 中的测距。该文件用于把视觉
`camera -> target_link` 与 UWB `map -> base_link` 合成为端到端
`map -> target_link`，不从 `ground_truth/` 读取姿态。
