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
