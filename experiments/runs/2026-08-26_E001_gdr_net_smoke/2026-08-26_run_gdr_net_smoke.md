# GDR-Net 代码取得与适配冒烟运行

## 目的

取得 GDR-Net 上游实现并验证 NEXUS 的离线适配边界：稠密对象坐标与可见性图可
转换为 `camera <- object` 的米制 6D 位姿。此运行不评估精度，也不使用实测图像。

## 版本与输入

- 运行编号：`2026-08-26_E001_gdr_net_smoke`
- 上游：`https://github.com/THU-DA-6D-Pose-Group/GDR-Net.git`
- 固定提交：`1be9fe73292fd748087aa88d7bf987434f271ebb`
- 上游许可证：Apache-2.0（`third_party/gdr_net/LICENSE`）
- 项目适配：`code/analysis/vision/gdr_net.py`
- 输入：测试代码即时构造的 16x20 非共面对象坐标图、全可见性图和 3x3 相机内参；
  没有真实视频、目标 CAD、BOP 数据集或模型权重。

## 环境

- 主机 Python：3.10.12
- GPU：NVIDIA GeForce RTX 4070 Laptop GPU，驱动 566.07
- 运行前检查：`torch`、`torchvision`、`detectron2`、`mmcv`、`pytorch_lightning`
  均未安装。

## 命令与结果

```bash
python3 -m pytest code/analysis/test/test_algorithm_router.py -q
# 4 passed

python3 -m pytest code/analysis/test -q
# 9 passed

cd code/analysis/vision/third_party/gdr_net
python3 core/gdrn_modeling/main_gdrn.py --help
# ModuleNotFoundError: No module named 'loguru'
```

适配器测试验证了：算法路由发现 `vision.gdr_net`；由合成稠密对应关系通过 PnP
恢复单位旋转和零平移；平移单位保持米。适配器的 PnP 后备解算不是论文中的可微
Patch-PnP，因而不可以此运行作为论文端到端训练或精度复现的证据。

## 结论与阻塞项

上游源码已纳入 Git submodule，NEXUS 稠密几何输出接口已可运行。完整上游推理
当前阻塞于兼容的 Python/CUDA 依赖、目标专属 GDR-Net checkpoint，以及 BOP 或
项目目标的图像、相机内参和 CAD/尺度数据。取得这些输入后，按
`code/analysis/vision/README.md` 的命令执行上游评测，并新建独立运行记录登记
模型 SHA256、数据位置/校验、配置、命令和结果；不得把本运行写成实测精度。

## 未验证项

- 上游 CUDA 扩展编译、Detectron2 模型加载和 GPU 推理。
- 项目目标的检测/ROI、CAD/尺度、域适配或微调。
- `camera -> target_link` 外参、ROS2 `TargetObservation` 发布以及实测指标。
