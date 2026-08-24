# 无标签视觉目标定位论文与开源实现检索

- 日期：2026-08-24
- 类型：research
- 状态：screened（论文、源码和接入条件初筛；尚无本项目实测精度）
- 问题边界：四基站 UWB 测距定位无人机自身；机载相机在不向沙盘目标粘贴 AprilTag 等人工标签的条件下，估计沙盘目标的相对/世界坐标。沙盘布置以 `docs/figures/沙盘实物图1.jpg`、`沙盘实物图2.jpg` 为固定场景先验，不假设更换场地。
- 与 `docs/2026-08-24_research_reusable_localization_algorithms.md` 的关系：本记录补充无标签路线；AprilTag 仅保留为低风险对照基线

## 结论

不建议把“无标签”理解为只换一个检测器。当前硬件边界下，UWB 和视觉的职责应固定为：

- **无人机自身**：四基站坐标 + 无人机标签到各基站的距离，经多边测量/鲁棒最小二乘得到 `map → base_link`；这不是用视觉替代 UWB。
- **沙盘目标**：机载相机识别目标并估计 `camera → target_link`，再用已标定的 `base_link → camera` 和 UWB 平台位姿变换到 `map → target_link`。

在此基础上，无标签视觉要获得可解释的三维坐标，必须选择下列可观测性之一：

1. **已知目标 CAD/尺寸**：从 RGB 或 RGB-D 图像估计目标 6D 位姿（FoundationPose、CosyPose、PVNet、GDR-Net），再把目标坐标变换到 `map`。
2. **已知静态场景/目标三维地图**：用 SuperPoint/SuperGlue 或 LightGlue、LoFTR 建立图像对应关系，结合 COLMAP/`hloc` 的 3D-2D 匹配和 PnP 求相机位姿；目标须在地图中有可辨识的纹理或几何。
3. **先提高无人机自身位姿**：用 ORB-SLAM3、VINS-Fusion 或 DROID-SLAM 估计 `base_link`，再将目标的视觉相对观测转换到世界坐标。这些方法不等于目标检测器，不能单独输出机外目标位置。

因此，适合本项目（固定沙盘、UWB 给无人机自身位置）的首轮无标签复现顺序是：

- **RGB-D 且有 CAD**：FoundationPose（论文级、无标签 6D 目标位姿）作为主候选；CosyPose 作为可复现对照。
- **只有 RGB 且目标可训练**：PVNet 或 GDR-Net；采用实拍数据加合成数据训练，输出目标位姿并用独立 PnP/重投影检查。
- **目标静止、环境纹理足够且可先建图**：固定沙盘先用 COLMAP 建图，再用 `hloc` + LightGlue/SuperPoint 做相机重定位；它可作为目标检测的几何先验，但不能单独识别目标。
- **无人机自身位姿对照（可选）**：ORB-SLAM3/VINS-Fusion/DROID-SLAM只与 UWB 多边测量结果比较，用来分析视觉姿态/短时连续性；不能把自身位姿精度直接写成目标定位精度，也不应替换 UWB 主链路。

论文报告的厘米级或毫米级结果不是本项目指标。只有同一相机、距离、光照、运动、真值、同步和评价脚本下的 `experiments/runs/` 结果才能进入答辩材料。

## 论文、源码和接入限制

| 优先级 | 论文（CCF/影响力） | 开源实现 | 本项目中的可复现输出 | 主要限制 |
|---|---|---|---|---|
| P0 | **FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects**, CVPR 2024（CCF-A；[DOI](https://doi.org/10.1109/CVPR52733.2024.01692)） | [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose) | 在具备 RGB-D、目标 CAD 和相机标定时，输出 `camera → target` 6D 位姿；跟踪阶段可连续输出；当前单目硬件需先确认是否能提供同步深度，不能直接假定可用 | NVIDIA License 明确限制为非商业研究/评估；通常需要 CUDA GPU、目标网格和初始化检测框/姿态；深度噪声、反光、遮挡和快速运动会造成跟踪丢失；树莓派级端侧不能假定实时 |
| P0 | **PVNet: Pixel-wise Voting Network for 6DoF Pose Estimation**, CVPR 2019（CCF-A；[DOI](https://doi.org/10.1109/CVPR.2019.00469)） | [zju3dv/pvnet](https://github.com/zju3dv/pvnet) | RGB 目标分割/关键点方向投票，再用 PnP 求目标 6D 位姿；适合已知单个目标、可采集训练集的沙盘 | 需每个目标的训练数据、3D 关键点和可见性标注；纹理少时优于局部匹配但仍受遮挡、极小目标、运动模糊影响；训练和依赖版本需要锁定，不能直接把类别检测结果当米制坐标 |
| P0 | **GDR-Net: Geometry-Guided Direct Regression Network for Monocular 6D Object Pose Estimation**, CVPR 2021（CCF-A；[DOI](https://doi.org/10.1109/CVPR46437.2021.01634)） | [THU-DA-6D-Pose/GDR-Net](https://github.com/THU-DA-6D-Pose/GDR-Net) | 单目 RGB 直接回归几何表示和位姿；可与目标检测器组合形成“检测→位姿”基线 | 需要已知物体模型/训练集；单目深度和尺度依赖相机标定及目标尺寸；域差（沙盘材质、相机曝光、俯视角）可能显著降低精度；只支持其验证过的 CUDA/PyTorch 组合需单独复现 |
| P1 | **CosyPose: Consistent Multi-view Multi-object 6D Pose Estimation**, ECCV 2020（CCF-B；[论文](https://arxiv.org/abs/2008.08465)） | [ylabbe/cosypose](https://github.com/ylabbe/cosypose) | 已知 CAD 物体的 RGB 多视图一致位姿；可用单帧/连续帧改造成目标位姿对照 | 目标模型和检测/分割是前置条件；多视图一致性要求视角重叠和时间同步；官方数据集与模型较大，端侧实时性和动态目标支持有限；MIT 许可不代表权重/数据集都有同一许可 |
| P1 | **LoFTR: Detector-Free Local Feature Matching with Transformers**, CVPR 2021（CCF-A；[DOI](https://doi.org/10.1109/CVPR46437.2021.00881)） | [zju3dv/LoFTR](https://github.com/zju3dv/LoFTR) | 无标签图像对稠密对应；与已知 3D 地图、PnP/Bundle Adjustment 组合得到相机位姿，再由坐标链得到目标坐标 | 需要参考图/三维地图；重复纹理、无纹理表面、动态物体和大视角/光照变化会产生错误匹配；Transformer 推理显存和延迟高，必须测量下采样对精度的影响 |
| P1 | **SuperPoint: Self-Supervised Interest Point Detection and Description**, CVPRW 2018（高引用；[DOI](https://doi.org/10.1109/CVPRW.2018.00060)）与 **SuperGlue: Learning Feature Matching with Graph Neural Networks**, CVPR 2020（CCF-A；[DOI](https://doi.org/10.1109/CVPR42600.2020.00499)） | [SuperPoint](https://github.com/magicleap/SuperPointPretrainedNetwork)、[SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)；更易部署的替代为 [LightGlue](https://github.com/cvg/LightGlue)（Apache-2.0） | 稀疏关键点/匹配 → 已知地图 3D-2D 对应 → EPnP/`solvePnPRansac`；可记录匹配数、内点率、重投影误差和位姿 | 依赖纹理和参考地图；SuperGlue/LoFTR 的预训练权重可能与实际相机域不匹配；单目仍需要已知尺度（目标尺寸或地图）；背景特征多于目标时会锁定错误平面 |
| P1 | **From Coarse to Fine: Robust Hierarchical Localization at Large Scale**, CVPR 2019（CCF-A；[DOI](https://doi.org/10.1109/CVPR.2019.01300)） | [cvg/Hierarchical-Localization (`hloc`)](https://github.com/cvg/Hierarchical-Localization) + [COLMAP](https://github.com/colmap/colmap) | 先检索参考图，再局部匹配和 SfM 3D-2D 定位；适合静态沙盘目标，无需在目标上贴标签 | 需要离线建图、参考图覆盖和稳定纹理；地图变化、移动目标、遮挡和视角超出覆盖范围会导致重定位失败；COLMAP/深度模型在树莓派端通常只能离线或地面端运行 |
| P1 | **Accelerated Coordinate Encoding: Learning to Relocalize in Minutes Using RGB and Poses (ACE)**, CVPR 2023（CCF-A；[DOI](https://doi.org/10.1109/CVPR52729.2023.00488)） | [nianticlabs/ace](https://github.com/nianticlabs/ace) | 用少量场景图像和相机位姿训练场景坐标回归器，单帧直接估计相机位姿；可作为“已知沙盘”重定位对照 | 场景专用训练，离开训练区域或光照/布局变化会失效；需要高质量训练位姿和覆盖视角；模型训练/推理仍偏 GPU，不能替代动态目标 6D 姿态 |
| P1 | **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM**, T-RO 2021（高引用；[DOI](https://doi.org/10.1109/TRO.2021.3075644)） | [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | 单目或单目+IMU 输出无人机 `base_link` 轨迹；与 UWB 自身位姿对照，或作为目标坐标变换的视觉来源 | 不是目标检测器；单目尺度不可观、长期漂移和回环依赖纹理/静态场景；动态背景、滚动快门、时间偏移和相机–IMU 外参会破坏结果；GPLv3 与 ROS2 集成需隔离验证 |
| P2 | **DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras**, NeurIPS 2021（CCF-A；[论文](https://arxiv.org/abs/2108.10869)） | [princeton-vl/DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) | 深度学习 SLAM 的自身位姿/深度对照；用于评估复杂光照和低纹理下的视觉里程计 | 训练分布与机载相机域差、GPU 显存和延迟较大；单目尺度仍需约束；不是机外目标位姿算法，不能绕过目标 CAD/地图和坐标变换 |

## 按本项目实际边界的接入架构

```text
四基站坐标 + UWB 距离 ─→ 多边测量/鲁棒 LS ─→ map → base_link（无人机自身）
                                      │
机载 RGB/RGB-D + 固定沙盘先验 ─→ 无标签目标 6D（FoundationPose / PVNet / GDR-Net）
                                      │ camera → target_link
                                      └─ base_link → camera、时间对齐、坐标变换 → map → target_link

固定沙盘几何先验：RGB ─→ hloc + LightGlue/SuperPoint + COLMAP/PnP ─→ camera 位姿/目标区域约束
```

同一实验应同时记录：目标检测/跟踪可用率、平移 RMSE/P95（m）、旋转误差（deg）、重投影误差（px）、端到端延迟（ms）、失锁恢复时间和失败原因。目标静止与动态、白天/低照、不同距离和遮挡必须分开统计。目标 CAD、相机内参/畸变、`camera→base_link` 外参和 UWB 时间偏移不能以默认值静默代替。

## 选择决策与停止条件

1. 先确认沙盘目标是否可提供 CAD/精确尺寸、是否有 RGB-D，以及目标是固定不动还是会在沙盘上移动。没有 CAD 且目标无纹理时，单目无标签米制定位在物理上缺少约束；应增加目标几何/深度/双目等可观测性，而不是继续换网络。
2. 选择一条主线和一条对照：有 CAD 时主线 FoundationPose（或 RGB-only 的 PVNet/GDR-Net）；无 CAD 但目标固定且纹理足够时，采用“固定沙盘建图 + 局部特征匹配 + PnP”；AprilTag 仅作可解释的工程基线。
3. 先在公开数据或录制的短序列上锁定容器、权重、相机标定和命令，再进入 `experiments/runs/`。未完成独立真值验证前，不得写“达到厘米级”。
4. 若无标签主线在目标距离/光照/运动的验收矩阵中出现持续失锁，优先记录失败证据（纹理、遮挡、延迟、域差、尺度）并更换可观测性或传感器；不要用标签结果冒充无标签算法精度。

## 证据与许可核对

- CCF 等级按论文发表 venue 的常用 CCF 推荐目录口径标注；“高引用”只表示检索时的影响力，不代表本项目可达到同等精度。
- 论文、源码、预训练权重和数据集的许可可能不同。FoundationPose 源码的 NVIDIA License 含“仅非商业研究/评估”限制；其余项目也应在 `code/THIRD_PARTY_MODULES.md` 锁定 commit、权重来源、许可证和 CUDA/PyTorch 版本后再用于答辩交付。
- 本项目的 UWB 多边测量误差、基站几何和时间同步必须与视觉目标误差分开统计；不能以“UWB 无人机位置准确”替代“沙盘目标 `map → target_link` 准确”。
