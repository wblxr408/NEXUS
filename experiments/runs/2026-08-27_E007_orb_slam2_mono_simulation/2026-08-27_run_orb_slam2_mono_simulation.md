# E007 ORB-SLAM2 单目沙盘连续轨迹复现

## 范围

- 算法：官方 `raulmur/ORB_SLAM2`，submodule commit `f2e6f51cdc8d067655d90a78c06261378e07e8f3`
- 传感器：单目 RGB；本实验未使用 ROS、UWB 或目标检测。
- 数据：`nexus_sandbox_orb_slam2_mono_v02`，180 帧、640×480、10 Hz，连续 figure-eight 相机轨迹；真值位于 `ground_truth/camera_pose_map.csv`，算法不可见。
- 生成命令：`python3 simulation/generate_orb_slam2_dataset.py --frames 180 --out data/processed/nexus_sandbox_orb_slam2_mono_v02`
- UWB 配置：`simulation/orb_slam2_dataset.yaml`；评测代码：`simulation/evaluate_orb_slam2_trajectory.py`、`simulation/fuse_orb_slam2_uwb.py`

## 构建与运行

官方代码未修改。由于 Ubuntu 22.04 的 OpenCV 4/Pangolin/gcc 兼容性，构建副本在 `/tmp/orb_slam2_buildsrc` 使用 OpenCV 兼容头、C++14、`EIGEN_DONT_ALIGN_STATICALLY` 和现代 `std::map` 类型；官方 submodule 保持原样。DBoW2、g2o、`mono_tum` 均成功编译。

运行：

```text
python3 simulation/run_orb_slam2.py \
  --dataset data/processed/nexus_sandbox_orb_slam2_mono_v02 \
  --binary /tmp/orb_slam2_buildsrc/Examples/Monocular/mono_tum \
  --vocabulary /tmp/orb_vocab/ORBvoc.txt \
  --output data/processed/nexus_sandbox_orb_slam2_mono_v02/orb_output_final
```

## 结果（实测）

初版 ORB-SLAM2 日志在第一个运动段完成单目初始化并建立 89 个地图点，但随后在初始全局 BA 的旧版 g2o 路径触发 `double free or corruption (out)`。现代兼容构建将这一 g2o 路径替换为 OpenCV PnP-RANSAC 位姿更新，并使用密集、可重复的 3D 纹理；官方 submodule 保持未修改。

兼容运行（`nexus_sandbox_orb_slam2_mono_v05`）实测创建 154 个地图点，导出 121 帧有效 `Tcw` 相机位姿（序列 30–150）。为隔离仍未解决的局部 BA/线程析构问题，运行在第 150 帧显式导出后退出，且建图后保持悬停。因此它验证了单目初始化、特征匹配和逐帧位姿输出，但不是完整动态轨迹的最终精度结论。

`metrics.json`：

- ORB 有效帧：121 / 180，`availability=0.6722`，`failure_rate=0.3278`
- 单目绝对尺度：不可观；本次有效段真值也为悬停，故 Sim(3) 对齐、ATE/RPE 不报告（避免退化对齐产生虚假的零误差）。
- 未对齐相机位置 RMSE：4.7696 m（单目任意尺度/原点，不能解读为地图绝对精度）。
- 同序列 UWB 多边定位（180 帧）：3D RMSE 0.1267 m、MAE 0.0966 m、P95 0.2599 m、可用率 1.0、10 Hz；该值来自带噪 UWB 测距的算法输出，不读取真值。

ORB-SLAM2 输出仍只是相机/无人机轨迹，不等于十个沙盘目标的 `map → target_link` 位姿。完整动态 ORB 轨迹、UWB 尺度/地图坐标融合和基于该轨迹的目标定位仍在继续，不能将本记录作为最终答辩精度。

## Windows 兼容构建与完整运行（补充）

在用户原有 Windows Conda `gdr_net` 环境中，使用 MSVC 19.44、OpenCV 4.5.5、Eigen
3.4、GLEW 2.2 和 Pangolin 0.8 编译通过 `mono_tum.exe`。该构建副本位于用户目录，
未复制进 Git；详细命令见 `simulation/2026-08-27_orb_slam2_windows.md`。

同一 180 帧序列完整运行并正常退出，`FrameTrajectoryTcw.csv` 有效帧为 169/180，
可用率 0.9389，平均跟踪耗时约 16.9 ms。逐帧评测补充了旋转误差：未对齐位置 RMSE
4.7843 m、旋转 RMSE 174.18°；仅用于坐标系诊断的 Sim(3) 对齐后位置 RMSE
0.00379 m、RPE 平移 RMSE 0.00366 m、旋转 RMSE 3.64°。单目地图没有绝对尺度和原点，
因此未对齐值不能当作地图精度，Sim(3) 值也不能替代固定 map 下的 ATE。

指标重算命令（Windows CSV 已复制到仓库数据目录）为：

```bash
python3 simulation/evaluate_orb_slam2_trajectory.py \
  --dataset data/processed/nexus_sandbox_orb_slam2_mono_v05 \
  --trajectory data/processed/nexus_sandbox_orb_slam2_mono_v05/orb_output/FrameTrajectoryTcw_windows.csv \
  --out data/processed/nexus_sandbox_orb_slam2_mono_v05/orb_output/orb_metrics_windows.json
```

UWB 后处理将视觉轨迹与独立多边定位做 Umeyama 坐标恢复，得到 `map → base_link`
位置 3D RMSE 0.01491 m、P95 0.01890 m、更新率 10 Hz；由同一位置拟合旋转得到的
旋转 RMSE 为 33.24°（受 UWB 几何噪声和全局旋转估计影响）。
该步骤是松耦合后处理，不是 ORB-SLAM2 内部紧耦合滤波器。ORB-SLAM2 本身不进行十个
沙盘物体识别；目标 `map → target_link` 必须由 GDR-Net/检测器提供
`camera → target_link` 后，再通过外参与平台位姿组合。当前 E007 尚未在同一时空数据上
完成该端到端目标评测，故目标指标仍记为 unavailable。

## 校验

已执行 `python3 -m compileall -q simulation code/analysis/evaluation` 和
`git diff --check`，均通过。当前 Linux 容器未安装 `pytest`（`pytest: command not found`），
因此 Python 单元测试尚未执行；Windows ORB 运行日志和逐帧 CSV 保存在用户本机数据目录，
仓库仅登记上述可复现实验结论。
