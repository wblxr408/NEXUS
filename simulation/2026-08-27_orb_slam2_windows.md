# ORB-SLAM2 Windows 复现说明

这份说明记录当前 `gdr_net` Conda 环境中的可复现实验，不把 Windows 构建产物或
139 MB 词袋文件提交到仓库。ORB-SLAM2 本身不依赖 ROS；本次只使用单目 TUM 示例。

## 依赖

- Windows 11，Visual Studio Build Tools 2022（MSVC 19.44，Windows SDK）
- CMake 3.31.6
- OpenCV 4.5.5（`C:\Users\wblxr\tools\opencv-4.5.5\opencv\build`）
- Eigen 3.4.0、GLEW 2.2、Pangolin 0.8
- Conda 环境 `C:\Users\wblxr\anaconda3\envs\gdr_net`

源码适配副本位于 `C:\Users\wblxr\nexus_orb_slam2_win`；官方 submodule
`code/analysis/vision/third_party/orb_slam2` 未被修改。适配仅包含 MSVC/C++17、
OpenCV 4、Eigen/g2o 兼容头和 Pangolin 链接修复。

## 运行

在 Anaconda Prompt 中，将 OpenCV、GLEW、ORB-SLAM2 的 `Release` 目录加入 `PATH`，
然后执行：

```powershell
& 'C:\Users\wblxr\nexus_orb_slam2_win\Examples\Monocular\Release\mono_tum.exe' `
  'C:\Users\wblxr\ORBvoc.txt' `
  'C:\Users\wblxr\nexus_orb_dataset_v05\camera.yaml' `
  'C:\Users\wblxr\nexus_orb_dataset_v05'
```

输出目录中的 `FrameTrajectoryTcw.csv` 是逐帧算法输出，`KeyFrameTrajectory.txt` 只
是关键帧摘要。仓库中的复现实验只登记 CSV 和指标，不登记图像、DLL、EXE 或词袋文件。

## 已实测结果

180 帧、640×480、10 Hz 序列成功完成单目初始化；Windows 运行正常退出，输出 169
帧有效位姿（11–179），`availability=0.9389`，平均跟踪耗时约 16.9 ms。对该 CSV
计算的未对齐位置 RMSE 为 4.7843 m（单目任意尺度/原点，不是绝对精度）；使用仅用于
诊断的 Sim(3) 对齐后位置 RMSE 为 0.00379 m、旋转 RMSE 为 3.64°。Sim(3) 结果不能
替代固定 `map` 坐标下的 ATE；UWB 恢复坐标的结果另行记录为后处理融合。

