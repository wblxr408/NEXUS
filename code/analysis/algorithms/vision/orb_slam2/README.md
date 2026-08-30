# ORB-SLAM2 单目

来源：`raulmur/ORB_SLAM2`，submodule 位于
`code/analysis/vision/third_party/orb_slam2`。`vision.orb_slam2_monocular` 已在
路由中登记为占位项；实际运行通过 `simulation/run_orb_slam2.py` 调用上游
`mono_tum`，再由 `evaluate_orb_slam2_trajectory.py` 读取逐帧 `Tcw` 输出。

ORB-SLAM2 只提供 `map → camera`（或关键帧）轨迹和稀疏地图，不提供目标类别或
`map → target_link`。单目地图尺度/原点任意，需用 UWB 或其他外部观测恢复绝对坐标；
目标识别仍由 GDR-Net/检测器完成。
