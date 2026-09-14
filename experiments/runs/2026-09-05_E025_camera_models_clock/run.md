# E025：模型部署、相机内参工具与跨设备时间映射

状态：**PARTIALLY COMPLETE**。模型导出／CPU 推理一致性／ROS 检测消息，以及真实跨设备时钟映射已验证；棋盘格采集按用户后续要求暂停，真实内参尚未求得。

## 环境和资产

- 记录时 HEAD：`232478ea42c695814247fe0b036b46b8bc204b41`，工作树包含既有与本次未提交修改；不能将 HEAD 当作本次完整代码版本。资产目录的 `source_snapshot/` 与 `source_checksums.json` 保存本次代码。
- Pi：192.168.1.143，IMX219 1640×1232、180° 旋转，camera-only TCP 14552、clock UDP 14553。相机服务为本次独立启动，验证结束已停止。
- 电脑：WSL Ubuntu-22.04 / ROS2 Humble。系统 OpenCV 保持 4.5.4；本次项目隔离库 4.10.0，NumPy 1.21.5，OpenCV 4 线程。
- 导出：Windows 现有 gdr_net Python 3.10.19 / torch 2.6.0+cu124 / Ultralytics 8.4.47，另在独立工具目录安装 onnx 1.17.0、onnxslim 0.1.34。
- 本地资产根目录：`/root/nexus_workspace/NEXUS/data/processed/2026-09-05_camera_models_v01/`（Git 忽略）。主要文件：`detector/detector_slim_manifest.json`、`superpoint/superpoint_manifest.json`、`model_parity.json`、`detector_ros.json`、`clock_probe.json`、`live_clock_ros_latest_frame.json`、`asset_checksums.json`。
- 检测源权重是 E023 的 best.pt，SHA256 `f0d3a67f3215ae565ed3423980061ec5ab19cf7b975e2f0fd516271711fdf333`。仅复用训练结果，本次未另行训练。
- SuperPoint 来自 rpautrat/SuperPoint commit `1411bbd68c50163555d39c1b26e9e046ebd48f27`；权重 git blob 校验 `f645e8db1b10a7be84d4e50d67a3c64dd240a49a`；MIT LICENSE 随资产保存。

## 已验证结果

1. 8 张 E023 留出测试图：PyTorch 与 WSL ONNX 密集检测输出在 `rtol=1e-4, atol=0.002` 下全部通过，最大绝对输出差 `0.002884`（较大坐标值由相对容差覆盖）；NMS 后类别相同、框坐标差小于 0.01 px，每图 3 个目标。CPU 前向中位数 47.48 ms，首轮最大 94.28 ms，不是端到端延迟。
2. 两张图的 SuperPoint dense logits / descriptors 与 PyTorch 通过 `rtol=1e-4, atol=1e-4` 比较；最大 logits 差小于 `8.59e-5`，提取 570 / 571 个特征。CPU 前向 369.26 / 269.02 ms，尚未满足整条链路实时性要求。
3. 实际 ROS `ObjectDetectionNode` 使用训练 ONNX，重放测试图产生 3 条 valid 消息，每条 3 个目标。测试域 89、仅本机 DDS；未向飞控发布控制命令。
4. Pi—WSL 60 次四时间戳探测无丢失，RTT 最小 5.837 ms、中位数 7.061 ms、95 分位 17.761 ms。各探测偏移跨度 11.722 ms，包含网络非对称变化，不能直接当成振荡器漂移。
5. 修改前第一次实时检查 20 s 仅收到 11 条元数据，数量检查失败；延长测量后发现接入延迟中位数 6116.96 ms、最大 8584.07 ms，延迟检查仍失败。最新帧策略后，30 帧通过时间戳映射、单调性及 <2 s 延迟检查：接入延迟中位数 91.55 ms、95 分位 159.85 ms、最大 163.63 ms，订阅到的帧序列约 2.57 Hz。
   `transport_latency_ms` 在此是 SensorTimestamp→JPEG 解码结束的延迟，**不含其后的 ROS 序列化、传输与定位计算**；不是同步误差。源配置 10 fps，接入端记录跳帧，因此不是 10 fps 无损接收结论。
6. 相机 BGR888→字节 RGB 的配置、缓冲区释放后独立副本通过软件测试；修正后尚未重新进行真实色卡验证，下一次相机启动生效。

## 验证命令

在 NEXUS 根目录（构建按各包既有方式）：

```bash
source /opt/ros/humble/setup.bash
cd code/ros2_ws
colcon build --packages-select nexus_vision_localization --symlink-install
colcon build --packages-select nexus_pi_readonly_ingress --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
cd ../..
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 \
  bash code/tools/run_with_camera_models.sh colcon test \
  --base-paths code/ros2_ws/src --build-base code/ros2_ws/build \
  --install-base code/ros2_ws/install \
  --packages-select nexus_pi_readonly_ingress nexus_vision_localization
colcon test-result --test-result-base code/ros2_ws/build/nexus_pi_readonly_ingress --verbose
colcon test-result --test-result-base code/ros2_ws/build/nexus_vision_localization --verbose
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test/test_camera_intrinsics.py -q
python3 -m pytest code/deployment/raspberry_pi_readonly_stack/test_readonly_stack.py code/deployment/raspberry_pi_readonly_stack/test_camera_format.py -q
bash code/tools/run_with_camera_models.sh python3 code/tools/verify_model_parity.py --root data/processed/2026-09-05_camera_models_v01
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 bash code/tools/run_with_camera_models.sh python3 code/tools/verify_detector_ros.py data/processed/2026-09-05_camera_models_v01
```

最终两个 ROS 包构建通过；colcon 汇总分别 13 / 83 checks，0 errors / failures / skipped（该统计包含 CTest 套件计数）；标定数值测试 2 项、Pi 只读栈与颜色配置测试合计 10 项通过。相关 Flake8 与任务范围 diff 检查通过。首次使用 symlink 构建 ingress 失败，原因是原复制安装生成目录与符号链接冲突，改回原方式通过。

系统 OpenCV 4.5.4：未简化 YOLO11 在 Add 导入阶段失败；简化后可加载但 forward 仍失败。项目隔离 OpenCV 4.10.0.84 执行通过，禁止将“可加载”作为“可推理”结论。官方下载曾超时，最终完整文件与官方 PyPI SHA256 核对成功。

全仓库 `git diff --check` 因既有 fcu_core_gcs 构建日志尾空格失败；任务源代码范围检查通过，未修改这些既有构建日志。

## 尚未完成与范围

- 棋盘格：9×6 内角点、25 mm，采集暂停；没有用估计内参填充 measured 文件。求解、留出重投影检查和 ROS 矫正入口已实现并通过合成测试，真实标定与真实矫正仍未验证。
- 检测器只在 E023 的单 episode CARLA 仿真图像上验证；真实目标泛化未验证。
- SuperPoint CPU 吞吐仍需后续实测优化；本次没有降低分辨率或修改定位精度目标。
- 没有开展相机—IMU 联合时间／外参标定，没有解锁、起飞或改动本轮飞控代码。未扩大任务范围。
