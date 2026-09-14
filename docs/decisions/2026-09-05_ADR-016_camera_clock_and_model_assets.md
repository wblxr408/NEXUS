# ADR-016：相机时钟、内参入口与真实模型资产

日期：2026-09-05。授权：用户要求完成模型准备、相机内参标定、相机—树莓派—电脑时间对齐；随后明确暂停棋盘格采集。相机—IMU 联合时延／外参标定不在本次范围。

## 决定

1. 相机 TCP 14552 保留原有接收时间字段，新增 SensorTimestamp 的 `CLOCK_BOOTTIME` 域与 boot ID。新增只读 UDP 14553 四时间戳探测。电脑以同一探测中的本地 Unix 时间和 Pi boot 时间估计偏移，每秒刷新，使用最近 5 秒内最低 RTT 样本；过期、重启或电脑时间跳变时拒绝以该映射发布图像。不修改任何设备的系统时钟。
2. 新入口 `camera_preparation.launch.py` 开启时钟对齐。ROS Image/CameraInfo 的时间戳来自映射后的 SensorTimestamp；旧入口默认保持接收时间模式。元数据报告 RTT、探测年龄和探测时刻的网络非对称误差范围。RTT/2 不是长期同步精度承诺，不包括独立测定的振荡器漂移、滚动快门或相机—IMU 延迟。
3. TCP 消费端每轮只解码最新完整帧，保留未完成报文并记录序号跳帧。实测旧逐帧处理造成数秒积压，故定位入口明确选择低延迟；这里不是无损视频记录器。
4. 用户棋盘格规格为 9×6 内角点、25 mm 格长、A4 横向 100% 打印。内参只由真实拍摄图像求解：至少 20 个不同视角，训练与留出帧分离，检查倾角、边缘覆盖及重投影误差。未采集前不生成 measured 内参。实际打印尺寸和平整度仍需拍摄时确认。
5. 有 measured 内参且分辨率、旋转一致时，发布原始 CameraInfo 的 K/D/R/P，以及 `/camera/image_rect` 和零畸变的 `/camera/camera_info`；没有内参则不发布矫正图像。相机格式改为 libcamera `BGR888`，获得字节顺序正确的 RGB 数组。
6. 检测器复用 E023 的 YOLO11n 三类权重，显式声明 `yolo_v11_detection`，不冒充 YOLOv8。SuperPoint 使用 rpautrat 的 MIT 版本及其原有半像素描述子采样约定 `superpoint_mit_v1`，保留原 `lightglue_v1` 的兼容性。均为密集 ONNX 输出，权重、来源、许可证和 SHA256 写入 manifest。
7. Ubuntu 系统 OpenCV 4.5.4 对 YOLO11 导出图的加载／执行均出现过失败。采用项目数据目录中独立安装的 OpenCV 4.10.0.84，以及导出期 onnx 1.17.0、onnxslim 0.1.34；系统与原训练环境的依赖不升级。运行包装器固定 OpenCV 4 线程，与本次验证保持一致。

## 依据与边界

- [libcamera SensorTimestamp](https://docs.libcamera.org/master/internal-api/namespacelibcamera_1_1controls.html)：CLOCK_BOOTTIME 语义。
- [Picamera2 官方数组格式映射](https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py)：RGB888→BGR、BGR888→RGB。
- [MIT SuperPoint 固定源码版本](https://github.com/rpautrat/SuperPoint/tree/1411bbd68c50163555d39c1b26e9e046ebd48f27)：使用其权重和采样约定；不沿用 Magic Leap 版本的来源／许可证声明。
- [Ultralytics 导出接口](https://docs.ultralytics.com/modes/export/)：本次实用版本为已有 8.4.47，不依据网页中的新模型替换用户模型。

替代方案：仅调整接收时间不能消除网络积压；直接升级系统 OpenCV 会影响其他 ROS 项目；改用另一套推理框架需要替换现有适配接口。本次选择保留既有 OpenCV 接口并隔离运行库。

验证证据见 [E025](../../experiments/runs/2026-09-05_E025_camera_models_clock/run.md)。本决定不宣称真实目标检测泛化、整条定位链路实时性或实机定位精度已通过。
