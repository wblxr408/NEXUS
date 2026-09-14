# E028：IMX219 实测内参与有效视场矫正

状态：**VALIDATED（相机链路）**。本记录仅覆盖静置无人机上的 IMX219 相机内参、去畸变有效视场，以及 ROS `Image + CameraInfo` 发布；不包含飞控、UWB、相机—IMU 外参或起飞测试。

## 输入与配置

- 棋盘格：9×6 内角点，方格边长 25.0 mm；共采集 39 张 `1640×1232`、旋转 180° 图像。
- 原始采集目录：`data/raw/2026-09-07_E026_imx219_checkerboard_v02/`（不纳入 Git）。
- 异常视角：`view_019.jpg` 未进入拟合，保留原始文件并在输出中登记。
- 有效输出区域（全图归一化坐标）：`[0.125, 0.0, 0.85, 0.95]`；对应全图像素框 `[205, 0, 1394, 1171]`。这是基于棋盘角点实际覆盖范围启用的区域，**不是**全画幅内参覆盖声明。
- 内参文件：`data/processed/2026-09-07_E026_imx219_intrinsics_v01/imx219_1640x1232_intrinsics.json`，SHA-256 为 `097ced445fcf4a3a0234d64545551f1ab5156f81a124ddb5e3169b57957230af`。
- 记录时 Git HEAD：`232478ea42c695814247fe0b036b46b8bc204b41`；工作树含其他已有未提交修改，不能把 HEAD 视为完整源码快照。

## 标定结果

- 训练 RMS：0.493451 px。
- 7 个独立留出视角 RMS（px）：0.783020、0.340643、0.584900、0.452100、0.394912、0.397966、0.203478；最大值 0.783020 px。
- 棋盘法线跨度：35.1916°。
- 上述结果通过项目门限：训练 RMS ≤ 1 px、每个留出视角 RMS ≤ 1 px、姿态跨度 ≥ 15°。

## ROS 实机验证

相机只读 TCP 流启动后，WSL ROS 2 节点载入上述内参并实际订阅验证：

- `/camera/image_rect` 实测尺寸：`1189×1171`。
- `/camera/camera_info` 尺寸相同，`D=[0,0,0,0,0]`，其时间戳与对应去畸变图完全一致。
- 一帧实际曝光元数据：15.671 ms。
- 滚动快门读出时间：`null`；当前相机驱动未给出此值，系统保留未知值而非填入估计值。

## 验证命令

```bash
cd code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nexus_pi_readonly_ingress

cd ../..
python3 code/analysis/calibration/calibrate_intrinsics.py \
  --input data/raw/2026-09-07_E026_imx219_checkerboard_v02 \
  --output data/processed/2026-09-07_E026_imx219_intrinsics_v01/imx219_1640x1232_intrinsics.json \
  --exclude view_019 --valid-roi 0.125 0.0 0.85 0.95
```

另外完成了真实 TCP 相机流到 ROS 的订阅断言：去畸变图和 `CameraInfo` 尺寸、时间戳、零畸变系数、有效区域及非空曝光值均通过。

## 未验证项

- 未测得滚动快门读出时间；因此滚动快门行补偿仍不得在真实数据上宣称已验证。
- 未进行相机—IMU 外参或时间标定、UWB 融合、目标定位精度评估、无人机起飞或飞控控制。
- 全画幅边缘没有足够棋盘覆盖；下游应使用 `/camera/image_rect`，不得把本次结果当作全画幅去畸变精度保证。
