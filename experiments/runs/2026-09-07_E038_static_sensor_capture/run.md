# E038：树莓派—飞控静置 UWB/IMU 真实采集

## 状态

PARTIALLY COMPLETE。四路原始 UWB 测距、平台里程计和飞控心跳已在真实设备上采集并可回放；
本次及补齐解码后的复查均未收到任何 IMU 数据，因此 IMU 噪声、时间戳与 UWB-IMU
同步验证仍未完成。

## 安全与配置

- 树莓派通过既有 ROS 容器 `fcu_ros2` 运行，ROS domain 为 `20`；实体 UWB tag ID 为
  `2`，二者不可混淆。
- 机载桥接参数为 `offboard=false`、`set_goal=false`。未发送解锁、模式、起飞、降落、
  位置或速度指令。
- 桥接仅向飞控发送被动 GCS heartbeat，以及一次性请求
  `SCALED_IMU`、`HIGHRES_IMU` 和 RAW_SENSORS 遥测流；日志确认飞控 heartbeat、
  电池、UWB 和里程计均持续到达。
- 锚点坐标来自用户确认的
  `code/deployment/raspberry_pi_readonly_stack/config/hardware_user_v02.yaml`；本记录不把
  多边测量结果当作外部空间真值。

## 原始数据

在飞行器静置、可见性条件未声明时录制 35 秒，实际有效时长为 33.500223693 秒：

| 项目 | 位置 / SHA-256 |
| --- | --- |
| rosbag sqlite | `data/raw/2026-09-07_E038_static_sensor_capture/2026-09-07_E038_static_sensor_capture_0.db3` / `cdec313f6d77114a7570d80c401d323450e7fd5981a2a5fc6a924aa539711860` |
| rosbag metadata | `data/raw/2026-09-07_E038_static_sensor_capture/metadata.yaml` / `1ab49e109aa7cc7ebe8f99faa76988e513b1cc416618ae7ccd968038dd712b7c` |
| 可复现派生遥测 | `data/raw/2026-09-07_E038_static_sensor_capture/telemetry_derived_from_rosbag.jsonl` / `287483ce92fdcbb6e3c6110c48414340599aa9b51ee306eb1502719ba3156150` |

rosbag 消息数：`/nexus/uwb/ranges` 33、`/odom_global_001` 336、`/imu_global_001` 0。

## 真实 IMX219 与 ROS 图像入口

本次发现树莓派既有 `nexus_readonly_stack` 的相机服务缺少只读帧源模块
`imx219_frame_source.py`。从仓库的
`code/deployment/raspberry_pi_e016_self_localization/app/` 复制该模块到树莓派既有只读栈，
SHA-256 为 `78014d41d5e5cb45dfc02a5dc5b990f51f14ea858e13207cb0d00aa62d45804f`；模块只打开
CSI 相机，不访问飞控串口或 MAVLink。

真实 IMX219 在 `1640 x 1232`、`10 Hz`、镜头朝下 `180°` 配置下成功启动。TCP 客户端在
4 秒内解码 39 帧；此时棋盘格不在画面内，所以没有保存有效标定视图。预览显示机架和地面，
证明当前实体画面不适合直接登记目标参考图。

本机 ROS 入口的 `camera_file_node.py` 存在但没有可执行位，导致 `ros2 run` 找不到它。只补
`u+x` 后执行 `colcon build --packages-select nexus_pi_readonly_ingress --symlink-install` 成功；
未修改节点内容和 CMake。加载 `/opt/ros/humble/setup.bash` 后，
`colcon test --packages-select nexus_pi_readonly_ingress` 的四个测试套件均通过（12 个
pytest 用例）；首次未加载该环境的尝试因 `ament_cmake_test` 模块不可见而失败，不能作为
代码失败。随后从同一实体流收到：

| ROS 项目 | 实测结果 |
| --- | --- |
| `/camera/camera_info` | 有效，rectified ROI 为 1189 x 1171，采用 E028 的 IMX219 内参 |
| `/nexus/pi/camera_health` | `valid=true`、`invalid_frames=0` |
| `/nexus/camera/imx219/metadata` | 1640 x 1232、曝光 `51285 us`、传输延迟 `51.51 ms` 的实际样本 |
| 滚动快门读出时间 | `null`，尚未由实体标定得到 |

相机链路已验证为真实设备输入；但当前机架遮挡、无棋盘格/目标完整入镜，故内参不重新估计，
也不把这批画面登记为实例参考图。

## UWB 实测结果

结果文件：[static_uwb_imu_summary.json](static_uwb_imu_summary.json)。

| 指标 | 结果 |
| --- | ---: |
| 完整四锚 UWB 样本 / 中位频率 | 33 / 0.999997 Hz |
| 四路均值 (m) | [5.688, 2.178, 3.923, 5.986] |
| 四路标准差 (m) | [0.0176, 0.0253, 0.0184, 0.0233] |
| 大于 0.25 m 的相邻四路向量跳变 | 0 |
| 内部测距拟合 RMS / 中位 RMS | 0.0530 m / 0.0505 m |
| 几何一致样本 | 33 / 33 |
| GDOP 均值 / 最大值 | 26.664 / 27.941 |

该段的 UWB 测距稳定且内部几何自洽，但 GDOP 很高，且无独立测量的 tag 位置。因此它仅证明
该静置段的测距链路可用，**不**证明 UWB 空间定位精度，也不可作为视觉/融合定位真值。

## IMU 复查与最小修复

初次采集时桥接器只发布 `SCALED_IMU`。为排除消息格式遗漏，在机载桥接源码
`/home/pi/fcu_core_onboard/src/fcu_core_ros2/src/fcu_bridge_001.cpp` 中加入了
`RAW_IMU`、`HIGHRES_IMU` 解码和 `HIGHRES_IMU` 的一次性请求，且成功执行：

```text
colcon build --packages-select fcu_core
Finished <<< fcu_core [46.5s]
```

原始机载源码备份 SHA-256 为
`b6e218b818700629f66e700a67e364d6921438854e1a72ea628498e541e8bbc7`；更新后为
`c3ab42cfe61d163f239c8ff8ffb929a616067756a65ae0bd9ac7295bc5a68cc9`。重启后日志确认
飞控已发现、心跳持续、三类 IMU 流请求已发送；随后 5 秒 `ros2 topic echo --once
imu_global_001` 仍无消息，且本 rosbag 的 IMU 消息数为 0。

结论：当前飞控遥测输出未提供可用的 `SCALED_IMU`、`RAW_IMU` 或 `HIGHRES_IMU`。
因此不能伪造 IMU 噪声、Allan 方差或时间同步结论。

## 复现分析

```bash
OPENBLAS_NUM_THREADS=1 python3 code/analysis/uwb/analyze_static_uwb_imu.py \
  --input data/raw/2026-09-07_E038_static_sensor_capture/telemetry_derived_from_rosbag.jsonl \
  --hardware-config code/deployment/raspberry_pi_readonly_stack/config/hardware_user_v02.yaml \
  --output experiments/runs/2026-09-07_E038_static_sensor_capture/static_uwb_imu_summary.json \
  --condition static_visibility_unspecified
```

## 后续实机步骤

1. 从飞控厂商接口开启实际 IMU MAVLink 输出，或接入外置 IMU 的只读 ROS 话题；确认至少 100 条静置 IMU 样本后再计算噪声。
2. 在明确标记 LOS、再由人员短时遮挡形成 NLOS 的两段条件下分别录制 UWB 数据。
3. 由尺/激光测量 tag 实际位置或标志物位姿，才可把 UWB 多边测量误差与真实精度关联。
