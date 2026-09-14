# E039：桥接恢复后的静置 UWB / 平台里程计 / IMU 复测

## 状态

PARTIALLY COMPLETE。树莓派 `fcu_ros2` 容器和 `fcu_bridge_001` 在本次采集前已由操作者启动；
真实 UWB 测距与平台里程计再次录制成功。`/imu_global_001` 被桥接器声明和订阅，但本次包内仍为
零条消息，因此 IMU 噪声、时间戳及 UWB--IMU 同步验证尚未完成。

## 安全与配置

- ROS domain 为 `20`；实际 UWB tag ID 为 `2`，二者不是同一编号。
- 本次仅通过 `ros2 bag record` 订阅 `/nexus/uwb/ranges`、`/odom_global_001`、`/imu_global_001`。
  未发送解锁、模式、起飞、降落、位置或速度指令。
- 条件记录为 `static_visibility_unspecified`：操作者未声明 LOS / NLOS，故不得将本段标为 LOS。
- 锚点地图坐标采用用户确认的
  `code/deployment/raspberry_pi_readonly_stack/config/hardware_user_v02.yaml`；没有独立量得的 tag
  位置，故多边测量结果不被视为外部空间真值。

## 原始记录

录制时长 `35.500000875 s`，消息数为：UWB `36`、平台里程计 `356`、IMU `0`。

| 项目 | 位置 / SHA-256 |
| --- | --- |
| ROS bag sqlite | `data/raw/2026-09-07_E039_static_sensor_capture/e039_static_sensor_capture/e039_static_sensor_capture_0.db3` / `45e53d4710dc68d82e7ef94523b4ff5db2e30e3231a471c9c1bc600952893269` |
| ROS bag 元数据 | `data/raw/2026-09-07_E039_static_sensor_capture/e039_static_sensor_capture/metadata.yaml` / `245306bb75380124cc148c17e5c0b9717ce3903430f1915289eb19af7db40062` |
| 可复现 UWB 派生记录 | `data/raw/2026-09-07_E039_static_sensor_capture/telemetry_derived_from_rosbag.jsonl` / `994b12c4b1fc510cb8ba78ca9a40e320ab9566804a322ffcc72b07c9c45bc456` |

## 结果

UWB 汇总见 [static_uwb_imu_summary.json](static_uwb_imu_summary.json)，平台里程计汇总见
[static_platform_odometry_summary.json](static_platform_odometry_summary.json)。

| 指标 | 实测结果 |
| --- | ---: |
| 完整四锚 UWB 样本 / 中位频率 | 36 / 1.000 Hz |
| 四路测距均值 (m) | [5.645, 2.116, 3.903, 5.991] |
| 四路测距标准差 (m) | [0.0157, 0.0240, 0.0137, 0.0248] |
| 大于 0.25 m 的相邻四路向量跳变 | 0 |
| 几何一致样本 | 36 / 36 |
| 内部测距拟合 RMS (m) | 0.0647 |
| GDOP 均值 / 最大值 | 25.809 / 26.778 |
| 平台里程计频率 / 最大间隔 | 10.000 Hz / 0.1052 s |
| 相邻里程计最大位置步长 | 0.0165 m |
| 大于 0.10 m 的相邻位置跳变 | 0 |
| 平台里程计位置标准差 (m) | [0.0059, 0.0109, 0.0032] |
| IMU 消息数 | 0 |

这证明此静置段的 UWB 测距及平台位置输出连续、无明显大跳变；但高 GDOP 和没有独立 tag
真值意味着它们**不能**被表述为空间定位精度或融合精度。IMU 仍未从当前飞控遥测到达，不能
伪造其噪声、Allan 方差或同步结论。

## 复现检查

```bash
source /opt/ros/humble/setup.bash
ros2 bag info data/raw/2026-09-07_E039_static_sensor_capture/e039_static_sensor_capture

OPENBLAS_NUM_THREADS=1 python3 code/analysis/uwb/analyze_static_uwb_imu.py \
  --input data/raw/2026-09-07_E039_static_sensor_capture/telemetry_derived_from_rosbag.jsonl \
  --hardware-config code/deployment/raspberry_pi_readonly_stack/config/hardware_user_v02.yaml \
  --output experiments/runs/2026-09-07_E039_static_sensor_capture/static_uwb_imu_summary.json \
  --condition static_visibility_unspecified
```

## 后续实机缺口

1. 在飞控厂商接口中开启可实际发送的 `SCALED_IMU`、`RAW_IMU` 或 `HIGHRES_IMU`，或接入外置
   IMU 的只读 ROS 话题；收到至少 100 条静置样本后才能测 IMU 噪声。
2. 由尺/激光登记 tag 的实际位置，再计算 UWB 位置误差；当前仅可分析内部一致性。
3. 按人员明确标注的 LOS 与短时遮挡 NLOS 条件，分别采集两段包，才能形成 NLOS 对照实验。
