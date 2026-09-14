# E041 机载心跳修复与真实 IMU 恢复

日期：2026-09-08。状态：IMU 恢复 **VALIDATED**；全包质量检查 **PARTIALLY COMPLETE**。

用户明确要求修复 IMU 并提供树莓派登录授权。已修改机载源码、构建、部署、重启桥接并验证真实输入。凭据未写入文件。

## 根因与对照

机载串口 `/dev/ttyAMA0 @ 115200` 的现有桥接参数是 `channel=0, offboard=false`。旧 `mav_send_heartbeat()` 将身份绑定到 offboard：false 发 GCS。此前成功脚本 `read_mavlink.py` 则发 ONBOARD_CONTROLLER。

在通过实时心跳确认未解锁后，停止单个桥接节点，确认其退出，再独占串口采集；其他任务进程保留。仅改变心跳身份，未发送遥测请求、参数写入、模式或运动指令：

| 阶段 | 时长 | SCALED_IMU |
|---|---:|---:|
| GCS 心跳 | 10 s | 0 |
| ONBOARD_CONTROLLER 心跳 | 20 s | 3781 |
| 恢复 GCS 心跳 | 10 s | 1891 |

机载身份阶段按飞控 `time_boot_ms` 计算为 **189.0 Hz**，时间戳无倒退/重复，最大间隔 15 ms。原始加速度模长均值 9816.90，按既有厂商 mm/s² 换算约 9.817 m/s²。详细统计见 `role_summary.json`。

这证明本轮 IMU 开启由机载身份触发；开启后 GCS 阶段仍有流，说明会保持状态。因此不会用“改回 GCS 仍有数据”否定触发条件，也不把旧标准请求的无 ACK 当作传感器故障。没有做飞控断电重启，冷启动后的自动恢复尚未单独实测。

## 已部署修改

树莓派 `/home/pi/fcu_core_onboard/src/fcu_core_ros2`：

- `src/fcu_bridge.h`：`offboard_ || mav_chan_ == MAVLINK_COMM_0` 选择机载心跳，串口身份不再取决于控制开关。
- `test/test_control_packets.cpp`：四种 channel/offboard 组合的身份、心跳字段和无额外控制报文断言；精确验证此前已有的两条消息周期请求及 RAW_SENSORS 回退。
- fcu_core 构建/安装产物已更新，实际运行的桥接与安装文件 SHA-256 均为 `327fc64f30aae0c5b5fa466dabfc663bb272cddb202dded7d8c84efa0aa22e72`，见 `deployed_sha256.txt`。

本仓库保存可审查补丁 `code/deployment/fcu_imu_recovery/imu_heartbeat.patch`，没有复制修改 ROS1 第三方源码。对原始备份执行 `patch --dry-run -l -p1` 通过，已检查补丁只涉及上述两个文件。

保留原有 `offboard=false`、domain 20、话题、remap、UWB、单位/轴向变换和飞行指令逻辑。原桥接恢复命令来自原进程 argv；没有重启飞控或整个机载任务容器。

## 实机验收

修正版桥接重启后，在树莓派 fcu_ros2 / ROS2 Humble / domain 20 中运行 `capture_ros_imu.py`：

- 20.028 s 窗口收到 IMU **3619**、里程计 **192**、UWB **19**；发现过程包含在窗口内。
- 第一至最后收到的 IMU 之间为 **189.106 Hz**，frame 为 `scaled_imu`。
- ROS 时间戳全部递增，最大间隔 **0.015116 s**，六轴均为有限数。
- 加速度模长均值 **9.82637 m/s²**。

见 `imu_recovery_ros_summary.json`。ROS 时间戳仍是既有接收时间，并未因此完成传感器时钟同步或标定。

另在地面 WSL Ubuntu 22.04 / ROS2 Humble / domain 20，以 SensorDataQoS 订阅 `/imu_global_001`：15.013 s 收到 **1603** 条，实际接收段速率 **107.095 Hz**，见 `ground_ros_summary.json`。已确认跨机有真实消息，但速率低于机载，跨机完整交付未验证，不能声称地面端无丢样。

## 构建与检查

构建环境：现有 `nexus_ros_build_env:20260902` ARM64 镜像，`--network none`，挂载机载工作空间，无设备映射。日志保留于 raw 数据目录。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon --log-base log/imu_verify_20260908 build --packages-select fcu_core --cmake-args -DBUILD_TESTING=ON
colcon --log-base log/imu_test_20260908 test --packages-select fcu_core --event-handlers console_direct+
colcon test-result --test-result-base build/fcu_core --verbose
```

构建通过（存在既有编译警告）；最终 CTest 7 项中 5 项通过：control_packets、cppcheck、flake8、lint_cmake、pep257。两个失败均明确保留，见 `test_summary.txt`：

- `uncrustify` 失败：全包格式不符合默认规则。将两个修复前备份单独放入隔离容器运行 `ament_uncrustify /baseline`，同样报告两文件格式差异。未扩大范围全包重排；当前格式检查仍未通过。
- `xmllint` 失败：隔离容器无法获取 `http://download.ros.org/schema/package_format3.xsd`。未关闭该检查，XML schema 验证仍未完成。

首轮 `control_packets` 失败（`initial_test.txt`）：旧测试只允许一条心跳回复，未兼容此前已存在的三条遥测请求。更新为逐条字段断言后通过，没有删除/跳过失败测试，也未削弱原飞行控制字段校验。所有控制报文测试仅在隔离容器内存中执行。

## 可追溯证据与限制

原始数据/完整日志：`data/raw/2026-09-08_E041_imu_heartbeat_recovery/`，逐文件字节数和 SHA-256 见 `raw_manifest.json`。其中 `role_probe.jsonl` 为真实 MAVLink 解码结果，`imu_recovery_ros.jsonl` 为修正版真实 ROS 六轴样本。机载副本在 `/home/pi/nexus_readonly_runs/2026-09-08_imu_recovery/`。

`probe_roles.py` 保存本次执行步骤，用于审查；它会短暂停桥接，且输出文件使用独占创建，不能当作后台服务或在采样飞行期间直接重跑。

未验证：飞控断电冷启动、地面完整采样率、IMU 噪声/Allan 方差、同步、融合精度。E040 四槽测距跳变及现场遮挡问题没有在本轮扩大处理。保留其他成员修改，未扩大任务范围。

最终本地证据 JSON、归档 Python 语法及新文档空白检查通过。全仓库 `git diff --check` 仍因预先修改的 `fcu_core_gcs/build/*/CMakeFiles/CMakeOutput.log` 尾随空白失败；本轮未改这些构建日志。
