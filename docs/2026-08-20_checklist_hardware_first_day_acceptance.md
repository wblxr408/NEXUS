# 硬件到货首日验收清单

- 日期：2026-08-20
- 类型：checklist
- 状态：ready_for_hardware
- 关联：`docs/architecture/interfaces.md`、`docs/architecture/coordinate-frames.md`、步骤 04～05

本清单的目的只是确认硬件事实与接口语义；不产生定位精度结论。每项必须填写“证据位置、操作者、时间、结果”。真实设备序列号、网络凭据和未经授权的视频不写入 Git。

## 0. 开始条件

- [ ] 设备、支架、线缆、电源与安全责任人已登记。
- [ ] 通信/视觉检查均在锁桨、拆桨或等效安全状态下开始。
- [ ] 已创建本次验收的运行目录；原始日志、照片和视频外部存储位置已登记。
- [ ] 未将 `/odom_global_001` 预先标记为外部目标位置。

## 1. 空载上电与官方链路

| 检查项 | 要记录的事实 | 通过标准 |
|---|---|---|
| 飞控供电 | 电源方式、异常发热/重启、指示状态 | 无异常重启或发热 |
| `fcu_core` | USB 或 TCP 连接方式、启动命令、软件提交 | 能连接且保留控制台日志 |
| 官方 topic | `rostopic list/type/echo -n 1` 的输出 | 确认 `odom_global_001`、`imu_global_001` 等实际存在的 topic 与类型 |
| 时间 | 飞控消息时间与 ROS 接收时间 | 两者分别记录；无法比较时标 `not_measured` |

## 2. UWB 事实确认

- [ ] 四个基站的天线参考点、安装方向、通道/配置版本和是否固定已登记。
- [ ] App/官方日志/厂商答复中的标签安装对象已确认：`platform` / `target` / `unknown`。
- [ ] 官方 XYZ 的 frame、轴向、单位、位置参考点和更新方式有证据；未知字段保持 `TBD`。
- [ ] 已确认是否开放逐基站原始测距；未开放时禁用本项目第三方 LS/Chan/Taylor/TDOA 路线。
- [ ] 在一个固定点连续记录稳定性数据；只记录频率、跳变、丢失和恢复，不报告真值精度。

## 3. 视觉输入事实确认

- [ ] 每台候选相机的原始帧可访问，记录接口、分辨率、帧率和快门类型。
- [ ] 确认是否有每帧采样时间戳以及该时间戳的来源。
- [ ] 取得或可标定相机内参；缺失时不启动 PnP 或多相机三角测量。
- [ ] 在固定目标和安全状态下记录一次原始帧与 AprilTag/ArUco 检测日志；“无检测”必须明确输出，而不是复用上一帧。
- [ ] 多相机只有在各路原始帧、时间戳、内参和固定相机位置均具备后，才允许启用 `caliscope`。

## 4. ROS1→ROS2 映射签核

每条实际映射填一行，未确认不填入目标观测：

| ROS1 topic | ROS2 topic | semantic_object | frame/轴向 | 单位 | sample/receive 时间 | owner | 证据 |
|---|---|---|---|---|---|---|---|
| `/odom_global_001` | `/nexus/fcu/odom` | `platform`（默认） | TBD | TBD | 分列 | TBD | TBD |
| `/imu_global_001` | `/nexus/fcu/imu` | `platform` | `scaled_imu` 待核对 | SI 待核对 | 分列 | TBD | TBD |
| TBD | `/nexus/vision/target_observation` | `target_observation` | `camera_i` 或 `map` | m | 分列 | TBD | TBD |

## 5. 首日停止条件

任一项发生即停止进入定位比较：标签对象不明；frame/单位不明；相机无可靠时间戳；供电/固定/安全不满足；设备移动后仍计划沿用旧外参。此时只记录问题与证据，不能用猜测补全字段。
