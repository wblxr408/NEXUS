# E029：实机静置 UWB/IMU 只读采样

状态：**PARTIALLY COMPLETE / BUG**。本次在无人机静置、未解锁、未起飞条件下采集真实飞控遥测；确认连接与 UWB 字段均有数据，但当前 UWB 槽位语义未通过一致性检查，且未收到可用的 `SCALED_IMU`，因此不得进入融合。

## 范围与输入

- 只读适配器连接飞控遥测端点，仅发送厂商要求的被动 GCS heartbeat；没有发送解锁、模式、速度、高度、航点或控制命令。
- 原始 JSONL：`data/raw/2026-09-07_E029_static_uwb_imu/telemetry.jsonl`（Git 忽略），SHA-256：`90e8077a57035ca62e17759e6e1f16bf9e22c576cc055bcd81c206f82760f798`。
- 使用用户确认的标签 ID 2 和四锚点 `map` 坐标；条件只记录为 `static_unclassified_los_nlos`，未把它声称为已确认 LOS 或 NLOS。
- 分析输出：`data/processed/2026-09-07_E029_static_uwb_imu/summary_v02.json`（Git 忽略）。

## 真实结果

- 共 2,596 条遥测快照，飞控在线状态 2,595 条为 `true`。
- UWB：161 条去重后的四锚点完整测距，采样率中位数 1.006 Hz；四槽测距标准差为 2.415、1.270、0.717、2.356 m，0.25 m 阈值以上跳变 142 次。
- 按登记的锚点 ID/坐标求解时，只有 55/161 条满足 0.25 m 内部测距拟合残差门限；中位拟合残差 0.552 m，全部样本 RMS 1.039 m。因此当前 `BATTERY_STATUS.voltages[2:6] → anchor 1..4 range` 的槽位/单位映射**未验证且不可用于定位融合**。
- IMU：0 条具有可用源时间戳的 `SCALED_IMU` 样本；IMU 频率、静态噪声、偏置和 UWB—IMU 融合均未完成。

## 下一步实机操作

1. 在飞控/地面站中确认可开启的原始 IMU 消息类型和发送频率，优先 `SCALED_IMU`；若只能输出 `HIGHRES_IMU` 或其他消息，先取得一段真实报文，再按其单位和时间基准扩展只读适配器，不能猜单位。
2. 在一个已测量的固定 `map` 点放置无人机，记录标签中心的 XYZ 坐标及标签相对 `base_link` 的杆臂；静置录制 120 秒。以该点到四个锚点的几何距离逐槽匹配实际显示的测距，确认槽位对应、单位、常量偏置和跳变规律。
3. 槽位确认后，在同一点分别录制无遮挡 LOS 与人为遮挡一个基站的 NLOS，各 120 秒；记录遮挡的锚点 ID 和遮挡物，而非仅凭数值给数据贴标签。
4. IMU 消息恢复后，静置录制至少 30 分钟用于噪声密度/偏置随机游走（Allan 偏差），再做桌面上的匀速直线和缓慢转动手持轨迹。最后才进入 UWB—IMU 融合与起飞验证。

## 复现命令

```bash
python3 code/analysis/uwb/analyze_static_uwb_imu.py \
  --input data/raw/2026-09-07_E029_static_uwb_imu/telemetry.jsonl \
  --hardware-config code/ros2_ws/src/nexus_pi_readonly_ingress/config/hardware_user_v02.yaml \
  --output data/processed/2026-09-07_E029_static_uwb_imu/summary_v02.json \
  --condition static_unclassified_los_nlos
```
