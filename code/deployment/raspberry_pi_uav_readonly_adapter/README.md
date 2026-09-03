# 无人机只读遥测适配器（机载树莓派部署件）

该程序运行在无人机树莓派（`raspberrypi`，aarch64，用户 `pi`）上，通过 TCP/MAVLink 连接幻思
Mcontroller 飞控，把厂商遥测投影成一套稳定的逐行 JSON（JSONL）。

它是**只读遥测适配器**，不是控制器。相关口径裁定见
[ADR-010](../../../docs/decisions/2026-09-01_ADR-010_rrd_sandbox_and_hardware_uwb_range_availability.md)。

## 安全边界（硬约束）

- 唯一发送的 MAVLink 消息是 GCS `HEARTBEAT`（`uav_readonly_adapter.py`
  的 `_send_readonly_gcs_heartbeat`），用于让厂商飞控开始回传遥测。
- 没有解锁、起飞、降落、移动、任务上传和参数写入接口。搬入本仓库时未做任何重构，
  语义与实机跑通的版本一致。
- TCP 断开或连续 5 秒没有遥测时，按 1、2、4、8、10 秒退避自动重连。
- Mcontroller 的 `192.168.1.126:333` 通常**只允许一个 TCP 客户端**；使用本程序时
  不要同时连接厂商 App。
- 首次实机验证保持**动力电池断开、App 断开、飞控锁定**。
- 本目录不含任何口令或凭据；飞控 IP/端口只是设备地址，可以入库。

## 运行

树莓派上已有虚拟环境 `~/mavlink_env` 和 `pymavlink`：

```bash
source ~/mavlink_env/bin/activate
python3 uav_readonly_adapter.py --frame-id uwb_raw
```

写入 JSONL 日志（建立运行记录时必须落盘，不要只看终端）：

```bash
python3 uav_readonly_adapter.py \
  --frame-id uwb_raw \
  --output <experiments/runs/...>/drone_001.jsonl
```

给 ROS2 在线链路分发时，可在保留 JSONL 记录的同时广播 UDP；适配器仍是唯一的
MAVLink/TCP 读取者，ROS2 侧只监听 UDP，不会再次打开飞控串口或 TCP：

```bash
python3 uav_readonly_adapter.py \
  --frame-id uwb_raw \
  --output <experiments/runs/...>/drone_001.jsonl \
  --udp-host <ros2_host> --udp-port 14551
```

按 `Ctrl+C` 安全停止。全部参数见 `python3 uav_readonly_adapter.py --help`。

关于 `--frame-id`：程序默认值为 `uwb_raw`，符合
`docs/architecture/coordinate-frames.md:15` 的「未知厂商 UWB 坐标一律记为
`uwb_raw`」。未经测量的 `uwb_raw → map` 变换不得由适配器或 ROS2 遥测节点猜测。

## 本机验证

本机（开发机）没有 `pymavlink`，但单测不依赖它：`mavutil` 由
`ReadOnlyMavlinkAdapter._load_mavutil()` 在 `run()` 里**延迟导入**，
而单测只构造假 message 对象喂给 `StateProjector`。

```bash
cd code/deployment/raspberry_pi_uav_readonly_adapter
python3 -m unittest -v
```

2026-09-01 在本机实跑：6 个用例全部通过（`OK`），环境无 `pymavlink`。
这只证明**字段投影与单位换算正确**，不证明任何实机精度（`AGENTS.md:35`）。

## 实机发现（2026-09-01，已核实）

1. **厂商确实暴露原始 UWB 测距。** 四个基站距离由厂商复用
   `BATTERY_STATUS.voltages[2:6]` 传输，单位 cm（除以 100 得米）；`0` 与 `65535`
   是「不可用」哨兵值。这**推翻了 ADR-001 中「FanciSwarm 可能不暴露原始 UWB 距离、
   只能拿厂家解算位置」的警告**（ADR-001 原文不改，取代关系在 ADR-010 中声明）。
   同一 `BATTERY_STATUS` 里 `voltages[1]` 是 App 显示的电池电压（mV），`voltages[0]`
   未使用。
2. **厂商解算位置是二维的。** 位置来自 `GLOBAL_VISION_POSITION_ESTIMATE`，`x`/`y`
   单位 cm，**没有可信 z**。注意该消息在标准 MAVLink 里是**伴随机发给飞控**的视觉位姿
   *输入*，这里却由飞控回传，属**厂商复用/回显，语义待厂商确认**，不得按标准语义解读。
   垂直方向必须另有来源。
   适配器仅投影 `x`/`y`，并明确保持 `position.z_m=null`；未验证字段不能作为高度输入。
3. **两条物理通道都存在。** 本适配器走 TCP/MAVLink 连 `192.168.1.126:333`；
   树莓派上另有 `~/read_uwb.py` 走串口 `/dev/ttyAMA0` @ 115200。TCP 侧通常只允许
   一个客户端，用本程序时不能同时连厂商 App。
4. **只读边界。** 见上面「安全边界」。
5. **锚点坐标仍未实测。** 有原始测距**不等于**能出坐标：一组距离要变成位置必须先知道
   四个基站在 `map` 中的坐标，那是**基准量（gauge）**，误差是系统误差、不随帧数减小。
   实机基站坐标至今未 survey。

## `uwb.anchor_ranges_m` 与 ADR-009 F6 的对应关系

- ADR-009 第 5 项把 UWB 的角色限定为两处：**段级尺度/基准因子 F6** 与视觉不可用时的
  **兜底通道**；并明确 **UWB 不进逐帧目标深度回路**。
- 本适配器的 `uwb.anchor_ranges_m`（四个 `float | null`，单位 m）正是 F6 在**实机**上
  的真实输入——ADR-009 裁定 F6 时仓库里只有仿真距离，现在有了硬件距离。
- 但 F6 仍**跑不起来**：F6 需要「距离 + 锚点坐标」，而实机锚点坐标未实测（发现 5）。
  ADR-009 第 5 项裁定的那组四点是**参数化沙盘的仿真设计值**
  （`survey_status: simulation_only_not_surveyed`），**不得套到实机基站上**。
  ADR-010 第 2 项把三套锚点分环境隔离。
- 索引到基站的映射（`anchor_ranges_m[0..3]` 是否就是 `anchor_0..anchor_3`）
  **未经实机验证**，是厂商顺序假设；F6 上线前必须用已知几何标定确认。

## 已知的其他限制（进入运行记录前必须知道）

- **没有逐字段新鲜度。** `StateProjector` 的 `state` 是累积的：某个字段一旦写入就一直
  被复用，只要任意一条消息到达 `online` 就是 `true`。所以一份 JSONL 行里的
  `anchor_ranges_m` 可能比 `timestamp` 旧很多，无法从数据本身判断。
- **时间只有主机墙钟**（`timestamp` / `timestamp_iso`），没有飞控时间戳，也没有与相机
  的时间同步；与视觉帧对齐的可行性未验证。
- **JSONL 没有头部元数据**（配置、代码版本、环境）。因此单独一份 `.jsonl`
  不满足 `AGENTS.md:7` 对运行记录的要求，必须另配 `experiments/runs/` 目录说明。

## 为什么正式适配器才是可用件（对照 `~/read_uwb.py`）

树莓派上那个临时验证脚本 `~/read_uwb.py`（78 行）**不可用于任何记录**：

- **不检查 `0` / `65535` 哨兵**：一个不可用的距离会静默变成 `655.35 m`。
  本适配器在 `uav_readonly_adapter.py:167` 正确置 `None`，并由
  `test_unavailable_uwb_ranges_become_null_instead_of_655_metres` 锁定。
- **直接索引 `voltages[2:6]`**：飞控少发字段就 IndexError。本适配器有
  `len(voltages) >= 6` 保护。
- **只取 x/y、丢掉高度、无时间戳、不落盘**，只往终端打印。

因此当前「UWB 已校验」**只是一次控制台演示**，`experiments/runs/` 里没有任何可追溯
记录。按 `AGENTS.md:35` 与 ADR-010 第 6 项：在补出带数据、配置、代码版本和结论的运行
记录之前，**不得在答辩材料里表述为「UWB 已验证」**。

## 输出字段

- `schema_version`、`agent_id`、`mission_id`、`task_id`、`timestamp`：跨 topic 通用字段。
- `online`、`armed`、`flight_mode`：连通与锁定状态（`armed` 来自 `HEARTBEAT.base_mode & 128`）。
- `position`：厂商 `GLOBAL_VISION_POSITION_ESTIMATE` 的厘米值转米；只信 `x_m`/`y_m`。
- `attitude_rad`：同一消息的 roll/pitch/yaw（弧度，厂商语义待确认）。
- `velocity_mps`：`GLOBAL_POSITION_INT` 的 cm/s 转 m/s。
- `battery`：`voltages[1]`（mV→V）、`current_battery`（cA→A）、`battery_remaining`（%）。
- `uwb.tag_id`、`uwb.anchor_ranges_m`：厂商复用 `BATTERY_STATUS.voltages[2:6]` 的四个
  基站距离（m，不可用为 `null`）。
- `source`：协议、传输、endpoint、最后一条消息类型。

当前输出是通信中立的统一 JSON；`--udp-host/--udp-port` 是给 ROS2
`nexus_bringup/telemetry_udp_node` 的只读分发出口，不改变飞控解析逻辑。
