# E040 飞控 TCP 四槽测距与 IMU 请求诊断

日期：2026-09-08（Asia/Shanghai）。状态：PARTIALLY COMPLETE。
授权：用户要求排查四槽、基站/tag 与遮挡，恢复实际 IMU 流，并允许修改。

## 环境与方法

- WSL Ubuntu 22.04，仓库基线 `232478ea42c695814247fe0b036b46b8bc204b41`，存在大量预先未提交修改，均保留。
- 直连 `192.168.1.126:333`，飞控心跳地址 `1/1`。本次心跳 base_mode=0。
- Windows/WSL 对 `pi@192.168.1.143` 的 BatchMode SSH 均被拒绝；22 端口开放，14551 拒绝连接。未访问机载串口。
- 新工具复用仓库 MAVLink C 头文件，不新增依赖。仅发 GCS HEARTBEAT 和遥测请求，没有飞行控制、参数写入、解锁或重启。
- 40 秒：前 10 秒基线；第 10 秒为 26/27/105 分别请求 20,000 us 周期；第 25 秒请求 RAW_SENSORS 50 Hz。请求只表示已发送，不表示飞控接受。
- `elapsed_s` 是接收循环的本机单调时间近似值，不是传感器采样时间。原始 payload 保留，未推定未知单位。

## 实测

有效原始文件：`data/raw/2026-09-08_E040_fcu_sensor_stream/tcp_probe_v03.jsonl`。
SHA-256：`9476002432ebcad004178bd1311fce6b636dc8743a5868b5260839a30cb8af42`。
详细统计见 `summary.json`。

收到 HEARTBEAT 40、BATTERY_STATUS 40、GLOBAL_VISION_POSITION_ESTIMATE 399、GLOBAL_POSITION_INT 399、SYSTEM_TIME 1、TIMESYNC 1。
SCALED_IMU / RAW_IMU / HIGHRES_IMU / COMMAND_ACK 均为 **0**。三个阶段均无实际 IMU。

四槽为 BATTERY_STATUS.voltages[2:6]，按既有厂商口径 cm 转 m；约 1.001 Hz，各槽 40 个样本，无 0/65535 哨兵。

| 槽 | 最小–最大 m | 标准差 m | 相邻变化 >0.25 m 次数（39 次比较） |
|---|---|---|---|
| 1 | 3.38–5.35 | 0.3243 | 4 |
| 2 | 0.60–5.44 | 1.5689 | 16 |
| 3 | 4.02–4.50 | 0.0726 | 2 |
| 4 | 2.19–2.26 | 0.0182 | 0 |

槽 2 优先排查，槽 1/3 也存在跳变；槽 4 此段相对稳定。非零只说明通过哨兵检查，不证明距离正确或物理基站在线。消息不包含已确认的逐基站在线状态、物理 tag ID 或 NLOS 指示；既有 tag=2 配置不是本轮读取的硬件身份。现场运动/遮挡未提供，不能把跳变归因于 NLOS、断电或 tag 冲突。

## 修改与验证

只新增 `code/tools/probe_fcu_sensor_stream.cpp` 和本运行记录/汇总；原始输出留在 data/raw，不提交设备数据。

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  -isystem code/ros1_ws/src/fcu_core_external/fcu_core/mavlink \
  code/tools/probe_fcu_sensor_stream.cpp -o /tmp/nexus_probe_fcu_sensor_stream
/tmp/nexus_probe_fcu_sensor_stream 192.168.1.126 333 request > /tmp/new_probe.jsonl
# 仅观察：将 request 替换成 observe（15 秒）。每次单独运行。
```

编译通过；40 秒实机请求运行退出 0；有效日志逐行 JSON 解析通过。未改 ROS 包，不涉及 catkin/colcon 构建。

最终源码重新编译后，以 observe 模式实测 15 秒并退出 0；`tcp_observe_final.jsonl` 共 328 条可解析消息，仍无 IMU。文件 SHA-256：`db6415b792eb3b11ff928bcc7cd397ff31369e7e9062ed7dc1912837fe1f8a87`。新增文件的差异与空白检查通过。

诊断过程中首版消息 ID 被 uint8_t 输出为字符，导致首份 tcp_probe.jsonl 不是有效 JSON，已修正并重采；该文件不参与统计。v02 在首轮连接尚未退出时打开第二条连接，未收到数据，不能作为飞控无流证据。最终工具增加零消息和未发现目标心跳的失败退出，避免这种情况误报成功。

## 当前阻塞与后续现场检查

IMU 未恢复。当前 TCP 上游没有 IMU，仅增加 ROS 解码不能解决；串口当前状态因 SSH 权限不可检查。需要恢复已有密钥登录，再核查机载桥接、厂商遥测配置与固件支持。无 ACK 不能证明指令被明确拒绝或固件一定不支持。

基站/tag 上电、ID、遮挡需现场观察或厂商界面证据。建议保持 tag 静止，先记录四基站指示灯/ID及无遮挡段，再逐个遮挡并标记时间，对比原始槽位；未执行物理操作，也未声称完成 LOS/NLOS 对照。

未修改飞控参数、固件、机载进程、既有 ROS 代码或历史结论；未扩大任务范围。
