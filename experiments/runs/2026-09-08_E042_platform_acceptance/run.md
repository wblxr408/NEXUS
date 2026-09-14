# E042 IMU 与平台定位实机验收

日期：2026-09-08。状态：**PARTIALLY COMPLETE / 实机融合验收未通过**。

用户要求完成 IMU 噪声、时间戳、机体坐标、UWB–IMU 融合以及静止/匀速/转动检查。软件修改、实机采集与真实数据回放均已执行；不能将执行完测试写成验收通过。

## 已完成工作

- 树莓派原始串口采集：先确认飞控未解锁，停止单个桥接，确认退出后独占 `/dev/ttyAMA0 @ 115200`，仅发送 ONBOARD_CONTROLLER 心跳；结束后恢复原参数/话题的桥接。未发送飞行指令、参数写入或固件重启。
- 用户确认静止后，完整采集 180 s；随后按用户回复分别标记俯仰、侧倾、偏航、平移与结束静置。
- 保留原始六轴、源时钟、接收时刻、四槽距离、飞控姿态和 BAD_DATA，未填补缺失样本。
- 平台节点允许从校准文件载入机体坐标下的初始陀螺/加速度 bias，缺省行为仍为零。
- 用户允许近似外参后，新增显式 `experimental_approximation` 校准状态及默认关闭的 opt-in；状态消息公开近似项。未把近似配置升级成 measured。
- 用既有生产 `PlatformWindow` 回放全部五段真实数据，不用仿真替代实测验收。

## 用户条件与修正

详见 `operator_conditions.json`：机头为前方；基站高度零点为沙盘底部；9° 地图偏航偏置保持；UWB–IMU 杠杆臂未知，用户允许近似，实验暂用共点模型并明确记录。

用户最初估计标签高于沙盘底部 12–14 cm，后来明确表示标签高度不确定，App 高度约 2 cm。后者的测量来源和物理参考点未确认，不能代替标签天线中心高度。因此回放中的 z=0.13 m 是**不确定初始化假设**，不是独立真值；本轮失败表明当前近似配置不能验收，不等于已证明正确标定下算法必然失败。

第一次平移往返抬高了机体，第二次保持高度。整段不能标为严格定高匀速，速度/距离没有独立测量真值。

## IMU 短时噪声与时间戳

成功静置段共 **34021 条 SCALED_IMU / 180.0 s / 189.0 Hz**；源采样间隔为 5–15 ms，无超过 50 ms 的缺口。

| 量 | x | y | z |
|---|---:|---:|---:|
| 加速度样本标准差 m/s² | 0.004251 | 0.003956 | 0.012211 |
| 陀螺样本标准差 rad/s | 0.0006083 | 0.0005420 | 0.0007012 |
| 静置陀螺偏置 rad/s | -0.0065783 | -0.0009980 | -0.0188573 |

加速度模长均值 9.8292 m/s²。完整轴向均值、短时噪声密度估计及重采样 Allan deviation 见 `stationary_summary.json`。单姿态不能分离全部加速度偏置和倾斜，模型仅作沿重力方向的模长修正；偏置随机游走仍保留现有模型默认值，未声称测得长时温漂。

噪声定义参考 [Kalibr IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)：白噪声密度与样本标准差之间包含采样周期因子，不能直接把标准差当作连续时间密度。这里均明确区分估计、模型值与实测样本统计。

源时钟相对接收时钟的拟合漂移约 -20.42 ppm，接收拟合残差 99% 分位约 7.24 ms。该拟合不提供绝对传感器延迟。

## 机体轴向与转动时间对应

用户完成抬机头、抬左侧和俯视逆时针转动；核对既有 `diag(1,-1,-1)` 原始轴变换：

- 抬机头主要为负 y 角速度；存在一段缺口，累计角只作为不完整积分，不作精确角度验收。
- 抬左侧主要为正 x，最大累计约 23.8°。
- 俯视逆时针主要为正 z，最大累计约 98.5°，与用户约 90° 操作相符。

这验证了轴符号与主轴对应，没有完成精密安装旋转外参测量。9° 是既有地图偏置，没有被这次转动检查重标定。

厂商 GLOBAL_VISION_POSITION_ESTIMATE.usec 的实际增量与毫秒时钟一致，分析先验证其速率再使用，未按标准字段名称假定微秒。将飞控姿态变化与 IMU 对比，主要运动轴的相关系数约 0.95–0.99，最佳相对时差约 -5～10 ms。飞控姿态是相关估计，不是外部真值；这个结果不能证明没有 UWB 测量延迟。

运动采集中记录了 **500 ms（俯仰）和 70 ms（平移）** 两处源 IMU 缺口。角积分明确略过缺口并标记不完整；时间相关计算不跨缺口插值。融合保持原 50 ms 拒绝阈值。

## UWB 与融合验收结果

静置四槽仍有异常跳变，完整统计见 `stationary_summary.json`。按先前 12–14 cm 标签高度假设，槽 2 的几何下限为 2.57 m，而中位距离为 2.355 m（176/180 组低于下限）。由于标签高度后来被声明为不确定，此项只能作为**条件性不一致检查**，不能直接用于估算固定补偿或证明基站高度错误。

四槽非零、App 有位置、数值平滑均不等于正确距离。固定测距偏差无法仅由未知标签位置的一组数据唯一确定；米级瞬变也不能用一个常数补偿消除。

生产融合器在 `platform_experimental.yaml` 明示近似条件下回放结果如下；“valid”仅指估计器状态标记，不代表已经满足物理验收：

| 段 | 更新数 | 估计器 valid 数 | valid 输出相邻最大位置步长 m |
|---|---:|---:|---:|
| 静置 | 179 | 63 | 0.714 |
| 俯仰 | 104 | 37 | 0.525 |
| 侧倾 | 57 | 3 | 3.494 |
| 偏航 | 176 | 6 | 0.690 |
| 平移 | 133 | 31 | 4.744 |

失败状态包括外部约束丢失、IMU 缺口、速度限制以及一次求解迭代耗尽，逐条保留于 `*_fusion_full.json`。偏航回放的估计器累计耗时约 382.7 s，长于约 178 s 数据时长，当前配置也不满足这段的实时性要求。没有通过扩大缺口阈值、删除异常或补造数据让验收变绿。

尚未将此失败的近似融合启动为正常定位输出。IMU 原始消息流和原机载桥接保持恢复状态。

## 改动与检查

本轮源码：

- `code/analysis/uwb/platform_acceptance.py`：源时钟、噪声、Allan 与四槽统计。
- `code/analysis/uwb/analyze_platform_motion.py`：带缺口标记的轴向积分与姿态相对时差。
- `code/analysis/uwb/replay_platform_acceptance.py`：真实数据驱动现有平台估计器并保留失败。
- `code/analysis/test/test_platform_acceptance.py`：白噪声 Allan、时钟漂移/单位、已知时延回归，3 项通过。
- `code/ros2_ws/src/nexus_fusion_localization/` 中 platform_node、platform launch 和对应测试：实测初始 bias 与显式近似 opt-in。
- ADR-019 与本次记录/配置/统计。

在 WSL Ubuntu 22.04 / ROS2 Humble 中执行：

```bash
source /opt/ros/humble/setup.bash
cd code/ros2_ws
OPENBLAS_NUM_THREADS=1 colcon build --packages-select nexus_fusion_localization
OPENBLAS_NUM_THREADS=1 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA colcon test --packages-select nexus_fusion_localization
colcon test-result --test-result-base build/nexus_fusion_localization --verbose
```

构建通过；包检查 **90 tests, 0 errors, 0 failures, 0 skipped**。详见 `build.txt`、`tests.txt`。较早测试调用错误覆盖了 ROS PYTHONPATH，导致 import rclpy 失败；修正调用环境后上述检查通过，未更改断言来掩盖失败。

分析命令（仓库根目录）：

```bash
python3 code/analysis/uwb/platform_acceptance.py --input data/raw/2026-09-08_E042_platform_acceptance/stationary_v02.jsonl --condition stationary_confirmed --output /tmp/static_summary.json
python3 code/analysis/uwb/analyze_platform_motion.py --input data/raw/2026-09-08_E042_platform_acceptance/motion.jsonl --stationary-summary experiments/runs/2026-09-08_E042_platform_acceptance/stationary_summary.json --output /tmp/motion_summary.json
PYTHONPATH=code/ros2_ws/src/nexus_fusion_localization/src OPENBLAS_NUM_THREADS=1 python3 code/analysis/uwb/replay_platform_acceptance.py --input data/raw/2026-09-08_E042_platform_acceptance/motion.jsonl --phase translation --calibration experiments/runs/2026-09-08_E042_platform_acceptance/platform_experimental.yaml --stationary-summary experiments/runs/2026-09-08_E042_platform_acceptance/stationary_summary.json --output /tmp/translation_fusion.json
```

## 证据与未完成项

原始数据位于 `data/raw/2026-09-08_E042_platform_acceptance/`，逐文件 SHA-256 见 `raw_manifest.json`；代码散列见 `source_manifest.json`。首次静置采集因 BAD_DATA 中 bytearray 不能 JSON 序列化而失败并恢复桥接，修正后重新完整采集，失败日志保留。实验脚本使用输出独占创建并会暂停桥接，不得在飞行时直接重跑。

未完成：独立标签位置/天线高度及测距偏差确认、运动缺口原因、UWB 测量延迟、通过实机连续性/实时性验收的融合配置、长时 IMU 噪声模型。需要安装实景和独立测距点进一步区分硬件、标定与模型问题，不能自行把 App 高度或拟合位置当作真值。

保留其他成员的预先修改；没有改基站历史坐标、飞控参数、地图偏置、固件或飞行控制。未扩大任务范围。
