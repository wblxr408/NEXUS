# 到货首日硬件证据自动采集

- 日期：2026-08-22
- 类型：guide
- 状态：accepted
- 关联：[硬件首日验收清单](2026-08-20_checklist_hardware_first_day_acceptance.md)、`code/tools/hardware_acceptance_capture.py`

## 目的和边界

本工具在 ROS1 Noetic 硬件兼容主机上，将首日验收的可复查事实写入指定运行目录的 `artifacts/acceptance/`。它执行只读的 `rostopic list`、`rostopic type`、`rostopic echo -n 1`，并记录 ROS/Git 版本、主机时间同步状态、SD CSV 表头、大小与 SHA256。

工具不会复制 rosbag、SD CSV 正文、视频、设备序列号或网络凭据；原始文件仍按运行记录登记外部位置与 SHA256。它不会从 ROS 接收时间推断飞控采样时间、坐标系、单位或标签对象。未知事实保持 `TBD`/`unknown`，不得据此形成精度结论。

## 到货首日命令

先完成安全状态、上电和 `fcu_core` 连接检查；在能够执行 `rostopic` 的 Noetic 终端中运行。`--output-dir` 应属于新建的验收运行目录，不能覆盖既有运行。

```bash
cd ~/nexus_workspace/NEXUS
source /opt/ros/noetic/setup.bash
source code/ros1_ws/devel/setup.bash

python3 code/tools/hardware_acceptance_capture.py \
  --output-dir experiments/runs/YYYY-MM-DD_E000_hardware_acceptance/artifacts/acceptance \
  --topic /odom_global_001 \
  --topic /imu_global_001 \
  --sd-csv /approved-external-path/fcu_sd_log.csv \
  --device-time-source TBD \
  --strict
```

在确认厂商资料和一条实际消息的时间语义后，才可将 `--device-time-source TBD` 替换为具体值；该值要同步写入运行记录。若厂商 SD 日志尚未导出，可省略 `--sd-csv`，工具仍生成 ROS 侧证据。`--strict` 会在 ROS topic 不存在、首条消息未收到、或给定 SD 文件无法读取表头时返回非零；即使失败，已生成的证据文件也必须保留并在运行记录中说明。

## 生成物与人工签核

| 文件 | 内容 | 不能代表什么 |
|---|---|---|
| `rostopic_list.txt` | 实际 ROS1 topic 列表 | 目标定位语义 |
| `topic_*.type.txt` / `topic_*.echo.yaml` | 请求 topic 的类型与首条消息 | 频率、精度或稳定性 |
| `versions.json` | ROS 与当前 Git 提交的命令输出 | 硬件兼容性结论 |
| `time_source.json` | 主机 UTC、NTP 状态、`/use_sim_time` 与设备时间源待确认项 | 飞控与 ROS 时钟已同步 |
| `metadata_draft.json` | SD 文件头、大小、SHA256 和待确认字段 | 可直接用于评估的标准化数据 |
| `capture_summary.json` | 采集完整/部分状态与失败项 | 验收通过或精度结论 |

随后由操作者在运行记录中填写设备、操作者、外部原始数据位置、UWB 标签对象、frame/轴向/单位和厂商 SD 字段解释。只有这些事实与独立标定、真值和正式实验运行齐全后，数据才能进入定位评估。
