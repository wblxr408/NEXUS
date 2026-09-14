# E043 实时 UWB 稳健均值输出

日期：2026-09-08。状态：IMPLEMENTED / PARTIALLY COMPLETE。软件与本地 ROS 消息链路已验证；树莓派启动服务返回 active 后网络断开，尚未取得线上平滑输出作为验收证据。

## 行为

新增 `nexus_pi_readonly_ingress/range_smoothing_node.py`，订阅 `/nexus/uwb/ranges`，输出 `/nexus/uwb/ranges_smoothed`，类型均为 std_msgs/msg/String（JSON）。不替换原始测距，不改 IMU 流、飞控参数或融合输入。输出是用户要求的静置窗口统计。

最近 200 组源时间戳递增的测距组成窗口；不足 200 组仍输出并置 ready=false。重复或倒序时间戳不计数、不刷新接收活性。每槽独立剔除非有限/非正/无效哨兵距离，再以 median/MAD 和 max(3*1.4826*MAD,0.01 m) 剔除离群值，求保留数据均值。满窗后每个新测距更新一次。标签/基站映射/时钟域变化或超过 5 s 断流后重建窗口。

JSON 的 ranges_m 为平滑均值，raw_ranges_m 为最新原始值；statistics 含各槽 kept/rejected/invalid、保留样本标准差和有效时间均值。window_samples、window_capacity、window_duration_s、ready、stale 公开窗口状态。sample_timestamp_ns 表示窗口最新输入时间，各槽均值的时间参考见 effective_timestamp_ns；不得将长窗均值直接作为瞬时融合测量。没有用样本标准差伪造测距精度或独立观测方差。

断流超过 5 s 发布一次 stale=true 且 ranges_m 为 null 的状态。1 Hz 下完整窗口约覆盖 199 s 的首尾间隔；200 组历史平均存在明显运动滞后。

## 实机部署

检查确认：Pi 的原始 /nexus/uwb/ranges 唯一发布者为 fcu_bridge_001，现有 ROS_DOMAIN_ID=20。原始桥接无需重启。

两份 Python 文件已复制到宿主机 `/home/pi/fcu_core_onboard/nexus_range_smoothing/nexus_pi_readonly_ingress/`，在既有 fcu_ros2 容器映射为 `/root/fcu_core_onboard/nexus_range_smoothing/nexus_pi_readonly_ingress/`。

安装并启用 `nexus-uwb-smoothing.service`，服务模板位于 `code/deployment/raspberry_pi_readonly_stack/nexus-uwb-smoothing.service`。`systemctl enable --now` 成功，`systemctl is-active` 返回 active；随后 SSH 连接停止响应，新连接两次返回 `[Errno 113] No route to host`。未读到平滑话题，不能将进程启动当作线上输出验收。

重连后的检查：

```bash
systemctl status nexus-uwb-smoothing.service --no-pager
docker exec fcu_ros2 bash -lc 'source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=20 ros2 topic echo /nexus/uwb/ranges_smoothed --once --full-length'
```

完整窗口应在连续接收约 200 s 后达到 window_samples=200、ready=true，之后保持 200。停止新增输出可运行 `sudo systemctl disable --now nexus-uwb-smoothing.service`，无需停止原飞控桥接。

## 验证

- WSL Ubuntu22.04 / ROS2 Humble：`colcon build --packages-select nexus_pi_readonly_ingress` 通过。
- `colcon test --packages-select nexus_pi_readonly_ingress` 与 `colcon test-result --test-result-base build/nexus_pi_readonly_ingress --verbose`：22 tests，0 errors，0 failures，0 skipped。完整输出见 validation.txt。
- 新增测试覆盖异常剔除、满窗滚动、重复包、无效槽、断流重置、标签变化及实际 ROS 发布/订阅和 stale 状态恢复。
- 实际 E042 180 组静置数据经生产 RobustRangeWindow 回放：各槽剔除数 [10,4,15,15]，均值 [5.0487058824,2.3551136364,4.0421212121,5.5225454545] m，与此前试算一致。结果与原始文件散列见 recorded_data_replay.json。
- 定向 git diff --check 通过；CMakeLists 中原有 camera 测试注册修改保留。本次仅添加 smoothing 可执行文件和对应测试注册。

未完成：网络恢复后的实机输出和满 200 组持续滚动验收。未扩大任务范围。
