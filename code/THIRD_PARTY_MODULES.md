# 第三方模块清单（2026-08-20）

本文件登记根据 `docs/module-map (1).html` 下载的上游 GitHub 源码。所有上游目录已登记为 Git submodule，并固定到下表提交；未修改任何上游文件，尚未将它们接入构建、启动或实验主链路。

克隆主仓库时使用 `git clone --recurse-submodules <repository-url>`；已克隆的工作区执行 `git submodule update --init --recursive`。更新 submodule 前必须先确认接口、许可证和实验需要，不能无记录地跟随上游最新提交。

| 模块 | 本地位置 | 上游提交 | 许可证 | 当前归类 |
|---|---|---|---|---|
| `fancinnov/fcu_core` | `ros1_ws/src/fcu_core_external/fcu_core` | `14e228c63cfe2bbdecebda8c4f1fc259f295e0aa` | 上游未见许可证文件，使用/分发前须向维护方确认 | 官方 ROS1 硬件基线 |
| `christianrauch/apriltag_ros` | `ros2_ws/src/apriltag_ros` | `beffb4c73bdd04fb6347d612e3306f75bb06c7e7` | MIT | ROS2 AprilTag 检测候选 |
| `RobotWebTools/rosbridge_suite` | `third_party/rosbridge_suite` | `aa9a7a33ddb3b1b45ddd6d2eead4c3d5eb800b14` | BSD-3-Clause | ROS2 浏览器数据出口（不纳入项目自有工作空间构建） |
| `cliansang/uwb-tracking-ros` | `analysis/uwb/third_party/uwb_tracking_ros` | `046a757bcf60b3a952f7724b5ee73f28e6a5d5b5` | MIT | 原始 UWB 数据的参考，不接入飞控链路 |
| `madfolio/Least-Squares-Trilateration` | `analysis/uwb/third_party/least_squares_trilateration` | `e112b8d3356b271843d5ee667e928cc4056a5f4d` | 上游未见许可证文件，使用/分发前须确认 | 原始测距离线基线 |
| `cliansang/positioning-algorithms-for-uwb-matlab` | `analysis/uwb/third_party/positioning_algorithms_for_uwb_matlab` | `2c24c478ae0c317840fec1c29631c15dbe29a630` | MIT | 原始测距离线算法参考 |
| `mprib/caliscope` | `analysis/calibration/third_party/caliscope` | `6819acc4a4855fa462e034d3f3a213c9a8adde27` | BSD-2-Clause | 条件性多相机标定/三角测量工具 |
| `THU-DA-6D-Pose-Group/GDR-Net` | `analysis/vision/third_party/gdr_net` | `1be9fe73292fd748087aa88d7bf987434f271ebb` | Apache-2.0 | 单目、已知目标几何的 6D 位姿候选；上游权重与 BOP 数据集尚未在本项目验证 |

`fcu_core` 和 `rosbridge_suite` 是当前硬件/展示链路的迁移项；`apriltag_ros` 是延后的工程基线，不是当前无标签主线。后三项（除 `uwb_tracking_ros` 外）需要适配，`uwb_tracking_ros` 虽标为可迁移，但与当前 FanciSwarm 的官方已解算位置接口不同，因此仅保留为原始观测可用时的参考。具体前置条件和后续修改范围见当前执行基线与各目录旁的适配说明。

不得把 `/odom_global_001` 或任何飞行平台状态自动当作赛题目标位姿；目标主输出仍只能是 ROS2 的 `/nexus/target/pose`。未获得厂商原始四基站测距前，不启用任何第三方 UWB 解算模块。
