# 2026-08-18 硬件到货前四阶段验证记录

## 环境

- Ubuntu 22.04 / WSL2
- ROS2 Humble
- Python 3.10、NumPy、OpenCV、pytest
- 当前环境没有 ROS1 Noetic 和真实飞行器/UWB/相机硬件

## 命令与结果

```text
source /opt/ros/humble/setup.bash && colcon list
结果：发现 6 个 ROS2 包（nexus_msgs 及 5 个自研包）。

source /opt/ros/humble/setup.bash && colcon build --symlink-install
结果：6 packages finished。

source /opt/ros/humble/setup.bash && colcon test
colcon test-result --verbose
结果：21 tests, 0 errors, 0 failures, 0 skipped。

python3 -m pytest -q
结果：23 passed（仓库 Python/ROS2 测试收集结果）。

code/analysis 下各工具及 code/tools/channel_contract_cli.py --help
结果：全部返回帮助信息。

git diff --check
结果：通过。
```

## 未验证项

未进行硬件验收、真实 UWB/相机接入、真实标定、ROS1 Noetic 实机联调、网络传输、现场实验、真实精度比较、EKF 调参、`motion_001` 回传或演示视频录制。因此本记录不包含任何精度结论或实验数据。
