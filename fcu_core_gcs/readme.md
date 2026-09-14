## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS2**
* Ubuntu 64-bit 22.04.
* ROS Humble. [ROS2 Installation](https://docs.ros.org/en/humble/)

### 1.2. **serial**
* cd fcu_core_ros2_ws/src/serial_ros2
* mkdir build && cd build
* cmake ..
* make -j$(nproc)
* sudo make install

### 1.3. **eigen**
* sudo apt-get install libeigen3-dev

## 2. Build
* cd fcu_core_ros2_ws
* colcon build --packages-select quadrotor_msgs
* source install/setup.bash
* colcon build

## 3. Source
* source install/setup.bash

## 4. 如果用到串口需要配置权限
* sudo chmod 777 /dev/ttyACM0

## 5. 运行launch文件
* ros2 launch fcu_core fcu_core_launch.py

## 6. 命令行横移功能

命令行模式通过 `/command` 向机载 `fcu_mission` 选择往复横移功能：

* `i`：发布命令 `13`，以按键时的位置为基准，沿沙盘固定 X 轴在 `+0.15 m` 与 `-0.15 m` 之间往复。
* `o`：发布命令 `14`，以按键时的位置为基准，沿沙盘固定 Y 轴在 `+0.15 m` 与 `-0.15 m` 之间往复。
* 机载端在接到命令时记录当前位置；每个最远点等待无人机进入 `0.02 m` 位置容差后停留 5 秒，共往复 5 次，最后回到命令起始位置。
* 横移期间保持起始时的另一水平轴坐标、高度和偏航；按 `s` 可中止并悬停。
* 横移执行期间再次按 `i` 或 `o`，会立即停止旧任务，并以当时位置为新原点重启所选轴任务，无需先按 `s`。

机载默认移动速度为 `0.06 m/s`。没有新鲜里程计时功能会拒绝启动；执行中里程计超过
1 秒未更新时会停止继续推进目标点。GCS 会显示 `/axis_sweep_status` 的实时反馈，包括命令
接受或拒绝、当前阶段、循环次数、绝对目标位置、实测位置和相对起点位移。

RViz 控制面板提供“X轴往复”和“Y轴往复”按钮，分别发布相同的 `13` 和 `14`
命令；执行中的任务也会按上述规则从点击时的当前位置重新启动。

## 7. 起飞自动相机回传

使用 `./start_gcs.sh` 启动命令行地面站。相机接收器启动后先保持等待；按 `t` 发布
起飞命令时，自动连接树莓派 `192.168.1.143:14552`，并在图像窗口持续显示
`/nexus/camera/imx219/image_raw`。巡航、横移和悬停不会中断视频；按 `l` 降落或按
`d` 上锁后自动断开。无图形界面时可设置 `NEXUS_CAMERA_VIEW=0` 关闭图像窗口，ROS
图像话题仍按相同规则启停。
