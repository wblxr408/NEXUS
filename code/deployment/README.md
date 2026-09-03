# 机载部署件

这里放**直接部署到机载计算机（树莓派）**上运行的代码与说明。每个子目录自带 README，
说明安全边界、启动命令和本机验证方式。

- `raspberry_pi_e016_self_localization/`：E016 自定位相关的 IMX219 实机相机输入接口
  （只取帧，不碰飞控，见 `CAMERA_RUNTIME.md`）。
- `raspberry_pi_uav_readonly_adapter/`：幻思 Mcontroller 飞控**只读**遥测适配器
  （TCP/MAVLink → JSONL；唯一外发消息是 GCS `HEARTBEAT`；口径见 ADR-010）。
- `install_domain_bridge.sh`：安装并验证 ROS 2 `domain_bridge`。开发机和树莓派都在
  **各自设备上**执行该脚本，不需要把 amd64 的 deb 包复制到 arm64 树莓派。

## 安装跨域桥

本机 Linux（ROS 2 Humble）：

```bash
./code/deployment/install_domain_bridge.sh
```

树莓派需运行 64 位 Ubuntu 22.04、已安装 ROS 2 Humble 并配置 ROS apt 源；把仓库复制到
树莓派后执行同一命令即可。脚本会按设备架构从 apt 获取对应的 `ros-humble-domain-bridge`
包，重复运行是安全的。安装后可用以下命令确认：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix domain_bridge
```

若树莓派运行的是 Raspberry Pi OS 或 32 位系统，ROS 官方 Humble deb 通常不可用；请先
改用 64 位 Ubuntu 22.04（或为目标 ROS 发行版配置对应的 ROS apt 源），再运行脚本。该
脚本不会静默跳过安装失败。

按 `AGENTS.md`：模型权重、rosbag、采集数据等大文件不入 Git，只在 `data/`、`outputs/`
中登记外部位置与校验信息；本目录也不得提交任何口令或凭据（设备 IP/端口不算凭据）。
