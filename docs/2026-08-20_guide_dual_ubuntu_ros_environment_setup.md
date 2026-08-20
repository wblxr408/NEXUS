# Ubuntu 22.04 + ROS 2 与 Ubuntu 20.04 + ROS 1 配置教程

日期：2026-08-20  
适用仓库：NEXUS  
适用阶段：硬件到货前的软件准备，以及到货首日的环境验收

## 1. 先确定部署方式

本项目使用两套**彼此独立**的受支持环境。不要在同一个 Ubuntu 安装中同时混装 ROS 1 Noetic 和 ROS 2 Humble；推荐使用两台主机，或同一台电脑双系统/两台虚拟机。

| 环境 | 操作系统 | ROS | 本项目职责 | 是否连接飞控 |
| --- | --- | --- | --- | --- |
| ROS 2 算法主机 | Ubuntu 22.04 LTS | ROS 2 Humble | AprilTag 视觉、坐标变换、融合、评估、RViz2 与演示 | 到联调阶段才经显式 bridge 接收数据 |
| ROS 1 硬件兼容主机 | Ubuntu 20.04 LTS | ROS 1 Noetic | 幻思 `fcu_core`、飞控状态读取、硬件录包 | 是，到货验收时连接 |

`fcu_core` 发布的 `odom_global_001` 是飞行平台状态；在未确认 UWB 标签实际安装对象和消息语义前，**它不是外部 `target_0` 的位置**。坐标、时间和拒绝规则见[接口指导](project_steps/2026-08-20_guide_step_02_interfaces_and_conventions.md)。

ROS 1 Noetic 已结束常规支持，因此只在 Ubuntu 20.04 的厂商硬件兼容层使用；新的算法工作放在 Ubuntu 22.04 + ROS 2 Humble。

## 2. 下载地址与安装介质

在另一台可联网电脑或目标电脑中下载以下 ISO。选择桌面版（Desktop），64 位 `amd64`；下载完成后在下载页核对 SHA256。

| 目标 | 下载页面 | 应下载文件 | 说明 |
| --- | --- | --- | --- |
| ROS 2 主机 | [Ubuntu 22.04 LTS 发布页](https://releases.ubuntu.com/22.04/) | `ubuntu-22.04.x-desktop-amd64.iso` | 最终应显示代号 `jammy` |
| ROS 1 主机 | [Ubuntu 20.04 LTS 发布页](https://releases.ubuntu.com/20.04/) | `ubuntu-20.04.x-desktop-amd64.iso` | 最终应显示代号 `focal` |
| ROS 2 安装说明 | [ROS 2 Humble Ubuntu 二进制安装页](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) | 不单独下载 | 本文命令以此为准 |
| ROS 1 安装说明 | [ROS Noetic Ubuntu 安装页](http://wiki.ros.org/noetic/Installation/Ubuntu) | 不单独下载 | Noetic 仅对应 Ubuntu 20.04 |
| 厂商 ROS1 驱动 | [fancinnov/fcu_core](https://github.com/fancinnov/fcu_core) | 由本仓库 Git 子模块取得 | 上游依赖 `quadrotor_msgs` |

用 Rufus、balenaEtcher 或 Ubuntu 的“启动盘创建器”把 ISO 写入一个空 U 盘。写入完成后从 U 盘启动，选择安装 Ubuntu。为避免把系统装进错误磁盘，安装器出现分区页面时先确认磁盘型号与容量；重要数据先做备份。

下载页面会给出与所选 ISO 精确对应的 SHA256。不要把不同小版本的校验值混用；在下载目录执行下面命令，将输出逐字符与该页面的同名 ISO 校验值比较：

```bash
cd ~/Downloads
sha256sum ubuntu-22.04.x-desktop-amd64.iso
# 或
sha256sum ubuntu-20.04.x-desktop-amd64.iso
```

将 `x` 替换为下载文件名中的实际小版本号。校验值不一致时删除该 ISO 并重新下载，不要写入 U 盘。安装时建议给每个系统至少预留 60 GB 空间，并接入网络；若使用双系统，关闭快速启动/休眠并确认不会覆盖另一套系统的 EFI 分区。

安装完成、首次进入桌面并联网后，打开终端验证系统版本：

```bash
lsb_release -a
uname -m
```

ROS 2 主机必须看到 `Ubuntu 22.04`、`jammy`、`x86_64`；ROS 1 主机必须看到 `Ubuntu 20.04`、`focal`、`x86_64`。若代号不匹配，停止后续 ROS 安装，改装正确的系统。

每台主机都先更新基础软件：

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl git gnupg lsb-release ca-certificates build-essential
```

## 3. Ubuntu 22.04：安装 ROS 2 Humble 与 NEXUS 算法工作空间

以下全部在 Ubuntu 22.04（`jammy`）主机执行。不要在 Ubuntu 20.04 上执行本节。

### 3.1 配置语言环境与 ROS 2 软件源

```bash
sudo apt install -y locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe

sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

验证软件源已被读取，输出中应出现 `packages.ros.org/ros2/ubuntu jammy`，且最后没有错误：

```bash
apt-cache policy ros-humble-desktop
```

若没有候选版本，先检查 `lsb_release -cs` 是否为 `jammy`、网络/DNS 是否可用，以及 `/etc/apt/sources.list.d/ros2.list` 中是否误写成其他代号。

### 3.2 安装 ROS 2、构建工具和项目依赖

```bash
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pytest \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-apriltag \
  ros-humble-apriltag-msgs \
  ros-humble-ament-cmake-mypy
```

初始化 `rosdep`（一台机器仅首次需要；如果提示已经初始化，可跳过 `sudo rosdep init`）：

```bash
sudo rosdep init
rosdep update
```

当前终端加载 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

验证基础 ROS 2 安装：

```bash
printenv ROS_DISTRO
ros2 --help
ros2 pkg list | rg '^(apriltag_ros|apriltag_msgs|cv_bridge)$'
ros2 doctor --report
```

预期 `ROS_DISTRO` 为 `humble`，包列表包含上述包。`ros2 doctor` 的提示项需记录，但只要没有致命错误可继续。确认无误后，才把环境加载写入用户 shell：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 3.3 下载仓库及其外部模块

在希望存放项目的父目录执行。下面以 `~/nexus_workspace` 为例；若目录不同，替换成自己的实际位置。首次克隆用第一组命令；已经克隆过仓库则用第二组命令。

```bash
mkdir -p ~/nexus_workspace
cd ~/nexus_workspace
git clone --recurse-submodules https://github.com/wblxr408/NEXUS.git
cd NEXUS
```

```bash
cd ~/nexus_workspace/NEXUS
git submodule sync --recursive
git submodule update --init --recursive
```

验证仓库和关键模块均已取得。子模块状态行开头不应是 `-`（缺失）或 `+`（与锁定提交不一致）：

```bash
git remote -v
git submodule status --recursive
test -f code/ros2_ws/src/apriltag_ros/package.xml && echo 'apriltag_ros: OK'
test -f code/ros1_ws/src/fcu_core_external/fcu_core/package.xml && echo 'fcu_core: OK'
```

`code/third_party/rosbridge_suite` 是参考源码，故意不参与 ROS 2 工作空间的默认构建；不要为了解决构建问题把它移动回 `code/ros2_ws/src/`。

### 3.4 解析依赖、构建并运行测试

```bash
cd ~/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

每条 `colcon` 命令都应以退出码 `0` 结束；`colcon test-result --verbose` 的汇总应为 `errors=0`、`failures=0`。允许测试框架明确标记的 `skipped`，但不要把 `error` 或 `failure` 当作可忽略结果。

再做两项无硬件验证。它们只解析 launch 文件，不会要求相机或飞控在线：

```bash
ros2 launch nexus_vision_localization apriltag_localization.launch.py --print-description
ros2 launch nexus_bringup target_localization.launch.py --print-description
```

最后可运行固定的离线样例；此样例是 `test_sample`，用于验证主链路而不是宣称精度：

```bash
cd ~/nexus_workspace/NEXUS
python3 code/analysis/replay_cli.py \
  --input code/analysis/test_samples/pre_hardware_replay_cases.json \
  --output /tmp/nexus_pre_hardware_replay_report.json
python3 -m pytest code/analysis/test code/tools/test_channel_contract.py -q
```

预期回放输出 `8/8` 通过，测试输出为通过状态。报告位于 `/tmp`，不应当作为实验结果或提交到 Git。

## 4. Ubuntu 20.04：安装 ROS 1 Noetic 与 `fcu_core`

以下全部在 Ubuntu 20.04（`focal`）主机执行。此环境只承担厂商飞控兼容和 ROS 1 数据采集；不要安装 ROS 2 Humble，也不要将 ROS 2 工作空间混入此处。

### 4.1 配置 ROS 1 软件源

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | \
  sudo tee /etc/apt/sources.list.d/ros1-noetic.list > /dev/null
sudo apt update
apt-cache policy ros-noetic-desktop-full
```

最后一条必须显示候选版本。若没有，确认系统代号为 `focal`，并检查软件源文件是否含 `focal main`。不要在 Ubuntu 22.04 上把该行改成 `jammy`：Noetic 没有对应的受支持二进制包。

### 4.2 安装 ROS 1 与厂商编译依赖

```bash
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-rosdep \
  python3-catkin-tools \
  ros-noetic-serial \
  libeigen3-dev
```

初始化依赖工具（首次执行；若已经初始化，跳过第一行）：

```bash
sudo rosdep init
rosdep update
```

加载并验证 ROS 1：

```bash
source /opt/ros/noetic/setup.bash
printenv ROS_DISTRO
rosversion -d
roscore --help
```

前三项的关键结果分别应为 `noetic`、`noetic`，且 `roscore --help` 正常显示帮助。确认后写入 shell 启动文件：

```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 4.3 下载 NEXUS 和 `quadrotor_msgs`

与 ROS 2 主机一样克隆本仓库及其锁定子模块：

```bash
mkdir -p ~/nexus_workspace
cd ~/nexus_workspace
git clone --recurse-submodules https://github.com/wblxr408/NEXUS.git
cd NEXUS
```

如果仓库已经存在：

```bash
cd ~/nexus_workspace/NEXUS
git submodule sync --recursive
git submodule update --init --recursive
```

`fcu_core` 还依赖幻思公开的 `quadrotor_msgs`。该依赖必须作为 ROS 1 工作空间 `src/` 下、与 `fcu_core_external` 并列的源码包取得：

```bash
cd ~/nexus_workspace/NEXUS/code/ros1_ws/src
git clone https://github.com/fancinnov/quadrotor_msgs.git
```

若 GitHub 网络不可用，可按上游 README 改用 `https://gitee.com/fancinnov/quadrotor_msgs.git`；两者择一，不能同时克隆成两个同名包。下载后验证：

```bash
test -f ~/nexus_workspace/NEXUS/code/ros1_ws/src/fcu_core_external/fcu_core/package.xml && echo 'fcu_core: OK'
test -f ~/nexus_workspace/NEXUS/code/ros1_ws/src/quadrotor_msgs/package.xml && echo 'quadrotor_msgs: OK'
```

`quadrotor_msgs` 是为本机 ROS 1 构建补充的上游依赖。若要将其版本纳入项目可追溯范围，应先记录固定提交和许可证，再决定是否作为子模块登记；本教程不改动仓库或 Git 状态。

### 4.4 构建并验证 `fcu_core`

```bash
cd ~/nexus_workspace/NEXUS/code/ros1_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack find quadrotor_msgs
rospack find fcu_core
roslaunch --find fcu_core fcu_core.launch
```

`catkin_make` 必须成功，三个检查应分别返回对应包目录和 launch 文件目录。最后一条只查找 launch 文件，不启动节点、不会连接飞控，适合硬件未到时验证。

确认后可把工作空间覆盖层写入启动文件。此行必须在 `/opt/ros/noetic/setup.bash` 之后：

```bash
echo 'source ~/nexus_workspace/NEXUS/code/ros1_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 5. 到货首日：通信权限、连接和最小硬件验收

在接线、桨叶安全处理、上电顺序和 IP/串口参数均由厂商资料确认前，不启动飞控连接节点。详细验收项见[硬件首日验收表](2026-08-20_checklist_hardware_first_day_acceptance.md)。

### 5.1 USB/串口连接

接入设备后先识别新出现的设备，而不是猜测它一定是 `/dev/ttyACM0`：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
dmesg --follow
```

普通用户访问串口采用 `dialout` 组，而不是厂商上游 README 中不可追溯的 `chmod 777`：

```bash
sudo usermod -aG dialout "$USER"
```

这一步后必须注销并重新登录（或重启），再验证：

```bash
groups
ls -l /dev/ttyACM0
```

若实际设备为 `/dev/ttyUSB0`，把最后一行替换为该实际设备名。`groups` 输出应含 `dialout`。如果设备节点不存在、属主/组不正确，或反复变化，应记录 `dmesg` 和 `lsusb` 输出，先排查线缆、供电和厂商驱动；不要用 `sudo chmod 777` 绕过问题。

### 5.2 TCP 连接

如果厂商确认使用网络连接，先让 ROS1 主机与飞控处于同一网络，检查本机地址和连通性：

```bash
hostname -I
ping -c 4 192.168.4.1
```

`192.168.4.1:333` 是当前厂商资料提到的示例地址/端口，实际连接方式、IP、端口、波特率和 launch 参数必须以到货设备资料与现场核对结果为准。不要在未核对参数前修改 `fcu_core` 源码或硬编码网络设置。

若主机启用了防火墙，先只检查状态：

```bash
sudo ufw status verbose
```

只有在已经确认确实是防火墙阻塞、且知道所需端口后，才按最小范围添加规则；不要为了“能连上”而关闭全部防火墙。

### 5.3 实机 ROS 1 验收顺序

在桨叶安全、设备固定和厂商允许上电的条件下，使用三个终端：

```bash
# 终端 A：ROS 主节点
source /opt/ros/noetic/setup.bash
roscore
```

```bash
# 终端 B：厂商节点；先确认 fcu_core 的实际连接配置
cd ~/nexus_workspace/NEXUS/code/ros1_ws
source devel/setup.bash
roslaunch fcu_core fcu_core.launch
```

```bash
# 终端 C：只读检查
source ~/nexus_workspace/NEXUS/code/ros1_ws/devel/setup.bash
rostopic list
rostopic type /odom_global_001
rostopic echo -n 1 /odom_global_001
```

预期是节点启动且能从实际可用 topic 收到一条消息。若 `/odom_global_001` 不存在，不要自行将别的 odometry topic 当成它；记录 `rostopic list`、设备固件版本和 launch 输出，再对照厂商资料确认实际 topic、frame、时间来源和其代表的物理对象。

## 6. 两台主机之间：当前只做网络验收

硬件到货前，ROS1 与 ROS2 主机只需能位于同一受控局域网并相互 ping 通：

```bash
# 在每台主机上查看本机地址
hostname -I

# 使用另一台主机的实际 IP，不要复制示例 IP
ping -c 4 <另一台主机_IP>
```

ROS 1—ROS 2 bridge 的部署位置、消息映射和时间/坐标转换须在首次联调记录中确定。目前不要为方便而直接设置相同 `ROS_MASTER_URI`、混用 shell 初始化脚本，或把飞行平台 odometry 映射成外部目标观测。

## 7. 验收清单与常见问题

| 阶段 | 命令/证据 | 合格条件 |
| --- | --- | --- |
| Ubuntu 22.04 | `lsb_release -cs` | `jammy` |
| ROS 2 基础 | `printenv ROS_DISTRO`、`ros2 doctor --report` | `humble`，无致命错误 |
| ROS 2 项目 | `colcon build --symlink-install`、`colcon test`、`colcon test-result --verbose` | 构建成功，测试 `errors=0`、`failures=0` |
| ROS 2 离线主链路 | `replay_cli.py` 与 pytest | 固定 `test_sample` 通过；不是精度实测 |
| Ubuntu 20.04 | `lsb_release -cs` | `focal` |
| ROS 1 基础 | `rosversion -d` | `noetic` |
| ROS 1 工程 | `catkin_make`、`rospack find fcu_core` | 构建成功且包可发现 |
| 串口权限 | 重新登录后的 `groups` | 含 `dialout` |
| 到货后的硬件连接 | `rostopic echo -n 1 <确认后的topic>` | 实际收到并记录消息，不推断目标语义 |

常见现象及处理方向：

- `apt` 找不到 `ros-humble-*` 或 `ros-noetic-*`：先验证系统代号和软件源文件，不要跨发行版安装。
- `gpg: no valid OpenPGP data found`：通常是网络代理/证书页面替换了下载内容；先用浏览器或 `curl -I` 检查网络，再重新下载 key。
- `rosdep init` 报已初始化：这是正常状态，直接运行 `rosdep update`。
- 新终端找不到 `ros2`、`roscore` 或项目包：依次执行对应 `/opt/ros/.../setup.bash`、工作空间 `install/setup.bash`（ROS2）或 `devel/setup.bash`（ROS1），再检查 `.bashrc` 的顺序。
- `catkin_make` 提示找不到 `quadrotor_msgs`：确认其目录为 `code/ros1_ws/src/quadrotor_msgs/`，含 `package.xml`，然后重新运行 `catkin_make`。
- `Permission denied: /dev/ttyACM0`：确认已重新登录、`groups` 含 `dialout`，并确认设备名真实存在；不要使用 `chmod 777`。
- ROS 2 launch 启动后没有目标姿态：在硬件未到且 `marker_size_m=0.0` 时是预期保护行为。到货后测量标签实体边长、确认相机内参和帧语义，再按视觉准备清单配置。

## 8. 完成后的记录要求

安装成功不等于硬件或定位精度通过。首次完成两套环境时，在项目日记录中写明：主机用途、Ubuntu/ROS 版本、仓库提交和子模块状态、构建/测试命令及结果、未验证的硬件项。真实 rosbag、视频、编译产物和系统镜像不提交到 Git；按仓库的数据与实验记录规则登记外部位置和校验信息。
