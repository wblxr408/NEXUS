# FCU 控制链路修正与隔离验证

日期：2026-09-05（北京时间）。

授权：用户要求采用已有串口记录、统一指令话题、本地持久设置域 20、补全已审查的控制报文缺陷。

状态：软件修改与隔离验证 VALIDATED；真实串口通信、跨机 DDS 和实机起飞 UNVERIFIED。没有向真实无人机发送控制指令，没有启动实机控制容器。

## 修改范围

本地 Ubuntu /root/nexus_workspace/NEXUS：

- fcu_core_gcs/start_gcs.sh：显式加载 Humble，启动前固定 ROS_DOMAIN_ID=20。
- fcu_core_gcs/src/fcu_core_ros2/src/fcu_command.cpp：话题固定为 /command；控制提示改为“已发送”，不代表飞控已执行。
- fcu_core_gcs/src/rviz2_custom_panel/src/mypanel.cpp：发布话题从 /fcu_bridge/command 统一为 /command。
- fcu_core_gcs/test/test_gcs_cli.py：使用真实编译后的 CLI，在只有 loopback 的隔离网络命名空间验证发布。
- /root/.bashrc：追加域 20；新交互终端以 printenv ROS_DOMAIN_ID 实测输出 20。已有终端需重新加载，启动脚本自身也明确设置域号。
- 本记录文件，以及上述两包编译所需的 build/install 生成物。

树莓派 192.168.1.143 的 /home/pi/fcu_core_onboard（在原 ROS 容器中映射为 /root/fcu_core_onboard）：

- start_onboard.sh：明确加载 ROS 环境，启动前固定域 20。
- src/fcu_core_ros2/launch/fcu_core_launch.py：/dev/ttyAMA0 波特率设为 115200；指令映射统一为 /command。
- src/fcu_core_ros2/src/fcu_bridge.h：初始化心跳、模式和控制报文字段；目标地址来自飞控心跳；未收到目标心跳时拒发控制指令；串口打开成功返回 0，失败输出端口和原因。
- src/fcu_core_ros2/src/fcu_bridge_001.cpp：记录目标地址、实际解锁状态、COMMAND_ACK 与有界 STATUSTEXT；订阅/发布 /command；串口连接失败退出。
- src/fcu_core_ros2/test/test_control_packets.cpp 与 CMakeLists.txt：注册直接调用生产解析器/指令处理器的内存报文测试，不打开设备。
- fcu_core 构建/安装生成物。

串口依据：docs/2026-09-02_design_online_target_localization_framework_v01.md 第 326 行的已有 115200 链路记录，以及树莓派 read_mavlink.py、read_uwb.py、fanciswarm_bridge.py 的配置。未写入或读取飞控当前 UART 参数。

协议决定：保留厂商既有 SET_MODE 解锁/上锁及 COMMAND_LONG 起飞/降落形式，不迁移为另一套飞控协议。以前未赋值的可选参数现初始化为 0，不新增高度指令。ROS 域 20 与 MAVLink system/component ID 是不同概念，后者从心跳获取，不硬编码为 20。真实固件对报文及可选参数的行为仍需实机验证。

## 备份

- 本地修改前文件和 /root/.bashrc：/tmp/nexus_fcu_fix_20260905_1745/（临时目录）。
- 机载修改前源码、启动文件、CMake 和旧桥接二进制：/home/pi/fcu_core_onboard_backups/2026-09-05_1745_control_fix/。
- 保留原有未提交修改；未执行 build.sh 中的全量删除命令，未重置 Git。

## 验证命令与结果

### 本地构建

环境：WSL Ubuntu 22.04，x86_64，ROS2 Humble，已有依赖。

    source /opt/ros/humble/setup.bash
    cd /root/nexus_workspace/NEXUS/fcu_core_gcs
    colcon --log-base /tmp/nexus_fcu_fix_20260905_1745/gcs_build_log build --packages-select fcu_core rviz2_custom_panel

结果：2 packages finished，退出码 0。有既存空日志格式字符串、未使用参数警告。RViz 插件二进制已核对包含 /command；未进行 RViz 图形界面点击测试。

### 本地实际 CLI 发布检查

    unshare --net bash -lc 'ip link set lo up && source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 python3 /root/nexus_workspace/NEXUS/fcu_core_gcs/test/test_gcs_cli.py'

结果：PASS。实际安装的 CLI 在 /command 发布 [1, 3, 4, 2]；帮助、退出不发控制消息；旧话题无发布者；进程正常退出。测试仅有 loopback 网卡，域 97，与真实局域网隔离。

### 机载构建

环境：树莓派 ARM64，已有 nexus_ros_build_env:20260902 镜像。只挂载源码工作空间，不挂载宿主 /dev，网络为 none。

在树莓派运行：

    docker run --name nexus_fcu_control_fix_20260905 --network none --entrypoint bash -v /home/pi/fcu_core_onboard:/root/fcu_core_onboard -w /root/fcu_core_onboard nexus_ros_build_env:20260902 -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && colcon --log-base /root/fcu_core_onboard/log/control_fix_20260905 build --packages-select fcu_core --symlink-install'

结果：1 package finished，退出码 0。保留 MAVLink 生成头的 packed/匿名结构体等既存警告，以及未使用参数、非当前串口路径的 wifi_connect 缺失返回值警告。

### 机载控制报文测试

    docker run --name nexus_fcu_control_test_20260905 --network none --entrypoint bash -v /home/pi/fcu_core_onboard:/root/fcu_core_onboard -w /root/fcu_core_onboard nexus_ros_build_env:20260902 -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && colcon --log-base /root/fcu_core_onboard/log/control_test_20260905 test --packages-select fcu_core --ctest-args -R control_packets --output-on-failure && colcon test-result --test-result-base build/fcu_core --verbose'

结果：1 test，0 errors，0 failures，0 skipped。另在同样隔离条件下以 ctest --test-dir build/fcu_core -R ^control_packets$ -V 核对实际执行输出，通过。

覆盖：两组不同目标地址；未收到心跳拒发；忽略地面站/自身心跳；解锁、上锁、起飞、降落、追踪报文的类型/地址/参数/确认字段；心跳反馈解锁状态；成功/拒绝 ACK；没有 NUL 的 50 字节 STATUSTEXT；未知指令不发包。所有数据均为测试生成，不是实测飞行证据。

完整输出：机载 log/control_test_20260905/latest_test/fcu_core/stdout.log。

以上命名测试容器执行后保留为 stopped；重跑须使用新名字或重新启动对应测试容器。不要把实机 fcu_ros2 容器用于隔离测试。

### 串口失败路径与静态检查

在隔离测试容器 nexus_fcu_serial_failure_test_20260905 中运行实际安装的桥接二进制：

    install/fcu_core/lib/fcu_core/fcu_bridge_001 --ros-args -p channel:=0 -p USB_PORT:=/tmp/no_fcu_device_for_test -p BANDRATE:=115200

测试环境：network none，ROS_DOMAIN_ID=97，ROS_LOCALHOST_ONLY=1，无宿主 /dev 映射。结果：输出 Unable to open 与端口名，以退出码 1 返回；测试断言通过，没有再次访问串口而崩溃。容器日志保留完整错误。

两端启动脚本 bash -n、launch/Python 测试文件 AST 解析、改动行空白检查通过。检查了本地 Git diff 和机载备份对比。未运行整个既有工程的全量 lint/test，也不声明其全部通过。

## 未验证与未改动

- 未启动 fcu_ros2 实机控制容器；本轮结束时仍为 exited，退出码 255（任务开始前已有状态）。
- 未验证真实飞控心跳、当前实机波特率、跨机 DDS 发现、飞控是否接受/执行起飞，不能宣称“已经能飞”。
- ACK/STATUSTEXT 显示在机载节点日志；没有新增地面站确认界面或自动重发飞行指令。
- 未修改飞控参数、飞行高度、路径算法、遥控器设置、串口接线。
- 已发现的 fcu_mission Ctrl+C 退出段错误未修改，与本次控制报文修正分开。
- 目标位置的可选 domain_bridge 10→1 配置不属于此次起飞链路，未修改。
- 未升级或新增软件依赖。未扩大任务范围。

## 安装二进制校验

- 本地 fcu_command SHA256：7b04f7e30cc39a06651dcb9df03ee0c241ff0c1b7f917db90cbe01be3be95f17
- 机载 fcu_bridge_001 SHA256：0f2ea0f29e2d55c45de2c34ed8673581e59b73e57c61a2d23a140a9f48c43bd7
