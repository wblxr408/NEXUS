# E047 实机恢复、运动初始化与目标链路复核

状态：PARTIALLY COMPLETE。IMU、相机、运动初始化与平台恢复已取得实机证据；尚无有效目标三维坐标和融合显示的完整实机通过证据。不能保证上机必然成功。

## 环境与范围

Windows + WSL Ubuntu 22.04 / ROS2 Humble，地面 domain 20 localhost-only，树莓派既有 fcu_ros2 容器；模型使用既有项目 GPU runtime。代码版本和文件 SHA256 见 provenance_v01.json，工作区原有大量未提交变更，未重置、未提交。未扩大任务范围；所有修改对应用户授权的只读 IMU、实时定位与目标输出链路，没有发送解锁、起飞或控制指令。

## 本轮修改

- 树莓派托管原有唯一串口桥接，启用桥接、只读传感器 TCP 转发、测距统计和相机服务。实际服务启动已验证；完整断电冷启动未验证。
- 原始 IMU、UWB 及飞控姿态通过 CDR/TCP 转发，地面四时间戳交换对齐时钟，保留原始话题。修复 Pi 与地面约 6–7 秒时钟差，并平滑偏移调整，避免调整时钟制造倒退。
- IMU 接收队列增大，平台可输出有期限的 IMU 预测；间断与外部约束丢失会重新请求初始化，不能无限发布失去约束的姿态位置。
- 最初实现静置 200 组统计初始化和当次静置零偏估计。用户明确手持模拟飞行后，当前 live 配置改为飞控实时姿态 + 当前 UWB 初始化，不要求持续静止；200 组统计独立输出，不作为移动瞬时测距。零偏先验来自 current_static_bias.json；初速度未知，采用 1 m/s 先验标准差。保留静置模式及其回归测试。
- 飞控桥接代码确认姿态采用 roll、-pitch、-yaw；只取姿态并叠加现有 9° 地图偏置，不直接采信其位置字段。绝对航向、相机安装和杠杆臂仍为近似。
- 相机 ROS Image 数据赋值改用 array('B')，保持像素内容与标定尺寸。camera_array_benchmark.json 验证字节一致；4 MB 数据 bytes setter 约 0.24 秒，数组 setter 微秒级，未包含数组构造时间。
- 已选目标保留基础特征预算，避免背景自适应预算缩小导致边缘小目标参考特征消失。reference_scale_probe.txt 中同一实拍图在半分辨率 256 特征时匹配失败，512/1024 时有 16 个几何内点；这属于复现与修复依据，不是实时跟踪验收。

## 实测

- pi_live_imu_v02.txt：20.04 秒 3782 个真实 IMU，约 189 Hz，同时 20 组 UWB、200 组飞控里程计，时间戳递增。
- ground_camera_bulk_v01.json：15 秒 2852 IMU、151 图像；约 189 Hz / 10 Hz；IMU 最大间隔约 20 ms，图像最大间隔约 107 ms。
- ground_live_bias_v01.json：早期静置 45 秒平台输出约 22.5 Hz，45 次接受 UWB 更新；近似位姿不能作为精度结果。
- moving_target_v01.json：改为运动初始化后实际收到 initialized 与 bounded_imu_prediction，不再等待静置。
- ground_moving_v01.json 暴露外部约束丢失后未恢复，随后修复。traffic_registration_v07.json 在 25 秒中有 202 条预测状态、23 条接受状态，且记录了一次外部约束失效后的重新初始化。输出并非无间断保证。
- 红绿灯参考图多次注册成功，但截至 traffic_registration_v07.json，observed 目标数量为零；目标几何输出主要是 target_not_observed 等无效状态，没有有效融合坐标。运动中实图多次显示目标离开视野、边缘遮挡或模糊；不能由此断言算法的其余环节已经全部正确。
- 用户最后居中的物体为白色圆顶绿色立柱，灯面不可见；另以 centered_object_001 做实例验证，不把它冒充红绿灯类别识别。traffic_registration_v09.json 的 30 秒窗口中该实例 observed=true 共 7 次；有效目标几何和融合坐标仍均为零，原因包括 insufficient_target_views、target_reference_anchor_not_visible、身份歧义及目标丢失。不能把无效 TargetKinematicState 的 NaN 坐标视为输出成功。

## 验证命令与结果

在 ROS 环境、code/ros2_ws 下执行：

```bash
colcon build --packages-select nexus_pi_readonly_ingress nexus_fusion_localization nexus_bringup nexus_vision_localization
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_pi_readonly_ingress nexus_fusion_localization nexus_bringup nexus_vision_localization --event-handlers console_direct+
git diff --check -- src/nexus_pi_readonly_ingress src/nexus_fusion_localization src/nexus_bringup src/nexus_vision_localization
```

构建分别记录于 final_build_v01.txt、moving_recovery_tests_v03.txt、target_budget_tests_v01.txt。最终四包 colcon 测试见 final_tests_v02.txt；210 项 pytest 测试零失败（按四包 xunit 统计，见 final_test_counts.json），各包 CTest 检查通过。diff 空白检查通过。

失败未删：E046 原始重启回归和 E047 早期 wall-clock 测试因合成输入间断失败；改用 ROS 模拟时钟后分别验证静置重启、IMU 间断。moving_initialization_tests_v01.txt 首次运动用例因测试共用消息头误改 frame_id 失败，修复测试后 v02 六项通过；moving_recovery_tests_v03.txt 十二项通过，包含真实 DDS 运动初始化与约束失效恢复。部署一次普通权限写 Pi 文件失败，改用现有 sudo 权限部署成功。旧 IMU 与 UWB 历史精度失败未改写。

## 复跑与限制

入口 code/tools/run_live_approximate_localization.sh，网页 http://127.0.0.1:8766/?rosbridge=ws%3A%2F%2F127.0.0.1%3A9091 。避免与当前已运行同名节点重复启动。

当前服务为分阶段更新后的运行实例；完整一键冷启动尚未实测。目标有效验收需同时检查 observed 跟踪、valid/position_observed 几何与融合消息、以及网页当前目标，历史状态或仅注册成功不算通过。

UWB 四槽多段持续为相同数值；按用户要求未校正系统误差，不能声称真实移动位置准确。近似外参、未知精确杠杆臂、初速度与航向先验会影响目标几何可解性。原始图像和事件保留在忽略目录 data/raw/2026-09-08_E047_recovery_and_pose_stream，校验见 provenance_v01.json；最初 traffic_light_current 文件曾被后续抓拍替换，因此它不能重现早期 v03/v04 注册参考，后续参考均独立编号。
