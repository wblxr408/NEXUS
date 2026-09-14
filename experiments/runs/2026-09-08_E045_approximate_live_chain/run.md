# E045 近似标定下四环节接通

日期：2026-09-08。状态：**本地软件链路 VALIDATED；实机联合验收 PARTIALLY COMPLETE（用户确认无人机电池耗尽）**。

用户明确接受近似标定并要求打通 E044 发现的环节。本轮实现了统一入口、现有桥接 IMU 接入、基于静置测距均值的初始化、目标处理与输出连接。未把近似配置改成 measured，未写飞控参数、发送控制指令或另开串口。

## 修改

- dual_localization.launch.py：显式近似许可传到平台与视觉节点；固定标记参考可选，缺省仍保留原行为。
- superpoint_node.py、target_metric_node.py：支持有非空近似说明且显式 opt-in 的校准；单位、矩阵和有效性门限保持。
- approximate_initial_pose_node.py：将 /imu_global_001 原样转到 /nexus/fcu/imu；200 组静置均值与最近连续 IMU 静置样本就绪后，测距求 tag 位置、补偿配置杠杆臂、由重力及显式初始 yaw 先验生成完整位姿，收到平台初始化回执后停止重复初始化。
- live_approximate_localization.launch.py、两份近似 profile 和 run_live_approximate_localization.sh：单一地面入口启动相机、统计、初始化、现有平台/目标/展示链路。
- 网页输入状态显示 200 组进度、四路均值和近似初始化状态；录包清单包含新增状态与统计话题。
- nexus_camera_stream.service 为已有 Pi 相机服务提供自启动模板，**由于断电尚未安装到 Pi**。源文件已在之前相机测试中部署，运行条件见操作说明。
- 增加初始化几何与 DDS 集成测试、统一入口近似配置测试、网页统计状态测试，以及可复现的 GPU 参考目标坐标联调脚本。

启动：`bash code/tools/run_live_approximate_localization.sh`；浏览器 `http://localhost:8766`。详见 `code/deployment/raspberry_pi_readonly_stack/live_approximate_chain.md`。本地验证用隔离 ROS_DOMAIN_ID=89/90，生产脚本缺省域为 20。

## 验证证据

1. 近似标定统一入口拒绝路径与允许路径均测试；固定参考关闭后不需要 AprilTag 配置。原严格默认保持。
2. 实际 DDS 测试驱动原始 IMU/测距 → 200 组滚动统计 → 完整初始位姿 → platform_node 输出 map Odometry；验证杠杆臂补偿与初始化回执。输入为加速合成数据，不是实机 200 s 记录。
3. **真实 GPU 推理 + 实际 ROS 节点**：使用合成平面纹理及已知运动相机位姿，通过与网页一致的参考图/目标请求接口注册 selected_patch；实际 YOLO 和 SuperPoint 模型推理、身份跟踪、目标几何、融合、展示均运行。共 24 组目标轨迹、20 条有效几何和融合结果、24 次当前目标展示记录。末次输出约 [-0.01208,-0.05372,2.98437] m，合成平面深度为 3 m；这是合成输入联调数字，不是实机精度。见 gpu_chain_v01/result.json、gpu_chain_v01.log。
4. 完整启动验证：修复权限后，实际执行新入口，发现 11 个节点，包括相机、统计、初始化、平台、检测、SuperPoint、目标几何、融合、策略、仪表板、rosbridge。启动阶段无 ERROR，见 unified_startup_v03.log、unified_nodes_v03.txt。Pi 无供电，运行中等待真实输入；此项没有产生实体目标坐标。
5. 相关四包 pytest 用例数：bringup 20、vision 86、fusion 81、dashboard 8，共 195，0 errors/failures/skipped；网页 12 项通过。test_counts.json 另外列出未改变算法的 ingress 17 项既有结果，不作为本轮新增测试数。colcon 总汇报还包含测试包装层/其他包旧结果，不能直接当作本轮用例数量。
6. 定向 git diff --check 通过，保留此前已有修改；源码散列见 source_manifest.json。

构建/测试日志为 build_tests_v01.txt、build_tests_v02.txt、final_validation.txt。复现 GPU 联调：

```bash
ROS_DOMAIN_ID=89 OPENBLAS_NUM_THREADS=1 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  bash code/tools/run_with_camera_models.sh python3 \
  code/ros2_ws/src/nexus_bringup/test/run_reference_coordinate_chain.py \
  --output /tmp/nexus_new_gpu_chain_run \
  --detector data/processed/2026-09-05_gpu_inference_v01/detector/detector_manifest.json \
  --superpoint data/processed/2026-09-05_gpu_inference_v01/superpoint/superpoint_manifest.json
```

输出目录须不存在。该 GPU 测试用已知平台位姿验证目标链，平台初始化另有 DDS 测试；没有把这两项描述为真实传感器同步全程验收。

## 发现并解决的启动问题

第一次完整启动失败：range_smoothing_node.py 缺执行权限，launch 提示 libexec 中找不到可执行文件；已补执行权限并重新构建。失败日志保留 unified_startup.log。第二次节点列表使用 daemon，只取得不完整发现结果；第三次改用 no-daemon/spin-time 5，取得全部节点。早期以进程组重复 SIGINT 停测试时产生 KeyboardInterrupt/重复 shutdown 错误，第三次改为仅通知 launch 统一停止；不把人工停止后的日志当作启动阶段故障。

## 明确边界

200 组均值接入初始化和界面统计，运动估计仍用原时间戳测量及现有稳健后端；历史均值不是瞬时位置。初始 yaw 假定零参考朝向加 9° 偏置，标签高度求解限制在顶部基站下方，相机旋转/平移为显式近似，模型仍可能与实物安装不符。基线不足、目标丢失或平台无效时不能制造有效坐标。

当前因断电无法验证 Pi 相机自启动、真实图像中目标识别、实际平台连续性和真实最终坐标。本轮没有解决此前 E042 所有测距/连续性误差，用户授权暂不以这些误差阻塞软件接线。上电后需保持静置积累窗口，再执行一个目标的注册、改变视角和结果验收。

未扩大任务范围。
