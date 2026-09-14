# E027：当前输入输出网页适配与目标注册

状态：网页/注册行为 VALIDATED；整个视觉工作区存在对照可复现的既有失败，不声明整套定位完成。

## 来源

Windows Edge、WSL Ubuntu 22.04 / ROS2 Humble、本机 RTX 4070；隔离 ROS_DOMAIN_ID=88。实际 SuperPoint 使用 E026 TorchScript，device=cuda:0，无 CPU 神经网络推理。输入为带 SYNTHETIC WEB QA 标识的 1640×1232 合成纹理及空白图，标定标为 synthetic_test，未连接实机。

资产：data/processed/2026-09-07_web_registration_v01/，包含 browser_result.json、原图 SHA256、events.jsonl、截图、运行配置、56 个代码文件快照及校验、HEAD 和 scope.diff。快照中的其他视觉/启动优化是并行工作区内容，本任务保留。

## 验证

- Node 11 项网页测试通过，覆盖历史状态、缩放框选、原图/时间、回执、拒绝、断线、取消、分片和输入状态。
- nexus_vision_localization、nexus_viz_dashboard、nexus_bringup 三包 colcon 构建通过。
- 网页包、启动包 colcon test-result 分别 11 / 22 项检查，0 错误、0 失败；套件计数不等同于独立单元用例。
- 真实浏览器经 rosbridge/ROS2 调用 CUDA SuperPoint 注册通过。原图校验与精确时间一致，选框 [328,246,984,740]，参考点 [820,616]。有效图返回 registered，空白图返回 insufficient stable features。
- 过期预览清除、取消、390px 窄屏无水平溢出均通过。白名单外 /nexus/web_qa/blocked 被服务器拒绝，测试订阅者收到 0 条。没有飞行命令。
- 已检查桌面和框选截图；相关 diff --check 通过。

## 失败记录

1. 初轮相机预览等待超时；下一轮还遭遇中断留下的测试端口占用，均未计为通过。停止本任务自建服务，按已有 LARGE_DATA 配置重跑，增加测试端口预检。
2. 原图被 rosbridge 分为 9 片，网页未重组导致定格超时。补齐重组后通过。
3. 双引号 glob 列表不被当前 rosbridge 解析器剥除引号，导致注册话题被拒绝；改用其接受的单引号列表，并验证实际解析器。
4. 原参考图像尽力传输订阅出现 reference_pair_timeout，可靠观察者已经收到同一原图。仅把注册图像订阅改为可靠 QoS 后通过。
5. 完整视觉回归的 test_target_metric_node、test_target_pipeline_nodes、test_visual_pipeline_nodes 共 6 个用例失败；colcon 连同套件计为 9 failures。使用本次 QoS 修改前的节点源码，在内存中替换后重跑同三组测试，仍为 6 failed / 3 passed。对照日志/XML 已归档；未回退共享工作区，未改其他优化代码。

## 重跑

~~~bash
node --experimental-default-type=module --test code/ros2_ws/src/nexus_viz_dashboard/web/test/*.test.mjs

ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA OPENBLAS_NUM_THREADS=1 \
bash code/tools/run_with_camera_models.sh python3 \
code/ros2_ws/src/nexus_viz_dashboard/test/run_registration_web_qa.py /tmp/新的运行目录 \
--web-port 18770 --bridge-port 19095 \
--manifest /root/nexus_workspace/NEXUS/data/processed/2026-09-05_gpu_inference_v01/superpoint/superpoint_manifest.json
~~~

另一个 Windows 终端用已有 Node 运行 web/test/live_registration.cjs，参数是运行目录的 UNC 路径和 http://127.0.0.1:18770/public/index.html?rosbridge=ws://127.0.0.1:19095。NEXUS_PLAYWRIGHT_MODULE 指向已有 Playwright 包目录。完成后向 phase.txt 写 stop，仅退出本次 QA。

最终运行目录 /tmp/nexus_web_registration_qa_07 已归档；构建日志 /tmp/nexus_web_final_build，测试日志 /tmp/nexus_web_final_test。本次使用的模型路径已记入 runtime.json。

## 未验证与范围

真实相机标定、相机—IMU 联合标定、真实泛化、飞行抖动、整链路延迟和定位精度未由本实验验证。相机采集继续暂停；飞控未操作。改动限于网页、注册白名单、参考图像 QoS 和相应验证，未扩大到其他算法或飞控修改。
