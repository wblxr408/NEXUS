# 双对象定位逐项实现检查表

日期：2026-09-03。目标：完整对应本任务用户设计的总体架构及第 I–XI 部分，补全代码并验证；本表不是已完成声明。

状态说明：已验证=指定检查已通过；部分=有代码但要求未覆盖完整；待实现=未找到对应实现；待核验=已有入口尚未完成当前状态审计；受外部条件限制=需要硬件/数据/标定，仍须完成可执行代码。所有合成测试仅证明软件行为。

| 编号 | 要求 | 当前证据/入口 | 状态与剩余工作 |
|---|---|---|---|
| A01 | 树莓派飞控兼容、UWB/IMU读取 | `code/deployment/raspberry_pi_uav_readonly_adapter/`、ROS1 bridge | 待核验；保留厂商控制闭环 |
| A02 | IMX219采集、采样时间、转发/缓存 | `code/deployment/raspberry_pi_e016_self_localization/` | 待核验；真实传输时钟与丢包行为待实机 |
| A03 | ROS1/ROS2 显式桥接、SI/frame约定 | `code/tools/nexus_channel_contract.py`、`telemetry_udp_node.py` | 待核验 |
| B01 | 平台状态p/v/q/ba/bg、IMU预积分 | fusion包 `platform_state.py`、`imu_preintegration.py`、`platform_node.py` | 数值及独立ROS接入已验证；重力、偏置修正、原采样区间和延迟插帧已覆盖；真实IMU标定待提供 |
| B02 | UWB绝对平台位置/测距因子 | `platform_measurements.py`、`platform_node.py` | 3D位置/原始测距/杆臂/创新门控已验证；实机survey、真实传输接入待核验 |
| B03 | SuperPoint静态特征、相对运动 | `superpoint_frontend.py`、`visual_motion.py`、`static_tracker.py`、`superpoint_node.py` | 时序参考维护、弱视差积累、精确帧配对、限时回滚、DDS图像至平台因子已验证；实际ONNX权重推理和实机效果未验证 |
| B04 | 动态区域屏蔽 | `static_background_mask`、`detection_node.py`、`superpoint_node.py`、`target_frontend.py` | 同帧检测/分割、边界扩张、目标参考候选及短时丢失区域屏蔽已通过合成/DDS测试；真实检测漏检与掩膜效果待数据验证 |
| B05 | 固定AprilTag场地参考、初始化/漂移校正 | vision包 `reference_geometry.py`、`fixed_reference_node.py` | 多Tag联合PnP、几何拒绝、map机体转换及协方差、独立DDS输出已验证；图像检测至平台估计全链联调待完成 |
| B06 | 平台鲁棒滑窗与边缘化先验 | `platform_window.py`、`marginalization.py`；目标 `online_solver.py`、`bundle_marginalization.py` | 平台真实Schur及目标平方根边缘化、无重复计数、秩亏保留、失败回滚已通过对应软件检查；平台/目标完整相关ROS联合后端仍未完成 |
| C01 | 已知类别检测/分割入口 | vision包 `object_detector.py`、`detection_node.py` | YOLOv5/v8显式ONNX接口、letterbox、类别NMS、同帧ROS检测/失败包已验证；分割输入可接外部mono8掩膜，真实模型推理未验证 |
| C02 | 参考图/首帧框选、SuperPoint目标条件化匹配 | `target_frontend.py`、`target_identity.py`、`superpoint_node.py` | ROS参考图+ROI注册、无类别框发现、几何关联、命名绑定与超时回滚已验证；尚无交互框选UI，真实模型/图像效果未验证 |
| C03 | 标签展示目标/多Tag联合PnP | ROS2 vision/PnP | 待核验多Tag与单目标参考定义 |
| D01 | 已知尺寸关键点PnP-RANSAC | `reference_geometry.solve_keypoint_pose` | 已知3D/2D对应、RANSAC/多解/重投影/正深度/协方差数值层已验证；在线模型关键点生产与接入仍未完成 |
| D02 | 多帧对应/三角测量，无纹理轮廓与先验 | `target_frontend.py`、`target_multiview.py`、`target_metric_node.py`，旧F1/F2/F3基线 | 固定参考特征对应→静态/匀速多帧求解→ROS米制状态已通过数值及DDS测试；无纹理轮廓/支撑面/尺寸先验在线分支仍待补 |
| D03 | 目标p/v/q与可观测性边界 | `MultiviewEstimate`、`TargetKinematicState.msg`、`kinematic_filter.py` | 静态/匀速几何及新消息p/v/SO(3)融合、部分观测NaN、交叉协方差、有限预测已验证；已知姿态消息消费有DDS测试，但自动动态完整姿态生产及一般运动模型未完成 |
| D04 | map/body/camera/target坐标链与不确定性 | `target_geometry.py`、`pose_timeline.py`、`target_multiview.py`、`kinematic_filter.py` | 原采样时刻几何与平台/共享外参传播已验证；相邻目标解的未知相关性通过CI处理，角协方差转换有独立数值导数对照；PnP在线生产及完整联合相关后端仍待补 |
| E01 | 独立track_id、运动/特征/外观/几何关联 | `target_identity.py`、`target_frontend.py` | 像素层运动/特征/外观/几何联合一对一分配与参考重识别已验证；米制位置/尺寸/深度关联待三维前端 |
| E02 | tentative/confirmed/degraded/lost/re-associated | `target_identity.py`、`kinematic_filter.py`、ROS目标状态 | 五态、歧义/参考点变更停止旧点预测、连续新证据恢复已验证；新融合不能提升前端tentative身份；真实交叉视频与身份评估仍待测 |
| F01 | 输入检查、过期/倒序、单位/frame | ROS契约、图像/平台/目标节点、`target_kinematic_node.py` | 增加新消息有效子块/未知NaN、未来采样及接收时间、精确质量配对、同帧invalid取消待处理包、限时回滚；软件/DDS通过，实机时钟/网络仍待验证 |
| F02 | 创新马氏门控、可疑降权、异常拒绝 | `robust_filter.py`、`kinematic_filter.py`、`platform_window.py` | 新米制状态按共同可观测维数门控，未知相关创新使用保守协方差界，硬拒绝不能被当次学习膨胀放行；旧目标/平台门控回归通过 |
| F03 | RANSAC/重投影/正深度/双向/分布/跨源检查 | 固定/目标PnP、静态/目标特征、`target_multiview.py` | 增加多视线RANSAC/Huber、当前帧粗差、深度/视差/速度/条件数检查及目标几何质量ROS生产；ROS前端IMU旋转预测接入仍待补 |
| F04 | 鲁棒核、动态协方差、因子级拒绝 | `bundle_solver.py`、`online_solver.py`、`bundle_marginalization.py`、ROS滤波/窗口/CI | 历史不重复计数/鲁棒加权已验证；新米制目标固定/规则/学习协方差及CI接入完成；完整共享因子图与真实退化收益仍待补/测 |
| F05 | 多源降级、短时预测、超时失效与恢复 | `kinematic_filter.py`、`target_kinematic_node.py`及已有前端/平台 | 新消息到融合的预测、超时、恢复、旧失败/未来包拒绝已通过数值/DDS；预测明确invalid且保留最后观测时间；RViz/Web消费仍待接入 |
| G01 | 多源质量特征、MLP 64/32/4、权重推理 | `reliability.py`、几何质量生产、`target_kinematic_node.py` | 新米制目标已消费真实MLP接口并按目标/source/参考点/时间精确配对，支持乱序及缺失掩码；软件测试通过，真实权重与学习收益未验证 |
| G02 | 正定有界协方差映射、硬门控优先 | `covariance_scales`、新旧目标filter、平台窗口 | 1–100倍尺度与硬门控优先已验证，新目标p/v/角交叉块整体保留；平台学习DDS闭环仍需专门核验 |
| G03 | 按采集批次划分训练/验证/测试、模型保存/加载 | `train_reliability.py` | 训练CLI、分组划分、训练集标准化、验证选轮次、NPZ/SHA256已验证；真实数据/标签未提供 |
| H01 | ROS2双输出、运行配置、启动路径 | platform/reference/static_visual、target_kinematic_fusion launch | 参考图→像素特征→平台时刻配对→目标米制→CI融合已通过DDS；static_visual默认包含融合，独立launch与安装入口已检查；完整展示与平台统一启动仍待完成 |
| H02 | RViz2/Web/rosbag记录、质量状态 | `nexus_viz_dashboard/`、bringup | 待核验新增状态与双输出 |
| I01 | 平台7组、目标7组消融配置与运行入口 | A/B/C/D静态目标链 | 部分；不能代替用户全部实验组合 |
| I02 | 3D/平面RMSE、P50/P95/最大、离群率 | `evaluation/` | 待核验双对象一致口径 |
| I03 | ID切换、丢失时长、恢复时间、延迟/FPS | `observation_quality.py`仅部分指标 | 部分；跟踪指标待补 |
| I04 | 真实复杂环境数据、标定与精度证据 | 本次未发现可支持最新完整链的记录 | 受外部条件限制；不能以合成结果代替 |

## 当前检查记录

- Ubuntu 22.04 WSL，系统 Python 3.10，已安装 NumPy/SciPy/OpenCV/Pytest，ROS2 Humble 可用；系统 Python 与仓库 `.venv` 中未检测到 PyTorch。
- 首次混合测试未包含 ROS 包 PYTHONPATH，收集报 `ModuleNotFoundError: nexus_fusion_localization`；补正确路径重跑通过。
- 基线：`PYTHONPATH=code/analysis:code/ros2_ws/src/nexus_fusion_localization/src python3 -m pytest code/analysis/test/test_localization_bundle.py code/analysis/test/test_online_frontend.py code/ros2_ws/src/nexus_fusion_localization/test -q` → **27 passed**。
- 仓库已有大量未提交修改和未跟踪文件；本次不得覆盖或回退它们。历史设计保留，方案差异见 ADR-011。

## 第一阶段增量验证

- 离线分析完整测试：`OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test -q` → **69 passed**。
- 相关数值与模型测试（不含ROS消息运行测试）→ **54 passed**，包括学习训练与保存加载、创新拒绝、恢复、几何退化和窗口失败回滚。
- ROS2：`colcon build --packages-up-to nexus_fusion_localization --symlink-install` → nexus_msgs 与 nexus_fusion_localization 构建成功。
- `colcon test --packages-select nexus_fusion_localization`、`colcon test-result --test-result-base build/nexus_fusion_localization --verbose` → **0 errors / 0 failures / 0 skipped**。包含27个pytest用例（含DDS收发、模型加载与精确帧质量输入测试），colcon同时计入4个CTest套件，汇总为31 tests。
- DDS测试首次因测试CMake使用APPEND_ENV拼接已有ROS_DOMAIN_ID而启动失败，已改ENV覆盖，重跑通过。该错误未接触实际定位运行或硬件。
- 检查实际新增/修改源码的Flake8，按现有长行风格忽略E501/W503；`git diff --check`通过。
- Flake8曾报告新增滤波器及消息测试的续行缩进E128，已修正后重跑。

## 仍必须继续完成

上述验证不等同于完整方案已实现。B01/B02/B05/B06的平台数值与独立ROS阶段已补全。继续 B03/B04、C02、E01/E02：SuperPoint静态前端与目标条件化身份关联；替换目标窗口弱F8，再接通H01/H02并补I01–I04。

目标在线窗口当前仍沿用弱F8先验近似，不是真正的Schur边缘化；重复窗口信息可能被反复计入，必须在B06/平台窗口阶段替换为明确的边缘化信息处理。该项未标为完成。

运行中固定Tag图、相机外参、IMU轴向/噪声与锚点survey仍需真实标定；不得填入猜测值冒充硬件参数。没有真实训练数据和完整实机运行记录，未宣称学习收益、实时性能或精度达标。

## 第二阶段增量验证：平台滑窗与固定参考

- 平台包含15维p/v/q/ba/bg、带偏置修正的IMU预积分、UWB位置与原始测距、单目方向/米制相对位姿、固定Tag绝对位姿、鲁棒核及真实Schur先验。
- 延迟观测在保留窗口中插入原采样时刻并拆分IMU边，不挪到其他帧；IMU断流不外推；大偏置修正重新预积分；错误SI数量级在入缓冲前拒绝。
- 固定Tag几何包含多Tag联合PnP、平面解歧义/退化检查、重投影内点重分类、外参与参考测量不确定性传播。CameraInfo明确区分原图K+D与矫正图P。
- 求解导数最初通过整图数值差分计算，运动测试曾约377 ms/更新，超过默认300 ms时效预算。现改为解析因子导数并缓存白化/重复残差，独立中心差分检验覆盖非零旋转、偏置、交叉协方差及优化器/局部切空间转换。
- `OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/ros2_ws/src/nexus_fusion_localization/src python3 -m pytest code/ros2_ws/src/nexus_fusion_localization/test/test_platform_window.py code/ros2_ws/src/nexus_fusion_localization/test/test_platform_jacobians.py -q --junitxml=/tmp/nexus_platform_cached.xml` → 25 passed（新增默认窗口场景之前）；运动偏置场景8次更新平均23.56 ms、P95 31.50 ms。
- 默认六状态、两次IRLS、30次混合视觉/UWB/Tag更新，包含Tag丢失、UWB跳变、25次边缘化：专项测试1 passed；平均13.32 ms、P95 15.32 ms。耗时是当前WSL合成用例测量，未包含SuperPoint推理、图像传输与真实飞行，不能证明完整链实时性。
- `colcon build --packages-up-to nexus_fusion_localization nexus_vision_localization --symlink-install` → 3包构建成功；最终两包`colcon test`通过：fusion为57个pytest+7个CTest套件=64，vision为26个pytest+5个CTest套件=31；合计83个pytest用例，95项colcon汇总，0 errors / 0 failures / 0 skipped。
- 初次解析导数后的测量仍有约0.4 s更新，进一步缓存及复测后降低；不以首次通过的正确性测试替代性能检查。Flake8曾报告5处E128续行缩进，已修正。
- 实机相机/IMU/UWB标定、已训练真实可靠性模型与完整链路数据仍缺少，均未声明已验证。

## 第三阶段进行中：SuperPoint与静态几何

- 增加本地ONNX模型描述/校验接口、65通道检测头解码、256维描述子双线性采样、原图坐标回映射、NMS、背景/ROI掩膜与双向ratio匹配。缺权重或SHA256不符明确报错。
- 增加Essential RANSAC、正深度/重投影/覆盖/视差门控、可选IMU旋转检验与五自由度几何协方差；不输出单目米制长度。
- 非零相机杆臂显式进入平台测量方程，解析导数用数值差分复核；平台支持插入视觉边的两个采样时刻，不要求与UWB时刻相同。此变更直接用于避免有效视觉边被错误拒绝。
- SuperPoint后处理专项9 passed；两视图几何5 passed；合并平台回归41 passed（增加异步双端插帧测试之前）。这些是合成头/三维点/传感器测试，尚未执行真实SuperPoint模型。
- 时序前端、同帧检测/掩膜生产、ROS图像至平台全链、参考目标轨迹与ID关联仍是待实现项，不能据此标记B03/C02/E01/E02完成。

## 本轮最终检查与限制

- 环境：Ubuntu 22.04 WSL、Python 3.10.12、NumPy 1.21.5、SciPy 1.8.0、OpenCV 4.5.4、ROS2 Humble；没有安装新的依赖或下载模型权重。
- 最新完整构建、两包colcon测试与结果汇总均通过。此后共享质量topic按subject分流的改动，另行运行`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=213 python3 -m pytest code/ros2_ws/src/nexus_fusion_localization/test/test_fusion_node.py -q` → 2 passed；平台质量消息不再被目标节点错误记录为格式异常。
- 数值相关42个用例（平台窗口23、解析Jacobian 5、SuperPoint后处理9、两视图几何5）通过；单独节点DDS和标定检查不包含在这42个中。禁止将“95项colcon汇总”误写为95个独立pytest用例。
- 新增/修改源码的Flake8检查（仅沿用E501/W503忽略规则）与`git diff --check`通过；已检查相关包清单/构建注册和工作区状态，保留其他成员既有改动。
- **性能仍未达成稳定证明**：最终colcon运行中，运动偏置场景平均249.31 ms、P95 334.63 ms；默认六状态混合源场景平均133.25 ms、P95 166.11 ms。较先前独立运行23.56/31.50 ms和13.32/15.32 ms存在明显波动，不能只选最快一次宣称实时达标。335 ms更新会超过默认300 ms观测时效，节点会明确发布solution_stale；仍需定位耗时波动并在目标运行环境测量完整链路。
- 预训练SuperPoint资产、真实质量训练标签与标定、实机性能/精度数据仍未提供。当前继续推进代码，不因这些缺项把其他可实现部分标为受阻或完成。

## 第四阶段增量：ROS静态图像链、参考目标身份与米制几何数值层

本节为上述阶段记录之后的当前证据，表格已按本节更新；此前阶段的“待实现”保留为历史过程，不代表当前状态。

### 已补全与已验证的部分

- `object_detector.py`/`detection_node.py`提供显式YOLOv5/v8 ONNX描述、图像letterbox、反映射、类别NMS与同帧检测JSON。模型推理失败明确发送invalid，不把失败解释为没有动态目标。
- `static_tracker.py`/`superpoint_node.py`维护有限时长静态参考，对弱视差积累多帧间隔；框或分割掩膜必须与图像精确配对。队列有界，推理和几何超时不提交参考/目标状态。
- `target_identity.py`/`target_frontend.py`提供实例身份、像素CV状态、参考特征/外观/局部几何、一对一分配、五态、歧义冻结、短时预测及重新关联。参考图+ROI可在没有类别框时建立命名轨迹；显式existing_track_id可以绑定已有匿名轨迹。
- ROS参考请求与图像支持乱序到达、历史参考时间、配对超时、处理超时与最近请求幂等；输出像素身份轨迹独立于背景运动是否可观测。所有目标候选和短时丢失预测框参与静态屏蔽。
- 数值层新增`solve_keypoint_pose`，与多Tag板共用鲁棒PnP；`compose_target_pose`传播平台、外参、相对观测及可选跨源相关协方差。这里只完成数值求解，尚未形成新的三维ROS生产链。

### 验证命令与结果

环境仍为Ubuntu 22.04 WSL、系统Python 3.10.12、ROS2 Humble及已有NumPy/SciPy/OpenCV；未安装依赖或下载权重。

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nexus_vision_localization --symlink-install
source install/setup.bash
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_vision_localization nexus_fusion_localization --event-handlers console_direct+
colcon test-result --test-result-base build/nexus_vision_localization --verbose
colcon test-result --test-result-base build/nexus_fusion_localization --verbose
```

- 构建3包成功；完整vision为57个pytest+12个CTest套件=69项，fusion为57个pytest+7个CTest套件=64项。合计 **114个独立pytest用例、133项colcon汇总，0 errors / 0 failures / 0 skipped**。已从各`test_results/*.xunit.xml`逐个汇总确认，不把CTest套件再算成独立用例。
- 新目标相关测试：身份8、参考前端3、ROS目标链4、米制几何5；另保留DDS检测→静态几何→平台因子2例，固定参考原6例几何+2例节点通过。
- 真实DDS链的网络边界显式替换为合成输出；几何/身份/平台求解及消息路由是真实代码。未运行真实YOLO/SuperPoint模型，不证明真实目标识别率、泛化或实机精度。
- 回归测试`test_identical_targets_lost_between_frames_cannot_reassociate_by_old_position`首次为 **1 failed / 7 passed**，发现长时间断帧后旧运动位置仍参与身份恢复。已修正为重新关联不使用过期位置消除歧义，完整回归通过。该失败与修复均保留在记录中。
- Flake8曾报告PnP共用代码2处E128续行缩进，已修正；最终检查新视觉源码、测试及launch时仅沿用E501/W503忽略规则并通过。`git diff --check`通过。
- 已检查vision包的安装入口、测试注册、运行依赖及最终变更；保留仓库其他既有修改，没有操作真实硬件或发送控制命令。

### 仍未完成，继续推进

1. D01/D02/D03/D04：已知模型关键点生产、参考目标多帧对应/三角化、动态目标三维速度/姿态与可观测标记、平台采样时刻位姿和协方差对齐、在线米制目标输出。当前像素轨迹不等于三维定位。
2. B06/F04：替换`code/analysis/localization/online_solver.py`的弱F8携带先验，完成目标窗口真实边缘化及信息相关性处理。
3. C03/H01/H02：多Tag目标基准在线接入、统一双对象启动、RViz/Web交互框选及双输出记录/状态展示。
4. A01–A03、I01–I03：既有采集/桥接当前状态审计，用户要求的7+7消融组合与跟踪/延迟等指标。
5. G01–G03、I04：真实模型权重、真实质量标签、实测标定和完整硬件运行证据。代码能继续实现；这些缺项不能由合成夹具替代。

当前整体状态：**PARTIALLY COMPLETE**。本阶段未扩大任务范围；上述剩余项继续保留在原目标中。

## 第五阶段增量：参考特征至ROS米制目标状态

本节是第四阶段之后的最新证据，已同步更新D02/D03/D04/F03/F05/G01/H01。此前“目标米制前端待补”的阶段记录保留，但不能继续据此忽略本次新增实现。

### 实现范围

- `target_identity.py`/`target_frontend.py`输出稳定参考ID、固定特征锚点、运动模型和经过几何验证的跨帧特征对应。注册可指定参考像素及static/constant_velocity模型；实际输出锚点位置明确报告。
- `pose_timeline.py`提供图像原采样时刻的平台位姿查询和有限插值，保留端点/插值不确定性。未被平台位姿包围的图像在有界队列等待；不使用最新接收位置替代。
- `target_multiview.py`使用多视线RANSAC和Huber优化估计静态p或匀速p/v，检查深度、视差、秩/条件数、速度及当前帧内点。共有外参与跨帧平台误差不能被错误地按独立测量平均。
- `target_metric_node.py`连接像素轨迹、CameraInfo和平台odom，输出新的`TargetKinematicState`及质量/诊断；积累初始化观测，拒绝粗差、超时、未来包及过时失败，支持失效后重新获得观测。
- 新消息明确p/v/q各自的可观测性，未知姿态/速度和相应协方差填NaN。当前位置是固定参考表面特征点；不能在评估时直接与物体中心真值混用。
- `static_visual.launch.py`已接入米制节点；新增明确标为unmeasured的标定模板。原TargetObservation消息保持原接口。节点使用已有nav_msgs/NumPy/SciPy/OpenCV，未安装软件或下载模型。

### 检查和测试证据

环境：Ubuntu 22.04 WSL、ROS2 Humble、Python 3.10.12及已有数值依赖。

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nexus_vision_localization --symlink-install
source install/setup.bash
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_msgs nexus_vision_localization nexus_fusion_localization
colcon test-result --test-result-base build/nexus_msgs --verbose
colcon test-result --test-result-base build/nexus_vision_localization --verbose
colcon test-result --test-result-base build/nexus_fusion_localization --verbose
```

- 构建3包成功。首次完整三包回归通过后，新增非零安装外参/旋转相机与未来invalid包回归检查，再构建并运行`colcon test --packages-select nexus_vision_localization`，最终vision为 **67 pytest + 14 CTest = 81项**。
- 本阶段未修改fusion源码和原TargetObservation接口，完整回归中fusion为57 pytest+7 CTest=64项；msgs为3 pytest+1 CTest=4项。
- 已读取每个xunit文件确认，合计 **127个独立pytest用例、149项colcon汇总，0 errors / 0 failures / 0 skipped**。相较上节额外纳入消息包3个既有用例，不能把127−114全称为新增测试。
- 新数值测试7例：静态粗差、当前帧粗差不刷新、匀速p/v与运动退化、无基线/乱序、共有不确定性、旋转/非零安装外参、平台采样时刻插值。新ROS测试3例：完整参考图到静态/CV米制输出、异步平台输入/插值/超时回滚/恢复/未来包拒绝。
- DDS端到端用例使用真实前端/几何/消息代码，但SuperPoint网络输出与平台odom为合成输入；没有实际YOLO/SuperPoint权重运行，不能称为实机结果或整体实时精度验证。
- 数值测试最初使用无误差预算依据的4mm门限，出现1 failed/5 passed；尝试单样本99%置信界也未通过。独立数值Jacobian最小二乘与Huber拟合复现同一带噪偏差（独立LS NEES约12.61），确认测试不应要求每个随机样本必在给定置信界内。已改为核对独立数值求解的一致性，并保留无噪声真值精确恢复；没有以更换随机种子或放大固定精度门限掩盖失败。
- Flake8最初发现4处E127/E128续行缩进，已修正；最终检查新增/修改的视觉源码、测试和launch通过（仅沿用E501/W503忽略规则）。`git diff --check`通过。
- 已审查消息生成、安装入口、依赖、launch及工作区变更；保留其他未提交工作，没有连接硬件或发送控制命令。

### 剩余要求保持不变

本分支只解决带固定参考点及static/CV假设的米制几何。以下内容仍未完成：

1. 新TargetKinematicState到目标融合/学习型权重/短时米制预测及RViz/Web的实际连接；原目标融合只消费旧消息，不能宣称新目标输出已经融合展示。
2. 旧目标在线窗口弱F8替换为真实Schur边缘化及联合信息相关性处理。
3. 已知模型自动关键点生产、多Tag目标基准、无纹理轮廓/尺寸/支撑面在线分支、动态目标完整姿态。
4. 完整双对象统一启动、交互框选、rosbag记录、7+7消融配置、双对象及身份/延迟指标口径。
5. 采集/桥接当前代码审计、真实模型资产、质量标签、标定与实机精度/性能证据。

当前整体仍为 **PARTIALLY COMPLETE**。本阶段修改视觉包、必要的新消息及配套文档，均属于原目标；未扩大任务范围。

## 第六阶段增量：目标窗口真实边缘化

本节更新 B06/F04，前述“目标窗口弱F8仍待替换”是此前阶段的历史状态，当前已由以下实现替代。整体任务仍未完成。

### 代码与对应要求

- `bundle_marginalization.py`：目标/相机/段级变量局部线性化、粗差拒绝、冻结IRLS、QR消元/SVD压缩，保留交叉块与不可观测零空间。
- `online_solver.py`：在N+1帧候选图求解后只边缘化离开窗口的测量与旧历史因子；F3/F8固定先验去重；保留已优化相机热启动；数值/边缘化失败原子回滚；重复和倒序帧拒绝。
- `bundle_solver.py`：已白化历史因子参与原有稀疏优化，历史行不再次应用鲁棒损失或单帧离群门控；历史几何按目标隔离，并继续检查位置秩。
- `factors.py`：接入平方根因子；轮廓残差计算不再原地修改观测法线，避免多次线性化改变输入。
- `test_target_marginalization.py`：新增19项独立数值/真实因子图回归；接口细节见 `architecture/2026-09-03_design_target_window_marginalization.md`。

### 本次验证

环境：Ubuntu 22.04 WSL，系统Python 3.10和已有NumPy/SciPy；未安装依赖、未下载模型、未连接硬件。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test/test_robust_bundle_regressions.py code/analysis/test/test_online_frontend.py code/analysis/test/test_localization_bundle.py -q
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test/test_target_marginalization.py -q
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/analysis python3 -m pytest code/analysis/test -q --junitxml=/tmp/nexus_target_marginalization_regression.xml
python3 -m flake8 --ignore=E501,W503 code/analysis/localization/online_solver.py code/analysis/localization/bundle_marginalization.py code/analysis/localization/bundle_solver.py code/analysis/localization/factors.py code/analysis/test/test_target_marginalization.py
git diff --check
```

- 原相关回归 **29 passed**；新增专项 **19 passed**；离线完整测试 **88 passed**，不是29+19+88个独立用例。
- Flake8首次失败：bundle求解器续行缩进E128与StateView已有单行Protocol方法E704。已修正对应格式，最终仅忽略原约定E501/W503的检查通过；`git diff --check`通过。
- 已审查相关源码、未跟踪新文件和工作区diff；其他既有未提交工作保留。本阶段没有ROS源码或消息变更，因此没有把上一阶段的colcon结果表述为本次新增ROS集成验证。
- 高斯/无噪声图的信息对照不证明所有非线性、强退化或带噪实机序列与全量重优化完全等价；边缘化采用局部线性化，鲁棒权重冻结，协方差统计校准与真实精度仍待数据验证。
- 本轮测试耗时受WSL运行条件影响；没有依据最快一次运行宣称100ms更新预算或系统实时性已经达成。

### 尚未完成

新 `TargetKinematicState` 到融合/学习质量/短时预测及RViz/Web；模型关键点、多Tag目标基准、无纹理几何在线分支、一般动态姿态；完整双对象统一启动/交互框选/记录；7+7消融及身份/延迟指标；采集桥接审计、真实模型/标定/数据。上述仍属于原目标，未减少要求，也未标记为完成。

下一步接口核验已确认：`fusion_node.py`仍只订阅旧`TargetObservation`，其`robust_filter.py`输入要求有效四元数且只消费位置3×3协方差；新消息允许未知姿态NaN并携带p/v交叉协方差，不能直接强填单位四元数或丢掉速度块后声称已完整融合。`TargetMetricNode`的相邻输出来自重叠多帧窗口并共享平台/外参信息，接入时必须处理估计间相关性，不能把每次输出都当新的独立测量无限累积信息。当前只是已核验的下一步实现边界，未宣称这一连接已完成。

本阶段只补全原目标中 B06/F04 的目标窗口部分及必要测试文档；**未扩大任务范围，整体 PARTIALLY COMPLETE**。

## 第七阶段增量：新米制目标状态到相关融合

本节为第六阶段之后的当前证据，更新表格D03/D04、E02、F01/F02/F04/F05、G01/G02、H01；之前“新消息到融合尚未接入”保留为历史过程。整体目标仍未完成。

### 已实现的主链

- 新增`kinematic_filter.py`：在共同局部坐标中对p/v/SO(3)估计做CI，保留所有可用交叉块。重复相关估计不被当作独立新信息；未知分量保持NaN，旋转的map角协方差转换通过数值Jacobian对照。
- 新增`target_kinematic_node.py`，直接消费`TargetKinematicState`并发布`/nexus/target/kinematics`。支持硬创新拒绝、可疑降权、固定/规则/学习尺度、来源/参考点/时间精确质量配对、乱序等待和缺失掩码。
- 有限预测明确为invalid/非观测，last_valid_sample_timestamp_ns不刷新；速度/姿态自身的支持时间也会过期。身份歧义或参考点切换待确认时不预测旧点；恢复依靠连续一致新证据。融合命中次数不能提升前端tentative身份。
- 输入过期/未来/倒序、状态求解过期及上游invalid不会静默污染状态。同帧invalid会取消尚未处理的有效包；较旧invalid不覆盖新状态。候选滤波器只有在结果仍及时且数值有效时提交。
- `TargetMetricNode`质量包增加source与position_reference；原source_mode和特征字段继续保留。原消息结构未加字段，只追加预测语义注释；旧TargetObservation路径未被强制转换或删改。
- `static_visual.launch.py`默认接入新融合节点，另有`target_kinematic_fusion.launch.py`独立入口；依赖和CMake安装/测试登记已补齐。新增topic与语义见ADR-012及`architecture/2026-09-03_design_target_kinematic_fusion_interfaces.md`。

### 验证证据

环境：Ubuntu 22.04 WSL、Python3.10、ROS2 Humble及已有NumPy/SciPy/OpenCV。没有安装新的数值依赖、下载模型或连接硬件。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=code/ros2_ws/src/nexus_fusion_localization/src python3 -m pytest code/ros2_ws/src/nexus_fusion_localization/test/test_kinematic_filter.py -q
cd code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nexus_vision_localization --symlink-install
source install/setup.bash
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_msgs nexus_fusion_localization nexus_vision_localization
colcon test-result --test-result-base build/nexus_msgs --verbose
colcon test-result --test-result-base build/nexus_fusion_localization --verbose
colcon test-result --test-result-base build/nexus_vision_localization --verbose
ros2 pkg executables nexus_fusion_localization
ros2 launch nexus_fusion_localization target_kinematic_fusion.launch.py --show-args
ros2 launch nexus_vision_localization static_visual.launch.py --show-args
```

- 新数值测试最初16项通过，补上前端tentative保护后最终 **17 passed**。
- 新ROS测试最初3项，加已知姿态/未知速度消息的DDS参数用例后最终 **4 passed**；已有视觉包的参考图到米制测试扩展为继续验证融合输出，包含static/CV两种情况，测试数量不重复加算。
- 三包完整构建和回归通过；最后补充状态保护后再次构建并运行fusion/vision两包全部测试。直接读取各xunit确认：msgs **3 pytest + 1 CTest=4**，fusion **78 pytest + 9 CTest=87**，vision **67 pytest + 14 CTest=81**。合计 **148个独立pytest用例，172项colcon汇总，0 errors / 0 failures / 0 skipped**。
- 新增执行文件`target_kinematic_fusion_node`在安装列表可见，两个launch的`--show-args`正确列出模式/模型及新融合启用参数。该检查没有启动实际设备或模型。
- Flake8曾报告新代码的E127/E128续行缩进，已修正；新增源码/测试/launch及改动的视觉节点按既有E501/W503忽略规则通过。`git diff --check`通过。
- 数值测试采用独立信息公式、标量搜索网格和旋转有限差分；DDS使用实际ROS消息与节点。图像前端端到端测试的网络输出/平台输入、质量网络训练数据均明确为合成测试，不作为实机或训练泛化结果。
- 同一环境最后两包回归约2分59秒，而之前约21秒，存在显著运行时波动。没有据此宣称目标100ms预算、完整实时性或实机精度达标。

### 仍须继续的原要求

RViz/Web新状态与预测显示、双对象统一bringup及rosbag记录；模型关键点/PnP、多Tag目标基准、无纹理几何在线分支及一般动态姿态；完整共享因子图；7+7消融、身份/丢失/恢复/延迟指标；树莓派采集与桥接审计；真实模型资产、标定、质量标签和复杂环境数据。

本次完成的是新目标米制消息到融合与启动链的连接，没有把CI消息融合冒充完整联合后端，也没有将软件合成验证写成真实精度结论。**整体 PARTIALLY COMPLETE；未扩大任务范围。**
