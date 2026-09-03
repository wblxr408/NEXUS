# 相关目标状态融合的数值与ROS接口

依据ADR-011/ADR-012；对应D03/D04、E02、F01/F02/F04/F05、G01/G02及H01。实测标定、质量模型和复杂环境效果仍按逐项清单单独验收。

## 输入、状态与信息相关性

`kinematic_filter.py`提供 `KinematicEstimate` / `KinematicTargetFilter`，不依赖ROS。输入必须明确目标ID、采样/接收时间、source、position_reference、frame和米制单位。位置必须可观测，速度与旋转可分别缺失。9×9协方差顺序是 `[p_map, v_map, theta_map]`；已知分量子块必须有限、对称、正定，未知分量对应的整行整列为NaN。姿态使用SO(3)，角误差在map固定轴中表达。

同一个目标ID只有相同的 `position_reference` 才可直接融合。这里仍沿用前端的固定表面参考特征点，不能把此点与物体中心或另一个参考点混合。前端tentative状态不会被滤波命中次数提升为confirmed。

相邻多帧解共享输入。对预测和当前估计，分别构造信息矩阵，求：

`P = (w * P_pred^-1 + (1-w) * R_obs^-1)^-1`

并使用对应信息向量组合均值。部分观测通过选择矩阵嵌入已知分量的并集；缺失分量只提供零信息，不产生人工测量。权重在0–1上最小化log(det(P))，检查可用端点；两个完全相同的相关估计反复输入不能因此缩小协方差。

姿态在共同参考姿态的Log坐标中组合。输入map角协方差经左Jacobian逆转换，输出再经左Jacobian恢复至map固定轴，连同位置/速度/旋转交叉项一起变换。不在接近pi的Log分支上强行插值。该处理属于局部高斯近似；非高斯、多模态身份、错误标定和运动模型失配不能由CI自动解决。

## 时间、门控、预测和恢复

- 逐目标/来源拒绝重复和倒序采样；不同来源同一时刻可以参与CI，但不累加连续帧确认次数。
- 使用原采样时间预测。已知速度采用CV及加速度过程噪声；速度未知时只作位置保持并增长未知运动协方差。没有角速度观测时保持姿态均值并增长角速率不确定性，不虚构角速度。
- 共同可观测分量执行完整马氏门控，阈值按3/6/9维卡方概率计算。未知相关性下以 `2(P+R)` 作为差值协方差的保守界；因此门控不是精确已知相关分布的统计检验。
- 当次硬门控使用未加学习尺度的观测协方差。可疑观测再按创新和质量放大协方差，明显离群不改变已接受的位姿。CI及有效协方差均在提交前检查。
- 速度/姿态只能在有限时间内沿用其最近观测约束，持续的位置更新不会无限延长这些分量的可用性。
- 超过预测时限、身份歧义或参考点更换后，要求连续一致的新观测才能恢复；此时不依赖旧位置决定身份。身份歧义和参考点变更待确认期间不发布旧点的运动外推。
- `maximum_tracks`、每目标来源数、保留时长以及节点待处理/质量队列均有界。完全超出保留时间的轨迹被移除，后续按tentative新初始化，不伪称已经恢复旧身份。

算法默认值：max_age=250ms、prediction_horizon=500ms、retention=5s、confirmation_hits=3、maximum_tracks=64。加速度sigma=2m/s²、未知速度sigma=1m/s、角速率sigma=0.5rad/s、最高速度=10m/s、门控概率0.95/0.999。这些是软件默认参数，不是实机标定结论。

## ROS消息与质量配对

节点可执行名 `target_kinematic_fusion_node`；默认输入 `/nexus/vision/target_kinematics`，输出 `/nexus/target/kinematics`。

| 输出情况 | valid / observed | 数值 | last_valid_sample_timestamp_ns |
|---|---|---|---|
| 接受的新估计 | valid=true；位置true，速度/姿态按可用观测信息 | 已知分量有限，未知分量NaN | 本次接受的原采样时刻 |
| 短时预测 | valid=false；三个observed均false；reason=prediction_only | 可预测分量有限，未知分量NaN；confidence=0 | 保持最后接受时刻 |
| 预测过期/身份歧义/拒绝 | valid=false；三个observed均false | 位置/速度/姿态和协方差NaN | 保持最后接受时刻 |

输出header在新估计中是原采样时间，在预测中是预测所对应的时刻。不得只读有限位置就将预测当成新观测；必须同时处理valid、reason、observed和last_valid_sample_timestamp_ns。新节点不订阅自己的输出，也不把预测反馈进滤波器。

共享 `/nexus/observations/quality` 的新增精确配对字段示例：

```json
{"schema_version":1,"subject":"target","target_id":"chosen","source_mode":3,"source":"superpoint_multiview_static","position_reference":"reference_feature:HASH:0","sample_timestamp_ns":1234567890,"features":{"inlier_ratio":0.9,"reprojection_error_px":0.4}}
```

只匹配目标、source、参考点和采样时间全部一致的质量包。平台质量、旧源质量、旧参考点或相邻帧质量都不会被借用。learned模式默认等待30ms，允许质量与估计乱序；到期仍缺质量时，现有MLP使用缺失特征掩码及真实延迟/时间间隔/可用速度，不切换成随机模型或借用上一帧特征。状态诊断中的quality_matched说明是否匹配成功。

等待质量期间收到较新的或同帧的上游invalid，会取消该目标已被覆盖的待处理包，保留有界失效时间记录，阻止迟到副本复活。未来采样/接收时间以及较旧invalid不会污染当前状态。求解完成后再次检查数据年龄，过期结果不提交状态、来源时间或恢复计数。

诊断 `/nexus/target/kinematic_fusion_status` 为String/JSON，包含schema_version=1、target_id、sample_timestamp_ns、reliability_mode、reason；有效处理另含source、position_reference、decision、d2、covariance_scale、ci_weight、track_state、quality_matched和计数。等待/丢失/非法输入只有适用字段，不伪造未计算的创新。

## 启动与验证边界

```bash
source code/ros2_ws/install/setup.bash
ros2 launch nexus_fusion_localization target_kinematic_fusion.launch.py
# 使用已有训练模型时：
ros2 launch nexus_fusion_localization target_kinematic_fusion.launch.py \
  reliability_mode:=learned reliability_model:=/path/to/trained_quality.npz
```

`static_visual.launch.py`现在默认启动该节点；使用该启动链时不要再重复运行独立融合launch。可通过enable_target_fusion关闭；target_reliability_mode和target_reliability_model分别传给新节点。平台odom仍需由已有平台节点提供，尚不是包含所有标定、固定Tag、记录和展示的统一bringup。

数值测试包含独立CI信息公式与标量权重对照、重复相关信息、p/v交叉项、SO(3)数值Jacobian、部分观测、三档门控、预测、不确定身份/参考点/长丢失恢复、缓存上限及前端tentative保护。ROS测试使用真实消息和DDS，覆盖未知姿态、已知旋转、质量乱序/缺失、状态过期、同帧invalid取消和限时回滚。参考图→SuperPoint特征→多帧米制解→融合输出测试已接入，但网络输出和平台输入为合成数据。

软件通过不等于实机精度或学习泛化通过。CI不替代原任务中的完整联合共享因子图；新消息的RViz/Web/rosbag消费、自动关键点与多Tag姿态生产、统一启动和消融评估仍按清单继续实现。
