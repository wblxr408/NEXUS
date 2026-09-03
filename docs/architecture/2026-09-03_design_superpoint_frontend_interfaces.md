# SuperPoint前端接口与验证边界

适用ADR-011的B03/B04/C02。SuperPoint模型只负责检测与描述；匹配、几何验证、目标身份和三维定位分别处理。使用现有OpenCV DNN执行本地ONNX，避免ROS系统Python强依赖完整训练框架。缺模型不回退到其他特征并称为SuperPoint。

## 模型输入/输出

模型描述JSON schema_version=1，architecture=superpoint_dense_v1，包含model_file（相对描述文件或绝对路径）、sha256、input_name、detector_output、descriptor_output、input_size_wh、descriptor_sampling=lightglue_v1、source与license。模型文件必须存在且SHA256相符；权重不随代码分发。source/license登记资产来源，不替代授权。

输入为float32、[1,1,H,W]、灰度[0,1]；H/W是8的倍数。输出检测logits [1,65,H/8,W/8]（前64通道为8×8位置，第65为dustbin），描述子[1,256,H/8,W/8]。只接受此明确格式，不猜测不同ONNX导出的稀疏输出含义。

图像保持原始像素坐标约定，输入网络前resize，输出点按像素中心映射回原图，后续几何使用原图对应CameraInfo。描述子按已声明的lightglue_v1采样坐标双线性插值并L2归一。参考：[原作者仓库](https://github.com/magicleap/SuperPointPretrainedNetwork)、[LightGlue描述子坐标约定](https://github.com/cvg/LightGlue/blob/main/lightglue/superpoint.py)。上游SuperPoint资产带独立研究许可，不能因本ROS包许可而改变权重许可。

## 特征与屏蔽

FeatureSet包含原图points_px [N,2]、L2 descriptors [N,256]、scores [N]与image_size_wh。掩膜True代表允许使用的像素；背景掩膜由同帧动态分割/目标框和外扩边界产生。屏蔽发生在选点/NMS之前，避免大量动态强特征挤占背景点配额。一张图只推理一次，密集输出可以分别为背景与指定ROI选点。

匹配使用归一描述子的欧氏距离、双向最近邻一致性、双向ratio检验与最大距离；仅输出对应关系，不把匹配数量或检测框中心当作三维精度。重复外观造成的等距匹配拒绝。后续静态视觉需几何验证与视差检查，目标跟踪需独立身份状态；这两部分不由特征提取器隐式完成。

## 验证

单元测试注入的是明确标注的合成密集头输出，只检验通道解码、resize/插值、NMS、掩膜、匹配以及输入错误；不得据此声称SuperPoint模型已推理或精度已验证。当前环境尚未提供本地可运行的SuperPoint ONNX资产与实机序列，实际推理及端到端集成保留未验证状态。

## 静态两视图几何

`visual_motion.estimate_static_motion`消费已经屏蔽动态区域的FeatureSet及原图相机内参/畸变，使用双向匹配、归一化坐标Essential RANSAC、正深度与重投影检验、特征覆盖和最小视差门控。可提供独立IMU相对旋转作一致性检查。纯旋转、低视差、几何病态或匹配不足时拒绝，不返回零平移伪装有效结果。

输出`R_camera_i_camera_j`和单位相机位移方向。协方差由内点Sampson残差对三维旋转与二维单位球切平移的Jacobian计算，像素噪声由配置声明；三维长度不在此估计中。6×6接口沿单位方向的径向块设为弱信息，平台仍按5个自由度门控。协方差条件于已知内参与外参，外参完整不确定性在融合端仍待扩展，不能当成已校准的实机误差。

`VisualMotion.body_observation`输出平台JSON观测。它只旋转相机方向并保留`sensor_body_m`杆臂，平台测量方程使用传感器原点位移；不会在未知单目尺度下直接套用完整SE(3)平移变换。平台会插入相对视觉两个实际采样时刻，不依赖UWB帧对齐。当前这条链在数值接口测试覆盖，ROS图像节点及目标身份关联仍待接通。

## 本轮ROS接入约定

- `object_detection_node`接收`/camera/image_rect`，从本地YOLO ONNX生成`/nexus/vision/detections`（String JSON）。字段：schema_version=1、sample_timestamp_ns、frame_id、image_size_wh、valid、reason、detections（class_id、label、confidence、bbox_xywh_px）、dynamic_boxes。类别不是track_id。
- `superpoint_motion_node`接收同一图像、CameraInfo及同帧检测JSON；也可显式选择`mask_input=segmentation`接收`/nexus/vision/dynamic_mask`（mono8 Image，非零为动态像素）。必须先验证帧、尺寸、有效性和原采样时间。检测/掩膜失效不等同于空动态区域。
- 图像和动态观测队列按原采样时间匹配，缓冲有上限，处理最新的完整配对。处理完成后仍检查时效；不把过期推理结果更新为当前参考帧。
- 静态时序跟踪保留可用参考帧以积累视差；超过参考时限或相机几何改变时重新建立参考。异常、低纹理、低视差只输出状态，不伪造相对运动。
- 输出`/nexus/vision/body_motion`、`/nexus/observations/quality`与`/nexus/vision/motion_status`。质量包与运动观测保持相同原时间戳；状态记录网络与处理耗时、队列丢弃、匹配和几何拒绝原因。
- 视觉标定YAML必须声明schema_version=1、calibration_id、calibration_status=measured、camera_frame、image_rectified、transform_body_camera；测试标定只能显式opt-in。模型描述路径为单独启动参数，缺模型立即报错，不启动替代检测器。

YOLO ONNX接口支持显式声明的`yolo_v8_detection`（4+C）和`yolo_v5_detection`（5+C）输出，layout指定channels_first或anchors_first，class_names与输出通道严格对应。输入为RGB [0,1]、固定尺寸letterbox；模型描述包含input_size_wh、padding_value、input_name、output_name及SHA256/source/license。输出类别分数必须已激活为概率。暂不声称该检测头支持实例分割；分割可通过上述独立掩膜接口接入。
