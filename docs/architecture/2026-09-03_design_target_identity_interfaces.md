# 指定目标的身份与像素轨迹接口

对应C02/E01/E02。该层输出像素框与身份，不把像素状态当作map三维状态。三维位置/速度/姿态仍由PnP、多帧几何与平台位姿链估计。

- 每个实例维护独立track_id、参考SuperPoint特征、最近可靠特征、HSV外观、类别、像素框、中心/速度协方差、最近观测时间与身份置信度。
- 类别检测候选不自带实例ID。关联由运动马氏距离、描述子匹配、局部几何验证、外观距离及尺度一致性组成，并执行一对一分配。
- 参考图或首帧ROI可显式注册命名目标；纹理匹配可在没有类别检测时产生候选框。局部单应性只用于二维关联/候选定位，不把它当作一般三维目标的姿态解。
- tentative/confirmed/degraded/lost/re-associated为身份轨迹状态。丢失时只允许有限时间像素预测，且observed=false；不产生新三维观测。显式注册目标保留参考以待重识别。
- 多个候选或多个轨迹的关联代价接近时，禁止强行一对一猜测：标记身份歧义、增大协方差、冻结外观/参考更新。发生过身份歧义后，单凭后来分开的运动位置不能自动恢复身份，必须重新获得可区分的外观/特征证据。
- 可输入已知的相机图像旋转/投影变换来修正像素运动预测；调用者必须保证其对应上一处理帧和当前帧，不能套用其他时刻的变换。

参考图初始化与真实连续图像效果必须使用实际模型和采集数据验证；合成特征测试只能证明分配、状态和异常处理逻辑。无纹理且外观相同的完全遮挡交叉，身份仍可能不可观测。

## ROS参考输入与轨迹输出

`superpoint_motion_node` 设置 `enable_target_tracking=true` 后开启。`static_visual.launch.py` 默认开启，单独运行节点时默认关闭以保留静态视觉入口。

1. 向 `/nexus/vision/target_reference_image` 发布 `sensor_msgs/Image`：参考图或首帧框选使用的原图，保留其时间戳和相机光学frame。
2. 向 `/nexus/vision/target_requests` 发布 `std_msgs/String` JSON（图像和请求允许任意到达顺序）：

```json
{
  "schema_version": 1,
  "request_id": "selection_001",
  "target_id": "chosen_object",
  "sample_timestamp_ns": 1000000000,
  "frame_id": "camera_optical_frame",
  "bbox_xywh_px": [40.0, 100.0, 40.0, 40.0]
}
```

上述数值仅说明接口。`sample_timestamp_ns` 必须与参考图完全相同，frame必须与标定相机一致，ROI为该参考图的像素坐标；参考图可早于实时窗口，不用历史图初始化当前运动。可选 `class_id` 为非负整数；若用户是在现有匿名轨迹上命名，显式传 `existing_track_id`，保留已有状态并绑定参考，避免重复创建同一实例。

- `/nexus/vision/target_request_status`：`pending/registered/rejected` 与原因。请求和图像缓存均有上限，等待或处理超过 `reference_timeout_s`（默认10秒）拒绝；最近64个已完成请求ID幂等返回，不重复推理/注册。
- `/nexus/vision/target_tracks`：schema1、原采样时间、frame、图像尺寸、`unit=px`、valid、每个track的状态/身份置信度/歧义/框/像素速度/协方差/最近实际观测时间/几何内点数。
- `covariance_px` 为行优先4×4，状态顺序 `[cx_px, cy_px, vx_px_s, vy_px_s]`，故各块单位分别为px²、px²/s及px²/s²。
- 当前该topic明确 `world_position_observed=false`、`orientation_observed=false`；不能直接转写成米制 `TargetObservation`。
- 类别候选和参考候选使用同一次密集SuperPoint推理。参考目标未被类别检测器发现时，仍可通过全图特征对应生成候选。所有目标候选以及短时丢失目标的保守预测框，均在静态背景选点前屏蔽。
- 框模式下检测包必须与图像同采样时刻；分割模式下掩膜必须同采样时刻，分割包不提供类别候选，但已注册参考仍可跟踪。缺少或失败的同帧动态输入不会被解释为没有动态物体。
- 像素目标轨迹独立于静态视觉运动的成功与否输出；超时推理/几何处理不提交任何新前端状态。watchdog以原样本时间发布lost/observed=false，不把墙上时钟推进为新观测。
- 长时间没有图像或目标丢失后，旧像素位置不能用于消除身份歧义；恢复依赖参考特征/外观/局部几何，仍无法区分时保持歧义。

## 验证边界

`test_target_identity.py`：同类目标、次序交换、遮挡交叉、长期断帧后的歧义、可区分外观恢复、有限预测、容量及事务回滚。

`test_target_frontend.py`：参考图独立发现、丢失掩膜、用户命名绑定与尚未提交的状态。

`test_target_pipeline_nodes.py`：真实DDS参考请求/图像与像素轨迹；同帧分割、缓存上限、参考超时、实时处理超时回滚与watchdog恢复。仅网络输出使用显式合成夹具；未验证真实SuperPoint或检测模型的效果。

## 后续接入：固定参考点的米制位置

像素包现在附带稳定参考ID、参考特征编号及其当前几何内点对应。注册可指定`motion_model`（static或constant_velocity，默认后者）与`anchor_reference_px`；节点会报告实际选中的最近SuperPoint特征点。`target_tracks`仍保持像素语义与world_position_observed=false，米制结果在新增`target_kinematics`topic单独输出。位置定义、可观测性、协方差与时刻对齐见`2026-09-03_design_target_multiview_pipeline.md`。
