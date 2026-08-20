# 步骤 02：冻结接口、坐标、单位和记录格式

- 日期：2026-08-20
- 类型：执行指导
- 状态：draft
- 对应总计划：步骤 2
- 关联文档：`docs/architecture/interfaces.md`、`docs/architecture/coordinate-frames.md`
- 本文范围：冻结数据契约和语义，不讨论具体 bridge 代码实现。

## 先给结论

信息通道不是简单转发 topic。必须先规定字段含义，再选择传输方式。一个位置数值如果没有对象、frame、单位和采样时间，就不能进入算法或答辩图表。

## 一、消息分层

### 平台状态

表示无人机自身的位置、姿态、速度和 IMU，属于 `base_link` 或平台世界状态。不能因为字段相似就改名为目标位置。

### 目标观测

表示 UWB、视觉或其他传感器对外部目标的观测，必须有 `target_id` 和来源模式，可处于 `camera_i`、`uwb` 或 `map` frame。

### 目标世界输出

表示 `target_link` 在 `map` 中的最终估计，是项目演示和评估主输出。

### 真值/评估消息

表示独立测量的参考位置、评估时刻和有效性。真值不能由待评估算法自己生成。

## 二、最小字段契约

建议目标观测至少包含：

```text
schema_version
sequence
sample_timestamp_ns
receive_timestamp_ns
frame_id
target_id
source_mode
position_m
orientation_or_none
covariance
confidence
validity
configuration_id
```

平台状态可以没有 `target_id`，但必须通过消息类型或 `platform_id` 明确区分。

## 三、时间语义

- 采样时间：传感器实际观测时间，用于评估和多源配对；
- 接收时间：ROS/bridge 收到数据的时间，用于计算链路延迟；
- 显示时间：网页或 RViz 刷新时间，只用于界面。

每条记录保留采样时间和接收时间。设备时钟不可比较时，标为 `not_measured`，不能伪造同步精度。

## 四、坐标和单位

必须冻结：

- `map` 是否为沙盘世界坐标；
- `camera_i` 的轴向；
- `base_link` 的机体轴；
- UWB 的 NED/ENU 或厂商自定义轴向；
- 所有长度是否统一为 m；
- 角度、四元数和协方差排列约定。

建议 ROS2 算法层统一使用米制和确认后的右手坐标系。单位转换只允许在适配层进行，不能在算法内部猜单位。

## 五、有效性和过期规则

建议定义：

| 状态 | 含义 | 是否可融合 |
|---|---|---|
| `valid` | 字段、时间、frame 均有效 | 是 |
| `stale` | 超过允许年龄 | 否 |
| `invalid_frame` | frame 未识别或不一致 | 否 |
| `invalid_unit` | 单位未确认 | 否 |
| `missing_target` | 缺少目标 ID | 否 |
| `transport_error` | 丢包、乱序或版本错误 | 否 |

无效数据不能静默变成零值、上一帧或其他来源复制值。任何补齐策略必须写入实验协议。

## 六、ROS1、ROS2、网页职责

### ROS1/官方层

读取 `fcu_core`，保留官方时间，执行已确认的单位/轴向转换，输出契约字段，不猜未知字段。

### ROS2 算法层

接收契约数据，执行视觉、坐标转换、融合和评估，拒绝 frame/单位/目标 ID/时间不一致的数据，输出 `/nexus/target/pose`。

### Dashboard 层

展示状态和结果，不负责坐标转换、真值生成、融合权重或指标计算；不允许网页临时修改正式实验标定参数。

## 七、可直接借用的方案

- `nexus_msgs` 目标观测字段；
- 官方 `fcu_core` 硬件兼容层；
- 模块图中的 rosbridge/ros2 bag；
- ROS2 `tf2` 坐标树。

## 八、必须自行确定的内容

- UWB 消息对象和 `source_mode`；
- bridge 部署位置、频率和重连规则；
- 过期、配对和丢包阈值；
- 协方差来源和版本；
- 无效数据的降级策略；
- 页面显示无数据和过期的方式。

## 九、接口登记模板

```text
message_type:
topic:
semantic_object: platform / target_observation / target_world_pose / truth
source_mode:
sample_timestamp:
receive_timestamp:
frame_id:
unit:
target_id:
covariance_definition:
expiry_rule:
invalid_rule:
schema_version:
configuration_id:
owner:
```

## 十、评审顺序

1. 目标负责人确认对象语义；
2. 坐标负责人确认 frame 和单位；
3. 实验负责人确认时间、真值和指标；
4. ROS 负责人确认可映射性；
5. Dashboard 负责人确认可展示性。

## 十一、产出物

- 接口总表；
- 字段语义字典；
- frame/单位/时间约定；
- 错误状态表；
- bridge 映射草案；
- schema 版本规则。

## 十二、通过与停止条件

### 通过条件

- 模拟消息可区分平台和目标；
- 每个坐标值都有 frame 和单位；
- 每条目标观测都有时间、ID、来源和有效性；
- 错误不会被静默补写；
- 改变传输方式不改变语义。

### 停止条件

- UWB 对象未确认；
- NED/ENU 或米/厘米未定；
- 采样时间和接收时间混用；
- 页面、算法、bridge 使用不同字段解释。

## 十三、禁止越界

- 不在 ROS2 算法层解析厂商私有串口；
- 不用 topic 名称替代对象语义；
- 不把接收时间冒充采样时间；
- 不把无效数据展示为 0 或上一帧；
- 不在接口未冻结前扩展复杂融合和回传控制。
