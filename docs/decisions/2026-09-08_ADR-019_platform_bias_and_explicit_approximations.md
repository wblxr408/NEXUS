# ADR-019：实测初始 IMU 零偏与显式实验近似

日期：2026-09-08。范围：平台 UWB–IMU 验收，证据 E042。

用户要求完成 IMU/平台定位检查，并在不知道 UWB–IMU 杠杆臂时明确允许近似。9° 地图偏航偏置保持。用户确认标签高于沙盘底部约 12–14 cm、基站高度采用相同零点、机头为前方，并执行三轴和手动平移检查。

静置测得非零陀螺偏置。平台初始化此前固定使用零 bias；现在允许 `imu.initial_gyro_bias_rps`、`imu.initial_accel_bias_mps2` 两个可选三维有限向量，含义均为转换后的机体坐标分量。缺省仍为零。实测偏置必须关联采集，单次静置加速度均值不能宣称完整三轴零偏标定。

新增 `calibration_status: experimental_approximation`，默认拒绝；仅当显式设置 `allow_approximate_calibration=true` 且配置含 `approximations` 时可用。原 measured / synthetic_test 边界不放宽，实际 status 消息增加 calibration_status 和 approximations，避免实验模型被误认成已标定模型。独立 platform launch 暴露此参数，默认 false。

E042 采用用户允许的共点近似 `tag_body_m=[0,0,0]`，这是未量得杠杆臂的模型选择，不是实测位置。机体参考原点定义在 IMU；尺度单位和轴符号由原始采集及手动测试核对。原始四槽、异常记录与时间缺口全部保留。

不通过以下方式获得“完成”：把近似写成 measured、放宽 IMU 最大缺口阈值、将缺失样本插值成实测、用飞控姿态/位置冒充独立真值、将 UWB 拟合残差当作定位精度。若融合验收失败，记录失败并暂停把结果作为有效平台定位输入。

采用独立实验 opt-in 而不修改历史 measured 标定或复用 allow_test_calibration，是为了让真实数据、近似参数与合成测试保持可追溯。本决定不修改飞控参数、固件、9° 偏置或飞行控制路径。
