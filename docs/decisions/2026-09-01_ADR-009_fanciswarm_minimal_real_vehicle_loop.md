# ADR-009: FanciSwarm 最小真机闭环演示扩展

日期：2026-09-01

## 决策

在不修改历史“机外目标定位”规划结论的前提下，增加一条受控的 FanciSwarm 真机演示链路：Mcontroller 通过 Raspberry Pi 的 MAVLink/UART 与 Ubuntu UDP 互通；Ubuntu 运行六个 UWB 算法和独立自主飞行程序。树莓派仅做透明字节桥接，六算法只读取 `BATTERY_STATUS.voltages[2:6]` 的四路距离，飞控位置只用于安全检查和对比显示，不回馈定位算法或飞控控制。

实时结果明确标记为 `2D/XY-only`。固定基站、Tag ID `2`、9 度坐标偏航和 0.05 m 测距标准差属于本演示配置；它们不是精度实测结论。自主飞行使用标准 pymavlink ARM、TAKEOFF、LOCAL_NED setpoint 和 LAND 接口，并以心跳、四路 UWB、连续飞控位置及 ACK 作为进入和继续任务的前提。异常时停止新航点并尝试降落。

## 范围与限制

本次扩展不依赖 ROS 或 `fcu_core` 源码，不改变离线 3D runner 默认行为，也不宣称未连接硬件时的真机成功。首次真机验收必须另行记录抓包、运行配置、代码版本和结果。

