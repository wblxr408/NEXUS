# 幻思硬件接口基准

核对日期：2026-08-18

## 已确认

| 部件 | 已确认事实 | 项目角色 |
|---|---|---|
| Mcontroller V7 | STM32H743、FreeRTOS、MAVLink、USB/串口、SD 日志、内置传感器与 EKF 相关库 | 实时飞控与安全闭环 |
| FanciSwarm UWB | 4 个基站 + 机载 DWM1000 标签；位置在机载端分布式解算；厂商标称三维精度小于 10cm，实际随环境/布局变化 | 室内绝对定位基线 |
| fcu_core | 幻思官方 ROS 工程；Ubuntu 20.04 + ROS Noetic + catkin；支持机载电脑和远程主机 | ROS 通信基线 |
| 通信 | 官方示例支持 USB `/dev/ttyACM0`，也支持网络 `192.168.4.1:333`；默认示例波特率为 460800 | 飞控↔ROS 链路 |
| ROS 输出 | 示例发布 `odom_global_001`、`imu_global_001` 等状态，并接收 `motion_001` 视觉位姿 | 读取状态/回传视觉 |
| 光流/激光测距 | 官方飞行前检查要求清晰地面纹理和良好光照；激光测距参与起飞/定高条件 | 室内稳定飞行辅助 |
| UM982 FGNSS | 可选高精度 GNSS 模组；室外需关注 GNSS 状态，室内不作为主定位源 | 室外参考/对照 |
| 视觉套件 | 官方产品线提供 Jetson Orin + D435/RGB、树莓派 Zero/5 等扩展路线 | 仅在实际购置/接口确认后接入 |

## 主链路

```text
FanciSwarm UWB ×4 / 光流 / 激光测距
        ↓
Mcontroller V7 内置位置估计与飞控闭环
        ↓ fcu_core（USB 或 TCP）
ROS1 Noetic：读取状态、ROS1 tf、硬件录包
        ↓ ros1_bridge / 轻量自定义 bridge
ROS2 Humble：视觉、tf2、融合、评估、RViz2、ros2 bag
        ↓ 视觉位姿映射为 ROS1 motion_001（按实验需要）
Mcontroller 位置环
```

ROS1 与 ROS2 不在同一系统环境中强行混装。推荐 Ubuntu 20.04 主机/容器运行 `fcu_core`，Ubuntu 22.04 主机运行 ROS2；桥接方式和部署位置在首次联调记录中确定。

## 不再假设

- 不使用 Nooploop 驱动；硬件不是 Nooploop。
- 不假设能读取原始 UWB 四基站测距；先向厂商确认。没有原始测距就不做 Chan/Taylor/TDOA 的伪比较。
- 不假设 800 万像素机载相机能被 ROS 直接访问；先确认原始帧、帧率、时间戳、内参和快门类型。
- 不把 GNSS 作为沙盘室内主定位源。
- 不在第一版替换 Mcontroller 的实时 EKF/位置控制闭环。

## 首次联调验收

1. 四基站和无人机标签在 App 中正常工作，能看到稳定位置。
2. `fcu_core` 能通过 USB 或 TCP 建立连接并发布 `odom_global_001`。
3. SD 日志能完整记录一次解锁到锁定周期。
4. ROS 端记录飞控时间、ROS 接收时间和视觉采样时间。
5. 视觉位姿先离线验证，再决定是否通过 `motion_001` 回传。

## 官方资料

- [Mcontroller V7 支持与开发文档](https://fancinnov.com/support-Mcontroller-v7)
- [FanciSwarm 产品与 UWB 说明](https://fancinnov.com/FanciSwarm)
- [FanciSwarm 支持与 ROS 工程说明](https://fancinnov.com/support-FanciSwarm)
- [UM982 高精度 GNSS](https://fancinnov.com/Fancinnov-FGNSS)
- [幻思 fcu_core](https://github.com/fancinnov/fcu_core)
