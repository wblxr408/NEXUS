# fcu_core 外部依赖登记

来源：幻思官方 [fcu_core](https://github.com/fancinnov/fcu_core)。

已下载的原样工作树：[`fcu_core/`](fcu_core/)，上游提交为
`14e228c63cfe2bbdecebda8c4f1fc259f295e0aa`（2025-07-07）。该上游目录未见
许可证文件，任何再分发或交付前须向维护方确认许可。未修改上游源码。

运行环境：Ubuntu 20.04 + ROS Noetic + catkin。实际源码、版本和许可证在首次部署时登记；本目录不复制厂商源码，也不在 ROS2 工作空间重新实现其协议。

将来适配只允许写在同级的项目自有 `nexus_fcu_bridge`：核实实际发布话题、`frame_id`、单位、采样时间和
`/motion_001` 语义后进行显式消息映射。不得改动官方 `fcu_core`，不得把其平台/UWB状态直接命名或转换成赛题
目标位姿。
