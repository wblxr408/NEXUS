# 代码与 ROS 工作空间

实现对象以 `../docs/decisions/2026-08-18_ADR-002_target-localization-scope.md` 为准：无人机是平台，ROS2 主输出是无人机之外目标的 `/nexus/target/pose`。无人机位姿、目标相对观测和目标世界位姿不得复用同一语义。

## 分类

- `ros1_ws/src/`：Ubuntu 20.04 + ROS1 Noetic/catkin 的幻思硬件兼容层。
- `ros2_ws/src/`：Ubuntu 22.04 + ROS2 Humble 的项目算法、坐标、融合和展示包。
- `analysis/`：离线算法、标定、GDOP 和评估脚本；不得偷偷读取未登记的外部真值。
- `configs/`：版本化 YAML/JSON 参数；设备密钥和本机路径放在未提交的 `.local` 文件。
- `tools/`：数据转换、录包回放和开发辅助命令。

## 包边界

`nexus_fcu_bridge` 位于 `ros1_ws`；其余自研 ROS 包位于 `ros2_ws`。职责与 `docs/architecture/README.md` 一致。

先复用并验证幻思官方 `fcu_core`，再写包 README 和接口；每个自有包应有 `config/`、`launch/`、`src/`、`test/`，参数不得散落在源码中。ROS1 侧使用 `roslaunch`、`rosbag`、`catkin_make`；ROS2 侧使用 `ros2 launch`、`ros2 bag`、`colcon build`。两侧只通过明确的 bridge 接口通信，不要求每个包同时支持两套 ROS。

## 第三方源码

开源模块以 Git submodule 固定版本，不复制进项目自有包。首次取得本仓库后执行：

```bash
git submodule update --init --recursive
```

模块位置、固定提交、许可证和适配边界见 [`THIRD_PARTY_MODULES.md`](THIRD_PARTY_MODULES.md)。
