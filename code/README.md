# 代码与 ROS2 工作空间

## 分类

- `ros2_ws/src/`：可构建的 ROS2 包，包名统一 `nexus_<layer>`。
- `analysis/`：离线算法、标定、GDOP 和评估脚本；不得偷偷读取未登记的外部真值。
- `configs/`：版本化 YAML/JSON 参数；设备密钥和本机路径放在未提交的 `.local` 文件。
- `tools/`：数据转换、录包回放和开发辅助命令。

## 包边界

`nexus_uwb_driver`、`nexus_vision_localization`、`nexus_coord_transform`、`nexus_fusion_localization`、`nexus_viz_dashboard`、`nexus_bringup` 的职责与 `docs/architecture/README.md` 一致。

先写包 README 和接口，再实现节点；每个包应有 `config/`、`launch/`、`src/`、`test/`，参数不得散落在源码中。
