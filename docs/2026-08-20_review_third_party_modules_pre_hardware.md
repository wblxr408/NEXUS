# 硬件到货前第三方模块审阅与启用门槛

- 日期：2026-08-20
- 类型：review
- 状态：pre_hardware_ready
- 关联：[第三方模块清单](../code/THIRD_PARTY_MODULES.md)

以下模块已经以 Git submodule 固定版本下载。审阅只覆盖目录、职责、许可证线索与项目边界；没有构建或运行它们，也没有修改上游源码。

| 模块 | 项目用途 | 到货前可做 | 启用门槛 | 禁止项 |
|---|---|---|---|---|
| `fcu_core` | ROS1 官方硬件兼容 | 阅读 launch、topic、依赖，准备 Noetic 主机 | 实机确认 USB/TCP、topic、frame、时间、单位 | 修改官方代码或把平台 odometry 当目标 |
| `apriltag_ros` | 单相机标签检测候选 | 阅读 `cfg/`、launch 与输入 topic | 真正的 Image、CameraInfo、frame、标签尺寸/家族 | 在上游中写死项目目标/外参 |
| `rosbridge_suite` | Dashboard 数据出口 | 阅读 ROS2 分支和 topic 白名单方案 | 使用经 Humble 验证的系统包或另行记录的兼容提交、网络策略、允许暴露的话题 | 在网页端做坐标/融合/指标计算 |
| `caliscope` | 条件性多相机标定 | 熟悉标定工作流和输出文件 | 至少两路原始帧、时间戳、内参、固定相机 | 未满足同步条件时做三角测量 |
| UWB 三个参考仓库 | 仅原始测距离线研究 | 阅读数据格式和算法输入 | 厂商明确开放逐基站测距，字段/单位/时间已确认 | 接管飞控或伪造原始测距 |

## 审阅命令

克隆后先恢复固定版本：

```bash
git submodule update --init --recursive
git submodule status --recursive
```

阅读上游时只在各 submodule 内使用只读命令，例如 `git log -1`、`git status --short`、`rg` 和 README；若计划更新提交、改上游或接入构建，先新增 ADR/适配记录并进行对应构建测试。

## 当前结论

可以开始学习接口和准备启动参数；不能把任一模块标记为“硬件已兼容”或“精度已验证”。`fcu_core` 与 `apriltag_ros` 是硬件到手后的最短主链路；`caliscope`、UWB 原始测距与 LNN 都是条件性后续项。
