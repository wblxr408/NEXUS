# ADR-014：本机 DDS 传输与旧基线测试模式

- 日期：2026-09-03。
- 授权：用户要求继续完成录包、旧链路回归、Web/RViz 实际联调。
- 本决定不改变静态目标、条件协方差或只读飞行边界，仍遵循 ADR-013。

## 决定

1. `dual_localization.launch.py` 新增 `dds_transport`，默认 `inherit`；显式 `LARGE_DATA` 时，仅为该启动产生的 ROS 节点及录包进程传入 `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`、`ROS_LOCALHOST_ONLY=0`。上游 ROS 数据源必须采用兼容的同一传输配置与 domain。本机经验证采用此模式；不改系统级环境或网络配置。
2. 该选择依据当前 WSL / ROS2 Humble / Fast DDS 2.6.12 的实测：默认传输下晚加入录包进程发现/收取失败，`LARGE_DATA` 下同一严格回环测试成功。其机制是保留 UDP 发现，同时允许 TCP/共享内存承载数据与参与者存活通信，见 [Fast DDS 官方传输说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/env_vars/env_vars.html)。尚未把默认传输异常定位到内核、网络驱动或 DDS 内部的具体缺陷；不声称已修复这些底层组件。
3. 录包 `run.json` 增加 `runtime_environment`，记录 RMW 环境选择、domain、localhost 标志和 Fast DDS 传输变量，包含 launch 的子进程覆盖值。未显式设置的 RMW 为 null，实际默认实现由验证记录补充。
4. 旧 `target_localization.launch.py` 显式暴露既有 `fusion_mode`，默认仍为 `robust`。验证异步配对选择最新源的旧测试设置 `information_baseline`，与其对应的 `fuse_observations` 单元合同一致。保留原输出数值和时间断言，不降低鲁棒滤波创新阈值。
5. 可选展示夹具使用独立 domain 231；录包回归使用 230。Web/RViz 探针只运行显式合成输入，不能作为实机精度证据。可选 RViz 渲染探针调用真实 RViz frame、插件及 `captureScreenShot`，不重新绘制替代图。

## 未采用的方案

- 不通过延长超时、跳过断言或绕开真实 rosbag 进程制造通过结果。
- 不将旧测试的“最新源选择”强加到鲁棒滤波器，也不为了 1 m 不一致观测放宽离群门限。
- 不升级/降级 DDS、不改防火墙，不将同进程 Python 消息接收代替跨进程展示验证。

最终证据与复现命令见[本机联调验证记录](../2026-09-03_record_local_integration_validation.md)。
