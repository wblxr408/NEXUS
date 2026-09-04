# 本机录包、旧链路与双对象展示闭环验证

日期：2026-09-03。状态：**VALIDATED（本机合成软件联调）**。

这是六项修正之后的补验记录，覆盖用户再次指定的三项工作。它更新此前录包/旧回归失败与 RViz/Web 整链未验证的状态，不改写历史失败，也不表示最初总体方案的全部算法或实机精度已完成。

## 结果

| 工作 | 最终证据 | 状态 |
|---|---|---|
| 录包跨进程发现与落盘回环 | 独立 `ros2 bag record` 发现并录制 11 个话题、25 条消息；`rosbag2_py` 逐条反序列化，核对图像、CameraInfo、IMU、UWB、平台、原始/融合目标和质量的采样时间；当前目标与历史输出均存在，目标速度仍未知 | VALIDATED |
| 旧异步 UWB 降级回归 | 测试选择它原本验证的 `information_baseline`，保留“最新 UWB、x=2.0”等断言；旧 launch 的 6 个测试通过，默认 robust 不变 | VALIDATED |
| ROS2 → 融合 → 展示 → rosbridge → Edge | 独立进程和真实 DDS/WebSocket，浏览器无 NexusAPI 注入；末轮收到 42 个双对象快照，验证无输入、确认目标、遮挡历史、原采样时间不刷新、重新识别、离群点拒绝、平台过期和桥接断开；相机预览确实解码合成 JPEG | VALIDATED |
| RViz 实际渲染 | 正常启动 `rviz2`；另用真实 RViz `VisualizationFrame` 加载同一配置与 MarkerArray 插件，从渲染窗口直接取图。当前目标、历史点、UAV、map 网格均可见；两次取图的 Global/Grid/MarkerArray 状态全部 OK | VALIDATED |

修改决策见 [ADR-014](decisions/2026-09-03_ADR-014_local_dds_transport_and_baseline_tests.md)。范围仅为本轮三项联调；没有新增或调用飞控速度、高度、航点、解锁或模式接口。

## 根因与修正

**录包传输**：相同测试在默认传输下失败，启用 Fast DDS `LARGE_DATA` 后通过。新增 `dds_transport:=LARGE_DATA` 启动选项及录包运行环境溯源，测试使用同一已验证配置。`inherit` 默认保留原环境，当前 WSL 上应显式采用已验证配置；上游发布端也必须一致。底层默认 UDP 路径的具体缺陷尚未归因，没有声称修复操作系统或中间件源码。

**旧测试模式**：测试针对信息配对基线，却随默认值进入鲁棒时序滤波器；后者会拒绝两条相差 1 m 的不一致观测，且来源字段语义不同。修正是显式选择基线，未删除输出断言，未放宽鲁棒拒绝阈值。算法包的既有鲁棒测试仍通过。

**展示测试隔离**：一次回归与展示夹具同用 domain 230，录包混入额外目标，历史数组断言失败。未放宽测试；停止夹具后通过，并将可选展示夹具默认 domain 设置为 231。随后再次并行运行展示验证和 bringup 回归，两者均通过。

**RViz 截图**：WSLg 外部窗口抓取返回黑图，不能用来声称渲染通过。改为真实 RViz 渲染接口取图；首次发现夹具缺少 map→base_link TF，补齐由同一合成平台位姿产生的 TF 后，Global Status 从 Warn 变为 OK。生产平台节点原已发布该 TF。

## 代码位置

- `nexus_bringup/launch/dual_localization.launch.py`：对子进程应用显式 DDS 传输配置；与上游运行环境要求一致。
- `nexus_bringup/src/nexus_bringup/dual_recording.py`：保存实际运行环境到记录元数据。
- `nexus_bringup/launch/target_localization.launch.py`、`test/test_realtime_pipeline_launch.py`：旧融合模式参数与基线测试配置。
- `nexus_bringup/test/test_dual_launch.py`、`CMakeLists.txt`：检查环境继承/覆盖与元数据，录包采用已验证传输。
- `nexus_bringup/test/run_dual_display_qa.py`：显式合成 DDS 夹具，独立 domain，只发布观测/TF/相机，子进程退出时清理。
- `nexus_viz_dashboard/web/test/live_localization.cjs`：实际 Edge 页面与 WebSocket 验证，不向应用注入目标坐标。
- `nexus_viz_dashboard/test/rviz_render_probe/`：可选 C++ 渲染取图工具，使用现有 RViz/Qt，不新增运行库安装。

以上路径相对 `code/ros2_ws/src/`。历史六项改动、旧资料和 UWB 第三方子模块原有改动均保留；本次未提交 Git。

## 验证环境和命令

Ubuntu 22.04 WSL、Python 3.10.12、ROS2 Humble、默认 `rmw_fastrtps_cpp`、Fast DDS 2.6.12、RViz 11.2.28、OpenGL 4.5 软件渲染、Windows Edge、已有 Node/Playwright。未安装或升级依赖。

```bash
cd code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nexus_bringup --symlink-install
source install/setup.bash
OPENBLAS_NUM_THREADS=1 colcon test --packages-select nexus_msgs nexus_fusion_localization nexus_vision_localization nexus_viz_dashboard nexus_bringup
colcon test-result --verbose
```

- 构建：7 个包通过。
- 最终 xunit：msgs 3、fusion 79、vision 67、viz 8、bringup 16，共 **173 个 Python/launch 用例，0 failures / 0 errors**；Web 模块 3 个 Node 测试另计，全部通过。CTest 包装项不重复计数。
- 全量运行中一次录包失败是上述 domain 串扰；隔离后 bringup 16 项重跑通过；修正夹具 domain 后，与真实浏览器验证并行再跑 bringup 16 项仍通过。
- 本轮改动 Python 文件通过 Flake8，沿用 E501/W503 忽略规则；`git diff --check` 通过。

当前 WSL 的定位/录包运行配置：

```bash
# 对已有上游 ROS 发布端使用相同环境；这里不启动或控制飞控。
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
# 再按六项清单中的真实标定/模型参数启动：
ros2 launch nexus_bringup dual_localization.launch.py --show-args
# 实际启动参数中加入 dds_transport:=LARGE_DATA
```

可选展示复验（输出目录必须全新）：

```bash
ROS_LOCALHOST_ONLY=0 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA LIBGL_ALWAYS_SOFTWARE=1 \
  python3 src/nexus_bringup/test/run_dual_display_qa.py /tmp/nexus_display_review --rviz
# 夹具默认使用 domain 231；页面 18766、rosbridge 19091。
```

在已有 Playwright 与 Edge 的 Node 环境中运行（WSL 目录传给 Windows Node 时使用相应 UNC 路径）：

```text
NEXUS_PLAYWRIGHT_MODULE=<已有 Playwright 模块目录>
node code/ros2_ws/src/nexus_viz_dashboard/web/test/live_localization.cjs <夹具输出目录>
```

渲染取图（浏览器测试完成后夹具仍运行；其 `phase.txt` 为 disconnect，保留目标历史）：

```bash
cmake -S src/nexus_viz_dashboard/test/rviz_render_probe -B /tmp/nexus_rviz_probe
cmake --build /tmp/nexus_rviz_probe -j2
export ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=0 FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA LIBGL_ALWAYS_SOFTWARE=1
/tmp/nexus_rviz_probe/rviz_render_probe src/nexus_viz_dashboard/rviz/nexus_dual_localization.rviz /tmp/nexus_display_review/rviz_history
# 将夹具 phase.txt 写成 observed 后，可再取当前点图；写成 stop 后结束夹具。
```

探针 PNG 来自 RViz 的渲染缓冲，JSON 来自真实显示插件属性/状态。查看 JSON 必须确认 Global Status、Grid、MarkerArray 为 OK 且有接收消息，不能仅以进程退出码判定显示正确。

## 证据与限制

本机证据目录：`C:/Users/wblxr/.codex/visualizations/2026/09/03/01a065c8-bb9e-7752-93d1-4ef1f2ba16d1/local_integration_v01/`。包含真实录包、浏览器截图/结果、RViz 渲染图/插件状态、DDS 接收 JSONL、测试 XML、运行环境及源码改动快照和 SHA256 清单。

证据是合成软件联调，不包含 IMX219 实拍、真实 UWB/IMU 标定、实飞、网络断续实测或厘米级精度评估。浏览器与 RViz 的数据只是测试夹具生成的静态目标和变速变高平台观测。全套验证进程结束后已清理；未扩大任务范围。
