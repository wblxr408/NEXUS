# 答辩演示录屏清单（Demo Recording Checklist）

- 来源：`docs/优化方案.md` §11.4（固定流程）、§11.5（三个硬缺口）、§11.6（安全提醒）；
  启动命令照抄 `simulation/2026-08-28_runbook_carla_dashboard_dual_window.md` §9 及其 §1–§8 展开。
- 状态：本文件是可执行清单，**尚未执行任何录制**。所有需要实测填写的位置一律写 `[待录制]`，
  不得先填猜测的 bag 路径、sha256 或录屏日期。
- 口径：录屏本身只是画面证据；任何数字必须能追到 `experiments/runs/` 的运行号（ADR-002）。

## 0 开录前置（必须先过 §6 现场检查清单）

`§6` 三项前置只要有一项没过，就不要开录：录出来的画面会是空 RViz 和空面板，
反而成为反证。

## 1 起服务顺序

按 runbook §9 的顺序执行，每步确认「验证点」后再进入下一步。

| 步骤 | 位置 | 命令 | 验证点 |
|---|---|---|---|
| 1 | Windows PowerShell | `cd D:\CARLA_0.9.16` + `.\CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low` | CARLA 窗口出图 |
| 2 | WSL | runbook §2 的端口探测脚本 | `2000/2001/2002 OPEN` |
| 3 | WSL | `simulation/carla/.venv-carla/bin/python simulation/carla_smoke_test.py --host 127.0.0.1 --port 2000` | 输出 `SMOKE_TEST_OK`，生成 `/tmp/carla_smoke_frame.png` |
| 4 | WSL | 见下方 §1.1 | bridge 日志出现 `CARLA status WebSocket listening on ws://<listen>:8766/ws/carla_status` |
| 5 | WSL | 见下方 §1.2 | rosbridge 监听 9090 |
| 6 | WSL | 见下方 §1.3 | `http://127.0.0.1:8765/public/index.html` 可打开 |
| 7 | WSL | 见下方 §1.4（runbook §9 未列，来自方案 §11.4 第 1 步） | `/nexus/target/pose` 有 `validity=1` 消息 |
| 8 | 浏览器 | 打开 §1.3 的带参 URL | 左侧 CARLA 卡片 CONNECTED，中间相机有帧 |

### 1.1 CARLA 状态桥（:8766）

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
./.venv-carla/bin/python carla_status_bridge.py \
  --host 127.0.0.1 --port 2000 --map sandbox-v29 \
  --web-port 8766 --spawn-demo-ego
```

答辩现场走共享网络时追加 `--listen 127.0.0.1`（默认是 `0.0.0.0`，见 §7）。

### 1.2 rosbridge（:9090）

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

### 1.3 Dashboard 静态页（:8765）

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws/src/nexus_viz_dashboard/web
python3 -m http.server 8765
```

```text
http://127.0.0.1:8765/public/index.html?carla_ws=ws://127.0.0.1:8766/ws/carla_status&rosbridge=ws://127.0.0.1:9090
```

`carla_status_client.js` 现在的默认状态流地址是 `:8765/ws/carla_status`（对应
`code/carla_bridge/server.py` 那种单端口网关拓扑）。本清单按 runbook 的
「静态页 8765 + 状态桥 8766」跑，所以 URL 里的 `?carla_ws=` 参数必须显式带上，不能省。

### 1.4 定位主链路 + RViz

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nexus_bringup target_localization.launch.py \
  hardware_required:=false \
  use_rviz:=true \
  image_topic:=/nexus/camera/imx219/image_raw \
  camera_info_topic:=/nexus/camera/imx219/camera_info \
  marker_size_m:=<实测标签边长_m> \
  position_variance_m2:=<实测像素噪声折算_m2> \
  max_pair_delta_ms:=50.0
```

- `transform_file` 默认指向 `nexus_bringup/config/pre_hardware_transforms.yaml`（v1.1，仅仿真数值）。
- `marker_size_m` / `position_variance_m2` 默认都是 `0.0`；保持 0 时视觉链路不会给出可用位姿，
  必须按当次使用的标签与相机填实测值，不要用默认值录屏。
- RViz 用 `nexus_viz_dashboard/rviz/nexus_target.rviz`（Fixed Frame=map），
  Marker 话题 `/nexus/viz/target_observation` 与 `/nexus/viz/target_pose`；
  新增的 `nexus_target_covariance` / `nexus_view_rays` / `nexus_support_plane`
  三个 namespace 复用 `/nexus/viz/target_pose`，无需改 RViz 配置。

## 2 `ros2 bag record` 与校验登记

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nexus_bringup record_topics.launch.py output:=nexus_demo_<YYYYMMDD>_<chain>
```

录制的 7 个话题（`record_topics.launch.py:12-17` 实测清单）：

1. `/nexus/fcu/odom`
2. `/nexus/fcu/imu`
3. `/nexus/vision/target_observation`
4. `/nexus/vision/map_target_observation`
5. `/nexus/uwb/raw_target_observation`
6. `/nexus/uwb/target_observation`
7. `/nexus/target/pose`

注意（核对结果，非方案原文）：

- 这 7 个话题里 `/nexus/fcu/odom` 与 `/nexus/fcu/imu` 在纯仿真下**没有发布者**
  （Gazebo 平台里程计发在 `/nexus/gazebo/uav/odom`，`/nexus/fcu/*` 是 ROS1 `fcu_core` 桥接名）。
  只跑仿真时这两个话题会录成空 topic，回放时不要当成平台证据。
- 相机话题和 `/nexus/viz/*` Marker 都不在录制清单里；画面证据只能靠屏幕录制（§4）。

每条 bag 录完后立刻登记：

```bash
cd <bag 输出目录的父目录>
sha256sum nexus_demo_<YYYYMMDD>_<chain>/*.db3 | tee nexus_demo_<YYYYMMDD>_<chain>.sha256
ros2 bag info nexus_demo_<YYYYMMDD>_<chain>
```

| 链路 | bag 路径（仓库外） | `.db3` sha256 | 时长 | 运行号 |
|---|---|---|---|---|
| A `chain_a_precision` | `[待录制]` | `[待录制]` | `[待录制]` | `[待录制]` |
| B `chain_b_robust` | `[待录制]` | `[待录制]` | `[待录制]` | `[待录制]` |
| C `chain_c_adaptive` | `[待录制]` | `[待录制]` | `[待录制]` | `[待录制]` |

bag 本体不入 Git（AGENTS.md），只在本表登记外部位置与校验值。

## 3 三链路切换，每档 60 s

```bash
ros2 topic pub --once /nexus/algorithm_route std_msgs/msg/String "{data: chain_a_precision}"
# 记录切换时刻，连续录 60 s；随后依次换 chain_b_robust、chain_c_adaptive
```

**缺口（必须先确认）**：全仓库 grep `/nexus/algorithm_route` 只命中 `docs/优化方案.md`
本身，没有任何节点发布或订阅该话题，`simulation/algorithm_routes.yaml` 也只登记候选名
（`default.uwb`/`default.vision` 均为 `null`）。所以在求解侧实现订阅之前，上面的
`ros2 topic pub` 只是往一个没有消费者的话题发字符串，**不会真的切链路**。
录屏前必须确认：

- [ ] 已有节点订阅 `/nexus/algorithm_route` 并按 `chain_a/b/c` 切换求解配置；
- [ ] 每档切换在 bag 与屏幕录制里都有可识别时刻（口播或界面上的 CHAIN 字段变化）；
- [ ] 前端 `当前目标位置` 面板的 `CHAIN` 字段有数据来源
      （目前该字段只能由 `NexusAPI.updateSolverProvenance({chain})` 注入，
      `TargetObservation.msg` 里没有 chain 字段）。

| 链路 | 切换时刻（本地时间） | 录制时长 | 备注 |
|---|---|---|---|
| A | `[待录制]` | 60 s | `[待录制]` |
| B | `[待录制]` | 60 s | `[待录制]` |
| C | `[待录制]` | 60 s | `[待录制]` |

## 4 屏幕录制（RViz + 浏览器双窗）

- [ ] 左半屏 RViz（Fixed Frame=map，Grid/Odometry/两个 Marker/TF 都打开），
      右半屏浏览器 Dashboard；两窗同时可见，分辨率与缩放固定后再开录。
- [ ] 开录前先让画面出现：支撑面立方体（`nexus_support_plane`，z=0.050 m，4.000×4.700 m）、
      视线束（`nexus_view_rays`）、协方差椭球（`nexus_target_covariance`，1σ 半轴，品红半透明）。
      椭球不出现说明协方差含 NaN 或非正定，dashboard 会打 warning 而不画假球——
      这时要修求解侧，不要接着录。σ 只有几厘米时椭球会被 `nexus_target` 的 0.15 m 实心球盖住，
      先在 RViz 的 Marker Namespaces 里关掉 `nexus_target` 再看。
- [ ] 录制内容按方案 §11.4 的演示脚本：初始化 → 目标出现 → 多视角运动 → 三链路切换 → 指标回放。
- [ ] 每档链路 60 s，中间不剪辑；口播说明当前链路名。
- [ ] 视频文件不入 Git，只在下表登记外部路径与 sha256。

| 项 | 值 |
|---|---|
| 录屏日期 | `[待录制]` |
| 录制工具 / 分辨率 / 帧率 | `[待录制]` |
| 视频外部路径 | `[待录制]` |
| 视频 sha256 | `[待录制]` |
| 对应 bag | `[待录制]` |
| 对应运行号 | `[待录制]` |

相机预览帧率说明：前端二次门与 rosbridge 节流已统一到 150 ms（≈6.7 FPS，IA §3.3 的
640×480 @5–10 FPS 区间内）。录屏时若相机窗口明显低于该帧率，先查 rosbridge 带宽与
Gazebo `update_rate`（world 里是 21 Hz），不要改门限凑画面。

## 5 回填 slide 占位

`defense/slides/2026-08-19_plan_defense_slide_content.md:242`（第 24 页）四个占位：

| 占位 | 回填值 |
|---|---|
| `[演示视频链接/文件]` | `[待录制]` |
| `[rosbag/运行编号]` | `[待录制]` |
| `[RViz 配置版本]` | `[待录制]`（`nexus_viz_dashboard/rviz/nexus_target.rviz` 的 git commit） |
| `[录屏日期]` | `[待录制]` |

同页还有 `[现场演示时长]`，一并回填。回填后本清单第 §2/§4 两表必须能对上同一个运行号。

## 6 现场检查清单（方案 §11.5 三个硬缺口，开录前逐项确认）

### 6.1 TF 表 —— 部分解决，仍有阻塞项

- [x] `nexus_bringup/config/pre_hardware_transforms.yaml` 已从 `transforms: []` 改为
      `calibration_version: "1.1"`，`data_status: simulation_only_pending_hardware_calibration`，
      填入 `base_link -> nexus_uav_camera_optical_frame`（平移 `[0, 0, -0.06]` m，
      四元数 xyzw `[0.70710678, -0.70710678, 0, 0]`，取自
      `simulation/nexus_sandbox_imx219.world` 的 imx219 sensor pose）。
      已用 `coord_transform_node._publish_from_file()` 实跑核对：解析通过并广播 1 条静态变换。
- [ ] **`map -> base_link` 仍然没有任何发布者**：它是动态平台位姿，不能进静态表；
      而 world 里 `libgazebo_ros_planar_move.so` 设了 `publish_odom_tf=false`，
      `/nexus/fcu/odom` 在纯仿真下也没有发布者。
      ⇒ `coord_transform_node` 对 camera 帧观测仍会回 `transform_unavailable`，
      `/nexus/target/pose` 出不来，RViz 与前端仍会是空的。
      开录前必须先让某一侧广播 `map->base_link`（启用 `publish_odom_tf`，
      或加一个 `/nexus/gazebo/uav/odom` → tf 的中继），并用
      `ros2 run tf2_ros tf2_echo map base_link` 确认。
- [ ] 硬件到货后本表必须整表替换为实测标定值（表里已写明数值仅来自仿真设计，不是实测）。

### 6.2 URDF / SDF / static_transform_publisher —— 部分解决

- [x] `base_link -> camera` 已经进入 TF 树：由 `coord_transform_node` 的
      `StaticTransformBroadcaster` 从上表广播（不再只存在于数据集 YAML 里）。
- [ ] 仍然没有飞机模型：全仓库无 `*.urdf` / `*.xacro`；`*.sdf` 只有
      `simulation/sandbox.sdf`（沙盘场景）和 `web/public/assets/sandbox/sandbox.sdf` 副本，
      没有 `robot_state_publisher`。⇒ RViz 里只有坐标轴，没有机体模型；
      除 `base_link` 与相机光学系外的机体 frame 都不存在。录屏画面按这个现状规划，
      不要在 slide 里声称有整机模型。

### 6.3 `gazebo_sandbox.launch.py` 的世界文件 —— 已解决（本次核对）

- [x] `gazebo_sandbox.launch.py:11-12` 指向
      `share/nexus_bringup/simulation/nexus_sandbox_imx219.world`；
      `nexus_bringup/CMakeLists.txt:13-15` 把仓库根 `simulation/` 下的 `*.world`/`*.yaml`
      安装到该目录，而 `simulation/nexus_sandbox_imx219.world` 现在存在（214792 B）。
      本次 `colcon build` 后已核对 `install/nexus_bringup/share/nexus_bringup/simulation/
      nexus_sandbox_imx219.world` 确实存在 ⇒ 方案 §11.5 记的「世界文件不存在」已过期。
- [ ] 前提：必须先 `colcon build`（含 install）再 launch；直接在未构建的源码树上跑仍会找不到。
- [ ] `gzserver` / `libgazebo_ros_init.so` / `libgazebo_ros_camera.so` /
      `libgazebo_ros_planar_move.so` 在本机可用性未核对，开录前先跑一次
      `ros2 launch nexus_bringup gazebo_sandbox.launch.py use_gui:=false` 确认不报缺插件。

## 7 安全提醒（方案 §11.6，本次逐条核对；只记录，不改代码）

核对对象：`code/carla_bridge/server.py`（193 行，目录当前是 Git **未跟踪**状态 `?? code/carla_bridge/`）。

已确认与方案一致的部分：

- `server.py:20-25` `CORSMiddleware` 配置为 `allow_origins=["*"]`、`allow_methods=["*"]`、
  `allow_headers=["*"]`。
- 四个 WebSocket 端点全部只 `await websocket.accept()`，没有 token、来源或身份校验：
  `/ws/carla`（:118）、`/ws`（:141）、`/ws/carla_status`（:163）、`/ws/nexus`（:180）。
- `/ws/nexus`（:186-191）接受 `type: "target"` 的 JSON 并广播给所有 `/ws` 客户端
  ⇒ 任何能连上的人都能注入目标。
- 比方案写得更严重的一点：`/ws`（:152-156）把客户端发来的 `type: "command"` 原样转发给
  `/ws/carla` 上游 ⇒ 不只是读遥测和注入目标，还能借这条路向 CARLA 侧发指令。

与方案 §11.6 表述不符的部分（按核对结果修正）：

- `server.py` **自身没有 CLI**，没有 `--host` 参数；`code/carla_bridge/` 下也没有 README。
  所以「README 指导以 `--host 0.0.0.0` 绑定」在 `code/carla_bridge/` 里找不到出处
  （该目录只有 `server.py`、`requirements.txt`、`.venv/`）。
- 仓库里真正默认对外监听的是另一支桥：`simulation/carla/carla_status_bridge.py:303`
  的 `--listen` 默认值就是 `0.0.0.0`，且 `simulation/2026-08-28_carla_windows_deploy_full.md:160`
  的示例命令显式写了 `--listen 0.0.0.0`。

答辩现场（共享网络）开录前的处置项：

- [ ] `server.py` 若要起，用 `uvicorn ... --host 127.0.0.1` 绑回本机，或放到只监听本机的反向代理后；
      需要跨机时加共享 token 并收紧 `allow_origins`。
- [ ] `carla_status_bridge.py` 启动参数加 `--listen 127.0.0.1`（见 §1.1）。
- [ ] 用 `ss -ltnp | grep -E '8765|8766|9090'` 确认三个端口都绑在 `127.0.0.1`，不是 `0.0.0.0`。
- [ ] 录屏画面里不出现内网 IP、主机名、账号或密钥。

以上属于本方案范围外的独立问题，本清单只登记核对结果，不在此次改动中修改 `server.py`。

