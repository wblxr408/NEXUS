# 当前网页入口（2026-09-07）

已适配双对象定位输入输出，支持“定格 → 框选 → 参考点 → 命名 → 后端确认注册”。操作和验证限制见[当前使用说明](../../../../../docs/2026-09-07_guide_web_target_registration.md)。

默认接入当前 ROS2 定位与输入状态；下方 CARLA 说明作为旧模式参考，仅在 URL 使用 legacy=carla 时连接。静态 HTTP 服务本身不启动相机或算法。

---

# Dashboard 前端骨架

此目录预留给浏览器端的 Dashboard。页面部署在 ROS2 Ubuntu 主机，由 `nexus_viz_dashboard` 的后续网关提供 HTTP/WebSocket；浏览器可在同网段的 Ubuntu 或 Windows 上访问。

当前页面采用“双窗口”方案：Windows 端 CARLA 负责 RRD 的真实 GPU 渲染，Web Dashboard 左侧消费 WSL bridge 推送的 CARLA、算法和日志数据，右侧保留相机证据、延迟和实验指标。

```text
web/
├── public/                 静态入口与已核准的静态资源
│   └── assets/
│       └── sandbox/        由 simulation 生成的 Web 静态资源（JSON/OBJ/MTL）
├── public/vendor/three/     本地 Three.js + OrbitControls（不依赖外网 CDN）
├── src/
│   ├── app/                应用装配、路由和页面状态
│   ├── components/         可复用展示组件
│   ├── features/           主视图、诊断、回放、实验对比等功能模块
│   ├── services/           HTTP/WebSocket 客户端与运行指标读取
│   ├── styles/             全局样式与状态语义
│   └── types/              浏览器端消息与界面状态类型
└── test/                   前端单元、集成和回放验收测试
```

## CARLA 数据流接入

当前主界面不再把参数化沙盘作为左侧主视觉。Windows CARLA 窗口负责 RRD 的真实渲染，Web 左侧通过 WSL bridge 消费 CARLA 状态、位姿、算法输出和滚动日志。

默认状态流地址：`ws://<host>:8765/ws/carla_status`；场景流地址：`ws://<host>:8765/ws`。浏览器也保留 rosbridge 的 `ws://<host>:9090` 连接，用于相机证据和 ROS2 观测。

状态流可发送 `carla_connected`、`map`、`tick_hz`、`sync_mode`、`actor_count`、`ego_pose`、`sensors`、`algorithms`、`target` 和 `log` 字段；前端只展示这些结构化结果，不在浏览器内做坐标转换或融合。

## 沙盘资源同步（历史资源）

源配置和生成器：

```text
simulation/sandbox_scene.yaml
simulation/generate_sandbox.py
```

生成后将以下派生资源同步到 `public/assets/sandbox/`：

```text
sandbox_scene.json   # 页面读取的场景坐标、物体、尺寸和置信度
sandbox.obj          # 后续 Three.js/OBJLoader 或其他 3D 引擎使用
sandbox.mtl          # OBJ 材质
```

当前 `scene_renderer.js` 使用本地 Three.js WebGL 渲染器读取 `assets/sandbox/sandbox_scene.json`，提供真实深度遮挡、透视、光照/阴影和 OrbitControls 鼠标旋转/滚轮缩放；它会绘制外围环形双车道、四角十字路口与人行横道、中央无虚线单车道、建筑、储罐和树阵。道路标识与 OBJ、SDF 共用生成器产出的 `road_marking` 对象，避免三种视图布局漂移。

地图右上角的固定信息窗会随鼠标悬停更新，显示物体语义、`map` 世界坐标（`x/y/z` 均保留到厘米）和坐标来源说明。空白状态也明确显示沙盘范围 `x=0.00–4.00 m`、`y=0.00–4.70 m` 与 `+z` 向上。页面另外绘制四角 UWB 基站（0.70 m 为尚待现场标定的标称高度）以及含 IMX219/Gazebo 相机、机载 UWB 标签的模拟无人机；UWB 安装坐标和无人机静态位置均为可视化示意，不能当作已测量精度证据。

重新生成模型后执行：

```bash
python3 simulation/generate_sandbox.py
cp simulation/sandbox_scene.json simulation/sandbox.obj simulation/sandbox.mtl \
  code/ros2_ws/src/nexus_viz_dashboard/web/public/assets/sandbox/
```

## 本地查看页面

不要直接双击 `public/index.html`。浏览器会阻止 `file://` 页面加载 ES Module 和场景 JSON。启动一个本地静态服务器：

```bash
cd code/ros2_ws/src/nexus_viz_dashboard/web
python3 -m http.server 8765
```

然后打开：

```text
http://127.0.0.1:8765/public/index.html
```

停止服务器按 Ctrl+C。页面会连接 ws://<host>:9090 rosbridge、ws://<host>:8765/ws/carla_status CARLA 状态流和 ws://<host>:8765/ws 场景流。可用 `?rosbridge=...&carla_ws=...&carla_scene_ws=...` 覆盖地址。未启动任一路 WebSocket 时会显示断开状态，不会伪造定位数据。

Gazebo 通过 `nexus_bringup` 的 `gazebo_sandbox.launch.py` 启动，完整命令与 IMX219 仿真范围见
[`nexus_bringup/README.md`](../../nexus_bringup/README.md)。前端只显示 Gazebo 的图像和位置，不在浏览器中做
坐标转换或融合。
