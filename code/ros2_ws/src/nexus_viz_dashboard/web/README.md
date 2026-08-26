# Dashboard 前端骨架

此目录预留给浏览器端的 Dashboard。页面部署在 ROS2 Ubuntu 主机，由 `nexus_viz_dashboard` 的后续网关提供 HTTP/WebSocket；浏览器可在同网段的 Ubuntu 或 Windows 上访问。

当前页面骨架已可读取参数化沙盘资源，展示其相对/绝对坐标布局；ROS2 实时消息仍由后续网关适配。仿真源文件不放在此目录，唯一源数据位于仓库根目录 `simulation/`。

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

## 沙盘资源同步

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

当前版本不需要编译，也不需要 `colcon build`。停止服务器按 `Ctrl+C`。页面的 ROS2/实时数据网关尚未接入时，页面会显示 `NO INPUT`，但沙盘布局仍应显示在主地图区域。
