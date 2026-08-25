# Dashboard 前端骨架

此目录预留给浏览器端的 Dashboard。页面部署在 ROS2 Ubuntu 主机，由 `nexus_viz_dashboard` 的后续网关提供 HTTP/WebSocket；浏览器可在同网段的 Ubuntu 或 Windows 上访问。

当前只建立目录边界，不包含前端框架、依赖清单、页面、模拟数据、网络地址或消息实现。具体技术栈和数据契约须在硬件接口、相机输入和网关方案确认后通过实验运行及接口记录固化。

```text
web/
├── public/                 静态入口与已核准的静态资源
│   └── assets/             坐标已配准的沙盘底图、图标等（不得放原始视频）
├── src/
│   ├── app/                应用装配、路由和页面状态
│   ├── components/         可复用展示组件
│   ├── features/           主视图、诊断、回放、实验对比等功能模块
│   ├── services/           HTTP/WebSocket 客户端与运行指标读取
│   ├── styles/             全局样式与状态语义
│   └── types/              浏览器端消息与界面状态类型
└── test/                   前端单元、集成和回放验收测试
```

实现前先阅读：

- `docs/architecture/2026-08-19_design_demo_dashboard_information_architecture.md`：页面信息架构、实时/回放状态和展示边界；同时参考 `docs/architecture/README.md` 与 `docs/2026-08-25_plan_current_execution_baseline.md` 的当前数据语义；
- `docs/architecture/interfaces.md`：ROS1/ROS2 话题语义；
- `code/tools/nexus_channel_contract.py`：传输 envelope 的最小字段和校验规则。

网页只能展示由 ROS2 网关验证过的消息。坐标转换、融合、指标计算和真值处理不应移到浏览器端；原始视频、rosbag 与未经登记的实验数据不放入本目录。
