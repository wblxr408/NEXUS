# 演示 Dashboard 信息架构设计

- 日期：2026-08-19
- 状态：proposed
- 范围：说明“完成演示应用”的网页应呈现哪些信息及其空间分布；不规定视觉风格、前端框架或通信实现。
- 相关边界：[ADR-002](../decisions/2026-08-18_ADR-002_target-localization-scope.md)、[接口表](interfaces.md)、[实验运行规则](../experiments/README.md)

## 1. 目的与边界

网页是答辩/录屏时的应用展示层，不替代 RViz2、rosbag、标定工具或离线评估脚本。它应让观看者在一个页面内回答四个问题：

1. 被定位的**外部目标**当前在哪里？
2. 这个位置来自 UWB、视觉还是融合，数据是否仍新鲜？
3. 视觉系统实际看到了什么？
4. 已完成实验的精度证据是什么？

页面不得把无人机/平台位姿当作目标位姿；不得把规划目标或未登记的临时数值显示为实测指标。网页支持三种明确标记的输入模式：`LIVE`（真实话题）、`REPLAY`（有运行编号的 ros2 bag 回放）和 `NO_INPUT`（没有传感器输入）。`REPLAY` 不是 `LIVE`，但可作为演示视频素材。

## 2. 页面总体分布

以答辩常用的 16:9 屏幕为主，信息优先级为“空间位置 → 视觉证据 → 数据健康 → 已验证指标”。推荐布局如下；比例是内容占位建议而非视觉规范。

```text
┌──────────────────────────────────────────────────────────────────────┐
│ [模式/运行编号]  [目标 ID]  [数据年龄]  [来源模式]  [录制/回放状态]       │  7%
├──────────────────────────────────────┬───────────────────────────────┤
│                                      │ 相机成像与检测证据              │
│  沙盘/地图空间视图                    │ - 最新压缩图像                  │
│  - map 坐标轴、边界、已知点            │ - 标签角点/目标框                │
│  - 最终目标位置与短轨迹                │ - 图像采样时刻、数据年龄          │
│  - UWB/视觉原始观测（可开关）          │ - 检测 ID、置信度、重投影误差     │
│  - 真值/规划点（仅适用时）             │ - 无图像/检测丢失原因             │
│                                      │                               │
│                 约 60%               │             约 40%              │  68%
├───────────────────────┬──────────────┴─────────────┬─────────────────┤
│ 当前定位数值与健康状态  │ 传感器/融合状态与最近趋势   │ 已登记实验指标    │
│ x, y, z / frame / 单位 │ UWB、视觉、配对、延迟、丢帧 │ RMSE/P95/可用率   │
│ source / confidence    │ 近 30--60 秒状态/延迟趋势   │ 运行编号和口径    │ 25%
└───────────────────────┴────────────────────────────┴─────────────────┘
```

窄屏时按上述顺序垂直堆叠；不得因为屏幕变窄而隐藏模式、数据年龄、来源模式或运行编号。

## 3. 各区域的内容与语义

### 3.1 顶部：本次演示的身份与有效性

顶部是全页始终可见的“证据标签”，不展示冗余的系统设置。

| 字段 | 内容 | 规则 |
|---|---|---|
| 输入模式 | `LIVE`、`REPLAY` 或 `NO_INPUT` | 模式不可省略；回放须额外显示运行编号。 |
| 运行编号 | 当前录制/回放关联的 `E###` 运行编号 | LIVE 尚未落盘时显示 `UNREGISTERED`，其数值不作为答辩证据。 |
| 目标 ID | 当前 `target_id` | 无目标时显示“未检测到目标”，不沿用旧 ID。 |
| 最终数据年龄 | 当前时刻减最终目标消息的采样时刻 | 页面核心健康指标；以 ms/s 表达。 |
| 来源模式 | UWB、VISION、FUSED 或明确的降级/无效状态 | 来源来自消息，不由前端推断。 |
| 会话状态 | 连接、录制中、回放中、已暂停或无输入 | 只描述事实，不显示“精度达标”类判断。 |

“新鲜/延迟/过期”的阈值由一次已记录的频率与时延测试决定，并写进运行配置。前端只按该配置显示状态；不以页面收到 WebSocket 包的时间伪造传感器实时性。采样时钟与浏览器时钟不可比时，数据年龄显示为 `unknown`，而不是猜测。

### 3.2 左侧主区：目标在任务空间中的位置

这是页面的主叙事区域，使用 `map` 平面俯视图；它首先服务“目标在哪里”，不是飞行控制地图。

必须呈现：

- `map` 坐标轴、单位（m）、原点和当前展示范围；沙盘底图只能是经过坐标配准的背景，否则只用网格。
- 融合/最终目标位置及最近一段有限长度轨迹；轨迹时间窗口和采样率在页面或配置中可查。
- 当前 `target_id` 和该位置的采样时间；不新鲜时保留最后位置作为历史轨迹，但不能画成“当前”。
- 可切换但默认可见的 UWB 与视觉原始目标观测，使评委能看出融合前后的关系。
- 已经测量并适用于当前会话的真值点/规划路径；真值与规划点的图例必须不同。没有同步真值时，不显示“实时误差箭头”。

平台 `base_link` 位置仅在平台间接测量模型下以次要辅助标记显示，并注明“平台”；它不与目标共用图例、颜色或名称。

### 3.3 右侧：相机成像与检测证据

页面应预留约三分之一至五分之二宽度给相机画面。这一块不是装饰，它证明视觉观测的来源，并解释视觉失效或切换。

画面层展示最新**压缩预览帧**；元数据层展示：相机 ID、图像采样时刻、图像年龄、分辨率/帧率、检测到的目标 ID。检测成功时在画面上叠加标签角点/目标框、目标 ID 和检测置信度；PnP 可用时再显示重投影误差（px）。检测失败时保留最后帧作为历史参考，但应覆盖“视觉当前不可用”和原因（无帧、无标签、过期、求解失败等），而不是冻结成似乎仍有效的画面。

相机预览与定位数据分两条通道：前者可降至 `640×480`、5--10 FPS 以保证演示可用，后者保留定位所需的原始采样时间戳。页面同时显示两者时间，不能假设“画面最新”即“定位最新”。原始视频/rosbag 不进入 Git；录制位置和校验值登记在实验运行记录。

### 3.4 底部左：当前定位数值

只放当前决策所需的少量字段：

- 最终目标 `x, y, z`，单位 m，`frame_id`；
- `source_mode`、置信度和位置协方差摘要（例如各轴标准差）；
- 采样时刻、数据年龄和最后一次有效更新序号；
- 若当前存在同步真值：瞬时三维误差（m）与真值点 ID；若不存在真值则明确写“当前无真值，未计算实时误差”。

这一区不显示平台坐标，避免用户误读对象。

### 3.5 底部中：链路与融合健康

该区让演示者解释“为什么当前位置可信/为什么进入降级”。每个输入源只需要显示状态、最近采样时刻、年龄和连续性：

| 项目 | 必要内容 |
|---|---|
| UWB 目标观测 | 是否有效、最后采样时刻/年龄、序号缺口数、坐标 frame。 |
| 视觉目标观测 | 是否有效、相机 ID、最后采样时刻/年龄、检测状态。 |
| 融合 | 是否产生输出、参与来源、最近时间配对差、来源模式。 |
| 通信链路 | ROS2 网关连接状态、最近消息序号、接收频率、丢帧/过期计数。 |

趋势图只保留一个小型、最近 30--60 秒的“数据年龄或端到端延迟”曲线。只有有持续同步真值时，才将其替换或并列为实时误差曲线。

### 3.6 底部右：已验证实验指标

这个区域是**实验快照**，不是从没有真值的实时流临时算出的精度结论。选择一个已登记运行后，展示该运行的算法、场景、样本数、单位和异常值规则，以及：3D RMSE、P95、最大误差、可用率、更新频率、P95 延迟。数据来源为运行目录 `metrics/` 和 `docs/records/evidence-index.md`。

尚无合格运行时，该区显示“无已登记实测指标”，可显示下一步实验名称，但不能填入 `5 cm` 等规划目标。这样页面把实时演示和可答辩证据明确分开。

## 4. 最小数据契约与更新策略

第一版只接入下表中的数据。最终目标、视觉原始观测和平台状态已在现有接口草案中有位置；图像和指标快照在硬件验收后按接口变更流程补充，不能提前假设话题名或消息语义。

| 数据 | 最小字段 | 页面用途 | 更新策略 |
|---|---|---|---|
| 最终目标观测 | `target_id`、`frame_id`、采样时间、位置（m）、协方差、置信度、来源模式、序号 | 主地图、顶部、当前数值 | 每收到一条更新；仅保留有限历史轨迹。 |
| UWB/视觉原始观测 | 同上及传感器标识 | 主地图的对照点、健康区 | 每收到一条更新；缺失计数。 |
| 平台状态（条件性） | 位姿、frame、采样时间 | 仅平台间接模型的辅助上下文 | 不能作为目标替身。 |
| 相机压缩预览 | 相机 ID、图像采样时间、编码、图像数据；检测叠加/质量 | 相机区 | 独立限帧；定位消息不等待图像。 |
| 已登记指标快照 | 运行编号、算法、样本数、单位、统计口径、RMSE/P95/可用率/延迟 | 实验指标区 | 页面加载或用户选择运行时读取；不随实时流改写。 |

网关向浏览器发送轻量状态快照或事件流，所有定位事件保留原始采样时间戳与序号。浏览器只负责展示和短时间轨迹缓存；指标计算、坐标转换、来源模式判定和数据有效性由 ROS2/离线分析侧完成。这样网络抖动不会让网页成为定位算法的一部分。

## 5. 必要交互与状态

第一版交互限制为：选择当前目标（多目标时）、切换原始观测图层、选择已登记实验运行、开始/暂停回放和查看当前会话详情。不得允许用户在页面中修改标定、融合权重或真值坐标；这些属于受版本控制的实验配置。

页面必须有下列显式状态：

- 没有目标消息：地图和数值区显示“等待真实输入”，不生成模拟点；
- 有旧目标但已过期：历史位置降级为轨迹，当前值显示“过期”；
- 视觉失效、UWB 仍有效：显示实际来源模式和视觉失效原因；
- 回放暂停或结束：固定为回放状态，所有年龄/播放位置依据回放时钟；
- 输入语义不完整（无 frame、单位、时间戳或目标 ID）：网关拒绝并在健康区计数，不能显示位置。

## 6. 分阶段实现与验收

| 阶段 | 页面内容 | 验收标准 |
|---|---|---|
| P1：位置演示 | 顶部身份、主地图、最终目标与 UWB/视觉观测、无输入/过期状态 | 能用真实 ROS2 消息或合法录包展示目标与来源，且不会混淆平台和目标。 |
| P2：视觉证据 | 相机预览、检测叠加、视觉健康状态 | 图像时间与定位时间分别可见；图像丢失不会阻塞定位展示。 |
| P3：证据闭环 | 已登记运行指标、回放控制、数据健康趋势 | 每个指标均关联运行编号、样本数、单位和统计口径；无实测时不显示精度结论。 |

在 P1 通过前不做复杂地图、账户、远程控制、历史数据库或视觉特效。网页的验收以“能清楚演示目标定位链路且不误导证据语义”为准，而不是页面的装饰程度。

## 7. 网页设计生成提示词（优化版）

下面的提示词用于生成网页高保真线框、UI 视觉稿或交给前端实现。它描述的是网页仪表板的内容层级和工程风格，不是 PPT 页面，也不是营销型落地页。

```text
Design a high-fidelity 16:9 desktop web dashboard for a real-time external-target localization system used in an indoor drone test arena.

The product is an engineering monitoring console, not a presentation slide, marketing landing page, sci-fi cockpit, or cyberpunk interface. Use a restrained light industrial style: warm off-white background, cool gray panels, thin graphite borders, medium corner radius, clear spacing, dark charcoal typography, and a small controlled accent palette. Keep the interface calm, credible, technical, and readable from a projected screen.

Use a strong three-level visual hierarchy:
1. The main spatial view is the dominant visual area.
2. The camera evidence panel is the secondary visual area.
3. Diagnostics and metadata are compact supporting areas.

Layout:
- A full-width top status bar, about 8% of the height, containing input mode, run ID, target ID, data age, source mode, and recording/replay state.
- A large left map panel, about 58–62% of the main area, showing a calibrated map frame in meters, coordinate axes, arena boundary, target position, short recent trajectory, optional UWB and vision observations, and known reference points when applicable.
- A right camera evidence panel, about 30–36% of the main area, showing a compressed camera preview with target bounding box or AprilTag/ArUco corner overlays, camera ID, image timestamp, frame age, detection status, confidence, and reprojection error when available.
- A compact bottom diagnostics row containing: current target coordinates and frame, sensor/fusion health, a small recent latency or data-age trend, and a registered experiment summary.

Visual semantics:
- UWB uses one consistent blue accent.
- Vision uses one consistent teal accent.
- Fusion uses one distinct but restrained accent color.
- Stale, missing, or invalid data uses amber/red only as a status signal.
- Do not use color as decoration; every accent must have a data meaning.

Make the map and target immediately recognizable from a distance. Use larger numeric typography for x/y/z and data age, while keeping metadata smaller. Do not give every card equal visual weight. Avoid excessive borders, dense paragraphs, glossy gradients, neon glow, 3D perspective, decorative icons, fake controls, and unnecessary charts.

The page must visually distinguish LIVE, REPLAY, and NO INPUT states. A stale timestamp must look stale; a replay state must not look live. Keep a clear separation between the external target and the drone platform. Do not show the drone platform as the target.

Use concise Chinese UI labels with a few technical English tokens where standard, for example: “目标位置”, “相机成像与检测证据”, “数据年龄”, “来源模式”, “融合状态”, “运行编号”, LIVE, REPLAY, UWB, VISION, FUSED, frame_id, RMSE, P95.

The final result should look like a real operational dashboard that updates continuously during a ROS2 live stream or ros2 bag replay. Show subtle live-state cues such as a changing data-age value, an updating trajectory, and refreshed detection overlays, but no flashy animation. Use clearly marked placeholder/demo values; do not present invented measurements as real experimental results.

Output: one polished web dashboard screen, front-facing, 16:9, high information density but easy to scan, no browser chrome, no PPT title layout, no marketing copy.
```

### 7.1 生成时的负面约束

若工具支持 negative prompt，追加：

```text
no PPT slide, no title-page composition, no marketing landing page, no cyberpunk neon, no black sci-fi cockpit, no glossy glassmorphism, no excessive gradients, no 3D dashboard, no ornamental gauges, no giant hero headline, no decorative drones, no unrelated maps, no fake precision claims, no illegible tiny text, no equal-weight card grid
```

## 8. 可选辅助视图与阶段性展示

主网页保留为“运行主视图”。其他信息不必全部塞入同一屏，而应作为标签页、侧抽屉或演示阶段切换的辅助视图。它们服务不同问题：主视图回答“现在在哪里”，辅助视图回答“链路是否正常、如何标定、出了什么故障、结果是否可复查”。

### 8.1 诊断视图：结构化终端日志

终端日志可以展示，但不建议把原始 stdout 或整屏黑色终端直接嵌进主页面。更适合做成“事件日志”面板：默认显示最近 8--12 条与定位链路有关的结构化事件，支持展开查看原始日志。

每条事件至少包含时间戳（优先传感器采样时间，另列接收时间）、组件（`bridge`、`uwb`、`vision`、`fusion`、`dashboard`）、级别（`INFO`、`WARN`、`ERROR`）、事件类型（连接、首帧、检测、坐标变换、融合切换、丢帧、过期、恢复）、目标 ID、运行编号和简短说明。

适合展示的日志示例：

```text
10:30:15.204  vision  INFO  TARGET_DETECTED  target=TARGET_001  camera=cam_1
10:30:16.011  fusion  WARN  SOURCE_STALE     source=VISION  age=0.24s
10:30:16.018  fusion  INFO  FALLBACK_UWB     target=TARGET_001
10:30:17.104  bridge  WARN  SEQUENCE_GAP     topic=/nexus/target/pose  missing=2
```

日志面板的价值是解释状态变化，而不是证明定位精度。原始高频日志仍保存在运行目录或外部文件中，网页只订阅经过限流和脱敏的事件流。不要展示密钥、设备序列号或无关系统日志。

### 8.2 链路诊断视图：数据是否真的实时

在需要说明实时性的阶段，打开链路诊断视图，展示从传感器到浏览器的时间链：

```text
传感器采样 → ROS2 接收 → 算法输出 → 网关转发 → 浏览器接收
     t0            t1             t2             t3             t4
```

对应显示各段时间差、消息序号连续性、最近接收频率、过期次数和 WebSocket 重连次数。该视图比单独显示“LIVE”更能说明实时链路；没有可比时钟时明确显示 `clock not synchronized`。

### 8.3 标定/坐标视图：解释坐标转换

标定阶段可临时展示：`map`、`camera_i`、`uwb_anchor_i`、`target_link` 的坐标轴和变换关系；相机画面中的检测点与三维目标点；基站位置与 GDOP 摘要；当前标定文件版本和来源运行编号。

该视图只用于标定验收和技术答辩，不应在普通实时演示中长期占据主屏。它必须把“平台位姿”和“外部目标位姿”分开标注。

### 8.4 回放/证据视图：把结果串成过程

对 `ros2 bag` 回放或已登记运行，增加时间轴和事件标记：开始采集、目标进入、视觉检测、融合接管、遮挡、恢复、结束。可联动地图轨迹、相机帧、结构化日志和指标卡。

该视图适合演示一次完整过程：

```text
正常融合 → 视觉丢失 → UWB 降级 → 视觉恢复 → 融合恢复
```

回放页面必须一直显示 `REPLAY` 和运行编号，不能模拟成 `LIVE`。

### 8.5 实验对比视图：算法过程之外的证据

当需要展示算法比较时，使用独立的证据视图或抽屉，不改写实时主地图。可选择同一运行中的 UWB、单视觉、多视觉和融合，显示误差分布、P95、可用率、延迟和样本数，并保留单位、坐标系、异常值规则和数据来源。

它是实验结果查看器，不是实时控制台；无正式运行时显示“无已登记实测指标”。

### 8.6 建议的演示阶段顺序

一场答辩或录屏不需要同时打开所有视图，推荐按以下顺序切换：

1. **主运行视图**：地图 + 相机 + 当前状态，先让评委看懂系统在定位什么。
2. **状态切换视图**：展示视觉丢失、UWB 接管和恢复，证明融合/降级确实工作。
3. **链路诊断视图**：短暂展示采样时间、延迟、序号和结构化日志，回答“是否实时收到”。
4. **回放/证据视图**：选择已登记运行，展示误差曲线和算法对比，回答“结果是否可信”。

终端日志、标定信息和算法指标都有展示价值，但它们属于不同的叙事阶段。主网页保持简洁，辅助视图按需出现，整体信息密度会比把所有内容固定在一张屏上更有冲击力。
