# NEXUS 在线目标定位算法框架设计 v01

- 日期：2026-09-02
- 类型：design
- 状态：draft
- 关联：[无标记目标定位优化方案 v01](2026-08-29_design_markerless_target_localization_optimization_v01.md)（本文件是其附五的独立成文与展开）、
  [ADR-003](decisions/2026-08-20_ADR-003_lnn_optional_fusion_enhancement.md)、
  [ADR-006](decisions/2026-08-21_ADR-006_target_observation_contract_v2.md)、
  [ADR-008](decisions/2026-08-25_ADR-008_markerless_visual_first_and_interface_authority.md)、
  [ADR-009](decisions/2026-08-29_ADR-009_markerless_depth_channel_and_symmetry.md)
- 标签口径：`REQUIREMENT / IMPLEMENTED / MEASURED / TARGET / REFERENCE / TBD`。
  `MEASURED` 必须能追到实测命令或 `experiments/runs/` 运行号。
- 传感器边界：**单目相机 + UWB 测距 + 飞控 IMU**。无深度相机、无激光、无 GNSS 依赖、无云台。
- 精度口径：`2 cm` 是 **TARGET 且仅对仿真成立**。已记录需求是
  `核心演示区 ≤5cm（视觉可见时）/ 全场景兜底 <30cm`。
- 本文档所有精度数字是**闭式推导或蒙特卡洛**（TARGET），**没有一项来自实飞运行**。

## 0 一句话

**用移动的单目相机做"时间基线"三角化得到目标三维位置，UWB 只在段级提供米制尺度与 `map` 基准，
IMU + 视觉提供段内相对几何；输出 2D 地面位置（保证达标）与 3D 位置（争取）并行，
两条高度通道互为交叉校验。全链在树莓派 5 的 CPU 上跑，不需要 GPU。**

## 1 系统总览

```text
┌─ 机载 (Raspberry Pi 5, 4x Cortex-A76, 无 GPU) ──────────────────────────┐
│                                                                          │
│  IMX219 单目 ──41.85 fps──┐                                              │
│                           ├─► L2 视觉前端 ─► 目标方位 (F1)                │
│  飞控 MAVLink ─┐          │      亚像素块跟踪 + 低频检测仲裁               │
│    SCALED_IMU  │          │                                              │
│      188.9 Hz  ├─► L1 平台端 ─► 平台位姿 + 段内相对几何 (F5/F7)           │
│    UWB 测距    │    UWB-aided VIO      + 段级尺度与基准 (F6)              │
│      1 Hz      │                                                         │
│    厂商位置    ┘                          ▼                              │
│      10 Hz                        L2 滑窗捆绑求解                         │
│                                           ▼                              │
│                        目标 2D/3D 位置 + 协方差 + validity                │
└──────────────────────────┬───────────────────────────────────────────────┘
                           │ (可选分体：图像回传，地面跑检测器与 VIO)
┌─ 地面 (Windows + RTX 4070) ─────────────────────────────────────────────┐
│  L3 低频：检测器重初始化 · 场景锚定 (F4) · 锚点在线精化 · 记录与出图        │
└──────────────────────────────────────────────────────────────────────────┘
```

**分工的核心主张（角色反转）**：

$$
\underbrace{\text{视觉}}_{\text{段内相对几何 + 亚像素方位}}
\quad\oplus\quad
\underbrace{\text{UWB}}_{\text{段级尺度 + }map\text{ 基准}}
\quad\oplus\quad
\underbrace{\text{IMU}}_{\text{高频姿态传播}}
$$

不是"UWB 定位无人机、视觉只出一个框"。理由见第 9 节：逐帧用 UWB 定基线时，
基线不确定性单独就贡献 `53 mm` 深度误差，比所有其他项之和还大。

## 2 坐标系定义

| 帧 | 记号 | 定义 | 手性 | 谁实现它 |
|---|---|---|---|---|
| CARLA 世界系 | $W_c$ | $x$ 前、$y$ **右**、$z$ 上 | **左手** | CARLA 仿真 |
| 地图系 | $M$ | 原点在场地一角，$x$ 东、$y$ 北、$z$ 上，单位 m | 右手 | **UWB 锚点坐标**（不是测距本身） |
| 机体系 | $B$ | REP-103：$x$ 前、$y$ **左**、$z$ 上 | 右手 | 飞控 / UWB 解算 |
| 相机光学系 | $C$ | $x$ 右、$y$ **下**、$z$ 沿光轴向前 | 右手 | `base_to_camera` 外参 |
| 目标系 | $T_k$ | 第 $k$ 个目标的几何参考中心 | 右手 | 目标先验配置 |
| 支撑面 | $\Pi$ | $M$ 中的水平面 $z = z_\Pi$ | — | 场地 survey |

记号约定（全文统一，与 `code/analysis/localization/factors.py` 一致）：
位姿 $T_{AB} = (R_{AB}, t_{AB})$ 把 $B$ 系中的点映到 $A$ 系，

$$
p^A = R_{AB}\, p^B + t_{AB},
\qquad
T_{AB} = \begin{bmatrix} R_{AB} & t_{AB} \\ 0^\top & 1 \end{bmatrix}
$$

## 3 坐标转换公式

### 3.1 CARLA 左手系 → `map` 右手系

CARLA 的 $y$ 轴指向右，构成左手系。翻转 $y$ 即可换到右手系。令

$$
S = \operatorname{diag}(1, -1, 1), \qquad S = S^{-1} = S^\top
$$

点与旋转分别按下式换算（旋转是**基变换**，要两边同乘）：

$$
p^{M} = S\, p^{W_c},
\qquad
R_{M\cdot} = S\, R_{W_c\cdot}\, S,
\qquad
t_{M\cdot} = S\, t_{W_c\cdot}
$$

由于 $\det S = -1$，单独用 $S R$ 会得到非正交的反射矩阵；实现里 $S$ 与后面的换轴矩阵
（$\det = -1$）复合，总行列式回到 $+1$，这一点在单测里断言。

### 3.2 CARLA 姿态角 → 旋转矩阵

CARLA 用度制的 $(\text{pitch}\ \theta,\ \text{yaw}\ \psi,\ \text{roll}\ \phi)$，其 `get_matrix()` 等价于

$$
R_{W_c A} =
\begin{bmatrix}
c_\theta c_\psi & c_\psi s_\theta s_\phi - s_\psi c_\phi & -c_\psi s_\theta c_\phi - s_\psi s_\phi \\
c_\theta s_\psi & s_\psi s_\theta s_\phi + c_\psi c_\phi & -s_\psi s_\theta c_\phi + c_\psi s_\phi \\
s_\theta & -c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix}
$$

其中 $c_\bullet=\cos$、$s_\bullet=\sin$，$A$ 为 actor 自身系。**不要**用一般的
$R_z R_y R_x$ 代替它，符号不同。

### 3.3 换轴矩阵：actor / 机体 → 光学系

光学系约定 $x$ 右、$y$ 下、$z$ 前。矩阵的**列**是光学系三轴在源系中的表示。

CARLA actor 系（$x$ 前、$y$ 右、$z$ 上）：右 $=+y$，下 $=-z$，前 $=+x$，故

$$
A_{\text{actor}\to C} = \begin{bmatrix} 0 & 0 & 1 \\ 1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix},
\qquad \det = -1
$$

REP-103 机体系（$x$ 前、$y$ 左、$z$ 上）：右 $=-y$，下 $=-z$，前 $=+x$，故

$$
R_{BC}^{\text{swap}} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}
$$

若相机还有安装俯仰 $\beta$（下视时 $\beta=-90^\circ$，绕机体 $y$ 轴），则

$$
R_{BC} = R_y(\beta)\, R_{BC}^{\text{swap}},
\qquad
R_y(\beta) = \begin{bmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{bmatrix}
$$

仿真里由 CARLA 相机变换直接得到 `map` 下的光学位姿：

$$
R_{MC} = S\, R_{W_c A}\, A_{\text{actor}\to C},
\qquad
t_{MC} = S\, t_{W_c A}
$$

（`det`: $(-1)\cdot(+1)\cdot(-1) = +1$，是合法旋转。）

### 3.4 完整位姿链

$$
T_{M T_k}
= \underbrace{T_{MB}}_{\text{UWB 位置 + 飞控/VIO 姿态}}
\cdot \underbrace{T_{BC}}_{\text{base\_to\_camera 外参}}
\cdot \underbrace{T_{C T_k}}_{\text{视觉估计（本方案要解的）}}
$$

展开成本方案实际使用的形式（与 `simulation/fuse_uwb_visual_targets.py` 一致）：

$$
R_{M T_k} = R_{MB}\, R_{BC}\, R_{C T_k},
\qquad
t_{M T_k} = t_{MB} + R_{MB}\big(t_{BC} + R_{BC}\, t_{C T_k}\big)
$$

**误差是串联的**：

$$
\Sigma_{M T_k} \approx
J_{B}\,\Sigma_{MB}\,J_{B}^{\top}
+ J_{\text{ext}}\,\Sigma_{BC}\,J_{\text{ext}}^{\top}
+ J_{C}\,\Sigma_{C T_k}\,J_{C}^{\top}
$$

所以「UWB 把无人机定准了」$\ne$「目标定准了」。$\Sigma_{BC}$（外参未标定）与
$\Sigma_{MB}$ 的系统分量（锚点未 survey）都不随帧数平均。

### 3.5 针孔投影与方位（视线）

内参 $K$，相机系点 $p^{C}=(X,Y,Z)^\top$，像素 $u$：

$$
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix},
\qquad
u = \pi(p^{C}) = \frac{1}{Z}\begin{bmatrix} f_x X + c_x Z \\ f_y Y + c_y Z\end{bmatrix}
$$

归一化像面坐标（本方案 F1 用它，避免把 $K$ 混进残差权重）：

$$
\tilde{x} = \begin{bmatrix} X/Z \\ Y/Z \end{bmatrix} = K^{-1}\begin{bmatrix} u \\ 1\end{bmatrix}\Big/\left[K^{-1}\begin{bmatrix} u \\ 1\end{bmatrix}\right]_3
$$

单位视线方向（在 $M$ 系中）：

$$
d^{M} = \frac{R_{MC}\,[\tilde{x}^\top,\,1]^\top}{\big\|R_{MC}\,[\tilde{x}^\top,\,1]^\top\big\|}
$$

像素噪声 $\sigma_u$ 折算成角度噪声：$\sigma_{\text{bearing}} \approx \sigma_u / f_x$（rad）。
实测 `1640×1232` 模式下 $f_x = 989.4$，故 $1\ \text{px} \approx 1.01\ \text{mrad}$。

### 3.6 输出的两种解法

**（A）3D 三角化**：$N$ 条视线 $(o_i, d_i)$ 到点 $p$ 的垂距平方和最小。令
$P_i = I - d_i d_i^\top$（投影到视线正交补），

$$
\hat{p} = \left(\sum_{i=1}^{N} P_i\right)^{-1}\left(\sum_{i=1}^{N} P_i\, o_i\right)
$$

深度精度的一阶近似（$Z$ 为距离，$B$ 为基线）：

$$
\sigma_Z \approx \frac{Z^{2}\,\sigma_u}{f_x\, B\, \sqrt{N}}
\quad\oplus\quad
\underbrace{Z\,\frac{\delta B}{B}}_{\text{基线不确定性}}
$$

**第二项是主误差**，也是必须用视觉（而非逐帧 UWB）提供基线的原因。

**（B）2D 支撑面求交**：目标停在已知平面 $z=z_\Pi$ 上时，单帧即可解

$$
\lambda_i = \frac{z_\Pi - o_{i,z}}{d_{i,z}},
\qquad
p_i = o_i + \lambda_i d_i,
\qquad
\hat{p}_{xy} = \frac{1}{N}\sum_{i=1}^{N} p_{i,xy}
$$

要求 $|d_{i,z}|$ 不接近 0（视线不能平行于支撑面）。下视构型天然满足。

### 3.7 UWB 测距与几何精度因子

锚点 $a_j \in M$，标签位置 $x$，测距 $r_j$。残差与雅可比：

$$
e_j(x) = \|x - a_j\| - r_j,
\qquad
J_{j} = \frac{\partial e_j}{\partial x} = \frac{(x - a_j)^\top}{\|x - a_j\|}
$$

高斯-牛顿迭代（初值必须与真值无关，见第 4.4 节）：

$$
x \leftarrow x - (J^\top J)^{-1} J^\top e
$$

几何精度因子（$Q = (J^\top J)^{-1}$）：

$$
\mathrm{HDOP} = \sqrt{Q_{11}+Q_{22}},\quad
\mathrm{VDOP} = \sqrt{Q_{33}},\quad
\mathrm{PDOP} = \sqrt{\operatorname{tr} Q},\quad
\sigma_{\text{pos}} = \mathrm{PDOP}\cdot\sigma_r
$$

### 3.8 段级尺度与基准（F6）

单目/VIO 轨迹 $\{s_i\}$ 只到相似变换。段内用 UWB 位置 $\{u_i\}$ 定尺度 $\alpha$ 与偏置 $b$：

$$
(\hat{\alpha}, \hat{b}) = \arg\min_{\alpha, b} \sum_{i} \left\| \alpha\, s_i + b - u_i \right\|^2
$$

（实现里 $\alpha$ 以 $\log\alpha$ 参数化保证正定。若需要同时估旋转，用 Umeyama 相似变换对齐，
仓库已有 `nexus_coord_transform.geometry.umeyama_alignment`。）
尺度残差随段内测距组数 $N_u$ 下降：$\delta\alpha/\alpha \propto \sigma_r \mathrm{PDOP} / (B\sqrt{N_u})$。

## 4 输入：是什么、怎么拿

### 4.1 输入清单（实机侧数字为 MEASURED，2026-09-02 `ssh pi@192.168.1.143` 实测）

| 输入 | 来源 | 速率 | 单位与坐标系 | 状态 |
|---|---|---:|---|---|
| 单目 RGB 图像 | IMX219 via `rpicam` / libcamera | `41.85 fps`（`1640×1232` 全幅 2×2 合并） | 像素，$C$ 系 | MEASURED（模式表） |
| IMU 加速度+角速度 | 飞控 `SCALED_IMU` | **`188.9 Hz`** | **加速度 `mm/s²`**、角速度 `mrad/s` | MEASURED |
| UWB 四路测距 | 飞控 `BATTERY_STATUS.voltages[2:6]` | `1.0 Hz` | `cm`；`0`/`65535` 为不可用哨兵 | MEASURED |
| 厂商融合位置 | 飞控 `GLOBAL_VISION_POSITION_ESTIMATE` | `10.0 Hz` | `x/y` 为 `cm`；`z`/`roll`/`pitch`/`yaw` 字段存在但**有效性未验证** | MEASURED（速率）/ TBD（字段） |
| 平台速度 | 飞控 `GLOBAL_POSITION_INT` | `10.0 Hz` | `cm/s` | MEASURED |
| 相机内参 $K$ | 离线标定 / 数据集 `calibration/camera.json` | 一次 | 像素 | IMPLEMENTED（仿真值） |
| 锚点坐标 $a_j$ | 场地 survey | 一次 | m，$M$ 系 | **TBD：未实测** |
| 支撑面高度 $z_\Pi$ | 场地 survey / 地图 | 一次 | m，$M$ 系 | **TBD：未实测** |
| `base_to_camera` 外参 $T_{BC}$ | 离线标定 | 一次 | m + 旋转 | **TBD：现为理想单位阵** |
| 目标标称尺寸/高度 | `config/rrd_target_priors_v01.yaml` | 一次 | m，声明的设计先验**不是真值** | 待建 |

**飞控明确不提供的两项**（实测 20 s 计数为 0）：`ATTITUDE`（融合姿态）、
`OPTICAL_FLOW` / `OPTICAL_FLOW_RAD`（尽管硬件表写"光流定位 支持"）。
所以姿态必须由 VIO 自己从 `SCALED_IMU` 传播，或确认能用
`GLOBAL_VISION_POSITION_ESTIMATE` 里回传的角度字段。

### 4.2 单位换算（照 MAVLink 规范解会错，必须按实测）

$$
a\,[\mathrm{m/s^2}] = \frac{\texttt{xacc,yacc,zacc}}{1000},
\qquad
\omega\,[\mathrm{rad/s}] = \frac{\texttt{xgyro,ygyro,zgyro}}{1000}
$$

实测样本 $(-582, 44, -9873)$，模长 $\approx 9890$，$\div 1000 = 9.89\ \mathrm{m/s^2} \approx g$
⇒ **加速度是厂商自定的 `mm/s²`，不是 MAVLink 规范的 mG**；按 mG 解会差约 $9.8\times$，
在 VIO 里直接发散。角速度与规范一致（静置 $\texttt{zgyro}=11 \Rightarrow 0.63\ ^\circ/\mathrm{s}$）。
磁力计 $\texttt{xmag}=20685$ 与 mgauss 对不上（会得 $2068\ \mu\mathrm{T}$，地磁只有 $\sim 50\ \mu\mathrm{T}$），
**语义未确认，暂不使用**。

UWB 测距：

$$
r_j\,[\mathrm{m}] = \frac{\texttt{voltages}[j+2]}{100},
\qquad
\texttt{voltages}[j+2] \in \{0,\ 65535\} \Rightarrow \text{该锚点不可用}
$$

`~/read_uwb.py` **没有检查这个哨兵**，一个不可用的距离会静默变成 `655.35 m`；
`code/deployment/raspberry_pi_uav_readonly_adapter/uav_readonly_adapter.py` 处理正确。

### 4.3 怎么拿到输入（链路与独占约束）

```text
飞控 ──UART /dev/ttyAMA0 @115200──► 树莓派
                                      │  独占！只能一个进程打开
                                      ▼
                        fanciswarm_bridge.py（唯一读串口者）
                                      │ UDP
                        ┌─────────────┴──────────────┐
                        ▼                            ▼
              在线定位适配器（订阅方）          地面站 / 记录
```

**硬约束（2026-09-02 实测踩到）**：`/dev/ttyAMA0` 是独占资源。排查时它被
`fanciswarm_bridge.py --serial /dev/ttyAMA0 --udp-host 192.168.1.140 --udp-port 14550`
占用，第二个读取者报 `device reports readiness to read but returned no data
(device disconnected or multiple access on port?)`。因此：
**由一个进程独占串口并经 UDP 分发，在线链路订阅 UDP，不得再抢串口。**

图像侧：`rpicam` / libcamera 直接取帧，与 MAVLink 链路无关，不存在争用。

### 4.4 输入侧的两条纪律

1. **算法核心不得读取仿真真值。** UWB 解算的初值必须来自锚点几何中心 + 先验高度，
   或上一帧自身的估计；用真实位置播种会让 `position_error_m` 变成乐观的最好情况。
2. **时间戳权威在采集时刻**，不是收到时刻。分体部署时链路延迟只影响显示新鲜度，
   不影响该时刻估计的正确性；但链路要进 `availability` 分母，丢链发 `INVALID`（ADR-006）。

## 5 输出：是什么、怎么得到

### 5.1 输出契约

**不新增 ROS 消息类型。** 沿用 `nexus_msgs/msg/TargetObservation`（IMPLEMENTED）：
`SOURCE_PLATFORM_RELATIVE=3` / `SOURCE_FUSED=4`，`float64[36] covariance`，
`validity` + `invalid_reason`。链路差异记在协方差与 validity 上。

逐次求解另落盘一条记录（`code/analysis/localization/records.py`，已实现）：

```text
frame_id, obj_id, class_name, chain,
x_map_m, y_map_m, z_map_m, qx, qy, qz, qw,
sigma_x_m, sigma_y_m, sigma_z_m, sigma_rot_deg,
n_views, baseline_m, depth_source, validity, degraded_reason, runtime_ms
```

**两路并行输出**（本框架的选定形态）：

| 输出 | 内容 | 高度来源 | 精度（TARGET） | 定位 |
|---|---|---|---|---|
| **2D 主输出** | $(x, y)$ + $z=z_\Pi$ | 支撑面（F3） | 水平 `5.68 mm` | 保证达标，`2 cm` 有 3.5 倍余量 |
| **3D 并行输出** | $(x, y, z)$ | 多视图交会（F1） | 3D `16.30 mm` | 争取；改善后 `5.07 mm` |
| **一致性量** | $\lvert z_{\text{3D}} - z_\Pi \rvert$ | — | 超阈值 ⇒ 标 degraded | 支撑面假设的检验量 |

**旋转不输出。** 只跟单个框中心时目标姿态不可观测，按 ADR-006 写 NaN +
`degraded_reason`，不得复用上一帧、不得报 `rotation_rmse_deg`。
（若 T2 跟踪目标上多个可区分块，姿态可部分可观测——那是后续项，不在 v01 承诺内。）

### 5.2 怎么得到输出：状态量与因子

状态向量（滑窗 $W$ 帧、$K$ 个目标）：

$$
\mathbf{x} = \Big[\underbrace{\{p_{T_k}\}_{k=1}^{K}}_{3K},\;
\underbrace{\{\xi_{C_i}\}_{i=1}^{W}}_{6W},\;
\underbrace{\log\alpha,\ b}_{4},\;
\underbrace{\{a_j\}_{j=1}^{4}}_{12\ \text{(可选，锚点自标定)}}\Big]
$$

$\xi_{C_i}\in\mathfrak{se}(3)$ 为第 $i$ 帧相机位姿的旋转矢量 + 平移。实测规模：
$K=3$、$W=8$ ⇒ $3\cdot3 + 6\cdot8 + 4 = 61$ 参数（含锚点 73 个），**内存 KB 级**。

目标函数（所有残差先按各自 $\sigma$ 白化，再乘链路权重 $w$）：

$$
\hat{\mathbf{x}} = \arg\min_{\mathbf{x}} \sum_{f\in\mathcal{F}} w_f\,\rho\!\left(\big\|r_f(\mathbf{x})\big\|^2\right),
\qquad \rho = \text{soft-}\ell_1\ \text{或 Huber}
$$

各因子残差（与 `factors.py` 逐行对应）：

$$
\begin{aligned}
\textbf{F1 视线：}\quad & r_{1} = \frac{1}{\sigma_{\tilde{x}}}\left(
\frac{\left[R_{MC_i}^{\top}(R_{MT_k} q + p_{T_k} - t_{MC_i})\right]_{1:2}}
     {\left[R_{MC_i}^{\top}(R_{MT_k} q + p_{T_k} - t_{MC_i})\right]_{3}} - \tilde{x}_{\text{obs}}\right)\\[4pt]
\textbf{F2 轮廓线：}\quad & r_{2} = \frac{1}{\sigma_u}\, n^{\top}\!\left(\pi(p^{C}) - u_{\text{obs}}\right)
\quad\text{（需目标网格，RRD 数据下不可用）}\\[4pt]
\textbf{F3 支撑面：}\quad & r_{3} = \frac{1}{\sigma_\Pi}\Big(\big[p_{T_k} + R_{MT_k}(0,0,-h_k/2)^\top\big]_{3} - z_\Pi\Big)\\[4pt]
\textbf{F4 控制点：}\quad & r_{4} = \frac{1}{\sigma_u}\left(\pi\!\left(R_{MC_i}^{\top}(g_m - t_{MC_i})\right) - u_{\text{obs}}\right)\\[4pt]
\textbf{F5 帧间相对位姿：}\quad & r_{5} = \begin{bmatrix}
\sigma_R^{-1}\,\log\!\left(\tilde{R}_{ij}^{\top} R_{MC_i}^{\top} R_{MC_j}\right)\\
\sigma_t^{-1}\left(R_{MC_i}^{\top}(t_{MC_j}-t_{MC_i}) - \tilde{t}_{ij}\right)
\end{bmatrix}\\[4pt]
\textbf{F6 段级尺度：}\quad & r_{6} = \sigma_{6}^{-1}\left(\alpha\, s_i + b - u_i\right)\\[4pt]
\textbf{F7 平台先验：}\quad & r_{7} = \begin{bmatrix}
\sigma_R^{-1}\log\!\left(\tilde{R}_{MC_i}^{\top} R_{MC_i}\right)\\
\sigma_t^{-1}\left(t_{MC_i} - \tilde{t}_{MC_i}\right)\end{bmatrix}\\[4pt]
\textbf{F8 弱先验：}\quad & r_{8} = \begin{bmatrix}
\sigma_p^{-1}(p_{T_k} - \bar{p}_{T_k})\\
\sigma_{\text{up}}^{-1}\left[\log R_{MT_k}\right]_{1:2}\\
\sigma_\psi^{-1}\,\mathrm{wrap}(\psi_k - \bar{\psi}_k)\end{bmatrix}
\end{aligned}
$$

$q$ 为目标系中的点，$g_m$ 为已 survey 的控制点，$\tilde\bullet$ 表示测量值。
F8 在在线模式下同时承担**窗口外信息的携带**：先验取上一窗口的边缘估计与其 $\sigma$
（乘一个膨胀系数），因此不需要新增因子类型。

协方差由高斯-牛顿的信息矩阵给出：

$$
\Sigma = \hat{s}^2\,(J^{\top} J)^{+},
\qquad
\hat{s}^2 = \frac{2\,\text{cost}}{m - n}
$$

$m$ 为残差数、$n$ 为参数数。已验证该协方差是校准的：合成观测下
$\mathbb{E}\!\left[e^2/\sigma^2\right] = 1.093$（理想 1.0），见优化方案 §14.1。

### 5.3 在线求解流程（每帧）

```text
1. 取帧 + 打时间戳（权威时刻）
2. L1 平台端：IMU 预积分 → 位姿传播；UWB 到达时更新尺度与基准
3. L2 视觉前端：
     if 检测器周期到（1–2 Hz）: 检测 → 目标 ID / 重初始化 / 否决跟踪漂移
     else:                      亚像素块跟踪 → 目标像素中心
4. 构因子：F1(方位) + F3(支撑面) + F5/F6/F7(来自 L1) + F8(上一窗口先验)
5. 滑窗求解（max_nfev 硬上限，关闭阶段调度与外点二次求解）
6. 门控：transform 不可用 / 通道打架 / 超预算 ⇒ INVALID，不发猜测位姿
7. 只有 valid 才更新携带状态；输出 2D + 3D + 一致性量 + 协方差
```

窗口滚动时，离窗帧的信息通过 F8 先验带入下一窗口；正规做法是 Schur 补边缘化

$$
\Lambda_{\text{keep}}' = \Lambda_{kk} - \Lambda_{km}\,\Lambda_{mm}^{-1}\,\Lambda_{mk}
$$

在 61–73 参数规模下这个稠密求解是微秒级，**不需要近似**。当前实现用的是 F8 先验近似，
升级到 Schur 边缘化是已登记的后续项。

## 6 用到的算法与开源库

**设计原则：优先拼装成熟开源件，自研压到胶水层（时间同步、单位换算、线程仲裁、因子构建、落盘）。**

| 槽位 | 算法 / 库 | 许可 | 仓库现状 |
|---|---|---|---|
| L1 平台端（主选） | [`aau-cns/uvio`](https://github.com/aau-cns/uvio) — UWB-aided VIO，OpenVINS 生态 | **待核** | 无 |
| L1 平台端（备选） | [`MISTLab/VIR-SLAM`](https://github.com/MISTLab/VIR-SLAM) — Visual-Inertial-**Ranging** SLAM | **待核** | 无 |
| L1 无 IMU 退路 | OpenCV `goodFeaturesToTrack` + `calcOpticalFlowPyrLK` + `findHomography` + `decomposeHomographyMat`（下视近平面**必须**用单应，5 点法退化） | Apache-2.0 | **已装** |
| L1 对照基线 | [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/Vins-Fusion) / [ROS2 移植](https://github.com/yangfuyuan/vins_fusion_ros2)、ORB-SLAM2 | GPLv3 ⚠ 分发边界 | ORB-SLAM2 已是 submodule |
| L2 亚像素跟踪 | OpenCV `TrackerCSRT` / `TrackerVit` + `findTransformECC` + `calcOpticalFlowPyrLK` | Apache-2.0 | **已装** |
| L2 目标估计方法学 | bearing-only target localization：[RTLS](https://arxiv.org/abs/2508.11289)（修 PLKF 偏置）、[PLKF + 滑窗 NLS](https://www.mdpi.com/2072-4292/17/23/3836)、[一致估计条件](https://arxiv.org/html/2507.07647v1) | 论文 REFERENCE | 滑窗已实现 |
| L2 后端滑窗 | `scipy.optimize.least_squares`（`trf` + `soft_l1` + `jac_sparsity`） | BSD | **已实现** |
| L2 后端升级 | [GTSAM](https://pypi.org/project/gtsam/) `IncrementalFixedLagSmoother`（正规 Schur 边缘化） | BSD-3 | 无，**与 C3「不新增依赖」冲突** |
| L3 检测器 | Ultralytics YOLO + NCNN 导出（ARM 最快路径） | **AGPL-3.0 ⚠** | **已在用** |
| L3 场景锚定 | [LightGlue](https://github.com/cvg/LightGlue)（E007 已复现沙盘匹配）/ XFeat | Apache-2.0 | **已是 submodule** |
| UWB 解算 | 仓库自有 `uwb/multilateration.py`（闭式线性化 + 高斯-牛顿精化，无真值入参） | — | **已实现** |
| 锚点自标定 | [INRoL](https://github.com/INRoL/inrol_uwb_localization) / [TIERS](https://github.com/TIERS/uwb-cooperative-mrs-localization)；理论见 [arXiv 2408.14081](https://arxiv.org/html/2408.14081v1)、[2412.16880](https://arxiv.org/abs/2412.16880v1) | 待核 | 无 |
| 轨迹反馈（可选） | FIM 观测性优化：[arXiv 2606.09188](https://arxiv.org/html/2606.09188v1)、[MDPI 9/9/510](https://www.mdpi.com/2313-7673/9/9/510/xml)（报条件数降 3 倍） | 论文 REFERENCE | 无 |
| 机载遥测 | `code/deployment/raspberry_pi_uav_readonly_adapter/`（只发 GCS HEARTBEAT） | — | **已进 Git** |

**明确不用的**：GDR-Net 不上机载（需 ONNX/NCNN 导出 + 量化，仓库一个都没有；且 RRD 的
bbox-only 数据训不了它），只留在地面作离线对照。**不用 RL 做估计**：问题有精确测量模型、
有解析最优解与协方差；文献里 RL 调因子图权重在 held-out 上只 `+3%`
（[Springer](https://link.springer.com/chapter/10.1007/978-981-95-4966-5_24)），而且违反
ADR-003（学习件不得直接输出目标坐标）。若要引入学习件，唯一合规位置是**学协方差**
（[arXiv 2309.09718](https://arxiv.org/html/2309.09718v1)、[2503.04933](https://arxiv.org/html/2503.04933v1)），
且必须离线训练、在线只推理，前提是 $\chi^2$ 校准图先证明手写 $\sigma$ 不校准。

## 7 运行时分层与算力预算

只有 L2 进 `100 ms` 硬预算（`10 Hz`）；检测器与场景锚定异步在 `1–2 Hz`。

| 层 | 速率 | 内容 | 核 | 预算 |
|---|---:|---|---:|---:|
| L0 | 一次 | 内参 / 外参 / 锚点 / 支撑面 / IMU 噪声 / 单位换算表 | — | 离线 |
| L1 | `188.9 Hz` IMU、`10 Hz` 位置、`1 Hz` 测距 | UWB-aided VIO | 1.2 | `35 ms` |
| L2 | `10 Hz` | 取帧+缩放 / 亚像素跟踪 / 因子构建 / 滑窗求解 / 落盘 | 1.8 | `44 ms` |
| L3 | `1–2 Hz` | 检测器 / 场景锚定 / 锚点精化 | 0.1（摊薄） | 异步 |
| — | — | **余量** | 1.0 | **`21 ms`** |

机载硬件实测：4× Cortex-A76 @ `2.4 GHz`、`7.9 GiB`（`5.3 GiB` 空闲）、
`63.7 °C`、`throttled=0x0`（无限频）。**内存不是瓶颈**（状态量 KB 级），约束是 CPU 核数。

⚠ 优化方案 3.2 的 **C6「`≤1` CPU 核」在机载跑 VIO + 检测器时不成立**，需放宽到 `≤3` 核，
或改分体部署（图像回传、地面跑重活）。这是已登记的规格冲突。

## 8 精度：2D 与 3D 的实算对比

蒙特卡洛，400 次试验，条件全部取实测硬件参数：`fx = 989.4`、`8 m` 高、`2 m` 环绕、
`100` 帧、UWB $\sigma_r = 0.03\ \mathrm{m}$ 配实测锚点布局（HDOP `1.242` / VDOP `0.845`）、
姿态 $\sigma = 0.2^\circ$、支撑面已知到 $\pm 20\ \mathrm{mm}$、视线 `0.3 px`。

| 估计器 | 水平 RMS | 垂直 RMS | 3D RMS |
|---|---:|---:|---:|
| **2D 平面求交** | **`5.68 mm`** | `19.85 mm` | `20.65 mm` |
| 3D 纯三角化 | `5.72 mm` | `16.09 mm` | `17.08 mm` |
| **3D 捆绑（视线 + 支撑面）** | `5.69 mm` | `15.28 mm` | **`16.30 mm`** |

**水平方向三种模式几乎一样，全部困难都在垂直方向。** 这就是"允许 2D"能显著降风险的原因。

### 8.1 灵敏度：哪一项才是瓶颈

逐项单独改善，看 3D 捆绑的响应：

| 改什么 | 水平 | 垂直 | 3D |
|---|---:|---:|---:|
| 基准 | `5.69` | `15.28` | `16.30` |
| **支撑面 20 → 5 mm** | `5.69` | **`4.87`** | **`7.49`** ← 最大杠杆 |
| **环绕半径 2 → 6 m** | `6.91` | **`7.85`** | **`10.46`** ← 第二大杠杆 |
| 姿态 `0.2° → 0.05°` | `4.08` | `15.01` | `15.56` |
| 平台位置噪声 ÷4 | `4.17` | `14.87` | `15.45` |
| 帧数 100 → 400 | `2.87` | `14.82` | `15.10` |
| 视线 `0.3 → 0.05 px` | `5.68` | `15.28` | `16.30` ← **零收益** |

三者组合（姿态 `0.05°` + 平台噪声 ÷4 + 支撑面 `5 mm`）：2D `1.46 mm` / 3D `5.07 mm`；
再把环绕放到 `6 m`：3D `4.71 mm`。

### 8.2 视线噪声在 ~1 px 饱和（**更正早前判断**）

姿态噪声 $0.2^\circ = 3.49\ \mathrm{mrad}$，而 $\sigma_u/f_x$ 在 `1 px` 时是 $1.01\ \mathrm{mrad}$：

| $\sigma_u$ | 折算视线 | 3D RMS |
|---:|---:|---:|
| `8.0 px` | `8.09 mrad` | `21.97 mm` |
| `4.0 px`（bbox 中心） | `4.04 mrad` | `18.30 mm` |
| `2.0 px` | `2.02 mrad` | `17.35 mm` |
| **`1.0 px`** | `1.01 mrad` | **`17.11 mm`** |
| `0.3 px` | `0.30 mrad` | `17.03 mm` |
| `0.1 px` | `0.10 mrad` | `17.03 mm` |

从 `4 px` 做到 `1 px` 值 `1.2 mm`，**值得做**；做到 `0.3 px` 以下**完全饱和、是浪费工**。
早前把亚像素跟踪当作关键杠杆是错的——在平台位置与姿态噪声面前它很快沉底。
优化方案附五.3 / 附五.7 的权重判断据此更正。

### 8.3 为什么基线必须由视觉给（不能逐帧用 UWB）

$$
\sigma_Z^{\text{baseline}} \approx Z\,\frac{\delta B}{B},
\qquad \delta B = \sqrt{2}\,\sigma_{\text{pos}}
$$

$Z = 5\ \mathrm{m}$、$B = 4\ \mathrm{m}$ 时：

| 基线来源 | $\delta B$ | 深度误差 |
|---|---:|---:|
| 逐帧 UWB（$\sigma_{\text{pos}}=30\ \mathrm{mm}$） | `42.4 mm` | **`53.0 mm`** ❌ |
| 逐帧 UWB（$45\ \mathrm{mm}$，实测布局） | `63.6 mm` | `79.5 mm` ❌ |
| 段级尺度，段内 40 组测距 | — | `12.6 mm` ⚠ |
| 段级尺度，段内 100 组测距 | — | `8.0 mm` ✅ |
| 对照：像素项 `σ_u=0.3 px` | — | `0.4 mm` |

段级尺度是标量，**必须乘在某个来源提供的段内相对几何上**——那个来源就是 VIO（F5）。
`RelativePoseFactor` 已实现但目前**没有生产者**，这是结构性缺口，不是待优化项。

### 8.4 锚点布局按实测 DOP 选定（推翻了"拉开 z"的直觉）

| 布局（相对首个 spawn，m） | PDOP | cond | $\sigma_{\text{pos}}$@$\sigma_r{=}0.03$ |
|---|---:|---:|---:|
| $(2,2,3,2)$ @ $\pm 20$ 近似共面 | `1.638` | `1.57` | `49.1 mm` |
| $(1.5,20,6,13)$ @ $\pm 20$ 拉开 z | `2.184` | `2.72` | `65.5 mm` ❌ 更差 |
| **$(\pm 13,\pm 13,1.5)$ 等高** | **`1.503`** | **`1.07`** | **`45.1 mm`** ✅ |

把某个锚点抬到接近标签自身高度时，其视线单位矢量的 $z$ 分量趋零，对高度几乎不提供信息，
反而把条件数做坏。**真正的教训是"几何要实测不要断言"**，不是"共面一定差"。

## 9 可观测性硬约束（写进实现，不许绕）

| # | 约束 | 数学表述 | 后果 |
|---|---|---|---|
| 1 | **单帧无深度** | 一条视线只给 2 个方程、3 个未知量 | 必须多视图或支撑面 |
| 2 | **单目无尺度** | $\{R, t\}$ 与 $\{R, \lambda t\}$ 观测等价 | 尺度必须外部给（UWB 段级 / IMU 激励） |
| 3 | **悬停时深度不可观测** | $B \to 0 \Rightarrow \sigma_Z \to \infty$ | **轨迹是算法的一部分**，必须环绕 |
| 4 | **`map` 基准 = 锚点** | 锚点误差是 gauge 误差，$\propto$ 常量而非 $1/\sqrt{N}$ | 必须 survey 或纳入状态 |
| 5 | **框中心不含姿态** | 单点对应，$\mathrm{rank}\,J_{R} < 3$ | 旋转写 NaN，不报旋转指标 |
| 6 | **视线不能平行支撑面** | $\lvert d_{i,z}\rvert \to 0 \Rightarrow \lambda_i$ 发散 | 下视构型天然满足，侧视要门控 |

## 10 验收与消融

每个组件都要证明自己值得存在。同一测试集、同一真值、同一评价脚本
（`code/analysis/evaluation/pose_metrics.py`），不做自由 SE(3) 对齐，无效帧计入
`availability` 分母。

| 实验 | 对照 | 判据 |
|---|---|---|
| F1(bbox) + F3 + F6 逐帧 | 基线 | 预期深度误差 `≈50 mm` 量级，用来验证 8.3 的推算 |
| **+ L1（VIO → F5）** | 上一行 | **深度误差应掉到 `10~15 mm`。掉不下来就停下重推，不许继续加东西**（ADR-008） |
| + L2 亚像素跟踪 | 上一行 | **必须实测报出 $\sigma_u$**，不许用 `0.3 px` 这个假设值交差；按 8.2 只需做到 ~`1 px` |
| + L3（F4 场景锚定） | 上一行 | 段间一致性、绝对基准误差 |
| 2D vs 3D 并行 | — | 两路 $z$ 的差值分布；超阈值比例即支撑面假设的失效率 |
| 在线 vs 批处理 | — | `latency_p95_ms ≤ 100`、`update_rate_hz ≥ 10`、`within_budget` 比例、协方差 $\chi^2$ 校准图 |

在线结果**不得**用批处理数字代替，反之亦然：两者是不同的可用性口径。

## 11 落地顺序

| 步 | 内容 | 否证点 |
|---|---|---|
| **S0** | 补两项硬件参数（`GLOBAL_VISION_POSITION_ESTIMATE` 的 `z`/姿态有效性、静置 IMU 噪声密度）。**必须走 UDP 被动抓包，不抢串口** | 姿态字段有效 ⇒ 省掉自研 AHRS |
| **S1** | 单位换算层（加速度 `mm/s²`）+ 时间同步；适配器扩输出 `SCALED_IMU`（现只处理 4 类消息、不含 IMU） | JSONL 里加速度 `÷1000` 后模长 $\approx 9.81$ |
| **S2** | 相机切 `1640×1232 @ 41.85 fps`；**支撑面与锚点 survey**；环绕半径 `2 → 6 m` | 三目标同时在画面内；$z_\Pi$ 已知到 `≤5 mm` |
| **S3** | 拉 uvio / VIR-SLAM，核 LICENSE + aarch64 编译；否则退回 OpenCV 单应平面 VO | 编得动且许可可接受 |
| **S4** | **关键否证**：VIO 接进 F5，跑消融第二行 | 深度 `≈50 → 10~15 mm`。花时间最少、否证力最强，**必须最先做** |
| S5 | OpenCV 亚像素跟踪 + 检测器降到 `1–2 Hz` + 跟踪/检测仲裁 | $\sigma_u$ 实测能否 `<1 px` |
| S6 | LightGlue 接 F4；锚点在线自标定 | 段间一致性 |
| S7 | 视 S4/S5 结果决定是否引入 GTSAM 替换 F8 先验近似 | 边缘化近似是否为瓶颈 |

**S2 里的 survey 是精度前提，不是收尾工作**：按 8.1，支撑面从 `20 mm` 测到 `5 mm`
单项就把 3D 从 `16.30` 打到 `7.49 mm`，比任何算法改动都有效。

## 12 命名与登记

| 层 | 名字 |
|---|---|
| 配置 | `config/chain_d_online_visual.yaml` |
| 算法注册表 | `vision.bundle_d`（与 `bundle_a/b/c` 同族，可并列对比） |
| 后端 | `code/analysis/localization/online_solver.py`（已实现） |
| 前端（待建） | `code/analysis/vision/online_frontend/`：`patch_tracker.py`、`detect_arbiter.py`、`vio_bridge.py` |
| 论文表述 | **"UWB 段级定标的在线视觉-惯性-测距融合目标定位"**；方法学名 **bearing-only target localization from a moving observer**，实现逐个列明所用现成件 |

**不声称自研新方法。** 本框架的贡献是系统集成、口径设计与可观测性分析。

## 13 未验证清单（不得表述为已验证）

| 项 | 原因 |
|---|---|
| 端到端在线精度 | 从未实飞；本文所有精度数字是闭式推导或蒙特卡洛（TARGET） |
| `≤10 Hz` / 核数预算 | 未在 Pi 5 上实测；预算表是 TARGET |
| `GLOBAL_VISION_POSITION_ESTIMATE` 的 `z`/姿态有效性 | 2026-09-02 复查时飞控静默（三个波特率裸读仅 4–78 字节/2.5 s，`ttyAMA10` 全 0），判定为飞控断电或 UART 断开，未采到 |
| 静置 IMU 噪声密度 | 同上 |
| 磁力计单位语义 | 与 mgauss 对不上，待厂商确认 |
| 锚点坐标 / 支撑面高度 / `base_to_camera` 外参 | **全部未实测**，是当前精度的三个真正前置 |
| `uvio` / `VIR-SLAM` / `INRoL` 的许可与 aarch64 可编译性 | 未拉 LICENSE 核对前不写进依赖 |
| Ultralytics **AGPL-3.0** 对 WebSocket 网关的影响 | 学术答辩无碍；闭源交付有碍 |
| 目标旋转 | bbox-only 下不可观测，v01 不承诺 |
| C3「不新增依赖」/ C6「`≤1` 核」 | 与引入 GTSAM / 机载跑 VIO 冲突，需决策记录裁定 |
