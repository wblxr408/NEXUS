# CARLA Windows 0.9.16 WSL 冒烟运行记录

## 配置

- CARLA Windows 包：`CARLA_0.9.16.zip`
- 解压目录：`D:\CARLA_0.9.16`
- 服务端命令：`CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low`
- 服务端地图：`Carla/Maps/Town10HD_Opt`
- WSL Python：3.12.14
- CARLA wheel：0.9.16（`simulation/carla/.venv-carla`）
- WSL 网络：mirrored（`wslinfo --networking-mode` 输出 `mirrored`）
- 本次联调地址：自动探测得到 `127.0.0.1:2000`

## 命令与结果

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
./.venv-carla/bin/python ../carla_smoke_test.py
./.venv-carla/bin/python ../carla_smoke_test.py --host localhost --port 2000
```

结果：

```text
client wheel version: 0.9.16
server version: 0.9.16
CARLA versions match: 0.9.16
current map: Carla/Maps/Town10HD_Opt
spawned vehicle: vehicle.audi.a2
ticked 20 frames; saved camera frame id: 11316
SMOKE_TEST_OK
```

输出图像：`/tmp/carla_smoke_frame.png`（640×480 PNG，SHA-256
`0e78b740cbe666a16fb727ef4198318be1fdb074dbb25549917f0945d543ff03`）。

Windows 防火墙规则已核验：`CARLA RPC` 为 Enabled/Inbound/Allow，TCP 端口
`2000-2002`。服务端进程命令行包含 `-carla-rpc-port=2000 -quality-level=Low`，
并已核验 `0.0.0.0:2000`、`:2001`、`:2002` 均处于 Listen 状态。

## 验证边界

本次成功证明 Windows CARLA 服务端与 WSL 客户端链路、车辆/相机生成和同步 tick
可用；测试结束后服务端恢复为异步模式。该结果不代表定位精度达标。ORB-SLAM2、
GDR-Net、UWB 接入属于后续独立实验，本运行未涉及。
