# CARLA 0.9.16 Windows 联调

当前 CARLA 服务端路线是 Windows 包；WSL 中只运行 Python 客户端冒烟测试。
`simulation/carla/run_carla_server.sh` 保留作历史记录，Linux 软件渲染路线已弃用，
不要再用它启动服务端。

## Windows 服务端

1. 打开 [GitHub CARLA 0.9.16 release](https://github.com/carla-simulator/carla/releases/tag/0.9.16)，
   下载其中的 Windows 包 `CARLA_0.9.16.zip`（发行页可能将大文件跳转到 CARLA CDN），
   解压到 D 盘，例如 `D:\CARLA_0.9.16`。
2. 在该目录启动低画质服务端（默认加载 Town10HD）：

   ```powershell
   .\CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low
   ```

   服务端启动后应能看到 Town10HD（发行包中通常显示为
   `Carla/Maps/Town10HD_Opt`），并监听 RPC 端口 `2000` 以及流式端口 `2001-2002`。
3. 以管理员身份放行一次防火墙端口：

   ```powershell
   New-NetFirewallRule -DisplayName "CARLA RPC" `
     -Direction Inbound -Action Allow -Protocol TCP -LocalPort 2000-2002
   ```

## WSL 网络

在 Windows 用户配置文件 `%UserProfile%\.wslconfig` 中加入：

```ini
[wsl2]
networkingMode=mirrored
```

保存后在 PowerShell 执行 `wsl --shutdown`，再重新打开 WSL。镜像网络下客户端
默认探测 `127.0.0.1`；NAT 网络下会读取 `/proc/net/route` 的默认网关。也可以用
`CARLA_HOST` 或 `--host` 明确指定地址，命令行参数优先于环境变量。

## WSL 冒烟测试

仓库已提供匹配 CARLA 0.9.16 wheel 的 `.venv-carla`，无需重装：

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
./.venv-carla/bin/python ../carla_smoke_test.py
```

需要显式地址时：

```bash
./.venv-carla/bin/python ../carla_smoke_test.py --host 127.0.0.1 --port 2000
# NAT 示例：--host 172.20.16.1
```

成功标准是 client/server 版本一致，脚本生成 `/tmp/carla_smoke_frame.png`，完成
车辆与相机生成、20 帧 tick，并打印 `SMOKE_TEST_OK`。这只证明 Windows CARLA
服务端与 WSL 客户端链路可用，不代表定位精度已达标；通过后再开展 ORB-SLAM2 /
GDR-Net / UWB 接入实验。

## WSL Dashboard bridge

在同一个 CARLA Python 虚拟环境安装 WebSocket 依赖：

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
pip3 install --target .venv-carla/lib/python3.12/site-packages -r requirements_bridge.txt
```

启动真实 CARLA 状态桥（Windows CARLA 已监听 `2000` 且当前地图为 `sandbox-v29`）：

```bash
./.venv-carla/bin/python carla_status_bridge.py \
  --host 127.0.0.1 --port 2000 --map sandbox-v29 --web-port 8766
```

如果 `sandbox-v29` 当前没有 ego 车辆，需要让 Dashboard 同时显示真实 CARLA 相机缩略图，
加上 `--spawn-demo-ego`。bridge 会在地图首个 spawn point 创建一个 `hero` 车辆和 RGB 相机；
退出 bridge 时会自动清理这两个临时 actor：

```bash
./.venv-carla/bin/python carla_status_bridge.py \
  --host 127.0.0.1 --port 2000 --map sandbox-v29 \
  --web-port 8766 --spawn-demo-ego
```

如果 WSL 使用 NAT 网络，把 `--host` 换成 Windows 主机在 WSL 中可达的网关地址。
浏览器访问 Dashboard 时默认连接 `ws://<WSL主机>:8766/ws/carla_status`；若端口不同，
在页面 URL 加 `?carla_ws=ws://<WSL主机>:<端口>/ws/carla_status`。

不启动 CARLA 也可以先验证 WebSocket/UI 链路：

```bash
./.venv-carla/bin/python carla_status_bridge.py --mock --web-port 8766
```

该模式会持续发送 `sandbox-v29`、ego 位姿、actor 数量、算法占位状态和日志，
但不会伪造真实 CARLA 渲染或定位精度。
