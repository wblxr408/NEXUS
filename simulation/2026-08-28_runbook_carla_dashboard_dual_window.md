# CARLA + WSL + Web Dashboard 联调运行手册

适用目标：

- Windows 端运行 CARLA，负责 `sandbox-v29` 的真实 GPU 渲染。
- WSL 端连接 CARLA，运行状态桥与算法/ROS2。
- Web Dashboard 左侧显示 CARLA 状态流，右侧显示 ROS2/算法结果。

## 1. Windows 启动 CARLA

PowerShell：

```powershell
cd D:\CARLA_0.9.16
.\CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low
```

说明：

- `-carla-rpc-port=2000`：WSL CARLA Python client 连接的 RPC 端口。
- `-quality-level=Low`：降低 GPU 负载，便于联调。

如果需要放行端口：

```powershell
New-NetFirewallRule -DisplayName "CARLA RPC" `
  -Direction Inbound -Action Allow -Protocol TCP -LocalPort 2000-2002
```

## 2. WSL 检查 CARLA 端口是否可达

```bash
python3 - <<'PY'
import socket
for p in (2000, 2001, 2002):
    s = socket.socket(); s.settimeout(0.5)
    try:
        s.connect(('127.0.0.1', p))
        print(p, 'OPEN')
    except OSError:
        print(p, 'CLOSED')
    finally:
        s.close()
PY
```

预期：`2000/2001/2002 OPEN`

## 3. WSL 运行 CARLA smoke test

```bash
cd /root/nexus_workspace/NEXUS
simulation/carla/.venv-carla/bin/python simulation/carla_smoke_test.py --host 127.0.0.1 --port 2000
```

关键参数：

- `--host 127.0.0.1`：Windows CARLA 在 mirrored networking 下对 WSL 可见。
- `--port 2000`：CARLA RPC 端口。

成功标志：

- 输出 `SMOKE_TEST_OK`
- 生成 `/tmp/carla_smoke_frame.png`

## 4. WSL 安装 bridge 依赖

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
pip3 install --target .venv-carla/lib/python3.12/site-packages -r requirements_bridge.txt
```

## 5. WSL 启动真实 CARLA 状态桥

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla

./.venv-carla/bin/python carla_status_bridge.py \
  --host 127.0.0.1 \
  --port 2000 \
  --map sandbox-v29 \
  --web-port 8766 \
  --spawn-demo-ego
```

关键参数：

- `--host 127.0.0.1`：CARLA RPC 主机地址。
- `--port 2000`：CARLA RPC 端口。
- `--map sandbox-v29`：bridge 会确保当前地图切换到 `sandbox-v29`。
- `--web-port 8766`：给浏览器消费的 WebSocket 端口。
- `--spawn-demo-ego`：若地图中没有现成 ego，自动生成一个 `hero` 车辆和 RGB 相机，便于 Dashboard 显示真实位姿和相机缩略图。

WebSocket 地址：

```text
ws://<WSL主机>:8766/ws/carla_status
```

## 6. 仅做链路测试时，启动 mock bridge

不依赖 CARLA：

```bash
cd /root/nexus_workspace/NEXUS/simulation/carla
./.venv-carla/bin/python carla_status_bridge.py --mock --web-port 8766
```

适合验证：

- WebSocket 是否通
- Dashboard 左侧是否刷新
- 算法/日志区是否能消费 JSON

## 7. 启动 rosbridge

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

关键参数：

- `--port 9090`：浏览器端 ROS bridge 地址。

默认地址：

```text
ws://<WSL主机>:9090
```

## 8. 启动 Web Dashboard 静态页面

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws/src/nexus_viz_dashboard/web
python3 -m http.server 8765
```

打开：

```text
http://127.0.0.1:8765/public/index.html
```

如果要显式指定 bridge 地址：

```text
http://127.0.0.1:8765/public/index.html?carla_ws=ws://127.0.0.1:8766/ws/carla_status&rosbridge=ws://127.0.0.1:9090
```

## 9. 推荐启动顺序

1. Windows：启动 `CarlaUE4.exe`
2. WSL：确认 `2000/2001/2002` 端口可达
3. WSL：运行 `carla_smoke_test.py`
4. WSL：启动 `carla_status_bridge.py`
5. WSL：启动 `rosbridge_websocket`
6. WSL：启动 Dashboard 静态服务器
7. 浏览器：打开 Dashboard

## 10. 验证命令

### 10.1 Dashboard 前端测试

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/nexus_viz_dashboard/test/test_dashboard_contract.py
```

### 10.2 ROS2 构建

```bash
cd /root/nexus_workspace/NEXUS/code/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select nexus_viz_dashboard --symlink-install
```

### 10.3 实时 CARLA WebSocket 抽查

```bash
cd /root/nexus_workspace/NEXUS
simulation/carla/.venv-carla/bin/python - <<'PY'
import asyncio, json, websockets
async def main():
    async with websockets.connect('ws://127.0.0.1:8766/ws/carla_status') as ws:
        data = json.loads(await ws.recv())
        print(json.dumps(data, ensure_ascii=False)[:1200])
asyncio.run(main())
PY
```

## 11. 常见问题

### 11.1 `127.0.0.1:2000 CLOSED`

说明 Windows CARLA 还没启动好，或者防火墙没放行。

### 11.2 地图不是 `sandbox-v29`

bridge 已带：

```bash
--map sandbox-v29
```

它会在连接后主动切换到该地图。

### 11.3 左侧有 CARLA 状态，但没有 ego / 缩略图

给 bridge 增加：

```bash
--spawn-demo-ego
```

### 11.4 浏览器连不上 WebSocket

检查：

- `8766` 是否启动了 `carla_status_bridge.py`
- `9090` 是否启动了 `rosbridge_websocket`
- URL 查询参数是否写错

## 12. 本次联调的关键地址

```text
Windows CARLA 安装目录: D:\CARLA_0.9.16
CARLA RPC:              127.0.0.1:2000
CARLA stream:           127.0.0.1:2001-2002
CARLA status WS:        ws://127.0.0.1:8766/ws/carla_status
ROS bridge WS:          ws://127.0.0.1:9090
Dashboard HTTP:         http://127.0.0.1:8765/public/index.html
```
