# CARLA 0.9.16 Windows + sandbox-v29 + WSL + Web Dashboard 部署全过程

本文覆盖从下载 CARLA Windows 压缩包、解压和合并定制地图，到 Windows GPU 渲染、WSL CARLA client、WebSocket bridge、ROS2、rosbridge 和 Dashboard 联调。

最终链路：

    Windows CarlaUE4.exe
      真实 GPU 渲染 sandbox-v29
        RPC 2000 / stream 2001-2002
          WSL carla_status_bridge.py
            WebSocket 8766
              Dashboard 左侧 CARLA 数据流

    WSL ROS2 + rosbridge 9090
      Dashboard 右侧相机、算法、健康状态和实验指标

说明：联调通过只代表通信、渲染和展示链路打通，不代表定位精度达标。

## 1. 下载 CARLA

1. 在 Windows 浏览器打开：

       https://github.com/carla-simulator/carla/releases/tag/0.9.16

2. 下载 Windows 包 CARLA_0.9.16.zip。
3. 保存到 D:\Downloads\CARLA_0.9.16.zip。

如果 Release 页面跳转到 CARLA CDN，保持默认下载。

## 2. 解压到 D 盘

PowerShell：

    New-Item -ItemType Directory -Force D:\CARLA_0.9.16 | Out-Null
    Expand-Archive -Path D:\Downloads\CARLA_0.9.16.zip -DestinationPath D:\CARLA_0.9.16 -Force

检查：

    Test-Path D:\CARLA_0.9.16\CarlaUE4.exe
    Test-Path D:\CARLA_0.9.16\CarlaUE4\Content\Carla

两个命令都必须返回 True。如果解压后出现 D:\CARLA_0.9.16\CARLA_0.9.16\CarlaUE4.exe，说明多了一层目录。把内层文件整体移动到 D:\CARLA_0.9.16。

## 3. 合并定制地图 sandbox-v29

仓库源目录：

    /root/nexus_workspace/NEXUS/simulation/carla/CarlaUE4/Content/CustomMaps/sandbox-v29

Windows 目标目录：

    D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29

必须完整包含：

    sandbox-v29.umap
    sandbox-v29.uexp
    OpenDrive\sandbox-v29.xodr
    Static\

### 3.1 PowerShell 通过 WSL 共享目录复制

先查看发行版名称：

    wsl.exe -l -q

假设发行版名称是 Ubuntu：

    $repoMap = "\\wsl$\Ubuntu\root\nexus_workspace\NEXUS\simulation\carla\CarlaUE4\Content\CustomMaps\sandbox-v29"
    $winMap  = "D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29"
    New-Item -ItemType Directory -Force $winMap | Out-Null
    robocopy $repoMap $winMap /E /COPY:DAT /R:2 /W:1
    if ($LASTEXITCODE -gt 7) { throw "robocopy failed: $LASTEXITCODE" }

如果发行版不是 Ubuntu，把路径中的名称改成实际名称。

### 3.2 备用：WSL 先复制到 D 盘

WSL：

    cp -a /root/nexus_workspace/NEXUS/simulation/carla/CarlaUE4/Content/CustomMaps/sandbox-v29 /mnt/d/Downloads/

PowerShell：

    $src = "D:\Downloads\sandbox-v29"
    $dst = "D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29"
    New-Item -ItemType Directory -Force $dst | Out-Null
    robocopy $src $dst /E /COPY:DAT /R:2 /W:1
    if ($LASTEXITCODE -gt 7) { throw "robocopy failed: $LASTEXITCODE" }

### 3.3 校验地图

    $map = "D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29"
    Get-Item "$map\sandbox-v29.umap"
    Get-Item "$map\sandbox-v29.uexp"
    Get-Item "$map\OpenDrive\sandbox-v29.xodr"
    (Get-ChildItem $map -Recurse -File).Count

不要只复制 umap；uexp、OpenDrive 和 Static 必须一起复制。

## 4. 配置 WSL 网络

推荐编辑 Windows 文件 %UserProfile%\.wslconfig，写入：

    [wsl2]
    networkingMode=mirrored

然后执行：

    wsl --shutdown

重新打开 Ubuntu。mirrored 模式通常使用 127.0.0.1。

NAT 模式执行：

    ip route | awk '/default/ {print $3; exit}'

把输出地址作为 CARLA bridge 的 --host。

## 5. Windows 启动 CARLA

    cd D:\CARLA_0.9.16
    .\CarlaUE4.exe -carla-rpc-port=2000 -quality-level=Low

必要时管理员 PowerShell 放行：

    New-NetFirewallRule -DisplayName "CARLA RPC" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 2000-2002

## 6. WSL 检查 CARLA 端口

    python3 - <<'PY'
    import socket
    for port in (2000, 2001, 2002):
        sock = socket.socket(); sock.settimeout(0.5)
        try:
            sock.connect(("127.0.0.1", port)); print(port, "OPEN")
        except OSError:
            print(port, "CLOSED")
        finally:
            sock.close()
    PY

预期：2000、2001、2002 都是 OPEN。

## 7. WSL CARLA smoke test

    cd /root/nexus_workspace/NEXUS
    simulation/carla/.venv-carla/bin/python simulation/carla_smoke_test.py --host 127.0.0.1 --port 2000

成功标志是 SMOKE_TEST_OK，并生成 /tmp/carla_smoke_frame.png。

## 8. 安装并启动 CARLA 状态 bridge

    cd /root/nexus_workspace/NEXUS/simulation/carla
    pip3 install --target .venv-carla/lib/python3.12/site-packages -r requirements_bridge.txt
    .venv-carla/bin/python -c 'import carla, websockets; print("BRIDGE_DEPS_OK")'

启动真实 bridge：

    .venv-carla/bin/python carla_status_bridge.py --host 127.0.0.1 --port 2000 --map sandbox-v29 --listen 0.0.0.0 --web-port 8766 --spawn-demo-ego

参数：

- --host：Windows CARLA 主机地址。
- --port：CARLA RPC 端口。
- --map：目标地图，必须是 sandbox-v29。
- --listen：bridge 监听地址。
- --web-port：Dashboard 状态 WebSocket 端口。
- --spawn-demo-ego：无现成 ego 时生成 hero 车辆和 RGB 相机。

状态流地址：ws://<WSL主机>:8766/ws/carla_status

## 9. 启动 ROS2 和 rosbridge

    cd /root/nexus_workspace/NEXUS/code/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -c 'from nexus_msgs.msg import TargetObservation; print("NEXUS_MSGS_OK")'
    ros2 run rosbridge_server rosbridge_websocket --port 9090

ROS2 节点应发布：

    /nexus/target/pose
    /nexus/vision/map_target_observation
    /nexus/uwb/target_observation
    /nexus/gazebo/uav/odom
    /nexus/camera/imx219/image_raw/compressed

## 10. 启动 Web Dashboard

    cd /root/nexus_workspace/NEXUS/code/ros2_ws/src/nexus_viz_dashboard/web
    python3 -m http.server 8765

浏览器打开：

    http://127.0.0.1:8765/public/index.html

显式指定两个 WebSocket：

    http://127.0.0.1:8765/public/index.html?carla_ws=ws://127.0.0.1:8766/ws/carla_status&rosbridge=ws://127.0.0.1:9090

## 11. 推荐启动顺序

1. 合并 sandbox-v29 到 D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29。
2. 启动 Windows CarlaUE4.exe。
3. 检查 2000、2001、2002。
4. 运行 smoke test。
5. 启动 carla_status_bridge.py。
6. source ROS2 环境。
7. 启动 rosbridge。
8. 启动算法和传感器 ROS2 节点。
9. 启动 Dashboard HTTP 服务。
10. 浏览器打开 Dashboard。

## 12. 分层验收

CARLA RPC：

    simulation/carla/.venv-carla/bin/python - <<'PY'
    import carla
    client = carla.Client("127.0.0.1", 2000); client.set_timeout(5.0)
    world = client.get_world()
    print(world.get_map().name)
    print("actors:", len(world.get_actors()))
    PY

地图名必须包含 sandbox-v29。

状态 WebSocket：

    simulation/carla/.venv-carla/bin/python - <<'PY'
    import asyncio, json, websockets
    async def main():
        async with websockets.connect("ws://127.0.0.1:8766/ws/carla_status") as ws:
            data = json.loads(await ws.recv())
            assert data["carla_connected"] is True
            assert "sandbox-v29" in data["map"]
            print(json.dumps(data, ensure_ascii=False)[:1200])
    asyncio.run(main())
    PY

使用 --spawn-demo-ego 时还应看到 ego_pose、rgb_front 和 thumbnail。

ROS2 测试和构建：

    cd /root/nexus_workspace/NEXUS/code/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m pytest -q src/nexus_viz_dashboard/test/test_dashboard_contract.py
    colcon build --packages-select nexus_viz_dashboard --symlink-install

页面验收：

- CARLA 窗口能看到 sandbox-v29 真实场景。
- Dashboard 左侧为 CONNECTED。
- 地图名包含 sandbox-v29。
- frame、actor、交通灯数据持续更新。
- 使用 --spawn-demo-ego 时显示 ego 位姿和 CARLA 相机缩略图。
- ROS2 节点发布后，右侧相机、UWB、Vision、Fusion 和延迟区更新。

## 13. 常见故障

- CarlaUE4.exe 找不到：检查是否多解压了一层目录。
- 地图加载失败：检查 umap、uexp、OpenDrive、Static 是否完整复制。
- 127.0.0.1:2000 CLOSED：检查 Windows CARLA、防火墙和 WSL 网络。
- bridge 为 carla_connected=false：检查 --host、--port，并先跑 smoke test。
- 没有 ego 或缩略图：bridge 增加 --spawn-demo-ego。
- Dashboard 连不上：确认 8766、9090 正在监听，且 URL 参数正确。

## 14. 关键地址

    Windows CARLA 根目录: D:\CARLA_0.9.16
    定制地图目录: D:\CARLA_0.9.16\CarlaUE4\Content\CustomMaps\sandbox-v29
    CARLA RPC: 127.0.0.1:2000
    CARLA stream: 127.0.0.1:2001-2002
    CARLA status WS: ws://127.0.0.1:8766/ws/carla_status
    ROS bridge WS: ws://127.0.0.1:9090
    Dashboard HTTP: http://127.0.0.1:8765/public/index.html
