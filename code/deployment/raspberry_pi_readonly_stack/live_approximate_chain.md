# 近似标定全链路运行

地面电脑 WSL，在仓库根目录运行：

```bash
bash code/tools/run_live_approximate_localization.sh
```

浏览器打开 `http://localhost:8766`，ROS bridge 为 `ws://localhost:9090`。默认 ROS_DOMAIN_ID=20；Pi 飞控桥接也使用该域。新入口包含相机接收、IMU 转发、测距统计、初始化、平台/目标计算和网页服务，不要同时运行另一套同名入口。

当前入口使用飞控实时姿态与当前 UWB 测距近似初始化，不要求识别时静止。200 组统计独立积累，约 1 Hz 时需 3 分 20 秒，不作为运动时刻的瞬时测距。网页应出现平台坐标。定格定位图像、框选目标并确认注册，随后改变观察视角以提供几何基线；目标有效时显示 map 坐标，单位米。没有足够基线时暂不输出有效位置是预期行为。

默认配置为 nexus_bringup/config/platform_live_approximate.yaml 和 vision_live_approximate.yaml。近似内容包括初始绝对 heading、相机朝下安装旋转与共点平移；初始 yaw 的 9° 偏置不等于实测绝对 heading。它们适用于运行验证，不代表厘米级定位精度。

## Pi 端前置

既有 fcu_ros2 容器和 fcu_bridge_001 需运行，并提供 /imu_global_001 和 /nexus/uwb/ranges。该入口不改飞控启动方式，也不发送飞行指令。

相机需运行已有 `/home/pi/nexus_readonly_stack/camera_stream_server.py`，同时提供图像端口 14552 和时钟端口 14553。随本次代码提供 nexus_camera_stream.service 自启动模板，供上电后安装；E047 已安装并验证服务运行，完整断电重启尚未验证。安装前先确认没有另一进程占用相机。Pi 上手动启动现有服务的命令是：

```bash
cd /home/pi/nexus_readonly_stack
python3 camera_stream_server.py --width 1640 --height 1232 --fps 10 --rotation 180
```

不要启动另一个飞控串口采集器。机载 E043 的 ranges_smoothed 服务可保留，地面初始化使用单独 startup_statistics 输出，避免同话题重复发布。

## 诊断与录制

网页输入状态显示 IMU、原始距离、200 组均值、近似初始化、相机、识别和几何状态。也可用：

```bash
ros2 topic echo /nexus/platform/initialization_status
ros2 topic echo /nexus/target/kinematics
```

在已加载 ROS 环境且同一 ROS_DOMAIN_ID 的终端执行。目标有效条件是 valid=true、position_observed=true、frame_id=map、unit=m；历史坐标不能算当前识别成功。

需要记录时给入口传入 `record:=true run_id:=本次ASCII运行名 record_output:=全新目录`；不覆盖已有目录。停掉地面 launch 不会停止或控制飞控。


## 2026-09-08 E047 实机恢复修订

当前一键入口默认网页 `http://127.0.0.1:8766/?rosbridge=ws%3A%2F%2F127.0.0.1%3A9091`。
9091 用于避免本机既有 9090 服务冲突；仍可在启动命令末尾覆盖 `rosbridge_port:=...`。
地面 ROS 使用 domain 20、localhost-only；跨机传感器走只读 TCP 14554，相机走 TCP 14552，时钟探测走 UDP 14553。
`fastdds_pi_peer.xml` 是本轮保留的部署文件名，最终内容只用于地面本机发现和共享内存传输，不再依赖跨机 DDS 自动发现。

树莓派服务为 `nexus-fcu-bridge.service`、`nexus-fcu-stream.service`、`nexus-uwb-smoothing.service` 和 `nexus_camera_stream.service`。
桥接使用原 argv 与持久化参数文件，保持 channel=0、offboard=false、set_goal=false；没有启动 mission 节点。
原始串口仍只有该桥接使用。服务已启用，桥接托管启动及真实 IMU/UWB 输出已验证；整机断电冷启动尚未验证。

地面入口把 Pi ROS Unix 时间转换为地面 Unix 时间，采用四时间戳探测及缓慢调整偏移，保留原消息于 `/nexus/pi/imu_raw`、`/nexus/pi/ranges_raw`。
对齐后分别发布 `/nexus/pi/imu_aligned`、`/nexus/pi/ranges_aligned`；这只解决两台计算机的时钟差，不声称恢复飞控硬件采样时刻。
有效探测过期、输入断流和无效时间仍会拒收。时钟、源接收计数与转换计数在 `/nexus/pi/fcu_clock_status` 中可查。

用户确认手持模拟飞行后，当前配置改用飞控实时姿态与当前 UWB 数据初始化；不把转动角速度当作零偏。零偏先验来自 E047 已采集的静置数据。静置 200 组初始化保留为非默认模式并有回归测试。
定位进程重启/IMU 时间断续会请求重新初始化。50 Hz 是预测发布定时器设置，实机实际输出率必须以采集为准（E047 一段 45 秒实测约 22.5 Hz）。
预测不会添加虚构测量，超过外部约束有效期停止输出。200 组统计均值仍不是移动时刻的瞬时距离。

实测与失败记录见 `experiments/runs/2026-09-08_E047_recovery_and_pose_stream/`。
目标需持续出现在相机视野内，并改变观察位置形成基线；不要求飞行识别时静止。近似结果不能作为精度达标材料。

飞控姿态只读流同时保留 `/nexus/pi/odom_raw` 与时间对齐的 `/nexus/pi/odom_aligned`。初始化只使用其姿态，叠加已有 9° 地图偏置；不直接把飞控自身位置当作本项目 map 坐标。飞控姿态的轴向转换已由桥接源代码核对，绝对航向精度仍未实测。
