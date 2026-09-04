# E024：树莓派只读接入与跨设备录包

日期：2026-09-04  
状态：VALIDATED（实机相机 + 合成飞控遥测）

## 环境

- 树莓派 5：`192.168.1.143`，IMX219，新增目录 `/home/pi/nexus_readonly_stack`
- 本机：WSL Ubuntu 22.04、ROS2 Humble
- 遥测端口：TCP 14551；相机端口：TCP 14552
- ROS Domain：测试使用 49–51；旧 Domain 文件未修改

## 执行与结果

1. 树莓派新增部署单测：3 passed。
2. 本机旧适配器 + 新部署测试：17 passed，包含无控制 API、每条 IMU 转发、
   不完整 UWB 包清空历史值等契约。
3. `colcon build --packages-select nexus_pi_readonly_ingress`：通过。
4. `colcon test --packages-select nexus_pi_readonly_ingress`：4 passed。
5. 实机 IMX219 本地采集：1640×1232，5 帧，SensorTimestamp 存在。
6. 实机 IMX219 跨设备 ROS2：raw、compressed、CameraInfo、metadata、health 均收到；
   本次观测发布约 2.07 fps，尚未做性能优化。
7. 树莓派合成遥测跨设备 ROS2：IMU、UWB、health 均收到；标签 2 与基站 1–4 正确。
8. 7 秒 rosbag 回环：七个主题全部落盘，计数见 `results/rosbag_counts.json`。
9. 树莓派 3 秒只读记录：相机 68 帧并落盘；真实飞控无电，遥测记录为 offline。
10. Windows 限定 UDP 防火墙规则安装尝试因当前进程非管理员而失败；最终采用本机主动
    连接树莓派 TCP 的方式完成跨设备验证，无需更改防火墙。
11. metadata 接口加入后，以同一 Pi v2 线协议做本机确定性回环：8 个现行话题全部
    落盘；IMU 1077、UWB 5、相机四路各 10，证明 UWB 约 1 Hz 去重和 metadata 记录契约，
    见 `results/rosbag_v2_contract_counts.json`。该项为软件协议验证，不冒充实机输入。

## 边界

- 遥测值来自合成发送器，只验证传输、消息、单位和时间处理，不证明真实飞控数据正确。
- 相机是实机数据，但没有内参、外参和目标标注，不证明目标定位精度。
- 真实飞控供电后仍需补 IMU/UWB 静态录包及已知点检查。
- 原始图像和 rosbag 保留在设备或 `/tmp`，未提交仓库。
