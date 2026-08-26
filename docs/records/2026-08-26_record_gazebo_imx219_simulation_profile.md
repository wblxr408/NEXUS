# Gazebo IMX219 仿真配置记录

日期：2026-08-26  
类型：配置与外部资料记录

## 结论

Gazebo Classic 11 已在开发环境中验证可运行，ROS2 Humble 的 `libgazebo_ros_camera.so` 和
`libgazebo_ros_p3d.so` 已安装。`simulation/nexus_sandbox_imx219.world` 由与 Web JSON 相同的
`simulation/sandbox_scene.yaml` 生成，因此 Gazebo、Web 沙盘的物体坐标和单位均为米制 map
坐标。浏览器经 rosbridge 显示 Gazebo 原始相机预览和模拟无人机位置；它不把仿真输出标记为
实测定位结果。

## 使用的参数与边界

`docs/IMX219.md` 是本次仿真配置的项目内来源：`3280×2464`、`79.3°`、`21 Hz`、`R8G8B8`。
Gazebo Classic 的 camera SDF 可配置图像宽高、`horizontal_fov`、更新率、裁剪距离和像素格式，
因此生成器将文档中的未限定 `79.3°` 明确解释为 **仿真的水平视场角**；实测 Gazebo 发布的
`CameraInfo` 为 `fx=fy=1978.892939 px, cx=1640.5 px, cy=1232.5 px`。此解释只服务当前仿真，
不能替代相机标定。

Gazebo Classic 相机不模拟 IMX219 的滚动快门逐行曝光、F2.0 光圈、镜头焦距、传感器噪声、自动
曝光/增益或真实畸变。因此这些项目没有被伪装为物理保真参数；实际 `K/D/R/P` 仍必须以同一
分辨率、裁切和焦点状态的标定为准。

## 联网核对

2026-08-26 查阅 Raspberry Pi 官方文档仓库（Camera Module 2 使用 Sony IMX219，分辨率
3280×2464）：

- [Camera Module 2 source](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/accessories/camera/cm2.adoc), fetched blob SHA `05696699ec03ecffc24e4f7bd62fdaa28b245f4f`
- [hardware specification source](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/accessories/camera/hardware_specification.adoc), fetched blob SHA `70e8720013a701c9f9241eec8c2eab0e1aa075a0`

官方 Raspberry Pi Camera Module 2 参考模块的镜头参数为 3.04 mm、水平/垂直视场角
62.2°/48.8°、F2.0、可调焦、约 10 cm 至无穷远、最大曝光 11.76 s、传感器图像面积
3.68×2.76 mm。它和项目 `IMX219.md` 中 2.85 mm/79.3° 的常见模组参数不一致，说明 **IMX219
只确定传感器，不确定镜头和模组**；CSV 已同时标注仿真值、文档值和真实设备待确认项。

## 验证命令

```bash
python3 simulation/generate_sandbox.py --scene simulation/sandbox_scene.yaml --out simulation
gzserver --verbose -s libgazebo_ros_init.so simulation/nexus_sandbox_imx219.world
ros2 topic list | rg '/nexus/(camera/imx219|gazebo/uav/odom)'
```

本记录仅说明仿真与接口可用性，不构成真实相机、视觉算法或定位精度的实验结论。

## 本次验证结果

环境为 Ubuntu 22.04、ROS2 Humble、Gazebo Classic 11.10.2。生成器成功生成 486 个沙盘对象（该数
量随当前沙盘 YAML 变化）。启动 world 后已实际收到：

- `/nexus/camera/imx219/camera_info`：`3280×2464`、`plumb_bob`、`D=[0,0,0,0,0]`，以及上述仿真内参；
- `/nexus/camera/imx219/image_raw`；
- `/nexus/gazebo/uav/odom`：`world` 中位置 `(2.0, 2.35, 2.5) m`。

执行 `colcon build --packages-select nexus_bringup`，以及 `colcon test --packages-select nexus_bringup`
中 `test_launch_contract`、`test_gazebo_contract` 均通过。开发环境未安装 Node.js，因而浏览器端
JavaScript 未进行 Node 语法检查；已按 rosbridge v2 的 `uint8[]` Base64 协议实现，待有浏览器和
rosbridge_server 的联调环境时进行端到端页面验收。
