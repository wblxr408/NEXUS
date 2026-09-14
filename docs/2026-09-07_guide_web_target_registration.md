# 当前定位网页：输入状态与目标注册

源码：code/ros2_ws/src/nexus_viz_dashboard/web。默认使用当前双对象 ROS2 接口；历史 CARLA 数据源仅在 URL 指定 legacy=carla 时连接。

## 操作

1. 启动相机接收和定位链路，确认 /camera/image_rect 与 /camera/camera_info 有输入。
2. 打开统一启动提供的 http://127.0.0.1:8766/public/index.html。
3. 点击 **定格并指定目标**。预览是原始压缩图，定格使用算法实际接收的去畸变原图。
4. 拖动矩形框住一个静态目标；可切换 **点选参考点** 在框内点击。参考点吸附到附近特征，默认靠近框中心，不代表车辆几何中心。
5. 填写目标名称；需要沿用已有轨迹时选择“绑定已有轨迹”，再点 **确认注册**。
6. 只有后端 registered 回执才显示成功；三维坐标仍需后续有效观测。节点重启后需重新注册。
7. 左侧选择目标查看当前/历史状态，右侧查看当前坐标。丢失或过期时不把历史位置显示成当前测量。

相机内参采集仍按要求暂停。没有去畸变图像时定格会提示检查标定；本次未生成虚假标定，未启动实机或飞控。

## 启动

GPU 入口保持三个实测标定文件要求，以下路径为占位说明：

~~~bash
bash code/tools/run_gpu_localization.sh \
  platform_calibration:=/实际路径/platform.yaml \
  vision_calibration:=/实际路径/vision.yaml \
  reference_calibration:=/实际路径/reference.yaml
~~~

ROS 域默认 20。WSL 大图像通信沿用 ADR-014 的显式 dds_transport:=LARGE_DATA，上游也需相同传输设置。

仅看布局，可在仓库根目录执行：

~~~bash
python3 -m http.server 18771 --bind 127.0.0.1 --directory code/ros2_ws/src/nexus_viz_dashboard/web
~~~

打开 http://127.0.0.1:18771/public/index.html。无 ROS 输入时显示等待。

URL 的 rosbridge 参数可指定桥接地址，image_topic 和 camera_info_topic 指定与定位算法一致的输入。修改统一启动后需重启其 rosbridge 以应用注册白名单，不必重启飞控。

## 数据语义与边界

- 输入状态显示 IMU、UWB、相机参数、检测、特征运动、平台估计、固定参考和目标几何。“收到数据”仅代表接收，不等于定位有效或精度达标。
- 无人机和目标来自 /nexus/viz/localization_state，保持 map / 米、当前与历史互斥，以及输入模式/运行编号。
- 输入面板时间为浏览器收包间隔；目标年龄来自后端。曝光年龄、配对置信度、重投影误差等没有来源时保持未知。
- 页面只可发布 /nexus/vision/target_reference_image 和 /nexus/vision/target_requests，没有飞行命令。
- 当前树莓派只读遥测经 TCP 14551 进入本地，图像经 TCP 14552；接收节点发布 IMU/UWB/健康状态，不把厂商二维位置直接发布为 /nexus/fcu/odom。/nexus/platform/odom 属于本地估计输出。

## 验证限制

真实浏览器 → rosbridge → ROS2 → CUDA SuperPoint 注册已通过，使用合成图像和测试标定，不代表实机精度。网页/启动包检查通过；完整视觉回归另有 6 个失败用例，在本次 QoS 修改前的节点上也复现，未在网页任务中修复。详见 [E027](../experiments/runs/2026-09-07_E027_web_target_registration/run.md)。
