# 下视 Gazebo 相机与 Web 预览记录

日期：2026-08-26  
范围：Gazebo IMX219 相机、rosbridge、Dashboard 相机窗口。

## 结论

`nexus_uav` 的 IMX219 相机位于机身下方，SDF 位姿为
`0 0 -0.06 0 1.57079632679 0`。Gazebo Classic 相机的本地光轴为 `+x`，该 `+90°`
pitch 将光轴旋转至世界 `-z`，所以相机观察无人机下方的沙盘。

相机插件 `libgazebo_ros_camera.so` 发布原始图像到
`/nexus/camera/imx219/image_raw`。Dashboard 的 rosbridge 客户端订阅该话题、将 `rgb8`/
`bgr8`/`mono8` 图像缩小为浏览器预览，并绘制到 `cam-canvas` 的“相机成像与检测证据”窗口。

## 验证

- 无 GUI 启动 Gazebo 后，实收 IMX219 图像：`3280 × 2464`、`rgb8`、步长 `9840`。
- 画面抽样像素包含 28 个不同值（范围 5–230），验证的是渲染出的沙盘画面而非空白/单色帧。
- `colcon test --packages-select nexus_bringup nexus_viz_dashboard` 已覆盖相机朝下位姿、图像话题与前端预览契约。

未进行真实硬件相机标定；本记录只证明 Gazebo 和浏览器预览链路。
