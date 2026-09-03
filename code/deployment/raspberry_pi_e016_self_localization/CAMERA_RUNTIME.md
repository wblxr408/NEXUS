# IMX219 实机相机输入

该接口把树莓派 CSI IMX219 的实时帧交给视觉推理程序，不打开飞控串口、不发送 MAVLink
数据，也不控制无人机。

启动前请拆桨或锁桨。以下命令持续更新 `outputs/live_camera/latest.jpg`（RGB）和匹配的
`latest.json`，不会保存视频或逐帧累积原始图像：

```bash
cd ~/NEXUS/code/deployment/raspberry_pi_e016_self_localization
./scripts/run_live_camera.sh --fps 15
```

用有限帧数验证接入：

```bash
./scripts/run_live_camera.sh --frames 10
```

GDR-Net 推理器应读取 `latest.jpg` 作为 RGB 输入，并从 `latest.json` 取得图像尺寸、相机
时间戳和旋转信息。真实位姿推理前必须为当前 `640x480`、`rotation=180` 工作模式提供实测
内参 `K`、畸变参数，以及 `camera -> body` 外参；当前仓库没有可在树莓派执行的 GDR-Net
权重或推理环境，因此相机接通不等同于已得到真实位姿。
