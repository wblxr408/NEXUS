export const INPUT_TOPICS = [
  { key: "imu", label: "飞控 IMU", topic: "/nexus/fcu/imu", type: "sensor_msgs/msg/Imu" },
  { key: "uwb", label: "UWB 测距", topic: "/nexus/uwb/ranges", type: "std_msgs/msg/String" },
  { key: "uwb_mean", label: "UWB 静置均值", topic: "/nexus/uwb/startup_statistics", type: "std_msgs/msg/String" },
  { key: "initialization", label: "近似位姿初始化", topic: "/nexus/platform/initialization_status", type: "std_msgs/msg/String" },
  { key: "camera_info", label: "相机参数", topic: "/camera/camera_info", type: "sensor_msgs/msg/CameraInfo" },
  { key: "detector", label: "YOLO11 检测", topic: "/nexus/vision/detections", type: "std_msgs/msg/String" },
  { key: "features", label: "SuperPoint 运动", topic: "/nexus/vision/body_motion", type: "std_msgs/msg/String" },
  { key: "platform", label: "无人机估计", topic: "/nexus/platform/status", type: "std_msgs/msg/String" },
  { key: "reference", label: "固定视觉参考", topic: "/nexus/vision/reference_status", type: "std_msgs/msg/String" },
  { key: "geometry", label: "目标几何", topic: "/nexus/vision/target_geometry_status", type: "std_msgs/msg/String" },
];
export function inputSummary(spec, message) {
  if (spec.type === "std_msgs/msg/String") {
    const value = JSON.parse(message.data);
    if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("invalid status");
    if (spec.key === "uwb_mean") return { accepted: !value.stale,
      detail: `${value.window_samples}/${value.window_capacity} 组 · ${value.stale ? "数据中断" : (value.ranges_m || []).map(v => Number.isFinite(v) ? v.toFixed(3) + " m" : "—").join(" / ")}` };
    return { accepted: value.valid !== false,
      detail: (value.calibration_status === "experimental_approximation" ? "近似标定 · " : "") + String(value.reason || value.state || (spec.key === "detector" ? (value.detections?.length ?? 0) + " 个候选目标" : "收到数据")) };
  }
  if (spec.key === "camera_info") {
    const calibrated = message.k?.[0] > 0 && message.k?.[4] > 0 && message.width > 0 && message.height > 0;
    return { accepted: calibrated, detail: calibrated ? message.width + " × " + message.height + " · 已收到内参" : "内参缺失或无效" };
  }
  return { accepted: true, detail: "收到加速度与角速度消息" };
}
