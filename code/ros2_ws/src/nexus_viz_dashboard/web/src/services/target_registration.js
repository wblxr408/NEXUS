// Explicit target registration only. This client cannot publish flight commands.
export const REFERENCE_TOPIC = "/nexus/vision/target_reference_image";
export const REQUEST_TOPIC = "/nexus/vision/target_requests";
export const REQUEST_STATUS_TOPIC = "/nexus/vision/target_request_status";

export function frameStamp(frame) {
  const stamp = frame?.header?.stamp;
  if (!Number.isInteger(stamp?.sec) || !Number.isInteger(stamp?.nanosec)
      || stamp.sec < 0 || stamp.nanosec < 0 || stamp.nanosec >= 1e9
      || !frame.header.frame_id) throw new Error("图像缺少有效的采样时间或相机坐标系");
  const ns = BigInt(stamp.sec) * 1000000000n + BigInt(stamp.nanosec);
  if (ns <= 0n) throw new Error("图像采样时间无效");
  return ns.toString();
}

export function imagePixels(frame) {
  frameStamp(frame);
  const channels = { rgb8: 3, bgr8: 3, mono8: 1 }[frame.encoding];
  const { width, height, step } = frame;
  if (!channels || !Number.isInteger(width) || !Number.isInteger(height)
      || width < 8 || height < 8 || !Number.isInteger(step) || step < width * channels)
    throw new Error("暂不支持该图像格式或尺寸");
  let bytes;
  if (typeof frame.data === "string") bytes = Uint8Array.from(atob(frame.data), c => c.charCodeAt(0));
  else if (Array.isArray(frame.data)) bytes = Uint8Array.from(frame.data);
  else throw new Error("图像像素缺失");
  if (bytes.length !== step * height) throw new Error("图像数据长度与尺寸不一致");
  return { bytes, channels };
}

// Coordinates refer to the original ROS image, never CSS or preview pixels.
export function imagePoint(clientX, clientY, rect, width, height) {
  return [
    Math.max(0, Math.min(width - 1, (clientX - rect.left) * width / rect.width)),
    Math.max(0, Math.min(height - 1, (clientY - rect.top) * height / rect.height)),
  ];
}
export function selectionBox(first, second) {
  const x = Math.floor(Math.min(first[0], second[0])), y = Math.floor(Math.min(first[1], second[1]));
  return [x, y, Math.ceil(Math.max(first[0], second[0])) - x, Math.ceil(Math.max(first[1], second[1])) - y];
}
export function registrationJson(frame, box, targetId, requestId, anchor = null, existingId = null) {
  const stamp = frameStamp(frame);
  if (!targetId?.trim() || !requestId) throw new Error("请填写目标名称");
  if (!Array.isArray(box) || box.length !== 4 || !box.every(Number.isFinite)
      || box[0] < 0 || box[1] < 0 || box[2] < 4 || box[3] < 4
      || box[0] + box[2] > frame.width || box[1] + box[3] > frame.height)
    throw new Error("请在图像内框选目标，框不能小于 4 × 4 像素");
  if (anchor !== null && (!Array.isArray(anchor) || anchor.length !== 2 || !anchor.every(Number.isFinite)
      || anchor[0] < box[0] || anchor[0] > box[0] + box[2] || anchor[1] < box[1] || anchor[1] > box[1] + box[3]))
    throw new Error("参考点必须位于目标框内");
  const request = { schema_version: 1, request_id: requestId, target_id: targetId.trim(),
    sample_timestamp_ns: stamp, frame_id: frame.header.frame_id, bbox_xywh_px: box, motion_model: "static" };
  if (anchor) request.anchor_reference_px = anchor;
  if (existingId) request.existing_track_id = existingId;
  // Python requires an integer. Avoid rounding nanoseconds through a JS Number.
  return JSON.stringify(request).replace('"sample_timestamp_ns":"' + stamp + '"', '"sample_timestamp_ns":' + stamp);
}

export function createRegistrationClient(send, connected, imageTopic, progress = () => {}) {
  let capture = null, pending = null;
  const finishCapture = (error, frame) => {
    if (!capture) return;
    const current = capture; capture = null; clearTimeout(current.timer);
    if (connected()) send({ op: "unsubscribe", id: current.id, topic: imageTopic });
    if (error) current.reject(error); else current.resolve(frame);
  };
  const finishRequest = (error, status) => {
    if (!pending) return;
    const current = pending; pending = null; clearTimeout(current.timer);
    if (error) current.reject(error); else current.resolve(status);
  };
  return {
    captureFrame() {
      if (!connected()) return Promise.reject(new Error("ROS 连接未建立，请连接后重试"));
      if (capture || pending) return Promise.reject(new Error("请等待当前请求结束"));
      return new Promise((resolve, reject) => {
        const id = "frame_" + crypto.randomUUID();
        capture = { id, resolve, reject, timer: setTimeout(() => finishCapture(new Error("未收到定位图像；请检查相机标定及去畸变图像输出")), 10000) };
        send({ op: "subscribe", id, topic: imageTopic, type: "sensor_msgs/msg/Image", throttle_rate: 1000, queue_length: 1 });
      });
    },
    cancelCapture() { finishCapture(new Error("已取消定格")); },
    register(frame, box, targetId, anchor, existingId) {
      if (!connected()) return Promise.reject(new Error("ROS 连接已断开，未提交目标"));
      if (pending) return Promise.reject(new Error("已有注册请求正在处理"));
      const id = "target_" + crypto.randomUUID();
      let data;
      try { data = registrationJson(frame, box, targetId, id, anchor, existingId); }
      catch (error) { return Promise.reject(error); }
      return new Promise((resolve, reject) => {
        pending = { id, targetId: targetId.trim(), resolve, reject,
          timer: setTimeout(() => finishRequest(new Error("注册回执超时，结果未知；请先检查目标列表和后端状态，再重新提交")), 15000) };
        try {
          send({ op: "advertise", id: id + "_image_type", topic: REFERENCE_TOPIC, type: "sensor_msgs/msg/Image" });
          send({ op: "advertise", id: id + "_request_type", topic: REQUEST_TOPIC, type: "std_msgs/msg/String" });
          send({ op: "publish", id: id + "_image", topic: REFERENCE_TOPIC, msg: frame });
          send({ op: "publish", id: id + "_request", topic: REQUEST_TOPIC, msg: { data } });
        } catch (error) { finishRequest(error); }
      });
    },
    handle(envelope) {
      if (envelope.op === "status" && envelope.level === "error") {
        if (pending && envelope.id?.startsWith(pending.id)) finishRequest(new Error(envelope.msg || "注册接口拒绝请求"));
        if (capture && envelope.id === capture.id) finishCapture(new Error(envelope.msg || "无法读取定位图像"));
      }
      if (envelope.op !== "publish") return;
      if (envelope.topic === imageTopic && capture) {
        try { imagePixels(envelope.msg); finishCapture(null, envelope.msg); }
        catch (error) { finishCapture(error); }
      }
      if (envelope.topic === REQUEST_STATUS_TOPIC) {
        let status;
        try { status = JSON.parse(envelope.msg.data); } catch { return; }
        if (!pending || status.request_id !== pending.id || status.target_id !== pending.targetId) return;
        progress(status);
        if (status.state === "registered") finishRequest(null, status);
        if (status.state === "rejected") finishRequest(new Error(status.reason || "目标注册被拒绝"));
      }
    },
    disconnect() {
      finishCapture(new Error("连接已断开，请重新连接后定格"));
      finishRequest(new Error("连接已断开，注册结果未知；请重连后检查目标状态"));
    },
  };
}
