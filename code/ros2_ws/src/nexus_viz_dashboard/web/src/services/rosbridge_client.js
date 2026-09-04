/** Minimal browser-side ROS2 bridge for live hardware and Gazebo inputs. */
const SOURCE_MODES = Object.freeze({ 1: "UWB", 2: "VISION", 3: "PLATFORM_RELATIVE", 4: "FUSED" });
const IMX219_COMPRESSED_TOPIC = "/nexus/camera/imx219/image_raw/compressed";
// IA §3.3 requires 640x480 @5-10 FPS; 150 ms keeps the preview at ~6.7 FPS on both
// the rosbridge throttle and the local decode gate.
const CAMERA_INTERVAL_MS = 150;

function websocketUrl() {
  const query = new URLSearchParams(window.location.search);
  if (query.has("rosbridge")) return query.get("rosbridge");
  if (window.NEXUS_ROSBRIDGE_URL) return window.NEXUS_ROSBRIDGE_URL;
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname}:9090`;
}

function stampText(header) {
  const stamp = header?.stamp;
  if (!stamp) return null;
  const seconds = stamp.sec ?? stamp.secs;
  const nanoseconds = stamp.nanosec ?? stamp.nsecs ?? 0;
  return seconds === undefined ? null : `${seconds}.${String(nanoseconds).padStart(9, "0")}`;
}

function decodeBase64(data) {
  const encoded = window.atob(data);
  const bytes = new Uint8Array(encoded.length);
  for (let index = 0; index < encoded.length; index += 1) bytes[index] = encoded.charCodeAt(index);
  return bytes;
}

function previewFromRawImage(message) {
  const { width, height, encoding, step, data } = message;
  const channels = encoding === "rgb8" || encoding === "bgr8" ? 3 : encoding === "mono8" ? 1 : 0;
  if (!width || !height || !channels || typeof data !== "string") return null;
  const bytes = decodeBase64(data);
  const sourceStep = step || width * channels;
  const maxWidth = 820;
  const scale = Math.max(1, width / maxWidth);
  const outputWidth = Math.round(width / scale);
  const outputHeight = Math.round(height / scale);
  const scratch = document.createElement("canvas");
  scratch.width = outputWidth;
  scratch.height = outputHeight;
  const context = scratch.getContext("2d");
  const output = context.createImageData(outputWidth, outputHeight);
  for (let y = 0; y < outputHeight; y += 1) {
    const sourceY = Math.min(height - 1, Math.floor(y * height / outputHeight));
    for (let x = 0; x < outputWidth; x += 1) {
      const sourceX = Math.min(width - 1, Math.floor(x * width / outputWidth));
      const source = sourceY * sourceStep + sourceX * channels;
      const destination = (y * outputWidth + x) * 4;
      const value = bytes[source];
      output.data[destination] = encoding === "bgr8" ? bytes[source + 2] : value;
      output.data[destination + 1] = channels === 1 ? value : bytes[source + 1];
      output.data[destination + 2] = encoding === "bgr8" ? value : channels === 1 ? value : bytes[source + 2];
      output.data[destination + 3] = 255;
    }
  }
  context.putImageData(output, 0, 0);
  return scratch.toDataURL("image/jpeg", 0.84);
}

function previewFromCompressedImage(message) {
  if (typeof message.data !== "string" || !message.data) return null;
  const format = (message.format || "jpeg").toLowerCase();
  const mime = format.includes("png") ? "image/png" : "image/jpeg";
  return `data:${mime};base64,${message.data}`;
}

function sigmaFromCovariance(covariance) {
  // Position variances on the diagonal of the row-major 6x6 block: indices 0, 7, 14.
  // Missing, NaN or negative variance stays null; the panel must not show a fake 0.
  const sigma = [0, 7, 14].map((index) => {
    const variance = Number(covariance?.[index]);
    return Number.isFinite(variance) && variance >= 0 ? Math.sqrt(variance) : null;
  });
  return { x: sigma[0], y: sigma[1], z: sigma[2] };
}

function observationToStore(store, message, kind) {
  const position = message.pose?.position;
  if (!position || message.validity !== 1) return;
  const algorithm = kind === "uwb" ? "uwb" : kind === "vision" ? "detector" : "fusion";
  store.updateAlgorithms({
    [algorithm]: {
      status: kind === "target" ? "VALID" : "OK",
      summary: message.target_id ? `${message.target_id} observation` : "target observation",
      lastUpdate: stampText(message.header),
    },
  });
  const metadata = {
    targetId: message.target_id,
    sourceMode: SOURCE_MODES[message.source_mode] || "UNKNOWN",
    frameId: message.header?.frame_id,
    timestamp: stampText(message.header),
    mode: "SIMULATION",
    session: "GAZEBO_LIVE",
    sigma: sigmaFromCovariance(message.covariance),
  };
  // nViews / baselineM / depthSource / chain have no field in TargetObservation.msg and
  // this plan adds no message type, so they arrive only through
  // store.updateSolverProvenance() from the solver-side CSV/JSON drop, never from here.
  if (kind === "target") store.updatePose(position.x, position.y, position.z, metadata);
  else store.updateObservations({ [kind]: position });
}

export function installRosbridgeGateway(store) {
  const socket = new WebSocket(websocketUrl());
  let sequence = 0;
  let lastCameraDecodeMs = 0;
  let lastLocalizationMs = null;
  const watchdog = window.setInterval(() => {
    if (lastLocalizationMs !== null && performance.now() - lastLocalizationMs > 1000)
      store.localizationDisconnected("STALE_CONNECTION");
  }, 250);
  const subscribe = (topic, type, throttleRate = 0) => socket.send(JSON.stringify({
    op: "subscribe", topic, type, throttle_rate: throttleRate, queue_length: 1,
  }));

  socket.addEventListener("open", () => {
    subscribe("/nexus/viz/localization_state", "std_msgs/msg/String", 100);
    subscribe("/nexus/target/pose", "nexus_msgs/msg/TargetObservation");
    subscribe("/nexus/vision/map_target_observation", "nexus_msgs/msg/TargetObservation");
    subscribe("/nexus/uwb/target_observation", "nexus_msgs/msg/TargetObservation");
    subscribe("/nexus/gazebo/uav/odom", "nav_msgs/msg/Odometry", 100);
    // JPEG avoids expanding an 8 MP raw image to a >30 MB WebSocket JSON message.
    subscribe(IMX219_COMPRESSED_TOPIC, "sensor_msgs/msg/CompressedImage", CAMERA_INTERVAL_MS);
    store.updateHealth({ bridge: { status: "CONNECTED", sequence } });
    store.updateStatus({ mode: "SIMULATION", session: "GAZEBO_LIVE" });
  });
  socket.addEventListener("close", () => {
    store.updateHealth({ bridge: { status: "DISCONNECTED", sequence } });
    store.localizationDisconnected();
  });
  socket.addEventListener("error", () => {
    store.updateHealth({ bridge: { status: "ERROR", sequence } });
    store.localizationDisconnected("ERROR");
  });
  socket.addEventListener("message", (event) => {
    let envelope;
    try { envelope = JSON.parse(event.data); } catch (_) { return; }
    if (envelope.op !== "publish") return;
    sequence += 1;
    store.updateHealth({ bridge: { status: "CONNECTED", sequence } });
    const message = envelope.msg || {};
    if (envelope.topic === "/nexus/viz/localization_state") {
      try {
        if (store.updateLocalization(JSON.parse(message.data))) lastLocalizationMs = performance.now();
      } catch (_) { /* A malformed packet must not refresh transport freshness. */ }
      return;
    }
    if (envelope.topic === "/nexus/target/pose") observationToStore(store, message, "target");
    if (envelope.topic === "/nexus/vision/map_target_observation") observationToStore(store, message, "vision");
    if (envelope.topic === "/nexus/uwb/target_observation") observationToStore(store, message, "uwb");
    if (envelope.topic === "/nexus/gazebo/uav/odom") store.updatePlatform(message.pose?.pose?.position);
    if (envelope.topic === IMX219_COMPRESSED_TOPIC) {
      const now = performance.now();
      if (now - lastCameraDecodeMs < CAMERA_INTERVAL_MS) return;
      lastCameraDecodeMs = now;
      const image = previewFromCompressedImage(message);
      store.updateCamera({
        image,
        status: image ? "COMPRESSED" : `UNSUPPORTED_${message.format || "IMAGE"}`,
        imageAge: null,
      });
    }
  });
  return Object.freeze({ close: () => { window.clearInterval(watchdog); socket.close(); }, url: websocketUrl() });
}
