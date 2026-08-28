function carlaStatusUrl() {
  const query = new URLSearchParams(window.location.search);
  if (query.has("carla_ws")) return query.get("carla_ws");
  if (window.NEXUS_CARLA_STATUS_URL) return window.NEXUS_CARLA_STATUS_URL;
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return protocol + "://" + window.location.hostname + ":8766/ws/carla_status";
}

function normalizeLogEntry(entry) {
  if (entry && typeof entry === "object" && !Array.isArray(entry)) {
    return { timestamp: entry.timestamp ?? entry.ts ?? null, level: String(entry.level ?? "INFO").toUpperCase(), text: entry.text ?? entry.message ?? JSON.stringify(entry) };
  }
  return { timestamp: null, level: "INFO", text: entry == null ? "null" : String(entry) };
}

function extractTargetPose(payload) {
  if (payload.target) return payload.target;
  if (payload.target_pose) return payload.target_pose;
  if (payload.fusion?.target_pose) return payload.fusion.target_pose;
  if (payload.fusion?.target_x !== undefined || payload.fusion?.target_y !== undefined || payload.fusion?.target_z !== undefined) {
    return { x: payload.fusion.target_x, y: payload.fusion.target_y, z: payload.fusion.target_z ?? 0 };
  }
  return null;
}

function applyPayload(store, payload) {
  if (!payload || typeof payload !== "object") return;
  payload = payload.data && typeof payload.data === "object" ? { ...payload.data, type: payload.type } : payload;
  store.updateStatus({ mode: "SIMULATION", session: "CARLA_LIVE" });
  store.updateCarlaStatus({ ...payload, connected: payload.connected ?? payload.carla_connected ?? true, thumbnail: payload.thumbnail ?? payload.camera_thumbnail ?? payload.cameraThumbnail });
  if (payload.ego_pose) store.updateEgoPose(payload.ego_pose);
  const target = extractTargetPose(payload);
  if (target && [target.x, target.y, target.z].every((value) => value !== undefined)) {
    store.updatePose(target.x, target.y, target.z, { targetId: target.id ?? payload.target_id, sourceMode: payload.source_mode ?? "FUSED", frameId: payload.frame_id ?? payload.frame, timestamp: payload.timestamp ?? null, mode: "SIMULATION", session: "CARLA_LIVE" });
  }
  if (payload.uwb_observation) store.updateObservations({ uwb: payload.uwb_observation });
  if (payload.vision_observation) store.updateObservations({ vision: payload.vision_observation });
  store.updateAlgorithms(payload.algorithms || { orb_slam2: payload.orb_slam2, uwb: payload.uwb, detector: payload.detector ?? payload.gdr_net, fusion: payload.fusion });
  if (payload.latency !== undefined) store.addLatency(payload.latency);
  if (payload.logs) store.appendLog(Array.isArray(payload.logs) ? payload.logs.map(normalizeLogEntry) : normalizeLogEntry(payload.logs));
  if (payload.log) store.appendLog(normalizeLogEntry(payload.log));
}

export function installCarlaStatusGateway(store) {
  const url = carlaStatusUrl();
  let socket = null;
  let retryTimer = null;
  let closed = false;
  const connect = () => {
    if (closed) return;
    try { socket = new WebSocket(url); } catch (_) { retryTimer = window.setTimeout(connect, 2000); return; }
    socket.addEventListener("open", () => { store.updateCarlaStatus({ connected: true }); store.appendLog({ level: "INFO", text: "CARLA status gateway connected: " + url }); });
    socket.addEventListener("close", () => {
      store.updateCarlaStatus({ connected: false });
      if (!closed) { store.appendLog({ level: "WARN", text: "CARLA status gateway closed; retrying" }); retryTimer = window.setTimeout(connect, 2000); }
    });
    socket.addEventListener("error", () => { store.updateCarlaStatus({ connected: false }); });
    socket.addEventListener("message", (event) => { let payload; try { payload = JSON.parse(event.data); } catch (_) { return; } applyPayload(store, payload); });
  };
  connect();
  return Object.freeze({ close: () => { closed = true; if (retryTimer) window.clearTimeout(retryTimer); if (socket) socket.close(); }, url });
}
