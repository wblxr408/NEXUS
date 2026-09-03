function carlaStatusUrl() {
  const query = new URLSearchParams(window.location.search);
  if (query.has("carla_ws")) return query.get("carla_ws");
  if (window.NEXUS_CARLA_STATUS_URL) return window.NEXUS_CARLA_STATUS_URL;
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return protocol + "://" + window.location.hostname + ":8765/ws/carla_status";
}

function carlaSceneUrl() {
  const query = new URLSearchParams(window.location.search);
  if (query.has("carla_scene_ws")) return query.get("carla_scene_ws");
  if (window.NEXUS_CARLA_SCENE_URL) return window.NEXUS_CARLA_SCENE_URL;
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return protocol + "://" + window.location.hostname + ":8765/ws";
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

function applyScene(store, scene) {
  if (!scene || scene.type !== "scene") return;
  const map = scene.map || {};
  const simulation = scene.simulation || {};
  const actors = Array.isArray(scene.actors) ? scene.actors : [];
  const vehicles = actors.filter((actor) => actor.type === "vehicle").length;
  const walkers = actors.filter((actor) => actor.type === "walker").length;
  const hero = actors.find((actor) => actor.role === "hero");
  store.updateCarlaStatus({
    protocol: scene.protocol,
    connected: true,
    endpoint: "127.0.0.1:2000",
    map: map.name,
    actor_count: actors.length,
    vehicle_count: vehicles,
    walker_count: walkers,
    frame: simulation.frame,
    timestamp: scene.timestamp,
    sync_mode: simulation.synchronous,
    ego_pose: hero?.position,
  });
  if (hero?.position) store.updateEgoPose({ ...hero.position, ...hero.rotation, speed: hero.speed });
  store.updateStatus({ mode: "SIMULATION", session: "CARLA_LIVE" });
  store.appendLog({ level: "INFO", text: `CARLA scene received: ${map.name || "unknown"} (${(map.roads || []).length} roads)` });
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
  const sceneUrl = carlaSceneUrl();
  let sceneSocket = null;
  let sceneRetryTimer = null;
  const connectScene = () => {
    if (closed) return;
    try { sceneSocket = new WebSocket(sceneUrl); } catch (_) { sceneRetryTimer = window.setTimeout(connectScene, 2000); return; }
    sceneSocket.addEventListener("open", () => store.appendLog({ level: "INFO", text: "CARLA scene gateway connected: " + sceneUrl }));
    sceneSocket.addEventListener("close", () => { if (!closed) sceneRetryTimer = window.setTimeout(connectScene, 2000); });
    sceneSocket.addEventListener("error", () => {});
    sceneSocket.addEventListener("message", (event) => { let payload; try { payload = JSON.parse(event.data); } catch (_) { return; } applyScene(store, payload); });
  };
  connectScene();
  return Object.freeze({ close: () => { closed = true; if (retryTimer) window.clearTimeout(retryTimer); if (sceneRetryTimer) window.clearTimeout(sceneRetryTimer); if (socket) socket.close(); if (sceneSocket) sceneSocket.close(); }, url, sceneUrl });
}
