import { EMPTY_METRICS, INPUT_MODES, SOURCE_MODES, finiteOrNull } from "../../types/dashboard_types.js";

const EMPTY_ALGORITHM = Object.freeze({ status: "NO_INPUT", summary: null, latencyMs: null, dropRate: null, lastUpdate: null });

function normalizeAlgorithm(previous, data = {}) {
  const next = Object.fromEntries(Object.entries(data).filter(([, value]) => value !== undefined && value !== null));
  return {
    ...previous,
    ...next,
    latencyMs: data.latencyMs === undefined ? previous.latencyMs : finiteOrNull(data.latencyMs),
    dropRate: data.dropRate === undefined ? previous.dropRate : finiteOrNull(data.dropRate),
  };
}

function normalizeLogEntry(entry) {
  if (entry && typeof entry === "object" && !Array.isArray(entry)) {
    return {
      timestamp: entry.timestamp ?? entry.ts ?? null,
      level: String(entry.level ?? "INFO").toUpperCase(),
      text: entry.text ?? entry.message ?? JSON.stringify(entry),
    };
  }
  return { timestamp: null, level: "INFO", text: entry == null ? "null" : String(entry) };
}

const initialState = () => ({
  mode: INPUT_MODES.NO_INPUT,
  session: "WAITING_FOR_INPUT",
  runId: "UNREGISTERED",
  targetId: null,
  sourceMode: SOURCE_MODES.UNKNOWN,
  pose: { x: null, y: null, z: null },
  sigma: { x: null, y: null, z: null },
  // 求解侧溯源字段；TargetObservation.msg 没有对应字段，只能由落盘 CSV/JSON 经
  // updateSolverProvenance() 注入，无数据时保持 null。
  nViews: null,
  baselineM: null,
  depthSource: null,
  chain: null,
  ego: { x: null, y: null, z: null, roll: null, pitch: null, yaw: null, speed: null },
  platform: { x: null, y: null, z: null },
  frameId: null,
  poseTimestamp: null,
  poseAge: null,
  trajectory: [],
  observations: { uwb: null, vision: null },
  camera: { confidence: null, reproj: null, imageAge: null, status: "NO_FRAME", image: null },
  carla: {
    connected: false,
    endpoint: "127.0.0.1:2000",
    mapName: null,
    synchronousMode: null,
    tickHz: null,
    actorCount: null,
    vehicleCount: null,
    walkerCount: null,
    trafficLightCount: null,
    frame: null,
    timestamp: null,
    trafficSummary: null,
    sensors: {},
    thumbnail: null,
  },
  algorithms: {
    orbSlam2: { ...EMPTY_ALGORITHM },
    uwb: { ...EMPTY_ALGORITHM },
    detector: { ...EMPTY_ALGORITHM },
    fusion: { ...EMPTY_ALGORITHM },
  },
  logs: [],
  health: {
    uwb: { status: "NO_INPUT", age: null },
    vision: { status: "NO_INPUT", age: null },
    fusion: { status: "NO_INPUT", pairingDelta: null },
    bridge: { status: "DISCONNECTED", sequence: null },
  },
  latency: [],
  metrics: { ...EMPTY_METRICS },
});

export function createDashboardStore() {
  let state = initialState();
  const listeners = new Set();
  const notify = () => listeners.forEach((listener) => listener(state));
  const patch = (updates) => { state = { ...state, ...updates }; notify(); };

  return {
    getState: () => state,
    subscribe(listener) { listeners.add(listener); listener(state); return () => listeners.delete(listener); },
    updatePose(x, y, z, metadata = {}) {
      const pose = { x: finiteOrNull(x), y: finiteOrNull(y), z: finiteOrNull(z) };
      if (Object.values(pose).some((value) => value === null)) return;
      const sigma = metadata.sigma === undefined ? state.sigma : {
        x: finiteOrNull(metadata.sigma?.x),
        y: finiteOrNull(metadata.sigma?.y),
        z: finiteOrNull(metadata.sigma?.z),
      };
      patch({
        pose,
        sigma,
        targetId: metadata.targetId ?? state.targetId,
        sourceMode: metadata.sourceMode ?? state.sourceMode,
        frameId: metadata.frameId ?? state.frameId,
        poseTimestamp: metadata.timestamp ?? state.poseTimestamp,
        poseAge: metadata.age ?? state.poseAge,
        mode: metadata.mode ?? state.mode,
        session: metadata.session ?? (state.mode === INPUT_MODES.NO_INPUT ? "LIVE" : state.session),
        trajectory: [...state.trajectory, { ...pose, timestamp: metadata.timestamp ?? null }].slice(-120),
      });
    },
    updateSolverProvenance(data = {}) {
      // 求解侧落盘记录的接入点（4.5 节的 n_views / baseline_m / depth_source / chain）。
      patch({
        nViews: data.nViews === undefined ? state.nViews : finiteOrNull(data.nViews),
        baselineM: data.baselineM === undefined ? state.baselineM : finiteOrNull(data.baselineM),
        depthSource: data.depthSource ?? state.depthSource,
        chain: data.chain ?? state.chain,
      });
    },
    updateCamera(data = {}) { patch({ camera: { ...state.camera, ...data } }); },
    updateEgoPose(data = {}) {
      const ego = {
        x: data.x === undefined ? state.ego.x : finiteOrNull(data.x),
        y: data.y === undefined ? state.ego.y : finiteOrNull(data.y),
        z: data.z === undefined ? state.ego.z : finiteOrNull(data.z),
        roll: data.roll === undefined ? state.ego.roll : finiteOrNull(data.roll),
        pitch: data.pitch === undefined ? state.ego.pitch : finiteOrNull(data.pitch),
        yaw: data.yaw === undefined ? state.ego.yaw : finiteOrNull(data.yaw),
        speed: data.speed === undefined ? state.ego.speed : finiteOrNull(data.speed),
      };
      patch({ ego });
    },
    updatePlatform(position = {}) {
      const platform = { x: finiteOrNull(position.x), y: finiteOrNull(position.y), z: finiteOrNull(position.z) };
      if (Object.values(platform).some((value) => value === null)) return;
      patch({ platform, ego: { ...state.ego, ...platform } });
    },
    updateObservations(data = {}) { patch({ observations: { ...state.observations, ...data } }); },
    updateMetrics(data = {}) {
      // `cep` 是旧单文件稿的字段名，转换只做字段兼容，不改变指标口径。
      const normalized = data.p95 === undefined && data.cep !== undefined ? { ...data, p95: data.cep } : data;
      patch({ metrics: { ...state.metrics, ...normalized }, runId: normalized.runId ?? state.runId });
    },
    updateStatus(data = {}) { patch(data); },
    updateHealth(data = {}) { patch({ health: { ...state.health, ...data } }); },
    updateCarlaStatus(data = {}) {
      const carla = {
        ...state.carla,
        connected: data.connected ?? data.carla_connected ?? state.carla.connected,
        endpoint: data.endpoint ?? data.rpc_endpoint ?? state.carla.endpoint,
        mapName: data.mapName ?? data.map ?? state.carla.mapName,
        synchronousMode: data.synchronousMode ?? data.syncMode ?? data.sync_mode ?? state.carla.synchronousMode,
        tickHz: data.tickHz === undefined && data.tick_hz === undefined ? state.carla.tickHz : finiteOrNull(data.tickHz ?? data.tick_hz),
        actorCount: data.actorCount === undefined && data.actor_count === undefined ? state.carla.actorCount : finiteOrNull(data.actorCount ?? data.actor_count),
        vehicleCount: data.vehicleCount === undefined && data.vehicle_count === undefined ? state.carla.vehicleCount : finiteOrNull(data.vehicleCount ?? data.vehicle_count),
        walkerCount: data.walkerCount === undefined && data.walker_count === undefined ? state.carla.walkerCount : finiteOrNull(data.walkerCount ?? data.walker_count),
        trafficLightCount: data.trafficLightCount === undefined && data.traffic_light_count === undefined ? state.carla.trafficLightCount : finiteOrNull(data.trafficLightCount ?? data.traffic_light_count),
        frame: data.frame ?? data.frameId ?? data.frame_id ?? state.carla.frame,
        timestamp: data.timestamp ?? data.sim_timestamp ?? state.carla.timestamp,
        trafficSummary: data.trafficSummary ?? data.traffic_summary ?? data.actor_summary ?? state.carla.trafficSummary,
        sensors: data.sensors ? { ...state.carla.sensors, ...data.sensors } : state.carla.sensors,
        thumbnail: data.thumbnail ?? data.cameraThumbnail ?? data.camera_thumbnail ?? state.carla.thumbnail,
      };
      patch({ carla });
    },
    updateAlgorithms(data = {}) {
      const incoming = data || {};
      const algorithms = {
        orbSlam2: normalizeAlgorithm(state.algorithms.orbSlam2, incoming.orbSlam2 ?? incoming.orb_slam2 ?? {}),
        uwb: normalizeAlgorithm(state.algorithms.uwb, incoming.uwb ?? {}),
        detector: normalizeAlgorithm(state.algorithms.detector, incoming.detector ?? incoming.gdrNet ?? incoming.gdr_net ?? {}),
        fusion: normalizeAlgorithm(state.algorithms.fusion, incoming.fusion ?? {}),
      };
      patch({ algorithms });
    },
    appendLog(entry) {
      const incoming = Array.isArray(entry) ? entry : [entry];
      patch({ logs: [...state.logs, ...incoming.map(normalizeLogEntry)].slice(-120) });
    },
    addLatency(value) {
      const latency = finiteOrNull(value);
      if (latency !== null) patch({ latency: [...state.latency, latency].slice(-60) });
    },
    reset() { state = initialState(); notify(); },
  };
}
