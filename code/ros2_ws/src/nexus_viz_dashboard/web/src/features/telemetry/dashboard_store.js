import { EMPTY_METRICS, INPUT_MODES, SOURCE_MODES, finiteOrNull } from "../../types/dashboard_types.js";

const initialState = () => ({
  mode: INPUT_MODES.NO_INPUT,
  session: "WAITING_FOR_INPUT",
  runId: "UNREGISTERED",
  targetId: null,
  sourceMode: SOURCE_MODES.UNKNOWN,
  pose: { x: null, y: null, z: null },
  frameId: null,
  poseTimestamp: null,
  trajectory: [],
  observations: { uwb: null, vision: null },
  camera: { confidence: null, reproj: null, imageAge: null, status: "NO_FRAME", image: null },
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
      patch({
        pose,
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
    updateCamera(data = {}) { patch({ camera: { ...state.camera, ...data } }); },
    updateObservations(data = {}) { patch({ observations: { ...state.observations, ...data } }); },
    updateMetrics(data = {}) {
      // `cep` 是旧单文件稿的字段名，转换只做字段兼容，不改变指标口径。
      const normalized = data.p95 === undefined && data.cep !== undefined ? { ...data, p95: data.cep } : data;
      patch({ metrics: { ...state.metrics, ...normalized }, runId: normalized.runId ?? state.runId });
    },
    updateStatus(data = {}) { patch(data); },
    updateHealth(data = {}) { patch({ health: { ...state.health, ...data } }); },
    addLatency(value) {
      const latency = finiteOrNull(value);
      if (latency !== null) patch({ latency: [...state.latency, latency].slice(-60) });
    },
    reset() { state = initialState(); notify(); },
  };
}
