// Source clocks stay as decimal strings. Browser arrival time is transport
// health only, never a replacement for sensor age or a ROS timestamp.
const emptyPoint = () => ({ x: null, y: null, z: null });
const point = (value) => value ? { x: value[0], y: value[1], z: value[2] } : emptyPoint();
const vector = (value, length) => value === null || (Array.isArray(value) && value.length === length && value.every(Number.isFinite));
const timestamp = (value) => typeof value === "string" && /^\d+$/.test(value);
const age = (value) => value === null || (Number.isFinite(value) && value >= 0);

export function validLocalization(data) {
  if (!data || data.schema_version !== 1 || data.frame_id !== "map" || data.unit !== "m"
      || typeof data.session_id !== "string" || !data.session_id || !timestamp(data.generated_timestamp_ns)
      || !["LIVE", "REPLAY", "SIMULATION", "NO_INPUT"].includes(data.input_mode)
      || typeof data.run_id !== "string" || !data.run_id
      || (data.input_mode === "REPLAY" && data.run_id === "UNREGISTERED")
      || !Array.isArray(data.targets) || data.targets.length > 64) return false;
  const p = data.platform;
  if (!p || !vector(p.position, 3) || !vector(p.orientation, 4) || !age(p.age_s)
      || !(p.speed_mps === null || Number.isFinite(p.speed_mps))) return false;
  const ids = new Set();
  return data.targets.every((t) => {
    if (!t || typeof t.target_id !== "string" || !t.target_id || ids.has(t.target_id)
        || typeof t.state !== "string" || !t.state
        || !["observed", "historical", "lost"].includes(t.display_state)
        || !timestamp(t.sample_timestamp_ns) || !timestamp(t.last_valid_sample_timestamp_ns)
        || BigInt(t.last_valid_sample_timestamp_ns) > BigInt(t.sample_timestamp_ns)
        || BigInt(t.sample_timestamp_ns) > BigInt(data.generated_timestamp_ns)
        || !vector(t.position, 3) || !vector(t.historical_position, 3) || !vector(t.sigma_m, 3)
        || !age(t.age_s) || (t.display_state === "observed") !== (t.position !== null)
        || (t.position !== null && t.historical_position !== null)) return false;
    ids.add(t.target_id);
    return true;
  });
}

export function formatStamp(value) {
  if (!timestamp(value) || value === "0") return null;
  const ns = BigInt(value);
  return `${ns / 1000000000n}.${String(ns % 1000000000n).padStart(9, "0")}`;
}

export function localizationProjection(dual, selectedId, previousHealth = {}) {
  const data = dual.snapshot;
  const target = data.targets.find((item) => item.target_id === selectedId) ?? data.targets[0];
  const live = dual.connection === "CONNECTED";
  const position = live ? target?.position : null;
  const platform = live ? data.platform.position : null;
  return {
    dual, targetId: target?.target_id ?? null,
    mode: data.input_mode, runId: data.run_id, session: dual.connection,
    sourceMode: target?.source ?? "UNKNOWN", frameId: "map",
    pose: point(position), sigma: point(position ? target.sigma_m : null),
    poseTimestamp: position ? formatStamp(target.last_valid_sample_timestamp_ns) : null,
    poseAge: live ? target?.age_s ?? null : null,
    platform: point(platform), ego: { ...point(platform), roll: null, pitch: null, yaw: null,
      speed: live ? data.platform.speed_mps : null },
    nViews: null, baselineM: null, depthSource: "多视角静态目标", chain: "只读双对象定位",
    trajectory: [],
    health: {
      uwb: { status: "NO_INPUT", age: null },
      vision: { status: "NO_INPUT", age: null },
      fusion: { status: !live ? dual.connection : target
        ? target.display_state === "observed" ? target.state.toUpperCase() : target.display_state.toUpperCase()
        : "NO_INPUT", pairingDelta: null },
      bridge: previousHealth.bridge ?? { status: "DISCONNECTED", sequence: null },
    },
  };
}
