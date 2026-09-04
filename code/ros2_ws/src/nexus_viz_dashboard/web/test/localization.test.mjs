import test from "node:test";
import assert from "node:assert/strict";
import { createDashboardStore } from "../src/features/telemetry/dashboard_store.js";
import { validLocalization, formatStamp } from "../src/features/telemetry/localization_state.js";
import { installRosbridgeGateway } from "../src/services/rosbridge_client.js";

function snapshot(time = "1780000000000000001") {
  return { schema_version: 1, session_id: "one", generated_timestamp_ns: time, input_mode: "LIVE", run_id: "UNREGISTERED",
    frame_id: "map", unit: "m", platform: { display_state: "observed", position: [-2, 0, 3], orientation: [0, 0, 0, 1], speed_mps: 5, age_s: .1 },
    targets: ["one", "two"].map((id, i) => ({ target_id: id, source: "correlated_target_fusion", position_reference: `reference:${id}`,
      state: "confirmed", reason: "", display_state: "observed", sample_timestamp_ns: time, last_valid_sample_timestamp_ns: time,
      age_s: 0, position: [i + 1, 2, .5], historical_position: null, sigma_m: [.1, .2, .3] })) };
}

test("dual ownership separates platform, selected target, and historical positions", () => {
  const store = createDashboardStore();
  const first = snapshot();
  assert.ok(store.updateLocalization(first));
  assert.deepEqual(store.getState().platform, { x: -2, y: 0, z: 3 });
  assert.equal(store.getState().pose.x, 1);
  store.selectTarget("two");
  assert.equal(store.getState().pose.x, 2);
  store.updatePose(100, 100, 100, { mode: "SIMULATION" });
  store.updateEgoPose({ x: 100 });
  store.updatePlatform({ x: 100, y: 100, z: 100 });
  store.updateStatus({ mode: "SIMULATION", session: "CARLA_LIVE" });
  assert.equal(store.getState().pose.x, 2);
  assert.equal(store.getState().ego.x, -2);
  assert.equal(store.getState().mode, "LIVE");
  store.updateHealth({ fusion: { status: "OLD_PIPELINE" } });
  assert.equal(store.getState().health.fusion.status, "CONFIRMED");
  const history = snapshot("1780000000000000002");
  Object.assign(history.targets[1], { position: null, historical_position: [2, 2, .5], sigma_m: null, display_state: "historical", state: "lost" });
  store.updateLocalization(history);
  assert.equal(store.getState().pose.x, null);
  assert.equal(store.getState().sigma.x, null);
  assert.equal(store.getState().health.fusion.status, "HISTORICAL");
  assert.deepEqual(store.getState().dual.snapshot.targets[1].historical_position, [2, 2, .5]);
  store.localizationDisconnected();
  store.updatePose(9, 9, 9);
  assert.equal(store.getState().pose.x, null);
  const recovered = snapshot("1780000000000000003");
  recovered.targets[1].state = "re-associated";
  assert.ok(store.updateLocalization(recovered));
  assert.equal(store.getState().pose.x, 2);
  assert.equal(store.getState().dual.snapshot.targets[1].state, "re-associated");
});

test("nanosecond order, invalid snapshots and explicit session reset", () => {
  const store = createDashboardStore();
  assert.ok(store.updateLocalization(snapshot()));
  assert.equal(store.updateLocalization(snapshot("1780000000000000000")), false);
  assert.equal(formatStamp("1780000000000000001"), "1780000000.000000001");
  const bad = snapshot("1780000000000000002"); bad.targets[0].position[0] = NaN;
  assert.equal(store.updateLocalization(bad), false);
  const replay = snapshot(); replay.input_mode = "REPLAY";
  assert.equal(validLocalization(replay), false);
  replay.run_id = "synthetic_test_run"; replay.session_id = "new";
  assert.ok(store.updateLocalization(replay));
  store.reset();
  assert.equal(store.getState().dual, null);
});

test("ROS snapshot gateway subscribes read-only and times out on transport silence", () => {
  let clock = 0;
  const callbacks = [];
  const sockets = [];
  globalThis.window = { location: { search: "", protocol: "http:", hostname: "localhost" },
    setInterval: (callback) => { callbacks.push(callback); return callbacks.length; }, clearInterval: () => {} };
  globalThis.performance = { now: () => clock };
  globalThis.WebSocket = class {
    listeners = {}; sent = [];
    constructor() { sockets.push(this); }
    addEventListener(name, callback) { this.listeners[name] = callback; }
    send(data) { this.sent.push(JSON.parse(data)); }
    close() { this.listeners.close(); }
  };
  const store = createDashboardStore();
  const gateway = installRosbridgeGateway(store), ws = sockets[0];
  ws.listeners.open();
  assert.ok(ws.sent.some((item) => item.topic === "/nexus/viz/localization_state"));
  assert.ok(ws.sent.every((item) => item.op === "subscribe"));
  const deliver = (data) => ws.listeners.message({ data: JSON.stringify({ op: "publish", topic: "/nexus/viz/localization_state", msg: { data: JSON.stringify(data) } }) });
  deliver(snapshot());
  assert.equal(store.getState().pose.x, 1);
  clock = 1001; callbacks[0]();
  assert.equal(store.getState().pose.x, null);
  assert.equal(store.getState().platform.x, null);
  assert.equal(store.getState().poseAge, null);
  assert.equal(store.getState().dual.connection, "STALE_CONNECTION");
  deliver(snapshot()); // repeated old snapshot must not count as fresh
  assert.equal(store.getState().pose.x, null);
  deliver(snapshot("1780000000000000002"));
  assert.equal(store.getState().pose.x, 1);
  gateway.close();
  assert.equal(store.getState().pose.x, null);
});
