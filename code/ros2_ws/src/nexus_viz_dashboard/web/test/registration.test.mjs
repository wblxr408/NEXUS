import test from "node:test";
import assert from "node:assert/strict";
import { imagePoint, selectionBox, frameStamp, imagePixels, registrationJson, createRegistrationClient,
  REFERENCE_TOPIC, REQUEST_TOPIC, REQUEST_STATUS_TOPIC } from "../src/services/target_registration.js";
import { createDashboardStore } from "../src/features/telemetry/dashboard_store.js";
import { INPUT_TOPICS, inputSummary } from "../src/services/input_status.js";

test("stationary mean progress and stale state are visible", () => {
  const spec = INPUT_TOPICS.find(item => item.key === "uwb_mean");
  const message = value => ({ data: JSON.stringify(value) });
  const value = { window_samples: 180, window_capacity: 200, ranges_m: [5.049, 2.355, 4.042, 5.523], stale: false };
  assert.match(inputSummary(spec, message(value)).detail, /180\/200.*5.049 m/);
  assert.equal(inputSummary(spec, message({ ...value, stale: true })).accepted, false);
});

const frame = () => ({ header: { stamp: { sec: 1780000000, nanosec: 123456789 }, frame_id: "camera_optical_frame" },
  width: 16, height: 12, encoding: "rgb8", step: 48, is_bigendian: 0, data: Buffer.alloc(576, 127).toString("base64") });
function rig() {
  const sent = []; let connected = true;
  const client = createRegistrationClient(message => sent.push(message), () => connected, "/camera/image_rect");
  return { client, sent, disconnect: () => { connected = false; client.disconnect(); },
    deliver: (topic, msg) => client.handle({ op: "publish", topic, msg }) };
}
test("CSS scaling, reverse drag and bounds stay in original image coordinates", () => {
  const rect = { left: 100, top: 50, width: 820, height: 616 };
  assert.deepEqual(imagePoint(305, 204, rect, 1640, 1232), [410, 308]);
  assert.deepEqual(imagePoint(-100, 3000, rect, 1640, 1232), [0, 1231]);
  assert.deepEqual(selectionBox([310.5, 80.2], [10.2, 30.5]), [10, 30, 301, 51]);
});
test("exact nanosecond integer and explicit static reference, no assumed class", () => {
  const image = frame();
  const data = registrationJson(image, [2, 2, 10, 8], " chosen ", "req", [4, 5], "automatic_1");
  assert.ok(data.includes('"sample_timestamp_ns":1780000000123456789'));
  assert.equal(frameStamp(image), "1780000000123456789");
  assert.equal(JSON.parse(data).class_id, undefined);
  assert.equal(JSON.parse(data).motion_model, "static");
  assert.equal(JSON.parse(data).existing_track_id, "automatic_1");
  assert.throws(() => registrationJson(image, [8, 2, 12, 8], "id", "req"), /框/);
  assert.throws(() => registrationJson(image, [2, 2, 10, 8], "id", "req", [15, 5]), /框内/);
  assert.throws(() => registrationJson(image, [2, 2, 10, 8], " ", "req"), /名称/);
});
test("raw images reject corrupt payloads and support padded rows", () => {
  assert.equal(imagePixels(frame()).bytes.length, 576);
  assert.throws(() => imagePixels({ ...frame(), data: "AA==" }), /长度/);
  assert.throws(() => imagePixels({ ...frame(), encoding: "16UC1" }), /格式/);
  assert.equal(imagePixels({ ...frame(), step: 52, data: Buffer.alloc(624).toString("base64") }).bytes.length, 624);
});
test("freeze unsubscribes and publishes unchanged original image only on explicit confirmation", async () => {
  const r = rig(), image = frame();
  const capture = r.client.captureFrame();
  assert.deepEqual(r.sent.map(m => m.op), ["subscribe"]);
  r.deliver("/camera/image_rect", image);
  const frozen = await capture;
  assert.deepEqual(r.sent.map(m => m.op), ["subscribe", "unsubscribe"]);
  const result = r.client.register(frozen, [1, 1, 12, 9], "chosen");
  const publications = r.sent.filter(m => m.op === "publish");
  assert.deepEqual(publications.map(m => m.topic), [REFERENCE_TOPIC, REQUEST_TOPIC]);
  assert.deepEqual(publications[0].msg, image);
  const request = JSON.parse(publications[1].msg.data);
  let finished = false; result.then(() => { finished = true; });
  r.deliver(REQUEST_STATUS_TOPIC, { data: JSON.stringify({ request_id: "other", target_id: "chosen", state: "registered" }) });
  await Promise.resolve(); assert.equal(finished, false);
  r.deliver(REQUEST_STATUS_TOPIC, { data: JSON.stringify({ request_id: request.request_id, target_id: "chosen", state: "pending" }) });
  await Promise.resolve(); assert.equal(finished, false);
  r.deliver(REQUEST_STATUS_TOPIC, { data: JSON.stringify({ request_id: request.request_id, target_id: "chosen", state: "registered" }) });
  assert.equal((await result).state, "registered");
});
test("backend rejection and disconnect are never successful registrations", async () => {
  const r = rig();
  let result = r.client.register(frame(), [1, 1, 12, 9], "chosen");
  const request = JSON.parse(r.sent.at(-1).msg.data);
  r.deliver(REQUEST_STATUS_TOPIC, { data: JSON.stringify({ ...request, state: "rejected", reason: "insufficient features" }) });
  await assert.rejects(result, /insufficient/);
  result = r.client.register(frame(), [1, 1, 12, 9], "chosen");
  r.disconnect(); await assert.rejects(result, /结果未知/);
  await assert.rejects(r.client.captureFrame(), /连接/);
});
test("camera cancellation and rosbridge permissions errors release pending operations", async () => {
  const r = rig();
  const capture = r.client.captureFrame(); r.client.cancelCapture();
  await assert.rejects(capture, /取消/);
  const result = r.client.register(frame(), [1, 1, 12, 9], "chosen");
  r.client.handle({ op: "status", level: "error", id: r.sent.at(-1).id, msg: "topic denied" });
  await assert.rejects(result, /denied/);
});
test("sensor receipt is separate from localization validity and browser arrival age", () => {
  const store = createDashboardStore();
  const spec = INPUT_TOPICS.find(s => s.key === "camera_info");
  assert.equal(inputSummary(spec, { k: [0, 0, 0, 0, 0], width: 640, height: 480 }).accepted, false);
  store.updateInput("uwb", { accepted: true, detail: "ranges" }, 100);
  assert.equal(store.getState().pose.x, null);
  assert.equal(store.getState().mode, "NO_INPUT");
  store.ageInputs(2201, true);
  assert.equal(store.getState().inputs.uwb.stale, true);
  store.ageInputs(2301, false);
  assert.equal(store.getState().inputs.uwb.disconnected, true);
});

test("rosbridge full-image fragments reassemble out of order without losing nanoseconds", async () => {
  const { createEnvelopeDecoder } = await import("../src/services/rosbridge_fragments.js");
  let now = 0;
  const decode = createEnvelopeDecoder(() => now);
  const original = JSON.stringify({ op: "publish", topic: "/camera/image_rect", msg: frame() });
  const midpoint = Math.floor(original.length / 2);
  const part = (num, data, id = "image") => JSON.stringify({ op: "fragment", id, num, total: 2, data });
  assert.equal(decode(part(1, original.slice(midpoint))), null);
  assert.equal(decode(part(1, original.slice(midpoint))), null);
  assert.deepEqual(decode(part(0, original.slice(0, midpoint))), JSON.parse(original));
  assert.equal(decode(part(0, original.slice(0, midpoint))), null);
  now = 10001;
  assert.equal(decode(part(1, original.slice(midpoint))), null);
  assert.equal(decode('{"op":"fragment","id":"bad","num":0,"total":999999,"data":"x"}'), null);
  assert.equal(decode("bad json"), null);
});
