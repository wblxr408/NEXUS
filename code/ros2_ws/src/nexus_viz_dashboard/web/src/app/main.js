import { dashboardLayout } from "../components/dashboard_layout.js";
import { createDashboardStore } from "../features/telemetry/dashboard_store.js";
import { installCarlaStatusGateway } from "../services/carla_status_client.js";
import { installNexusAPI } from "../services/nexus_api.js";
import { installRosbridgeGateway } from "../services/rosbridge_client.js";
import { formatNumber } from "../types/dashboard_types.js";

const root = document.querySelector("#nexus-app");
const store = createDashboardStore();
root.innerHTML = dashboardLayout();
installNexusAPI(store);
installRosbridgeGateway(store);
installCarlaStatusGateway(store);

const byId = (id) => document.getElementById(id);
const setText = (id, value) => { const element = byId(id); if (element) element.textContent = value; };
const ageText = (value) => value === null || value === undefined ? "unknown" : `${formatNumber(value, 2)} s`;
const metricText = (value, suffix = " m") => value === null || value === undefined ? "—" : `${formatNumber(value)}${suffix}`;
let renderedCameraImage = null;
let renderedCarlaThumbnail = null;

function renderCamera(imageData) {
  const canvas = byId("cam-canvas");
  const empty = byId("camera-empty");
  if (!imageData || imageData === renderedCameraImage) {
    empty.hidden = Boolean(renderedCameraImage);
    return;
  }
  if (typeof imageData !== "string") return;
  const image = new Image();
  image.onload = () => {
    const context = canvas.getContext("2d");
    context.drawImage(image, 0, 0, canvas.width, canvas.height);
    renderedCameraImage = imageData;
    empty.hidden = true;
  };
  image.src = imageData;
}

function drawLatency(values) {
  const canvas = byId("latency-canvas");
  const empty = byId("latency-empty");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, width * ratio);
  canvas.height = Math.max(1, height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  empty.hidden = values.length > 0;
  if (!values.length || !width || !height) return;
  const max = Math.max(...values, 0.001);
  context.strokeStyle = "#67507f";
  context.lineWidth = 2;
  context.beginPath();
  values.forEach((value, index) => {
    const x = values.length === 1 ? width / 2 : (index / (values.length - 1)) * width;
    const y = height - (value / max) * (height - 6) - 3;
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();
}

function formatPoseText(point, digits = 2) {
  if (!point || [point.x, point.y, point.z].some((value) => value === null || value === undefined || Number.isNaN(Number(value)))) return "—";
  return "x " + formatNumber(point.x, digits) + " · y " + formatNumber(point.y, digits) + " · z " + formatNumber(point.z, digits);
}

function formatAttitudeText(ego) {
  const values = [ego?.roll, ego?.pitch, ego?.yaw];
  if (values.some((value) => value === null || value === undefined || Number.isNaN(Number(value)))) return "—";
  return "r " + formatNumber(ego.roll, 1) + "° · p " + formatNumber(ego.pitch, 1) + "° · y " + formatNumber(ego.yaw, 1) + "°";
}

function renderCarlaThumbnail(imageData) {
  const image = byId("ui-carla-thumb");
  const empty = byId("ui-carla-thumb-empty");
  if (!image || !empty) return;
  if (!imageData || imageData === renderedCarlaThumbnail) {
    image.hidden = !renderedCarlaThumbnail;
    empty.hidden = Boolean(renderedCarlaThumbnail);
    return;
  }
  if (typeof imageData !== "string") return;
  image.src = imageData;
  image.hidden = false;
  empty.hidden = true;
  renderedCarlaThumbnail = imageData;
}

function renderAlgorithms(algorithms) {
  const list = byId("ui-algorithm-list");
  if (!list) return;
  list.replaceChildren();
  [["ORB-SLAM2", algorithms.orbSlam2], ["UWB", algorithms.uwb], ["GDR-NET", algorithms.detector], ["FUSION", algorithms.fusion]].forEach(([label, data]) => {
    const row = document.createElement("div"); row.className = "algorithm-row";
    const name = document.createElement("strong"); name.className = "algorithm-name"; name.textContent = label;
    const status = document.createElement("span"); status.className = "algorithm-status " + String(data?.status || "UNKNOWN").toLowerCase(); status.textContent = data?.status || "UNKNOWN";
    const meta = document.createElement("span"); meta.className = "algorithm-meta";
    const parts = []; if (data?.summary) parts.push(data.summary); if (data?.latencyMs != null) parts.push(formatNumber(data.latencyMs, 1) + " ms"); if (data?.dropRate != null) parts.push("drop " + formatNumber(data.dropRate, 1) + "%"); if (data?.lastUpdate) parts.push("@" + data.lastUpdate);
    meta.textContent = parts.join(" · ") || "暂无模块输出"; row.append(name, status, meta); list.append(row);
  });
}

function renderLogs(logs) {
  const list = byId("ui-log-list"); if (!list) return; list.replaceChildren();
  if (!logs.length) { const empty = document.createElement("div"); empty.className = "log-empty"; empty.textContent = "暂无日志；等待 CARLA bridge 推送。"; list.append(empty); return; }
  logs.slice(-40).reverse().forEach((entry) => {
    const row = document.createElement("div"); row.className = "log-row";
    const ts = document.createElement("span"); ts.className = "log-ts"; ts.textContent = entry.timestamp || "—";
    const level = document.createElement("span"); level.className = "log-level " + String(entry.level || "INFO").toLowerCase(); level.textContent = entry.level || "INFO";
    const text = document.createElement("span"); text.className = "log-text"; text.textContent = entry.text || "—"; row.append(ts, level, text); list.append(row);
  });
}

function render(state) {
  setText("ui-mode", state.mode.replace("_", " "));
  setText("ui-run-id", state.runId || "UNREGISTERED");
  setText("ui-target-id", state.targetId || "未检测到目标");
  setText("ui-global-age", ageText(state.poseAge));
  setText("ui-source", state.sourceMode || "UNKNOWN");
  setText("ui-session", state.session || "WAITING_FOR_INPUT");
  ["x", "y", "z"].forEach((axis) => setText(`ui-${axis}`, formatNumber(state.pose[axis])));
  setText("ui-frame", `frame: ${state.frameId || "—"}`);
  setText("ui-pose-time", state.poseTimestamp || "unknown");
  setText("ui-cam-conf", metricText(state.camera.confidence, ""));
  setText("ui-cam-reproj", metricText(state.camera.reproj, " px"));
  setText("ui-image-age", ageText(state.camera.imageAge));
  setText("ui-camera-status", state.camera.status || "NO_FRAME");
  renderCamera(state.camera.image);
  const carla = state.carla;
  setText("ui-carla-conn-pill", carla.connected ? "CONNECTED" : "DISCONNECTED");
  const pill = byId("ui-carla-conn-pill"); if (pill) pill.className = "stream-pill " + (carla.connected ? "stream-pill-online" : "stream-pill-offline");
  setText("ui-carla-endpoint", carla.endpoint || "127.0.0.1:2000"); setText("ui-carla-map", carla.mapName || "—");
  setText("ui-carla-tick", carla.tickHz == null ? "—" : formatNumber(carla.tickHz, 1) + " Hz");
  setText("ui-carla-sync", carla.synchronousMode == null ? "—" : (carla.synchronousMode ? "SYNC" : "ASYNC"));
  const actorSummary = [carla.actorCount == null ? null : carla.actorCount + " total", carla.vehicleCount == null ? null : carla.vehicleCount + " veh", carla.walkerCount == null ? null : carla.walkerCount + " ped", carla.trafficLightCount == null ? null : carla.trafficLightCount + " tl"].filter(Boolean).join(" · ");
  setText("ui-carla-actors", actorSummary || "—"); setText("ui-carla-sensors", Object.entries(carla.sensors || {}).map(([name, status]) => name + ":" + status).join(" · ") || "—");
  renderCarlaThumbnail(carla.thumbnail); setText("ui-ego-pos", formatPoseText(state.ego.x == null ? state.platform : state.ego));
  setText("ui-ego-att", formatAttitudeText(state.ego)); setText("ui-ego-speed", state.ego.speed == null ? "—" : formatNumber(state.ego.speed, 2) + " m/s");
  setText("ui-target-pos-stream", formatPoseText(state.pose)); setText("ui-traffic-state", carla.trafficSummary || actorSummary || "—");
  setText("ui-carla-frame-time", (carla.frame ?? state.frameId ?? "—") + " / " + (carla.timestamp || state.poseTimestamp || "unknown"));
  renderAlgorithms(state.algorithms); renderLogs(state.logs);
  const health = [["uwb", "UWB"], ["vision", "vis"], ["fusion", "fus"]];
  health.forEach(([key, suffix]) => {
    setText(`ui-h-${suffix}`, state.health[key]?.status || "UNKNOWN");
    setText(`ui-h-${suffix}-age`, key === "fusion" ? `Δt ${ageText(state.health[key]?.pairingDelta).replace(" s", "")}` : ageText(state.health[key]?.age));
  });
  setText("ui-h-bridge", state.health.bridge?.status || "UNKNOWN");
  setText("ui-h-seq", `seq ${state.health.bridge?.sequence ?? "—"}`);
  const metrics = state.metrics;
  const hasMetrics = Object.values(metrics).some((value) => value !== null && value !== undefined);
  byId("metrics-empty").hidden = hasMetrics;
  setText("ui-m-rmse", metricText(metrics.rmse));
  setText("ui-m-p95", metricText(metrics.p95));
  setText("ui-m-max", metricText(metrics.max));
  setText("ui-m-availability", metrics.availability == null ? "—" : `${formatNumber(metrics.availability)}%`);
  setText("ui-m-samples", metrics.samples == null ? "—" : String(metrics.samples));
  setText("ui-metric-run", metrics.runId || "—");
  drawLatency(state.latency);
}

store.subscribe(render);
