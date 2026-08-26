import { dashboardLayout } from "../components/dashboard_layout.js";
import { createSceneRenderer } from "../features/map/scene_renderer.js";
import { createDashboardStore } from "../features/telemetry/dashboard_store.js";
import { installNexusAPI } from "../services/nexus_api.js";
import { formatNumber } from "../types/dashboard_types.js";

const root = document.querySelector("#nexus-app");
const store = createDashboardStore();
root.innerHTML = dashboardLayout();
installNexusAPI(store);

const byId = (id) => document.getElementById(id);
const setText = (id, value) => { const element = byId(id); if (element) element.textContent = value; };
const ageText = (value) => value === null || value === undefined ? "unknown" : `${formatNumber(value, 2)} s`;
const metricText = (value, suffix = " m") => value === null || value === undefined ? "—" : `${formatNumber(value)}${suffix}`;
let renderedCameraImage = null;

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
createSceneRenderer(byId("map-canvas"), byId("map-wrapper"), store);
