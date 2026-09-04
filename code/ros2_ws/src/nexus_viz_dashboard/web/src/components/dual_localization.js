import { formatNumber } from "../types/dashboard_types.js";
import { formatStamp } from "../features/telemetry/localization_state.js";

export const dualLocalizationMarkup = `
  <section class="stream-card dual-state-card" aria-label="无人机与静态目标定位">
    <div class="stream-card-header"><span>无人机 · 实际估计状态</span><span id="dual-platform-state" class="stream-pill">等待输入</span></div>
    <dl class="dual-platform-grid">
      <div><dt>位置 / map · m</dt><dd id="dual-platform-position">—</dd></div>
      <div><dt>实际速度 · m/s</dt><dd id="dual-platform-speed">—</dd></div>
      <div><dt>姿态四元数 / xyzw</dt><dd id="dual-platform-orientation">—</dd></div>
      <div><dt>数据年龄 · 服务端时钟</dt><dd id="dual-platform-age">—</dd></div>
    </dl>
    <div class="dual-map-wrap"><canvas id="dual-map" aria-label="map 坐标平面，无人机、当前目标与历史位置"></canvas></div>
    <div class="dual-legend"><span>● 无人机</span><span>◆ 当前目标</span><span>○ 历史位置</span><span>坐标单位 m</span></div>
    <div class="dual-target-toolbar"><label for="dual-target-select">静态目标</label><select id="dual-target-select"><option value="">等待目标</option></select><strong id="dual-target-state">当前未观测</strong></div>
    <dl class="stream-kv dual-target-meta">
      <div><dt>历史位置 / m</dt><dd id="dual-target-history">—</dd></div>
      <div><dt>最后有效采样</dt><dd id="dual-target-time">—</dd></div>
      <div><dt>定位参考点</dt><dd id="dual-target-reference">—</dd></div>
      <div><dt>状态原因</dt><dd id="dual-target-reason">—</dd></div>
    </dl>
    <p class="dual-note">目标保持静止；历史位置不代表当前观测。无人机状态只读。目标 σ 按平台位姿与安装变换已知计算。</p>
  </section>`;

const labels = { observed: "当前观测", confirmed: "已确认", tentative: "待确认", degraded: "质量下降",
  "re-associated": "重新识别", historical: "历史位置 · 当前未观测", lost: "丢失 · 当前未观测",
  stale: "数据过期", no_input: "等待输入" };
const xyz = (values) => values ? values.map((value) => formatNumber(value, 3)).join(" / ") : "—";

export function renderDualLocalization(state) {
  const text = (id, value) => { document.getElementById(id).textContent = value; };
  const active = Boolean(state.dual) || state.mode === "NO_INPUT";
  document.querySelector(".data-stream-panel").classList.toggle("dual-active", active);
  document.querySelector(".nexus-container").classList.toggle("has-dual-state", active);
  text("data-title", active ? "无人机与静态目标定位" : "CARLA 实时数据流");
  document.querySelector(".data-panel > .panel-header .header-meta").textContent = active ? "MAP / READ ONLY" : "WINDOWS GPU RENDER / WSL JSON BRIDGE";
  const data = state.dual?.snapshot;
  const live = state.dual?.connection === "CONNECTED";
  const platform = data?.platform;
  text("dual-platform-state", live ? labels[platform.display_state] ?? platform.display_state : "等待有效状态");
  text("dual-platform-position", xyz(live ? platform.position : null));
  text("dual-platform-speed", live && platform.speed_mps !== null ? formatNumber(platform.speed_mps, 3) : "—");
  text("dual-platform-orientation", xyz(live ? platform.orientation : null));
  text("dual-platform-age", live && platform.age_s !== null ? `${formatNumber(platform.age_s, 2)} s` : "unknown");
  const select = document.getElementById("dual-target-select");
  const ids = data?.targets.map((target) => target.target_id) ?? [];
  if (JSON.stringify(Array.from(select.options).map((option) => option.value)) !== JSON.stringify(ids.length ? ids : [""])) {
    select.replaceChildren(...(ids.length ? ids : [""]).map((id) => {
      const option = document.createElement("option"); option.value = id; option.textContent = id || "等待目标"; return option;
    }));
  }
  select.disabled = !ids.length;
  select.value = state.targetId ?? "";
  const target = data?.targets.find((item) => item.target_id === state.targetId);
  const current = live && target?.position !== null && target?.position !== undefined;
  const historyPosition = current ? null : target?.historical_position ?? target?.position;
  const targetLabel = current ? labels[target.state] ?? target.state
    : target?.state === "lost" || !historyPosition ? labels.lost : labels.historical;
  text("dual-target-state", target ? targetLabel : "等待目标");
  text("dual-target-history", xyz(historyPosition));
  text("dual-target-time", formatStamp(target?.last_valid_sample_timestamp_ns) ?? "unknown");
  text("dual-target-reference", target?.position_reference || "—");
  text("dual-target-reason", !live && data ? `通信状态：${state.dual.connection}` : target?.reason || "—");
  drawMap(data, live, state.targetId);
}

function drawMap(data, live, selectedId) {
  const canvas = document.getElementById("dual-map");
  const width = canvas.clientWidth, height = canvas.clientHeight;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(width * ratio)); canvas.height = Math.max(1, Math.round(height * ratio));
  const ctx = canvas.getContext("2d"); ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if (!width || !height) return;
  const points = (data?.targets ?? []).flatMap((target) => {
    const position = target.position ?? target.historical_position;
    return position ? [{ position, label: target.target_id, history: !live || !target.position, selected: target.target_id === selectedId }] : [];
  });
  if (live && data?.platform.position) points.push({ position: data.platform.position, label: "UAV", platform: true });
  ctx.font = "12px monospace";
  if (!points.length) { ctx.fillStyle = "#737770"; ctx.textAlign = "center"; ctx.fillText("等待 map 坐标输入", width / 2, height / 2); return; }
  const xs = [0, ...points.map((item) => item.position[0])], ys = [0, ...points.map((item) => item.position[1])];
  const xmin = Math.min(...xs) - .5, xmax = Math.max(...xs) + .5, ymin = Math.min(...ys) - .5, ymax = Math.max(...ys) + .5;
  const scale = Math.max(.001, Math.min((width - 90) / (xmax - xmin), (height - 65) / (ymax - ymin)));
  const px = (x) => 50 + (x - xmin) * scale, py = (y) => height - 30 - (y - ymin) * scale;
  ctx.strokeStyle = "#d2d4cd"; ctx.fillStyle = "#6b7068"; ctx.textAlign = "left";
  for (let i = 0; i <= 4; i++) {
    const x = xmin + (xmax - xmin) * i / 4, y = ymin + (ymax - ymin) * i / 4;
    ctx.beginPath(); ctx.moveTo(px(x), py(ymin)); ctx.lineTo(px(x), py(ymax)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px(xmin), py(y)); ctx.lineTo(px(xmax), py(y)); ctx.stroke();
    ctx.fillText(x.toFixed(1), px(x) - 10, height - 12); ctx.fillText(y.toFixed(1), 4, py(y) + 4);
  }
  ctx.fillText("x / m", width - 50, height - 12); ctx.fillText("y / m", 4, 14);
  points.forEach((item) => {
    const x = px(item.position[0]), y = py(item.position[1]);
    ctx.strokeStyle = ctx.fillStyle = item.platform ? "#2d6fa3" : item.history ? "#a56b18" : "#2b817d";
    ctx.lineWidth = item.selected ? 3 : 1.5;
    ctx.beginPath(); ctx.arc(x, y, item.selected ? 7 : 5, 0, Math.PI * 2);
    if (item.history) ctx.stroke(); else ctx.fill();
    const label = item.label + (item.history ? " · 历史" : "");
    const labelWidth = ctx.measureText(label).width;
    ctx.fillText(label, Math.max(4, Math.min(x + 10, width - labelWidth - 4)), y - 10, width - 8);
  });
}
