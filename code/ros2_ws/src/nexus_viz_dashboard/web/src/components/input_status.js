import { INPUT_TOPICS } from "../services/input_status.js";
export const inputStatusMarkup = '<section class="stream-card input-state-card" aria-label="当前输入与算法状态">'
  + '<div class="stream-card-header"><span>当前输入与算法状态</span><span class="header-meta">YOLO11 / SUPERPOINT</span></div>'
  + '<div id="input-state-list" class="input-state-list"></div>'
  + '<p class="dual-note">“收到数据”仅表示通信有输入；定位是否有效以输出状态为准。时间为距浏览器上次收包的间隔。</p></section>';
export function renderInputStatus(inputs) {
  const list = document.getElementById("input-state-list");
  list.replaceChildren(...INPUT_TOPICS.map(spec => {
    const value = inputs[spec.key];
    const row = document.createElement("div"); row.className = "input-state-row";
    const label = document.createElement("strong"); label.textContent = spec.label;
    const state = document.createElement("span");
    state.textContent = !value ? "等待输入" : value.disconnected ? "连接断开" : value.stale ? "数据中断"
      : value.accepted ? "收到数据" : "无效 / 拒绝";
    state.className = value && !value.stale && !value.disconnected && value.accepted ? "input-ok" : "input-wait";
    const detail = document.createElement("small"); detail.textContent = value
      ? value.detail + " · " + value.arrivalAgeS.toFixed(1) + " s" : spec.topic;
    row.append(label, state, detail); return row;
  }));
}
