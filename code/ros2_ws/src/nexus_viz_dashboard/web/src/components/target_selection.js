import { imagePixels, imagePoint, selectionBox, frameStamp } from "../services/target_registration.js";
import { formatStamp } from "../features/telemetry/localization_state.js";

export const targetSelectionMarkup = [
  '<div class="target-selection-entry"><button id="target-freeze" type="button">定格并指定目标</button>',
  '<span id="target-registration-result" role="status">选择静态目标后，系统提取外观特征并注册。</span></div>',
  '<dialog id="target-dialog" aria-labelledby="target-dialog-title">',
  '<div class="selection-heading"><h2 id="target-dialog-title">指定要定位的目标</h2><button id="target-close" type="button">关闭</button></div>',
  '<p class="selection-help">① 拖动框住一个目标　② 可在框内点选定位参考点　③ 命名并确认</p>',
  '<div class="selection-image-wrap"><canvas id="target-selection-canvas" aria-label="冻结的定位图像，拖动框选目标"></canvas></div>',
  '<p id="target-frame-info" class="selection-help">正在等待定位图像…</p>',
  '<div class="selection-tools"><button id="target-tool-box" type="button" aria-pressed="true">框选目标</button>',
  '<button id="target-tool-anchor" type="button" aria-pressed="false">点选参考点</button><button id="target-clear" type="button">清除选择</button></div>',
  '<div class="selection-fields"><label>目标名称<input id="target-name" placeholder="例如 target_01" maxlength="96" autocomplete="off"></label>',
  '<label>绑定已有轨迹（可选）<select id="target-existing"><option value="">注册为新目标</option></select></label></div>',
  '<p class="selection-help">参考点会吸附到附近特征；未点选时取靠近框中心的特征，不代表物体几何中心。注册在节点本次运行期间有效。</p>',
  '<div class="selection-footer"><span id="target-selection-status" role="status">等待图像</span>',
  '<button id="target-confirm" type="button" disabled>确认注册</button></div></dialog>',
].join("");

export function installTargetSelection(registration, store) {
  const el = id => document.getElementById(id);
  const dialog = el("target-dialog"), canvas = el("target-selection-canvas");
  let frame = null, base = null, box = null, anchor = null, start = null, tool = "box", busy = false, generation = 0;
  const status = message => { el("target-selection-status").textContent = message; };
  const update = () => {
    el("target-confirm").disabled = busy || !frame || !box || !el("target-name").value.trim();
    for (const id of ["target-clear", "target-tool-box", "target-tool-anchor", "target-name", "target-existing"])
      el(id).disabled = busy || !frame;
    el("target-close").disabled = busy;
  };
  const paint = () => {
    const context = canvas.getContext("2d");
    context.clearRect(0, 0, canvas.width, canvas.height);
    if (!base) return;
    context.drawImage(base, 0, 0);
    const scale = canvas.width / Math.max(1, canvas.getBoundingClientRect().width);
    context.lineWidth = 2 * scale;
    context.strokeStyle = "#38d7ae";
    if (box) { context.fillStyle = "rgba(56,215,174,.12)"; context.fillRect(...box); context.strokeRect(...box); }
    if (anchor) {
      context.strokeStyle = "#ffce70"; context.beginPath();
      context.arc(anchor[0], anchor[1], 6 * scale, 0, Math.PI * 2); context.stroke();
    }
  };
  const setTool = next => {
    tool = next;
    el("target-tool-box").setAttribute("aria-pressed", String(next === "box"));
    el("target-tool-anchor").setAttribute("aria-pressed", String(next === "anchor"));
  };
  const close = () => {
    if (busy) return;
    generation += 1; registration.cancelCapture(); dialog.close();
    frame = base = box = anchor = start = null;
  };
  el("target-close").addEventListener("click", close);
  dialog.addEventListener("cancel", event => { event.preventDefault(); close(); });
  el("target-freeze").addEventListener("click", async () => {
    const attempt = ++generation;
    frame = base = box = anchor = start = null; busy = false; setTool("box");
    el("target-name").value = "";
    const existing = el("target-existing");
    existing.replaceChildren(new Option("注册为新目标", ""));
    for (const target of store.getState().dual?.snapshot.targets ?? [])
      existing.add(new Option(target.target_id, target.target_id));
    el("target-frame-info").textContent = "等待去畸变图像；后台采集继续。";
    status("正在定格…"); dialog.showModal(); update(); paint();
    try {
      const received = await registration.captureFrame();
      if (attempt !== generation) return;
      const { bytes, channels } = imagePixels(received);
      base = document.createElement("canvas");
      base.width = canvas.width = received.width; base.height = canvas.height = received.height;
      const context = base.getContext("2d"), pixels = context.createImageData(base.width, base.height);
      for (let y = 0; y < base.height; y++) for (let x = 0; x < base.width; x++) {
        const source = y * received.step + x * channels, dest = (y * base.width + x) * 4;
        pixels.data[dest] = received.encoding === "bgr8" ? bytes[source + 2] : bytes[source];
        pixels.data[dest + 1] = channels === 1 ? bytes[source] : bytes[source + 1];
        pixels.data[dest + 2] = channels === 1 ? bytes[source] : received.encoding === "bgr8" ? bytes[source] : bytes[source + 2];
        pixels.data[dest + 3] = 255;
      }
      context.putImageData(pixels, 0, 0); frame = received;
      el("target-frame-info").textContent = received.width + " × " + received.height + " · " + received.header.frame_id
        + " · 采样 " + formatStamp(frameStamp(received)) + " · 已冻结";
      status("请拖动鼠标框住一个静态目标"); paint(); update();
    } catch (error) {
      if (attempt === generation) { status(error.message); update(); }
    }
  });
  const point = event => imagePoint(event.clientX, event.clientY, canvas.getBoundingClientRect(), frame.width, frame.height);
  canvas.addEventListener("pointerdown", event => {
    if (!frame || busy || event.button !== 0) return;
    event.preventDefault();
    if (tool === "anchor") {
      const p = point(event);
      if (!box || p[0] < box[0] || p[0] > box[0] + box[2] || p[1] < box[1] || p[1] > box[1] + box[3]) {
        status("请先框选目标，再在框内点击参考点"); return;
      }
      anchor = p; status("已指定参考点；请命名并确认注册"); paint(); return;
    }
    start = point(event); box = anchor = null; canvas.setPointerCapture(event.pointerId); update();
  });
  canvas.addEventListener("pointermove", event => {
    if (!start || busy) return;
    box = selectionBox(start, point(event)); paint();
  });
  canvas.addEventListener("pointerup", event => {
    if (!start || busy) return;
    box = selectionBox(start, point(event)); start = null;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    if (box[2] < 4 || box[3] < 4) { box = null; status("框太小，请重新拖动框选"); }
    else status("已框选 " + box[2] + " × " + box[3] + " 像素；可指定参考点并填写目标名称");
    paint(); update();
  });
  canvas.addEventListener("pointercancel", () => { start = box = null; paint(); update(); });
  el("target-tool-box").addEventListener("click", () => setTool("box"));
  el("target-tool-anchor").addEventListener("click", () => setTool("anchor"));
  el("target-clear").addEventListener("click", () => { box = anchor = start = null; setTool("box"); status("请重新框选"); paint(); update(); });
  el("target-name").addEventListener("input", update);
  el("target-existing").addEventListener("change", event => {
    if (event.target.value) el("target-name").value = event.target.value;
    update();
  });
  el("target-confirm").addEventListener("click", async () => {
    if (busy || !frame || !box) return;
    const identifier = el("target-name").value.trim(), existing = el("target-existing").value || null;
    const tracks = store.getState().dual?.snapshot.targets ?? [];
    if (tracks.some(target => target.target_id === identifier) && existing !== identifier) {
      status("该名称已有轨迹，请在绑定列表中选择它或使用新名称"); return;
    }
    busy = true; update(); status("已提交，等待后端注册回执…");
    try {
      await registration.register(frame, box, identifier, anchor, existing);
      el("target-registration-result").textContent = "已注册 " + identifier + "；等待实时匹配和有效定位。";
      store.preferTarget(identifier);
      busy = false; close();
    } catch (error) {
      busy = false; status(error.message); el("target-registration-result").textContent = error.message; update();
    }
  });
}
