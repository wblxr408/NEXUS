import { dualLocalizationMarkup } from "./dual_localization.js";
export function dashboardLayout() {
  return `
    <div class="nexus-container">
      <header class="topbar">
        <div class="brand-name">NEXUS <span>3D LOCALIZATION</span></div>
        <div class="status-ribbon" aria-label="会话状态">
          <div class="status-block"><span class="status-label">INPUT MODE</span><strong id="ui-mode" class="status-value status-chip">NO INPUT</strong></div>
          <div class="status-block"><span class="status-label">RUN ID</span><strong id="ui-run-id" class="status-value">UNREGISTERED</strong></div>
          <div class="status-block"><span class="status-label">TARGET ID</span><strong id="ui-target-id" class="status-value">未检测到目标</strong></div>
          <div class="status-block"><span class="status-label">DATA AGE</span><strong id="ui-global-age" class="status-value">unknown</strong></div>
          <div class="status-block"><span class="status-label">SOURCE MODE</span><strong id="ui-source" class="status-value source-fused">UNKNOWN</strong></div>
          <div class="status-block"><span class="status-label">SESSION</span><strong id="ui-session" class="status-value">WAITING_FOR_INPUT</strong></div>
        </div>
      </header>
      <main class="dashboard-grid">
        <section class="panel data-panel" aria-labelledby="data-title">
          <div class="panel-header"><span id="data-title">CARLA 实时数据流</span><span class="header-meta">WINDOWS GPU RENDER / WSL JSON BRIDGE</span></div>
          <div class="data-stream-panel">
            ${dualLocalizationMarkup}
            <section class="stream-card stream-card-primary" aria-labelledby="carla-connection-title">
              <div class="stream-card-header">
                <span id="carla-connection-title">连接状态</span>
                <span id="ui-carla-conn-pill" class="stream-pill stream-pill-offline">DISCONNECTED</span>
              </div>
              <div class="stream-primary-grid">
                <div class="thumbnail-wrap">
                  <img id="ui-carla-thumb" class="thumbnail-image" alt="最近一帧 CARLA 相机缩略图" hidden />
                  <div id="ui-carla-thumb-empty" class="empty-state">暂无 CARLA 缩略图</div>
                </div>
                <dl class="stream-kv">
                  <div><dt>RPC</dt><dd id="ui-carla-endpoint">127.0.0.1:2000</dd></div>
                  <div><dt>MAP</dt><dd id="ui-carla-map">—</dd></div>
                  <div><dt>TICK</dt><dd id="ui-carla-tick">—</dd></div>
                  <div><dt>MODE</dt><dd id="ui-carla-sync">—</dd></div>
                  <div><dt>ACTORS</dt><dd id="ui-carla-actors">—</dd></div>
                  <div><dt>SENSORS</dt><dd id="ui-carla-sensors">—</dd></div>
                </dl>
              </div>
            </section>
            <div class="stream-grid">
              <section class="stream-card" aria-labelledby="pose-stream-title">
                <div class="stream-card-header"><span id="pose-stream-title">实时位姿与状态流</span><span class="header-meta">EGO / TARGET / ACTORS</span></div>
                <dl class="stream-kv">
                  <div><dt>EGO POS</dt><dd id="ui-ego-pos">—</dd></div>
                  <div><dt>EGO ATT</dt><dd id="ui-ego-att">—</dd></div>
                  <div><dt>EGO SPD</dt><dd id="ui-ego-speed">—</dd></div>
                  <div><dt>TARGET</dt><dd id="ui-target-pos-stream">—</dd></div>
                  <div><dt>TRAFFIC</dt><dd id="ui-traffic-state">—</dd></div>
                  <div><dt>FRAME / TS</dt><dd id="ui-carla-frame-time">—</dd></div>
                </dl>
              </section>
              <section class="stream-card" aria-labelledby="algorithm-stream-title">
                <div class="stream-card-header"><span id="algorithm-stream-title">算法输出流</span><span class="header-meta">ORB-SLAM2 / UWB / GDR-NET / FUSION</span></div>
                <div id="ui-algorithm-list" class="algorithm-list" aria-live="polite"></div>
              </section>
            </div>
            <section class="stream-card stream-log-card" aria-labelledby="log-stream-title">
              <div class="stream-card-header"><span id="log-stream-title">日志流</span><span class="header-meta">CARLA / SENSORS / FUSION</span></div>
              <div id="ui-log-list" class="log-list" aria-live="polite"></div>
            </section>
          </div>
        </section>
        <div class="column column-evidence">
          <section class="panel camera-panel" aria-labelledby="camera-title">
            <div class="panel-header"><span id="camera-title">相机成像与检测证据</span><span class="header-meta">IMX219 / GAZEBO</span></div>
            <div class="camera-content"><div class="camera-viewport"><canvas id="cam-canvas" width="640" height="480" aria-label="相机预览"></canvas><div id="camera-empty" class="empty-state">无相机帧</div></div><dl class="evidence-meta"><div><dt>TOPIC</dt><dd>/nexus/camera/imx219/image_raw/compressed</dd></div><div><dt>CONFIDENCE</dt><dd id="ui-cam-conf">—</dd></div><div><dt>REPROJECTION</dt><dd id="ui-cam-reproj">—</dd></div><div><dt>IMAGE AGE</dt><dd id="ui-image-age">unknown</dd></div><div><dt>STATUS</dt><dd id="ui-camera-status">NO_FRAME</dd></div></dl></div>
          </section>
          <section class="panel latency-panel" aria-labelledby="latency-title"><div class="panel-header"><span id="latency-title">数据年龄 / 延迟趋势</span><span class="header-meta">LAST 60 S</span></div><div class="chart-wrap"><canvas id="latency-canvas" aria-label="最近数据年龄趋势"></canvas><div id="latency-empty" class="empty-state">暂无同步数据</div></div></section>
        </div>
        <div class="column column-diagnostics">
          <section class="panel position-panel" aria-labelledby="position-title"><div class="panel-header"><span id="position-title">当前目标位置</span><span id="ui-frame" class="header-meta">frame: —</span></div><div class="readout-block"><div class="coord-row"><span class="coord-axis axis-x">x</span><strong id="ui-x" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="coord-row"><span class="coord-axis axis-y">y</span><strong id="ui-y" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="coord-row"><span class="coord-axis axis-z">z</span><strong id="ui-z" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="subreadout">采样时间 <span id="ui-pose-time">unknown</span></div><dl class="evidence-meta"><div><dt>σx</dt><dd id="ui-sigma-x">—</dd></div><div><dt>σy</dt><dd id="ui-sigma-y">—</dd></div><div><dt>σz</dt><dd id="ui-sigma-z">—</dd></div><div><dt>N VIEWS</dt><dd id="ui-n-views">—</dd></div><div><dt>BASELINE</dt><dd id="ui-baseline">—</dd></div><div><dt>DEPTH SOURCE</dt><dd id="ui-depth-source">—</dd></div><div><dt>CHAIN</dt><dd id="ui-chain">—</dd></div></dl></div></section>
          <section class="panel health-panel" aria-labelledby="health-title"><div class="panel-header"><span id="health-title">链路与融合健康</span><span class="header-meta">/SENSOR_HEALTH</span></div><div class="health-list"><div class="health-row"><span class="health-name axis-x">UWB</span><strong id="ui-h-uwb">NO_INPUT</strong><span id="ui-h-uwb-age">unknown</span></div><div class="health-row"><span class="health-name axis-y">VISION</span><strong id="ui-h-vis">NO_INPUT</strong><span id="ui-h-vis-age">unknown</span></div><div class="health-row"><span class="health-name axis-z">FUSION</span><strong id="ui-h-fus">NO_INPUT</strong><span id="ui-h-fus-age">Δt unknown</span></div><div class="health-row"><span class="health-name">BRIDGE</span><strong id="ui-h-bridge">DISCONNECTED</strong><span id="ui-h-seq">seq —</span></div></div></section>
          <section class="panel experiment-panel" aria-labelledby="experiment-title"><div class="panel-header"><span id="experiment-title">已登记实验指标</span><span class="header-meta">/EVIDENCE</span></div><div id="metrics-empty" class="metrics-empty">无已登记实测指标</div><dl class="exp-grid"><div><dt>3D RMSE</dt><dd id="ui-m-rmse">—</dd></div><div><dt>P95</dt><dd id="ui-m-p95">—</dd></div><div><dt>MAX ERROR</dt><dd id="ui-m-max">—</dd></div><div><dt>AVAILABLE</dt><dd id="ui-m-availability">—</dd></div><div><dt>SAMPLES (N)</dt><dd id="ui-m-samples">—</dd></div><div><dt>RUN</dt><dd id="ui-metric-run">—</dd></div></dl></section>
        </div>
      </main>
    </div>`;
}
