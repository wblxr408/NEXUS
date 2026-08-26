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
        <section class="panel map-panel" aria-labelledby="map-title">
          <div class="panel-header"><span id="map-title">任务空间 · 目标位置</span><span class="header-meta">MAP / m · DRAG TO ORBIT · WHEEL TO ZOOM</span></div>
          <div id="map-wrapper" class="map-wrapper"><canvas id="map-canvas" aria-label="map 坐标系目标位置；悬停查看物体语义和厘米级坐标"></canvas><div id="map-object-info" class="map-object-info" aria-live="polite"><span class="map-object-kicker">MAP OBJECT / HOVER</span><strong id="map-object-name">沙盘任务空间</strong><span id="map-object-type">地图坐标系：原点在西南角</span><span id="map-object-position" class="map-object-position">x 0.00–4.00 · y 0.00–4.70 · z 向上</span><span id="map-object-note">标称布局坐标；厘米显示不代表实测精度</span></div><div class="map-legend"><span class="legend-fused">● FUSED</span><span class="legend-uwb">● UWB ANCHOR</span><span class="legend-vision">● VISION</span><span class="legend-uav">◆ SIM UAV</span></div></div>
        </section>
        <div class="column column-evidence">
          <section class="panel camera-panel" aria-labelledby="camera-title">
            <div class="panel-header"><span id="camera-title">相机成像与检测证据</span><span class="header-meta">IMX219 / GAZEBO</span></div>
            <div class="camera-content"><div class="camera-viewport"><canvas id="cam-canvas" width="640" height="480" aria-label="相机预览"></canvas><div id="camera-empty" class="empty-state">无相机帧</div></div><dl class="evidence-meta"><div><dt>TOPIC</dt><dd>/nexus/camera/imx219/image_raw/compressed</dd></div><div><dt>CONFIDENCE</dt><dd id="ui-cam-conf">—</dd></div><div><dt>REPROJECTION</dt><dd id="ui-cam-reproj">—</dd></div><div><dt>IMAGE AGE</dt><dd id="ui-image-age">unknown</dd></div><div><dt>STATUS</dt><dd id="ui-camera-status">NO_FRAME</dd></div></dl></div>
          </section>
          <section class="panel latency-panel" aria-labelledby="latency-title"><div class="panel-header"><span id="latency-title">数据年龄 / 延迟趋势</span><span class="header-meta">LAST 60 S</span></div><div class="chart-wrap"><canvas id="latency-canvas" aria-label="最近数据年龄趋势"></canvas><div id="latency-empty" class="empty-state">暂无同步数据</div></div></section>
        </div>
        <div class="column column-diagnostics">
          <section class="panel position-panel" aria-labelledby="position-title"><div class="panel-header"><span id="position-title">当前目标位置</span><span id="ui-frame" class="header-meta">frame: —</span></div><div class="readout-block"><div class="coord-row"><span class="coord-axis axis-x">x</span><strong id="ui-x" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="coord-row"><span class="coord-axis axis-y">y</span><strong id="ui-y" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="coord-row"><span class="coord-axis axis-z">z</span><strong id="ui-z" class="coord-val">—</strong><span class="coord-unit">m</span></div><div class="subreadout">采样时间 <span id="ui-pose-time">unknown</span></div></div></section>
          <section class="panel health-panel" aria-labelledby="health-title"><div class="panel-header"><span id="health-title">链路与融合健康</span><span class="header-meta">/SENSOR_HEALTH</span></div><div class="health-list"><div class="health-row"><span class="health-name axis-x">UWB</span><strong id="ui-h-uwb">NO_INPUT</strong><span id="ui-h-uwb-age">unknown</span></div><div class="health-row"><span class="health-name axis-y">VISION</span><strong id="ui-h-vis">NO_INPUT</strong><span id="ui-h-vis-age">unknown</span></div><div class="health-row"><span class="health-name axis-z">FUSION</span><strong id="ui-h-fus">NO_INPUT</strong><span id="ui-h-fus-age">Δt unknown</span></div><div class="health-row"><span class="health-name">BRIDGE</span><strong id="ui-h-bridge">DISCONNECTED</strong><span id="ui-h-seq">seq —</span></div></div></section>
          <section class="panel experiment-panel" aria-labelledby="experiment-title"><div class="panel-header"><span id="experiment-title">已登记实验指标</span><span class="header-meta">/EVIDENCE</span></div><div id="metrics-empty" class="metrics-empty">无已登记实测指标</div><dl class="exp-grid"><div><dt>3D RMSE</dt><dd id="ui-m-rmse">—</dd></div><div><dt>P95</dt><dd id="ui-m-p95">—</dd></div><div><dt>MAX ERROR</dt><dd id="ui-m-max">—</dd></div><div><dt>AVAILABLE</dt><dd id="ui-m-availability">—</dd></div><div><dt>SAMPLES (N)</dt><dd id="ui-m-samples">—</dd></div><div><dt>RUN</dt><dd id="ui-metric-run">—</dd></div></dl></section>
        </div>
      </main>
    </div>`;
}
