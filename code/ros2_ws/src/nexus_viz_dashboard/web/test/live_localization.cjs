// Opt-in real browser + external ROS fixture. No NexusAPI state injection.
// NEXUS_PLAYWRIGHT_MODULE points to an already installed Playwright module.
const { chromium } = require(process.env.NEXUS_PLAYWRIGHT_MODULE || "playwright");
const fs = require("node:fs");
const path = require("node:path");
const assert = require("node:assert/strict");

(async () => {
  const output = path.resolve(process.argv[2]);
  const url = process.argv[3] || "http://localhost:18766/public/index.html?rosbridge=ws://localhost:19091";
  const browser = await chromium.launch({ channel: process.env.NEXUS_BROWSER_CHANNEL || "msedge", headless: true });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = [], snapshots = [], sent = [];
  page.on("pageerror", error => errors.push(error.message));
  page.on("websocket", socket => {
    socket.on("framesent", event => { try { sent.push(JSON.parse(event.payload)); } catch {} });
    socket.on("framereceived", event => {
      try {
        const envelope = JSON.parse(event.payload);
        if (envelope.topic === "/nexus/viz/localization_state") snapshots.push(JSON.parse(envelope.msg.data));
      } catch {}
    });
  });
  const phase = value => fs.writeFileSync(path.join(output, "phase.txt"), value);
  const screenshot = name => page.screenshot({ path: path.join(output, `web_${name}.png`), fullPage: true });
  const wait = async predicate => {
    const deadline = Date.now() + 10000;
    while (!predicate() && Date.now() < deadline) await new Promise(resolve => setTimeout(resolve, 50));
    assert.ok(predicate(), "expected real ROS snapshot was not received");
  };
  try {
    await page.goto(url);
    await wait(() => snapshots.length > 0);
    assert.equal(snapshots.at(-1).input_mode, "NO_INPUT");
    assert.equal((await page.locator("#ui-x").innerText()).trim(), "—");
    await screenshot("no_input");

    phase("observed");
    await page.waitForFunction(() => document.getElementById("ui-x").textContent.trim() === "0.40");
    await wait(() => snapshots.some(s => s.targets[0]?.position?.[0] === .4));
    await page.waitForFunction(() => document.getElementById("dual-target-state").textContent === "已确认"
      && document.getElementById("ui-camera-status").textContent === "COMPRESSED");
    assert.equal(snapshots.at(-1).input_mode, "SIMULATION");
    assert.equal(snapshots.at(-1).run_id, "synthetic_live_display_qa");
    await screenshot("observed");

    phase("historical");
    await page.waitForFunction(() => document.getElementById("ui-x").textContent.trim() === "—"
      && document.getElementById("dual-target-history").textContent.includes("0.400"));
    const lastValid = snapshots.at(-1).targets[0].last_valid_sample_timestamp_ns;
    await page.waitForTimeout(700);
    assert.equal(snapshots.at(-1).targets[0].last_valid_sample_timestamp_ns, lastValid);
    assert.equal(snapshots.at(-1).platform.display_state, "observed");
    await screenshot("historical");

    phase("recovered");
    await wait(() => snapshots.some(s => s.targets[0]?.state === "re-associated"));
    await page.waitForFunction(() => document.getElementById("ui-x").textContent.trim() === "0.40");
    await screenshot("recovered");

    phase("outlier");
    await page.waitForFunction(() => document.getElementById("ui-x").textContent.trim() === "—"
      && document.getElementById("dual-target-reason").textContent.includes("outlier"));
    assert.deepEqual(snapshots.at(-1).targets[0].historical_position, [.4, .2, .1]);
    assert.ok(snapshots.every(s => !s.targets[0]?.position || s.targets[0].position[0] === .4));
    await screenshot("outlier");

    phase("pause_all");
    await page.waitForFunction(() => document.getElementById("dual-platform-position").textContent.trim() === "—");
    await screenshot("stale_platform");
    phase("disconnect");
    await page.waitForFunction(() => document.getElementById("dual-target-reason").textContent.includes("DISCONNECTED"));
    assert.equal((await page.locator("#ui-x").innerText()).trim(), "—");
    await screenshot("disconnected");

    assert.deepEqual(errors, []);
    assert.ok(sent.length > 0 && sent.every(message => message.op === "subscribe"));
    fs.writeFileSync(path.join(output, "browser_result.json"), JSON.stringify({ passed: true,
      snapshots: snapshots.length, script_errors: errors, sent_operations: [...new Set(sent.map(m => m.op))],
      phases: ["no_input", "observed", "historical", "re-associated", "outlier", "stale_platform", "disconnected"] }, null, 2));
    console.log("Real ROS2 -> fusion -> display -> rosbridge -> Edge checks passed", snapshots.length, "snapshots");
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exitCode = 1; });
