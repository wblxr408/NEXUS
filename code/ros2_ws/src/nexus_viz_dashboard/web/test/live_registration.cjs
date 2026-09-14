// Real browser -> rosbridge -> SuperPoint GPU registration fixture. No UI state injection.
const { chromium } = require(process.env.NEXUS_PLAYWRIGHT_MODULE || "playwright");
const fs = require("node:fs");
const path = require("node:path");
const assert = require("node:assert/strict");

(async () => {
  const output = process.argv[2];
  const url = process.argv[3] || "http://127.0.0.1:18768/public/index.html?rosbridge=ws://127.0.0.1:19093";
  const browser = await chromium.launch({ channel: "msedge", headless: true });
  const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
  const errors = [], operations = [], references = [];
  page.on("pageerror", error => errors.push(error.message));
  page.on("websocket", socket => socket.on("framesent", event => {
    try {
      const message = JSON.parse(event.payload);
      operations.push({ op: message.op, topic: message.topic });
      if (message.op === "publish" && message.topic === "/nexus/vision/target_requests")
        references.push(JSON.parse(message.msg.data));
    } catch {}
  }));
  const phase = value => fs.writeFileSync(path.join(output, "phase.txt"), value);
  const capture = name => page.screenshot({ path: path.join(output, name + ".png"), fullPage: true });
  const drag = async () => {
    const canvas = page.locator("#target-selection-canvas");
    const box = await canvas.boundingBox();
    await page.mouse.move(box.x + box.width * .2, box.y + box.height * .2);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width * .8, box.y + box.height * .8, { steps: 8 });
    await page.mouse.up();
  };
  const freeze = async () => {
    await page.locator("#target-freeze").click();
    await page.waitForFunction(() => document.getElementById("target-frame-info").textContent.includes("已冻结"), { timeout: 15000 });
  };
  try {
    await page.goto(url);
    await page.waitForFunction(() => document.getElementById("ui-camera-status").textContent === "COMPRESSED");
    assert.equal((await page.locator("#ui-mode").innerText()).trim(), "SIMULATION");
    assert.equal(await page.locator("#ui-carla-map").isVisible(), false);
    await capture("dashboard");
    await freeze();
    const dimensions = await page.locator("#target-selection-canvas").evaluate(c => [c.width, c.height]);
    assert.deepEqual(dimensions, [1640, 1232]);
    const frozenStamp = await page.locator("#target-frame-info").innerText();
    await drag();
    await page.locator("#target-tool-anchor").click();
    const bounds = await page.locator("#target-selection-canvas").boundingBox();
    await page.mouse.click(bounds.x + bounds.width * .5, bounds.y + bounds.height * .5);
    await page.locator("#target-name").fill("web_chosen");
    await capture("selection");
    await page.waitForTimeout(1100);
    assert.equal(await page.locator("#target-frame-info").innerText(), frozenStamp);
    await page.locator("#target-confirm").click();
    await page.waitForFunction(() => !document.getElementById("target-dialog").open, { timeout: 20000 });
    assert.match(await page.locator("#target-registration-result").innerText(), /已注册 web_chosen/);
    assert.equal(references.length, 1);
    for (let i = 0; i < 4; i++)
      assert.ok(Math.abs(references[0].bbox_xywh_px[i] - [328, 246, 984, 740][i]) <= 2);
    assert.ok(Math.abs(references[0].anchor_reference_px[0] - 820) < 2);
    await capture("registered");

    phase("blank"); await page.waitForTimeout(1600);
    await freeze(); await drag();
    await page.locator("#target-name").fill("blank_rejected");
    await page.locator("#target-confirm").click();
    await page.waitForFunction(() => document.getElementById("target-selection-status").textContent.includes("insufficient"), { timeout: 20000 });
    assert.equal(await page.locator("#target-dialog").evaluate(d => d.open), true);
    assert.doesNotMatch(await page.locator("#target-registration-result").innerText(), /已注册 blank_rejected/);
    await capture("rejected");
    await page.locator("#target-close").click();

    phase("pause");
    await page.waitForFunction(() => document.getElementById("ui-camera-status").textContent === "STALE");
    assert.equal(await page.locator("#camera-empty").isVisible(), true);
    await page.locator("#target-freeze").click();
    await page.locator("#target-close").click(); // cancel pending capture
    assert.equal(await page.locator("#target-dialog").evaluate(d => d.open), false);
    await page.setViewportSize({ width: 390, height: 844 });
    await capture("mobile");
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth + 1), true);
    assert.deepEqual(errors, []);
    const publications = operations.filter(m => m.op === "publish");
    assert.ok(publications.every(m => ["/nexus/vision/target_reference_image", "/nexus/vision/target_requests"].includes(m.topic)));
    // Harmless isolated test topic proves that the server does not permit general publication.
    await new Promise((resolve, reject) => {
      const socket = new WebSocket(new URL(url).searchParams.get("rosbridge"));
      socket.addEventListener("error", reject);
      socket.addEventListener("open", () => {
        socket.send(JSON.stringify({ op: "advertise", id: "qa_denied_type", topic: "/nexus/web_qa/blocked", type: "std_msgs/msg/String" }));
        socket.send(JSON.stringify({ op: "publish", id: "qa_denied_publish", topic: "/nexus/web_qa/blocked", msg: { data: "synthetic boundary check" } }));
        setTimeout(() => { socket.close(); resolve(); }, 750);
      });
    });
    assert.match(fs.readFileSync(path.join(output, "server_0.log"), "utf8"), /cancelling publish to: \/nexus\/web_qa\/blocked/);
    const events = fs.readFileSync(path.join(output, "events.jsonl"), "utf8").trim().split("\n").map(JSON.parse);
    assert.equal(events.filter(e => e.kind === "blocked_received").length, 0);
    const images = events.filter(e => e.kind === "reference");
    assert.equal(images.length, 2);
    assert.ok(images.every(e => e.data.matches_emitted));
    assert.ok(events.some(e => e.kind === "status" && e.data.target_id === "web_chosen" && e.data.state === "registered"));
    assert.ok(events.some(e => e.kind === "status" && e.data.target_id === "blank_rejected" && e.data.state === "rejected"));
    fs.writeFileSync(path.join(output, "browser_result.json"), JSON.stringify({
      passed: true, errors, operations, original_image_integrity: images.map(e => e.data),
      scenarios: ["GPU registration", "scaled ROI", "anchor", "frozen timestamp", "backend rejection", "stale camera", "cancel", "mobile", "publication allowlist"] }, null, 2));
    console.log("Browser/ROS/GPU target registration checks passed");
  } catch (error) {
    await capture("failure");
    fs.writeFileSync(path.join(output, "browser_failure.json"), JSON.stringify({
      error: error.message, page_errors: errors, operations,
      selection_status: await page.locator("#target-selection-status").textContent() }, null, 2));
    throw error;
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exitCode = 1; });
