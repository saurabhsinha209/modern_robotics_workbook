// SO-101 phone teleop: stream viewer pose + clutch + gripper% + orient-lock,
// send episode events (start / success / fail / discard), show robot status.
const $ = (id) => document.getElementById(id);

let ws = null, xrSession = null;
let clutch = false;            // toggle — the ratchet
let orientLock = true;         // true: ignore phone rotation
let gripper = 50;              // 0 closed .. 100 open (continuous)
let recording = false, sentCount = 0;

const send = (o) => { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); };
const evt = (name, extra = {}) => send({ type: "event", name, ...extra });
function setState(on, text) { $("dot").classList.toggle("on", on); $("state").textContent = text; }

function connect() {
  ws = new WebSocket(`wss://${location.host}/ws`);
  ws.onopen = () => { ws.send(JSON.stringify({ type: "hello", role: "xr" })); setState(true, "connected"); };
  ws.onclose = () => { setState(false, "disconnected — retrying…"); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "status") applyStatus(m);
  };
}

function chip(el, text, cls) { el.textContent = text; el.className = "chip " + cls; }
function applyStatus(m) {
  if (m.instruction !== undefined && document.activeElement !== $("instr") && !recording)
    $("instr").value = m.instruction;
  if (m.state !== undefined) {
    recording = m.state === "recording";
    chip($("epchip"), recording ? `● REC ${m.ep_t.toFixed(0)}s · #${m.ep}` : `IDLE · next #${m.ep}`,
         recording ? "rec" : "idle");
    $("instr").disabled = $("tgt").disabled = $("dst").disabled = recording;
    $("startEp").disabled = recording;
    $("succBtn").disabled = !recording;
    $("failBtn").disabled = !recording;
    $("discardBtn").disabled = !recording;
  }
  if (m.auto !== undefined) {
    $("nextBtn").style.display = ""; $("autochip").style.display = "";
    const t = { planning: ["PLANNING…", "warn"], ready: [`${m.auto_target} → ${m.auto_dest}${m.auto_masks ? "" : " (no mask)"}`, "ok"],
                failed: ["PLAN FAILED · manual", "bad"], idle: ["REARRANGE → NEXT TASK", "idle"] }[m.auto] || ["", "idle"];
    chip($("autochip"), t[0], t[1]);
    $("autochip").title = m.auto_err || "";
  }
  if (m.ik_ok !== undefined) {
    if (m.homing) chip($("ikchip"), "HOMING", "warn");
    else if (m.resynced) chip($("ikchip"), "ARM STALLED · resync", "bad");
    else if (m.held) chip($("ikchip"), "IK HOLD", "bad");
    else if (!m.ik_ok) chip($("ikchip"), "OUT OF REACH", "warn");
    else if (m.rate_limited) chip($("ikchip"), "speed-limited", "warn");
    else chip($("ikchip"), "IK ok", "ok");
  }
  if (m.link_ms !== undefined) {
    // what the robot actually sees, not what this button thinks
    const b = $("clutchBtn");
    const dead = m.link_ms === null || m.link_ms > 250;
    const lost = !dead && m.tracked === false;
    b.classList.toggle("nosig", clutch && (dead || lost));
    b.textContent = !clutch ? "CLUTCH OFF"
      : dead ? "NO SIGNAL" : lost ? "TRACKING LOST" : "CLUTCH ON";
  }
  if (m.n_success !== undefined) $("counts").textContent = `✓${m.n_success} ✗${m.n_fail}`;
}

// ---- cameras: show a feed when the server says it is live, else the placeholder ----
function attachCam(name) {
  const box = $("cam-" + name), img = box.querySelector("img");
  setInterval(async () => {
    try {
      const live = (await (await fetch("/cams", { cache: "no-store" })).json())[name];
      if (live && !box.classList.contains("live")) { img.src = `/cam/${name}.mjpg?t=${Date.now()}`; box.classList.add("live"); }
      if (!live && box.classList.contains("live")) { img.removeAttribute("src"); box.classList.remove("live"); }
    } catch (e) {}
  }, 1500);
}

// ---- controls ----
function setClutch(on) {
  clutch = on;
  const b = $("clutchBtn");
  b.textContent = on ? "CLUTCH ON" : "CLUTCH OFF";
  b.classList.toggle("on", on); b.classList.toggle("off", !on);
}
function setOrient(lock) {
  orientLock = lock;
  const b = $("orientBtn");
  b.innerHTML = lock ? "ORIENT<br>LOCK" : "ORIENT<br>FREE";
  b.classList.toggle("free", !lock);
}
function setGripper(v) {
  gripper = Math.max(0, Math.min(100, Math.round(v)));
  $("gripslider").value = gripper;
  $("gripval").textContent = gripper + "%";
  document.querySelectorAll(".preset").forEach(
    (p) => p.classList.toggle("sel", Number(p.dataset.v) === gripper));
}
const tap = (id, fn) => $(id).addEventListener("click", (e) => { e.stopPropagation(); fn(e); });

function wireControls() {
  tap("clutchBtn", () => setClutch(!clutch));
  tap("orientBtn", () => setOrient(!orientLock));
  tap("homeBtn", () => { setClutch(false); evt("home"); });
  tap("fwdBtn", () => evt("setforward"));
  tap("startEp", () => { setClutch(false); evt("start", { instruction: $("instr").value.trim(), target: $("tgt").value.trim(), destination: $("dst").value.trim() }); });
  tap("nextBtn", () => evt("ready"));
  tap("succBtn", () => { setClutch(false); evt("success"); });
  tap("failBtn", () => { setClutch(false); evt("fail"); });
  tap("discardBtn", () => { setClutch(false); evt("discard"); });
  $("gripslider").addEventListener("input", (e) => setGripper(Number(e.target.value)));
  document.querySelectorAll(".preset").forEach(
    (p) => p.addEventListener("click", (e) => { e.stopPropagation(); setGripper(Number(p.dataset.v)); }));
  // stop slider / text taps from leaking into the XR "select" gesture
  ["gripslider", "instr", "tgt", "dst"].forEach((id) =>
    ["pointerdown", "touchstart"].forEach((t) => $(id).addEventListener(t, (e) => e.stopPropagation())));
  setGripper(gripper); setOrient(true);
  $("succBtn").disabled = $("failBtn").disabled = $("discardBtn").disabled = true;
}

async function startAR() {
  xrSession = await navigator.xr.requestSession("immersive-ar", {
    optionalFeatures: ["dom-overlay"], domOverlay: { root: $("overlay") },
  });
  $("overlay").style.display = "flex";
  attachCam("wrist");

  const glCanvas = document.createElement("canvas");
  const gl = glCanvas.getContext("webgl", { xrCompatible: true });
  await xrSession.updateRenderState({ baseLayer: new XRWebGLLayer(xrSession, gl) });
  const refSpace = await xrSession.requestReferenceSpace("local");  // y-up, metres = frame {ar}

  xrSession.addEventListener("end", () => {
    xrSession = null; setClutch(false);
    $("overlay").style.display = "none";
    $("support").textContent = "AR session ended.";
  });

  const onFrame = (t, frame) => {
    if (!xrSession) return;
    xrSession.requestAnimationFrame(onFrame);
    const pose = frame.getViewerPose(refSpace);
    if (!pose) { $("xrStats").textContent = "no pose (tracking lost)"; return; }
    if (!ws || ws.readyState !== 1) return;
    const p = pose.transform.position, q = pose.transform.orientation;
    ws.send(JSON.stringify({
      type: "xr", t, p: [p.x, p.y, p.z], q: [q.x, q.y, q.z, q.w],
      clutch, gripper, orient_lock: orientLock, tracked: !pose.emulatedPosition,
    }));
    if (++sentCount % 15 === 0)
      $("xrStats").textContent =
        `${p.x.toFixed(2)} ${p.y.toFixed(2)} ${p.z.toFixed(2)} m · ` +
        `${pose.emulatedPosition ? "3-DoF (no position!)" : "6-DoF"}`;
  };
  xrSession.requestAnimationFrame(onFrame);
}

(async function init() {
  connect(); wireControls();
  if (!navigator.xr) { $("support").textContent = "WebXR not available in this browser."; return; }
  const ok = await navigator.xr.isSessionSupported("immersive-ar").catch(() => false);
  if (!ok) { $("support").textContent = "WebXR present but immersive-ar isn't supported/enabled."; return; }
  $("support").textContent = "immersive-ar supported — ready.";
  const btn = $("startBtn"); btn.disabled = false;
  btn.addEventListener("click", () =>
    startAR().catch((e) => { $("support").textContent = `failed to start AR: ${e.message}`; }));
})();
