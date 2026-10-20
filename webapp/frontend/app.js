// Glaucus I operator app. Vanilla JS, no build step: the topside serves these
// files as-is, and anyone on the team can edit them without a toolchain.
"use strict";

const $ = (id) => document.getElementById(id);
const canvas = $("hull");
const ctx = canvas.getContext("2d");

let hull = null;      // from /api/hull
let status = null;    // latest status push
let coverage = null;  // latest coverage push
let ws = null;
let lastMsg = 0;

// ---------- plain-language state ----------
function sentenceFor(s) {
  if (!s || s.state === "IDLE") return "Robot is idle. Start a sweep when it's attached to the hull at the stern.";
  if (s.mission === "E-STOP") return "Emergency stop. Wheels and brush are off; the robot is holding on to the hull.";
  if (s.mission === "STOPPED") return "Sweep stopped. The robot is holding on to the hull.";
  if (s.paused) return "Paused. The robot is holding its position.";
  const lane = (s.lane_index ?? 0) + 1;
  const of = s.planned_lanes ? ` of ${s.planned_lanes}` : "";
  const toward = s.direction > 0 ? "toward the bow" : "toward the stern";
  switch (s.state) {
    case "LANE": return `Cleaning lane ${lane}${of}, ${toward}.`;
    case "VERIFY": return `Something looks like the hull edge early. Creeping forward to check.`;
    case "TURN": return "Turning in place.";
    case "STEP_DOWN": return s.lane_index < 0 ? "Moving down to the first lane." : `Moving down to lane ${lane + 1}.`;
    case "SHIFT": return "The hull curves away below. Shifting along to try going deeper again.";
    case "DONE": return "Sweep complete. Bring the robot up when you're ready.";
    default: return s.state;
  }
}

const EVENT_TEXT = {
  mission_started: () => "Sweep started.",
  paused: () => "Paused.",
  resumed: () => "Resumed.",
  mission_stopped: () => "Sweep stopped from the app.",
  estop: () => "Emergency stop pressed.",
  end_of_lane: (e) => `Reached the ${e.direction > 0 ? "bow" : "stern"} end of lane ${e.lane + 1}` +
    (e.verified ? " (confirmed after a second look)." : "."),
  end_rejected: (e) => `False alarm on lane ${e.lane + 1}: no edge after creeping forward. Kept going.`,
  step_down_blocked: (e) => `Couldn't go deeper here. Shifting over (attempt ${e.retry ?? e.value}).`,
  sweep_complete: (e) => e.reason === "cannot_descend"
    ? "Stopped: the hull curves away and the robot can't go deeper."
    : `Sweep complete: ${e.lanes ?? ""} lanes cleaned.`,
};

// ---------- drawing ----------
function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr)) { canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  if (!hull) return;

  const padL = 46, padR = 14, padT = 30, padB = 30;
  const sx = (W - padL - padR) / hull.length;
  const sz = (H - padT - padB) / (hull.draft + 0.05);
  const X = (x) => padL + x * sx;
  const Z = (z) => padT + z * sz;
  const ink = css("--ink"), muted = css("--muted"), rule = css("--rule");

  // water
  ctx.fillStyle = css("--water");
  ctx.fillRect(0, Z(0), W, H - Z(0));

  // hull side (clip everything else to it)
  const path = new Path2D();
  hull.outline.forEach(([x, z], i) => (i ? path.lineTo(X(x), Z(z)) : path.moveTo(X(x), Z(z))));
  path.closePath();
  ctx.save();
  ctx.clip(path);
  ctx.fillStyle = css("--fouled");
  ctx.fillRect(0, 0, W, H);
  if (coverage && coverage.rows && coverage.rows.length) {
    const r = coverage.res;
    ctx.fillStyle = css("--clean");
    coverage.rows.forEach((row, i) => {
      for (let j = 0; j < row.length; j++) {
        if (row[j] === "#") ctx.fillRect(X(j * r), Z(i * r), r * sx + 0.6, r * sz + 0.6);
      }
    });
  }
  // planned lanes
  ctx.setLineDash([3, 5]);
  ctx.lineWidth = 1;
  for (let k = 0, z = hull.first_lane_depth; z <= hull.max_depth + 1e-6; k++, z += hull.lane_spacing) {
    const current = status && status.lane_index === k && ["LANE", "VERIFY"].includes(status.state);
    ctx.strokeStyle = current ? "rgba(255,255,255,0.95)" : "rgba(255,255,255,0.35)";
    ctx.beginPath(); ctx.moveTo(0, Z(z)); ctx.lineTo(W, Z(z)); ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.restore();

  ctx.strokeStyle = ink; ctx.lineWidth = 1.5; ctx.stroke(path);

  // waterline + above-water hull band
  ctx.fillStyle = css("--surface");
  ctx.fillRect(X(hull.outline[0][0]), Z(0) - 14, X(hull.length) - X(hull.outline[0][0]), 14);
  ctx.strokeStyle = ink; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(0, Z(0)); ctx.lineTo(W, Z(0)); ctx.stroke();
  ctx.fillStyle = muted; ctx.font = `500 13px ${css("--f-ui")}`;
  ctx.textAlign = "right"; ctx.fillText("Waterline", W - padR, Z(0) - 18);

  // draft marks on the stern, like the numbers painted on a real hull
  ctx.textAlign = "right"; ctx.textBaseline = "middle"; ctx.fillStyle = ink;
  ctx.font = `600 15px ${css("--f-num")}`;
  for (let z = 0.2; z <= hull.draft + 1e-6; z += 0.2) {
    ctx.fillText(z.toFixed(1), padL - 10, Z(z));
    ctx.fillRect(padL - 6, Z(z) - 0.75, 6, 1.5);
  }
  ctx.font = `500 13px ${css("--f-ui")}`; ctx.fillStyle = muted;
  ctx.fillText("m", padL - 10, Z(0) - 18);

  // stern / bow
  ctx.textBaseline = "alphabetic"; ctx.fillStyle = muted;
  ctx.textAlign = "left"; ctx.fillText("Stern", padL, H - 8);
  ctx.textAlign = "right"; ctx.fillText("Bow", W - padR, H - 8);
  ctx.textAlign = "center";
  for (let x = 2; x < hull.length; x += 2) {
    if (X(x) > padL + 60 && X(x) < W - padR - 60) ctx.fillText(`${x} m`, X(x), H - 8);  // don't collide with Stern/Bow
  }

  // robot
  if (status && status.x_est != null && status.state !== "IDLE") {
    const px = X(status.x_est), pz = Z(status.depth || 0);
    const th = (status.heading_deg || 0) * Math.PI / 180;
    ctx.strokeStyle = css("--bot"); ctx.fillStyle = css("--bot"); ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(px, pz); ctx.lineTo(px + 18 * Math.cos(th), pz + 18 * Math.sin(th)); ctx.stroke();
    ctx.beginPath(); ctx.arc(px, pz, 7, 0, 2 * Math.PI); ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = css("--paper"); ctx.stroke();
  }
}

// ---------- readouts, buttons, log ----------
// Heading in plain words: 0 = bow, 180 = stern, 90 = down, -90 = up.
function facing(deg) {
  const d = ((deg % 360) + 540) % 360 - 180;
  const name = Math.abs(d) <= 30 ? "Bow" : Math.abs(d) >= 150 ? "Stern" : d > 0 ? "Down" : "Up";
  return `${name}<small>${Math.round(d)}°</small>`;
}

function render() {
  const s = status;
  $("sentence").textContent = sentenceFor(s);
  const has = s && s.state !== "IDLE";
  $("r-lane").innerHTML = has && s.lane_index >= 0 ? `${s.lane_index + 1}<small>/${s.planned_lanes ?? "?"}</small>` : "–";
  $("r-depth").innerHTML = has ? `${(s.depth ?? 0).toFixed(2)}<small>m</small>` : "–";
  $("r-heading").innerHTML = has ? facing(s.heading_deg ?? 0) : "–";
  const cov = coverage && coverage.coverage != null ? coverage.coverage : s && s.coverage;
  $("r-coverage").innerHTML = cov != null && has ? `${Math.round(cov * 100)}<small>%</small>` : "–";
  $("r-score").value = has ? Math.min(1.2, s.end_score || 0) : 0;

  const running = s && ["RUNNING", "PAUSED"].includes(s.mission) && s.state !== "DONE";
  $("btn-start").disabled = running;
  $("btn-pause").disabled = !running;
  $("btn-stop").disabled = !running;
  $("btn-pause").textContent = s && s.paused ? "Resume" : "Pause";
  $("mock-note").hidden = !(s && s.mock);

  const list = $("events");
  const evs = (s && s.events) || [];
  if (!evs.length) { list.innerHTML = '<li class="empty">Nothing yet.</li>'; }
  else {
    list.innerHTML = evs.map((e) => {
      const text = (EVENT_TEXT[e.name] || (() => e.name))(e);
      const t = e.t != null ? `${Math.floor(e.t / 60)}:${String(Math.floor(e.t % 60)).padStart(2, "0")}` : "";
      return `<li><time>${t}</time><span>${text}</span></li>`;
    }).join("");
  }
  draw();
}

function setLink(state, text) { const el = $("link"); el.dataset.state = state; el.textContent = text; }

function connect() {
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (m) => {
    lastMsg = Date.now();
    const d = JSON.parse(m.data);
    if (d.status) status = d.status;
    if (d.coverage) coverage = d.coverage;
    setLink(status && status.connected === false ? "lost" : "ok",
            status && status.connected === false ? "Topside up, robot not responding" : "Connected");
    render();
  };
  ws.onclose = () => { setLink("lost", "Connection lost. Retrying"); setTimeout(connect, 1500); };
}
setInterval(() => { if (lastMsg && Date.now() - lastMsg > 3000) setLink("lost", "No data for 3 s"); }, 1000);

let toastTimer;
function toast(msg, kind) {
  const el = $("toast"); el.textContent = msg; el.dataset.kind = kind || ""; el.hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => (el.hidden = true), 3500);
}

async function post(url, body) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" },
                                 body: body ? JSON.stringify(body) : undefined });
    const d = await r.json();
    toast(d.message, d.ok ? "" : "error");
  } catch (e) {
    toast("Couldn't reach the topside. Check you're on the robot's Wi-Fi.", "error");
  }
}

$("btn-start").onclick = () => post("/api/mission/start");
$("btn-pause").onclick = () => post("/api/mission/pause", { paused: !(status && status.paused) });
$("btn-stop").onclick = () => { if (confirm("Stop the sweep? The robot will hold its position on the hull.")) post("/api/mission/stop"); };
$("btn-estop").onclick = () => post("/api/estop");   // never ask twice for an e-stop

window.addEventListener("resize", draw);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);

fetch("/api/hull").then((r) => r.json()).then((h) => { hull = h; render(); });
connect();
