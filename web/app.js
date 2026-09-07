"use strict";

// ---------------------------------------------------------------- state

const WINDOW_S = 15;          // rolling chart window
const MAX_POINTS = 50 * WINDOW_S;
const MAX_EVENTS = 200;
const MAX_LOG_LINES = 300;

// per-channel ring buffers: t (shared per node), value
const buf = {
  L: { t: [], hg: [], ax: [], gx: [], f0: [], f1: [] },
  R: { t: [], hg: [], ax: [], gx: [], f0: [], f1: [] },
};
let events = [];
let ws = null;
let paused = false;
let lastSeen = { L: 0, R: 0 };

// ---------------------------------------------------------------- charts

const CSS = getComputedStyle(document.documentElement);
const COL_L = CSS.getPropertyValue("--left").trim();
const COL_R = CSS.getPropertyValue("--right").trim();

function mkChart(elId, unit) {
  const el = document.getElementById(elId);
  const opts = {
    width: el.clientWidth || 600,
    height: el.clientHeight || 160,
    ms: false,
    series: [
      {},
      { label: "L", stroke: COL_L, width: 1, points: { show: false } },
      { label: "R", stroke: COL_R, width: 1, points: { show: false } },
    ],
    axes: [
      { stroke: "#7a8090", grid: { stroke: "#2a2e38" }, ticks: { stroke: "#2a2e38" } },
      { stroke: "#7a8090", grid: { stroke: "#2a2e38" }, ticks: { stroke: "#2a2e38" },
        label: unit, labelSize: 12 },
    ],
    scales: { x: { time: false } },
    legend: { show: true },
    // drag is our own pan handler; sync ties the cursor across all charts so
    // the legend of every chart reads the same instant
    cursor: { drag: { setScale: false }, sync: { key: "boxe" } },
  };
  const u = new uPlot(opts, [[], [], []], el);
  new ResizeObserver(() => u.setSize({ width: el.clientWidth, height: el.clientHeight }))
    .observe(el);
  setupInteractions(u);
  return u;
}

// ------------------------------------------------------- zoom / pan
//
// The charts normally follow the live tail: setData() re-ranges x every frame.
// The first wheel or drag drops out of follow mode and freezes the window so a
// waveform can be inspected while data keeps arriving; double-click (or the
// Live button) resumes. All charts share one x window.

let follow = true;
let syncing = false;

function setFollow(on) {
  follow = on;
  document.getElementById("btn-live").classList.toggle("armed", on);
}

function propagateX(src) {
  if (syncing) return;
  syncing = true;
  const { min, max } = src.scales.x;
  for (const p of Object.values(charts)) {
    if (p !== src) p.setScale("x", { min, max });
  }
  syncing = false;
}

function setupInteractions(u) {
  const over = u.over;
  over.style.cursor = "grab";

  over.addEventListener("dblclick", (e) => { e.preventDefault(); setFollow(true); });

  over.addEventListener("wheel", (e) => {
    e.preventDefault();
    const { min, max } = u.scales.x;
    if (min == null) return;
    setFollow(false);
    const rect = over.getBoundingClientRect();
    const at = min + ((e.clientX - rect.left) / rect.width) * (max - min);
    const f = e.deltaY < 0 ? 0.8 : 1.25;
    u.setScale("x", { min: at - (at - min) * f, max: at + (max - at) * f });
    propagateX(u);
  }, { passive: false });

  over.addEventListener("mousedown", (e) => {
    if (e.button !== 0) return;
    const { min, max } = u.scales.x;
    if (min == null) return;
    setFollow(false);
    const startX = e.clientX, x0 = min, x1 = max;
    const unitPerPx = (max - min) / over.clientWidth;
    over.style.cursor = "grabbing";
    e.preventDefault();
    const move = (ev) => {
      const shift = -(ev.clientX - startX) * unitPerPx;
      u.setScale("x", { min: x0 + shift, max: x1 + shift });
      propagateX(u);
    };
    const up = () => {
      window.removeEventListener("mousemove", move);
      window.removeEventListener("mouseup", up);
      over.style.cursor = "grab";
    };
    window.addEventListener("mousemove", move);
    window.addEventListener("mouseup", up);
  });
}

const charts = {
  hg: mkChart("chart-hg", "g"),
  f0: mkChart("chart-f0", "counts"),
  f1: mkChart("chart-f1", "counts"),
  ax: mkChart("chart-ax", "mg"),
  gx: mkChart("chart-gx", "dps"),
};

// merge L and R (different timestamps) onto one x axis by resampling:
// uPlot wants one x array, so we plot against host-relative seconds and
// align by nearest sample. Simpler: use the union of timestamps with nulls.
function chartData(field) {
  const tL = buf.L.t, tR = buf.R.t;
  const xs = [];
  const l = [];
  const r = [];
  let i = 0, j = 0;
  while (i < tL.length || j < tR.length) {
    const a = i < tL.length ? tL[i] : Infinity;
    const b = j < tR.length ? tR[j] : Infinity;
    if (a <= b) {
      xs.push(a); l.push(buf.L[field][i]); r.push(null); i++;
      if (b === a) { r[r.length - 1] = buf.R[field][j]; j++; }
    } else {
      xs.push(b); l.push(null); r.push(buf.R[field][j]); j++;
    }
  }
  return [xs, l, r];
}

function redraw() {
  // second arg is uPlot's resetScales: re-range while following, hold the
  // frozen window otherwise
  for (const [field, u] of Object.entries(charts)) {
    u.setData(chartData(field), follow);
  }
}
setInterval(() => { if (!paused) redraw(); }, 120);

// ---------------------------------------------------------------- pose
//
// What a wrist IMU can and cannot tell you, and what this view does about it:
//
//   * Absolute position: not recoverable. Position is acceleration integrated
//     twice, so any bias grows as t^2 and the answer is nonsense within about
//     a second. There is no external reference to pull it back.
//   * Orientation: recoverable. Accel gives pitch and roll from gravity, the
//     gyro fills in the fast motion between. Yaw has nothing to anchor it
//     (no magnetometer in the stream) and drifts slowly; "Zero" re-anchors it.
//   * Per-punch path: recoverable with a trick. A punch is 200-300 ms between
//     two moments when the hand is still. Reset velocity to zero at every
//     still moment (ZUPT) and integrate only during the swing, and drift has
//     no time to accumulate. The result is the path and reach of one punch
//     relative to where it started - not where the fist is in the room.
//
// Orientation is a Mahony-style filter: gyro integration corrected toward the
// measured gravity direction, with the correction gain dropping to zero when
// |a| is far from 1 g (mid-punch the accelerometer is not measuring gravity).

const POSE = {
  KP: 2.0,               // tilt correction gain (1/s) when |a| ~ 1 g
  STILL_A: 0.12,         // |a| within this of 1 g ...
  STILL_W: 30,           // ... and |w| below this (dps) counts as still
  STILL_N: 6,            // for this many consecutive samples -> ZUPT
  SWING_MAX_S: 0.8,      // integration budget per swing before drift wins
  TRAIL_S: 0.6,          // how much path to draw
};

const qMul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
const qConj = (q) => [q[0], -q[1], -q[2], -q[3]];
function qNorm(q) {
  const n = Math.hypot(...q) || 1;
  return q.map((x) => x / n);
}
// rotate body-frame vector v into the world frame by q (body -> world)
function qRot(q, v) {
  const r = qMul(qMul(q, [0, v[0], v[1], v[2]]), qConj(q));
  return [r[1], r[2], r[3]];
}

class Pose {
  constructor() {
    this.q = [1, 0, 0, 0];   // body -> world
    this.qRef = null;        // guard reference, set by Zero or first stillness
    this.v = [0, 0, 0];
    this.p = [0, 0, 0];
    this.trail = [];         // [{t, p}]
    this.reach = 0;          // peak |p| of the current / last swing (m)
    this.stillN = 0;
    this.still = true;
    this.swingT = 0;
    this.lastTus = null;
    this.lastT = 0;
  }

  update(rec) {
    // dt from the node's own clock: host arrival time jitters by tens of ms
    // and packets carry five samples with one timestamp
    let dt = 0.01;
    if (this.lastTus != null) {
      dt = ((rec.t_us - this.lastTus) >>> 0) / 1e6;
      if (dt <= 0 || dt > 0.05) dt = 0.01;
    }
    this.lastTus = rec.t_us;
    this.lastT = rec.t;

    const a = [rec.ax / 1000, rec.ay / 1000, rec.az / 1000];   // g
    const an = Math.hypot(...a);
    const wdps = [rec.gx, rec.gy, rec.gz];
    const wn = Math.hypot(...wdps);
    const w = wdps.map((x) => (x * Math.PI) / 180);             // rad/s

    // --- orientation: gyro, corrected toward gravity when it is trustworthy
    if (an > 0.5) {
      const gb = qRot(qConj(this.q), [0, 0, 1]);   // predicted gravity, body
      const ab = a.map((x) => x / an);               // measured gravity, body
      // error = measured x predicted; zero when they agree
      const e = [
        ab[1] * gb[2] - ab[2] * gb[1],
        ab[2] * gb[0] - ab[0] * gb[2],
        ab[0] * gb[1] - ab[1] * gb[0],
      ];
      // trust falls to zero 0.25 g away from 1 g: a jab is a 1-3 g push,
      // and correcting toward that would bend "gravity" into the punch and
      // eat the very acceleration the path integrates
      const trust = Math.max(0, 1 - Math.abs(an - 1) / 0.25);
      for (let i = 0; i < 3; i++) w[i] += POSE.KP * trust * e[i];
    }
    const dq = qMul(this.q, [0, w[0], w[1], w[2]]).map((x) => 0.5 * x * dt);
    this.q = qNorm(this.q.map((x, i) => x + dq[i]));

    // --- stillness -> zero-velocity reset
    const stillNow = Math.abs(an - 1) < POSE.STILL_A && wn < POSE.STILL_W;
    this.stillN = stillNow ? this.stillN + 1 : 0;
    if (this.stillN >= POSE.STILL_N) {
      if (!this.still) this.trail = [];            // new swing starts clean
      this.still = true;
      this.v = [0, 0, 0];
      this.p = [0, 0, 0];
      this.swingT = 0;
      if (!this.qRef) this.qRef = this.q.slice();  // first stillness = guard
      return;
    }

    // --- swing: integrate gravity-free world acceleration, drift-budgeted
    if (this.still) { this.still = false; this.reach = 0; this.trail = []; }
    this.swingT += dt;
    if (this.swingT > POSE.SWING_MAX_S) return;    // drift wins from here
    const aw = qRot(this.q, a);
    aw[2] -= 1;                                     // remove gravity
    for (let i = 0; i < 3; i++) {
      this.v[i] += aw[i] * 9.81 * dt;
      this.p[i] += this.v[i] * dt;
    }
    this.reach = Math.max(this.reach, Math.hypot(...this.p));
    this.trail.push({ t: rec.t, p: this.p.slice() });
    while (this.trail.length && rec.t - this.trail[0].t > POSE.TRAIL_S) {
      this.trail.shift();
    }
  }

  // orientation relative to the guard reference, for display
  qDisplay() {
    return this.qRef ? qMul(qConj(this.qRef), this.q) : this.q;
  }
}

const pose = { L: new Pose(), R: new Pose() };
const flash = { L: 0, R: 0 };   // last punch event time, for the glow

document.getElementById("btn-zero").onclick = () => {
  for (const n of ["L", "R"]) pose[n].qRef = pose[n].q.slice();
};

// --- rendering: a small fixed camera and orthographic projection, enough to
// read a box's attitude. Camera looks from front-left, slightly above.
const CAM = (() => {
  const yaw = (-35 * Math.PI) / 180, pitch = (22 * Math.PI) / 180;
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  // world (x fwd, y left, z up) -> screen (right, up)
  return (v) => {
    const x1 = v[0] * cy - v[1] * sy;
    const y1 = v[0] * sy + v[1] * cy;
    const z1 = v[2];
    return [y1, z1 * cp - x1 * sp];
  };
})();

// glove as a box: forearm axis along body +X, knuckles at +X end. The true
// mounting axis is whatever the strap gives; Zero makes the guard pose the
// reference, so the box only ever shows change from guard.
const BOX = [[-0.55, -0.3, -0.22], [0.55, -0.3, -0.22], [0.55, 0.3, -0.22], [-0.55, 0.3, -0.22],
             [-0.55, -0.3, 0.22], [0.55, -0.3, 0.22], [0.55, 0.3, 0.22], [-0.55, 0.3, 0.22]];
const BOX_EDGES = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4],
                   [0, 4], [1, 5], [2, 6], [3, 7]];
const KNUCKLE_FACE = [1, 2, 6, 5];

function drawPose() {
  const cv = document.getElementById("pose-canvas");
  const W = cv.clientWidth, H = cv.clientHeight;
  if (!W || !H) return;
  if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, W, H);

  const now = Date.now();
  ["L", "R"].forEach((n, k) => {
    const P = pose[n];
    const col = n === "L" ? COL_L : COL_R;
    const cx = W * (0.25 + 0.5 * k), cy = H * 0.5;
    const S = Math.min(W / 4, H) * 0.42;           // px per box unit
    const q = P.qDisplay();
    const glow = now - flash[n] < 250;

    // punch path, world frame, 1 m = 2.2 box units so a jab fits the tile
    if (P.trail.length > 1) {
      ctx.beginPath();
      P.trail.forEach((pt, i) => {
        const s = CAM(pt.p.map((x) => x * 2.2));
        const sx = cx + s[0] * S, sy = cy - s[1] * S;
        i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
      });
      ctx.strokeStyle = col; ctx.globalAlpha = 0.45; ctx.lineWidth = 2;
      ctx.stroke(); ctx.globalAlpha = 1;
    }

    // box at the current fist position, oriented by q
    const origin = P.trail.length ? P.trail[P.trail.length - 1].p.map((x) => x * 2.2) : [0, 0, 0];
    const pts = BOX.map((v) => {
      const r = qRot(q, v);
      const s = CAM([r[0] + origin[0], r[1] + origin[1], r[2] + origin[2]]);
      return [cx + s[0] * S, cy - s[1] * S];
    });
    ctx.beginPath();
    KNUCKLE_FACE.forEach((i, j) => (j ? ctx.lineTo(...pts[i]) : ctx.moveTo(...pts[i])));
    ctx.closePath();
    ctx.fillStyle = col; ctx.globalAlpha = glow ? 0.9 : 0.35; ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath();
    for (const [a, b] of BOX_EDGES) { ctx.moveTo(...pts[a]); ctx.lineTo(...pts[b]); }
    ctx.strokeStyle = glow ? "#fff" : col; ctx.lineWidth = glow ? 2 : 1.2; ctx.stroke();

    // readouts
    ctx.fillStyle = col; ctx.font = "bold 12px system-ui"; ctx.textAlign = "left";
    ctx.fillText(n, cx - W / 4 + 8, 16);
    ctx.fillStyle = "#7a8090"; ctx.font = "11px system-ui";
    const state = P.lastTus == null ? "no data" : P.still ? "still" : "swing";
    ctx.fillText(state, cx - W / 4 + 8, 30);
    ctx.textAlign = "right";
    ctx.fillText(`reach ${P.reach.toFixed(2)} m`, cx + W / 4 - 8, 16);
    if (!P.qRef) ctx.fillText("waiting for stillness to zero", cx + W / 4 - 8, 30);
  });

  // divider
  ctx.strokeStyle = "#2a2e38"; ctx.beginPath();
  ctx.moveTo(W / 2, 6); ctx.lineTo(W / 2, H - 6); ctx.stroke();
}
(function poseLoop() { drawPose(); requestAnimationFrame(poseLoop); })();

// ---------------------------------------------------------------- records

const t0 = Date.now() / 1000;

function pushImu(rec) {
  const b = buf[rec.node];
  if (!b) return;
  pose[rec.node].update(rec);
  const t = rec.t - t0;
  b.t.push(t); b.hg.push(rec.hg); b.ax.push(rec.ax); b.gx.push(rec.gx);
  b.f0.push(rec.f0); b.f1.push(rec.f1);
  if (b.t.length > MAX_POINTS) {
    b.t.shift(); b.hg.shift(); b.ax.shift(); b.gx.shift();
    b.f0.shift(); b.f1.shift();
  }
}

function pushEvent(rec) {
  if (flash[rec.node] !== undefined) flash[rec.node] = Date.now();
  events.unshift(rec);
  if (events.length > MAX_EVENTS) events.pop();
  const tbody = document.querySelector("#events-table tbody");
  const tr = document.createElement("tr");
  const t = new Date(rec.t * 1000).toLocaleTimeString();
  const hit = rec.contact ? "✔" : "miss";
  tr.className = rec.contact ? "" : "miss";
  tr.innerHTML =
    `<td>${t}</td><td class="hand-${rec.node}">${rec.node}</td>` +
    `<td class="${rec.sat ? "sat" : ""}">${rec.peak_g}${rec.sat ? "⚠" : ""}</td>` +
    `<td>${rec.f0}</td><td>${rec.width_ms || ""}</td>` +
    `<td>${rec.exec_ms ?? ""}</td><td>${hit}</td>`;
  tbody.prepend(tr);
  while (tbody.children.length > MAX_EVENTS) tbody.lastChild.remove();
  document.getElementById("ev-count").textContent = `(${events.length})`;
}

function pushStatus(rec) {
  const n = rec.node;
  const set = (k, v) => {
    const el = document.getElementById(`st-${n}-${k}`);
    if (el) el.textContent = v ?? "—";
  };
  set("batt", rec.batt_mv);
  set("loop", rec.loop_hz);
  set("drop", rec.dropped);
  set("ev", rec.events);
  set("rssi", rec.rssi);
  const battEl = document.getElementById(`st-${n}-batt`);
  if (battEl && rec.batt_mv) {
    battEl.style.color = rec.batt_mv < 3400 ? "var(--warn)" : "";
  }
}

function pushLog(rec) {
  const con = document.getElementById("log-console");
  const div = document.createElement("div");
  div.className = rec.level || "info";
  div.textContent = `${new Date(rec.t * 1000).toLocaleTimeString()} ${rec.msg}`;
  con.appendChild(div);
  while (con.children.length > MAX_LOG_LINES) con.firstChild.remove();
  con.scrollTop = con.scrollHeight;
}

function handle(rec) {
  if (paused && rec.type === "imu") return;
  if (rec.node === "L" || rec.node === "R") lastSeen[rec.node] = Date.now();
  switch (rec.type) {
    case "imu": pushImu(rec); break;
    case "event": pushEvent(rec); break;
    case "status": pushStatus(rec); break;
    case "log": pushLog(rec); break;
  }
}

// node liveness badges
setInterval(() => {
  for (const n of ["L", "R"]) {
    const on = Date.now() - lastSeen[n] < 3000;
    document.getElementById(`badge-${n}`).classList.toggle("on", on);
  }
}, 500);

// ---------------------------------------------------------------- ws / replay

function connect() {
  const url = document.getElementById("ws-url").value;
  if (ws) { ws.close(); ws = null; }
  ws = new WebSocket(url);
  const badge = document.getElementById("badge-host");
  ws.onopen = () => { badge.classList.add("on"); pushLog({ t: Date.now() / 1000, level: "info", msg: `connected ${url}` }); };
  ws.onclose = () => { badge.classList.remove("on"); pushLog({ t: Date.now() / 1000, level: "warn", msg: "host disconnected" }); };
  ws.onerror = () => badge.classList.remove("on");
  ws.onmessage = (e) => handle(JSON.parse(e.data));
}

document.getElementById("btn-connect").onclick = connect;

document.getElementById("btn-live").onclick = () => setFollow(true);
setFollow(true);

document.getElementById("btn-pause").onclick = (e) => {
  paused = !paused;
  e.target.classList.toggle("active", paused);
  e.target.textContent = paused ? "Resume" : "Pause";
};

// JSONL replay: load a past session, replay at real speed factor 10x
document.getElementById("replay-file").onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const text = await file.text();
  const recs = text.split("\n").filter(Boolean).map((l) => JSON.parse(l));
  pushLog({ t: Date.now() / 1000, level: "info", msg: `replaying ${file.name} (${recs.length} records, 10x)` });
  if (ws) { ws.close(); ws = null; }
  const start = recs[0]?.t ?? 0;
  const wall0 = Date.now() / 1000;
  for (const rec of recs) {
    const due = wall0 + (rec.t - start) / 10;
    const wait = due - Date.now() / 1000;
    if (wait > 0) await new Promise((r) => setTimeout(r, wait * 1000));
    handle(rec);
  }
  pushLog({ t: Date.now() / 1000, level: "info", msg: "replay done" });
};

connect();
