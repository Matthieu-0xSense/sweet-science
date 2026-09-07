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

// ---------------------------------------------------------------- records

const t0 = Date.now() / 1000;

function pushImu(rec) {
  const b = buf[rec.node];
  if (!b) return;
  const t = rec.t - t0;
  b.t.push(t); b.hg.push(rec.hg); b.ax.push(rec.ax); b.gx.push(rec.gx);
  b.f0.push(rec.f0); b.f1.push(rec.f1);
  if (b.t.length > MAX_POINTS) {
    b.t.shift(); b.hg.shift(); b.ax.shift(); b.gx.shift();
    b.f0.shift(); b.f1.shift();
  }
}

function pushEvent(rec) {
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
