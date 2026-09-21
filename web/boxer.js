"use strict";

// Boxer view. Consumes the same WebSocket / JSONL records as the debug panel
// (protocol.md) and keeps only what changes what a boxer does next: output,
// power, balance, combinations, and how all of it holds up across rounds.
//
// Everything is computed from `event` records. Time is the record's own `t`
// (host unix seconds), never the arrival time, so a loaded session file gives
// the same numbers as the live one did.

const COMBO_GAP_S = 0.7;      // punches closer than this belong to one combo
const COMBO_SHOW_S = 1.5;     // chips stay up this long after the last punch
const FLASH_MS = 350;
const BEST_AFTER = 5;         // no "best" call-outs before this many punches

const CSS = getComputedStyle(document.documentElement);
const COL = { L: CSS.getPropertyValue("--left").trim(), R: CSS.getPropertyValue("--right").trim() };

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- state

let punches = [];             // {t, hand, g, landed, round}
let rounds = [];              // {n, label, start, end|null}  work periods only
let swap = false;
let live = true;              // false once a session file is loaded
let lastRecT = 0;
let lastSeen = { L: 0, R: 0 };
let ws = null;

const timer = { mode: "free", round: 0, endsAt: 0, cfg: null };

const clock = () => (live ? Date.now() / 1000 : lastRecT);

function resetSession() {
  $("last-hand").textContent = "–"; $("last-g").textContent = "–"; $("last-note").textContent = " ";
  punches = [];
  rounds = [];
  timer.mode = "free"; timer.round = 0;
  render();
}

// ---------------------------------------------------------------- rounds

function beep(freq, ms, n = 1) {
  try {
    const ac = beep.ac || (beep.ac = new AudioContext());
    for (let i = 0; i < n; i++) {
      const o = ac.createOscillator(), g = ac.createGain();
      o.frequency.value = freq; o.connect(g); g.connect(ac.destination);
      const t0 = ac.currentTime + i * (ms / 1000 + 0.12);
      g.gain.setValueAtTime(0.25, t0);
      g.gain.exponentialRampToValueAtTime(0.001, t0 + ms / 1000);
      o.start(t0); o.stop(t0 + ms / 1000);
    }
  } catch { /* no audio device: the clock still runs */ }
}

function startWork(n) {
  const now = clock();
  timer.mode = "work"; timer.round = n; timer.endsAt = now + timer.cfg.work;
  timer.warned = false;
  rounds.push({ n, label: `R${n}`, start: now, end: null });
  beep(880, 500);
}

function closeRound() {
  const r = rounds[rounds.length - 1];
  if (r && r.end == null) r.end = clock();
}

$("btn-start").onclick = () => {
  if (!live) return;
  timer.cfg = { rounds: +$("cfg-rounds").value, work: +$("cfg-work").value, rest: +$("cfg-rest").value };
  // punches thrown before the first bell were a warm-up, not round 1
  punches = []; rounds = [];
  startWork(1);
  render();
};

$("btn-stop").onclick = () => {
  if (timer.mode === "free") return;
  closeRound();
  timer.mode = "done";
  render();
};

function tickTimer() {
  if (timer.mode !== "work" && timer.mode !== "rest") return;
  const left = timer.endsAt - clock();
  if (timer.mode === "work" && !timer.warned && left <= 10) { timer.warned = true; beep(660, 120, 3); }
  if (left > 0) return;
  if (timer.mode === "work") {
    closeRound();
    beep(440, 900);
    if (timer.round >= timer.cfg.rounds) { timer.mode = "done"; }
    else if (timer.cfg.rest > 0) { timer.mode = "rest"; timer.endsAt = clock() + timer.cfg.rest; }
    else startWork(timer.round + 1);
  } else {
    startWork(timer.round + 1);
  }
}

// ---------------------------------------------------------------- records

function onEvent(rec) {
  // same classes as boxe_host.classify_event; only thrown punches count.
  // Logs from before `kind` existed are rebuilt the same way metrics.py does.
  const kind = rec.kind ??
    (rec.peak_g >= 5 ? (rec.contact ? "punch" : "miss") : (rec.contact ? "press" : "other"));
  if (kind !== "punch" && kind !== "miss") return;

  let round = 0;
  if (timer.mode === "work") round = timer.round;
  else if (timer.mode !== "free") return;      // resting or finished: not part of the score

  const hand = swap ? (rec.node === "L" ? "R" : "L") : rec.node;
  // Power is the harder of the two accelerometers: an unloaded punch can read
  // more on the LSM6 than on the ADXL375 (which starts resolving at ~5 g),
  // an impact the other way round.
  const g = Math.max(rec.peak_g ?? 0, rec.peak_lg_g ?? 0);
  const best = punches.length >= BEST_AFTER && g > Math.max(...punches.map((p) => p.g));
  punches.push({ t: rec.t, hand, g, landed: kind === "punch", round });

  flashLast(hand, g, best, kind === "punch");
  render();
}

function handle(rec) {
  if (rec.t) lastRecT = rec.t;
  if (rec.node === "L" || rec.node === "R") lastSeen[rec.node] = Date.now();
  if (rec.type === "event") onEvent(rec);
}

// ---------------------------------------------------------------- stats

const mean = (a) => (a.length ? a.reduce((s, x) => s + x, 0) / a.length : null);
const fmt = (x, d = 1) => (x == null || !isFinite(x) ? "–" : x.toFixed(d));
const pct = (a, b) => (b ? `${Math.round((100 * a) / b)}%` : "–");

function combosOf(ps) {
  const out = [];
  let cur = [];
  for (const p of ps) {
    if (cur.length && p.t - cur[cur.length - 1].t > COMBO_GAP_S) {
      if (cur.length > 1) out.push(cur);
      cur = [];
    }
    cur.push(p);
  }
  if (cur.length > 1) out.push(cur);
  return out;
}

function statsOf(ps, span) {
  const gs = ps.map((p) => p.g);
  const combos = combosOf(ps);
  let pause = 0;
  for (let i = 1; i < ps.length; i++) pause = Math.max(pause, ps[i].t - ps[i - 1].t);
  return {
    n: ps.length,
    L: ps.filter((p) => p.hand === "L").length,
    R: ps.filter((p) => p.hand === "R").length,
    pace: span > 0 ? (ps.length / span) * 60 : null,
    avg: mean(gs), max: gs.length ? Math.max(...gs) : null,
    landed: ps.filter((p) => p.landed).length,
    combos, comboMax: combos.reduce((m, c) => Math.max(m, c.length), 0),
    pause,
  };
}

// span a set of punches is judged over: the round's own clock when there is
// one, first-to-last punch (at least 10 s) in a free session
function roundSpan(r) {
  return (r.end ?? clock()) - r.start;
}
function freeSpan(ps) {
  return ps.length ? Math.max(10, clock() - ps[0].t) : 0;
}

// ---------------------------------------------------------------- render

let flashTimer = null;
function flashLast(hand, g, best, landed) {
  const tile = $("last-tile");
  tile.classList.remove("flash-L", "flash-R");
  if (live) {                  // a loaded file scores hundreds of punches in one pass
    void tile.offsetWidth;
    tile.classList.add(`flash-${hand}`);
    clearTimeout(flashTimer);
    flashTimer = setTimeout(() => tile.classList.remove("flash-L", "flash-R"), FLASH_MS);
  }
  $("last-hand").textContent = hand; $("last-hand").className = hand;
  $("last-g").textContent = fmt(g);
  $("last-note").textContent = best ? "★ best of the session" : landed ? "landed" : "no contact";
  $("last-note").className = best ? "sub best" : "sub";
}

function renderHand(h, ps) {
  const mine = ps.filter((p) => p.hand === h);
  const s = statsOf(mine, 0);
  $(`${h}-n`).textContent = s.n;
  $(`${h}-avg`).textContent = fmt(s.avg);
  $(`${h}-max`).textContent = fmt(s.max);
  $(`${h}-land`).textContent = pct(s.landed, s.n);
}

function renderRounds(all) {
  const tb = document.querySelector("#rounds-table tbody");
  const rows = rounds.length
    ? rounds.map((r) => ({ label: r.label, s: statsOf(punches.filter((p) => p.round === r.n), roundSpan(r)) }))
    : [];
  const first = rows[0]?.s;
  const cell = (s, base) => {
    const down = (k) => (base && base !== s && s[k] != null && base[k] && s[k] < 0.85 * base[k] ? "down" : "");
    return `<td>${s.n}</td><td><span class="l">${s.L}</span> / <span class="r">${s.R}</span></td>` +
      `<td class="${down("pace")}">${fmt(s.pace, 0)}</td><td class="${down("avg")}">${fmt(s.avg)}</td>` +
      `<td>${fmt(s.max)}</td><td>${pct(s.landed, s.n)}</td><td>${s.comboMax || "–"}</td>`;
  };
  tb.innerHTML = rows.map((r) => `<tr><td>${r.label}</td>${cell(r.s, first)}</tr>`).join("") +
    `<tr class="total"><td>${rounds.length ? "all" : "free"}</td>${cell(all)}</tr>`;

  // fatigue: last finished round against the first
  const done = rows.filter((_, i) => rounds[i].end != null);
  let txt = "";
  if (done.length >= 2) {
    const a = done[0].s, b = done[done.length - 1].s;
    const d = (x, y) => (x ? Math.round((100 * (y - x)) / x) : 0);
    txt = `${done[done.length - 1].label} vs R1: power ${d(a.avg, b.avg) >= 0 ? "+" : ""}${d(a.avg, b.avg)}% · ` +
          `pace ${d(a.pace, b.pace) >= 0 ? "+" : ""}${d(a.pace, b.pace)}%`;
  }
  $("fatigue-txt").textContent = txt;
}

function renderTimeline() {
  const cv = $("timeline");
  const W = cv.clientWidth, H = cv.clientHeight;
  if (!W || !H) return;
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== W * dpr || cv.height !== H * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  if (!punches.length) {
    ctx.fillStyle = "#7a8090"; ctx.font = "13px system-ui"; ctx.textAlign = "center";
    ctx.fillText("waiting for the first punch", W / 2, H / 2);
    return;
  }
  const t0 = Math.min(punches[0].t, rounds[0]?.start ?? Infinity);
  const tEnd = timer.mode === "done" ? rounds[rounds.length - 1].end : clock();
  const t1 = Math.max(tEnd, punches[punches.length - 1].t, t0 + 30);
  const gMax = Math.max(10, ...punches.map((p) => p.g)) * 1.1;
  const padL = 30, padB = 16, padT = 6;
  const X = (t) => padL + ((t - t0) / (t1 - t0)) * (W - padL - 6);
  const Y = (g) => H - padB - (g / gMax) * (H - padB - padT);

  // rounds as bands, so rest reads as rest
  ctx.font = "11px system-ui"; ctx.textAlign = "left";
  for (const r of rounds) {
    const x0 = X(r.start), x1 = X(r.end ?? clock());
    ctx.fillStyle = "rgba(63,191,111,0.07)"; ctx.fillRect(x0, padT, x1 - x0, H - padB - padT);
    ctx.fillStyle = "#7a8090"; ctx.fillText(r.label, x0 + 3, padT + 11);
  }
  // g grid
  ctx.strokeStyle = "#2a2e38"; ctx.fillStyle = "#7a8090"; ctx.textAlign = "right";
  const step = gMax > 60 ? 20 : gMax > 25 ? 10 : 5;
  for (let g = 0; g < gMax; g += step) {
    ctx.beginPath(); ctx.moveTo(padL, Y(g)); ctx.lineTo(W - 6, Y(g)); ctx.stroke();
    ctx.fillText(g, padL - 4, Y(g) + 4);
  }
  ctx.textAlign = "center";
  const tStep = t1 - t0 > 600 ? 120 : t1 - t0 > 180 ? 60 : 15;
  for (let t = 0; t < t1 - t0; t += tStep) {
    ctx.fillText(`${Math.floor(t / 60)}:${String(Math.floor(t % 60)).padStart(2, "0")}`, X(t0 + t), H - 3);
  }
  // one stick per punch; missed punches hollow-ish
  for (const p of punches) {
    ctx.strokeStyle = COL[p.hand]; ctx.globalAlpha = p.landed ? 0.95 : 0.5; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(X(p.t), Y(0)); ctx.lineTo(X(p.t), Y(p.g)); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  // 30 s average power: the line a tiring boxer watches sag
  ctx.strokeStyle = "#e8842c"; ctx.lineWidth = 2; ctx.beginPath();
  let pen = false;
  for (let t = t0; t < t1; t += 30) {
    const gs = punches.filter((p) => p.t >= t && p.t < t + 30).map((p) => p.g);
    if (!gs.length) { pen = false; continue; }
    const y = Y(mean(gs)), xa = X(t), xb = X(Math.min(t + 30, t1));
    pen ? ctx.lineTo(xa, y) : ctx.moveTo(xa, y);
    ctx.lineTo(xb, y); pen = true;
  }
  ctx.stroke();
}

function renderTimer() {
  const tile = $("timer-tile");
  const now = clock();
  let secs, state, round = "";
  if (timer.mode === "work" || timer.mode === "rest") {
    secs = Math.max(0, Math.ceil(timer.endsAt - now));
    state = timer.mode === "work" ? "work" : "rest";
    round = `round ${timer.round + (timer.mode === "rest" ? 1 : 0)} / ${timer.cfg.rounds}` +
            (timer.mode === "rest" ? " next" : "");
  } else {
    secs = punches.length ? Math.floor(now - punches[0].t) : 0;
    state = timer.mode === "done" ? "finished" : live ? "free session" : "loaded session";
    if (timer.mode === "done") secs = 0;
  }
  tile.className = `tile ${timer.mode}` + (timer.mode === "work" && secs <= 10 ? " last10" : "");
  $("timer-state").textContent = state;
  $("timer-round").textContent = round;
  $("timer-clock").textContent = `${Math.floor(secs / 60)}:${String(secs % 60).padStart(2, "0")}`;
}

function render() {
  const cur = timer.mode === "work" ? punches.filter((p) => p.round === timer.round) : punches;
  const workSpan = rounds.length ? rounds.reduce((s, r) => s + roundSpan(r), 0) : freeSpan(punches);
  // between and after rounds the pace is the session's over time actually
  // worked, not a number that decays while the boxer sits on the stool
  const curSpan = timer.mode === "work" ? roundSpan(rounds[rounds.length - 1]) : workSpan;
  const all = statsOf(punches, workSpan);
  const now = statsOf(cur, curSpan);

  $("n-total").textContent = all.n;
  $("n-round").textContent = now.n;
  $("n-round-label").textContent = timer.mode === "work" ? `round ${timer.round}` : "session";
  $("n-pace").textContent = fmt(now.pace, 0);
  const t = clock();
  $("n-pace10").textContent = punches.filter((p) => t - p.t <= 10).length * 6;
  $("n-pause").textContent = fmt(now.pause, 0);

  renderHand("L", punches); renderHand("R", punches);
  const share = all.n ? all.L / all.n : 0.5;
  $("balance-L").style.width = `${share * 100}%`;
  $("balance-txt").textContent = all.n ? `${Math.round(share * 100)}% left · ${Math.round((1 - share) * 100)}% right` : "–";

  $("c-n").textContent = all.combos.length;
  $("c-max").textContent = all.comboMax;
  $("c-avg").textContent = fmt(mean(all.combos.map((c) => c.length)));
  renderChips();
  renderRounds(all);
  renderTimeline();
  renderTimer();
}

// the combination in flight, or the one just finished
function renderChips() {
  const box = $("combo-chips");
  const t = clock();
  const tail = [];
  for (let i = punches.length - 1; i >= 0; i--) {
    const next = tail.length ? tail[0].t : t;
    if (next - punches[i].t > (tail.length ? COMBO_GAP_S : COMBO_SHOW_S)) break;
    tail.unshift(punches[i]);
  }
  box.innerHTML = tail.map((p) => `<span class="chip ${p.hand} ${p.landed ? "" : "miss"}">${p.hand}</span>`).join("");
}

setInterval(() => {
  if (live) { tickTimer(); renderTimer(); renderChips(); }
  for (const n of ["L", "R"]) $(`badge-${n}`).classList.toggle("on", Date.now() - lastSeen[n] < 3000);
}, 200);
setInterval(() => { if (live) render(); }, 2000);   // pace and timeline keep moving between punches
new ResizeObserver(renderTimeline).observe($("timeline"));

// ---------------------------------------------------------------- io

function connect() {
  if (ws) { ws.close(); ws = null; }
  live = true;
  ws = new WebSocket($("ws-url").value);
  const badge = $("badge-host");
  ws.onopen = () => badge.classList.add("on");
  ws.onclose = () => badge.classList.remove("on");
  ws.onerror = () => badge.classList.remove("on");
  ws.onmessage = (e) => handle(JSON.parse(e.data));
}

$("btn-connect").onclick = () => { resetSession(); connect(); };
$("btn-reset").onclick = resetSession;
$("btn-swap").onclick = (e) => {
  swap = !swap;
  e.target.classList.toggle("active", swap);
  for (const p of punches) p.hand = p.hand === "L" ? "R" : "L";
  render();
};

// A past session is scored in one pass rather than replayed: a boxer wants the
// sheet, not to sit through the round again.
$("load-file").onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  if (ws) { ws.close(); ws = null; }
  live = false;
  resetSession();
  const flip = swap; swap = false;
  for (const line of (await file.text()).split("\n")) {
    if (line) handle(JSON.parse(line));
  }
  if (flip) { swap = true; for (const p of punches) p.hand = p.hand === "L" ? "R" : "L"; }
  render();
};

connect();
render();
