#!/usr/bin/env python3
"""Boxe_AI session analyzer.

Reads a JSONL session log (logs/session_*.jsonl) and prints per-round,
per-hand statistics: output, power, timing, combos, guard proxy, fatigue.

Usage:
  python metrics.py ../logs/session_20260715_174021.jsonl
  python metrics.py session.jsonl --round 180 --rest 60      # 3 min rounds
  python metrics.py session.jsonl --json report.json

Rounds are cut on a fixed grid (round+rest) starting at the first record.
Everything is stdlib — no numpy needed at this scale.
"""

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

COMBO_GAP_S = 0.6          # max gap between punches of one combination
GUARD_QUIET_G = 1.6        # |a| close to 1 g => quasi-static
GUARD_QUIET_GYRO = 60      # dps


def mean(xs):
    return sum(xs) / len(xs) if xs else None


def median(xs):
    if not xs:
        return None
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2


def slope_per_min(pairs):
    """Least-squares slope of (t_seconds, value) -> value/minute."""
    if len(pairs) < 3:
        return None
    n = len(pairs)
    sx = sum(p[0] for p in pairs)
    sy = sum(p[1] for p in pairs)
    sxx = sum(p[0] * p[0] for p in pairs)
    sxy = sum(p[0] * p[1] for p in pairs)
    den = n * sxx - sx * sx
    if den == 0:
        return None
    return (n * sxy - sx * sy) / den * 60


def load(path):
    recs = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    recs.sort(key=lambda r: r.get("t", 0))
    return recs


def guard_pitch_series(imu, t0):
    """Forearm pitch (deg, 90 = vertical/guard-up) on quasi-static samples,
    averaged per 10 s bucket. Returns list of (t_rel_s, pitch_deg)."""
    buckets = {}
    for r in imu:
        ax, ay, az = r["ax"], r["ay"], r["az"]
        a_mag = math.sqrt(ax * ax + ay * ay + az * az) / 1000  # g
        g_mag = max(abs(r["gx"]), abs(r["gy"]), abs(r["gz"]))
        if abs(a_mag - 1.0) > (GUARD_QUIET_G - 1.0) or g_mag > GUARD_QUIET_GYRO:
            continue
        # pitch of the forearm long axis (y on the Feather, mounted along arm)
        pitch = math.degrees(math.asin(max(-1, min(1, ay / (a_mag * 1000)))))
        b = int((r["t"] - t0) // 10)
        buckets.setdefault(b, []).append(pitch)
    return [(b * 10 + 5, mean(v)) for b, v in sorted(buckets.items())]


def analyze_round(events, imu, t_start, t_end):
    ev = [e for e in events if t_start <= e["t"] < t_end]
    im = [r for r in imu if t_start <= r["t"] < t_end]
    dur_min = (t_end - t_start) / 60
    out = {}

    for hand in ("L", "R", "*"):
        e = ev if hand == "*" else [x for x in ev if x["node"] == hand]
        hits = [x for x in e if x.get("contact")]
        peaks = [x["peak_g"] for x in e if x.get("peak_g")]
        execs = [x["exec_ms"] for x in e if x.get("exec_ms")]
        retr = [x["retract_ms"] for x in e if x.get("retract_ms")]
        out[hand] = {
            "punches": len(e),
            "per_min": round(len(e) / dur_min, 1) if dur_min else 0,
            "hit_rate": round(len(hits) / len(e), 2) if e else None,
            "peak_g_mean": round(mean(peaks), 1) if peaks else None,
            "peak_g_max": round(max(peaks), 1) if peaks else None,
            "power_decay_per_min": round(
                slope_per_min([(x["t"] - t_start, x["peak_g"])
                               for x in e if x.get("peak_g")]) or 0, 2),
            "exec_ms_median": round(median(execs), 1) if execs else None,
            "retract_ms_median": round(median(retr), 1) if retr else None,
        }

    # combos: both hands merged, chronological
    seq = sorted(ev, key=lambda x: x["t"])
    combos, cur = [], []
    for x in seq:
        if cur and x["t"] - cur[-1]["t"] <= COMBO_GAP_S:
            cur.append(x)
        else:
            if len(cur) >= 2:
                combos.append(cur)
            cur = [x]
    if len(cur) >= 2:
        combos.append(cur)
    patterns = Counter("".join(p["node"] for p in c) for c in combos)
    gaps = [b["t"] - a["t"] for a, b in zip(seq, seq[1:])
            if b["t"] - a["t"] <= COMBO_GAP_S]
    out["combos"] = {
        "count": len(combos),
        "longest": max((len(c) for c in combos), default=0),
        "top_patterns": patterns.most_common(5),
        "intra_gap_ms_median": round(median(gaps) * 1000, 0) if gaps else None,
    }

    # movement intensity: accel RMS around gravity (mg), whole round
    if im:
        dev = [math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2) - 1000
               for r in im]
        out["movement_rms_mg"] = round(math.sqrt(mean([d * d for d in dev])), 0)
    else:
        out["movement_rms_mg"] = None

    return out


def analyze(recs, round_s, rest_s):
    events = [r for r in recs if r["type"] == "event"]
    imu = [r for r in recs if r["type"] == "imu"]
    if not recs:
        sys.exit("empty log")
    t0, t_last = recs[0]["t"], recs[-1]["t"]

    report = {"session_s": round(t_last - t0, 1), "rounds": []}
    t = t0
    n = 1
    while t < t_last:
        r_end = min(t + round_s, t_last)
        report["rounds"].append({
            "round": n, "t_start_s": round(t - t0, 1),
            **analyze_round(events, imu, t, r_end),
        })
        t = r_end + rest_s
        n += 1

    report["guard"] = {
        hand: guard_pitch_series([r for r in imu if r["node"] == hand], t0)
        for hand in ("L", "R")
    }
    # fatigue summary: round-1 vs last-round per_min and peak_g
    rr = report["rounds"]
    if len(rr) >= 2:
        report["fatigue"] = {
            "output_drop_pct": _drop(rr[0]["*"]["per_min"], rr[-1]["*"]["per_min"]),
            "power_drop_pct": _drop(rr[0]["*"]["peak_g_mean"], rr[-1]["*"]["peak_g_mean"]),
        }
    return report


def _drop(first, last):
    if not first or last is None:
        return None
    return round((first - last) / first * 100, 1)


def print_report(rep):
    print(f"session: {rep['session_s']} s, {len(rep['rounds'])} round(s)\n")
    hdr = f"{'rd':>2} {'hand':>4} {'n':>4} {'/min':>5} {'hit%':>5} " \
          f"{'g_avg':>6} {'g_max':>6} {'decay':>6} {'exec':>6} {'retr':>6}"
    print(hdr)
    print("-" * len(hdr))
    for r in rep["rounds"]:
        for hand in ("L", "R", "*"):
            s = r[hand]
            hit = f"{int(s['hit_rate']*100)}" if s["hit_rate"] is not None else "-"
            print(f"{r['round']:>2} {hand:>4} {s['punches']:>4} "
                  f"{s['per_min']:>5} {hit:>5} "
                  f"{s['peak_g_mean'] or '-':>6} {s['peak_g_max'] or '-':>6} "
                  f"{s['power_decay_per_min'] or '-':>6} "
                  f"{s['exec_ms_median'] or '-':>6} "
                  f"{s['retract_ms_median'] or '-':>6}")
        c = r["combos"]
        pats = " ".join(f"{p}x{n}" for p, n in c["top_patterns"])
        print(f"   combos: {c['count']} (max {c['longest']}) "
              f"gap {c['intra_gap_ms_median'] or '-'} ms | {pats}")
        print(f"   movement RMS: {r['movement_rms_mg'] or '-'} mg\n")
    if "fatigue" in rep:
        f = rep["fatigue"]
        print(f"fatigue: output {f['output_drop_pct']}% | "
              f"power {f['power_drop_pct']}% (round 1 -> last)")
    for hand in ("L", "R"):
        pts = rep["guard"][hand]
        if pts:
            first, last = pts[0][1], pts[-1][1]
            print(f"guard {hand}: pitch {first:.0f}° -> {last:.0f}° "
                  f"({len(pts)} buckets)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logfile", type=Path)
    ap.add_argument("--round", type=int, default=180, help="round length s")
    ap.add_argument("--rest", type=int, default=60, help="rest length s")
    ap.add_argument("--json", type=Path, help="also write full report JSON")
    args = ap.parse_args()

    rep = analyze(load(args.logfile), args.round, args.rest)
    print_report(rep)
    if args.json:
        args.json.write_text(json.dumps(rep, indent=2), encoding="utf-8")
        print(f"\nreport written to {args.json}")


if __name__ == "__main__":
    main()
