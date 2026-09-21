#!/usr/bin/env python3
"""Fit the punch-detection thresholds offline against a raw capture.

The firmware constants in boxe.h were guessed, never fitted. Changing them on
the board costs a rebuild, a reflash of two nodes and a fresh set of punches,
so no two candidates are ever scored on the same data. This scores every
candidate on one recording.

    # 1. record, with the punches counted by hand as you throw them
    python boxe_host.py --ble --raw          # writes logs/raw_L_*.bin
    python mark.py --hand R                  # one key per punch -> labels

    # 2. fit
    python sweep.py ../logs/raw_L_*.bin --labels ../logs/labels_*.txt

    # 3. paste the printed block into firmware/src/boxe.h, reflash

Scoring needs ground truth. Two kinds, in order of usefulness:

  --labels FILE   from mark.py: time, hand, type, hit/miss per punch. Only
                  this node's hand is scored. Detections are matched to
                  labels within --tol seconds; the score is F1, so a run that
                  fires twice per punch is punished as hard as one that misses
                  punches. hit_acc is how often the detected contact flag
                  agrees with the label. This is the one to use.
  --truth N       total punch count only. Scores |detected - N|, which cannot
                  tell a missed punch plus a false positive from a clean
                  result. Use it when a capture was not labelled.

With neither, every combination is still replayed and reported, but ranked
only by how plausible the event shape is (see plausibility()) -- treat that as
a shortlist to eyeball, not a fit.
"""

import argparse
import itertools
import json
import statistics
import sys
import time
from pathlib import Path

import mark
import punch_detect
import rawlog
from punch_detect import Params, PunchDetector

# Deliberately coarse. A fine grid over a capture of 300 punches invites
# fitting the noise; land in the right neighbourhood, then refine.
GRID = {
    "hg_start_lsb": [40, 60, 80, 120, 160, 200, 280, 400],   # 2 g .. 20 g
    "fsr_contact": [100, 200, 300, 450, 600],
    "window_ms": [150, 250, 400],
    "refract_ms": [100, 150, 250],
    "quiet_ms": [30],
    "baseline_mode": ["idle_refract", "always"],
    "open_on": ["hg"],
    "contact_end_pct": [0, 50, 75],
}
STR_PARAMS = ("baseline_mode", "open_on")


def parse_values(spec: str, cast=int):
    """"100,200,300" or "100:600:100" (inclusive of the stop)."""
    if ":" in spec:
        a, b, step = (cast(x) for x in spec.split(":"))
        out, v = [], a
        while v <= b:
            out.append(v)
            v += step
        return out
    return [cast(x.strip()) for x in spec.split(",") if x.strip()]


def load_labels(path: Path, node: str):
    """Labels for this node's hand, plus the sync marks.

    mark.py writes hand, type and outcome per label; the capture knows which
    hand it is, so the other hand's punches are dropped here rather than
    counted as misses. Labels marked '?' (or the old timestamp-only format)
    are kept for either node.
    """
    labels, syncs = mark.parse_labels(path)
    mine = [x for x in labels if x["hand"] in (node, "?")]
    return mine, syncs


def match(det_times, times, tol):
    """Greedy nearest match. Returns {label index: detection index}.

    Greedy is fine here because labels are seconds apart and tol is a fraction
    of a second; the optimal assignment and the greedy one agree unless the
    tolerance is set wider than the gap between punches.
    """
    used = [False] * len(det_times)
    pairs = {}
    for j, lab in enumerate(times):
        best, best_d = -1, tol
        for i, t in enumerate(det_times):
            if used[i]:
                continue
            d = abs(t - lab)
            if d <= best_d:
                best, best_d = i, d
        if best >= 0:
            used[best] = True
            pairs[j] = best
    return pairs


def score_against_labels(events, det_times, labels, tol):
    """Returns tp, fp, fn, precision, recall, f1, median lag, contact accuracy.

    Contact accuracy is the share of matched punches whose detected `contact`
    flag agrees with the label's hit/miss — the hit-rate metric is only as
    good as this number.
    """
    pairs = match(det_times, [x["t"] for x in labels], tol)
    tp = len(pairs)
    fp = len(det_times) - tp
    fn = len(labels) - tp
    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
    offsets = [det_times[i] - labels[j]["t"] for j, i in pairs.items()]
    agree = sum(1 for j, i in pairs.items()
                if events[i].contact == labels[j]["hit"])
    hit_acc = agree / tp if tp else 0.0
    return (tp, fp, fn, prec, rec, f1,
            statistics.median(offsets) if offsets else 0.0, hit_acc)


def plausibility(events):
    """Weak unsupervised score, for captures with no ground truth.

    Rewards what a real punch stream looks like: most events make contact,
    contact pulses are short rather than pinned to the detector's own window
    ceiling, and execution times are not zero. It cannot rank two good
    parameter sets against each other -- it only pushes obviously broken ones
    down the list.
    """
    if not events:
        return 0.0
    contact = [e for e in events if e.contact]
    if not contact:
        return 0.0
    frac_contact = len(contact) / len(events)
    widths = [e.width_ms for e in contact]
    frac_sane_width = sum(1 for w in widths if 3 <= w <= 60) / len(widths)
    execs = [e.exec_ms for e in events]
    frac_sane_exec = sum(1 for x in execs if 30 <= x <= 500) / len(execs)
    # Mean, not product: one implausible aspect should push a candidate down
    # the list, not flatten the whole column to zero and destroy the ordering.
    return round((frac_contact + frac_sane_width + frac_sane_exec) / 3, 4)


def replay(samples, params: Params):
    det = PunchDetector(params)
    out = []
    for t_us, f0, f1, mag in samples:
        ev = det.feed_mag(t_us, f0, f1, mag)
        if ev is not None:
            out.append(ev)
    return out


def verify_port(samples_raw, samples_fast):
    """feed() and feed_mag() must agree — cheap insurance on the precompute."""
    a = punch_detect.run(samples_raw[:20000], Params())
    b = replay(samples_fast[:20000], Params())
    if [e.t_us for e in a] != [e.t_us for e in b]:
        sys.exit("internal error: precomputed magnitudes changed the result")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("--labels", type=Path,
                    help="file of host timestamps, one punch per line")
    ap.add_argument("--truth", type=int, help="known total punch count")
    ap.add_argument("--tol", type=float, default=0.25,
                    help="label match tolerance, seconds (default 0.25)")
    ap.add_argument("--top", type=int, default=15, help="rows to print")
    ap.add_argument("--json", type=Path, help="write the full ranking")
    for name, values in GRID.items():
        ap.add_argument(f"--{name.replace('_', '-')}",
                        default=",".join(str(v) for v in values),
                        help=f"comma-separated, default {values}")
    args = ap.parse_args()

    grid = {k: parse_values(getattr(args, k), str if k in STR_PARAMS else int)
            for k in GRID}

    reader = rawlog.RawReader(args.capture)
    print(f"[sweep] loading {args.capture.name}", flush=True)
    samples_raw = list(reader.samples())
    if not samples_raw:
        sys.exit("capture has no samples")
    offset = reader.clock_offset()

    # A hole in the capture is not a quiet moment: the replayed state machine
    # cannot see across it, and every candidate is scored on a recording that
    # is missing punches nobody can account for. Say so before the fit rather
    # than after someone has pasted the numbers into boxe.h.
    st = rawlog.stats(args.capture)
    if st["loss_pct"] > 1.0 or st["seq_restarts"]:
        print(f"[sweep] WARNING: {st['loss_pct']}% of this capture is missing "
              f"({st['queue_dropped_packets']} queue, "
              f"{st['radio_dropped_packets']} radio"
              + (f", {st['seq_restarts']} seq restart(s)"
                 if st["seq_restarts"] else "") + ").")
        print("[sweep] Fitted thresholds will reflect the holes. Re-record "
              "one node at a time (boxe_host.py --raw L) before trusting "
              "this.")
    # hg_mag() has no parameters, so it is computed once here rather than
    # once per candidate — this is most of the runtime of the whole sweep.
    samples = [(s.t_us, s.f0, s.f1, punch_detect.hg_mag(s.hgx, s.hgy, s.hgz))
               for s in samples_raw]
    dur = len(samples) / reader.rate_hz
    print(f"[sweep] {len(samples)} samples, {dur:.1f} s, node {reader.node}")

    verify_port(samples_raw, samples)
    del samples_raw

    labels, syncs = (load_labels(args.labels, reader.node)
                     if args.labels else (None, []))
    if labels:
        hits = sum(1 for x in labels if x["hit"])
        print(f"[sweep] {len(labels)} labelled punches for node {reader.node} "
              f"({hits} hit, {len(labels) - hits} miss), tol {args.tol}s")
    elif args.truth:
        print(f"[sweep] scoring against a count of {args.truth}")
    elif args.labels:
        print(f"[sweep] WARNING: {args.labels.name} has no labels for node "
              f"{reader.node} — ranking by plausibility only")
    else:
        print("[sweep] no ground truth — ranking by plausibility only")
    if syncs:
        # The sync clap is an impact on both nodes and a keypress: with the
        # firmware thresholds it should be detected, and its lag is the
        # labeller's reaction time with no swing in front of it.
        det = [offset + e.t_us / 1e6 for e in replay(samples, Params())]
        pairs = match(det, syncs, 1.0)
        lags = [round((det[i] - syncs[j]) * 1000) for j, i in pairs.items()]
        print(f"[sweep] {len(syncs)} sync mark(s), {len(pairs)} detected"
              + (f", lag {lags} ms" if lags else ""))

    keys = list(grid)
    combos = list(itertools.product(*(grid[k] for k in keys)))
    print(f"[sweep] {len(combos)} combinations", flush=True)

    rows = []
    t0 = time.time()
    for i, values in enumerate(combos):
        params = Params(**dict(zip(keys, values)))
        events = replay(samples, params)
        det_times = [offset + e.t_us / 1e6 for e in events]

        row = dict(params.as_dict())
        row["events"] = len(events)
        row["contact_pct"] = (round(100 * sum(1 for e in events if e.contact)
                                    / len(events), 1) if events else 0.0)
        row["plausibility"] = plausibility(events)
        row["median_peak_g"] = (round(statistics.median(e.peak_g for e in events), 1)
                                if events else 0.0)

        if labels:
            tp, fp, fn, prec, rec, f1, off, hit_acc = score_against_labels(
                events, det_times, labels, args.tol)
            row.update(tp=tp, fp=fp, fn=fn, prec=round(prec, 3),
                       recall=round(rec, 3), f1=round(f1, 4),
                       lag_ms=round(off * 1000, 1), hit_acc=round(hit_acc, 3))
            row["score"] = row["f1"]
        elif args.truth:
            row["count_err"] = abs(len(events) - args.truth)
            row["score"] = round(1 / (1 + row["count_err"]), 4)
        else:
            row["score"] = row["plausibility"]
        rows.append(row)

        if (i + 1) % 25 == 0 or i + 1 == len(combos):
            el = time.time() - t0
            print(f"\r[sweep] {i + 1}/{len(combos)}  {el:.0f}s", end="",
                  flush=True)
    print()

    rows.sort(key=lambda r: (-r["score"], -r["plausibility"]))

    cols = ["hg_start_lsb", "fsr_contact", "window_ms", "refract_ms",
            "baseline_mode", "contact_end_pct", "events", "contact_pct",
            "median_peak_g", "plausibility"]
    if labels:
        cols += ["tp", "fp", "fn", "prec", "recall", "f1", "lag_ms", "hit_acc"]
    elif args.truth:
        cols += ["count_err"]

    widths = [max(len(c), 9) for c in cols]
    print()
    print("  ".join(c.rjust(w) for c, w in zip(cols, widths)))
    for row in rows[:args.top]:
        print("  ".join(str(row[c]).rjust(w) for c, w in zip(cols, widths)))

    best = rows[0]
    print("\nbest:")
    print(f"""
/* fitted on {args.capture.name} ({dur:.0f} s{
    f", {len(labels)} labelled punches" if labels else ""}) */
#define PUNCH_HG_START_LSB   {best['hg_start_lsb']}
#define PUNCH_FSR_CONTACT    {best['fsr_contact']}
#define PUNCH_WINDOW_MS      {best['window_ms']}
#define PUNCH_REFRACT_MS     {best['refract_ms']}
#define PUNCH_OPEN_ON_HG     {1 if best['open_on'] == 'hg' else 0}
#define PUNCH_CONTACT_END_PCT {best['contact_end_pct']}""".rstrip())
    if best["baseline_mode"] == "always":
        print("\n/* NOTE: this fit wants baseline tracking in every state, "
              "which\n   punch_detect.c does not do — see the baseline_mode "
              "note in\n   punch_detect.py before pasting the numbers. */")

    if args.json:
        args.json.write_text(json.dumps({
            "capture": str(args.capture),
            "samples": len(samples),
            "duration_s": round(dur, 2),
            "labels": len(labels) if labels else None,
            "truth": args.truth,
            "rows": rows,
        }, indent=2), encoding="utf-8")
        print(f"\nfull ranking -> {args.json}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError) as exc:
        sys.exit(f"error: {exc}")
