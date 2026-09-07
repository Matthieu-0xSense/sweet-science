#!/usr/bin/env python3
"""
Fit the FSR counts -> newtons constant against a bathroom/kitchen scale.

    python calibrate_fsr.py --node L --channel f0
    python calibrate_fsr.py --node L --channel f0 --r-fixed 100000

Method: the sensor sits on a scale with a rigid puck over its 9.53 mm sensing
area; you press through the puck, hold, and type the scale reading. The script
takes the median of a short window of live samples for each point, then fits

    G = 1 / R_fsr = k * F

which is the relation the FlexiForce datasheet's recommended circuit is built
around — conductance is what is linear in force, not resistance and not the
divider's output voltage.

Two things the datasheet insists on and this script enforces:

  * Conditioning. Repeatability and hysteresis are specified for a
    *conditioned* sensor. The script walks you through loading it several
    times before any point is recorded; skipping that leaves the fit drifting
    under you.
  * A rigid puck. Without one the load spreads across the substrate and only a
    fraction reaches the element, so the fit describes your fingertip rather
    than the sensor.

Writes fsr_calib.json next to this file; boxe_host.py picks it up on start.
"""

import argparse
import asyncio
import json
import statistics
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

from boxe_host import CHAR_IMU, parse_imu
from fsr_calib import CALIB_PATH, counts_to_ohms

G_PER_KG = 9.80665
SETTLE_S = 2.0          # window averaged per calibration point
CONDITION_CYCLES = 5


class Sampler:
    """Keeps the most recent stream samples for one FSR channel."""

    def __init__(self, node: str, channel: str):
        self.node = node
        self.channel = channel
        self.recent: list[int] = []

    def on_imu(self, _handle, data: bytearray):
        for s in parse_imu(bytes(data), self.node):
            self.recent.append(s[self.channel])
        del self.recent[:-400]

    def window(self, seconds: float) -> list[int]:
        """Median-friendly window: the stream runs at 100 Hz per sample."""
        want = int(seconds * 100)
        return self.recent[-want:]

    def clear(self):
        self.recent.clear()


async def capture(sampler: Sampler, seconds: float = SETTLE_S) -> tuple[float, float]:
    sampler.clear()
    await asyncio.sleep(seconds)
    w = sampler.window(seconds)
    if not w:
        return 0.0, 0.0
    return statistics.median(w), max(w)


def fit(points: list[tuple[float, float]], r_fixed: float, baseline: float):
    """Least squares through the origin: k = sum(G*F) / sum(F*F)."""
    num = den = 0.0
    rows = []
    for newtons, counts in points:
        ohms = counts_to_ohms(counts + baseline, r_fixed)
        if ohms in (0.0, float("inf")):
            print(f"  skipping {newtons:.1f} N: {counts:.0f} counts is out of range")
            continue
        g = 1.0 / ohms
        rows.append((newtons, counts, ohms, g))
        num += g * newtons
        den += newtons * newtons
    if den == 0.0 or not rows:
        return None, rows, 0.0
    k = num / den
    # R^2 of the conductance fit
    gs = [r[3] for r in rows]
    mean_g = sum(gs) / len(gs)
    ss_tot = sum((g - mean_g) ** 2 for g in gs)
    ss_res = sum((g - k * f) ** 2 for f, _c, _o, g in rows)
    r2 = 1.0 - ss_res / ss_tot if ss_tot else 1.0
    return k, rows, r2


async def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", choices=["L", "R"], default="L")
    ap.add_argument("--channel", choices=["f0", "f1"], default="f0")
    ap.add_argument("--r-fixed", type=float, default=47000.0,
                    help="divider's fixed resistor in ohms (default 47000)")
    ap.add_argument("--out", type=Path, default=CALIB_PATH)
    args = ap.parse_args()

    name = f"BOXE-{args.node}"
    print(f"scanning for {name} ...")
    dev = await BleakScanner.find_device_by_name(name, timeout=20.0)
    if dev is None:
        sys.exit(f"{name} not found. Is boxe_host.py holding the connection?")

    sampler = Sampler(args.node, args.channel)
    async with BleakClient(dev) as client:
        await client.start_notify(CHAR_IMU, sampler.on_imu)
        print(f"connected to {dev.address}, reading {args.channel}\n")

        print("=" * 68)
        print("SETUP")
        print("  1. Sensor flat on the scale, sensing pad (the round tip) up.")
        print("  2. Rigid puck centred on the pad — a coin or a ~9 mm disc.")
        print("     Without it you are calibrating your fingertip, not the sensor.")
        print("  3. Press only through the puck, straight down.")
        print("=" * 68)
        input("\npress Enter when the sensor is set up ... ")

        print(f"\nCONDITIONING — {CONDITION_CYCLES} cycles at the hardest force "
              f"you intend to measure.")
        print("The datasheet's repeatability and hysteresis figures assume this.")
        for i in range(1, CONDITION_CYCLES + 1):
            input(f"  cycle {i}/{CONDITION_CYCLES}: press hard, release, Enter ... ")

        print("\nZERO — hands off the sensor completely.")
        input("  press Enter, then do not touch it for 2 s ... ")
        baseline, _ = await capture(sampler)
        print(f"  baseline = {baseline:.0f} counts")

        print("\nPOINTS — press and hold steady, read the scale, type the value.")
        print("Aim for 8 or so levels spread across your working range.")
        print("Blank line to finish.\n")

        points: list[tuple[float, float]] = []
        while True:
            raw = input(f"  [{len(points)+1}] scale reading in kg (blank = done): ").strip()
            if not raw:
                break
            try:
                kg = float(raw.replace(",", "."))
            except ValueError:
                print("      not a number")
                continue
            print("      hold it ... ", end="", flush=True)
            med, peak = await capture(sampler)
            rel = max(0.0, med - baseline)
            newtons = kg * G_PER_KG
            print(f"{rel:.0f} counts (peak {peak - baseline:.0f})  ->  {newtons:.1f} N")
            points.append((newtons, rel))

        await client.stop_notify(CHAR_IMU)

    if len(points) < 2:
        sys.exit("\nneed at least 2 points to fit")

    k, rows, r2 = fit(points, args.r_fixed, baseline)
    if k is None:
        sys.exit("\nno usable points")

    print("\n" + "=" * 68)
    print(f"{'force N':>10} {'counts':>8} {'R_fsr':>12} {'G uS':>9} {'fit N':>9} {'err':>8}")
    for newtons, counts, ohms, g in rows:
        pred = g / k
        print(f"{newtons:10.1f} {counts:8.0f} {ohms:11.0f}Ω {g*1e6:9.2f} "
              f"{pred:9.1f} {pred-newtons:+8.1f}")
    print("=" * 68)
    print(f"k  = {k:.6e} S/N")
    print(f"R2 = {r2:.4f}")
    if r2 < 0.95:
        print("  low R2 — suspect an unconditioned sensor, a missing puck, or")
        print("  the load shifting between points.")

    out = {
        "k_siemens_per_newton": k,
        "r_fixed_ohms": args.r_fixed,
        "baseline_counts": baseline,
        "channel": args.channel,
        "node": args.node,
        "r_squared": r2,
        "points": [{"newtons": n, "counts": c} for n, c in points],
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    args.out.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(f"\nwrote {args.out}")
    print("boxe_host.py will fill force_n from it on next start.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
