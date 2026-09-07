#!/usr/bin/env python3
"""Hand-label punches while a raw capture runs: Enter once per punch.

Writes host timestamps, one per line, to logs/labels_<time>.txt. sweep.py
matches detections against these, so this is what turns a recording into a
fit rather than a guess.

    python mark.py                    # Enter per punch, Ctrl-C to stop
    python mark.py --note "3x3min bag, right hand only"

Someone other than the boxer should press the key: reaction time adds a
roughly constant lag, and sweep.py reports it as `lag_ms` so a systematic
offset is visible rather than silently absorbed into the tolerance. A jab
lands about 250 ms after it starts, so keep --tol above the lag.
"""

import argparse
import sys
import time
from pathlib import Path

LOG_DIR = Path(__file__).parent.parent / "logs"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, help="output file")
    ap.add_argument("--note", default="", help="stored as a comment")
    args = ap.parse_args()

    LOG_DIR.mkdir(exist_ok=True)
    out = args.out or LOG_DIR / time.strftime("labels_%Y%m%d_%H%M%S.txt")

    n = 0
    t_first = None
    print(f"marking to {out}")
    print("Enter = one punch, Ctrl-C = done")

    with open(out, "w", encoding="utf-8", buffering=1) as f:
        f.write(f"# punch labels, host unix time, one per line\n")
        if args.note:
            f.write(f"# {args.note}\n")
        try:
            while True:
                sys.stdin.readline()
                t = time.time()
                if t_first is None:
                    t_first = t
                n += 1
                f.write(f"{t:.3f}\n")
                print(f"\r{n} punches, {t - t_first:6.1f}s elapsed", end="",
                      flush=True)
        except KeyboardInterrupt:
            pass

    print(f"\n{n} punches -> {out}")


if __name__ == "__main__":
    main()
