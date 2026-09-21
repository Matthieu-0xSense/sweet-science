#!/usr/bin/env python3
"""Hand-label punches while a raw capture runs: one key per punch.

Writes one line per label to logs/labels_<time>.txt:

    <host unix time>  <hand L|R|?>  <type>  <hit|miss>
    <host unix time>  sync

sweep.py matches detections against these, so this is what turns a
recording into a fit rather than a guess. Hand and outcome are what let it
fit each node on its own punches and score the contact flag, and type is
what a later classifier trains on — a timestamp alone cannot give any of
that back afterwards.

    python mark.py --hand R                  # one glove, right hand
    python mark.py --stance orthodox         # both gloves, jab=L cross=R
    python mark.py --note "3x3min bag"

Keys (no Enter needed):

    j c h u     landed jab / cross / hook / uppercut
    J C H U     the same, missed (shift = miss)
    space       landed, type unknown
    m           missed, type unknown
    l / r       hand for the following labels (hooks, uppercuts, unknowns)
    s           sync mark: both gloves tapped together, in front of the camera
    x           delete the last label
    q / Ctrl-C  stop

With --stance, jab and cross pick the hand themselves (orthodox: jab=L,
cross=R; southpaw: the reverse). Every other key uses the current hand, which
starts at --hand and follows l / r.

Someone other than the boxer should press the keys: reaction time adds a
roughly constant lag, and sweep.py reports it as `lag_ms` so a systematic
offset is visible rather than silently absorbed into the tolerance. A jab
lands about 250 ms after it starts, so keep --tol above the lag.

The sync mark is the same idea for the camera: hit the two gloves together
once at the start, with the phone filming. That instant is a spike on both
nodes, a frame on the video and an `s` line here, which is what lines all
three up afterwards.
"""

import argparse
import sys
import time
from pathlib import Path

LOG_DIR = Path(__file__).parent.parent / "logs"

TYPES = {"j": "jab", "c": "cross", "h": "hook", "u": "uppercut"}
STANCE_HAND = {
    "orthodox": {"jab": "L", "cross": "R"},
    "southpaw": {"jab": "R", "cross": "L"},
}


def getch():
    """One key, no Enter, no echo. Returns '' on EOF."""
    if sys.platform == "win32":
        import msvcrt
        ch = msvcrt.getwch()
        if ch in ("\x00", "\xe0"):     # arrow / function key prefix
            msvcrt.getwch()
            return ""
        return ch
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def parse_labels(path: Path):
    """Read a labels file. Returns (labels, syncs).

    labels: list of dicts {t, hand, type, hit}; syncs: list of times. The
    original one-timestamp-per-line format is still read, as hand '?',
    type 'punch', hit True.
    """
    labels, syncs = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        t = float(parts[0])
        if len(parts) == 1:
            labels.append(dict(t=t, hand="?", type="punch", hit=True))
        elif parts[1] == "sync":
            syncs.append(t)
        else:
            hand, typ, outcome = parts[1], parts[2], parts[3]
            labels.append(dict(t=t, hand=hand, type=typ, hit=outcome == "hit"))
    labels.sort(key=lambda x: x["t"])
    return labels, syncs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, help="output file")
    ap.add_argument("--note", default="", help="stored as a comment")
    ap.add_argument("--hand", choices=("L", "R"), default="?",
                    help="initial hand for labels that do not imply one")
    ap.add_argument("--stance", choices=tuple(STANCE_HAND),
                    help="jab/cross pick the hand from the stance")
    args = ap.parse_args()

    LOG_DIR.mkdir(exist_ok=True)
    out = args.out or LOG_DIR / time.strftime("labels_%Y%m%d_%H%M%S.txt")

    hand = args.hand
    lines = []          # (t, text) so `x` can take the last one back
    t_first = None
    print(f"marking to {out}")
    print("j/c/h/u landed  J/C/H/U missed  space landed?  m missed?  "
          "l/r hand  s sync  x undo  q quit")

    def status():
        n = sum(1 for _, s in lines if "sync" not in s)
        el = (time.time() - t_first) if t_first else 0.0
        last = lines[-1][1] if lines else "-"
        print(f"\r{n:4d} labels {el:6.1f}s  hand {hand}  last: {last:<24}",
              end="", flush=True)

    def header(f):
        f.write("# punch labels: <host unix time> <hand> <type> <hit|miss>, "
                "or <time> sync\n")
        if args.note:
            f.write(f"# {args.note}\n")
        if args.stance:
            f.write(f"# stance {args.stance}\n")

    with open(out, "w", encoding="utf-8", buffering=1) as f:
        header(f)
        try:
            while True:
                ch = getch()
                t = time.time()
                if ch in ("q", "\x03", "\x04", ""):
                    break
                if ch in ("l", "r"):
                    hand = ch.upper()
                    status()
                    continue
                if ch == "x":
                    if lines:
                        lines.pop()
                        f.seek(0)
                        f.truncate()
                        header(f)
                        for _, s in lines:
                            f.write(s + "\n")
                    status()
                    continue

                if ch == "s":
                    text = f"{t:.3f} sync"
                elif ch == " ":
                    text = f"{t:.3f} {hand} punch hit"
                elif ch == "m":
                    text = f"{t:.3f} {hand} punch miss"
                elif ch.lower() in TYPES:
                    typ = TYPES[ch.lower()]
                    h = hand
                    if args.stance and typ in STANCE_HAND[args.stance]:
                        h = STANCE_HAND[args.stance][typ]
                    outcome = "miss" if ch.isupper() else "hit"
                    text = f"{t:.3f} {h} {typ} {outcome}"
                else:
                    continue

                if t_first is None:
                    t_first = t
                lines.append((t, text))
                f.write(text + "\n")
                status()
        except KeyboardInterrupt:
            pass

    n = sum(1 for _, s in lines if "sync" not in s)
    print(f"\n{n} labels -> {out}")


if __name__ == "__main__":
    main()
