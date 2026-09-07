#!/usr/bin/env python3
"""Python port of firmware/src/punch_detect.c, with the thresholds exposed.

The point of this file is to be *boring and identical*. Every candidate
threshold set is scored by replaying a raw capture through this, so if it
drifts from the C the fitted numbers are fitted against a detector that does
not exist. Integer widths and truncation are reproduced deliberately:

  * timestamps are uint32 microseconds and wrap every ~71.6 minutes, so all
    time differences go through u32_sub()
  * the EMA baseline uses C integer division, which truncates toward zero,
    not Python's floor
  * hg_mag() is the same sqrt-free approximation (hi + lo/2), not a real norm

Anything changed here has to be changed in the C, and vice versa. The one
deliberate addition is `baseline_mode`: the firmware implements exactly one of
its three settings, and the other two exist so the choice can be re-scored on
real punches rather than argued about -- see Params below.
"""

from dataclasses import dataclass, replace
from typing import Iterator, List, Optional

SAMPLE_HZ = 1000

# Defaults are the current firmware values, firmware/src/boxe.h.
DEFAULTS = dict(
    hg_start_lsb=102,     # PUNCH_HG_START_LSB, ~5 g at 49 mg/LSB
    fsr_contact=150,      # PUNCH_FSR_CONTACT, counts above baseline
    window_ms=400,        # PUNCH_WINDOW_MS
    refract_ms=150,       # PUNCH_REFRACT_MS
    quiet_ms=30,          # hardcoded at punch_detect.c:93
    baseline_mode="idle_refract",
    baseline_shift=6,     # EMA divisor is 1 << shift; the C uses 64
)

BASELINE_MODES = ("idle", "idle_refract", "always")


@dataclass(frozen=True)
class Params:
    """Detector constants.

    baseline_mode:
      "idle"         - the original tracking rule: baseline moves only in
                       IDLE. Combined with the original zero-seeded baseline
                       this deadlocked — a gloved sensor reads thousands of
                       counts above a baseline of 0, contact asserts on
                       sample #1, the state machine leaves IDLE, the baseline
                       freezes and the latch feeds itself. Measured at
                       1.82 events/s on a still node, exactly
                       1/(window_ms + refract_ms). Priming defuses that on
                       its own, so this mode is no longer the disaster it
                       was; it is kept to show what the tracking rule alone
                       is worth.
      "idle_refract" - what the firmware does now, and the default: also
                       track during REFRACT, so the baseline can catch up
                       with a preload that changed during the punch. ACTIVE
                       stays excluded or the baseline climbs into the pulse
                       and clips its tail.
      "always"       - track in every state, ACTIVE included. Kept as the
                       upper bound on how aggressive recovery can get.

    All three prime the baseline from the first sample, as the firmware now
    does. Sweeping them says how much the choice is worth on real punches
    rather than in an argument.
    """
    hg_start_lsb: int = DEFAULTS["hg_start_lsb"]
    fsr_contact: int = DEFAULTS["fsr_contact"]
    window_ms: int = DEFAULTS["window_ms"]
    refract_ms: int = DEFAULTS["refract_ms"]
    quiet_ms: int = DEFAULTS["quiet_ms"]
    baseline_mode: str = DEFAULTS["baseline_mode"]
    baseline_shift: int = DEFAULTS["baseline_shift"]

    def replace(self, **kw) -> "Params":
        return replace(self, **kw)

    def as_dict(self) -> dict:
        return dict(self.__dict__)


@dataclass
class Event:
    """Same fields as struct event_packet, same units."""
    t_us: int
    contact: bool
    saturated: bool
    peak_hg: int
    f0_peak: int
    f1_peak: int
    width_ms10: int
    impulse: int
    seq: int
    t_start_us: int
    retract_ms10: int

    @property
    def peak_g(self) -> float:
        return self.peak_hg * 0.049

    @property
    def width_ms(self) -> float:
        return self.width_ms10 / 10

    @property
    def exec_ms(self) -> float:
        return u32_sub(self.t_us, self.t_start_us) / 1000


def u32_sub(a: int, b: int) -> int:
    """(a - b) in uint32 arithmetic, as the C does it."""
    return (a - b) & 0xFFFFFFFF


def c_div(a: int, b: int) -> int:
    """C integer division: truncates toward zero. Python's // floors."""
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


def hg_mag(x: int, y: int, z: int) -> int:
    """The firmware's sqrt-free magnitude: max + half min of |components|.

    Reads about 12 % high on a diagonal impact versus a true norm, which does
    not matter for a threshold as long as the threshold is fitted against the
    same approximation. That is exactly why this is duplicated here rather
    than replaced with math.hypot.
    """
    ax, ay, az = abs(x), abs(y), abs(z)
    hi = max(ax, ay, az)
    lo = min(ax, ay, az)
    return hi + lo // 2


IDLE, ACTIVE, REFRACT = 0, 1, 2


class PunchDetector:
    """Fed one sample at a time at SAMPLE_HZ, exactly like the firmware."""

    def __init__(self, params: Params = Params()):
        self.p = params
        self.st = IDLE
        self.t_start_us = 0
        self.t_last_active_us = 0
        self.t_contact_us = 0
        self.peak_hg = 0
        self.f0_peak = self.f1_peak = 0
        self.f0_base = self.f1_base = 0
        self.impulse = 0
        self.contact_start_us = self.contact_end_us = 0
        self.contact = False
        self.seq = 0
        self._primed = False

    def feed(self, t_us: int, f0: int, f1: int,
             hgx: int, hgy: int, hgz: int) -> Optional[Event]:
        return self.feed_mag(t_us, f0, f1, hg_mag(hgx, hgy, hgz))

    def feed_mag(self, t_us: int, f0: int, f1: int,
                 mag: int) -> Optional[Event]:
        """As feed(), with the magnitude already computed.

        hg_mag() does not depend on any threshold, so a sweep computes it once
        per sample instead of once per candidate parameter set. Splitting it
        out here rather than inlining the state machine into the sweep keeps
        one copy of the logic.
        """
        p = self.p

        # Seed from the first sample, as punch_detect.c now does. Walking up
        # from a zero baseline meant the first sample of a gloved sensor read
        # thousands of counts "above baseline" and latched contact on sample
        # #1 — see baseline_mode in Params.
        if not self._primed:
            self.f0_base, self.f1_base = f0, f1
            self._primed = True

        div = 1 << p.baseline_shift
        if (p.baseline_mode == "always" or self.st == IDLE or
                (p.baseline_mode == "idle_refract" and self.st == REFRACT)):
            self.f0_base += c_div(f0 - self.f0_base, div)
            self.f1_base += c_div(f1 - self.f1_base, div)

        f0_rel = f0 - self.f0_base if f0 > self.f0_base else 0
        f1_rel = f1 - self.f1_base if f1 > self.f1_base else 0
        contact_now = f0_rel > p.fsr_contact or f1_rel > p.fsr_contact
        active_now = mag > p.hg_start_lsb or contact_now

        out = None

        if self.st == IDLE:
            if active_now:
                self.peak_hg = 0
                self.f0_peak = self.f1_peak = 0
                self.impulse = 0
                self.contact = False
                self.contact_start_us = self.contact_end_us = 0
                self.st = ACTIVE
                self.t_start_us = t_us
                self.t_last_active_us = t_us

        elif self.st == ACTIVE:
            self.peak_hg = max(self.peak_hg, mag)
            self.f0_peak = max(self.f0_peak, f0_rel)
            self.f1_peak = max(self.f1_peak, f1_rel)
            if contact_now:
                if not self.contact:
                    self.contact = True
                    self.contact_start_us = t_us
                    self.t_contact_us = t_us
                self.contact_end_us = t_us
                self.impulse += (f0_rel + f1_rel) // (SAMPLE_HZ // 1000)
            if active_now:
                self.t_last_active_us = t_us

            if (u32_sub(t_us, self.t_last_active_us) > p.quiet_ms * 1000 or
                    u32_sub(t_us, self.t_start_us) > p.window_ms * 1000):
                self.seq += 1
                out = Event(
                    t_us=self.t_contact_us if self.contact else self.t_start_us,
                    contact=self.contact,
                    saturated=self.peak_hg >= 4000,
                    peak_hg=self.peak_hg,
                    f0_peak=self.f0_peak,
                    f1_peak=self.f1_peak,
                    width_ms10=(u32_sub(self.contact_end_us,
                                        self.contact_start_us) // 100
                                if self.contact else 0),
                    impulse=self.impulse,
                    seq=self.seq,
                    t_start_us=self.t_start_us,
                    retract_ms10=(u32_sub(t_us, self.contact_end_us) // 100
                                  if self.contact else 0),
                )
                self.st = REFRACT
                self.t_last_active_us = t_us

        elif self.st == REFRACT:
            if u32_sub(t_us, self.t_last_active_us) > p.refract_ms * 1000:
                self.st = IDLE

        return out


def run(samples: Iterator, params: Params = Params()) -> List[Event]:
    """Replay an iterable of rawlog.Sample through a fresh detector."""
    det = PunchDetector(params)
    events = []
    for s in samples:
        ev = det.feed(s.t_us, s.f0, s.f1, s.hgx, s.hgy, s.hgz)
        if ev is not None:
            events.append(ev)
    return events


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    import rawlog

    ap = argparse.ArgumentParser(
        description="Replay a raw capture through the detector and list events")
    ap.add_argument("capture", type=Path)
    for name in ("hg_start_lsb", "fsr_contact", "window_ms", "refract_ms",
                 "quiet_ms", "baseline_shift"):
        ap.add_argument(f"--{name.replace('_', '-')}", type=int,
                        default=DEFAULTS[name])
    ap.add_argument("--baseline-mode", choices=BASELINE_MODES,
                    default=DEFAULTS["baseline_mode"])
    args = ap.parse_args()

    prm = Params(**{k: v for k, v in vars(args).items() if k != "capture"})
    reader = rawlog.RawReader(args.capture)
    evs = run(reader.samples(), prm)

    print(f"{len(evs)} events with {prm.as_dict()}")
    print(f"{'seq':>4} {'t_s':>9} {'peak_g':>7} {'contact':>8} "
          f"{'width_ms':>9} {'exec_ms':>8} {'f0':>6}")
    t0 = None
    for e in evs:
        if t0 is None:
            t0 = e.t_us
        print(f"{e.seq:4d} {u32_sub(e.t_us, t0) / 1e6:9.3f} {e.peak_g:7.1f} "
              f"{str(e.contact):>8} {e.width_ms:9.1f} {e.exec_ms:8.1f} "
              f"{e.f0_peak:6d}")
