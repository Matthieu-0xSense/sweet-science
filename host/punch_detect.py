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

Anything changed here has to be changed in the C, and vice versa. The
deliberate additions are the rule switches -- `baseline_mode`, `open_on`,
`contact_end_pct`: the firmware implements one setting of each, and the
others exist so the choice can be re-scored on real punches rather than
argued about -- see Params below.
"""

import math
from dataclasses import dataclass, replace
from typing import Iterator, List, Optional

SAMPLE_HZ = 1000

# Defaults are the current firmware values, firmware/src/boxe.h.
DEFAULTS = dict(
    hg_start_lsb=102,     # PUNCH_HG_START_LSB, ~5 g at 49 mg/LSB
    lg_start_mg=3000,     # PUNCH_LG_START_MG, LSM6 |a|; 0 = off
    fsr_contact=150,      # PUNCH_FSR_CONTACT, counts above baseline
    window_ms=400,        # PUNCH_WINDOW_MS
    refract_ms=150,       # PUNCH_REFRACT_MS
    quiet_ms=100,         # PUNCH_QUIET_MS
    baseline_mode="idle_refract",
    baseline_shift=6,     # EMA divisor is 1 << shift; the C uses 64
    confirm_ms=3,         # PUNCH_CONFIRM_MS
    open_on="hg",         # PUNCH_OPEN_ON_HG
    contact_end_pct=50,   # PUNCH_CONTACT_END_PCT
)

BASELINE_MODES = ("idle", "idle_refract", "always")
OPEN_MODES = ("any", "hg")


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
    # The LSM6 opens an event too. Unloaded punches peak at 4-9 g on the
    # wrist, which is the ADXL375's threshold plus its offset and noise; the
    # low-g part sees them at 4-14 g over a 1.0 g floor. Only takes effect
    # when the replay feeds low-g through set_lowg() — a raw capture carries
    # none, so replaying one scores the high-g path alone.
    lg_start_mg: int = DEFAULTS["lg_start_mg"]
    fsr_contact: int = DEFAULTS["fsr_contact"]
    window_ms: int = DEFAULTS["window_ms"]
    refract_ms: int = DEFAULTS["refract_ms"]
    quiet_ms: int = DEFAULTS["quiet_ms"]
    baseline_mode: str = DEFAULTS["baseline_mode"]
    baseline_shift: int = DEFAULTS["baseline_shift"]
    # Consecutive active samples needed to leave IDLE. 1 is the old behaviour:
    # a single sample starts an event. On a gloved, USB-powered node the FSR
    # line carries ~130 counts p-p of 50 Hz hum plus 1-2 ms spikes, and
    # hum-crest + spike clears fsr_contact for exactly one or two samples —
    # 1 event/s on a node lying on a table. A real contact lasts >= 5 ms.
    confirm_ms: int = DEFAULTS["confirm_ms"]
    # What may open an event. "any": high-g over hg_start_lsb OR an FSR rise
    # over fsr_contact (the original rule). "hg": high-g only; the FSR is
    # read inside the event for contact, never to start one.
    #
    # Every session with a hand in the glove showed why: the resting preload
    # wanders by hundreds of counts with every clench and wrist movement, so
    # the FSR opened an event, kept it active to the window cap, and the
    # baseline (frozen in ACTIVE) never caught up — one event every
    # window+refract, 0.55 s, for minutes at a time, peak 2-4 g, which is a
    # hand moving and no punch. Those are exactly the "press" events the
    # host classifies and metrics.py throws away. A thrown punch, landed or
    # not, always carries the high-g signature, so nothing a boxer wants
    # counted is lost by ignoring FSR-only rises.
    open_on: str = DEFAULTS["open_on"]
    # Contact is over once the FSR has fallen below this share of its peak
    # in the event (still bounded below by fsr_contact). 0 = the original
    # rule, contact lasts while f_rel > fsr_contact, which on a glove that
    # settles at a new preload after impact runs to the window cap
    # (measured: 250-400 ms widths on real hits; foam contact is 20-50 ms).
    contact_end_pct: int = DEFAULTS["contact_end_pct"]

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
    swing: bool = False
    peak_lg10: int = 0

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
        self._run = 0            # consecutive active samples while IDLE
        self._run_start_us = 0
        self.lg_sq = 0           # latest low-g |a|^2, mg^2
        self.peak_lg_sq = 0
        self.swing = False

    def set_lowg(self, ax_mg: int, ay_mg: int, az_mg: int) -> None:
        """punch_detect_lowg(): latest LSM6 sample, held until the next."""
        self.lg_sq = ax_mg * ax_mg + ay_mg * ay_mg + az_mg * az_mg

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
        pressed = f0_rel > p.fsr_contact or f1_rel > p.fsr_contact
        # Once contact has begun, it only counts as continuing while the FSR
        # stays above contact_end_pct of the event's peak; the glove settling
        # at a higher preload is not a fist still on the bag.
        contact_now = pressed
        if pressed and self.st == ACTIVE and self.contact and p.contact_end_pct:
            hold = max(p.fsr_contact,
                       max(self.f0_peak, self.f1_peak) * p.contact_end_pct // 100)
            contact_now = f0_rel > hold or f1_rel > hold
        swing = (mag > p.hg_start_lsb or
                 (p.lg_start_mg > 0 and
                  self.lg_sq > p.lg_start_mg * p.lg_start_mg))
        open_now = swing if p.open_on == "hg" else (swing or contact_now)
        active_now = swing or contact_now

        out = None

        if self.st == IDLE:
            if not open_now:
                self._run = 0
            else:
                if self._run == 0:
                    self._run_start_us = t_us
                self._run += 1
            # confirm_ms exists for the FSR: hum and 1-2 ms spikes clear
            # fsr_contact for a sample or two. A high-g spike has no such
            # impostor, and on the captures a real impact is 1-2 samples wide
            # above 5 g (45 g one sample, 2.4 g the next), so waiting for a
            # third would drop most of them. High-g opens on its first sample.
            need = 1 if swing else p.confirm_ms * (SAMPLE_HZ // 1000)
            if open_now and self._run >= need:
                self._run = 0
                self.peak_hg = 0
                self.f0_peak = self.f1_peak = 0
                self.impulse = 0
                self.contact = False
                self.contact_start_us = self.contact_end_us = 0
                self.peak_lg_sq = 0
                self.swing = False
                self.st = ACTIVE
                # onset is the first sample of the confirmed run, not the
                # sample that confirmed it — exec_ms is measured from here
                self.t_start_us = self._run_start_us
                self.t_last_active_us = t_us
                # fall through: the sample that opened the event is part of
                # it. Skipping it lost the whole punch when the impact spike
                # was the opener — 45 g on the capture, 10 g in the event.

        if self.st == ACTIVE:
            self.peak_hg = max(self.peak_hg, mag)
            self.peak_lg_sq = max(self.peak_lg_sq, self.lg_sq)
            if swing:
                self.swing = True
            self.f0_peak = max(self.f0_peak, f0_rel)
            self.f1_peak = max(self.f1_peak, f1_rel)
            if contact_now:
                if not self.contact:
                    self.contact = True
                    self.contact_start_us = t_us
                    self.t_contact_us = t_us
                self.contact_end_us = t_us
            if pressed:
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
                    swing=self.swing,
                    peak_lg10=min(math.isqrt(self.peak_lg_sq) // 100, 255),
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
    for name in ("hg_start_lsb", "lg_start_mg", "fsr_contact", "window_ms", "refract_ms",
                 "quiet_ms", "baseline_shift", "confirm_ms",
                 "contact_end_pct"):
        ap.add_argument(f"--{name.replace('_', '-')}", type=int,
                        default=DEFAULTS[name])
    ap.add_argument("--baseline-mode", choices=BASELINE_MODES,
                    default=DEFAULTS["baseline_mode"])
    ap.add_argument("--open-on", choices=OPEN_MODES,
                    default=DEFAULTS["open_on"])
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
