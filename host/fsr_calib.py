"""
FSR counts -> newtons.

The divider is not linear in force, but nothing is lost by that: counts map to
a voltage by fixed arithmetic, the voltage maps to R_fsr by the divider
equation, and the FlexiForce's *conductance* is what is linear in force. So the
whole calibration is a single constant k in

    G = 1 / R_fsr = k * F

Everything before G is exact algebra with no fitted terms.

The constant is written by calibrate_fsr.py into fsr_calib.json next to this
file. Without that file force_n stays None and the rest of the pipeline is
unaffected.
"""

import json
from pathlib import Path

# --- fixed by the overlay and the board ---------------------------------
#
# 12-bit SAADC, ADC_GAIN_1_6 with the 0.6 V internal reference -> a 3.6 V
# window over 4095 counts.
ADC_FULL_SCALE_MV = 3600.0
ADC_MAX_COUNTS = 4095.0
VCC_MV = 3300.0

CALIB_PATH = Path(__file__).parent / "fsr_calib.json"


def counts_to_mv(counts: float) -> float:
    return counts * ADC_FULL_SCALE_MV / ADC_MAX_COUNTS


def counts_to_ohms(counts: float, r_fixed: float) -> float:
    """Divider is 3V3 - FSR - node - r_fixed - GND, node read by the ADC.

    Returns inf at zero counts (open sensor) rather than dividing by zero.
    """
    mv = counts_to_mv(counts)
    if mv <= 0.0:
        return float("inf")
    if mv >= VCC_MV:
        return 0.0
    return r_fixed * (VCC_MV - mv) / mv


class FsrCalibration:
    """Loaded from fsr_calib.json; .newtons() returns None when uncalibrated."""

    def __init__(self, data: dict | None):
        self.data = data or {}
        self.k = self.data.get("k_siemens_per_newton")
        self.r_fixed = self.data.get("r_fixed_ohms", 47000.0)
        # Resting counts at zero load when the calibration was taken. Event
        # packets carry f0_peak *relative to the node's own EMA baseline*, so
        # the absolute reading has to be reconstructed before the divider maths
        # applies.
        self.baseline = self.data.get("baseline_counts", 0.0)

    @property
    def valid(self) -> bool:
        return bool(self.k)

    def newtons(self, counts_rel: float) -> float | None:
        """counts_rel: baseline-relative ADC counts, as carried in an event."""
        if not self.valid or counts_rel is None or counts_rel <= 0:
            return None
        ohms = counts_to_ohms(counts_rel + self.baseline, self.r_fixed)
        if ohms == float("inf") or ohms <= 0.0:
            return None
        return (1.0 / ohms) / self.k


def load(path: Path = CALIB_PATH) -> FsrCalibration:
    try:
        return FsrCalibration(json.loads(path.read_text(encoding="utf-8")))
    except FileNotFoundError:
        return FsrCalibration(None)
    except (OSError, ValueError) as exc:
        print(f"[fsr_calib] ignoring {path.name}: {exc}")
        return FsrCalibration(None)
