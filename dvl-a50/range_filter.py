"""Persistence on selected clearance; update only for fresh sensor frames."""

import math
from typing import Optional

# Provisional replay defaults, metres and seconds. See docs/range-smoothing.md.
RANGE_SWITCH_HYSTERESIS_M = 0.10
RANGE_CANDIDATE_TOLERANCE_M = 0.05
RANGE_DEEPER_PERSISTENCE_S = 0.5
RANGE_DEEPER_TAU_S = 1.0
RANGE_NOISE_TAU_S = 0.25
RANGE_CONTINUITY_GAP_S = 0.4


class RangePersistenceFilter:
    """Fixed candidate band prevents wandering returns from earning confirmation."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Forget the estimate and all confirmation across a discontinuity."""
        self.accepted: Optional[float] = None
        self.timestamp: Optional[float] = None
        self.candidate: Optional[float] = None
        self.candidate_since = 0.0
        self.confirmed = False

    def update(self, distance: Optional[float], timestamp: float) -> Optional[float]:
        """No output on invalid/duplicate/rewound input; no timer runs without input.

        Small errors use a symmetric LPF (including downward noise). Only a
        decrease beyond hysteresis snaps to the selected measurement. A fixed
        confirmed target remains eligible until the input leaves its band.
        """
        if distance is None or not math.isfinite(distance) or distance <= 0 or not math.isfinite(timestamp):
            self.reset()
            return None
        if self.timestamp is not None and timestamp <= self.timestamp:
            self.reset()
            return None
        if self.timestamp is not None and timestamp - self.timestamp > RANGE_CONTINUITY_GAP_S:
            self.reset()
        if self.accepted is None:
            self.accepted = distance
            self.timestamp = timestamp
            return distance

        dt = timestamp - self.timestamp
        self.timestamp = timestamp
        error = distance - self.accepted
        if error < -RANGE_SWITCH_HYSTERESIS_M:
            self.accepted = distance
            self.candidate = None
            self.confirmed = False
        elif self.confirmed and abs(distance - self.candidate) <= RANGE_CANDIDATE_TOLERANCE_M:
            self.accepted += -math.expm1(-dt / RANGE_DEEPER_TAU_S) * error
        elif abs(error) <= RANGE_SWITCH_HYSTERESIS_M:
            self.candidate = None
            self.confirmed = False
            self.accepted += -math.expm1(-dt / RANGE_NOISE_TAU_S) * error
        else:
            if self.candidate is None or abs(distance - self.candidate) > RANGE_CANDIDATE_TOLERANCE_M:
                self.candidate = distance
                self.candidate_since = timestamp
                self.confirmed = False
            if timestamp - self.candidate_since >= RANGE_DEEPER_PERSISTENCE_S - 1e-9:
                self.confirmed = True
            if self.confirmed:
                self.accepted += -math.expm1(-dt / RANGE_DEEPER_TAU_S) * error
        return self.accepted
