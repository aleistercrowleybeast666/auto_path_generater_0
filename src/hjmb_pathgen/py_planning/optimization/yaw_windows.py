"""Continuous full-segment yaw profiles for V4.0 leg optimization.

The public ``YawWindowProfile`` name and legacy fields are kept for project/JSON
compatibility, but new trajectories distribute the resolved yaw change uniformly
over the complete stop-to-stop arclength.  This avoids concentrating rotation in
low-speed windows and lets the combined wheel-speed limit be the only rotational
speed constraint during time parameterization.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hjmb_pathgen.py_domain.enums import YawPolicy

FULL_TURN_DDEG = 3600.0
EPSILON = 1.0e-9


@dataclass(frozen=True)
class YawSample:
    yaw_ddeg: float
    yaw_ddeg_per_mm: float
    yaw_ddeg_per_mm2: float


@dataclass(frozen=True)
class YawWindowProfile:
    start_yaw_ddeg: float
    finish_yaw_ddeg: float
    policy: YawPolicy = YawPolicy.SHORTEST
    alpha: float = 0.5
    start_window_end_s_ratio: float = 0.25
    finish_window_start_s_ratio: float = 0.75

    @property
    def resolved_delta_ddeg(self) -> float:
        return resolve_yaw_delta(self.start_yaw_ddeg, self.finish_yaw_ddeg, self.policy)

    @property
    def resolved_finish_yaw_ddeg(self) -> float:
        return self.start_yaw_ddeg + self.resolved_delta_ddeg

    def validated(self) -> "YawWindowProfile":
        if not math.isfinite(self.start_yaw_ddeg) or not math.isfinite(self.finish_yaw_ddeg):
            raise ValueError("yaw endpoints must be finite")
        if not 0.0 <= self.alpha <= 1.0:
            raise ValueError("yaw alpha must be in [0, 1]")
        if not 0.0 <= self.start_window_end_s_ratio <= self.finish_window_start_s_ratio <= 1.0:
            raise ValueError("yaw windows must satisfy 0 <= start_end <= finish_start <= 1")
        return self

    def evaluate(self, s_mm: float, total_length_mm: float) -> YawSample:
        profile = self.validated()
        if total_length_mm <= EPSILON:
            raise ValueError("yaw profile total length must be positive")

        # Uniform yaw per unit arclength is intentional.  Since translational
        # speed is zero at START/ARRIVAL, wz is also zero at both ends even
        # though d(yaw)/ds is constant.  During the leg, wz follows the same
        # single acceleration/deceleration envelope as chassis speed instead
        # of repeatedly starting and stopping inside two artificial windows.
        ratio = min(1.0, max(0.0, s_mm / total_length_mm))
        delta = profile.resolved_delta_ddeg
        yaw_per_mm = delta / total_length_mm
        return YawSample(
            yaw_ddeg=profile.start_yaw_ddeg + delta * ratio,
            yaw_ddeg_per_mm=yaw_per_mm,
            yaw_ddeg_per_mm2=0.0,
        )

    def to_dict(self, *, total_length_mm: float | None = None) -> dict[str, object]:
        data: dict[str, object] = {
            "model": "MONOTONIC_BSPLINE",
            "direction": self.policy.value,
            "degree": 1,
            "normalized_knots": [0.0, 0.0, 1.0, 1.0],
            "control_yaw_ddeg": [self.start_yaw_ddeg, self.resolved_finish_yaw_ddeg],
            "start_yaw_ddeg": self.start_yaw_ddeg,
            "finish_yaw_ddeg": self.resolved_finish_yaw_ddeg,
            "yaw_distribution": "FULL_SEGMENT_UNIFORM_ARCLENGTH",
        }
        if total_length_mm is not None:
            data["total_length_mm"] = total_length_mm
            data["yaw_ddeg_per_mm"] = self.resolved_delta_ddeg / total_length_mm if total_length_mm > EPSILON else 0.0
        return data


def resolve_yaw_delta(start_yaw_ddeg: float, finish_yaw_ddeg: float, policy: YawPolicy | str) -> float:
    policy_value = YawPolicy(str(policy))
    delta = finish_yaw_ddeg - start_yaw_ddeg
    if policy_value == YawPolicy.SHORTEST:
        return ((delta + FULL_TURN_DDEG * 0.5) % FULL_TURN_DDEG) - FULL_TURN_DDEG * 0.5
    if policy_value == YawPolicy.CW_ONLY:
        while delta > 0.0:
            delta -= FULL_TURN_DDEG
        return delta
    if policy_value == YawPolicy.CCW_ONLY:
        while delta < 0.0:
            delta += FULL_TURN_DDEG
        return delta
    raise ValueError(f"unsupported yaw policy: {policy}")
