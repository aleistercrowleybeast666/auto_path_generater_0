"""Two-low-speed-window yaw profiles for Phase 6 leg optimization."""

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
        ratio = min(1.0, max(0.0, s_mm / total_length_mm))
        delta = profile.resolved_delta_ddeg
        first_delta = delta * profile.alpha
        second_delta = delta - first_delta
        start_end = profile.start_window_end_s_ratio
        finish_start = profile.finish_window_start_s_ratio
        if ratio <= start_end and start_end > EPSILON:
            smooth = _smoothstep5(ratio / start_end)
            ds_ratio = 1.0 / (start_end * total_length_mm)
            return YawSample(
                yaw_ddeg=profile.start_yaw_ddeg + first_delta * smooth.value,
                yaw_ddeg_per_mm=first_delta * smooth.first * ds_ratio,
                yaw_ddeg_per_mm2=first_delta * smooth.second * ds_ratio * ds_ratio,
            )
        if ratio < finish_start:
            return YawSample(yaw_ddeg=profile.start_yaw_ddeg + first_delta, yaw_ddeg_per_mm=0.0, yaw_ddeg_per_mm2=0.0)
        finish_span = 1.0 - finish_start
        if finish_span <= EPSILON:
            return YawSample(yaw_ddeg=profile.resolved_finish_yaw_ddeg, yaw_ddeg_per_mm=0.0, yaw_ddeg_per_mm2=0.0)
        u = (ratio - finish_start) / finish_span
        smooth = _smoothstep5(u)
        ds_ratio = 1.0 / (finish_span * total_length_mm)
        return YawSample(
            yaw_ddeg=profile.start_yaw_ddeg + first_delta + second_delta * smooth.value,
            yaw_ddeg_per_mm=second_delta * smooth.first * ds_ratio,
            yaw_ddeg_per_mm2=second_delta * smooth.second * ds_ratio * ds_ratio,
        )

    def to_dict(self, *, total_length_mm: float | None = None) -> dict[str, float | str]:
        data: dict[str, float | str] = {
            "model": "TWO_LOW_SPEED_WINDOWS",
            "direction": self.policy.value,
            "alpha": self.alpha,
            "start_window_end_s_ratio": self.start_window_end_s_ratio,
            "finish_window_start_s_ratio": self.finish_window_start_s_ratio,
            "start_yaw_ddeg": self.start_yaw_ddeg,
            "finish_yaw_ddeg": self.resolved_finish_yaw_ddeg,
        }
        if total_length_mm is not None:
            data["start_window_end_s_mm"] = total_length_mm * self.start_window_end_s_ratio
            data["finish_window_start_s_mm"] = total_length_mm * self.finish_window_start_s_ratio
        return data


@dataclass(frozen=True)
class _SmoothStep:
    value: float
    first: float
    second: float


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


def _smoothstep5(u: float) -> _SmoothStep:
    clamped = min(1.0, max(0.0, u))
    value = clamped**3 * (10.0 - 15.0 * clamped + 6.0 * clamped * clamped)
    first = 30.0 * clamped * clamped * (clamped - 1.0) * (clamped - 1.0)
    second = 60.0 * clamped * (2.0 * clamped * clamped - 3.0 * clamped + 1.0)
    return _SmoothStep(value=value, first=first, second=second)
