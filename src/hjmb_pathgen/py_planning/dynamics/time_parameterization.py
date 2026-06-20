"""Robust V4.0 time parameterization for finite manual geometry.

The planner works on z = v^2 and treats high candidate speed as a reducible
envelope issue. Structural failures are reserved for malformed geometry or
physically impossible limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import StrEnum
from typing import Any, Iterable

from hjmb_pathgen.py_domain.compiled import NodeV40
from hjmb_pathgen.py_domain.enums import NodeFlag
from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.project import ProjectV40

EPSILON = 1.0e-9
DDEG_TO_RAD = math.pi / 1800.0
RAD_TO_DDEG = 1800.0 / math.pi


class SpeedFailureCategory(StrEnum):
    INVALID_INPUT = "INVALID_INPUT"
    STRUCTURAL_GEOMETRY_ERROR = "STRUCTURAL_GEOMETRY_ERROR"
    INVALID_LIMITS = "INVALID_LIMITS"
    NUMERICAL_FAILURE = "NUMERICAL_FAILURE"
    QUANTIZATION_RANGE_ERROR = "QUANTIZATION_RANGE_ERROR"
    NO_FINITE_TIME_SOLUTION = "NO_FINITE_TIME_SOLUTION"


@dataclass(frozen=True)
class GeometrySample:
    s_mm: float
    x_mm: float
    y_mm: float
    yaw_ddeg: float
    tangent_x: float
    tangent_y: float
    curvature_1_per_mm: float = 0.0
    yaw_ddeg_per_mm: float = 0.0
    yaw_ddeg_per_mm2: float = 0.0
    flags: int = 0
    arrival_state_id: str = ""
    max_speed_mmps: float | None = None

    @property
    def is_stop_boundary(self) -> bool:
        return bool(self.flags & int(NodeFlag.START | NodeFlag.ARRIVAL))

    def normalized(self) -> "GeometrySample":
        norm = math.hypot(self.tangent_x, self.tangent_y)
        if norm <= EPSILON:
            raise ValueError("sample tangent must be non-zero")
        return replace(self, tangent_x=self.tangent_x / norm, tangent_y=self.tangent_y / norm)


@dataclass(frozen=True)
class TimeParameterizationLimits:
    max_speed_mmps: float
    linear_accel_mmps2: float
    braking_accel_mmps2: float
    lateral_accel_mmps2: float
    max_wz_ddegps: float
    angular_accel_moving_ddegps2: float
    wheel_radius_mm: float
    wheel_rotation_radius_mm: float
    wheel_plan_limit_rpm: float
    hard_wheel_limit_rpm: float | None = None
    max_spacing_mm: float = 25.0
    max_yaw_step_ddeg: float = 30.0
    max_iterations: int = 24
    max_repair_iterations: int = 4
    constraint_margin_ratio: float = 0.0

    @classmethod
    def from_project(cls, project: ProjectV40, *, profile_name: str = "default") -> "TimeParameterizationLimits":
        dynamics = project.dynamics
        wheel = project.vehicle.get("wheel", {})
        planner_profiles = project.planner_profiles
        profile = planner_profiles.get(profile_name, {}) if isinstance(planner_profiles, dict) else {}
        margin = float(dynamics.get("dynamic_margin_ratio", 0.0))
        return cls(
            max_speed_mmps=float(dynamics["max_speed_mmps"]),
            linear_accel_mmps2=float(dynamics["linear_accel_mmps2"]),
            braking_accel_mmps2=float(dynamics["braking_accel_mmps2"]),
            lateral_accel_mmps2=float(dynamics["lateral_accel_mmps2"]),
            max_wz_ddegps=float(dynamics["max_wz_ddegps"]),
            angular_accel_moving_ddegps2=float(dynamics["angular_accel_moving_ddegps2"]),
            wheel_radius_mm=float(wheel["radius_mm"]),
            wheel_rotation_radius_mm=float(wheel["rotation_radius_mm"]),
            wheel_plan_limit_rpm=float(wheel["plan_limit_rpm"]),
            hard_wheel_limit_rpm=float(wheel.get("hard_limit_rpm", wheel["plan_limit_rpm"])),
            max_spacing_mm=float(profile.get("max_spacing_mm", 25.0)),
            max_yaw_step_ddeg=float(profile.get("max_yaw_step_ddeg", 30.0)),
            max_iterations=int(profile.get("max_iterations", 24)),
            max_repair_iterations=int(profile.get("max_repair_iterations", 4)),
            constraint_margin_ratio=margin,
        )

    def validated(self) -> "TimeParameterizationLimits":
        positive = {
            "max_speed_mmps": self.max_speed_mmps,
            "linear_accel_mmps2": self.linear_accel_mmps2,
            "braking_accel_mmps2": self.braking_accel_mmps2,
            "lateral_accel_mmps2": self.lateral_accel_mmps2,
            "max_wz_ddegps": self.max_wz_ddegps,
            "angular_accel_moving_ddegps2": self.angular_accel_moving_ddegps2,
            "wheel_radius_mm": self.wheel_radius_mm,
            "wheel_rotation_radius_mm": self.wheel_rotation_radius_mm,
            "wheel_plan_limit_rpm": self.wheel_plan_limit_rpm,
            "max_spacing_mm": self.max_spacing_mm,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.max_iterations <= 0 or self.max_repair_iterations < 0:
            raise ValueError("iteration limits are invalid")
        if self.constraint_margin_ratio < 0 or self.constraint_margin_ratio >= 0.9:
            raise ValueError("constraint_margin_ratio must be in [0, 0.9)")
        scale = 1.0 - self.constraint_margin_ratio
        return replace(
            self,
            max_speed_mmps=self.max_speed_mmps * scale,
            linear_accel_mmps2=self.linear_accel_mmps2 * scale,
            braking_accel_mmps2=self.braking_accel_mmps2 * scale,
            lateral_accel_mmps2=self.lateral_accel_mmps2 * scale,
            max_wz_ddegps=self.max_wz_ddegps * scale,
            angular_accel_moving_ddegps2=self.angular_accel_moving_ddegps2 * scale,
            wheel_plan_limit_rpm=self.wheel_plan_limit_rpm * scale,
        )


@dataclass(frozen=True)
class TimeParameterizationRequest:
    samples: tuple[GeometrySample, ...]
    limits: TimeParameterizationLimits
    segment_break_indices: tuple[int, ...] = ()


@dataclass(frozen=True)
class TimeSample:
    s_mm: float
    x_mm: float
    y_mm: float
    yaw_ddeg: float
    speed_mmps: float
    vx_mmps: float
    vy_mmps: float
    wz_ddegps: float
    flags: int
    arrival_state_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "s_mm": self.s_mm,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "yaw_ddeg": self.yaw_ddeg,
            "speed_mmps": self.speed_mmps,
            "vx_mmps": self.vx_mmps,
            "vy_mmps": self.vy_mmps,
            "wz_ddegps": self.wz_ddegps,
            "flags": self.flags,
            "arrival_state_id": self.arrival_state_id,
        }


@dataclass(frozen=True)
class TimeParameterizationResult:
    success: bool
    failure_category: SpeedFailureCategory | None
    reason: str
    samples: tuple[TimeSample, ...]
    nodes: tuple[NodeV40, ...]
    planned_time_ms: int
    segment_times_ms: tuple[int, ...]
    max_metrics: dict[str, float]
    limiting_constraints: dict[str, int]
    subdivision_count: int
    repair_iterations: int
    iteration_count: int
    quantization_margins: dict[str, float]
    warnings: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "failure_category": self.failure_category.value if self.failure_category else None,
            "reason": self.reason,
            "planned_time_ms": self.planned_time_ms,
            "segment_times_ms": list(self.segment_times_ms),
            "sample_count": len(self.samples),
            "node_count": len(self.nodes),
            "max_metrics": dict(self.max_metrics),
            "limiting_constraints": dict(self.limiting_constraints),
            "subdivision_count": self.subdivision_count,
            "repair_iterations": self.repair_iterations,
            "iteration_count": self.iteration_count,
            "quantization_margins": dict(self.quantization_margins),
            "warnings": list(self.warnings),
        }


def time_parameterize(request: TimeParameterizationRequest) -> TimeParameterizationResult:
    try:
        limits = request.limits.validated()
        base_samples = _validate_samples(request.samples)
        samples, subdivision_count = _adaptive_subdivide(base_samples, limits)
        z_caps, limiting_constraints = _local_z_caps(samples, limits)
        stop_indices = {index for index, sample in enumerate(samples) if sample.is_stop_boundary}
        z, iteration_count = _propagate_z(samples, z_caps, stop_indices, limits)
        for repair_iteration in range(limits.max_repair_iterations + 1):
            time_samples, planned_time_ms, segment_times_ms, max_metrics = _integrate(samples, z, request.segment_break_indices, limits)
            violation = _first_metric_violation(max_metrics, limits)
            if violation is None:
                nodes, margins = _quantize_nodes(time_samples)
                return TimeParameterizationResult(
                    success=True,
                    failure_category=None,
                    reason="OK",
                    samples=time_samples,
                    nodes=nodes,
                    planned_time_ms=planned_time_ms,
                    segment_times_ms=segment_times_ms,
                    max_metrics=max_metrics,
                    limiting_constraints=limiting_constraints,
                    subdivision_count=subdivision_count,
                    repair_iterations=repair_iteration,
                    iteration_count=iteration_count,
                    quantization_margins=margins,
                )
            z_caps = _tighten_for_violation(samples, z_caps, violation)
            z, iteration_count = _propagate_z(samples, z_caps, stop_indices, limits)
        return _failure(SpeedFailureCategory.NUMERICAL_FAILURE, f"repair did not converge: {violation}")
    except ValueError as exc:
        return _failure(_classify_value_error(str(exc)), str(exc))
    except V40ValidationError as exc:
        return _failure(SpeedFailureCategory.QUANTIZATION_RANGE_ERROR, str(exc))
    except ArithmeticError as exc:
        reason = str(exc)
        category = SpeedFailureCategory.NO_FINITE_TIME_SOLUTION if "no finite time" in reason else SpeedFailureCategory.NUMERICAL_FAILURE
        return _failure(category, reason)


def _failure(category: SpeedFailureCategory, reason: str) -> TimeParameterizationResult:
    return TimeParameterizationResult(
        success=False,
        failure_category=category,
        reason=reason,
        samples=(),
        nodes=(),
        planned_time_ms=0,
        segment_times_ms=(),
        max_metrics={},
        limiting_constraints={},
        subdivision_count=0,
        repair_iterations=0,
        iteration_count=0,
        quantization_margins={},
    )


def _classify_value_error(reason: str) -> SpeedFailureCategory:
    if "must be positive" in reason or "iteration limits" in reason or "constraint_margin_ratio" in reason:
        return SpeedFailureCategory.INVALID_LIMITS
    if "strictly increasing" in reason or "zero-length" in reason:
        return SpeedFailureCategory.STRUCTURAL_GEOMETRY_ERROR
    return SpeedFailureCategory.INVALID_INPUT


def _validate_samples(samples: tuple[GeometrySample, ...]) -> tuple[GeometrySample, ...]:
    if len(samples) < 2:
        raise ValueError("at least two geometry samples are required")
    normalized = tuple(sample.normalized() for sample in samples)
    if not (normalized[0].flags & int(NodeFlag.START)):
        raise ValueError("first sample must be START")
    if not (normalized[-1].flags & int(NodeFlag.ARRIVAL)):
        raise ValueError("last sample must be ARRIVAL")
    previous_s = normalized[0].s_mm
    if abs(previous_s) > EPSILON:
        raise ValueError("first sample s_mm must be zero")
    for index, sample in enumerate(normalized[1:], start=1):
        if not all(
            math.isfinite(value)
            for value in (
                sample.s_mm,
                sample.x_mm,
                sample.y_mm,
                sample.yaw_ddeg,
                sample.tangent_x,
                sample.tangent_y,
                sample.curvature_1_per_mm,
                sample.yaw_ddeg_per_mm,
                sample.yaw_ddeg_per_mm2,
            )
        ):
            raise ValueError(f"sample {index} contains non-finite values")
        if sample.s_mm <= previous_s + EPSILON:
            raise ValueError(f"samples must be strictly increasing at index {index}")
        previous_s = sample.s_mm
    return normalized


def _adaptive_subdivide(samples: tuple[GeometrySample, ...], limits: TimeParameterizationLimits) -> tuple[tuple[GeometrySample, ...], int]:
    result: list[GeometrySample] = [samples[0]]
    inserted = 0
    for left, right in zip(samples, samples[1:]):
        ds = right.s_mm - left.s_mm
        dyaw = abs(right.yaw_ddeg - left.yaw_ddeg)
        count = max(1, math.ceil(ds / limits.max_spacing_mm), math.ceil(dyaw / max(limits.max_yaw_step_ddeg, EPSILON)))
        for step in range(1, count):
            ratio = step / count
            flags = 0
            result.append(_interpolate_sample(left, right, ratio, flags=flags))
            inserted += 1
        result.append(right)
    return tuple(result), inserted


def _interpolate_sample(left: GeometrySample, right: GeometrySample, ratio: float, *, flags: int) -> GeometrySample:
    return GeometrySample(
        s_mm=_lerp(left.s_mm, right.s_mm, ratio),
        x_mm=_lerp(left.x_mm, right.x_mm, ratio),
        y_mm=_lerp(left.y_mm, right.y_mm, ratio),
        yaw_ddeg=_lerp(left.yaw_ddeg, right.yaw_ddeg, ratio),
        tangent_x=_lerp(left.tangent_x, right.tangent_x, ratio),
        tangent_y=_lerp(left.tangent_y, right.tangent_y, ratio),
        curvature_1_per_mm=_lerp(left.curvature_1_per_mm, right.curvature_1_per_mm, ratio),
        yaw_ddeg_per_mm=_lerp(left.yaw_ddeg_per_mm, right.yaw_ddeg_per_mm, ratio),
        yaw_ddeg_per_mm2=_lerp(left.yaw_ddeg_per_mm2, right.yaw_ddeg_per_mm2, ratio),
        flags=flags,
        max_speed_mmps=_min_optional(left.max_speed_mmps, right.max_speed_mmps),
    ).normalized()


def _local_z_caps(samples: tuple[GeometrySample, ...], limits: TimeParameterizationLimits) -> tuple[list[float], dict[str, int]]:
    z_caps: list[float] = []
    limiting: dict[str, int] = {}
    for sample in samples:
        caps: list[tuple[str, float]] = [("max_speed", limits.max_speed_mmps)]
        if sample.max_speed_mmps is not None:
            caps.append(("point_max_speed", sample.max_speed_mmps))
        if abs(sample.curvature_1_per_mm) > EPSILON:
            caps.append(("lateral_accel", math.sqrt(limits.lateral_accel_mmps2 / abs(sample.curvature_1_per_mm))))
        if abs(sample.yaw_ddeg_per_mm) > EPSILON:
            caps.append(("yaw_rate", limits.max_wz_ddegps / abs(sample.yaw_ddeg_per_mm)))
        wheel_coeff = _wheel_rpm_per_mmps(sample, limits)
        if wheel_coeff > EPSILON:
            caps.append(("wheel_rpm", limits.wheel_plan_limit_rpm / wheel_coeff))
        finite_caps = [(name, value) for name, value in caps if math.isfinite(value) and value >= 0]
        if not finite_caps:
            raise ArithmeticError("no finite local speed cap")
        limiter, speed_cap = min(finite_caps, key=lambda item: item[1])
        limiting[limiter] = limiting.get(limiter, 0) + 1
        z_caps.append(0.0 if sample.is_stop_boundary else speed_cap * speed_cap)
    z_caps[0] = 0.0
    z_caps[-1] = 0.0
    return z_caps, limiting


def _propagate_z(
    samples: tuple[GeometrySample, ...],
    z_caps: list[float],
    stop_indices: set[int],
    limits: TimeParameterizationLimits,
) -> tuple[list[float], int]:
    z = [max(0.0, cap) for cap in z_caps]
    for index in stop_indices:
        z[index] = 0.0
    iteration_count = 0
    for iteration in range(limits.max_iterations):
        iteration_count = iteration + 1
        changed = False
        for index in range(len(samples) - 1):
            ds = samples[index + 1].s_mm - samples[index].s_mm
            a_pos, _a_neg = _interval_accel_caps(samples[index], samples[index + 1], max(z[index], z[index + 1]), limits)
            reachable = z[index] + 2.0 * a_pos * ds
            new_value = min(z[index + 1], z_caps[index + 1], reachable)
            if new_value < z[index + 1] - 1.0e-6:
                z[index + 1] = max(0.0, new_value)
                changed = True
        for index in range(len(samples) - 2, -1, -1):
            ds = samples[index + 1].s_mm - samples[index].s_mm
            _a_pos, a_neg = _interval_accel_caps(samples[index], samples[index + 1], max(z[index], z[index + 1]), limits)
            controllable = z[index + 1] + 2.0 * a_neg * ds
            new_value = min(z[index], z_caps[index], controllable)
            if new_value < z[index] - 1.0e-6:
                z[index] = max(0.0, new_value)
                changed = True
        for index in stop_indices:
            if z[index] != 0.0:
                z[index] = 0.0
                changed = True
        if not changed:
            return z, iteration_count
    return z, iteration_count


def _interval_accel_caps(
    left: GeometrySample,
    right: GeometrySample,
    z_estimate: float,
    limits: TimeParameterizationLimits,
) -> tuple[float, float]:
    mid_curvature = max(abs(left.curvature_1_per_mm), abs(right.curvature_1_per_mm))
    lateral = mid_curvature * max(0.0, z_estimate)
    remaining_total = max(0.0, limits.linear_accel_mmps2 * limits.linear_accel_mmps2 - lateral * lateral)
    total_accel_cap = math.sqrt(remaining_total)
    q = max(abs(left.yaw_ddeg_per_mm), abs(right.yaw_ddeg_per_mm))
    q_prime = max(abs(left.yaw_ddeg_per_mm2), abs(right.yaw_ddeg_per_mm2))
    beta_residual = max(0.0, limits.angular_accel_moving_ddegps2 - q_prime * max(0.0, z_estimate))
    beta_accel_cap = beta_residual / q if q > EPSILON else limits.angular_accel_moving_ddegps2
    accel = min(limits.linear_accel_mmps2, total_accel_cap, beta_accel_cap)
    braking = min(limits.braking_accel_mmps2, total_accel_cap, beta_accel_cap)
    return max(0.0, accel), max(0.0, braking)


def _integrate(
    samples: tuple[GeometrySample, ...],
    z: list[float],
    segment_break_indices: tuple[int, ...],
    limits: TimeParameterizationLimits,
) -> tuple[tuple[TimeSample, ...], int, tuple[int, ...], dict[str, float]]:
    timed: list[TimeSample] = []
    max_speed = 0.0
    max_lateral = 0.0
    max_total = 0.0
    max_wz = 0.0
    max_beta = 0.0
    max_wheel = 0.0
    elapsed_ms = 0.0
    break_set = set(segment_break_indices)
    segment_times: list[int] = []
    segment_start_ms = 0.0
    for index, sample in enumerate(samples):
        speed = math.sqrt(max(0.0, z[index]))
        vx = sample.tangent_x * speed
        vy = sample.tangent_y * speed
        wz = sample.yaw_ddeg_per_mm * speed
        timed.append(
            TimeSample(
                s_mm=sample.s_mm,
                x_mm=sample.x_mm,
                y_mm=sample.y_mm,
                yaw_ddeg=sample.yaw_ddeg,
                speed_mmps=speed,
                vx_mmps=vx,
                vy_mmps=vy,
                wz_ddegps=wz,
                flags=sample.flags,
                arrival_state_id=sample.arrival_state_id,
            )
        )
        max_speed = max(max_speed, speed)
        max_wz = max(max_wz, abs(wz))
        max_wheel = max(max_wheel, _wheel_rpm(sample, speed, limits))
        if index < len(samples) - 1:
            ds = samples[index + 1].s_mm - sample.s_mm
            v_next = math.sqrt(max(0.0, z[index + 1]))
            if speed + v_next <= EPSILON:
                raise ArithmeticError(f"zero-speed interval has no finite time at index {index}")
            dt_s = 2.0 * ds / (speed + v_next)
            elapsed_ms += dt_s * 1000.0
            z_mid = 0.5 * (z[index] + z[index + 1])
            a_t = (z[index + 1] - z[index]) / (2.0 * ds)
            curvature = max(abs(sample.curvature_1_per_mm), abs(samples[index + 1].curvature_1_per_mm))
            lateral = curvature * z_mid
            q = 0.5 * (sample.yaw_ddeg_per_mm + samples[index + 1].yaw_ddeg_per_mm)
            q_prime = 0.5 * (sample.yaw_ddeg_per_mm2 + samples[index + 1].yaw_ddeg_per_mm2)
            beta = q * a_t + q_prime * z_mid
            max_lateral = max(max_lateral, abs(lateral))
            max_total = max(max_total, math.hypot(a_t, lateral))
            max_beta = max(max_beta, abs(beta))
            if index + 1 in break_set:
                segment_times.append(round(elapsed_ms - segment_start_ms))
                segment_start_ms = elapsed_ms
    if not segment_times or segment_start_ms < elapsed_ms:
        segment_times.append(round(elapsed_ms - segment_start_ms))
    metrics = {
        "max_speed_mmps": max_speed,
        "max_lateral_accel_mmps2": max_lateral,
        "max_total_accel_mmps2": max_total,
        "max_wz_ddegps": max_wz,
        "max_beta_ddegps2": max_beta,
        "max_wheel_rpm": max_wheel,
    }
    return tuple(timed), round(elapsed_ms), tuple(segment_times), metrics


def _first_metric_violation(max_metrics: dict[str, float], limits: TimeParameterizationLimits) -> str | None:
    checks = {
        "max_speed_mmps": limits.max_speed_mmps,
        "max_lateral_accel_mmps2": limits.lateral_accel_mmps2,
        "max_total_accel_mmps2": limits.linear_accel_mmps2,
        "max_wz_ddegps": limits.max_wz_ddegps,
        "max_beta_ddegps2": limits.angular_accel_moving_ddegps2,
        "max_wheel_rpm": limits.wheel_plan_limit_rpm,
    }
    for key, limit in checks.items():
        if max_metrics.get(key, 0.0) > limit + 1.0e-6:
            return key
    return None


def _tighten_for_violation(samples: tuple[GeometrySample, ...], z_caps: list[float], violation: str) -> list[float]:
    del samples
    factor = 0.90 if violation in {"max_total_accel_mmps2", "max_beta_ddegps2"} else 0.95
    return [cap * factor for cap in z_caps]


def _quantize_nodes(samples: tuple[TimeSample, ...]) -> tuple[tuple[NodeV40, ...], dict[str, float]]:
    nodes: list[NodeV40] = []
    max_position_error = 0.0
    max_velocity_error = 0.0
    arrival_id = 0
    for index, sample in enumerate(samples):
        flags = sample.flags
        if index == 0:
            flags |= int(NodeFlag.START | NodeFlag.EXACT_PASS)
        if index == len(samples) - 1:
            flags |= int(NodeFlag.EXACT_PASS)
        if flags & int(NodeFlag.ARRIVAL):
            node_arrival_id = arrival_id
            arrival_id += 1
            flags |= int(NodeFlag.EXACT_PASS)
        else:
            node_arrival_id = 0xFF
        if flags & int(NodeFlag.START | NodeFlag.ARRIVAL):
            vx = 0
            vy = 0
            wz = 0
        else:
            vx = round(sample.vx_mmps)
            vy = round(sample.vy_mmps)
            wz = round(sample.wz_ddegps)
        node = NodeV40(
            s_mm=round(sample.s_mm),
            x_mm=round(sample.x_mm),
            y_mm=round(sample.y_mm),
            yaw_ddeg=round(sample.yaw_ddeg),
            vx_mmps=vx,
            vy_mmps=vy,
            wz_ddegps=wz,
            arrival_id=node_arrival_id,
            flags=flags,
        )
        node = NodeV40.from_tuple(node.to_tuple())
        nodes.append(node)
        max_position_error = max(max_position_error, abs(node.x_mm - sample.x_mm), abs(node.y_mm - sample.y_mm), abs(node.s_mm - sample.s_mm))
        max_velocity_error = max(max_velocity_error, abs(node.vx_mmps - sample.vx_mmps), abs(node.vy_mmps - sample.vy_mmps), abs(node.wz_ddegps - sample.wz_ddegps))
    if arrival_id == 0:
        raise V40ValidationError("TimeParameterization", "nodes", "at least one ARRIVAL node is required")
    return tuple(nodes), {"max_position_error": max_position_error, "max_velocity_error": max_velocity_error}


def _wheel_rpm(sample: GeometrySample, speed: float, limits: TimeParameterizationLimits) -> float:
    return _wheel_rpm_per_mmps(sample, limits) * speed


def _wheel_rpm_per_mmps(sample: GeometrySample, limits: TimeParameterizationLimits) -> float:
    yaw_rad = sample.yaw_ddeg * DDEG_TO_RAD
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    vx_body_coeff = cos_yaw * sample.tangent_x + sin_yaw * sample.tangent_y
    vy_body_coeff = -sin_yaw * sample.tangent_x + cos_yaw * sample.tangent_y
    wz_rad_coeff = sample.yaw_ddeg_per_mm * DDEG_TO_RAD
    rotation_coeff = limits.wheel_rotation_radius_mm * wz_rad_coeff
    wheel_linear_coeffs = (
        vx_body_coeff - vy_body_coeff - rotation_coeff,
        vx_body_coeff + vy_body_coeff + rotation_coeff,
        vx_body_coeff + vy_body_coeff - rotation_coeff,
        vx_body_coeff - vy_body_coeff + rotation_coeff,
    )
    rpm_per_mmps = 60.0 / (2.0 * math.pi * limits.wheel_radius_mm)
    return max(abs(value) for value in wheel_linear_coeffs) * rpm_per_mmps


def samples_from_points(points: Iterable[tuple[float, float, float]], *, arrival_flags: Iterable[int] | None = None) -> tuple[GeometrySample, ...]:
    point_list = list(points)
    if len(point_list) < 2:
        raise ValueError("at least two points are required")
    arrival_set = set(arrival_flags or ())
    cumulative = [0.0]
    for left, right in zip(point_list, point_list[1:]):
        cumulative.append(cumulative[-1] + math.hypot(right[0] - left[0], right[1] - left[1]))
    result: list[GeometrySample] = []
    total = cumulative[-1]
    for index, (x_mm, y_mm, yaw_ddeg) in enumerate(point_list):
        if index < len(point_list) - 1:
            dx = point_list[index + 1][0] - x_mm
            dy = point_list[index + 1][1] - y_mm
        else:
            dx = x_mm - point_list[index - 1][0]
            dy = y_mm - point_list[index - 1][1]
        distance = max(math.hypot(dx, dy), EPSILON)
        if index == 0:
            flags = int(NodeFlag.START)
        elif index == len(point_list) - 1 or index in arrival_set:
            flags = int(NodeFlag.ARRIVAL)
        else:
            flags = 0
        yaw_rate = 0.0
        if total > EPSILON and index < len(point_list) - 1:
            yaw_rate = (point_list[-1][2] - point_list[0][2]) / total
        result.append(
            GeometrySample(
                s_mm=cumulative[index],
                x_mm=x_mm,
                y_mm=y_mm,
                yaw_ddeg=yaw_ddeg,
                tangent_x=dx / distance,
                tangent_y=dy / distance,
                yaw_ddeg_per_mm=yaw_rate,
                flags=flags,
            )
        )
    return tuple(result)


def _lerp(left: float, right: float, ratio: float) -> float:
    return left + (right - left) * ratio


def _min_optional(left: float | None, right: float | None) -> float | None:
    if left is None:
        return right
    if right is None:
        return left
    return min(left, right)
