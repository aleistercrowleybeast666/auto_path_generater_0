"""Deterministic V4 path planning used by MANUAL and assisted paths.

The XY geometry is never searched or moved.  User points are preserved and only
non-exact WAYPOINT corners are rounded with the mature local quadratic-Bezier
constructor from the field editor.  Yaw is then distributed into two low-speed
windows around every START/ARRIVAL stop and several deterministic window choices
are retimed; the fastest feasible result is selected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Callable, Iterable

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32
from hjmb_pathgen.py_domain.compiled import ActionV40, CompiledTrajectoryV40, HeaderV40, SegmentV40
from hjmb_pathgen.py_domain.enums import (
    ActionCode,
    ActionMode,
    GenerationMode,
    HeaderFlag,
    ManualPathPointType,
    NodeFlag,
    RouteFamily,
    SegmentFlag,
    YawPolicy,
)
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.manual_path import ManualPathV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.protocol import REQUIRED_HEADER_FLAGS
from hjmb_pathgen.py_planning.dynamics.time_parameterization import (
    GeometrySample,
    TimeParameterizationLimits,
    TimeParameterizationRequest,
    TimeParameterizationResult,
    time_parameterize,
)
from hjmb_pathgen.py_planning.optimization.yaw_windows import YawWindowProfile
from hjmb_pathgen.py_ui.v35_base.path_geometry import generate_geometry
from hjmb_pathgen.py_ui.v35_base.path_models import (
    EditPoint,
    PlannerConfig,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    SITE_ID_FREE,
    YAW_UNSPECIFIED_DDEG,
)

_EPSILON = 1.0e-9


@dataclass(frozen=True)
class ManualCasePlanResult:
    case: CaseManifestV40
    trajectory: CompiledTrajectoryV40 | None
    timing: TimeParameterizationResult

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.case.traj_id,
            "success": self.timing.success,
            "timing": self.timing.to_dict(),
            "node_count": len(self.trajectory.nodes) if self.trajectory else 0,
        }


def build_manual_spatial_path(
    manual_path: dict[str, Any] | ManualPathV40,
    project: ProjectV40 | None = None,
    *,
    profile_name: str = "default",
    yaw_policy: YawPolicy | str = YawPolicy.SHORTEST,
    window_ratio: float = 0.30,
    alpha_bias: float = 0.0,
) -> tuple[GeometrySample, ...]:
    """Build a dense deterministic path without moving the user's sparse points.

    ``yaw_ddeg == 0xFFFF`` is kept in JSON but is treated as an unconstrained yaw
    anchor here.  The resolved BIN yaw simply continues from the preceding stop,
    which is the minimum-time choice unless a later explicit yaw requires motion.
    """

    path = manual_path if isinstance(manual_path, ManualPathV40) else ManualPathV40.from_dict(manual_path)
    planner = _geometry_planner(project, profile_name)
    try:
        raw_samples, point_s_mm = _piecewise_stop_geometry(path, planner)
    except ValueError as exc:
        raise CompileError(str(exc)) from exc
    return _geometry_samples_with_yaw(
        path,
        raw_samples,
        point_s_mm,
        yaw_policy=YawPolicy(str(yaw_policy)),
        window_ratio=window_ratio,
        alpha_bias=alpha_bias,
    )


def retime_path(
    manual_path: dict[str, Any] | ManualPathV40,
    project: ProjectV40,
    *,
    profile_name: str = "default",
    yaw_policy: YawPolicy | str = YawPolicy.SHORTEST,
    cancel_check: Callable[[], bool] | None = None,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
) -> TimeParameterizationResult:
    """Try deterministic yaw-window variants and return the fastest feasible one."""

    limits = TimeParameterizationLimits.from_project(project, profile_name=profile_name)
    successes: list[tuple[int, float, float, TimeParameterizationResult]] = []
    failures: list[TimeParameterizationResult] = []
    # The candidate set is deliberately small and reproducible.  It covers a short,
    # medium and long low-speed window plus three rotation allocations.
    candidates = tuple(
        (window_ratio, alpha_bias)
        for window_ratio in (0.20, 0.30, 0.40)
        for alpha_bias in (-0.20, 0.0, 0.20)
    )
    for candidate_index, (window_ratio, alpha_bias) in enumerate(candidates, start=1):
        if cancel_check is not None and cancel_check():
            raise RuntimeError("CANCELLED")
        if progress_callback is not None:
            progress_callback(
                {
                    "stage": "SEMI_YAW_OPTIMIZATION",
                    "message": f"比较低速旋转分配 {candidate_index}/{len(candidates)}",
                    "percent": 12 + round(70 * (candidate_index - 1) / len(candidates)),
                    "completed_count": candidate_index - 1,
                    "total_count": len(candidates),
                    "candidate": candidate_index,
                }
            )
        samples = build_manual_spatial_path(
            manual_path,
            project,
            profile_name=profile_name,
            yaw_policy=yaw_policy,
            window_ratio=window_ratio,
            alpha_bias=alpha_bias,
        )
        timing = time_parameterize(TimeParameterizationRequest(samples=samples, limits=limits))
        if timing.success:
            successes.append((timing.planned_time_ms, window_ratio, alpha_bias, timing))
        else:
            failures.append(timing)
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_YAW_OPTIMIZATION",
                "message": "低速旋转分配比较完成",
                "percent": 82,
                "completed_count": len(candidates),
                "total_count": len(candidates),
            }
        )
    if not successes:
        if failures:
            return failures[0]
        raise CompileError("deterministic path planner produced no yaw candidate")
    _, window_ratio, alpha_bias, best = min(successes, key=lambda item: (item[0], item[1], abs(item[2])))
    return replace(
        best,
        warnings=tuple(best.warnings)
        + (f"selected low-speed yaw windows: ratio={window_ratio:.2f}, alpha_bias={alpha_bias:+.2f}",),
    )


def retime_case(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    profile_name: str = "default",
) -> TimeParameterizationResult:
    if case.generation_mode != GenerationMode.MANUAL:
        raise CompileError("retime_case supports MANUAL cases; use retime_path for assisted paths")
    if case.manual_path is None:
        raise CompileError("MANUAL case has no manual_path")
    return retime_path(
        case.manual_path,
        project,
        profile_name=profile_name,
        yaw_policy=_case_yaw_policy(case),
    )


def plan_manual_case(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    profile_name: str = "default",
) -> ManualCasePlanResult:
    timing = retime_case(case, project, profile_name=profile_name)
    trajectory = (
        trajectory_from_deterministic_timing(
            case,
            project,
            timing,
            route_family=RouteFamily.MANUAL,
            segment_flags=int(SegmentFlag.NORMAL | SegmentFlag.MANUAL_OVERRIDE),
            header_manual_override=True,
        )
        if timing.success
        else None
    )
    return ManualCasePlanResult(case=case, trajectory=trajectory, timing=timing)


def trajectory_from_deterministic_timing(
    case: CaseManifestV40,
    project: ProjectV40,
    timing: TimeParameterizationResult,
    *,
    route_family: RouteFamily,
    segment_flags: int,
    header_manual_override: bool,
) -> CompiledTrajectoryV40:
    """Compile a retimed deterministic path into V4 nodes and arrival segments."""

    if not timing.nodes:
        raise CompileError("successful timing result did not produce nodes")
    nodes = list(timing.nodes)
    if not (nodes[-1].flags & int(NodeFlag.ARRIVAL)):
        raise CompileError("deterministic path must end at ARRIVAL")
    nodes[-1] = replace(
        nodes[-1],
        flags=(nodes[-1].flags & ~int(NodeFlag.SAFE_END))
        | int(NodeFlag.FINISH_ARM | NodeFlag.EXACT_PASS | NodeFlag.ARRIVAL),
        vx_mmps=0,
        vy_mmps=0,
        wz_ddegps=0,
    )
    segments = _segments_from_nodes(nodes, timing, segment_flags=segment_flags)
    flags = REQUIRED_HEADER_FLAGS
    if header_manual_override:
        flags |= HeaderFlag.MANUAL_OVERRIDE
    mechanism_ms = sum(
        int(item.get("estimated_time_ms", 0))
        for item in case.actions.get("source", ())
        if isinstance(item, dict)
    )
    header = HeaderV40(
        traj_id=case.traj_id,
        bean_code=case.bean_code,
        drop_code=case.drop_code,
        route_family=int(route_family),
        flags=int(flags),
        planned_motion_time_ms=timing.planned_time_ms,
        planned_total_estimate_ms=timing.planned_time_ms + mechanism_ms,
        source_case_hash32=canonical_json_crc32(case.to_dict()),
        source_project_hash32=canonical_json_crc32(project.to_dict()),
    )
    actions = compile_point_bound_actions(case, nodes)
    trajectory = CompiledTrajectoryV40(
        header=header,
        nodes=tuple(nodes),
        segments=segments,
        actions=actions,
    ).normalized()
    trajectory.validate()
    return trajectory


def compile_point_bound_actions(case: CaseManifestV40, nodes: list) -> tuple[ActionV40, ...]:
    """Compile MANUAL/SEMI actions whose STOP bindings use editor point indices."""

    source = list(case.actions.get("source") or case.actions.get("compiled") or ())
    if not source:
        return ()
    path_data = case.manual_path or case.semi_path or {}
    arrival_by_point_index: dict[int, int] = {}
    arrival_s: dict[int, int] = {}
    arrival_id = 0
    for point_index, point in enumerate(path_data.get("points", [])):
        if str(point.get("type")) == ManualPathPointType.ARRIVAL.value:
            arrival_by_point_index[point_index] = arrival_id
            arrival_id += 1
    for node in nodes:
        if int(node.arrival_id) != 0xFF:
            arrival_s[int(node.arrival_id)] = int(node.s_mm)

    result: list[ActionV40] = []
    previous_stop_s = 0
    for index, item in enumerate(source):
        mode = _enum_number(ActionMode, item.get("mode", ActionMode.STOP_AND_WAIT))
        bound_arrival = 0xFF
        if mode == int(ActionMode.STOP_AND_WAIT):
            raw_arrival = item.get("arrival_id")
            if raw_arrival is None:
                raw_arrival = item.get("arrival_point_index", item.get("arrival_point_id"))
                if raw_arrival is not None:
                    raw_arrival = arrival_by_point_index.get(int(raw_arrival))
            if raw_arrival is None and item.get("arrival_state_id") is not None:
                state_id = str(item["arrival_state_id"])
                # Deterministic path states may explicitly carry their editor row.
                for point_index, point in enumerate(path_data.get("points", [])):
                    if str(point.get("state_id", "")) == state_id:
                        raw_arrival = arrival_by_point_index.get(point_index)
                        break
                if raw_arrival is None and state_id.startswith("MANUAL_ARRIVAL_"):
                    raw_arrival = arrival_by_point_index.get(int(state_id.rsplit("_", 1)[1]))
            if raw_arrival is None:
                raise CompileError(f"action {index} STOP_AND_WAIT has no valid ARRIVAL binding")
            bound_arrival = int(raw_arrival)
            previous_stop_s = arrival_s.get(bound_arrival, previous_stop_s)
        check_start = 0xFFFF
        if mode == int(ActionMode.KINEMATIC):
            # The full KINEMATIC scan is performed by the referenced-leg compiler.
            # For a deterministic user path the preceding stop is the safe lower bound.
            check_start = previous_stop_s
        result.append(
            ActionV40(
                action_seq=index,
                action=_enum_number(ActionCode, item.get("action", ActionCode.NONE)),
                mode=mode,
                arrival_id=bound_arrival,
                timeout_ms=int(item.get("timeout_ms", 1000)),
                post_wait_ms=int(item.get("post_wait_ms", 0)),
                check_start_s_mm=check_start,
                accel_limit_mmps2=int(item.get("accel_limit_mmps2", 0)),
                beta_limit_ddegps2=int(item.get("beta_limit_ddegps2", 0)),
                wz_limit_ddegps=int(item.get("wz_limit_ddegps", 0)),
                speed_limit_mmps=int(item.get("speed_limit_mmps", 0)),
                stable_time_ms=int(item.get("stable_time_ms", 0)),
            )
        )
    return tuple(result)


def _geometry_planner(project: ProjectV40 | None, profile_name: str) -> PlannerConfig:
    profile: dict[str, Any] = {}
    max_speed = 2000
    linear_accel = 1200
    lateral_accel = 1200
    max_wz = 4.0
    beta_moving = 2.0
    beta_rotate = 5.0
    if project is not None:
        profiles = project.planner_profiles if isinstance(project.planner_profiles, dict) else {}
        profile = dict(profiles.get(profile_name, profiles.get(profile_name.upper(), profiles.get("default", {}))))
        dynamics = project.dynamics
        max_speed = int(dynamics.get("max_speed_mmps", max_speed))
        linear_accel = int(dynamics.get("linear_accel_mmps2", linear_accel))
        lateral_accel = int(dynamics.get("lateral_accel_mmps2", lateral_accel))
        max_wz = math.radians(float(dynamics.get("max_wz_ddegps", 2292)) / 10.0)
        beta_moving = math.radians(float(dynamics.get("angular_accel_moving_ddegps2", 1146)) / 10.0)
        beta_rotate = math.radians(float(dynamics.get("angular_accel_rotate_ddegps2", 2865)) / 10.0)
    max_spacing = max(1, min(50, int(profile.get("max_spacing_mm", 25))))
    nominal_spacing = max(1, min(max_spacing, int(profile.get("nominal_spacing_mm", max_spacing))))
    return PlannerConfig(
        max_speed_mmps=max_speed,
        nominal_spacing_mm=nominal_spacing,
        max_spacing_mm=max_spacing,
        linear_accel_mmps2=linear_accel,
        lateral_accel_mmps2=lateral_accel,
        max_wz_radps=max_wz,
        angular_accel_moving_radps2=beta_moving,
        angular_accel_rotate_radps2=beta_rotate,
    )


def _piecewise_stop_geometry(
    path: ManualPathV40,
    planner: PlannerConfig,
) -> tuple[tuple[object, ...], dict[int, float]]:
    """Generate each START/ARRIVAL interval independently.

    A full-path central-difference curvature calculation incorrectly looks
    through an ARRIVAL into the following leg.  At a sharp stop this creates a
    huge fictitious endpoint curvature and can force the speed envelope to
    zero for a finite-length interval.  Independent leg geometry keeps local
    Bezier rounding while making every stop a clean derivative boundary.
    """

    edit_points = _edit_points(path)
    boundary_indices = [
        index
        for index, point in enumerate(path.points)
        if point.point_type in (ManualPathPointType.START, ManualPathPointType.ARRIVAL)
    ]
    samples: list[object] = []
    point_s_mm: dict[int, float] = {}
    global_offset = 0.0
    for segment_number, (start_index, end_index) in enumerate(zip(boundary_indices, boundary_indices[1:])):
        local_points: list[EditPoint] = []
        for global_index in range(start_index, end_index + 1):
            original = edit_points[global_index]
            local_type = original.type
            if global_index == start_index:
                local_type = POINT_TYPE_START
            elif global_index == end_index:
                local_type = POINT_TYPE_ARRIVAL
            local_points.append(
                replace(
                    original,
                    point_id=global_index - start_index,
                    type=local_type,
                    exact_pass=(True if global_index in (start_index, end_index) else original.exact_pass),
                    corner_trim_mm=(0.0 if global_index in (start_index, end_index) else original.corner_trim_mm),
                )
            )
        geometry = generate_geometry(local_points, planner)
        local_total = float(geometry.samples[-1].s_mm)
        for local_point_index, local_s in geometry.point_s_mm.items():
            point_s_mm[start_index + int(local_point_index)] = global_offset + float(local_s)
        for local_sample_index, sample in enumerate(geometry.samples):
            if segment_number > 0 and local_sample_index == 0:
                continue
            source_point = getattr(sample, "source_point", None)
            source_segment = int(getattr(sample, "source_segment", 0)) + start_index
            mapped_source = None if source_point is None else int(source_point) + start_index
            curvature = float(getattr(sample, "curvature_kappa_per_m", 0.0))
            if local_sample_index == 0 or local_sample_index == len(geometry.samples) - 1:
                curvature = 0.0
            samples.append(
                replace(
                    sample,
                    s_mm=global_offset + float(sample.s_mm),
                    source_point=mapped_source,
                    source_segment=source_segment,
                    curvature_kappa_per_m=curvature,
                )
            )
        global_offset += local_total
    if len(samples) < 2:
        raise CompileError("deterministic path produced fewer than two geometry samples")
    return tuple(samples), point_s_mm


def _edit_points(path: ManualPathV40) -> list[EditPoint]:
    result: list[EditPoint] = []
    for index, point in enumerate(path.points):
        if point.point_type == ManualPathPointType.START:
            ptype = POINT_TYPE_START
        elif point.point_type == ManualPathPointType.ARRIVAL:
            ptype = POINT_TYPE_ARRIVAL
        else:
            ptype = POINT_TYPE_WAYPOINT
        result.append(
            EditPoint(
                point_id=index,
                type=ptype,
                site_id=SITE_ID_FREE,
                x_mm=float(point.x_mm),
                y_mm=float(point.y_mm),
                yaw_ddeg=(YAW_UNSPECIFIED_DDEG if point.yaw_ddeg is None else int(point.yaw_ddeg)),
                max_speed_mmps=int(point.max_speed_mmps or 0),
                corner_trim_mm=float(point.corner_trim_mm),
                exact_pass=bool(point.exact_pass),
            )
        )
    return result


def _geometry_samples_with_yaw(
    path: ManualPathV40,
    raw_samples: Iterable[object],
    point_s_mm: dict[int, float],
    *,
    yaw_policy: YawPolicy,
    window_ratio: float,
    alpha_bias: float,
) -> tuple[GeometrySample, ...]:
    samples = tuple(raw_samples)
    boundary_indices = [
        index
        for index, point in enumerate(path.points)
        if point.point_type in (ManualPathPointType.START, ManualPathPointType.ARRIVAL)
    ]
    boundary_s = [float(point_s_mm[index]) for index in boundary_indices]
    boundary_yaw = _resolved_boundary_yaws(path, boundary_indices, yaw_policy)
    segment_profiles: list[tuple[float, float, YawWindowProfile]] = []
    for segment_index, (left_s, right_s) in enumerate(zip(boundary_s, boundary_s[1:])):
        length = right_s - left_s
        if length <= _EPSILON:
            raise CompileError("manual_path contains a zero-length stop interval")
        alpha = _curvature_weighted_alpha(samples, left_s, right_s, window_ratio)
        alpha = min(0.95, max(0.05, alpha + alpha_bias))
        profile = YawWindowProfile(
            start_yaw_ddeg=boundary_yaw[segment_index],
            finish_yaw_ddeg=boundary_yaw[segment_index + 1],
            policy=yaw_policy,
            alpha=alpha,
            start_window_end_s_ratio=window_ratio,
            finish_window_start_s_ratio=1.0 - window_ratio,
        )
        segment_profiles.append((left_s, right_s, profile))

    result: list[GeometrySample] = []
    segment_index = 0
    for sample in samples:
        while segment_index + 1 < len(segment_profiles) and float(sample.s_mm) > segment_profiles[segment_index][1] + 1.0e-7:
            segment_index += 1
        left_s, right_s, profile = segment_profiles[min(segment_index, len(segment_profiles) - 1)]
        local_s = min(right_s - left_s, max(0.0, float(sample.s_mm) - left_s))
        yaw = profile.evaluate(local_s, right_s - left_s)
        source_point = getattr(sample, "source_point", None)
        flags = 0
        arrival_state_id = ""
        if source_point is not None:
            point = path.points[int(source_point)]
            if point.point_type == ManualPathPointType.START:
                flags |= int(NodeFlag.START | NodeFlag.EXACT_PASS)
            elif point.point_type == ManualPathPointType.ARRIVAL:
                flags |= int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS)
                arrival_state_id = str((point.hints or {}).get("state_id", f"MANUAL_ARRIVAL_{source_point}"))
            elif point.exact_pass:
                flags |= int(NodeFlag.EXACT_PASS)
        max_speed = _sample_speed_hint(path, point_s_mm, float(sample.s_mm))
        result.append(
            GeometrySample(
                s_mm=float(sample.s_mm),
                x_mm=float(sample.x_mm),
                y_mm=float(sample.y_mm),
                yaw_ddeg=yaw.yaw_ddeg,
                tangent_x=float(sample.tangent_x),
                tangent_y=float(sample.tangent_y),
                curvature_1_per_mm=float(sample.curvature_kappa_per_m) / 1000.0,
                yaw_ddeg_per_mm=yaw.yaw_ddeg_per_mm,
                yaw_ddeg_per_mm2=yaw.yaw_ddeg_per_mm2,
                flags=flags,
                arrival_state_id=arrival_state_id,
                max_speed_mmps=max_speed,
            )
        )
    if not result or not (result[0].flags & int(NodeFlag.START)):
        raise CompileError("deterministic geometry lost START")
    if not (result[-1].flags & int(NodeFlag.ARRIVAL)):
        raise CompileError("deterministic geometry lost final ARRIVAL")
    return tuple(result)


def _resolved_boundary_yaws(path: ManualPathV40, boundary_indices: list[int], policy: YawPolicy) -> list[float]:
    explicit = [
        int(path.points[index].yaw_ddeg)
        for index in boundary_indices
        if path.points[index].yaw_ddeg is not None and int(path.points[index].yaw_ddeg) != YAW_UNSPECIFIED_DDEG
    ]
    current = float(explicit[0] if explicit else 0)
    resolved: list[float] = []
    for index in boundary_indices:
        raw = path.points[index].yaw_ddeg
        if raw is not None and int(raw) != YAW_UNSPECIFIED_DDEG:
            target = float(raw)
            probe = YawWindowProfile(current, target, policy=policy)
            current = probe.resolved_finish_yaw_ddeg
        # Unspecified means no orientation requirement: keep current yaw.
        resolved.append(current)
    return resolved


def _curvature_weighted_alpha(samples: tuple[object, ...], left_s: float, right_s: float, window_ratio: float) -> float:
    length = right_s - left_s
    window = max(1.0, length * window_ratio)
    start_values = [
        abs(float(getattr(sample, "curvature_kappa_per_m", 0.0)))
        for sample in samples
        if left_s - 1.0e-7 <= float(sample.s_mm) <= left_s + window + 1.0e-7
    ]
    finish_values = [
        abs(float(getattr(sample, "curvature_kappa_per_m", 0.0)))
        for sample in samples
        if right_s - window - 1.0e-7 <= float(sample.s_mm) <= right_s + 1.0e-7
    ]
    start_slow = 1.0 + (sum(start_values) / max(len(start_values), 1))
    finish_slow = 1.0 + (sum(finish_values) / max(len(finish_values), 1))
    return start_slow / (start_slow + finish_slow)


def _sample_speed_hint(path: ManualPathV40, point_s_mm: dict[int, float], sample_s: float) -> float | None:
    hints: list[float] = []
    for index, point in enumerate(path.points):
        if point.max_speed_mmps is None:
            continue
        if abs(point_s_mm[index] - sample_s) <= 30.0:
            hints.append(float(point.max_speed_mmps))
    return min(hints) if hints else None


def _segments_from_nodes(nodes: list, timing: TimeParameterizationResult, *, segment_flags: int) -> tuple[SegmentV40, ...]:
    arrival_indices = [index for index, node in enumerate(nodes) if node.flags & int(NodeFlag.ARRIVAL)]
    if not arrival_indices or arrival_indices[-1] != len(nodes) - 1:
        raise CompileError("deterministic path segments require final ARRIVAL")
    segment_times = _segment_times(timing, [nodes[index].s_mm for index in arrival_indices])
    segments: list[SegmentV40] = []
    start_index = 0
    for segment_id, (end_index, planned_time_ms) in enumerate(zip(arrival_indices, segment_times, strict=True)):
        start_arrival = 0xFF if segment_id == 0 else int(nodes[start_index].arrival_id)
        segments.append(
            SegmentV40(
                segment_id=segment_id,
                start_node_index=start_index,
                end_node_index=end_index,
                start_s_mm=nodes[start_index].s_mm,
                end_s_mm=nodes[end_index].s_mm,
                start_arrival_id=start_arrival,
                end_arrival_id=int(nodes[end_index].arrival_id),
                flags=segment_flags,
                planned_time_ms=planned_time_ms,
                source_leg_hash32=0,
            )
        )
        start_index = end_index
    return tuple(segments)


def _segment_times(timing: TimeParameterizationResult, end_s_values: list[int]) -> list[int]:
    totals = [0.0 for _ in end_s_values]
    segment_index = 0
    for left, right in zip(timing.samples, timing.samples[1:]):
        while segment_index + 1 < len(end_s_values) and left.s_mm >= end_s_values[segment_index] - 1.0e-7:
            segment_index += 1
        ds = right.s_mm - left.s_mm
        if ds <= 0.0:
            continue
        denominator = left.speed_mmps + right.speed_mmps
        if denominator <= _EPSILON:
            continue
        totals[segment_index] += 2000.0 * ds / denominator
    result = [max(0, round(value)) for value in totals]
    difference = timing.planned_time_ms - sum(result)
    if result:
        result[-1] += difference
    return result


def _case_yaw_policy(case: CaseManifestV40) -> YawPolicy:
    raw = str(case.selected_plan.get("yaw_direction", YawPolicy.SHORTEST.value))
    try:
        return YawPolicy(raw)
    except ValueError:
        return YawPolicy.SHORTEST


def _enum_number(enum_type, value: object) -> int:
    if isinstance(value, enum_type):
        return int(value)
    if isinstance(value, str):
        text = value.removeprefix("PATH_ACT_").removeprefix("ACTION_MODE_")
        try:
            return int(enum_type[text])
        except KeyError as exc:
            raise CompileError(f"unsupported {enum_type.__name__}: {value}") from exc
    return int(enum_type(int(value)))
