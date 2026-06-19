# -*- coding: utf-8 -*-
"""HJMB V3.5 yaw, speed, wheel, action, and time planning."""
from __future__ import annotations

import bisect
import math
from dataclasses import replace
from typing import Dict, List, Optional, Sequence, Tuple

from .path_geometry import generate_geometry
from .path_models import (
    ACTION_MODE_ASYNC,
    ACTION_MODE_CODES,
    ACTION_MODE_KINEMATIC,
    ACTION_MODE_STOP_AND_WAIT,
    ACTIONS,
    ArrivalDepartureLock,
    DROP_ACTIONS,
    MAX_ACTIONS,
    MAX_ARRIVALS,
    MAX_NODES,
    MAX_TRAJ_ID,
    PATH_ACT_STORE,
    PREP_STORE_ACTION_SLOTS,
    EditPoint,
    GeometryResult,
    GeometrySample,
    MechanicalAction,
    PathProject,
    PlanResult,
    PlanSummary,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    ResolvedMechanicalAction,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_END,
    TRAJ_FLAG_START,
    TRAJ_FLAG_WAYPOINT,
    TrajectoryNode,
    VehicleProfile,
    YAW_ROTATION_CCW_ONLY,
    YAW_ROTATION_CW_ONLY,
    YAW_ROTATION_POLICIES,
    YAW_ROTATION_SHORTEST,
    resolve_edit_points,
)

SPEED_EPS_MMPS = 1e-6
DISTANCE_EPS_MM = 1e-6
CURVATURE_EPS_PER_M = 1e-6
Q_EPS_RAD_PER_MM = 1e-9
ACCEL_TOLERANCE_MMPS2 = 8.0
BETA_TOLERANCE_RADPS2 = 0.02
WHEEL_TOLERANCE_RPM = 0.75
SPEED_TOLERANCE_MMPS = 2.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalized_delta(target_rad: float, reference_rad: float) -> float:
    delta = target_rad - reference_rad
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    if abs(delta) < 1e-12:
        return 0.0
    return delta


def _yaw_delta_by_policy(
    start_rad: float,
    target_rad: float,
    policy: str,
) -> float:
    delta = _normalized_delta(target_rad, start_rad)
    if policy == YAW_ROTATION_SHORTEST:
        return delta
    if policy == YAW_ROTATION_CW_ONLY:
        return delta if delta <= 0.0 else delta - 2.0 * math.pi
    if policy == YAW_ROTATION_CCW_ONLY:
        return delta if delta >= 0.0 else delta + 2.0 * math.pi
    raise ValueError(f"planner.yaw_rotation_policy={policy!r} 非法")


def _unwrap_near(angle_rad: float, reference_rad: float) -> float:
    return reference_rad + _normalized_delta(angle_rad, reference_rad)


def _plan_yaw(
    points: Sequence[EditPoint],
    geometry: GeometryResult,
    policy: str,
) -> Tuple[List[float], List[float], List[float]]:
    """Plan yaw from START/ARRIVAL anchors; WAYPOINT never anchors yaw."""
    if policy not in YAW_ROTATION_POLICIES:
        raise ValueError(f"planner.yaw_rotation_policy={policy!r} 非法")
    anchors: List[Tuple[float, float]] = []
    previous_yaw: Optional[float] = None
    for index, point in enumerate(points):
        if point.type not in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
            continue
        point_s = geometry.point_s_mm[index]
        raw_yaw = math.radians(point.yaw_ddeg / 10.0)
        yaw = raw_yaw
        if previous_yaw is not None:
            yaw = previous_yaw + _yaw_delta_by_policy(previous_yaw, raw_yaw, policy)
        anchors.append((point_s, yaw))
        previous_yaw = yaw

    if len(anchors) < 2:
        raise ValueError("V3.5 yaw 规划至少需要 START 和一个 ARRIVAL 两个 yaw 锚点")

    yaw_values: List[float] = []
    q_values: List[float] = []
    q_prime_values: List[float] = []
    anchor_index = 0
    for sample in geometry.samples:
        while (
            anchor_index + 1 < len(anchors) - 1
            and sample.s_mm > anchors[anchor_index + 1][0]
        ):
            anchor_index += 1
        start_s, start_yaw = anchors[anchor_index]
        end_s, end_yaw = anchors[min(anchor_index + 1, len(anchors) - 1)]
        if end_s <= start_s + DISTANCE_EPS_MM:
            yaw = end_yaw
            q = 0.0
            q_prime = 0.0
        else:
            t = _clamp((sample.s_mm - start_s) / (end_s - start_s), 0.0, 1.0)
            interval = end_s - start_s
            delta_yaw = end_yaw - start_yaw
            blend = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
            blend_prime = 30.0 * t**2 - 60.0 * t**3 + 30.0 * t**4
            blend_second = 60.0 * t - 180.0 * t**2 + 120.0 * t**3
            yaw = start_yaw + delta_yaw * blend
            q = delta_yaw * blend_prime / interval
            q_prime = delta_yaw * blend_second / (interval * interval)
        yaw_values.append(yaw)
        q_values.append(q)
        q_prime_values.append(q_prime)
    return yaw_values, q_values, q_prime_values


def mecanum_wheel_rpm(
    vx_world_mmps: float,
    vy_world_mmps: float,
    wz_radps: float,
    yaw_rad: float,
    vehicle: VehicleProfile,
) -> Tuple[float, float, float, float]:
    """Return FL, FR, RL, RR wheel speeds using the selected X-drive convention."""
    if vehicle.wheel_radius_mm <= 0:
        raise ValueError("vehicle_profile.wheel_radius_mm 必须大于 0")
    if vehicle.rotation_radius_mm <= 0:
        raise ValueError("vehicle_profile.rotation_radius_mm 必须大于 0")
    if vehicle.mecanum_convention != "X_FL_FR_RL_RR":
        raise ValueError(
            "当前仅实现 mecanum_convention='X_FL_FR_RL_RR'；"
            f"当前为 {vehicle.mecanum_convention!r}"
        )
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    vx_body = cos_yaw * vx_world_mmps + sin_yaw * vy_world_mmps
    vy_body = -sin_yaw * vx_world_mmps + cos_yaw * vy_world_mmps
    rotation = vehicle.rotation_radius_mm * wz_radps
    wheel_linear = (
        vx_body - vy_body - rotation,
        vx_body + vy_body + rotation,
        vx_body + vy_body - rotation,
        vx_body - vy_body + rotation,
    )
    scale = 60.0 / (2.0 * math.pi * vehicle.wheel_radius_mm)
    return tuple(value * scale for value in wheel_linear)


def _max_wheel_rpm_per_path_speed(
    sample: GeometrySample,
    yaw_rad: float,
    q_rad_per_mm: float,
    vehicle: VehicleProfile,
) -> float:
    wheel_rpm = mecanum_wheel_rpm(
        sample.tangent_x,
        sample.tangent_y,
        q_rad_per_mm,
        yaw_rad,
        vehicle,
    )
    return max(abs(value) for value in wheel_rpm)


def _point_speed_limit(
    sample_s: float,
    points: Sequence[EditPoint],
    point_s_mm: Dict[int, float],
    global_limit: float,
) -> float:
    point_positions = [point_s_mm[index] for index in range(len(points))]
    right = bisect.bisect_right(point_positions, sample_s)
    left_index = max(0, right - 1)
    right_index = min(len(points) - 1, right)
    limits = [
        points[index].max_speed_mmps
        for index in {left_index, right_index}
        if points[index].max_speed_mmps > 0
    ]
    return min([global_limit] + limits)


def _local_speed_limits(
    project: PathProject,
    points: Sequence[EditPoint],
    geometry: GeometryResult,
    yaw_values: Sequence[float],
    q_values: Sequence[float],
    q_prime_values: Sequence[float],
) -> Tuple[List[float], List[str], List[float]]:
    planner = project.planner
    lateral_accel = (
        planner.lateral_accel_mmps2
        if planner.lateral_accel_mmps2 > 0
        else planner.linear_accel_mmps2
    )
    limits: List[float] = []
    sources: List[str] = []
    wheel_units: List[float] = []
    stop_by_source = {
        index for index, point in enumerate(points) if point.type == POINT_TYPE_ARRIVAL
    }
    for index, sample in enumerate(geometry.samples):
        candidates: List[Tuple[float, str]] = [
            (float(planner.max_speed_mmps), "global speed"),
            (
                _point_speed_limit(
                    sample.s_mm,
                    points,
                    geometry.point_s_mm,
                    planner.max_speed_mmps,
                ),
                "point speed",
            ),
        ]
        curvature_per_mm = abs(sample.curvature_kappa_per_m) / 1000.0
        if curvature_per_mm > CURVATURE_EPS_PER_M / 1000.0:
            candidates.append(
                (math.sqrt(lateral_accel / curvature_per_mm), "curvature")
            )
            candidates.append(
                (
                    math.sqrt(planner.linear_accel_mmps2 / curvature_per_mm),
                    "acceleration",
                )
            )
        if abs(q_values[index]) > Q_EPS_RAD_PER_MM:
            candidates.append(
                (planner.max_wz_radps / abs(q_values[index]), "yaw rate")
            )
        if abs(q_prime_values[index]) > 1e-12:
            candidates.append(
                (
                    math.sqrt(
                        planner.angular_accel_moving_radps2
                        / abs(q_prime_values[index])
                    ),
                    "angular accel",
                )
            )
        wheel_unit = _max_wheel_rpm_per_path_speed(
            sample,
            yaw_values[index],
            q_values[index],
            project.vehicle_profile,
        )
        wheel_units.append(wheel_unit)
        if wheel_unit > 1e-9:
            candidates.append(
                (
                    project.vehicle_profile.wheel_plan_limit_rpm / wheel_unit,
                    "wheel rpm",
                )
            )
        if sample.source_point in stop_by_source:
            candidates.append((0.0, "arrival stop"))
        value, source = min(candidates, key=lambda item: item[0])
        limits.append(max(0.0, value))
        sources.append(source)
    return limits, sources, wheel_units


def _acceleration_interval(
    sample: GeometrySample,
    speed_mmps: float,
    q_rad_per_mm: float,
    q_prime_rad_per_mm2: float,
    project: PathProject,
) -> Optional[Tuple[float, float]]:
    curvature_per_mm = abs(sample.curvature_kappa_per_m) / 1000.0
    a_n = speed_mmps * speed_mmps * curvature_per_mm
    total_limit = float(project.planner.linear_accel_mmps2)
    if a_n > total_limit + ACCEL_TOLERANCE_MMPS2:
        return None
    remaining_sq = max(0.0, total_limit * total_limit - a_n * a_n)
    available = math.sqrt(remaining_sq)
    lower = -available
    upper = available

    beta_limit = project.planner.angular_accel_moving_radps2
    offset = q_prime_rad_per_mm2 * speed_mmps * speed_mmps
    if abs(q_rad_per_mm) <= Q_EPS_RAD_PER_MM:
        if abs(offset) > beta_limit + BETA_TOLERANCE_RADPS2:
            return None
        return lower, upper

    beta_lower = (-beta_limit - offset) / q_rad_per_mm
    beta_upper = (beta_limit - offset) / q_rad_per_mm
    if beta_lower > beta_upper:
        beta_lower, beta_upper = beta_upper, beta_lower
    lower = max(lower, beta_lower)
    upper = min(upper, beta_upper)
    if lower > upper + 1e-9:
        return None
    return lower, upper


def _plan_speed(
    project: PathProject,
    geometry: GeometryResult,
    q_values: Sequence[float],
    q_prime_values: Sequence[float],
    local_limits: Sequence[float],
) -> Tuple[List[float], int]:
    speeds = list(local_limits)
    speeds[0] = 0.0
    speeds[-1] = 0.0

    converged_iteration = project.planner.max_iterations
    for iteration in range(project.planner.max_iterations):
        previous_speeds = list(speeds)
        speeds[0] = 0.0
        for index in range(len(speeds) - 1):
            ds = geometry.samples[index + 1].s_mm - geometry.samples[index].s_mm
            interval = _acceleration_interval(
                geometry.samples[index],
                speeds[index],
                q_values[index],
                q_prime_values[index],
                project,
            )
            if interval is None:
                raise ValueError(
                    f"s={geometry.samples[index].s_mm:.1f} mm 处速度 "
                    f"{speeds[index]:.1f} mm/s 无法满足合成加速度或 beta 约束"
                )
            maximum_accel = max(0.0, interval[1])
            candidate = math.sqrt(
                max(0.0, speeds[index] * speeds[index] + 2.0 * maximum_accel * ds)
            )
            speeds[index + 1] = min(
                speeds[index + 1], local_limits[index + 1], candidate
            )

        speeds[-1] = 0.0
        for index in range(len(speeds) - 2, -1, -1):
            ds = geometry.samples[index + 1].s_mm - geometry.samples[index].s_mm
            interval = _acceleration_interval(
                geometry.samples[index + 1],
                speeds[index + 1],
                q_values[index + 1],
                q_prime_values[index + 1],
                project,
            )
            if interval is None:
                raise ValueError(
                    f"s={geometry.samples[index + 1].s_mm:.1f} mm 处减速边界不可行"
                )
            maximum_decel = max(0.0, -interval[0])
            candidate = math.sqrt(
                max(
                    0.0,
                    speeds[index + 1] * speeds[index + 1]
                    + 2.0 * maximum_decel * ds,
                )
            )
            if index == 0:
                speeds[index] = 0.0
            else:
                speeds[index] = min(speeds[index], local_limits[index], candidate)

        for index in range(len(speeds) - 1):
            ds = geometry.samples[index + 1].s_mm - geometry.samples[index].s_mm
            if ds <= DISTANCE_EPS_MM:
                continue
            interval_start = _acceleration_interval(
                geometry.samples[index],
                speeds[index],
                q_values[index],
                q_prime_values[index],
                project,
            )
            interval_end = _acceleration_interval(
                geometry.samples[index + 1],
                speeds[index + 1],
                q_values[index + 1],
                q_prime_values[index + 1],
                project,
            )
            if interval_start is None or interval_end is None:
                raise ValueError(
                    f"s={geometry.samples[index].s_mm:.1f}~"
                    f"{geometry.samples[index + 1].s_mm:.1f} mm 的速度不可行"
                )
            lower = max(interval_start[0], interval_end[0])
            upper = min(interval_start[1], interval_end[1])
            if lower > upper + 1e-9:
                raise ValueError(
                    f"s={geometry.samples[index].s_mm:.1f}~"
                    f"{geometry.samples[index + 1].s_mm:.1f} mm 无可行切向加速度交集"
                )
            actual_accel = (
                speeds[index + 1] * speeds[index + 1]
                - speeds[index] * speeds[index]
            ) / (2.0 * ds)
            if actual_accel > upper:
                speeds[index + 1] = min(
                    speeds[index + 1],
                    math.sqrt(
                        max(
                            0.0,
                            speeds[index] * speeds[index] + 2.0 * upper * ds,
                        )
                    ),
                )
            elif actual_accel < lower and index > 0:
                candidate = math.sqrt(
                    max(
                        0.0,
                        speeds[index + 1] * speeds[index + 1] - 2.0 * lower * ds,
                    )
                )
                speeds[index] = min(speeds[index], candidate)

        maximum_change = max(
            abs(current - previous)
            for current, previous in zip(speeds, previous_speeds)
        )
        if maximum_change < project.planner.speed_convergence_mmps:
            converged_iteration = iteration + 1
            break

    for index, speed in enumerate(speeds):
        if not math.isfinite(speed) or speed < -SPEED_EPS_MMPS:
            raise ValueError(f"s={geometry.samples[index].s_mm:.1f} mm 规划出非法速度")
    speeds[0] = 0.0
    speeds[-1] = 0.0
    return speeds, converged_iteration


def _arrival_ids(points: Sequence[EditPoint]) -> Dict[int, int]:
    result: Dict[int, int] = {}
    next_arrival_id = 0
    for index, point in enumerate(points):
        if point.type == POINT_TYPE_ARRIVAL:
            result[index] = next_arrival_id
            next_arrival_id += 1
    return result


def _node_flags(
    points: Sequence[EditPoint],
    source_point: Optional[int],
    arrival_id_by_source: Dict[int, int],
) -> Tuple[int, int]:
    if source_point is None:
        return 0, 0xFF
    point = points[source_point]
    flags = 0
    arrival_id = 0xFF
    if point.type == POINT_TYPE_START:
        flags |= TRAJ_FLAG_START
    elif point.type == POINT_TYPE_ARRIVAL:
        flags |= TRAJ_FLAG_ARRIVAL
        arrival_id = arrival_id_by_source[source_point]
        if source_point == len(points) - 1:
            flags |= TRAJ_FLAG_END
    elif point.type == POINT_TYPE_WAYPOINT and point.exact_pass:
        flags |= TRAJ_FLAG_WAYPOINT
    return flags, arrival_id


def _build_nodes(
    project: PathProject,
    points: Sequence[EditPoint],
    geometry: GeometryResult,
    yaw_values: Sequence[float],
    q_values: Sequence[float],
    q_prime_values: Sequence[float],
    speeds: Sequence[float],
    constraint_sources: Sequence[str],
) -> List[TrajectoryNode]:
    nodes: List[TrajectoryNode] = []
    segment_accels: List[float] = []
    arrival_id_by_source = _arrival_ids(points)
    for previous_index in range(len(geometry.samples) - 1):
        ds = (
            geometry.samples[previous_index + 1].s_mm
            - geometry.samples[previous_index].s_mm
        )
        segment_accels.append(
            (
                speeds[previous_index + 1] * speeds[previous_index + 1]
                - speeds[previous_index] * speeds[previous_index]
            )
            / (2.0 * ds)
            if ds > DISTANCE_EPS_MM
            else 0.0
        )
    for index, sample in enumerate(geometry.samples):
        speed = speeds[index]
        if index == 0:
            a_t = segment_accels[0]
        elif index == len(geometry.samples) - 1:
            a_t = segment_accels[-1]
        else:
            incoming = segment_accels[index - 1]
            outgoing = segment_accels[index]
            incoming_beta = (
                q_values[index] * incoming
                + q_prime_values[index] * speed * speed
            )
            outgoing_beta = (
                q_values[index] * outgoing
                + q_prime_values[index] * speed * speed
            )
            incoming_score = max(
                abs(incoming) / max(project.planner.linear_accel_mmps2, 1),
                abs(incoming_beta)
                / max(project.planner.angular_accel_moving_radps2, 1e-9),
            )
            outgoing_score = max(
                abs(outgoing) / max(project.planner.linear_accel_mmps2, 1),
                abs(outgoing_beta)
                / max(project.planner.angular_accel_moving_radps2, 1e-9),
            )
            a_t = incoming if incoming_score >= outgoing_score else outgoing
        curvature_per_mm = abs(sample.curvature_kappa_per_m) / 1000.0
        a_n = speed * speed * curvature_per_mm
        a_total = math.hypot(a_t, a_n)
        beta = q_values[index] * a_t + q_prime_values[index] * speed * speed
        vx = sample.tangent_x * speed
        vy = sample.tangent_y * speed
        wz = q_values[index] * speed
        flags, arrival_id = _node_flags(points, sample.source_point, arrival_id_by_source)
        if flags & (TRAJ_FLAG_START | TRAJ_FLAG_ARRIVAL):
            speed = 0.0
            vx = 0.0
            vy = 0.0
            wz = 0.0
        wheels = mecanum_wheel_rpm(
            vx,
            vy,
            wz,
            yaw_values[index],
            project.vehicle_profile,
        )
        nodes.append(
            TrajectoryNode(
                s_mm=sample.s_mm,
                x_mm=sample.x_mm,
                y_mm=sample.y_mm,
                yaw_rad=yaw_values[index],
                vx_mmps=vx,
                vy_mmps=vy,
                wz_radps=wz,
                arrival_id=arrival_id,
                flags=flags,
                speed_mmps=speed,
                a_t_mmps2=a_t,
                a_n_mmps2=a_n,
                a_total_mmps2=a_total,
                beta_radps2=beta,
                curvature_kappa_per_m=sample.curvature_kappa_per_m,
                q_rad_per_mm=q_values[index],
                q_prime_rad_per_mm2=q_prime_values[index],
                max_wheel_rpm=max(abs(value) for value in wheels),
                constraint_source=constraint_sources[index],
                source_point=sample.source_point,
            )
        )
    return nodes


def _integrate_formal_time(nodes: Sequence[TrajectoryNode]) -> int:
    time_s = 0.0
    for previous, current in zip(nodes[:-1], nodes[1:]):
        ds = current.s_mm - previous.s_mm
        denominator = previous.speed_mmps + current.speed_mmps
        if ds > DISTANCE_EPS_MM and denominator <= SPEED_EPS_MMPS:
            raise ValueError(
                f"s={previous.s_mm:.1f}~{current.s_mm:.1f} mm 两端速度均为 0，"
                "无法积分正式轨迹时间"
            )
        if ds > DISTANCE_EPS_MM:
            time_s += 2.0 * ds / denominator
    return int(round(time_s * 1000.0))


def _estimate_mechanical_wait(project: PathProject) -> int:
    total = 0
    for action in project.actions:
        if action.mode != ACTION_MODE_STOP_AND_WAIT:
            continue
        name = ACTIONS.get(action.action, "")
        duration = project.mechanism_profile.action_duration_ms.get(name, 0)
        total += duration + action.post_wait_ms
    return total


def _point_lookup(
    points: Sequence[EditPoint],
    geometry: GeometryResult,
) -> Dict[int, Tuple[int, EditPoint, float]]:
    lookup: Dict[int, Tuple[int, EditPoint, float]] = {}
    for index, point in enumerate(points):
        lookup[point.point_id] = (index, point, geometry.point_s_mm[index])
    return lookup


def _segment_time_ms(previous: TrajectoryNode, current: TrajectoryNode) -> float:
    ds = current.s_mm - previous.s_mm
    denominator = previous.speed_mmps + current.speed_mmps
    if ds <= DISTANCE_EPS_MM:
        return 0.0
    if denominator <= SPEED_EPS_MMPS:
        return 0.0
    return 2000.0 * ds / denominator


def _action_conditions_met(action: MechanicalAction, node: TrajectoryNode) -> bool:
    if action.speed_limit_mmps and node.speed_mmps > action.speed_limit_mmps:
        return False
    if action.accel_limit_mmps2 and node.a_total_mmps2 > action.accel_limit_mmps2:
        return False
    wz_ddegps = abs(math.degrees(node.wz_radps) * 10.0)
    if action.wz_limit_ddegps and wz_ddegps > action.wz_limit_ddegps:
        return False
    beta_ddegps2 = abs(math.degrees(node.beta_radps2) * 10.0)
    if action.beta_limit_ddegps2 and beta_ddegps2 > action.beta_limit_ddegps2:
        return False
    return True


def _find_kinematic_check_start(
    action: MechanicalAction,
    nodes: Sequence[TrajectoryNode],
    lower_bound_s_mm: float,
) -> Optional[int]:
    stable_required_ms = float(action.stable_time_ms)
    stable_start_s: Optional[float] = None
    stable_time_ms = 0.0
    previous: Optional[TrajectoryNode] = None
    previous_ok = False

    for node in nodes:
        if node.s_mm + DISTANCE_EPS_MM < lower_bound_s_mm:
            previous = node
            previous_ok = False
            continue
        ok = _action_conditions_met(action, node)
        if ok and stable_start_s is None:
            stable_start_s = max(node.s_mm, lower_bound_s_mm)
            stable_time_ms = 0.0
        if ok and previous is not None and previous_ok:
            stable_time_ms += _segment_time_ms(previous, node)
            if stable_time_ms + 1e-6 >= stable_required_ms:
                return int(round(stable_start_s if stable_start_s is not None else node.s_mm))
        elif not ok:
            stable_start_s = None
            stable_time_ms = 0.0
        previous = node
        previous_ok = ok
    return None


def _arrival_s_by_id(
    points: Sequence[EditPoint],
    geometry: GeometryResult,
) -> Dict[int, float]:
    return {
        arrival_id: geometry.point_s_mm[index]
        for index, arrival_id in _arrival_ids(points).items()
    }


def _fallback_arrival_after(
    arrival_s_by_id: Dict[int, float],
    lower_bound_s_mm: float,
) -> Tuple[int, int]:
    ordered = sorted(arrival_s_by_id.items(), key=lambda item: item[1])
    for arrival_id, arrival_s in ordered:
        if arrival_s > lower_bound_s_mm + DISTANCE_EPS_MM:
            return arrival_id, int(round(arrival_s))
    arrival_id, arrival_s = ordered[-1]
    return arrival_id, int(round(arrival_s))


def _derive_departure_locks(
    actions: Sequence[ResolvedMechanicalAction],
) -> List[ArrivalDepartureLock]:
    by_arrival: Dict[int, List[int]] = {}
    for action in actions:
        if action.mode != ACTION_MODE_STOP_AND_WAIT:
            continue
        by_arrival.setdefault(action.arrival_id, []).append(action.action_seq)
    return [
        ArrivalDepartureLock(
            arrival_id=arrival_id,
            departure_action_seq=max(action_seqs),
            bound_action_seqs=sorted(action_seqs),
        )
        for arrival_id, action_seqs in sorted(by_arrival.items())
    ]


def resolve_actions(
    project: PathProject,
    points: Sequence[EditPoint],
    geometry: GeometryResult,
    nodes: Sequence[TrajectoryNode],
) -> List[ResolvedMechanicalAction]:
    errors = validate_actions(project, points, geometry)
    if errors:
        raise ValueError("\n".join(errors))

    lookup = _point_lookup(points, geometry)
    arrival_id_by_source = _arrival_ids(points)
    arrival_id_by_point_id = {
        points[index].point_id: arrival_id
        for index, arrival_id in arrival_id_by_source.items()
    }
    total_length = geometry.samples[-1].s_mm
    resolved: List[ResolvedMechanicalAction] = []
    for row, action in enumerate(project.actions):
        if action.mode == ACTION_MODE_STOP_AND_WAIT:
            arrival_point_id = action.arrival_point_id
            if arrival_point_id is None:
                raise ValueError(f"机械动作 {row} STOP_AND_WAIT 缺少 arrival_point_id")
            if arrival_point_id not in lookup:
                raise ValueError(f"机械动作 {row} 引用不存在的 arrival_point_id={arrival_point_id}")
            _index, point, _s_mm = lookup[arrival_point_id]
            if point.type != POINT_TYPE_ARRIVAL:
                raise ValueError(f"机械动作 {row} STOP_AND_WAIT 只能引用 ARRIVAL")
            resolved.append(
                ResolvedMechanicalAction(
                    action_seq=action.action_seq,
                    action=action.action,
                    mode=action.mode,
                    arrival_id=arrival_id_by_point_id[arrival_point_id],
                    timeout_ms=action.timeout_ms,
                    post_wait_ms=action.post_wait_ms,
                    check_start_s_mm=0xFFFF,
                    accel_limit_mmps2=0,
                    beta_limit_ddegps2=0,
                    wz_limit_ddegps=0,
                    speed_limit_mmps=0,
                    stable_time_ms=0,
                    execution_hint="ARRIVAL_STOP",
                )
            )
            continue

        if action.mode == ACTION_MODE_ASYNC:
            resolved.append(
                ResolvedMechanicalAction(
                    action_seq=action.action_seq,
                    action=action.action,
                    mode=action.mode,
                    arrival_id=0xFF,
                    timeout_ms=action.timeout_ms,
                    post_wait_ms=action.post_wait_ms,
                    check_start_s_mm=0xFFFF,
                    accel_limit_mmps2=0,
                    beta_limit_ddegps2=0,
                    wz_limit_ddegps=0,
                    speed_limit_mmps=0,
                    stable_time_ms=0,
                    execution_hint="FIFO_HEAD",
                )
            )
            continue

        if action.mode == ACTION_MODE_KINEMATIC:
            lower_bound_s = 0.0
            for previous_action in reversed(project.actions[:row]):
                if previous_action.mode != ACTION_MODE_STOP_AND_WAIT:
                    continue
                previous_arrival = previous_action.arrival_point_id
                if previous_arrival is not None and previous_arrival in lookup:
                    lower_bound_s = lookup[previous_arrival][2]
                    break
            check_start = _find_kinematic_check_start(action, nodes, lower_bound_s)
            execution_hint = "MOVING"
            fallback_arrival_id = None
            if check_start is None:
                fallback_arrival_id, check_start = _fallback_arrival_after(
                    _arrival_s_by_id(points, geometry),
                    lower_bound_s,
                )
                execution_hint = "ARRIVAL_FALLBACK"
            resolved.append(
                ResolvedMechanicalAction(
                    action_seq=action.action_seq,
                    action=action.action,
                    mode=action.mode,
                    arrival_id=0xFF,
                    timeout_ms=action.timeout_ms,
                    post_wait_ms=action.post_wait_ms,
                    check_start_s_mm=check_start,
                    accel_limit_mmps2=action.accel_limit_mmps2,
                    beta_limit_ddegps2=action.beta_limit_ddegps2,
                    wz_limit_ddegps=action.wz_limit_ddegps,
                    speed_limit_mmps=action.speed_limit_mmps,
                    stable_time_ms=action.stable_time_ms,
                    execution_hint=execution_hint,
                    fallback_arrival_id=fallback_arrival_id,
                )
            )
            continue
        raise ValueError(f"机械动作 {row} mode={action.mode!r} 非法")
    return resolved


def validate_actions(
    project: PathProject,
    points: Sequence[EditPoint],
    geometry: Optional[GeometryResult] = None,
) -> List[str]:
    errors: List[str] = []
    if len(project.actions) > MAX_ACTIONS:
        errors.append(f"机械动作数量不能超过 {MAX_ACTIONS}")
        return errors

    pending_store_slot = 0
    arrival_point_ids = {point.point_id for point in points if point.type == POINT_TYPE_ARRIVAL}
    for row, action in enumerate(project.actions):
        if action.action_seq != row:
            errors.append(f"机械动作 {row} 的 action_seq={action.action_seq}，应为 {row}")
        if action.action not in ACTIONS:
            errors.append(f"机械动作 {row} 的 action=0x{action.action:02X} 非法")
        if action.mode not in ACTION_MODE_CODES:
            errors.append(f"机械动作 {row} mode={action.mode!r} 非法")
        if action.timeout_ms <= 0:
            errors.append(f"机械动作 {row} timeout_ms 必须大于 0")
        for field_name in (
            "timeout_ms",
            "post_wait_ms",
            "accel_limit_mmps2",
            "beta_limit_ddegps2",
            "wz_limit_ddegps",
            "speed_limit_mmps",
            "stable_time_ms",
        ):
            value = getattr(action, field_name)
            if field_name.endswith("offset_mm"):
                if not -0x8000 <= value <= 0x7FFF:
                    errors.append(f"机械动作 {row} 的 {field_name}={value} 超出 int16_t")
            elif not 0 <= value <= 0xFFFF:
                errors.append(f"机械动作 {row} 的 {field_name}={value} 超出 uint16_t")

        if action.mode == ACTION_MODE_STOP_AND_WAIT:
            if action.arrival_point_id not in arrival_point_ids:
                errors.append(f"机械动作 {row} STOP_AND_WAIT 必须引用 ARRIVAL")
            if any(
                (
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                    action.stable_time_ms,
                )
            ):
                errors.append(f"机械动作 {row} STOP_AND_WAIT 含无效运动限制字段")
        elif action.mode == ACTION_MODE_ASYNC:
            if any(
                (
                    action.arrival_point_id is not None,
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                    action.stable_time_ms,
                )
            ):
                errors.append(f"机械动作 {row} ASYNC 含无效 arrival 或运动限制字段")
        elif action.mode == ACTION_MODE_KINEMATIC:
            if action.arrival_point_id is not None:
                errors.append(f"机械动作 {row} KINEMATIC 含无效 arrival 字段")
            if not any(
                (
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                )
            ):
                errors.append(f"机械动作 {row} KINEMATIC 至少需要一个运动限制")
            if action.stable_time_ms <= 0:
                errors.append(f"机械动作 {row} KINEMATIC stable_time_ms 必须大于 0")

        if action.action in PREP_STORE_ACTION_SLOTS:
            if pending_store_slot != 0:
                errors.append(
                    f"机械动作 {row} PREP_STORE 前一个暂存仓 "
                    f"{pending_store_slot} 尚未执行 STORE"
                )
            pending_store_slot = PREP_STORE_ACTION_SLOTS[action.action]
        elif action.action == PATH_ACT_STORE:
            if pending_store_slot == 0:
                errors.append(f"机械动作 {row} STORE 前没有 PREP_STORE")
            pending_store_slot = 0
        elif action.action in DROP_ACTIONS:
            pass
    if pending_store_slot:
        errors.append(
            f"文件结束时 pending_store_slot={pending_store_slot}，"
            "缺少对应 STORE"
        )
    return errors


def validate_resolved_actions(
    actions: Sequence[ResolvedMechanicalAction],
    total_length_mm: float,
    arrival_count: int,
) -> List[str]:
    errors: List[str] = []
    pending_store_slot = 0
    previous_stop_arrival_id = -1
    quantized_total_length_mm = int(round(total_length_mm))
    for row, action in enumerate(actions):
        if action.action_seq != row:
            errors.append(f"action[{row}].action_seq={action.action_seq}，应为 {row}")
        if action.action not in ACTIONS:
            errors.append(f"action[{row}].action=0x{action.action:02X} 非法")
        if action.mode not in ACTION_MODE_CODES:
            errors.append(f"action[{row}].mode={action.mode!r} 非法")
        if action.timeout_ms <= 0:
            errors.append(f"action[{row}].timeout_ms 必须大于 0")
        if not 0 <= action.post_wait_ms <= 0xFFFF:
            errors.append(f"action[{row}].post_wait_ms 超出 uint16_t")
        if action.mode == ACTION_MODE_STOP_AND_WAIT:
            if not 0 <= action.arrival_id < arrival_count:
                errors.append(f"action[{row}] STOP_AND_WAIT arrival_id 非法")
            if action.arrival_id < previous_stop_arrival_id:
                errors.append(f"action[{row}] STOP_AND_WAIT arrival_id 必须按 action_seq 非递减")
            previous_stop_arrival_id = max(previous_stop_arrival_id, action.arrival_id)
            if action.check_start_s_mm != 0xFFFF:
                errors.append(f"action[{row}] STOP_AND_WAIT check_start_s_mm 必须为 0xFFFF")
            if any(
                (
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                    action.stable_time_ms,
                )
            ):
                errors.append(f"action[{row}] STOP_AND_WAIT 运动限制必须为 0")
        elif action.mode == ACTION_MODE_ASYNC:
            if action.arrival_id != 0xFF or action.check_start_s_mm != 0xFFFF:
                errors.append(f"action[{row}] ASYNC arrival/check_start 字段非法")
            if any(
                (
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                    action.stable_time_ms,
                )
            ):
                errors.append(f"action[{row}] ASYNC 运动限制必须为 0")
        elif action.mode == ACTION_MODE_KINEMATIC:
            if action.arrival_id != 0xFF:
                errors.append(f"action[{row}] KINEMATIC arrival_id 必须为 0xFF")
            if not 0 <= action.check_start_s_mm <= quantized_total_length_mm:
                errors.append(f"action[{row}] KINEMATIC check_start_s_mm 超出轨迹范围")
            if not any(
                (
                    action.accel_limit_mmps2,
                    action.beta_limit_ddegps2,
                    action.wz_limit_ddegps,
                    action.speed_limit_mmps,
                )
            ):
                errors.append(f"action[{row}] KINEMATIC 至少需要一个运动限制")
            if action.stable_time_ms <= 0:
                errors.append(f"action[{row}] KINEMATIC stable_time_ms 必须大于 0")
        if action.action in PREP_STORE_ACTION_SLOTS:
            if pending_store_slot != 0:
                errors.append(f"action[{row}] PREP_STORE 缺少前序 STORE")
            pending_store_slot = PREP_STORE_ACTION_SLOTS[action.action]
        elif action.action == PATH_ACT_STORE:
            if pending_store_slot == 0:
                errors.append(f"action[{row}] STORE 前没有 PREP_STORE")
            pending_store_slot = 0
    if pending_store_slot:
        errors.append("文件结束时 pending_store_slot 未清空")
    return errors


def _validate_nodes(
    project: PathProject,
    points: Sequence[EditPoint],
    nodes: Sequence[TrajectoryNode],
) -> List[str]:
    errors: List[str] = []
    if not (2 <= len(nodes) <= MAX_NODES):
        errors.append(f"规划节点数量必须为 2~{MAX_NODES}，当前为 {len(nodes)}")
        return errors
    if nodes[-1].s_mm > 0xFFFF:
        errors.append(f"轨迹总长度 {nodes[-1].s_mm:.1f} mm 超过 uint16_t 上限 65535 mm")
    if not nodes[0].flags & TRAJ_FLAG_START:
        errors.append("首节点必须设置 START")
    if nodes[0].flags & (TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_WAYPOINT | TRAJ_FLAG_END):
        errors.append("START 首节点禁止 ARRIVAL/WAYPOINT/END")
    if nodes[0].arrival_id != 0xFF:
        errors.append("START 首节点 arrival_id 必须为 0xFF")
    if nodes[0].speed_mmps > SPEED_TOLERANCE_MMPS or abs(nodes[0].wz_radps) > 0.01:
        errors.append("START 首节点前馈速度必须为 0")
    last_flags = nodes[-1].flags
    if last_flags & (TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END) != TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END:
        errors.append("末节点必须设置 ARRIVAL|END")
    if sum(bool(node.flags & TRAJ_FLAG_START) for node in nodes) != 1:
        errors.append("START 节点必须全文件唯一")
    if sum(bool(node.flags & TRAJ_FLAG_END) for node in nodes) != 1:
        errors.append("END 节点必须全文件唯一")

    arrival_ids = [node.arrival_id for node in nodes if node.flags & TRAJ_FLAG_ARRIVAL]
    if arrival_ids != list(range(len(arrival_ids))):
        errors.append(f"arrival_id 必须按路径连续，当前为 {arrival_ids}")
    if len(arrival_ids) > MAX_ARRIVALS:
        errors.append(f"arrival_count 不能超过 {MAX_ARRIVALS}")

    previous_s = -1.0
    previous_yaw = nodes[0].yaw_rad
    for index, node in enumerate(nodes):
        if node.s_mm <= previous_s and index != 0:
            errors.append(f"节点 {index} 的 s_mm 未严格递增")
        previous_s = node.s_mm
        if node.flags & TRAJ_FLAG_ARRIVAL:
            if node.arrival_id == 0xFF:
                errors.append(f"ARRIVAL 节点 {index} 缺少 arrival_id")
            if (
                abs(node.vx_mmps) > SPEED_TOLERANCE_MMPS
                or abs(node.vy_mmps) > SPEED_TOLERANCE_MMPS
                or abs(node.wz_radps) > 0.01
            ):
                errors.append(f"ARRIVAL 节点 {index} 的前馈速度不为 0")
        elif node.arrival_id != 0xFF:
            errors.append(f"非 ARRIVAL 节点 {index} arrival_id 必须为 0xFF")
        if node.a_total_mmps2 > project.planner.linear_accel_mmps2 + ACCEL_TOLERANCE_MMPS2:
            errors.append(
                f"节点 {index} s={node.s_mm:.1f} 的 a_total={node.a_total_mmps2:.1f} "
                "超过合成平移加速度上限"
            )
        if abs(node.beta_radps2) > (
            project.planner.angular_accel_moving_radps2 + BETA_TOLERANCE_RADPS2
        ):
            errors.append(
                f"节点 {index} s={node.s_mm:.1f} 的 beta={node.beta_radps2:.3f} "
                "超过移动角加速度上限"
            )
        if node.max_wheel_rpm > (
            project.vehicle_profile.wheel_plan_limit_rpm + WHEEL_TOLERANCE_RPM
        ):
            errors.append(
                f"节点 {index} s={node.s_mm:.1f} 的轮速 {node.max_wheel_rpm:.2f} rpm "
                "超过规划软限制"
            )
        if index > 0 and abs(node.yaw_rad - previous_yaw) > 2.0 * math.pi + 1e-6:
            errors.append(f"节点 {index} 的 yaw 出现异常跳变")
        previous_yaw = node.yaw_rad
        if node.source_point is not None:
            source = points[node.source_point]
            if source.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
                expected_yaw = _unwrap_near(
                    math.radians(source.yaw_ddeg / 10.0),
                    node.yaw_rad,
                )
                if abs(node.yaw_rad - expected_yaw) > 1e-8:
                    errors.append(
                        f"yaw 锚点 {source.point_id} 未精确达到指定角度"
                    )
                if abs(node.wz_radps) > 1e-8:
                    errors.append(
                        f"yaw 锚点 {source.point_id} 的规划角速度必须为 0"
                    )
    return errors


def _build_summary(
    project: PathProject,
    nodes: Sequence[TrajectoryNode],
) -> PlanSummary:
    formal_time_ms = _integrate_formal_time(nodes)
    mechanical_wait_ms = _estimate_mechanical_wait(project)
    max_wheel_node = max(nodes, key=lambda node: node.max_wheel_rpm)
    wheel_limited_length = 0.0
    high_accel_length = 0.0
    for previous, current in zip(nodes[:-1], nodes[1:]):
        ds = current.s_mm - previous.s_mm
        if (
            previous.constraint_source == "wheel rpm"
            or current.constraint_source == "wheel rpm"
        ):
            wheel_limited_length += ds
        if max(previous.a_total_mmps2, current.a_total_mmps2) >= (
            2.0 * project.planner.linear_accel_mmps2 / 3.0
        ):
            high_accel_length += ds
    return PlanSummary(
        total_length_mm=nodes[-1].s_mm,
        formal_time_ms=formal_time_ms,
        mechanical_wait_time_ms=mechanical_wait_ms,
        estimated_total_time_ms=formal_time_ms + mechanical_wait_ms,
        max_speed_mmps=max(node.speed_mmps for node in nodes),
        max_a_total_mmps2=max(node.a_total_mmps2 for node in nodes),
        max_a_n_mmps2=max(node.a_n_mmps2 for node in nodes),
        max_wz_radps=max(abs(node.wz_radps) for node in nodes),
        max_beta_radps2=max(abs(node.beta_radps2) for node in nodes),
        max_wheel_rpm=max_wheel_node.max_wheel_rpm,
        max_wheel_rpm_s_mm=max_wheel_node.s_mm,
        wheel_limited_length_mm=wheel_limited_length,
        high_accel_length_mm=high_accel_length,
    )


def validate_project_config(project: PathProject) -> List[str]:
    errors: List[str] = []
    if not 0 <= project.traj_id <= MAX_TRAJ_ID:
        errors.append(f"traj_id 必须为 0~{MAX_TRAJ_ID}")
    if project.field.width_mm != 4000 or project.field.height_mm != 2000:
        errors.append("V3.5 field.width_mm/height_mm 必须固定为 4000/2000")
    planner = project.planner
    if planner.max_speed_mmps <= 0:
        errors.append("planner.max_speed_mmps 必须大于 0")
    if planner.linear_accel_mmps2 <= 0:
        errors.append("planner.linear_accel_mmps2 必须大于 0")
    if planner.max_wz_radps <= 0:
        errors.append("planner.max_wz_radps 必须大于 0")
    if planner.angular_accel_moving_radps2 <= 0:
        errors.append("planner.angular_accel_moving_radps2 必须大于 0")
    if planner.yaw_rotation_policy not in YAW_ROTATION_POLICIES:
        errors.append("planner.yaw_rotation_policy 非法")
    if not 1 <= planner.nominal_spacing_mm <= 50:
        errors.append("planner.nominal_spacing_mm 必须为 1~50")
    if not 1 <= planner.max_spacing_mm <= 50:
        errors.append("planner.max_spacing_mm 必须为 1~50")
    vehicle = project.vehicle_profile
    if vehicle.wheel_radius_mm <= 0 or vehicle.rotation_radius_mm <= 0:
        errors.append("车辆轮半径和旋转半径必须大于 0")
    if vehicle.wheel_plan_limit_rpm <= 0:
        errors.append("wheel_plan_limit_rpm 必须大于 0")
    if vehicle.wheel_hard_limit_rpm < vehicle.wheel_plan_limit_rpm:
        errors.append("wheel_hard_limit_rpm 必须不小于 wheel_plan_limit_rpm")
    for label, config in (
        ("start_check", project.start_check),
        ("arrival_check", project.arrival_check),
    ):
        for field_name, value in vars(config).items():
            if value < 0:
                errors.append(f"{label}.{field_name} 不能为负数")
    return errors


def plan_project(project: PathProject) -> PlanResult:
    config_errors = validate_project_config(project)
    if config_errors:
        raise ValueError("\n".join(config_errors))
    points = resolve_edit_points(project)
    geometry = generate_geometry(points, project.planner)

    yaw_values, q_values, q_prime_values = _plan_yaw(
        points,
        geometry,
        project.planner.yaw_rotation_policy,
    )
    local_limits, constraint_sources, _wheel_units = _local_speed_limits(
        project,
        points,
        geometry,
        yaw_values,
        q_values,
        q_prime_values,
    )
    speeds, convergence_iterations = _plan_speed(
        project,
        geometry,
        q_values,
        q_prime_values,
        local_limits,
    )
    nodes = _build_nodes(
        project,
        points,
        geometry,
        yaw_values,
        q_values,
        q_prime_values,
        speeds,
        constraint_sources,
    )
    action_errors = validate_actions(project, points, geometry)
    if action_errors:
        raise ValueError("\n".join(action_errors))
    resolved_actions = resolve_actions(project, points, geometry, nodes)
    departure_locks = _derive_departure_locks(resolved_actions)
    errors = _validate_nodes(project, points, nodes)
    errors.extend(
        validate_resolved_actions(
            resolved_actions,
            nodes[-1].s_mm,
            sum(bool(node.flags & TRAJ_FLAG_ARRIVAL) for node in nodes),
        )
    )
    if errors:
        raise ValueError("\n".join(errors))

    summary = _build_summary(project, nodes)
    warnings = [
        f"速度规划在 {convergence_iterations} 次正反向扫描内收敛",
        project.vehicle_profile.geometry_note,
        "mechanical_wait_time_ms 是编辑器估算，不写入 BIN 且不代表确定执行时间",
    ]
    return PlanResult(
        nodes=nodes,
        actions=resolved_actions,
        summary=summary,
        warnings=warnings,
        departure_locks=departure_locks,
    )


def clone_actions(actions: Sequence[MechanicalAction]) -> List[MechanicalAction]:
    return [replace(action) for action in actions]
