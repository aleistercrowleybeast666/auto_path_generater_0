# -*- coding: utf-8 -*-
"""HJMB V3.3 yaw, speed, wheel, action, and time planning."""
from __future__ import annotations

import bisect
import math
from dataclasses import replace
from typing import Dict, List, Optional, Sequence, Tuple

from path_geometry import generate_geometry, validate_cut_in_straight
from path_models import (
    ACTION_FLAG_HOLD_PATH,
    ACTION_FLAG_LOCKED,
    ACTION_FLAG_REQUIRED_AT_END,
    ACTION_GATE_ACCEL,
    ACTION_GATE_UNCONDITIONAL,
    ACTIONS,
    MAX_ACTIONS,
    MAX_NODES,
    MAX_TRAJ_ID,
    DROP_ACTIONS,
    PREP_STORE_ACTION_SLOTS,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_CUT_IN,
    TRAJ_FLAG_END,
    TRAJ_FLAG_GATE,
    TRAJ_FLAG_SCAN,
    TRAJ_FLAG_STOP,
    TRAJ_FLAG_WAYPOINT,
    VALID_ACTION_FLAGS_MASK,
    CutInPreviewResult,
    EditPoint,
    GeometryResult,
    GeometrySample,
    MechanicalAction,
    PathProject,
    PlanResult,
    PlanSummary,
    TrajectoryNode,
    VehicleProfile,
    PATH_ACT_PICK,
    PATH_ACT_STORE,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_CUT_IN,
    POINT_TYPE_WAYPOINT,
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


def _unwrap_near(angle_rad: float, reference_rad: float) -> float:
    while angle_rad - reference_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad - reference_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def _plan_yaw(
    points: Sequence[EditPoint],
    geometry: GeometryResult,
) -> Tuple[List[float], List[float], List[float]]:
    """Plan yaw from CUT_IN/ARRIVAL anchors; WAYPOINT never anchors yaw."""
    anchors: List[Tuple[float, float]] = []
    previous_yaw: Optional[float] = None
    for index, point in enumerate(points):
        if point.type not in (POINT_TYPE_CUT_IN, POINT_TYPE_ARRIVAL):
            continue
        point_s = geometry.point_s_mm[index]
        yaw = math.radians(point.yaw_ddeg / 10.0)
        if previous_yaw is not None:
            yaw = _unwrap_near(yaw, previous_yaw)
        anchors.append((point_s, yaw))
        previous_yaw = yaw

    if len(anchors) < 2:
        raise ValueError("V3.3 yaw 规划至少需要 CUT_IN 和 END ARRIVAL 两个 yaw 锚点")

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
        index
        for index, point in enumerate(project.points)
        if point.type == POINT_TYPE_ARRIVAL and point.stop_required
    }
    for index, sample in enumerate(geometry.samples):
        candidates: List[Tuple[float, str]] = [
            (float(planner.max_speed_mmps), "global speed"),
            (
                _point_speed_limit(
                    sample.s_mm,
                    project.points,
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
            candidates.append((0.0, "stop boundary"))
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
    target_speed = float(project.cut_in.target_speed_mmps)
    if local_limits[0] + SPEED_TOLERANCE_MMPS < target_speed:
        raise ValueError(
            f"CUT_IN 目标速度 {target_speed:.1f} mm/s 超过首节点局部上限 "
            f"{local_limits[0]:.1f} mm/s"
        )
    speeds = list(local_limits)
    speeds[0] = target_speed
    speeds[-1] = 0.0

    converged_iteration = project.planner.max_iterations
    for iteration in range(project.planner.max_iterations):
        previous_speeds = list(speeds)
        speeds[0] = target_speed
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
                if candidate + SPEED_TOLERANCE_MMPS < target_speed:
                    raise ValueError(
                        "CUT_IN 目标速度无法在后续 STOP/曲率约束前安全减速；"
                        "请降低切入速度或调整首段"
                    )
                speeds[index] = target_speed
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
            elif actual_accel < lower:
                candidate = math.sqrt(
                    max(
                        0.0,
                        speeds[index + 1] * speeds[index + 1] - 2.0 * lower * ds,
                    )
                )
                if index == 0 and candidate + SPEED_TOLERANCE_MMPS < target_speed:
                    raise ValueError(
                        "CUT_IN 目标速度在首段合成加速度/beta 约束下不可行"
                    )
                if index > 0:
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
    return speeds, converged_iteration


def _node_flags(
    project: PathProject,
    source_point: Optional[int],
) -> Tuple[int, int]:
    if source_point is None:
        return 0, 0xFF
    point = project.points[source_point]
    flags = 0
    gate_id = 0xFF
    if point.type == POINT_TYPE_CUT_IN:
        flags |= TRAJ_FLAG_CUT_IN
    elif point.type == POINT_TYPE_ARRIVAL:
        flags |= TRAJ_FLAG_ARRIVAL
        if point.stop_required:
            flags |= TRAJ_FLAG_STOP
        if point.scan:
            flags |= TRAJ_FLAG_SCAN
        if point.gate_id != 0xFF:
            flags |= TRAJ_FLAG_GATE
            gate_id = point.gate_id
        if point.is_end:
            flags |= TRAJ_FLAG_END
    elif point.type == POINT_TYPE_WAYPOINT and point.exact_pass:
        flags |= TRAJ_FLAG_WAYPOINT
    return flags, gate_id


def _build_nodes(
    project: PathProject,
    geometry: GeometryResult,
    yaw_values: Sequence[float],
    q_values: Sequence[float],
    q_prime_values: Sequence[float],
    speeds: Sequence[float],
    constraint_sources: Sequence[str],
) -> List[TrajectoryNode]:
    nodes: List[TrajectoryNode] = []
    segment_accels: List[float] = []
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
        flags, gate_id = _node_flags(project, sample.source_point)
        if flags & TRAJ_FLAG_STOP:
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
                gate_id=gate_id,
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


def estimate_cut_in_preview(project: PathProject) -> CutInPreviewResult:
    preview = project.preview_initial_pose
    if not preview.enabled:
        return CutInPreviewResult(enabled=False)
    if not project.points:
        return CutInPreviewResult(
            enabled=True, warning="没有 CUT_IN 点，无法估算切入段"
        )
    cut_in = project.points[0]
    distance = math.hypot(cut_in.x_mm - preview.x_mm, cut_in.y_mm - preview.y_mm)
    initial_speed = max(0.0, preview.initial_speed_mmps)
    final_speed = float(project.cut_in.target_speed_mmps)
    maximum_speed = float(project.cut_in.approach_max_speed_mmps)
    accel = float(project.planner.linear_accel_mmps2)
    result = CutInPreviewResult(enabled=True, distance_mm=distance)
    if accel <= 0 or maximum_speed <= 0 or final_speed <= 0:
        result.warning = "切入速度或加速度参数非法"
        return result
    if initial_speed > maximum_speed + SPEED_TOLERANCE_MMPS:
        result.warning = "预览初速度超过 approach_max_speed"
        return result

    minimum_distance = abs(final_speed * final_speed - initial_speed * initial_speed) / (
        2.0 * accel
    )
    if distance + 1e-6 < minimum_distance:
        result.warning = (
            f"距离不足：至少需要 {minimum_distance:.1f} mm 才能从 "
            f"{initial_speed:.1f} mm/s 到达 {final_speed:.1f} mm/s"
        )
        return result

    peak_sq = accel * distance + 0.5 * (
        initial_speed * initial_speed + final_speed * final_speed
    )
    unconstrained_peak = math.sqrt(max(0.0, peak_sq))
    if unconstrained_peak <= maximum_speed + 1e-6:
        peak = max(unconstrained_peak, initial_speed, final_speed)
        time_s = abs(peak - initial_speed) / accel + abs(peak - final_speed) / accel
    else:
        peak = maximum_speed
        accel_distance = max(
            0.0, (peak * peak - initial_speed * initial_speed) / (2.0 * accel)
        )
        decel_distance = max(
            0.0, (peak * peak - final_speed * final_speed) / (2.0 * accel)
        )
        cruise_distance = max(0.0, distance - accel_distance - decel_distance)
        time_s = (
            abs(peak - initial_speed) / accel
            + abs(peak - final_speed) / accel
            + cruise_distance / max(peak, SPEED_EPS_MMPS)
        )
    result.reachable = True
    result.peak_speed_mmps = peak
    result.time_ms = int(round(time_s * 1000.0))
    return result


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
        name = ACTIONS.get(action.action, "")
        duration = project.mechanism_profile.action_duration_ms.get(name, 0)
        if action.flags & ACTION_FLAG_HOLD_PATH:
            total += duration
        elif action.flags & ACTION_FLAG_REQUIRED_AT_END:
            total += duration
    return total


def _find_accel_gate_window(
    project: PathProject,
    nodes: Sequence[TrajectoryNode],
    action: MechanicalAction,
) -> Optional[Tuple[int, int]]:
    required_ms = (
        action.stable_time_ms
        + project.mechanism_profile.action_duration_ms.get(
            ACTIONS.get(action.action, ""),
            0,
        )
        + project.mechanism_profile.drop_safety_margin_ms
    )
    accumulated_ms = 0.0
    start_index: Optional[int] = None
    for index, node in enumerate(nodes):
        beta_ddegps2 = abs(math.degrees(node.beta_radps2) * 10.0)
        condition = (
            (action.accel_limit_mmps2 == 0 or node.a_total_mmps2 <= action.accel_limit_mmps2)
            and (
                action.beta_limit_ddegps2 == 0
                or beta_ddegps2 <= action.beta_limit_ddegps2
            )
            and (
                action.speed_limit_mmps == 0
                or node.speed_mmps <= action.speed_limit_mmps
            )
            and not (node.flags & TRAJ_FLAG_STOP)
        )
        if not condition:
            accumulated_ms = 0.0
            start_index = None
            continue
        if start_index is None:
            start_index = index
        if index > start_index:
            previous = nodes[index - 1]
            ds = node.s_mm - previous.s_mm
            denominator = node.speed_mmps + previous.speed_mmps
            if denominator > SPEED_EPS_MMPS:
                accumulated_ms += 2000.0 * ds / denominator
        if accumulated_ms + 1e-6 >= required_ms:
            return int(round(nodes[start_index].s_mm)), int(round(node.s_mm))
    return None


def _prepare_accel_gate_actions(
    project: PathProject,
    nodes: Sequence[TrajectoryNode],
) -> None:
    for action in project.actions:
        if action.action not in DROP_ACTIONS or action.unlock_gate_id != ACTION_GATE_ACCEL:
            continue
        window = _find_accel_gate_window(project, nodes, action)
        if window is None:
            raise ValueError(
                f"机械动作 {action.action_seq} 的 0xFE "
                f"{ACTIONS[action.action]} 找不到满足 a_total、beta、speed "
                "和持续时间要求的区间；请改为停车 DROP"
            )
        action.arm_s_mm, action.disarm_s_mm = window


def validate_actions(
    project: PathProject,
    nodes: Sequence[TrajectoryNode],
) -> List[str]:
    errors: List[str] = []
    if len(project.actions) > MAX_ACTIONS:
        errors.append(f"机械动作数量不能超过 {MAX_ACTIONS}")
        return errors
    gate_nodes = {
        node.gate_id: node
        for node in nodes
        if node.flags & TRAJ_FLAG_GATE and node.gate_id != 0xFF
    }
    expected_gates = list(range(len(gate_nodes)))
    if sorted(gate_nodes) != expected_gates:
        errors.append(f"轨迹 Gate 必须连续为 {expected_gates}，当前为 {sorted(gate_nodes)}")

    pending_store_slot = 0
    previous_numbered_gate = -1
    previous_blocking_s = 0.0
    for row, action in enumerate(project.actions):
        if action.action_seq != row:
            errors.append(f"机械动作 {row} 的 action_seq={action.action_seq}，应为 {row}")
        if action.action not in ACTIONS:
            errors.append(f"机械动作 {row} 的 action=0x{action.action:02X} 非法")
        if action.flags & ~VALID_ACTION_FLAGS_MASK:
            errors.append(f"机械动作 {row} 的 flags 含未定义位")
        for field_name in (
            "timeout_ms",
            "arm_s_mm",
            "disarm_s_mm",
            "accel_limit_mmps2",
            "beta_limit_ddegps2",
            "speed_limit_mmps",
            "stable_time_ms",
        ):
            value = getattr(action, field_name)
            if not 0 <= value <= 0xFFFF:
                errors.append(f"机械动作 {row} 的 {field_name}={value} 超出 uint16_t")

        locked = bool(action.flags & ACTION_FLAG_LOCKED)
        hold_path = bool(action.flags & ACTION_FLAG_HOLD_PATH)
        gate_id = action.unlock_gate_id
        if gate_id == ACTION_GATE_UNCONDITIONAL:
            if locked:
                errors.append(f"机械动作 {row} 使用 0xFF 时不能设置 LOCKED")
            if hold_path:
                errors.append(f"机械动作 {row} 使用 0xFF 时不能设置 HOLD_PATH")
        elif gate_id == ACTION_GATE_ACCEL:
            if not locked:
                errors.append(f"机械动作 {row} 使用 0xFE 时必须设置 LOCKED")
            if hold_path:
                errors.append(f"机械动作 {row} 使用 0xFE 时禁止 HOLD_PATH")
            if action.disarm_s_mm != 0xFFFF and action.disarm_s_mm < action.arm_s_mm:
                errors.append(f"机械动作 {row} 的 disarm_s_mm 小于 arm_s_mm")
            if action.arm_s_mm > nodes[-1].s_mm:
                errors.append(f"机械动作 {row} 的 arm_s_mm 超出轨迹总长度")
            if (
                action.disarm_s_mm != 0xFFFF
                and action.disarm_s_mm > nodes[-1].s_mm
            ):
                errors.append(f"机械动作 {row} 的 disarm_s_mm 超出轨迹总长度")
            if (
                action.disarm_s_mm != 0xFFFF
                and action.disarm_s_mm + 1e-6 < previous_blocking_s
            ):
                errors.append(
                    f"机械动作 {row} 的 0xFE 窗口在前序 Gate 之后不可达，存在 FIFO 死锁风险"
                )
        elif gate_id in gate_nodes:
            if not locked:
                errors.append(f"机械动作 {row} 引用编号 Gate 时必须设置 LOCKED")
            if gate_id < previous_numbered_gate:
                errors.append(f"机械动作 {row} 的编号 Gate 倒退，存在 FIFO 死锁风险")
            previous_numbered_gate = gate_id
            previous_blocking_s = max(previous_blocking_s, gate_nodes[gate_id].s_mm)
        else:
            errors.append(f"机械动作 {row} 引用不存在的 Gate {gate_id}")

        if action.action == PATH_ACT_PICK:
            gate_node = gate_nodes.get(gate_id)
            if gate_node is None or not (
                gate_node.flags & TRAJ_FLAG_ARRIVAL
                and gate_node.flags & TRAJ_FLAG_GATE
                and gate_node.flags & TRAJ_FLAG_STOP
            ):
                errors.append(f"机械动作 {row} PICK 必须引用 ARRIVAL|GATE|STOP")
            if not hold_path:
                errors.append(f"机械动作 {row} PICK 必须设置 HOLD_PATH")

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
            if gate_id not in (ACTION_GATE_ACCEL, ACTION_GATE_UNCONDITIONAL):
                gate_node = gate_nodes.get(gate_id)
                if gate_node is None or not (
                    gate_node.flags & TRAJ_FLAG_ARRIVAL
                    and gate_node.flags & TRAJ_FLAG_GATE
                    and gate_node.flags & TRAJ_FLAG_STOP
                ):
                    errors.append(
                        f"机械动作 {row} 停车 {ACTIONS[action.action]} "
                        "必须引用 ARRIVAL|GATE|STOP"
                    )
                if not hold_path:
                    errors.append(
                        f"机械动作 {row} 停车 {ACTIONS[action.action]} "
                        "必须设置 HOLD_PATH"
                    )
            if gate_id == ACTION_GATE_UNCONDITIONAL:
                errors.append(
                    f"机械动作 {row} {ACTIONS[action.action]} "
                    "不能使用 0xFF 无条件 Gate"
                )
    if pending_store_slot:
        errors.append(
            f"文件结束时 pending_store_slot={pending_store_slot}，"
            "缺少对应 STORE"
        )
    return errors


def _validate_nodes(project: PathProject, nodes: Sequence[TrajectoryNode]) -> List[str]:
    errors: List[str] = []
    if not (2 <= len(nodes) <= MAX_NODES):
        errors.append(f"规划节点数量必须为 2~{MAX_NODES}，当前为 {len(nodes)}")
        return errors
    if nodes[-1].s_mm > 0xFFFF:
        errors.append(f"轨迹总长度 {nodes[-1].s_mm:.1f} mm 超过 uint16_t 上限 65535 mm")
    if not nodes[0].flags & TRAJ_FLAG_CUT_IN:
        errors.append("首节点必须设置 CUT_IN")
    if nodes[0].flags & (TRAJ_FLAG_STOP | TRAJ_FLAG_GATE):
        errors.append("CUT_IN 首节点禁止 STOP/GATE")
    if abs(nodes[0].speed_mmps - project.cut_in.target_speed_mmps) > SPEED_TOLERANCE_MMPS:
        errors.append("CUT_IN 首节点速度与 cut_in.target_speed_mmps 不一致")
    last_flags = nodes[-1].flags
    if (
        last_flags & (TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END | TRAJ_FLAG_STOP)
        != TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END | TRAJ_FLAG_STOP
    ):
        errors.append("末节点必须设置 ARRIVAL|END|STOP")
    if sum(bool(node.flags & TRAJ_FLAG_CUT_IN) for node in nodes) != 1:
        errors.append("CUT_IN 节点必须全文件唯一")
    if sum(bool(node.flags & TRAJ_FLAG_END) for node in nodes) != 1:
        errors.append("END 节点必须全文件唯一")

    previous_s = -1.0
    previous_yaw = nodes[0].yaw_rad
    for index, node in enumerate(nodes):
        if node.s_mm <= previous_s and index != 0:
            errors.append(f"节点 {index} 的 s_mm 未严格递增")
        previous_s = node.s_mm
        if node.flags & TRAJ_FLAG_STOP and (
            abs(node.vx_mmps) > SPEED_TOLERANCE_MMPS
            or abs(node.vy_mmps) > SPEED_TOLERANCE_MMPS
            or abs(node.wz_radps) > 0.01
        ):
            errors.append(f"STOP 节点 {index} 的前馈速度不为 0")
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
        if index > 0 and abs(node.yaw_rad - previous_yaw) > math.pi:
            errors.append(f"节点 {index} 的 yaw 出现超过 180° 的跳变")
        previous_yaw = node.yaw_rad
        if node.source_point is not None:
            source = project.points[node.source_point]
            if source.type in (POINT_TYPE_CUT_IN, POINT_TYPE_ARRIVAL):
                expected_yaw = _unwrap_near(
                    math.radians(source.yaw_ddeg / 10.0),
                    node.yaw_rad,
                )
                if abs(node.yaw_rad - expected_yaw) > 1e-8:
                    errors.append(
                        f"yaw 锚点 {node.source_point} 未精确达到指定角度"
                    )
                if abs(node.wz_radps) > 1e-8:
                    errors.append(
                        f"yaw 锚点 {node.source_point} 的规划角速度必须为 0"
                    )
        if (
            node.s_mm <= project.cut_in.straight_length_mm + 1e-6
            and node.speed_mmps + SPEED_TOLERANCE_MMPS
            < project.cut_in.target_speed_mmps
        ):
            errors.append(
                f"CUT_IN 直线段 s={node.s_mm:.1f} mm 的速度 {node.speed_mmps:.1f} "
                f"低于目标速度 {project.cut_in.target_speed_mmps} mm/s"
            )
    return errors


def _build_summary(
    project: PathProject,
    nodes: Sequence[TrajectoryNode],
    preview: CutInPreviewResult,
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
    cut_in_time_ms = preview.time_ms if preview.enabled and preview.reachable else 0
    return PlanSummary(
        total_length_mm=nodes[-1].s_mm,
        formal_time_ms=formal_time_ms,
        cut_in_preview_time_ms=cut_in_time_ms,
        mechanical_wait_time_ms=mechanical_wait_ms,
        estimated_total_time_ms=formal_time_ms + cut_in_time_ms + mechanical_wait_ms,
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
        errors.append("V3.3 field.width_mm/height_mm 必须固定为 4000/2000")
    planner = project.planner
    if planner.max_speed_mmps <= 0:
        errors.append("planner.max_speed_mmps 必须大于 0")
    if planner.linear_accel_mmps2 <= 0:
        errors.append("planner.linear_accel_mmps2 必须大于 0")
    if planner.max_wz_radps <= 0:
        errors.append("planner.max_wz_radps 必须大于 0")
    if planner.angular_accel_moving_radps2 <= 0:
        errors.append("planner.angular_accel_moving_radps2 必须大于 0")
    if not 1 <= planner.nominal_spacing_mm <= 50:
        errors.append("planner.nominal_spacing_mm 必须为 1~50")
    if not 1 <= planner.max_spacing_mm <= 50:
        errors.append("planner.max_spacing_mm 必须为 1~50")
    if project.cut_in.capture_radius_mm <= 0:
        errors.append("cut_in.capture_radius_mm 必须大于 0")
    if project.cut_in.target_speed_mmps <= 0:
        errors.append("cut_in.target_speed_mmps 必须大于 0")
    if project.cut_in.target_speed_mmps > planner.max_speed_mmps:
        errors.append("cut_in.target_speed_mmps 不能超过 planner.max_speed_mmps")
    if project.cut_in.approach_max_speed_mmps < project.cut_in.target_speed_mmps:
        errors.append("cut_in.approach_max_speed_mmps 必须不小于 target_speed_mmps")
    vehicle = project.vehicle_profile
    if vehicle.wheel_radius_mm <= 0 or vehicle.rotation_radius_mm <= 0:
        errors.append("车辆轮半径和旋转半径必须大于 0")
    if vehicle.wheel_plan_limit_rpm <= 0:
        errors.append("wheel_plan_limit_rpm 必须大于 0")
    if vehicle.wheel_hard_limit_rpm < vehicle.wheel_plan_limit_rpm:
        errors.append("wheel_hard_limit_rpm 必须不小于 wheel_plan_limit_rpm")
    return errors


def plan_project(project: PathProject) -> PlanResult:
    config_errors = validate_project_config(project)
    if config_errors:
        raise ValueError("\n".join(config_errors))
    geometry = generate_geometry(project.points, project.planner)
    straight_errors = validate_cut_in_straight(
        geometry, project.points, project.cut_in.straight_length_mm
    )
    if straight_errors:
        raise ValueError("\n".join(straight_errors))

    yaw_values, q_values, q_prime_values = _plan_yaw(project.points, geometry)
    local_limits, constraint_sources, _wheel_units = _local_speed_limits(
        project,
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
        geometry,
        yaw_values,
        q_values,
        q_prime_values,
        speeds,
        constraint_sources,
    )
    planned_actions = clone_actions(project.actions)
    planned_project = replace(project, actions=planned_actions)
    action_errors = validate_actions(planned_project, nodes)
    if action_errors:
        raise ValueError("\n".join(action_errors))
    _prepare_accel_gate_actions(planned_project, nodes)
    errors = _validate_nodes(project, nodes)
    errors.extend(validate_actions(planned_project, nodes))
    if errors:
        raise ValueError("\n".join(errors))

    preview = estimate_cut_in_preview(project)
    summary = _build_summary(project, nodes, preview)
    warnings = [
        f"速度规划在 {convergence_iterations} 次正反向扫描内收敛",
        project.vehicle_profile.geometry_note,
    ]
    if preview.enabled and not preview.reachable:
        warnings.append(preview.warning)
    return PlanResult(
        nodes=nodes,
        actions=planned_actions,
        summary=summary,
        cut_in_preview=preview,
        warnings=warnings,
    )


def clone_actions(actions: Sequence[MechanicalAction]) -> List[MechanicalAction]:
    return [replace(action) for action in actions]
