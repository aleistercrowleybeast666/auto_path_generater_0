# -*- coding: utf-8 -*-
"""V3.3 sparse-point geometry and arc-length resampling."""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from path_models import (
    EditPoint,
    GeometryResult,
    GeometrySample,
    PlannerConfig,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_CUT_IN,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)

GEOMETRY_EPS_MM = 1e-6
MIN_EDGE_LENGTH_MM = 1.0
BEZIER_CHORD_ERROR_MM = 0.75
BEZIER_TANGENT_CHANGE_RAD = math.radians(1.5)
ARRIVAL_REFINEMENT_RADIUS_MM = 80.0
ARRIVAL_SPACING_MM = 8.0
CURVE_SPACING_MM = 20.0


@dataclass
class _DensePoint:
    x_mm: float
    y_mm: float
    source_segment: int
    source_point: Optional[int] = None
    is_curve: bool = False


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _normalize(x: float, y: float) -> Tuple[float, float]:
    length = math.hypot(x, y)
    if length <= GEOMETRY_EPS_MM:
        return 1.0, 0.0
    return x / length, y / length


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _point_lerp(
    a: Tuple[float, float], b: Tuple[float, float], t: float
) -> Tuple[float, float]:
    return _lerp(a[0], b[0], t), _lerp(a[1], b[1], t)


def _quadratic_point(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    t: float,
) -> Tuple[float, float]:
    one_minus_t = 1.0 - t
    return (
        one_minus_t * one_minus_t * p0[0]
        + 2.0 * one_minus_t * t * p1[0]
        + t * t * p2[0],
        one_minus_t * one_minus_t * p0[1]
        + 2.0 * one_minus_t * t * p1[1]
        + t * t * p2[1],
    )


def _angle_between(
    vector_a: Tuple[float, float], vector_b: Tuple[float, float]
) -> float:
    ax, ay = _normalize(*vector_a)
    bx, by = _normalize(*vector_b)
    dot = max(-1.0, min(1.0, ax * bx + ay * by))
    return math.acos(dot)


def _quadratic_flat_enough(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
) -> bool:
    chord_dx = p2[0] - p0[0]
    chord_dy = p2[1] - p0[1]
    chord_length = math.hypot(chord_dx, chord_dy)
    if chord_length <= GEOMETRY_EPS_MM:
        return True
    cross = abs((p1[0] - p0[0]) * chord_dy - (p1[1] - p0[1]) * chord_dx)
    chord_error = cross / chord_length
    tangent_change = _angle_between(
        (p1[0] - p0[0], p1[1] - p0[1]),
        (p2[0] - p1[0], p2[1] - p1[1]),
    )
    return (
        chord_error <= BEZIER_CHORD_ERROR_MM
        and tangent_change <= BEZIER_TANGENT_CHANGE_RAD
    )


def _flatten_quadratic(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    depth: int = 0,
) -> List[Tuple[float, float]]:
    if depth >= 16 or _quadratic_flat_enough(p0, p1, p2):
        return [p0, p2]
    p01 = _point_lerp(p0, p1, 0.5)
    p12 = _point_lerp(p1, p2, 0.5)
    p012 = _point_lerp(p01, p12, 0.5)
    left = _flatten_quadratic(p0, p01, p012, depth + 1)
    right = _flatten_quadratic(p012, p12, p2, depth + 1)
    return left[:-1] + right


def validate_edit_points(points: Sequence[EditPoint]) -> List[str]:
    errors: List[str] = []
    if not (2 <= len(points) <= 100):
        errors.append(f"编辑点数量必须为 2~100，当前为 {len(points)}")
        return errors

    cut_in_rows = [index for index, point in enumerate(points) if point.type == POINT_TYPE_CUT_IN]
    if cut_in_rows != [0]:
        errors.append(f"必须恰好有一个 CUT_IN 且位于第 0 行，当前位置为 {cut_in_rows}")

    end_rows = [index for index, point in enumerate(points) if point.is_end]
    if len(end_rows) != 1:
        errors.append(f"必须恰好有一个 END ARRIVAL，当前数量为 {len(end_rows)}")

    gate_ids: List[int] = []
    for index, point in enumerate(points):
        if point.point_id != index:
            errors.append(f"编辑点 {index} 的 point_id={point.point_id}，应为 {index}")
        if point.type not in (POINT_TYPE_CUT_IN, POINT_TYPE_WAYPOINT, POINT_TYPE_ARRIVAL):
            errors.append(f"编辑点 {index} 的 type={point.type!r} 非法")
        if not (-32768 <= point.x_mm <= 32767 and -32768 <= point.y_mm <= 32767):
            errors.append(f"编辑点 {index} 的坐标超出 int16_t 范围")
        if point.type == POINT_TYPE_WAYPOINT:
            if point.yaw_ddeg != YAW_UNSPECIFIED_DDEG:
                errors.append(
                    f"WAYPOINT {index} 的 yaw_ddeg 必须为 0xFF，"
                    "WAYPOINT 不参与 yaw 规划"
                )
        elif not (-32768 <= point.yaw_ddeg <= 32767):
            errors.append(f"编辑点 {index} 的 yaw_ddeg 超出 int16_t 范围")
        if point.max_speed_mmps < 0:
            errors.append(f"编辑点 {index} 的 max_speed_mmps 不能为负数")
        if point.corner_trim_mm < 0:
            errors.append(f"编辑点 {index} 的 corner_trim_mm 不能为负数")
        if point.type == POINT_TYPE_CUT_IN:
            if index != 0:
                errors.append(f"编辑点 {index} 的 CUT_IN 只能位于第 0 行")
            if point.stop_required or point.gate_id != 0xFF or point.scan or point.is_end:
                errors.append("CUT_IN 不允许 STOP、Gate、SCAN 或 END")
        if point.type == POINT_TYPE_WAYPOINT:
            if point.stop_required or point.gate_id != 0xFF or point.scan or point.is_end:
                errors.append(f"WAYPOINT {index} 不允许 STOP、Gate、SCAN 或 END")
        if point.type == POINT_TYPE_ARRIVAL:
            if point.gate_id != 0xFF:
                if not 0 <= point.gate_id < 32:
                    errors.append(f"ARRIVAL {index} 的 gate_id 必须为 0xFF 或 0~31")
                else:
                    gate_ids.append(point.gate_id)
            if point.is_end and not point.stop_required:
                errors.append(f"END ARRIVAL {index} 必须设置 stop_required")
        elif point.gate_id != 0xFF:
            errors.append(f"编辑点 {index} 不是 ARRIVAL，不能设置 Gate")

    if gate_ids != list(range(len(gate_ids))):
        errors.append(f"编号 Gate 必须按路径顺序连续，当前为 {gate_ids}")
    if end_rows and end_rows[0] != len(points) - 1:
        errors.append("END ARRIVAL 必须是最后一个编辑点")

    for index in range(1, len(points)):
        previous = (points[index - 1].x_mm, points[index - 1].y_mm)
        current = (points[index].x_mm, points[index].y_mm)
        if _distance(previous, current) < MIN_EDGE_LENGTH_MM:
            errors.append(f"编辑点 {index - 1} 与 {index} 重复或距离小于 1 mm")
    return errors


def _is_roundable(point: EditPoint, index: int, point_count: int) -> bool:
    return (
        0 < index < point_count - 1
        and point.type == POINT_TYPE_WAYPOINT
        and not point.exact_pass
        and point.corner_trim_mm > GEOMETRY_EPS_MM
    )


def _append_dense(
    dense: List[_DensePoint],
    point: _DensePoint,
) -> None:
    if dense and _distance(
        (dense[-1].x_mm, dense[-1].y_mm), (point.x_mm, point.y_mm)
    ) <= GEOMETRY_EPS_MM:
        if point.source_point is not None:
            dense[-1].source_point = point.source_point
        dense[-1].is_curve = dense[-1].is_curve or point.is_curve
        return
    dense.append(point)


def _build_dense_polyline(points: Sequence[EditPoint]) -> List[_DensePoint]:
    corners: Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]] = {}
    for index in range(1, len(points) - 1):
        point = points[index]
        if not _is_roundable(point, index, len(points)):
            continue
        a = (points[index - 1].x_mm, points[index - 1].y_mm)
        b = (point.x_mm, point.y_mm)
        c = (points[index + 1].x_mm, points[index + 1].y_mm)
        length_ab = _distance(a, b)
        length_bc = _distance(b, c)
        trim = min(point.corner_trim_mm, 0.45 * length_ab, 0.45 * length_bc)
        if trim < MIN_EDGE_LENGTH_MM:
            continue
        pin = _point_lerp(b, a, trim / length_ab)
        pout = _point_lerp(b, c, trim / length_bc)
        corners[index] = (pin, pout)

    dense = [
        _DensePoint(
            points[0].x_mm,
            points[0].y_mm,
            source_segment=0,
            source_point=0,
        )
    ]
    current = (points[0].x_mm, points[0].y_mm)
    for index in range(1, len(points) - 1):
        point = points[index]
        if index in corners:
            pin, pout = corners[index]
            _append_dense(
                dense,
                _DensePoint(pin[0], pin[1], source_segment=index - 1),
            )
            curve = _flatten_quadratic(
                pin,
                (point.x_mm, point.y_mm),
                pout,
            )
            for x_mm, y_mm in curve[1:]:
                _append_dense(
                    dense,
                    _DensePoint(
                        x_mm,
                        y_mm,
                        source_segment=index,
                        is_curve=True,
                    ),
                )
            current = pout
        else:
            target = (point.x_mm, point.y_mm)
            _append_dense(
                dense,
                _DensePoint(
                    target[0],
                    target[1],
                    source_segment=index - 1,
                    source_point=index,
                ),
            )
            current = target

    last_index = len(points) - 1
    last = (points[last_index].x_mm, points[last_index].y_mm)
    if _distance(current, last) < MIN_EDGE_LENGTH_MM:
        raise ValueError(
            f"最后一段长度小于 {MIN_EDGE_LENGTH_MM:g} mm，无法生成稳定轨迹"
        )
    _append_dense(
        dense,
        _DensePoint(
            last[0],
            last[1],
            source_segment=last_index - 1,
            source_point=last_index,
        ),
    )
    return dense


def _cumulative_lengths(dense: Sequence[_DensePoint]) -> List[float]:
    cumulative = [0.0]
    for previous, current in zip(dense[:-1], dense[1:]):
        segment_length = _distance(
            (previous.x_mm, previous.y_mm), (current.x_mm, current.y_mm)
        )
        if segment_length <= GEOMETRY_EPS_MM:
            continue
        cumulative.append(cumulative[-1] + segment_length)
    if len(cumulative) != len(dense):
        raise ValueError("内部几何包含重复点，无法建立严格递增弧长")
    return cumulative


def _sample_dense_at_s(
    dense: Sequence[_DensePoint],
    cumulative: Sequence[float],
    s_mm: float,
    exact_source_by_s: Dict[float, int],
) -> GeometrySample:
    if s_mm <= 0.0:
        item = dense[0]
        return GeometrySample(
            0.0,
            item.x_mm,
            item.y_mm,
            source_segment=item.source_segment,
            source_point=exact_source_by_s.get(0.0, item.source_point),
        )
    if s_mm >= cumulative[-1]:
        item = dense[-1]
        return GeometrySample(
            cumulative[-1],
            item.x_mm,
            item.y_mm,
            source_segment=item.source_segment,
            source_point=exact_source_by_s.get(cumulative[-1], item.source_point),
        )
    right = bisect.bisect_right(cumulative, s_mm)
    left = right - 1
    span = cumulative[right] - cumulative[left]
    t = 0.0 if span <= GEOMETRY_EPS_MM else (s_mm - cumulative[left]) / span
    a = dense[left]
    b = dense[right]
    source_point = None
    for exact_s, point_index in exact_source_by_s.items():
        if abs(exact_s - s_mm) <= 1e-7:
            source_point = point_index
            break
    return GeometrySample(
        s_mm=s_mm,
        x_mm=_lerp(a.x_mm, b.x_mm, t),
        y_mm=_lerp(a.y_mm, b.y_mm, t),
        source_segment=a.source_segment if t < 0.5 else b.source_segment,
        source_point=source_point,
    )


def _calculate_differential_geometry(samples: List[GeometrySample]) -> None:
    for index, sample in enumerate(samples):
        if index == 0:
            a = samples[0]
            b = samples[1]
        elif index == len(samples) - 1:
            a = samples[-2]
            b = samples[-1]
        else:
            a = samples[index - 1]
            b = samples[index + 1]
        tx, ty = _normalize(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
        sample.tangent_x = tx
        sample.tangent_y = ty
        sample.normal_x = -ty
        sample.normal_y = tx

    for index, sample in enumerate(samples):
        if index == 0 or index == len(samples) - 1:
            sample.curvature_kappa_per_m = 0.0
            continue
        a = samples[index - 1]
        b = sample
        c = samples[index + 1]
        ab = _distance((a.x_mm, a.y_mm), (b.x_mm, b.y_mm))
        bc = _distance((b.x_mm, b.y_mm), (c.x_mm, c.y_mm))
        ac = _distance((a.x_mm, a.y_mm), (c.x_mm, c.y_mm))
        denominator = ab * bc * ac
        if denominator <= GEOMETRY_EPS_MM:
            sample.curvature_kappa_per_m = 0.0
            continue
        cross = (b.x_mm - a.x_mm) * (c.y_mm - a.y_mm) - (
            b.y_mm - a.y_mm
        ) * (c.x_mm - a.x_mm)
        sample.curvature_kappa_per_m = 2000.0 * cross / denominator

    if len(samples) >= 3:
        samples[0].curvature_kappa_per_m = samples[1].curvature_kappa_per_m
        samples[-1].curvature_kappa_per_m = samples[-2].curvature_kappa_per_m


def _nearest_dense_s(
    point: EditPoint,
    dense: Sequence[_DensePoint],
    cumulative: Sequence[float],
) -> float:
    best_index = min(
        range(len(dense)),
        key=lambda index: (dense[index].x_mm - point.x_mm) ** 2
        + (dense[index].y_mm - point.y_mm) ** 2,
    )
    return cumulative[best_index]


def _unique_sorted(values: Iterable[float]) -> List[float]:
    result: List[float] = []
    for value in sorted(values):
        if not result or abs(result[-1] - value) > 1e-7:
            result.append(value)
    return result


def generate_geometry(
    points: Sequence[EditPoint],
    planner: PlannerConfig,
) -> GeometryResult:
    errors = validate_edit_points(points)
    if errors:
        raise ValueError("\n".join(errors))
    if planner.nominal_spacing_mm <= 0:
        raise ValueError("planner.nominal_spacing_mm 必须大于 0")
    if not (0 < planner.max_spacing_mm <= 50):
        raise ValueError("planner.max_spacing_mm 必须在 1~50 mm")

    dense = _build_dense_polyline(points)
    cumulative = _cumulative_lengths(dense)
    total_length = cumulative[-1]
    if total_length <= GEOMETRY_EPS_MM:
        raise ValueError("轨迹总长度为 0")

    point_s_mm: Dict[int, float] = {}
    exact_source_by_s: Dict[float, int] = {}
    for index, point in enumerate(points):
        if _is_roundable(point, index, len(points)):
            point_s_mm[index] = _nearest_dense_s(point, dense, cumulative)
            continue
        matching = [
            dense_index
            for dense_index, dense_point in enumerate(dense)
            if dense_point.source_point == index
        ]
        if not matching:
            raise ValueError(f"精确编辑点 {index} 未进入内部几何")
        point_s = cumulative[matching[0]]
        point_s_mm[index] = point_s
        exact_source_by_s[point_s] = index

    breakpoints = {0.0, total_length}
    breakpoints.update(exact_source_by_s)
    for index, point in enumerate(points):
        if point.type == POINT_TYPE_ARRIVAL or point.exact_pass:
            center_s = point_s_mm[index]
            breakpoints.add(max(0.0, center_s - ARRIVAL_REFINEMENT_RADIUS_MM))
            breakpoints.add(min(total_length, center_s + ARRIVAL_REFINEMENT_RADIUS_MM))
    breakpoints = set(_unique_sorted(breakpoints))

    target_s = set(breakpoints)
    ordered_breakpoints = sorted(breakpoints)
    base_spacing = min(planner.nominal_spacing_mm, planner.max_spacing_mm)
    for start_s, end_s in zip(ordered_breakpoints[:-1], ordered_breakpoints[1:]):
        length = end_s - start_s
        if length <= GEOMETRY_EPS_MM:
            continue
        midpoint = 0.5 * (start_s + end_s)
        near_arrival = any(
            (point.type == POINT_TYPE_ARRIVAL or point.exact_pass)
            and abs(point_s_mm[index] - midpoint) <= ARRIVAL_REFINEMENT_RADIUS_MM
            for index, point in enumerate(points)
        )
        spacing = ARRIVAL_SPACING_MM if near_arrival else base_spacing
        count = max(1, int(math.ceil(length / spacing)))
        for step in range(1, count):
            target_s.add(start_s + length * step / count)

    samples = [
        _sample_dense_at_s(dense, cumulative, s_mm, exact_source_by_s)
        for s_mm in _unique_sorted(target_s)
    ]
    _calculate_differential_geometry(samples)

    for _iteration in range(4):
        additions: List[float] = []
        for previous, current in zip(samples[:-1], samples[1:]):
            ds = current.s_mm - previous.s_mm
            tangent_change = _angle_between(
                (previous.tangent_x, previous.tangent_y),
                (current.tangent_x, current.tangent_y),
            )
            curve_spacing = (
                min(base_spacing, CURVE_SPACING_MM)
                if max(
                    abs(previous.curvature_kappa_per_m),
                    abs(current.curvature_kappa_per_m),
                )
                > 0.01
                else base_spacing
            )
            required_spacing = min(curve_spacing, planner.max_spacing_mm)
            if ds > required_spacing + 1e-6 or (
                ds > 2.0 and tangent_change > math.radians(3.0)
            ):
                additions.append(0.5 * (previous.s_mm + current.s_mm))
        if not additions:
            break
        samples.extend(
            _sample_dense_at_s(dense, cumulative, s_mm, exact_source_by_s)
            for s_mm in additions
        )
        samples.sort(key=lambda sample: sample.s_mm)
        _calculate_differential_geometry(samples)

    for previous, current in zip(samples[:-1], samples[1:]):
        if current.s_mm <= previous.s_mm:
            raise ValueError("弧长采样不是严格递增")
        if current.s_mm - previous.s_mm > planner.max_spacing_mm + 1e-6:
            raise ValueError(
                f"s={previous.s_mm:.1f}~{current.s_mm:.1f} mm 的节点间距超过 "
                f"{planner.max_spacing_mm} mm"
            )
    return GeometryResult(samples=samples, point_s_mm=point_s_mm)


def validate_cut_in_straight(
    geometry: GeometryResult,
    points: Sequence[EditPoint],
    straight_length_mm: float,
) -> List[str]:
    errors: List[str] = []
    if straight_length_mm <= 0:
        errors.append("cut_in.straight_length_mm 必须大于 0")
        return errors
    if geometry.samples[-1].s_mm + 1e-6 < straight_length_mm:
        errors.append(
            f"正式轨迹总长 {geometry.samples[-1].s_mm:.1f} mm 小于切入直线要求 "
            f"{straight_length_mm:.1f} mm"
        )
        return errors

    first = geometry.samples[0]
    for sample in geometry.samples:
        if sample.s_mm > straight_length_mm + 1e-6:
            break
        tangent_change = _angle_between(
            (first.tangent_x, first.tangent_y),
            (sample.tangent_x, sample.tangent_y),
        )
        if tangent_change > math.radians(3.0):
            errors.append(
                f"CUT_IN 后 s={sample.s_mm:.1f} mm 切线变化 "
                f"{math.degrees(tangent_change):.2f}°，超过近直线限制 3°"
            )
            break

    for index, point in enumerate(points[1:], start=1):
        point_s = geometry.point_s_mm[index]
        if point_s > straight_length_mm + 1e-6:
            continue
        if (
            point.stop_required
            or point.scan
            or point.gate_id != 0xFF
            or point.type == POINT_TYPE_ARRIVAL
        ):
            errors.append(
                f"CUT_IN 后 {straight_length_mm:.0f} mm 内的编辑点 {index} "
                "包含 ARRIVAL、STOP、SCAN 或 Gate"
            )
    return errors
