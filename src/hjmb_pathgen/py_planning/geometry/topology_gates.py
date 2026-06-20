"""Ordered virtual topology gate validation."""

from __future__ import annotations

from typing import Iterable

from hjmb_pathgen.py_domain.topology import (
    TopologyGate,
    TopologyGateCrossing,
    TopologyGateDirection,
    TopologyValidationResult,
)

EPSILON = 1.0e-9


def validate_ordered_topology_gates(points: Iterable[object], gates: tuple[TopologyGate, ...]) -> TopologyValidationResult:
    if not gates:
        return TopologyValidationResult(success=True)
    path_points = tuple((_x(point), _y(point)) for point in points)
    if len(path_points) < 2:
        return TopologyValidationResult(success=False, errors=("path has fewer than two points",))
    crossings: list[TopologyGateCrossing] = []
    min_global_parameter = -EPSILON
    for gate in gates:
        crossing = _find_gate_crossing(path_points, gate, min_global_parameter)
        if crossing is None:
            return TopologyValidationResult(
                success=False,
                crossings=tuple(crossings),
                errors=(f"topology gate {gate.gate_id} was not crossed in order",),
            )
        crossings.append(crossing)
        min_global_parameter = crossing.global_path_parameter
    return TopologyValidationResult(success=True, crossings=tuple(crossings))


def _find_gate_crossing(points: tuple[tuple[float, float], ...], gate: TopologyGate, min_global_parameter: float) -> TopologyGateCrossing | None:
    gate_vector = (gate.bx_mm - gate.ax_mm, gate.by_mm - gate.ay_mm)
    best: TopologyGateCrossing | None = None
    for index in range(0, len(points) - 1):
        left = points[index]
        right = points[index + 1]
        path_vector = (right[0] - left[0], right[1] - left[1])
        if abs(_cross(path_vector, gate_vector)) <= EPSILON:
            # Collinear sliding along a virtual gate is ambiguous topology, not a crossing.
            continue
        hit = _segment_intersection(left, path_vector, (gate.ax_mm, gate.ay_mm), gate_vector)
        if hit is None:
            continue
        path_ratio, gate_ratio = hit
        global_path_parameter = index + path_ratio
        if global_path_parameter <= min_global_parameter + EPSILON:
            continue
        signed_direction = _cross(gate_vector, path_vector)
        if gate.direction == TopologyGateDirection.POSITIVE and signed_direction <= EPSILON:
            continue
        if gate.direction == TopologyGateDirection.NEGATIVE and signed_direction >= -EPSILON:
            continue
        candidate = TopologyGateCrossing(
            gate_id=gate.gate_id,
            path_segment_index=index,
            path_ratio=path_ratio,
            gate_ratio=gate_ratio,
            signed_direction=signed_direction,
            global_path_parameter=global_path_parameter,
        )
        if best is None or candidate.global_path_parameter < best.global_path_parameter:
            best = candidate
    return best


def _segment_intersection(
    path_start: tuple[float, float],
    path_vector: tuple[float, float],
    gate_start: tuple[float, float],
    gate_vector: tuple[float, float],
) -> tuple[float, float] | None:
    denominator = _cross(path_vector, gate_vector)
    if abs(denominator) <= EPSILON:
        return None
    delta = (gate_start[0] - path_start[0], gate_start[1] - path_start[1])
    path_ratio = _cross(delta, gate_vector) / denominator
    gate_ratio = _cross(delta, path_vector) / denominator
    if -EPSILON <= path_ratio <= 1.0 + EPSILON and -EPSILON <= gate_ratio <= 1.0 + EPSILON:
        return path_ratio, gate_ratio
    return None


def _cross(left: tuple[float, float], right: tuple[float, float]) -> float:
    return left[0] * right[1] - left[1] * right[0]


def _x(point: object) -> float:
    if isinstance(point, dict):
        return float(point.get("x_mm", point.get("x", 0.0)))
    return float(getattr(point, "x_mm", getattr(point, "x", 0.0)))


def _y(point: object) -> float:
    if isinstance(point, dict):
        return float(point.get("y_mm", point.get("y", 0.0)))
    return float(getattr(point, "y_mm", getattr(point, "y", 0.0)))
