"""Deterministic initial XY guesses for Phase 6 leg optimization."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from hjmb_pathgen.py_planning.geometry.bezier import Point2D, point_from_dict
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationRequest
from hjmb_pathgen.py_domain.topology import TopologyGate
from hjmb_pathgen.py_planning.geometry.obstacle_detours import obstacle_aware_seeds


@dataclass(frozen=True)
class InitialGuess:
    guess_id: str
    source: str
    waypoints: tuple[Point2D, ...]
    tension: float = 0.75


def build_initial_guesses(request: LegOptimizationRequest) -> tuple[InitialGuess, ...]:
    start = Point2D(request.from_pose.x_mm, request.from_pose.y_mm)
    finish = Point2D(request.to_pose.x_mm, request.to_pose.y_mm)
    guesses: list[InitialGuess] = []
    if request.initial_control_points:
        guesses.append(_manual_guess(start, finish, request.initial_control_points, "MANUAL_CONTROL_POINTS"))
    if request.warm_start_leg is not None and request.warm_start_leg.control_points:
        guesses.append(_warm_start_guess(start, finish, request.warm_start_leg.control_points))
    for seed in obstacle_aware_seeds(request):
        guesses.append(
            InitialGuess(
                seed.seed_id,
                seed.source,
                seed.waypoints,
                tension=seed.tension,
            )
        )
    guesses.append(InitialGuess("straight", "STRAIGHT", (start, finish)))
    if request.topology_gates:
        guesses.append(InitialGuess("gate_center", "TOPOLOGY_GATE_CENTER", _gate_points(start, finish, request.topology_gates, offset_ratio=0.0)))
        guesses.append(InitialGuess("gate_offset_neg", "TOPOLOGY_GATE_OFFSET", _gate_points(start, finish, request.topology_gates, offset_ratio=-0.25)))
        guesses.append(InitialGuess("gate_offset_pos", "TOPOLOGY_GATE_OFFSET", _gate_points(start, finish, request.topology_gates, offset_ratio=0.25)))
    return _deduplicate_guesses(tuple(guesses))


def _manual_guess(start: Point2D, finish: Point2D, control_points: Iterable[dict[str, object]], source: str) -> InitialGuess:
    points = [start]
    for item in control_points:
        try:
            point = point_from_dict(dict(item))
        except (KeyError, TypeError, ValueError):
            continue
        if math.hypot(point.x_mm - points[-1].x_mm, point.y_mm - points[-1].y_mm) > 1.0e-9:
            points.append(point)
    if math.hypot(finish.x_mm - points[-1].x_mm, finish.y_mm - points[-1].y_mm) > 1.0e-9:
        points.append(finish)
    return InitialGuess(source.lower(), source, tuple(points))


def _warm_start_guess(start: Point2D, finish: Point2D, control_points: Iterable[dict[str, object]]) -> InitialGuess:
    ordered = sorted((dict(item) for item in control_points), key=lambda item: int(item.get("order", 0)))
    if ordered and all(str(item.get("representation", "")) == "PIECEWISE_CUBIC_BEZIER" for item in ordered):
        # control_points_dicts stores P0,P1,P2,P3 and then P1,P2,P3 per segment.
        # The through waypoints are P0 and every segment end P3.
        through = [ordered[0]]
        through.extend(ordered[index] for index in range(3, len(ordered), 3))
        return _manual_guess(start, finish, through, "WARM_START_BEZIER_ENDPOINTS")
    return _manual_guess(start, finish, ordered, "WARM_START_NODES")


def _gate_points(start: Point2D, finish: Point2D, gates: tuple[TopologyGate, ...], *, offset_ratio: float) -> tuple[Point2D, ...]:
    points: list[Point2D] = [start]
    for gate in gates:
        cx, cy = gate.center
        vx, vy = gate.vector
        points.append(Point2D(cx + vx * offset_ratio, cy + vy * offset_ratio))
    points.append(finish)
    return tuple(points)


def _deduplicate_guesses(guesses: tuple[InitialGuess, ...]) -> tuple[InitialGuess, ...]:
    seen: set[tuple[tuple[int, int], ...]] = set()
    result: list[InitialGuess] = []
    for guess in guesses:
        key = tuple((round(point.x_mm * 1000), round(point.y_mm * 1000)) for point in guess.waypoints)
        if key in seen:
            continue
        seen.add(key)
        result.append(guess)
    return tuple(result)
