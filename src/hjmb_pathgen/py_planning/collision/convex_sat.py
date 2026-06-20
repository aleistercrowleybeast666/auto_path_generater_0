"""Convex polygon collision and distance checks."""

from __future__ import annotations

import math
from dataclasses import dataclass

from hjmb_pathgen.py_planning.collision.distance import polygon_distance
from hjmb_pathgen.py_planning.collision.primitives import Point2


@dataclass(frozen=True)
class ConvexDistance:
    signed_clearance_mm: float
    intersects: bool
    min_overlap_mm: float


def convex_polygon_signed_distance(
    a_vertices: tuple[Point2, ...],
    b_vertices: tuple[Point2, ...],
    *,
    epsilon: float,
) -> ConvexDistance:
    _validate_polygon(a_vertices, "a_vertices")
    _validate_polygon(b_vertices, "b_vertices")
    min_overlap = math.inf
    separated = False
    for axis in _axes(a_vertices) + _axes(b_vertices):
        a_min, a_max = _project(a_vertices, axis)
        b_min, b_max = _project(b_vertices, axis)
        overlap = min(a_max, b_max) - max(a_min, b_min)
        if overlap < -epsilon:
            separated = True
            break
        min_overlap = min(min_overlap, max(0.0, overlap))
    if separated:
        return ConvexDistance(
            signed_clearance_mm=polygon_distance(a_vertices, b_vertices, epsilon=epsilon),
            intersects=False,
            min_overlap_mm=0.0,
        )
    clearance = 0.0 if min_overlap <= epsilon else -min_overlap
    return ConvexDistance(
        signed_clearance_mm=clearance,
        intersects=True,
        min_overlap_mm=0.0 if not math.isfinite(min_overlap) else min_overlap,
    )


def _validate_polygon(vertices: tuple[Point2, ...], name: str) -> None:
    if len(vertices) < 3:
        raise ValueError(f"{name} must contain at least three vertices")
    for left, right in zip(vertices, vertices[1:] + vertices[:1]):
        if math.hypot(right.x - left.x, right.y - left.y) <= 1.0e-9:
            raise ValueError(f"{name} contains a degenerate edge")


def _axes(vertices: tuple[Point2, ...]) -> tuple[Point2, ...]:
    result = []
    for left, right in zip(vertices, vertices[1:] + vertices[:1]):
        dx = right.x - left.x
        dy = right.y - left.y
        length = math.hypot(dx, dy)
        if length <= 1.0e-12:
            raise ValueError("degenerate polygon edge")
        result.append(Point2(-dy / length, dx / length))
    return tuple(result)


def _project(vertices: tuple[Point2, ...], axis: Point2) -> tuple[float, float]:
    values = [vertex.x * axis.x + vertex.y * axis.y for vertex in vertices]
    return min(values), max(values)
