"""Cubic Bezier path representation with arc-length sampling."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

EPSILON = 1.0e-9


@dataclass(frozen=True)
class Point2D:
    x_mm: float
    y_mm: float

    def to_dict(self) -> dict[str, float]:
        return {"x_mm": self.x_mm, "y_mm": self.y_mm}


@dataclass(frozen=True)
class PathSample2D:
    s_mm: float
    x_mm: float
    y_mm: float
    tangent_x: float
    tangent_y: float
    curvature_1_per_mm: float


@dataclass(frozen=True)
class CubicBezier:
    p0: Point2D
    p1: Point2D
    p2: Point2D
    p3: Point2D

    def evaluate(self, t: float) -> Point2D:
        u = 1.0 - t
        b0 = u * u * u
        b1 = 3.0 * u * u * t
        b2 = 3.0 * u * t * t
        b3 = t * t * t
        return Point2D(
            b0 * self.p0.x_mm + b1 * self.p1.x_mm + b2 * self.p2.x_mm + b3 * self.p3.x_mm,
            b0 * self.p0.y_mm + b1 * self.p1.y_mm + b2 * self.p2.y_mm + b3 * self.p3.y_mm,
        )

    def derivative(self, t: float) -> tuple[float, float]:
        u = 1.0 - t
        dx = (
            3.0 * u * u * (self.p1.x_mm - self.p0.x_mm)
            + 6.0 * u * t * (self.p2.x_mm - self.p1.x_mm)
            + 3.0 * t * t * (self.p3.x_mm - self.p2.x_mm)
        )
        dy = (
            3.0 * u * u * (self.p1.y_mm - self.p0.y_mm)
            + 6.0 * u * t * (self.p2.y_mm - self.p1.y_mm)
            + 3.0 * t * t * (self.p3.y_mm - self.p2.y_mm)
        )
        return dx, dy

    def second_derivative(self, t: float) -> tuple[float, float]:
        u = 1.0 - t
        dx = 6.0 * u * (self.p2.x_mm - 2.0 * self.p1.x_mm + self.p0.x_mm) + 6.0 * t * (
            self.p3.x_mm - 2.0 * self.p2.x_mm + self.p1.x_mm
        )
        dy = 6.0 * u * (self.p2.y_mm - 2.0 * self.p1.y_mm + self.p0.y_mm) + 6.0 * t * (
            self.p3.y_mm - 2.0 * self.p2.y_mm + self.p1.y_mm
        )
        return dx, dy

    def curvature(self, t: float) -> float:
        dx, dy = self.derivative(t)
        ddx, ddy = self.second_derivative(t)
        speed_sq = dx * dx + dy * dy
        if speed_sq <= EPSILON:
            raise ValueError("Bezier derivative is degenerate")
        return (dx * ddy - dy * ddx) / (speed_sq ** 1.5)

    def tangent(self, t: float) -> tuple[float, float]:
        dx, dy = self.derivative(t)
        norm = math.hypot(dx, dy)
        if norm <= EPSILON:
            raise ValueError("Bezier derivative is degenerate")
        return dx / norm, dy / norm

    def to_control_points(self) -> tuple[dict[str, float], ...]:
        return (self.p0.to_dict(), self.p1.to_dict(), self.p2.to_dict(), self.p3.to_dict())


@dataclass(frozen=True)
class BezierPath:
    segments: tuple[CubicBezier, ...]

    @classmethod
    def from_waypoints(cls, points: Iterable[Point2D], *, tension: float = 1.0) -> "BezierPath":
        point_list = tuple(points)
        if len(point_list) < 2:
            raise ValueError("Bezier path requires at least two waypoints")
        _reject_duplicate_waypoints(point_list)
        # Cubic Hermite interpolation over cumulative chord length.  A single
        # physical derivative dP/ds is assigned to every interior waypoint,
        # while each segment's Bezier handle is scaled by that segment's chord
        # length.  This is C1 in the path parameter and avoids both failure
        # modes seen in uneven A* polylines: unrelated handles at a join and
        # handles that are far too short on a long neighboring segment.
        chord_lengths = tuple(
            math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm)
            for left, right in zip(point_list, point_list[1:])
        )
        slopes = _natural_spline_slopes(point_list, chord_lengths, tension=tension)
        segments: list[CubicBezier] = []
        for index, (left, right) in enumerate(zip(point_list, point_list[1:])):
            length = chord_lengths[index]
            left_local = (slopes[index][0] * length, slopes[index][1] * length)
            right_local = (slopes[index + 1][0] * length, slopes[index + 1][1] * length)
            segments.append(
                CubicBezier(
                    p0=left,
                    p1=Point2D(left.x_mm + left_local[0] / 3.0, left.y_mm + left_local[1] / 3.0),
                    p2=Point2D(right.x_mm - right_local[0] / 3.0, right.y_mm - right_local[1] / 3.0),
                    p3=right,
                )
            )
        return cls(segments=tuple(segments))

    def sample_arclength(self, *, max_spacing_mm: float = 25.0, oversample_per_segment: int = 48) -> tuple[PathSample2D, ...]:
        if max_spacing_mm <= 0.0:
            raise ValueError("max_spacing_mm must be positive")
        dense = self._dense_points(oversample_per_segment=max(8, oversample_per_segment))
        total = dense[-1][0]
        if total <= EPSILON:
            raise ValueError("Bezier path total length is zero")
        count = max(2, math.ceil(total / max_spacing_mm) + 1)
        targets = [total * index / (count - 1) for index in range(count)]
        return tuple(self._interpolate_dense(dense, target) for target in targets)

    def control_points_dicts(self) -> tuple[dict[str, float], ...]:
        points: list[dict[str, float]] = []
        for segment_index, segment in enumerate(self.segments):
            segment_points = segment.to_control_points()
            if segment_index:
                segment_points = segment_points[1:]
            points.extend(segment_points)
        return tuple(points)

    def _dense_points(self, *, oversample_per_segment: int) -> list[tuple[float, int, float, Point2D]]:
        dense: list[tuple[float, int, float, Point2D]] = []
        cumulative = 0.0
        previous: Point2D | None = None
        for segment_index, segment in enumerate(self.segments):
            for step in range(oversample_per_segment + 1):
                if segment_index and step == 0:
                    continue
                t = step / oversample_per_segment
                point = segment.evaluate(t)
                if previous is not None:
                    cumulative += math.hypot(point.x_mm - previous.x_mm, point.y_mm - previous.y_mm)
                dense.append((cumulative, segment_index, t, point))
                previous = point
        return dense

    def _interpolate_dense(self, dense: list[tuple[float, int, float, Point2D]], target_s: float) -> PathSample2D:
        if target_s <= 0.0:
            segment_index, t, point = dense[0][1], dense[0][2], dense[0][3]
            return self._sample_at(segment_index, t, 0.0, point)
        if target_s >= dense[-1][0]:
            segment_index, t, point = dense[-1][1], dense[-1][2], dense[-1][3]
            return self._sample_at(segment_index, t, dense[-1][0], point)
        for index in range(1, len(dense)):
            left = dense[index - 1]
            right = dense[index]
            if right[0] < target_s:
                continue
            span = max(right[0] - left[0], EPSILON)
            ratio = (target_s - left[0]) / span
            t = left[2] + (right[2] - left[2]) * ratio if left[1] == right[1] else right[2]
            segment_index = right[1] if left[1] != right[1] else left[1]
            point = self.segments[segment_index].evaluate(t)
            return self._sample_at(segment_index, t, target_s, point)
        segment_index, t, point = dense[-1][1], dense[-1][2], dense[-1][3]
        return self._sample_at(segment_index, t, dense[-1][0], point)

    def _sample_at(self, segment_index: int, t: float, s_mm: float, point: Point2D) -> PathSample2D:
        segment = self.segments[segment_index]
        tangent_x, tangent_y = segment.tangent(t)
        return PathSample2D(
            s_mm=s_mm,
            x_mm=point.x_mm,
            y_mm=point.y_mm,
            tangent_x=tangent_x,
            tangent_y=tangent_y,
            curvature_1_per_mm=segment.curvature(t),
        )


def point_from_dict(data: dict[str, object]) -> Point2D:
    return Point2D(x_mm=float(data["x_mm"]), y_mm=float(data["y_mm"]))



def _natural_spline_slopes(
    points: tuple[Point2D, ...],
    chord_lengths: tuple[float, ...],
    *,
    tension: float,
) -> tuple[tuple[float, float], ...]:
    """Solve non-uniform cubic-spline dP/ds values at every waypoint.

    With ``tension == 1`` the resulting piecewise cubic is C2 over cumulative
    chord length, so curvature is continuous at waypoint joins.  Smaller
    values remain supported for imported legacy drafts, but all current
    automatic/manual-template seeds use the C2 value 1.0.
    """

    if not 0.25 <= tension <= 1.25:
        raise ValueError("Bezier tension must be in [0.25, 1.25]")
    if any(length <= EPSILON for length in chord_lengths):
        raise ValueError("Bezier path contains a zero-length segment")
    count = len(points)
    if count == 2:
        length = chord_lengths[0]
        slope = (
            (points[1].x_mm - points[0].x_mm) / length * tension,
            (points[1].y_mm - points[0].y_mm) / length * tension,
        )
        return (slope, slope)

    lower = [0.0] * count
    diagonal = [0.0] * count
    upper = [0.0] * count
    rhs_x = [0.0] * count
    rhs_y = [0.0] * count

    first_dx = (points[1].x_mm - points[0].x_mm) / chord_lengths[0]
    first_dy = (points[1].y_mm - points[0].y_mm) / chord_lengths[0]
    diagonal[0] = 2.0
    upper[0] = 1.0
    rhs_x[0] = 3.0 * first_dx
    rhs_y[0] = 3.0 * first_dy

    for index in range(1, count - 1):
        previous_h = chord_lengths[index - 1]
        next_h = chord_lengths[index]
        previous_dx = (points[index].x_mm - points[index - 1].x_mm) / previous_h
        previous_dy = (points[index].y_mm - points[index - 1].y_mm) / previous_h
        next_dx = (points[index + 1].x_mm - points[index].x_mm) / next_h
        next_dy = (points[index + 1].y_mm - points[index].y_mm) / next_h
        lower[index] = next_h
        diagonal[index] = 2.0 * (previous_h + next_h)
        upper[index] = previous_h
        rhs_x[index] = 3.0 * (next_h * previous_dx + previous_h * next_dx)
        rhs_y[index] = 3.0 * (next_h * previous_dy + previous_h * next_dy)

    last_dx = (points[-1].x_mm - points[-2].x_mm) / chord_lengths[-1]
    last_dy = (points[-1].y_mm - points[-2].y_mm) / chord_lengths[-1]
    lower[-1] = 1.0
    diagonal[-1] = 2.0
    rhs_x[-1] = 3.0 * last_dx
    rhs_y[-1] = 3.0 * last_dy

    slopes_x = _solve_tridiagonal(lower, diagonal, upper, rhs_x)
    slopes_y = _solve_tridiagonal(lower, diagonal, upper, rhs_y)
    return tuple((x * tension, y * tension) for x, y in zip(slopes_x, slopes_y))


def _solve_tridiagonal(
    lower: list[float],
    diagonal: list[float],
    upper: list[float],
    rhs: list[float],
) -> tuple[float, ...]:
    """Thomas solve with explicit singularity checks."""

    count = len(diagonal)
    c_prime = [0.0] * count
    d_prime = [0.0] * count
    pivot = diagonal[0]
    if abs(pivot) <= EPSILON:
        raise ValueError("Bezier spline system is singular")
    c_prime[0] = upper[0] / pivot
    d_prime[0] = rhs[0] / pivot
    for index in range(1, count):
        pivot = diagonal[index] - lower[index] * c_prime[index - 1]
        if abs(pivot) <= EPSILON:
            raise ValueError("Bezier spline system is singular")
        c_prime[index] = upper[index] / pivot if index < count - 1 else 0.0
        d_prime[index] = (rhs[index] - lower[index] * d_prime[index - 1]) / pivot
    result = [0.0] * count
    result[-1] = d_prime[-1]
    for index in range(count - 2, -1, -1):
        result[index] = d_prime[index] - c_prime[index] * result[index + 1]
    return tuple(result)

def _waypoint_tangents(points: tuple[Point2D, ...]) -> tuple[tuple[float, float], ...]:
    tangents: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        if index == 0:
            dx = points[1].x_mm - point.x_mm
            dy = points[1].y_mm - point.y_mm
        elif index == len(points) - 1:
            dx = point.x_mm - points[index - 1].x_mm
            dy = point.y_mm - points[index - 1].y_mm
        else:
            dx = points[index + 1].x_mm - points[index - 1].x_mm
            dy = points[index + 1].y_mm - points[index - 1].y_mm
        norm = math.hypot(dx, dy)
        if norm <= EPSILON:
            raise ValueError("Bezier waypoint tangent is degenerate")
        tangents.append((dx / norm, dy / norm))
    return tuple(tangents)


def _reject_duplicate_waypoints(points: tuple[Point2D, ...]) -> None:
    for index, (left, right) in enumerate(zip(points, points[1:])):
        if math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm) <= EPSILON:
            raise ValueError(f"duplicate Bezier waypoint at segment {index}")
