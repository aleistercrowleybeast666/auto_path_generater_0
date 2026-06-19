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
    def from_waypoints(cls, points: Iterable[Point2D], *, tension: float = 0.75) -> "BezierPath":
        point_list = tuple(points)
        if len(point_list) < 2:
            raise ValueError("Bezier path requires at least two waypoints")
        _reject_duplicate_waypoints(point_list)
        tangents = _waypoint_tangents(point_list)
        segments: list[CubicBezier] = []
        for index, (left, right) in enumerate(zip(point_list, point_list[1:])):
            length = math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm)
            if length <= EPSILON:
                raise ValueError("Bezier path contains a zero-length segment")
            handle = length * tension / 3.0
            left_tangent = tangents[index]
            right_tangent = tangents[index + 1]
            segments.append(
                CubicBezier(
                    p0=left,
                    p1=Point2D(left.x_mm + left_tangent[0] * handle, left.y_mm + left_tangent[1] * handle),
                    p2=Point2D(right.x_mm - right_tangent[0] * handle, right.y_mm - right_tangent[1] * handle),
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
