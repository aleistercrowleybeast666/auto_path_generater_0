"""Small geometry primitives for V4.0 collision checks."""

from __future__ import annotations

import math
from dataclasses import dataclass


def require_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def require_positive(value: float, name: str) -> float:
    result = require_finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


@dataclass(frozen=True)
class Point2:
    x: float
    y: float

    def __post_init__(self) -> None:
        require_finite(self.x, "point.x")
        require_finite(self.y, "point.y")

    def to_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass(frozen=True)
class Circle:
    center: Point2
    radius_mm: float

    def __post_init__(self) -> None:
        require_positive(self.radius_mm, "circle.radius_mm")


@dataclass(frozen=True)
class OrientedRect:
    center: Point2
    length_mm: float
    width_mm: float
    yaw_ddeg: float = 0.0

    def __post_init__(self) -> None:
        require_positive(self.length_mm, "rect.length_mm")
        require_positive(self.width_mm, "rect.width_mm")
        require_finite(self.yaw_ddeg, "rect.yaw_ddeg")

    @property
    def half_length(self) -> float:
        return self.length_mm * 0.5

    @property
    def half_width(self) -> float:
        return self.width_mm * 0.5


@dataclass(frozen=True)
class Segment2:
    a: Point2
    b: Point2

    @property
    def length(self) -> float:
        return math.hypot(self.b.x - self.a.x, self.b.y - self.a.y)
