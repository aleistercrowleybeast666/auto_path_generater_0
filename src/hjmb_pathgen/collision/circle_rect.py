"""Analytic circle to oriented-rectangle collision distance."""

from __future__ import annotations

import math
from dataclasses import dataclass

from hjmb_pathgen.collision.primitives import Circle, OrientedRect, Point2
from hjmb_pathgen.collision.transforms import body_to_world, world_to_body


@dataclass(frozen=True)
class CircleRectDistance:
    signed_clearance_mm: float
    closest_world: Point2
    feature: str


def circle_rect_signed_distance(circle: Circle, rect: OrientedRect) -> CircleRectDistance:
    local = world_to_body(circle.center, rect.center, rect.yaw_ddeg)
    clamped_x = max(-rect.half_length, min(rect.half_length, local.x))
    clamped_y = max(-rect.half_width, min(rect.half_width, local.y))
    closest_local = Point2(clamped_x, clamped_y)
    dx = local.x - clamped_x
    dy = local.y - clamped_y
    outside_distance = math.hypot(dx, dy)
    if outside_distance > 0.0:
        clearance = outside_distance - circle.radius_mm
        feature = _feature_for_clamped_point(clamped_x, clamped_y, rect)
        return CircleRectDistance(
            signed_clearance_mm=clearance,
            closest_world=body_to_world(closest_local, rect.center, rect.yaw_ddeg),
            feature=feature,
        )

    margins = {
        "RIGHT": rect.half_length - local.x,
        "LEFT": local.x + rect.half_length,
        "TOP": rect.half_width - local.y,
        "BOTTOM": local.y + rect.half_width,
    }
    feature = min(margins, key=margins.get)
    inside_clearance = -(circle.radius_mm + margins[feature])
    if feature == "RIGHT":
        closest_local = Point2(rect.half_length, local.y)
    elif feature == "LEFT":
        closest_local = Point2(-rect.half_length, local.y)
    elif feature == "TOP":
        closest_local = Point2(local.x, rect.half_width)
    else:
        closest_local = Point2(local.x, -rect.half_width)
    return CircleRectDistance(
        signed_clearance_mm=inside_clearance,
        closest_world=body_to_world(closest_local, rect.center, rect.yaw_ddeg),
        feature="INTERIOR",
    )


def _feature_for_clamped_point(x: float, y: float, rect: OrientedRect) -> str:
    on_x = abs(abs(x) - rect.half_length) <= 1.0e-9
    on_y = abs(abs(y) - rect.half_width) <= 1.0e-9
    if on_x and on_y:
        return "CORNER"
    return "EDGE"
