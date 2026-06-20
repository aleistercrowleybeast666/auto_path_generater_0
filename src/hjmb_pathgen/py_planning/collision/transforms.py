"""Coordinate transforms for V4.0 world/body collision geometry."""

from __future__ import annotations

import math

from hjmb_pathgen.py_planning.collision.primitives import OrientedRect, Point2

DDEG_TO_RAD = math.pi / 1800.0
RAD_TO_DDEG = 1800.0 / math.pi


def rotate(point: Point2, yaw_ddeg: float) -> Point2:
    yaw_rad = yaw_ddeg * DDEG_TO_RAD
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return Point2(c * point.x - s * point.y, s * point.x + c * point.y)


def body_to_world(body_point: Point2, origin: Point2, yaw_ddeg: float) -> Point2:
    rotated = rotate(body_point, yaw_ddeg)
    return Point2(origin.x + rotated.x, origin.y + rotated.y)


def world_to_body(world_point: Point2, origin: Point2, yaw_ddeg: float) -> Point2:
    dx = world_point.x - origin.x
    dy = world_point.y - origin.y
    yaw_rad = yaw_ddeg * DDEG_TO_RAD
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return Point2(c * dx + s * dy, -s * dx + c * dy)


def rect_vertices(rect: OrientedRect) -> tuple[Point2, Point2, Point2, Point2]:
    corners = (
        Point2(-rect.half_length, -rect.half_width),
        Point2(rect.half_length, -rect.half_width),
        Point2(rect.half_length, rect.half_width),
        Point2(-rect.half_length, rect.half_width),
    )
    return tuple(body_to_world(corner, rect.center, rect.yaw_ddeg) for corner in corners)  # type: ignore[return-value]
