"""Robot footprint construction for V4.0 collision profiles."""

from __future__ import annotations

import math

from hjmb_pathgen.collision.primitives import Point2, require_positive
from hjmb_pathgen.collision.transforms import body_to_world


def validate_radii(r_large_mm: float, r_small_mm: float) -> tuple[float, float]:
    r_large = require_positive(r_large_mm, "r_large_mm")
    r_small = require_positive(r_small_mm, "r_small_mm")
    if r_small >= r_large:
        raise ValueError("r_large_mm must be greater than r_small_mm")
    return r_large, r_small


def clipped_disk_chord_half_height(r_large_mm: float, r_small_mm: float) -> float:
    r_large, r_small = validate_radii(r_large_mm, r_small_mm)
    return math.sqrt(max(0.0, r_large * r_large - r_small * r_small))


def clipped_disk_body_vertices(
    r_large_mm: float,
    r_small_mm: float,
    arc_segments: int,
) -> tuple[Point2, ...]:
    """Return CCW vertices for u^2+v^2<=R_large^2 and u<=R_small.

    The chord is on the robot front side at u=R_small. The circular arc uses
    the retained back/side portion of the large circle.
    """

    r_large, r_small = validate_radii(r_large_mm, r_small_mm)
    if arc_segments < 3:
        raise ValueError("arc_segments must be at least 3")
    theta = math.acos(r_small / r_large)
    start = theta
    end = (2.0 * math.pi) - theta
    vertices = []
    for index in range(arc_segments + 1):
        ratio = index / arc_segments
        angle = start + (end - start) * ratio
        u = r_large * math.cos(angle)
        v = r_large * math.sin(angle)
        vertices.append(Point2(u, v))
    return tuple(vertices)


def clipped_disk_world_vertices(
    origin: Point2,
    yaw_ddeg: float,
    r_large_mm: float,
    r_small_mm: float,
    arc_segments: int,
) -> tuple[Point2, ...]:
    return tuple(
        body_to_world(vertex, origin, yaw_ddeg)
        for vertex in clipped_disk_body_vertices(r_large_mm, r_small_mm, arc_segments)
    )


def is_convex_ccw(vertices: tuple[Point2, ...], *, tolerance: float = 1.0e-9) -> bool:
    if len(vertices) < 3:
        return False
    signs = []
    for index, point in enumerate(vertices):
        nxt = vertices[(index + 1) % len(vertices)]
        after = vertices[(index + 2) % len(vertices)]
        ax = nxt.x - point.x
        ay = nxt.y - point.y
        bx = after.x - nxt.x
        by = after.y - nxt.y
        cross = ax * by - ay * bx
        if abs(cross) > tolerance:
            signs.append(cross > 0.0)
    return bool(signs) and all(signs)
