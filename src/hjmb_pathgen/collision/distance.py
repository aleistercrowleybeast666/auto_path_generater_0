"""Distance helpers for deterministic 2D collision diagnostics."""

from __future__ import annotations

import math

from hjmb_pathgen.collision.primitives import Point2, Segment2


def dot(a: Point2, b: Point2) -> float:
    return a.x * b.x + a.y * b.y


def subtract(a: Point2, b: Point2) -> Point2:
    return Point2(a.x - b.x, a.y - b.y)


def point_segment_distance(point: Point2, segment: Segment2) -> float:
    ab = subtract(segment.b, segment.a)
    ap = subtract(point, segment.a)
    denom = dot(ab, ab)
    if denom <= 0.0:
        return math.hypot(point.x - segment.a.x, point.y - segment.a.y)
    t = max(0.0, min(1.0, dot(ap, ab) / denom))
    closest = Point2(segment.a.x + ab.x * t, segment.a.y + ab.y * t)
    return math.hypot(point.x - closest.x, point.y - closest.y)


def segments_intersect(a: Segment2, b: Segment2, *, epsilon: float = 0.0) -> bool:
    def orient(p: Point2, q: Point2, r: Point2) -> float:
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)

    def on_segment(p: Point2, q: Point2, r: Point2) -> bool:
        return (
            min(p.x, r.x) - epsilon <= q.x <= max(p.x, r.x) + epsilon
            and min(p.y, r.y) - epsilon <= q.y <= max(p.y, r.y) + epsilon
        )

    o1 = orient(a.a, a.b, b.a)
    o2 = orient(a.a, a.b, b.b)
    o3 = orient(b.a, b.b, a.a)
    o4 = orient(b.a, b.b, a.b)
    if o1 * o2 < -epsilon and o3 * o4 < -epsilon:
        return True
    if abs(o1) <= epsilon and on_segment(a.a, b.a, a.b):
        return True
    if abs(o2) <= epsilon and on_segment(a.a, b.b, a.b):
        return True
    if abs(o3) <= epsilon and on_segment(b.a, a.a, b.b):
        return True
    if abs(o4) <= epsilon and on_segment(b.a, a.b, b.b):
        return True
    return False


def segment_segment_distance(a: Segment2, b: Segment2, *, epsilon: float = 0.0) -> float:
    if segments_intersect(a, b, epsilon=epsilon):
        return 0.0
    return min(
        point_segment_distance(a.a, b),
        point_segment_distance(a.b, b),
        point_segment_distance(b.a, a),
        point_segment_distance(b.b, a),
    )


def polygon_edges(vertices: tuple[Point2, ...]) -> tuple[Segment2, ...]:
    return tuple(Segment2(vertices[index], vertices[(index + 1) % len(vertices)]) for index in range(len(vertices)))


def polygon_distance(a: tuple[Point2, ...], b: tuple[Point2, ...], *, epsilon: float = 0.0) -> float:
    best = math.inf
    a_len = len(a)
    b_len = len(b)
    for i in range(a_len):
        ax1, ay1 = a[i].x, a[i].y
        ax2, ay2 = a[(i + 1) % a_len].x, a[(i + 1) % a_len].y
        for j in range(b_len):
            bx1, by1 = b[j].x, b[j].y
            bx2, by2 = b[(j + 1) % b_len].x, b[(j + 1) % b_len].y
            distance = _segment_segment_distance_xy(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, epsilon)
            if distance <= epsilon:
                return 0.0
            best = min(best, distance)
    return best


def _segment_segment_distance_xy(
    ax1: float,
    ay1: float,
    ax2: float,
    ay2: float,
    bx1: float,
    by1: float,
    bx2: float,
    by2: float,
    epsilon: float,
) -> float:
    if _segments_intersect_xy(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, epsilon):
        return 0.0
    return min(
        _point_segment_distance_xy(ax1, ay1, bx1, by1, bx2, by2),
        _point_segment_distance_xy(ax2, ay2, bx1, by1, bx2, by2),
        _point_segment_distance_xy(bx1, by1, ax1, ay1, ax2, ay2),
        _point_segment_distance_xy(bx2, by2, ax1, ay1, ax2, ay2),
    )


def _point_segment_distance_xy(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    closest_x = ax + dx * t
    closest_y = ay + dy * t
    return math.hypot(px - closest_x, py - closest_y)


def _segments_intersect_xy(
    ax1: float,
    ay1: float,
    ax2: float,
    ay2: float,
    bx1: float,
    by1: float,
    bx2: float,
    by2: float,
    epsilon: float,
) -> bool:
    def orient(px: float, py: float, qx: float, qy: float, rx: float, ry: float) -> float:
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    def on_segment(px: float, py: float, qx: float, qy: float, rx: float, ry: float) -> bool:
        return (
            min(px, rx) - epsilon <= qx <= max(px, rx) + epsilon
            and min(py, ry) - epsilon <= qy <= max(py, ry) + epsilon
        )

    o1 = orient(ax1, ay1, ax2, ay2, bx1, by1)
    o2 = orient(ax1, ay1, ax2, ay2, bx2, by2)
    o3 = orient(bx1, by1, bx2, by2, ax1, ay1)
    o4 = orient(bx1, by1, bx2, by2, ax2, ay2)
    if o1 * o2 < -epsilon and o3 * o4 < -epsilon:
        return True
    if abs(o1) <= epsilon and on_segment(ax1, ay1, bx1, by1, ax2, ay2):
        return True
    if abs(o2) <= epsilon and on_segment(ax1, ay1, bx2, by2, ax2, ay2):
        return True
    if abs(o3) <= epsilon and on_segment(bx1, by1, ax1, ay1, bx2, by2):
        return True
    if abs(o4) <= epsilon and on_segment(bx1, by1, ax2, ay2, bx2, by2):
        return True
    return False
