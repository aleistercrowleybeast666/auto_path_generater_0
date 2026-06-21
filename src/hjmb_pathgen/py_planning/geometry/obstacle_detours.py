"""Deterministic obstacle-aware waypoint seeds for automatic leg planning.

The optimizer still performs the authoritative continuous collision validation.
This module only supplies useful initial geometry.  It intentionally uses a
slightly conservative centre-point model so a route that reaches the expensive
validator has a realistic chance of being valid instead of trying only a
straight line.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Any, Iterable

from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationRequest
from hjmb_pathgen.py_planning.geometry.bezier import Point2D


@dataclass(frozen=True)
class DetourSeed:
    seed_id: str
    source: str
    waypoints: tuple[Point2D, ...]
    tension: float = 0.24


@dataclass(frozen=True)
class _Rect:
    obstacle_id: str
    cx: float
    cy: float
    half_length: float
    half_width: float
    yaw_rad: float
    inflate: float


@dataclass(frozen=True)
class _Circle:
    obstacle_id: str
    cx: float
    cy: float
    radius: float


def obstacle_aware_seeds(request: LegOptimizationRequest) -> tuple[DetourSeed, ...]:
    """Return prioritized deterministic detours for a directed transition."""

    start = Point2D(request.from_pose.x_mm, request.from_pose.y_mm)
    finish = Point2D(request.to_pose.x_mm, request.to_pose.y_mm)
    seeds: list[DetourSeed] = []

    state_seed = _state_lane_seed(request, start, finish)
    if state_seed is not None:
        seeds.append(state_seed)

    s_seed = _s_route_seed(request, start, finish)
    if s_seed is not None:
        seeds.append(s_seed)
        # A pickup-to-drop transfer with ordered virtual gates has exactly one
        # legal route family.  Generic A* seeds do not preserve that ordered
        # S topology and are expensive to validate, so they must not displace
        # the official gate seed from AUTOMATIC's small initial-guess budget.
        if request.topology_gates:
            return _deduplicate(seeds)

    for margin_index, margin in enumerate((30.0, 65.0)):
        grid = _grid_detour(request, start, finish, safety_margin_mm=margin)
        if grid is not None:
            seeds.append(
                DetourSeed(
                    seed_id=f"grid_detour_{margin_index}",
                    source="OBSTACLE_ASTAR",
                    waypoints=grid,
                    tension=0.20 if margin_index == 0 else 0.16,
                )
            )

    return _deduplicate(seeds)


def _state_lane_seed(
    request: LegOptimizationRequest,
    start: Point2D,
    finish: Point2D,
) -> DetourSeed | None:
    from_id = request.from_state_id
    to_id = request.to_state_id

    # The middle pickup box is approached from two sides.  A left-side aisle
    # gives the planner a valid seed for 2L->3 and 2R->1 instead of cutting
    # through PICKUP_BOX_2.
    if from_id.startswith("P_PICK_") and to_id.startswith("P_PICK_"):
        aisle_x = min(start.x_mm, finish.x_mm, 1120.0)
        points = _clean_points(
            (
                start,
                Point2D(aisle_x, start.y_mm),
                Point2D(aisle_x, finish.y_mm),
                finish,
            )
        )
        if len(points) >= 3:
            return DetourSeed("pickup_left_aisle", "STATE_SAFE_AISLE", points, tension=0.20)

    # All five drop boxes are served from the open aisle on their right.  The
    # two staging points keep the cubic curve from rounding through a box.
    if from_id.startswith("DROP_STEP_") and to_id.startswith("DROP_STEP_"):
        aisle_x = max(start.x_mm, finish.x_mm, -1200.0)
        points = _clean_points(
            (
                start,
                Point2D(aisle_x, start.y_mm),
                Point2D(aisle_x, finish.y_mm),
                finish,
            )
        )
        if len(points) >= 3:
            return DetourSeed("drop_right_aisle", "STATE_SAFE_AISLE", points, tension=0.16)
    return None


def _s_route_seed(
    request: LegOptimizationRequest,
    start: Point2D,
    finish: Point2D,
) -> DetourSeed | None:
    """Seed the official two-cylinder S traversal on pickup-to-drop legs."""

    if not request.from_state_id.startswith("P_PICK_"):
        return None
    if not request.to_state_id.startswith("DROP_STEP_"):
        return None

    if request.topology_gates:
        # The ordered gate centres define the official S route.  Use exactly
        # one through point per gate, plus one collinear staging point at each
        # stop endpoint.  The previous three-points-per-gate seed introduced
        # unnecessary curvature reversals; the time parameterizer then found
        # adjacent zero-speed samples and rejected otherwise collision-free
        # FULL_AUTO transfers.
        gate_points = [Point2D(*gate.center) for gate in request.topology_gates]
        first = gate_points[0]
        last = gate_points[-1]
        start_mid = Point2D(
            (start.x_mm + first.x_mm) * 0.5,
            (start.y_mm + first.y_mm) * 0.5,
        )
        finish_mid = Point2D(
            (last.x_mm + finish.x_mm) * 0.5,
            (last.y_mm + finish.y_mm) * 0.5,
        )
        points = _clean_points((start, start_mid, *gate_points, finish_mid, finish))
        return DetourSeed("official_s_gate_seed", "TOPOLOGY_GATE_S", points, tension=0.18)

    route_family = str(request.route_family)
    sign = -1.0 if route_family == "PICK_1_TO_3" else 1.0
    points = _clean_points(
        (
            start,
            Point2D(1220.0, 320.0 * sign),
            Point2D(760.0, 320.0 * sign),
            Point2D(0.0, 0.0),
            Point2D(-760.0, -320.0 * sign),
            Point2D(-1220.0, -320.0 * sign),
            finish,
        )
    )
    return DetourSeed("official_s_seed", "ROUTE_FAMILY_S", points, tension=0.18)


def _grid_detour(
    request: LegOptimizationRequest,
    start: Point2D,
    finish: Point2D,
    *,
    safety_margin_mm: float,
) -> tuple[Point2D, ...] | None:
    project = request.project
    footprint = dict(project.vehicle.get("footprint", {}))
    r_large = float(footprint.get("r_large_mm", 120.0))
    r_small = float(footprint.get("r_small_mm", 70.0))
    field = dict(project.field_objects.get("field_boundary", {}))
    x_min = float(field.get("x_min_mm", -2000.0)) + r_large + safety_margin_mm
    x_max = float(field.get("x_max_mm", 2000.0)) - r_large - safety_margin_mm
    y_min = float(field.get("y_min_mm", -1000.0)) + r_large + safety_margin_mm
    y_max = float(field.get("y_max_mm", 1000.0)) - r_large - safety_margin_mm
    if x_min >= x_max or y_min >= y_max:
        return None

    rects, circles = _planning_obstacles(request, r_large, r_small, safety_margin_mm)
    start_escape = _escape_point(request.from_state_id, start, rects, safety_margin_mm)
    finish_escape = _escape_point(request.to_state_id, finish, rects, safety_margin_mm)
    start_escape = _clamp(start_escape, x_min, x_max, y_min, y_max)
    finish_escape = _clamp(finish_escape, x_min, x_max, y_min, y_max)

    step = 70.0
    nx = max(2, int(math.floor((x_max - x_min) / step)) + 1)
    ny = max(2, int(math.floor((y_max - y_min) / step)) + 1)

    def point_of(node: tuple[int, int]) -> Point2D:
        ix, iy = node
        return Point2D(x_min + ix * step, y_min + iy * step)

    blocked_cache: dict[tuple[int, int], bool] = {}

    def blocked(node: tuple[int, int]) -> bool:
        if node in blocked_cache:
            return blocked_cache[node]
        value = _point_blocked(point_of(node), rects, circles)
        blocked_cache[node] = value
        return value

    start_node = _nearest_free_node(start_escape, x_min, y_min, step, nx, ny, blocked)
    finish_node = _nearest_free_node(finish_escape, x_min, y_min, step, nx, ny, blocked)
    if start_node is None or finish_node is None:
        return None
    nodes = _astar(start_node, finish_node, nx, ny, blocked, point_of)
    if not nodes:
        return None
    raw = [point_of(node) for node in nodes]
    raw[0] = start_escape
    raw[-1] = finish_escape
    simplified = _simplify(raw, rects, circles)

    staged: list[Point2D] = [start]
    _append_endpoint_staging(staged, start, start_escape)
    for point in simplified:
        _append_unique(staged, point)
    finish_staging: list[Point2D] = []
    _append_endpoint_staging(finish_staging, finish, finish_escape, reverse=True)
    for point in finish_staging:
        _append_unique(staged, point)
    _append_unique(staged, finish)
    result = _clean_points(staged)
    return result if len(result) >= 2 else None


def _planning_obstacles(
    request: LegOptimizationRequest,
    r_large: float,
    r_small: float,
    margin: float,
) -> tuple[tuple[_Rect, ...], tuple[_Circle, ...]]:
    objects = request.project.field_objects
    rects: list[_Rect] = []
    circles: list[_Circle] = []
    for raw in objects.get("cylinders", []):
        if not raw.get("enabled", True) or not raw.get("configured", True):
            continue
        circles.append(
            _Circle(
                obstacle_id=str(raw.get("obstacle_id", "CYLINDER")),
                cx=float(raw["center_x_mm"]),
                cy=float(raw["center_y_mm"]),
                radius=float(raw["radius_mm"]) + r_large + margin,
            )
        )
    for collection, inflate in ((objects.get("pickup_boxes", []), r_large), (objects.get("drop_boxes", []), r_small)):
        for raw in collection:
            if not raw.get("enabled", True) or not raw.get("configured", True):
                continue
            rects.append(
                _Rect(
                    obstacle_id=str(raw.get("obstacle_id", "BOX")),
                    cx=float(raw["center_x_mm"]),
                    cy=float(raw["center_y_mm"]),
                    half_length=float(raw["length_mm"]) * 0.5,
                    half_width=float(raw["width_mm"]) * 0.5,
                    yaw_rad=math.radians(float(raw.get("yaw_ddeg", 0.0)) / 10.0),
                    inflate=inflate + margin,
                )
            )
    return tuple(rects), tuple(circles)


def _escape_point(
    state_id: str,
    endpoint: Point2D,
    rects: tuple[_Rect, ...],
    margin: float,
) -> Point2D:
    candidate = _associated_rect(state_id, endpoint, rects)
    if candidate is None:
        return endpoint
    local_x, local_y = _to_local(endpoint, candidate)
    bound_x = candidate.half_length + candidate.inflate + 45.0
    bound_y = candidate.half_width + candidate.inflate + 45.0
    nx = abs(local_x) / max(bound_x, 1.0)
    ny = abs(local_y) / max(bound_y, 1.0)
    if nx >= ny:
        local_x = math.copysign(bound_x, local_x if abs(local_x) > 1.0e-9 else -1.0)
    else:
        local_y = math.copysign(bound_y, local_y if abs(local_y) > 1.0e-9 else 1.0)
    return _to_world(local_x, local_y, candidate)


def _associated_rect(state_id: str, endpoint: Point2D, rects: tuple[_Rect, ...]) -> _Rect | None:
    expected: str | None = None
    if state_id == "P_PICK_1":
        expected = "PICKUP_BOX_1"
    elif state_id in {"P_PICK_2L", "P_PICK_2R"}:
        expected = "PICKUP_BOX_2"
    elif state_id == "P_PICK_3":
        expected = "PICKUP_BOX_3"
    if expected is not None:
        return next((item for item in rects if item.obstacle_id == expected), None)
    if state_id.startswith("DROP_STEP_"):
        drops = [item for item in rects if item.obstacle_id.startswith("DROP_BOX_")]
        return min(drops, key=lambda item: math.hypot(endpoint.x_mm - item.cx, endpoint.y_mm - item.cy), default=None)
    return None


def _astar(
    start: tuple[int, int],
    goal: tuple[int, int],
    nx: int,
    ny: int,
    blocked: Any,
    point_of: Any,
) -> list[tuple[int, int]]:
    frontier: list[tuple[float, float, tuple[int, int]]] = []
    heapq.heappush(frontier, (0.0, 0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    while frontier:
        _priority, current_cost, current = heapq.heappop(frontier)
        if current == goal:
            break
        if current_cost > cost_so_far.get(current, math.inf) + 1.0e-9:
            continue
        for dx, dy in directions:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny) or blocked(nxt):
                continue
            step_cost = math.sqrt(2.0) if dx and dy else 1.0
            new_cost = current_cost + step_cost
            if new_cost + 1.0e-9 >= cost_so_far.get(nxt, math.inf):
                continue
            cost_so_far[nxt] = new_cost
            came_from[nxt] = current
            p = point_of(nxt)
            q = point_of(goal)
            heuristic = math.hypot(p.x_mm - q.x_mm, p.y_mm - q.y_mm) / 70.0
            heapq.heappush(frontier, (new_cost + heuristic, new_cost, nxt))
    if goal not in came_from:
        return []
    result: list[tuple[int, int]] = []
    node: tuple[int, int] | None = goal
    while node is not None:
        result.append(node)
        node = came_from[node]
    result.reverse()
    return result


def _nearest_free_node(
    point: Point2D,
    x_min: float,
    y_min: float,
    step: float,
    nx: int,
    ny: int,
    blocked: Any,
) -> tuple[int, int] | None:
    base = (
        min(nx - 1, max(0, round((point.x_mm - x_min) / step))),
        min(ny - 1, max(0, round((point.y_mm - y_min) / step))),
    )
    if not blocked(base):
        return base
    for radius in range(1, 7):
        choices: list[tuple[float, tuple[int, int]]] = []
        for ix in range(max(0, base[0] - radius), min(nx, base[0] + radius + 1)):
            for iy in range(max(0, base[1] - radius), min(ny, base[1] + radius + 1)):
                if max(abs(ix - base[0]), abs(iy - base[1])) != radius:
                    continue
                node = (ix, iy)
                if not blocked(node):
                    choices.append((math.hypot(ix - base[0], iy - base[1]), node))
        if choices:
            choices.sort()
            return choices[0][1]
    return None


def _simplify(points: list[Point2D], rects: tuple[_Rect, ...], circles: tuple[_Circle, ...]) -> list[Point2D]:
    if len(points) <= 2:
        return points
    result = [points[0]]
    index = 0
    while index < len(points) - 1:
        candidate = len(points) - 1
        while candidate > index + 1 and not _line_clear(points[index], points[candidate], rects, circles):
            candidate -= 1
        result.append(points[candidate])
        index = candidate
    return result


def _line_clear(a: Point2D, b: Point2D, rects: tuple[_Rect, ...], circles: tuple[_Circle, ...]) -> bool:
    distance = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
    count = max(2, int(math.ceil(distance / 25.0)) + 1)
    for index in range(count):
        ratio = index / (count - 1)
        point = Point2D(a.x_mm + (b.x_mm - a.x_mm) * ratio, a.y_mm + (b.y_mm - a.y_mm) * ratio)
        if _point_blocked(point, rects, circles):
            return False
    return True


def _point_blocked(point: Point2D, rects: tuple[_Rect, ...], circles: tuple[_Circle, ...]) -> bool:
    for circle in circles:
        if math.hypot(point.x_mm - circle.cx, point.y_mm - circle.cy) < circle.radius:
            return True
    for rect in rects:
        x, y = _to_local(point, rect)
        if abs(x) < rect.half_length + rect.inflate and abs(y) < rect.half_width + rect.inflate:
            return True
    return False


def _to_local(point: Point2D, rect: _Rect) -> tuple[float, float]:
    dx = point.x_mm - rect.cx
    dy = point.y_mm - rect.cy
    c = math.cos(rect.yaw_rad)
    s = math.sin(rect.yaw_rad)
    return c * dx + s * dy, -s * dx + c * dy


def _to_world(x: float, y: float, rect: _Rect) -> Point2D:
    c = math.cos(rect.yaw_rad)
    s = math.sin(rect.yaw_rad)
    return Point2D(rect.cx + c * x - s * y, rect.cy + s * x + c * y)


def _append_endpoint_staging(
    target: list[Point2D],
    endpoint: Point2D,
    escape: Point2D,
    *,
    reverse: bool = False,
) -> None:
    distance = math.hypot(escape.x_mm - endpoint.x_mm, escape.y_mm - endpoint.y_mm)
    if distance <= 5.0:
        if reverse:
            _append_unique(target, escape)
        return
    midpoint = Point2D((endpoint.x_mm + escape.x_mm) * 0.5, (endpoint.y_mm + escape.y_mm) * 0.5)
    order = (escape, midpoint) if reverse else (midpoint, escape)
    for point in order:
        _append_unique(target, point)


def _clamp(point: Point2D, x_min: float, x_max: float, y_min: float, y_max: float) -> Point2D:
    return Point2D(min(x_max, max(x_min, point.x_mm)), min(y_max, max(y_min, point.y_mm)))


def _append_unique(points: list[Point2D], point: Point2D) -> None:
    if not points or math.hypot(points[-1].x_mm - point.x_mm, points[-1].y_mm - point.y_mm) > 1.0:
        points.append(point)


def _clean_points(points: Iterable[Point2D]) -> tuple[Point2D, ...]:
    result: list[Point2D] = []
    for point in points:
        _append_unique(result, point)
    return tuple(result)


def _deduplicate(seeds: list[DetourSeed]) -> tuple[DetourSeed, ...]:
    seen: set[tuple[tuple[int, int], ...]] = set()
    result: list[DetourSeed] = []
    for seed in seeds:
        key = tuple((round(item.x_mm), round(item.y_mm)) for item in seed.waypoints)
        if len(key) < 2 or key in seen:
            continue
        seen.add(key)
        result.append(seed)
    return tuple(result)
