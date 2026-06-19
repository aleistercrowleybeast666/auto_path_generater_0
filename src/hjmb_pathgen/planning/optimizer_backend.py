"""Small deterministic coordinate refinement backend for Phase 6."""

from __future__ import annotations

import math

from hjmb_pathgen.geometry.initial_guess import InitialGuess
from hjmb_pathgen.geometry.bezier import Point2D


def perturb_waypoints(guess: InitialGuess, *, pass_index: int, step_mm: float) -> tuple[InitialGuess, ...]:
    if len(guess.waypoints) <= 2 or step_mm <= 0.0:
        return ()
    variants: list[InitialGuess] = []
    for point_index in range(1, len(guess.waypoints) - 1):
        before = guess.waypoints[point_index - 1]
        after = guess.waypoints[point_index + 1]
        dx = after.x_mm - before.x_mm
        dy = after.y_mm - before.y_mm
        norm = math.hypot(dx, dy)
        if norm <= 1.0e-9:
            continue
        normal = (-dy / norm, dx / norm)
        for sign in (-1.0, 1.0):
            points = list(guess.waypoints)
            point = points[point_index]
            points[point_index] = Point2D(point.x_mm + normal[0] * step_mm * sign, point.y_mm + normal[1] * step_mm * sign)
            variants.append(
                InitialGuess(
                    guess_id=f"{guess.guess_id}_p{pass_index}_{point_index}_{'pos' if sign > 0 else 'neg'}",
                    source=f"{guess.source}_REFINE",
                    waypoints=tuple(points),
                )
            )
    return tuple(variants)
