"""Shared numerical collision semantics."""

from __future__ import annotations

from hjmb_pathgen.models.collision import ClearanceClass


def classify_clearance(clearance_mm: float, epsilon_mm: float) -> ClearanceClass:
    if epsilon_mm < 0.0:
        raise ValueError("epsilon_mm must be non-negative")
    if clearance_mm > epsilon_mm:
        return ClearanceClass.CLEAR
    if clearance_mm < -epsilon_mm:
        return ClearanceClass.PENETRATING
    return ClearanceClass.TOUCHING
