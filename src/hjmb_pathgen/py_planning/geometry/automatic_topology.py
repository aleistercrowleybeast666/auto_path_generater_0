"""Automatic S-topology profiles for FULL_AUTO transfer legs.

Only the cross-field transition from the last pickup arrival to the first drop
arrival is constrained by ordered virtual gates.  Local pickup/drop legs use no
virtual gates.  Existing projects may override the generated gates by placing a
non-empty ``gates`` list in their route-family topology profile.
"""

from __future__ import annotations

from typing import Any

from hjmb_pathgen.py_domain.project import ProjectV40

NO_GATE_PROFILE_ID = "LOCAL_NO_GATES"
TRANSFER_PROFILE_IDS = {
    "PICK_1_TO_3": "S_LEFT_TRANSFER",
    "PICK_3_TO_1": "S_RIGHT_TRANSFER",
}


def is_cross_field_transfer(from_state_id: str, to_state_id: str) -> bool:
    return from_state_id.startswith("P_PICK_") and to_state_id.startswith("DROP_STEP_")


def topology_profile_for_transition(
    project: ProjectV40,
    route_family: str,
    from_state_id: str,
    to_state_id: str,
) -> str:
    if not is_cross_field_transfer(from_state_id, to_state_id):
        return NO_GATE_PROFILE_ID
    route_profile = project.topology_profiles.get(route_family, {})
    if isinstance(route_profile, dict):
        configured = route_profile.get("transfer_profile_id")
        if configured:
            return str(configured)
    return TRANSFER_PROFILE_IDS.get(route_family, f"{route_family}_TRANSFER")


def topology_profile_object(
    project: ProjectV40,
    profile_id: str,
    *,
    route_family: str = "",
) -> dict[str, Any]:
    if profile_id == NO_GATE_PROFILE_ID:
        return {"profile_id": NO_GATE_PROFILE_ID, "gates": []}

    direct = project.topology_profiles.get(profile_id)
    if isinstance(direct, dict):
        return dict(direct)
    for value in project.topology_profiles.values():
        if isinstance(value, dict) and str(value.get("profile_id", "")) == profile_id:
            return dict(value)
        if isinstance(value, dict) and str(value.get("transfer_profile_id", "")) == profile_id:
            route_profile = dict(value)
            configured_gates = route_profile.get("gates")
            if isinstance(configured_gates, list) and configured_gates:
                return {**route_profile, "profile_id": profile_id}
            return {
                **route_profile,
                "profile_id": profile_id,
                "gates": list(default_transfer_gates(project, route_family)),
                "generated": True,
            }

    if profile_id in TRANSFER_PROFILE_IDS.values() or profile_id.endswith("_TRANSFER"):
        return {
            "profile_id": profile_id,
            "gates": list(default_transfer_gates(project, route_family)),
            "generated": True,
        }
    return {"profile_id": profile_id, "gates": []}


def default_transfer_gates(project: ProjectV40, route_family: str) -> tuple[dict[str, Any], ...]:
    """Build two ordered vertical gates that enforce the official S traversal.

    PICK_1_TO_3 ends at pickup 3 and is the user's *left* route: it passes
    below the pickup-side cylinder and above the drop-side cylinder.
    PICK_3_TO_1 ends at pickup 1 and is the user's *right* route: it passes
    above the pickup-side cylinder and below the drop-side cylinder.
    """

    cylinders = [
        dict(item)
        for item in project.field_objects.get("cylinders", [])
        if isinstance(item, dict) and item.get("enabled", True) and item.get("configured", True)
    ]
    if len(cylinders) < 2:
        return ()
    cylinders.sort(key=lambda item: float(item.get("center_x_mm", 0.0)), reverse=True)
    pickup_side, drop_side = cylinders[0], cylinders[-1]

    footprint = dict(project.vehicle.get("footprint", {}))
    r_large = float(footprint.get("r_large_mm", 120.0))
    route_profile = project.topology_profiles.get(route_family, {})
    route_profile = dict(route_profile) if isinstance(route_profile, dict) else {}
    extra = float(route_profile.get("gate_clearance_mm", 65.0))
    minimum_half_gap = max(
        float(pickup_side.get("radius_mm", 51.0)),
        float(drop_side.get("radius_mm", 51.0)),
    ) + r_large + extra

    boundary = dict(project.field_objects.get("field_boundary", {}))
    y_min = float(boundary.get("y_min_mm", -1000.0)) + r_large + 20.0
    y_max = float(boundary.get("y_max_mm", 1000.0)) - r_large - 20.0
    if y_min >= -minimum_half_gap or y_max <= minimum_half_gap:
        return ()

    def lower_gate(gate_id: str, x_mm: float, center_y: float) -> dict[str, Any]:
        upper = min(center_y - minimum_half_gap, -minimum_half_gap)
        return {
            "gate_id": gate_id,
            "a": {"x_mm": x_mm, "y_mm": y_min},
            "b": {"x_mm": x_mm, "y_mm": upper},
            "direction": "POSITIVE",
        }

    def upper_gate(gate_id: str, x_mm: float, center_y: float) -> dict[str, Any]:
        lower = max(center_y + minimum_half_gap, minimum_half_gap)
        return {
            "gate_id": gate_id,
            "a": {"x_mm": x_mm, "y_mm": lower},
            "b": {"x_mm": x_mm, "y_mm": y_max},
            "direction": "POSITIVE",
        }

    px = float(pickup_side.get("center_x_mm", 1000.0))
    py = float(pickup_side.get("center_y_mm", 0.0))
    dx = float(drop_side.get("center_x_mm", -1000.0))
    dy = float(drop_side.get("center_y_mm", 0.0))
    if route_family == "PICK_1_TO_3":
        return (
            lower_gate("S_PICKUP_SIDE_LOWER", px, py),
            upper_gate("S_DROP_SIDE_UPPER", dx, dy),
        )
    return (
        upper_gate("S_PICKUP_SIDE_UPPER", px, py),
        lower_gate("S_DROP_SIDE_LOWER", dx, dy),
    )
