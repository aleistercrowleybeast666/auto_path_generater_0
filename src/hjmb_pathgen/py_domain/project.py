"""V4.0 project JSON model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.py_io.codecs.legacy_rejection import reject_deleted_fields, reject_legacy_format

from .errors import V40ValidationError, expect_equal, reject_unknown_fields, require_fields
from .protocol import (
    DROP_SITE_KEYS,
    NOMINAL_FIELD_LENGTH_MM,
    NOMINAL_FIELD_WIDTH_MM,
    PICKUP_SITE_KEYS,
    PROJECT_FORMAT,
    REQUIRED_SITE_KEYS,
    REQUIRED_UNLOAD_PROFILE_KEYS,
    YAW_UNSPECIFIED_DDEG,
)

PROJECT_REQUIRED_FIELDS = {
    "format",
    "project_id",
    "protocol_version",
    "nominal_field",
    "coordinate_system",
    "site_pose_provider",
    "sites",
    "field_objects",
    "vehicle",
    "dynamics",
    "unload_profiles",
    "topology_profiles",
    "action_profiles",
    "planner_profiles",
    "start_check",
    "arrival_check",
    "finish_policy",
    "output",
    "traj_table",
}
COMMON_SITE_FIELDS = {"configured", "x_mm", "y_mm", "yaw_ddeg"}
UNLOAD_PROFILE_FIELDS = {"configured", "yaw_ddeg", "dx_mm", "dy_mm", "estimated_action_time_ms"}
FOOTPRINT_REQUIRED_FIELDS = {
    "r_large_mm",
    "r_small_mm",
    "collision_resolution_mm",
    "strict_validation_resolution_mm",
    "numerical_epsilon_mm",
    "pickup_arc_segments",
    "field_boundary_footprint_profile",
}
FIELD_OBJECT_FIELDS = {"cylinders", "pickup_boxes", "drop_boxes", "field_boundary"}
CYLINDER_FIELDS = {"obstacle_id", "center_x_mm", "center_y_mm", "radius_mm", "configured", "enabled"}
PICKUP_BOX_FIELDS = {"obstacle_id", "physical_pick_site", "center_x_mm", "center_y_mm", "length_mm", "width_mm", "yaw_ddeg", "configured", "enabled"}
DROP_BOX_FIELDS = {"obstacle_id", "physical_drop_site", "center_x_mm", "center_y_mm", "length_mm", "width_mm", "yaw_ddeg", "configured", "enabled"}
FIELD_BOUNDARY_FIELDS = {"enabled", "x_min_mm", "x_max_mm", "y_min_mm", "y_max_mm", "footprint_profile"}
PHYSICAL_PICK_SITES = ("PICK_1", "PICK_2", "PICK_3")


@dataclass(frozen=True)
class ProjectV40:
    project_id: str
    nominal_field: dict[str, Any]
    coordinate_system: dict[str, Any]
    site_pose_provider: dict[str, Any]
    sites: dict[str, Any]
    field_objects: dict[str, Any]
    vehicle: dict[str, Any]
    dynamics: dict[str, Any]
    unload_profiles: dict[str, Any]
    topology_profiles: dict[str, Any]
    action_profiles: dict[str, Any]
    planner_profiles: dict[str, Any]
    start_check: dict[str, Any]
    arrival_check: dict[str, Any]
    finish_policy: dict[str, Any]
    output: dict[str, Any]
    traj_table: dict[str, Any]
    protocol_version: int = 40
    format: str = PROJECT_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ProjectV40":
        reject_deleted_fields(data, "ProjectV40")
        reject_legacy_format(data.get("format"), "ProjectV40")
        reject_unknown_fields(data, PROJECT_REQUIRED_FIELDS, "ProjectV40")
        require_fields(data, PROJECT_REQUIRED_FIELDS, "ProjectV40")
        expect_equal(data["format"], PROJECT_FORMAT, "ProjectV40", "format")
        expect_equal(data["protocol_version"], 40, "ProjectV40", "protocol_version")
        nominal = dict(data["nominal_field"])
        expect_equal(nominal.get("length_mm"), NOMINAL_FIELD_LENGTH_MM, "ProjectV40", "nominal_field.length_mm")
        expect_equal(nominal.get("width_mm"), NOMINAL_FIELD_WIDTH_MM, "ProjectV40", "nominal_field.width_mm")
        provider = dict(data["site_pose_provider"])
        expect_equal(provider.get("type"), "MANUAL", "ProjectV40", "site_pose_provider.type")
        sites = validate_project_sites(data["sites"], "ProjectV40", "sites")
        unload_profiles = validate_unload_profiles(data["unload_profiles"], "ProjectV40", "unload_profiles")
        field_objects = validate_project_field_objects(data["field_objects"], "ProjectV40", "field_objects")
        vehicle = validate_project_vehicle(data["vehicle"], "ProjectV40", "vehicle")
        return cls(
            project_id=str(data["project_id"]),
            protocol_version=int(data["protocol_version"]),
            nominal_field=nominal,
            coordinate_system=dict(data["coordinate_system"]),
            site_pose_provider=provider,
            sites=sites,
            field_objects=field_objects,
            vehicle=vehicle,
            dynamics=dict(data["dynamics"]),
            unload_profiles=unload_profiles,
            topology_profiles=dict(data["topology_profiles"]),
            action_profiles=dict(data["action_profiles"]),
            planner_profiles=dict(data["planner_profiles"]),
            start_check=dict(data["start_check"]),
            arrival_check=dict(data["arrival_check"]),
            finish_policy=dict(data["finish_policy"]),
            output=dict(data["output"]),
            traj_table=dict(data["traj_table"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": self.format,
            "project_id": self.project_id,
            "protocol_version": self.protocol_version,
            "nominal_field": self.nominal_field,
            "coordinate_system": self.coordinate_system,
            "site_pose_provider": self.site_pose_provider,
            "sites": self.sites,
            "field_objects": self.field_objects,
            "vehicle": self.vehicle,
            "dynamics": self.dynamics,
            "unload_profiles": self.unload_profiles,
            "topology_profiles": self.topology_profiles,
            "action_profiles": self.action_profiles,
            "planner_profiles": self.planner_profiles,
            "start_check": self.start_check,
            "arrival_check": self.arrival_check,
            "finish_policy": self.finish_policy,
            "output": self.output,
            "traj_table": self.traj_table,
        }


def validate_project_sites(data: object, object_type: str, field_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError(object_type, field_path, "must be an object", actual=type(data).__name__)
    actual_keys = set(data)
    expected_keys = set(REQUIRED_SITE_KEYS)
    if actual_keys != expected_keys:
        raise V40ValidationError(
            object_type,
            field_path,
            "sites must contain exactly the eight reusable fixed poses",
            actual=sorted(actual_keys),
            expected=list(REQUIRED_SITE_KEYS),
        )
    result: dict[str, Any] = {}
    for key in REQUIRED_SITE_KEYS:
        raw = data[key]
        if not isinstance(raw, dict):
            raise V40ValidationError(object_type, f"{field_path}.{key}", "site must be an object", actual=type(raw).__name__)
        reject_unknown_fields(raw, COMMON_SITE_FIELDS, object_type, f"{field_path}.{key}")
        require_fields(raw, COMMON_SITE_FIELDS, object_type, f"{field_path}.{key}")
        configured = _expect_bool(raw["configured"], object_type, f"{field_path}.{key}.configured")
        item = {
            "configured": configured,
            "x_mm": _expect_site_int(raw["x_mm"], object_type, f"{field_path}.{key}.x_mm"),
            "y_mm": _expect_site_int(raw["y_mm"], object_type, f"{field_path}.{key}.y_mm"),
        }
        yaw_ddeg = _expect_site_int(raw["yaw_ddeg"], object_type, f"{field_path}.{key}.yaw_ddeg")
        # Earlier builds accidentally used the one-byte 0xFF marker for the
        # JSON/UI yaw sentinel.  Accept it only as an input migration and
        # normalize immediately so a real 25.5 degree yaw is never displayed.
        item["yaw_ddeg"] = YAW_UNSPECIFIED_DDEG if yaw_ddeg == 0xFF else yaw_ddeg
        result[key] = item
    return result


def validate_unload_profiles(data: object, object_type: str, field_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError(object_type, field_path, "must be an object", actual=type(data).__name__)
    actual_keys = set(data)
    expected_keys = set(REQUIRED_UNLOAD_PROFILE_KEYS)
    if actual_keys != expected_keys:
        raise V40ValidationError(
            object_type,
            field_path,
            "unload_profiles must contain exactly the five allowed V4.0 unload masks",
            actual=sorted(actual_keys),
            expected=list(REQUIRED_UNLOAD_PROFILE_KEYS),
        )
    result: dict[str, Any] = {}
    for key in REQUIRED_UNLOAD_PROFILE_KEYS:
        raw = data[key]
        if not isinstance(raw, dict):
            raise V40ValidationError(object_type, f"{field_path}.{key}", "profile must be an object", actual=type(raw).__name__)
        reject_unknown_fields(raw, UNLOAD_PROFILE_FIELDS, object_type, f"{field_path}.{key}")
        require_fields(raw, UNLOAD_PROFILE_FIELDS, object_type, f"{field_path}.{key}")
        result[key] = {
            "configured": _expect_bool(raw["configured"], object_type, f"{field_path}.{key}.configured"),
            "yaw_ddeg": _expect_site_int(raw["yaw_ddeg"], object_type, f"{field_path}.{key}.yaw_ddeg"),
            "dx_mm": _expect_site_int(raw["dx_mm"], object_type, f"{field_path}.{key}.dx_mm"),
            "dy_mm": _expect_site_int(raw["dy_mm"], object_type, f"{field_path}.{key}.dy_mm"),
            "estimated_action_time_ms": _expect_nonnegative_int(
                raw["estimated_action_time_ms"],
                object_type,
                f"{field_path}.{key}.estimated_action_time_ms",
            ),
        }
    return result


def validate_project_vehicle(data: object, object_type: str, field_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError(object_type, field_path, "must be an object", actual=type(data).__name__)
    result = dict(data)
    footprint = result.get("footprint")
    if not isinstance(footprint, dict):
        raise V40ValidationError(object_type, f"{field_path}.footprint", "must be an object", actual=type(footprint).__name__)
    missing = FOOTPRINT_REQUIRED_FIELDS - set(footprint)
    if missing:
        raise V40ValidationError(
            object_type,
            f"{field_path}.footprint",
            "Phase 5 collision footprint fields are required",
            actual=sorted(footprint),
            expected=sorted(FOOTPRINT_REQUIRED_FIELDS),
        )
    r_large = _expect_positive_number(footprint["r_large_mm"], object_type, f"{field_path}.footprint.r_large_mm")
    r_small = _expect_positive_number(footprint["r_small_mm"], object_type, f"{field_path}.footprint.r_small_mm")
    if r_large <= r_small:
        raise V40ValidationError(object_type, f"{field_path}.footprint.r_large_mm", "must be greater than r_small_mm", actual=r_large, expected=f"> {r_small}")
    collision_resolution = _expect_positive_number(footprint["collision_resolution_mm"], object_type, f"{field_path}.footprint.collision_resolution_mm")
    strict_resolution = _expect_positive_number(
        footprint["strict_validation_resolution_mm"],
        object_type,
        f"{field_path}.footprint.strict_validation_resolution_mm",
    )
    if strict_resolution > collision_resolution:
        raise V40ValidationError(
            object_type,
            f"{field_path}.footprint.strict_validation_resolution_mm",
            "strict validation resolution must not be coarser than collision_resolution_mm",
            actual=strict_resolution,
            expected=f"<= {collision_resolution}",
        )
    epsilon = _expect_nonnegative_number(footprint["numerical_epsilon_mm"], object_type, f"{field_path}.footprint.numerical_epsilon_mm")
    if epsilon > 1.0:
        raise V40ValidationError(object_type, f"{field_path}.footprint.numerical_epsilon_mm", "epsilon must not be used as safety margin", actual=epsilon)
    arc_segments = _expect_site_int(footprint["pickup_arc_segments"], object_type, f"{field_path}.footprint.pickup_arc_segments")
    if arc_segments < 3:
        raise V40ValidationError(object_type, f"{field_path}.footprint.pickup_arc_segments", "must be at least 3", actual=arc_segments)
    expect_equal(
        footprint["field_boundary_footprint_profile"],
        "LARGE_CIRCLE",
        object_type,
        f"{field_path}.footprint.field_boundary_footprint_profile",
    )
    result["footprint"] = dict(footprint)
    return result


def validate_project_field_objects(data: object, object_type: str, field_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError(object_type, field_path, "must be an object", actual=type(data).__name__)
    reject_unknown_fields(data, FIELD_OBJECT_FIELDS, object_type, field_path)
    require_fields(data, FIELD_OBJECT_FIELDS, object_type, field_path)
    cylinders = _validate_object_list(data["cylinders"], 2, CYLINDER_FIELDS, object_type, f"{field_path}.cylinders")
    pickup_boxes = _validate_object_list(data["pickup_boxes"], 3, PICKUP_BOX_FIELDS, object_type, f"{field_path}.pickup_boxes")
    drop_boxes = _validate_object_list(data["drop_boxes"], 5, DROP_BOX_FIELDS, object_type, f"{field_path}.drop_boxes")
    _expect_exact_sites(
        tuple(str(item["physical_pick_site"]) for item in pickup_boxes),
        PHYSICAL_PICK_SITES,
        object_type,
        f"{field_path}.pickup_boxes",
    )
    _expect_exact_sites(
        tuple(str(item["physical_drop_site"]) for item in drop_boxes),
        DROP_SITE_KEYS,
        object_type,
        f"{field_path}.drop_boxes",
    )
    _validate_numeric_obstacles(cylinders, object_type, f"{field_path}.cylinders", ("center_x_mm", "center_y_mm", "radius_mm"))
    _validate_numeric_obstacles(pickup_boxes, object_type, f"{field_path}.pickup_boxes", ("center_x_mm", "center_y_mm", "length_mm", "width_mm", "yaw_ddeg"))
    _validate_numeric_obstacles(drop_boxes, object_type, f"{field_path}.drop_boxes", ("center_x_mm", "center_y_mm", "length_mm", "width_mm", "yaw_ddeg"))
    boundary = _validate_field_boundary(data["field_boundary"], object_type, f"{field_path}.field_boundary")
    return {
        "cylinders": cylinders,
        "pickup_boxes": pickup_boxes,
        "drop_boxes": drop_boxes,
        "field_boundary": boundary,
    }


def _expect_bool(value: object, object_type: str, field_path: str) -> bool:
    if not isinstance(value, bool):
        raise V40ValidationError(object_type, field_path, "must be a boolean", actual=value, expected="true/false")
    return value


def _expect_site_int(value: object, object_type: str, field_path: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise V40ValidationError(object_type, field_path, "must be an integer", actual=value)
    return value


def _expect_nonnegative_int(value: object, object_type: str, field_path: str) -> int:
    result = _expect_site_int(value, object_type, field_path)
    if result < 0:
        raise V40ValidationError(object_type, field_path, "must be non-negative", actual=value)
    return result


def _expect_positive_number(value: object, object_type: str, field_path: str) -> float:
    result = _expect_number(value, object_type, field_path)
    if result <= 0.0:
        raise V40ValidationError(object_type, field_path, "must be positive", actual=value)
    return result


def _expect_nonnegative_number(value: object, object_type: str, field_path: str) -> float:
    result = _expect_number(value, object_type, field_path)
    if result < 0.0:
        raise V40ValidationError(object_type, field_path, "must be non-negative", actual=value)
    return result


def _expect_number(value: object, object_type: str, field_path: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise V40ValidationError(object_type, field_path, "must be numeric", actual=value)
    result = float(value)
    if result != result or result in (float("inf"), float("-inf")):
        raise V40ValidationError(object_type, field_path, "must be finite", actual=value)
    return result


def _validate_object_list(data: object, count: int, fields: set[str], object_type: str, field_path: str) -> list[dict[str, Any]]:
    if not isinstance(data, list):
        raise V40ValidationError(object_type, field_path, "must be an array", actual=type(data).__name__)
    if len(data) != count:
        raise V40ValidationError(object_type, field_path, f"must contain exactly {count} items", actual=len(data), expected=count)
    result: list[dict[str, Any]] = []
    for index, item in enumerate(data):
        if not isinstance(item, dict):
            raise V40ValidationError(object_type, f"{field_path}[{index}]", "must be an object", actual=type(item).__name__)
        reject_unknown_fields(item, fields, object_type, f"{field_path}[{index}]")
        require_fields(item, fields, object_type, f"{field_path}[{index}]")
        parsed = dict(item)
        parsed["configured"] = _expect_bool(parsed["configured"], object_type, f"{field_path}[{index}].configured")
        parsed["enabled"] = _expect_bool(parsed["enabled"], object_type, f"{field_path}[{index}].enabled")
        result.append(parsed)
    return result


def _validate_numeric_obstacles(items: list[dict[str, Any]], object_type: str, field_path: str, keys: tuple[str, ...]) -> None:
    for index, item in enumerate(items):
        for key in keys:
            value = _expect_number(item[key], object_type, f"{field_path}[{index}].{key}")
            if key in {"radius_mm", "length_mm", "width_mm"} and value <= 0:
                raise V40ValidationError(object_type, f"{field_path}[{index}].{key}", "must be positive", actual=item[key])


def _expect_exact_sites(actual: tuple[str, ...], expected: tuple[str, ...], object_type: str, field_path: str) -> None:
    if set(actual) != set(expected) or len(actual) != len(expected):
        raise V40ValidationError(object_type, field_path, "must reference the exact V4.0 physical sites", actual=sorted(actual), expected=list(expected))


def _validate_field_boundary(data: object, object_type: str, field_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError(object_type, field_path, "must be an object", actual=type(data).__name__)
    reject_unknown_fields(data, FIELD_BOUNDARY_FIELDS, object_type, field_path)
    require_fields(data, FIELD_BOUNDARY_FIELDS, object_type, field_path)
    boundary = dict(data)
    boundary["enabled"] = _expect_bool(boundary["enabled"], object_type, f"{field_path}.enabled")
    expect_equal(boundary["footprint_profile"], "LARGE_CIRCLE", object_type, f"{field_path}.footprint_profile")
    expected_values = {
        "x_min_mm": -NOMINAL_FIELD_LENGTH_MM / 2,
        "x_max_mm": NOMINAL_FIELD_LENGTH_MM / 2,
        "y_min_mm": -NOMINAL_FIELD_WIDTH_MM / 2,
        "y_max_mm": NOMINAL_FIELD_WIDTH_MM / 2,
    }
    for key, expected in expected_values.items():
        actual = _expect_number(boundary[key], object_type, f"{field_path}.{key}")
        if actual != expected:
            raise V40ValidationError(object_type, f"{field_path}.{key}", "must use nominal V4.0 field bounds", actual=actual, expected=expected)
    return boundary
