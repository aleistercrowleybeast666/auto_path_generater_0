"""Build and validate V4.0 collision worlds from project.json."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.collision.obstacles import CollisionWorld, CylinderObstacle, FieldBoundary, RectObstacle
from hjmb_pathgen.collision.primitives import OrientedRect, Point2
from hjmb_pathgen.models.collision import FootprintProfile, ObstacleType
from hjmb_pathgen.models.errors import CompileError
from hjmb_pathgen.models.project import ProjectV40

COLLISION_ALGORITHM_VERSION = "phase5.collision.v1"


@dataclass(frozen=True)
class CollisionConfigReport:
    ok: bool
    collision_config_hash: str
    obstacle_geometry_hash: str
    warnings: tuple[str, ...]
    errors: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "collision_config_hash": self.collision_config_hash,
            "obstacle_geometry_hash": self.obstacle_geometry_hash,
            "warnings": list(self.warnings),
            "errors": list(self.errors),
        }


def validate_collision_config(project: ProjectV40) -> CollisionConfigReport:
    warnings: list[str] = []
    errors: list[str] = []
    try:
        world = build_collision_world(project)
    except CompileError as exc:
        return CollisionConfigReport(
            ok=False,
            collision_config_hash="",
            obstacle_geometry_hash="",
            warnings=(),
            errors=(str(exc),),
        )
    if not world.cylinders:
        errors.append("no enabled configured cylinders")
    if len(world.pickup_boxes) != 3:
        errors.append("expected three enabled configured pickup boxes")
    if len(world.drop_boxes) != 5:
        errors.append("expected five enabled configured drop boxes")
    return CollisionConfigReport(
        ok=not errors,
        collision_config_hash=world.collision_config_hash,
        obstacle_geometry_hash=world.obstacle_geometry_hash,
        warnings=tuple(warnings),
        errors=tuple(errors),
    )


def compute_collision_hashes(project: ProjectV40) -> dict[str, str]:
    footprint = project.vehicle.get("footprint", {})
    config_payload = {
        "algorithm_version": COLLISION_ALGORITHM_VERSION,
        "footprint": _functional_value(footprint),
        "field_boundary": _functional_value(project.field_objects.get("field_boundary", {})),
    }
    obstacle_payload = {
        "cylinders": _functional_value(project.field_objects.get("cylinders", [])),
        "pickup_boxes": _functional_value(project.field_objects.get("pickup_boxes", [])),
        "drop_boxes": _functional_value(project.field_objects.get("drop_boxes", [])),
    }
    return {
        "collision_config_hash": canonical_json_crc32_hex(config_payload),
        "obstacle_geometry_hash": canonical_json_crc32_hex(obstacle_payload),
    }


def build_collision_world(project: ProjectV40) -> CollisionWorld:
    footprint = project.vehicle["footprint"]
    hashes = compute_collision_hashes(project)
    field_objects = project.field_objects
    boundary_data = field_objects["field_boundary"]
    boundary = FieldBoundary(
        enabled=bool(boundary_data["enabled"]),
        x_min_mm=float(boundary_data["x_min_mm"]),
        x_max_mm=float(boundary_data["x_max_mm"]),
        y_min_mm=float(boundary_data["y_min_mm"]),
        y_max_mm=float(boundary_data["y_max_mm"]),
        footprint_profile=FootprintProfile(str(boundary_data["footprint_profile"])),
    )
    return CollisionWorld(
        r_large_mm=float(footprint["r_large_mm"]),
        r_small_mm=float(footprint["r_small_mm"]),
        collision_resolution_mm=float(footprint["collision_resolution_mm"]),
        strict_validation_resolution_mm=float(footprint["strict_validation_resolution_mm"]),
        numerical_epsilon_mm=float(footprint["numerical_epsilon_mm"]),
        pickup_arc_segments=int(footprint["pickup_arc_segments"]),
        field_boundary=boundary,
        cylinders=_build_cylinders(field_objects["cylinders"]),
        pickup_boxes=_build_rect_obstacles(field_objects["pickup_boxes"], ObstacleType.PICKUP_BOX),
        drop_boxes=_build_rect_obstacles(field_objects["drop_boxes"], ObstacleType.DROP_BOX),
        collision_config_hash=hashes["collision_config_hash"],
        obstacle_geometry_hash=hashes["obstacle_geometry_hash"],
    )


def _build_cylinders(items: list[dict[str, Any]]) -> tuple[CylinderObstacle, ...]:
    result: list[CylinderObstacle] = []
    for item in items:
        if item.get("enabled") and not item.get("configured"):
            raise CompileError(f"enabled cylinder is not configured: {item.get('obstacle_id')}")
        if not item.get("enabled"):
            continue
        result.append(
            CylinderObstacle(
                obstacle_id=str(item["obstacle_id"]),
                center=Point2(float(item["center_x_mm"]), float(item["center_y_mm"])),
                radius_mm=float(item["radius_mm"]),
                enabled=True,
            )
        )
    return tuple(result)


def _build_rect_obstacles(items: list[dict[str, Any]], obstacle_type: ObstacleType) -> tuple[RectObstacle, ...]:
    result: list[RectObstacle] = []
    site_key = "physical_pick_site" if obstacle_type == ObstacleType.PICKUP_BOX else "physical_drop_site"
    for item in items:
        if item.get("enabled") and not item.get("configured"):
            raise CompileError(f"enabled {obstacle_type.value} is not configured: {item.get('obstacle_id')}")
        if not item.get("enabled"):
            continue
        result.append(
            RectObstacle(
                obstacle_id=str(item["obstacle_id"]),
                obstacle_type=obstacle_type,
                physical_site=str(item[site_key]),
                enabled=True,
                rect=OrientedRect(
                    center=Point2(float(item["center_x_mm"]), float(item["center_y_mm"])),
                    length_mm=float(item["length_mm"]),
                    width_mm=float(item["width_mm"]),
                    yaw_ddeg=float(item["yaw_ddeg"]),
                ),
            )
        )
    return tuple(result)


def _functional_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            key: _functional_value(val)
            for key, val in value.items()
            if key not in {"notes", "note", "ui_state", "updated_at", "generated_at", "self_hash", "self_hash32"}
        }
    if isinstance(value, list):
        return [_functional_value(item) for item in value]
    return value
