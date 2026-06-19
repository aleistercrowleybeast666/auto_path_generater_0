"""Collision world obstacle definitions."""

from __future__ import annotations

from dataclasses import dataclass

from hjmb_pathgen.collision.primitives import OrientedRect, Point2
from hjmb_pathgen.models.collision import FootprintProfile, ObstacleType


@dataclass(frozen=True)
class CylinderObstacle:
    obstacle_id: str
    center: Point2
    radius_mm: float
    enabled: bool = True
    obstacle_type: ObstacleType = ObstacleType.CYLINDER


@dataclass(frozen=True)
class RectObstacle:
    obstacle_id: str
    obstacle_type: ObstacleType
    rect: OrientedRect
    physical_site: str
    enabled: bool = True


@dataclass(frozen=True)
class FieldBoundary:
    x_min_mm: float
    x_max_mm: float
    y_min_mm: float
    y_max_mm: float
    footprint_profile: FootprintProfile = FootprintProfile.LARGE_CIRCLE
    enabled: bool = True
    obstacle_id: str = "FIELD_BOUNDARY"
    obstacle_type: ObstacleType = ObstacleType.FIELD_BOUNDARY


@dataclass(frozen=True)
class CollisionWorld:
    r_large_mm: float
    r_small_mm: float
    collision_resolution_mm: float
    strict_validation_resolution_mm: float
    numerical_epsilon_mm: float
    pickup_arc_segments: int
    field_boundary: FieldBoundary
    cylinders: tuple[CylinderObstacle, ...]
    pickup_boxes: tuple[RectObstacle, ...]
    drop_boxes: tuple[RectObstacle, ...]
    collision_config_hash: str
    obstacle_geometry_hash: str

    @property
    def obstacles(self) -> tuple[object, ...]:
        result: list[object] = []
        if self.field_boundary.enabled:
            result.append(self.field_boundary)
        result.extend(item for item in self.cylinders if item.enabled)
        result.extend(item for item in self.pickup_boxes if item.enabled)
        result.extend(item for item in self.drop_boxes if item.enabled)
        return tuple(result)
