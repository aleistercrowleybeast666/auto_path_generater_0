"""V4.0 collision model enums and serializable diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class FootprintProfile(StrEnum):
    LARGE_CIRCLE = "LARGE_CIRCLE"
    SMALL_CIRCLE = "SMALL_CIRCLE"
    PICKUP_CLIPPED_DISK = "PICKUP_CLIPPED_DISK"


class ObstacleType(StrEnum):
    CYLINDER = "CYLINDER"
    DROP_BOX = "DROP_BOX"
    PICKUP_BOX = "PICKUP_BOX"
    FIELD_BOUNDARY = "FIELD_BOUNDARY"


class ClearanceClass(StrEnum):
    CLEAR = "CLEAR"
    TOUCHING = "TOUCHING"
    PENETRATING = "PENETRATING"


class CollisionStatus(StrEnum):
    NOT_CHECKED = "NOT_CHECKED"
    PASSED = "PASSED"
    FAILED = "FAILED"
    STALE = "STALE"
    NUMERICAL_ERROR = "NUMERICAL_ERROR"
    NO_GEOMETRY = "NO_GEOMETRY"


FOOTPRINT_BY_OBSTACLE_TYPE = {
    ObstacleType.CYLINDER: FootprintProfile.LARGE_CIRCLE,
    ObstacleType.DROP_BOX: FootprintProfile.SMALL_CIRCLE,
    ObstacleType.PICKUP_BOX: FootprintProfile.PICKUP_CLIPPED_DISK,
    ObstacleType.FIELD_BOUNDARY: FootprintProfile.LARGE_CIRCLE,
}


@dataclass(frozen=True)
class RobotPose:
    x_mm: float
    y_mm: float
    yaw_ddeg: float
    s_mm: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {
            "s_mm": self.s_mm,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "yaw_ddeg": self.yaw_ddeg,
        }


@dataclass(frozen=True)
class CollisionContact:
    obstacle_id: str
    obstacle_type: ObstacleType
    footprint_profile: FootprintProfile
    clearance_class: ClearanceClass
    signed_clearance_mm: float
    penetration_depth_mm: float
    pose: RobotPose
    source: dict[str, Any]
    diagnostic: dict[str, Any]

    @property
    def is_collision(self) -> bool:
        return self.clearance_class == ClearanceClass.PENETRATING

    def to_dict(self) -> dict[str, Any]:
        return {
            "obstacle_id": self.obstacle_id,
            "obstacle_type": self.obstacle_type.value,
            "footprint_profile": self.footprint_profile.value,
            "clearance_class": self.clearance_class.value,
            "signed_clearance_mm": self.signed_clearance_mm,
            "penetration_depth_mm": self.penetration_depth_mm,
            "pose": self.pose.to_dict(),
            "source": dict(self.source),
            "diagnostic": dict(self.diagnostic),
        }


@dataclass(frozen=True)
class PoseCollisionResult:
    is_valid: bool
    pose: RobotPose
    min_signed_clearance_mm: float
    closest_obstacle_id: str | None
    contacts: tuple[CollisionContact, ...]
    violations: tuple[CollisionContact, ...]
    numerical_epsilon_mm: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "is_valid": self.is_valid,
            "pose": self.pose.to_dict(),
            "min_signed_clearance_mm": self.min_signed_clearance_mm,
            "closest_obstacle_id": self.closest_obstacle_id,
            "contacts": [contact.to_dict() for contact in self.contacts],
            "violations": [contact.to_dict() for contact in self.violations],
            "numerical_epsilon_mm": self.numerical_epsilon_mm,
        }


@dataclass(frozen=True)
class PathCollisionResult:
    status: CollisionStatus
    checked_config_hash: str
    checked_path_hash: str
    validation_resolution_mm: float
    min_clearance_mm: float | None
    min_clearance_pose: RobotPose | None
    min_clearance_obstacle: str | None
    collision_count: int
    first_collision: CollisionContact | None
    collisions: tuple[CollisionContact, ...]
    checked_pose_count: int
    subdivision_count: int
    elapsed_ms: float
    warnings: tuple[str, ...]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return self.status == CollisionStatus.PASSED

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "checked_config_hash": self.checked_config_hash,
            "checked_path_hash": self.checked_path_hash,
            "validation_resolution_mm": self.validation_resolution_mm,
            "min_clearance_mm": self.min_clearance_mm,
            "min_clearance_pose": self.min_clearance_pose.to_dict() if self.min_clearance_pose else None,
            "min_clearance_obstacle": self.min_clearance_obstacle,
            "collision_count": self.collision_count,
            "first_collision": self.first_collision.to_dict() if self.first_collision else None,
            "collisions": [collision.to_dict() for collision in self.collisions],
            "checked_pose_count": self.checked_pose_count,
            "subdivision_count": self.subdivision_count,
            "elapsed_ms": self.elapsed_ms,
            "warnings": list(self.warnings),
            "errors": list(self.errors),
        }
