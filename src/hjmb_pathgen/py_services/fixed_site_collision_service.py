"""Discrete collision checks for the eight reusable V4.0 fixed sites."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from hjmb_pathgen.py_domain.collision import CollisionContact, PoseCollisionResult, RobotPose
from hjmb_pathgen.py_domain.competition_task_config import (
    EXPECTED_UNLOAD_POSE_CATALOG,
    LOGICAL_DROP_STATIONS,
    UNLOAD_POSE_PROFILE_IDS,
)
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import REQUIRED_SITE_KEYS, YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_planning.collision.validator import check_pose_collision
from hjmb_pathgen.py_services.collision_config_service import build_collision_world


class FixedSiteCollisionResult(StrEnum):
    PASSED = "PASSED"
    FAILED = "FAILED"
    INCOMPLETE = "INCOMPLETE"


@dataclass(frozen=True)
class FixedSiteCollisionViolation:
    obstacle_id: str
    obstacle_type: str
    penetration_depth_mm: float
    signed_clearance_mm: float

    @classmethod
    def from_contact(cls, contact: CollisionContact) -> "FixedSiteCollisionViolation":
        return cls(
            obstacle_id=contact.obstacle_id,
            obstacle_type=contact.obstacle_type.value,
            penetration_depth_mm=float(contact.penetration_depth_mm),
            signed_clearance_mm=float(contact.signed_clearance_mm),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "obstacle_id": self.obstacle_id,
            "obstacle_type": self.obstacle_type,
            "penetration_depth_mm": self.penetration_depth_mm,
            "signed_clearance_mm": self.signed_clearance_mm,
        }


@dataclass(frozen=True)
class FixedSiteCollisionEntry:
    site_key: str
    profile_id: str | None
    x_mm: float | None
    y_mm: float | None
    yaw_ddeg: float | None
    checked: bool
    passed: bool
    min_signed_clearance_mm: float | None
    closest_obstacle_id: str | None
    collisions: tuple[FixedSiteCollisionViolation, ...] = ()
    incomplete_reason: str = ""

    @property
    def has_collision(self) -> bool:
        return bool(self.collisions)

    def to_dict(self) -> dict[str, Any]:
        return {
            "site_key": self.site_key,
            "profile_id": self.profile_id,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "yaw_ddeg": self.yaw_ddeg,
            "checked": self.checked,
            "passed": self.passed,
            "min_signed_clearance_mm": self.min_signed_clearance_mm,
            "closest_obstacle_id": self.closest_obstacle_id,
            "collisions": [item.to_dict() for item in self.collisions],
            "incomplete_reason": self.incomplete_reason,
        }


@dataclass(frozen=True)
class FixedSiteCollisionReport:
    result: FixedSiteCollisionResult
    entries: tuple[FixedSiteCollisionEntry, ...]
    errors: tuple[str, ...] = ()

    @property
    def passed_count(self) -> int:
        return sum(1 for entry in self.entries if entry.checked and entry.passed)

    @property
    def collision_count(self) -> int:
        return sum(1 for entry in self.entries if entry.has_collision)

    @property
    def incomplete_count(self) -> int:
        return sum(1 for entry in self.entries if not entry.checked) + len(self.errors)

    def to_dict(self) -> dict[str, Any]:
        return {
            "result": self.result.value,
            "passed_count": self.passed_count,
            "collision_count": self.collision_count,
            "incomplete_count": self.incomplete_count,
            "entries": [entry.to_dict() for entry in self.entries],
            "errors": list(self.errors),
        }


def check_fixed_site_collisions(project: ProjectV40) -> FixedSiteCollisionReport:
    """Check the current ProjectV40 fixed-site snapshot without mutating it."""

    try:
        world = build_collision_world(project)
    except (CompileError, KeyError, TypeError, ValueError) as exc:
        return FixedSiteCollisionReport(
            result=FixedSiteCollisionResult.INCOMPLETE,
            entries=(),
            errors=(f"碰撞世界无法建立：{exc}",),
        )

    entries: list[FixedSiteCollisionEntry] = []
    use_unload_pose_profiles = _use_unload_pose_profiles(project)
    for site_key in REQUIRED_SITE_KEYS:
        site = project.sites.get(site_key)
        if not isinstance(site, dict):
            entries.append(_incomplete(site_key, None, "固定点不存在"))
            continue
        if not bool(site.get("configured", False)):
            entries.append(_incomplete(site_key, None, "固定点未配置"))
            continue

        x_mm = float(site["x_mm"])
        y_mm = float(site["y_mm"])
        yaw_ddeg = int(site["yaw_ddeg"])
        if yaw_ddeg != YAW_UNSPECIFIED_DDEG:
            entries.append(_check_pose(site_key, None, RobotPose(x_mm, y_mm, yaw_ddeg), world))
            continue

        if site_key in LOGICAL_DROP_STATIONS and use_unload_pose_profiles:
            entries.extend(_check_unload_pose_profiles(project, site_key, x_mm, y_mm, world))
        else:
            entries.append(_incomplete(site_key, None, "未指定 yaw"))

    result = _report_result(entries)
    return FixedSiteCollisionReport(result=result, entries=tuple(entries))


def _use_unload_pose_profiles(project: ProjectV40) -> bool:
    default_profile = project.planner_profiles.get("default", {})
    if not isinstance(default_profile, dict):
        return False
    return bool(default_profile.get("use_unload_pose_profiles", False))


def _check_unload_pose_profiles(project: ProjectV40, site_key: str, x_mm: float, y_mm: float, world: Any) -> list[FixedSiteCollisionEntry]:
    entries: list[FixedSiteCollisionEntry] = []
    for profile_id in UNLOAD_POSE_PROFILE_IDS:
        spec = EXPECTED_UNLOAD_POSE_CATALOG[profile_id]
        if str(spec["station_site"]) != site_key:
            continue
        profile = project.unload_pose_profiles.get(profile_id)
        if not isinstance(profile, dict) or not bool(profile.get("configured", False)):
            continue
        yaw_ddeg = int(profile["yaw_ddeg"])
        if yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            entries.append(_incomplete(site_key, profile_id, "倒货姿态未指定 yaw"))
            continue
        pose = RobotPose(
            x_mm=x_mm + float(profile["dx_mm"]),
            y_mm=y_mm + float(profile["dy_mm"]),
            yaw_ddeg=yaw_ddeg,
        )
        entries.append(_check_pose(site_key, profile_id, pose, world))

    if not entries:
        return [_incomplete(site_key, None, "无可用已配置倒货姿态")]
    return entries


def _check_pose(site_key: str, profile_id: str | None, pose: RobotPose, world: Any) -> FixedSiteCollisionEntry:
    result: PoseCollisionResult = check_pose_collision(
        pose,
        world,
        {},
        source={"fixed_site": site_key, "unload_pose_profile_id": profile_id or ""},
    )
    collisions = tuple(FixedSiteCollisionViolation.from_contact(contact) for contact in result.violations)
    return FixedSiteCollisionEntry(
        site_key=site_key,
        profile_id=profile_id,
        x_mm=float(pose.x_mm),
        y_mm=float(pose.y_mm),
        yaw_ddeg=float(pose.yaw_ddeg),
        checked=True,
        passed=not collisions,
        min_signed_clearance_mm=float(result.min_signed_clearance_mm),
        closest_obstacle_id=result.closest_obstacle_id,
        collisions=collisions,
    )


def _incomplete(site_key: str, profile_id: str | None, reason: str) -> FixedSiteCollisionEntry:
    return FixedSiteCollisionEntry(
        site_key=site_key,
        profile_id=profile_id,
        x_mm=None,
        y_mm=None,
        yaw_ddeg=None,
        checked=False,
        passed=False,
        min_signed_clearance_mm=None,
        closest_obstacle_id=None,
        incomplete_reason=reason,
    )


def _report_result(entries: list[FixedSiteCollisionEntry]) -> FixedSiteCollisionResult:
    if any(entry.has_collision for entry in entries):
        return FixedSiteCollisionResult.FAILED
    if any(not entry.checked for entry in entries):
        return FixedSiteCollisionResult.INCOMPLETE
    return FixedSiteCollisionResult.PASSED


__all__ = [
    "FixedSiteCollisionEntry",
    "FixedSiteCollisionReport",
    "FixedSiteCollisionResult",
    "FixedSiteCollisionViolation",
    "check_fixed_site_collisions",
]
