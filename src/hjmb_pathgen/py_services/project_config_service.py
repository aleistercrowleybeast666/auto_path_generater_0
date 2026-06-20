"""Phase 4 project configuration validation and functional hashes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Iterable

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import (
    FUNCTIONAL_HASH_KEYS,
    REQUIRED_SITE_KEYS,
    REQUIRED_UNLOAD_PROFILE_KEYS,
)
from hjmb_pathgen.py_services.collision_config_service import compute_collision_hashes, validate_collision_config

NONFUNCTIONAL_KEYS = {
    "generated_at",
    "updated_at",
    "created_at",
    "modified_at",
    "notes",
    "note",
    "ui_state",
    "self_hash",
    "self_hash32",
}


class ProjectReadiness(StrEnum):
    ROUTE_TABLE_ONLY = "ROUTE_TABLE_ONLY"
    SEMANTIC_CANDIDATES_READY = "SEMANTIC_CANDIDATES_READY"
    MANUAL_PLANNING_READY = "MANUAL_PLANNING_READY"
    SEMI_AUTO_PLANNING_READY = "SEMI_AUTO_PLANNING_READY"
    FULL_360_PLANNING_READY = "FULL_360_PLANNING_READY"


@dataclass(frozen=True)
class ProjectConfigurationReport:
    readiness: ProjectReadiness
    ready_for_route_table: bool
    ready_for_semantic_candidates: bool
    ready_for_manual_planning: bool
    ready_for_semi_auto_planning: bool
    ready_for_full_360_planning: bool
    configured_sites: tuple[str, ...]
    missing_sites: tuple[str, ...]
    configured_unload_profiles: tuple[str, ...]
    missing_unload_profiles: tuple[str, ...]
    functional_hashes: dict[str, str]
    warnings: tuple[str, ...]
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {
            "readiness": self.readiness.value,
            "ready_for_route_table": self.ready_for_route_table,
            "ready_for_semantic_candidates": self.ready_for_semantic_candidates,
            "ready_for_manual_planning": self.ready_for_manual_planning,
            "ready_for_semi_auto_planning": self.ready_for_semi_auto_planning,
            "ready_for_full_360_planning": self.ready_for_full_360_planning,
            "configured_sites": list(self.configured_sites),
            "missing_sites": list(self.missing_sites),
            "configured_unload_profiles": list(self.configured_unload_profiles),
            "missing_unload_profiles": list(self.missing_unload_profiles),
            "functional_hashes": dict(self.functional_hashes),
            "warnings": list(self.warnings),
            "errors": list(self.errors),
        }


def validate_project_site_configuration(project: ProjectV40) -> ProjectConfigurationReport:
    configured_sites = tuple(key for key in REQUIRED_SITE_KEYS if bool(project.sites[key].get("configured")))
    missing_sites = tuple(key for key in REQUIRED_SITE_KEYS if key not in configured_sites)
    configured_profiles = tuple(key for key in REQUIRED_UNLOAD_PROFILE_KEYS if bool(project.unload_profiles[key].get("configured")))
    missing_profiles = tuple(key for key in REQUIRED_UNLOAD_PROFILE_KEYS if key not in configured_profiles)
    hashes = compute_project_functional_hashes(project)
    errors: list[str] = []
    warnings: list[str] = []
    if missing_sites:
        warnings.append(f"unconfigured sites: {', '.join(missing_sites)}")
    if missing_profiles:
        warnings.append(f"unconfigured unload profiles: {', '.join(missing_profiles)}")
    if not _vehicle_complete(project.vehicle):
        errors.append("vehicle configuration is incomplete")
    collision_report = validate_collision_config(project)
    errors.extend(collision_report.errors)
    warnings.extend(collision_report.warnings)
    if not _dynamics_complete(project.dynamics):
        errors.append("dynamics configuration is incomplete")
    if not project.planner_profiles:
        warnings.append("planner_profiles is empty; defaults may be used by services")

    ready_manual = _vehicle_complete(project.vehicle) and _dynamics_complete(project.dynamics)
    ready_semi = ready_manual and bool(project.topology_profiles)
    drop_boxes_ready = all(
        bool(item.get("configured"))
        for item in project.field_objects.get("drop_boxes", [])
    )
    ready_full = (
        ready_semi
        and not missing_sites
        and not missing_profiles
        and drop_boxes_ready
        and bool(project.action_profiles)
    )
    if ready_full:
        readiness = ProjectReadiness.FULL_360_PLANNING_READY
    elif ready_semi:
        readiness = ProjectReadiness.SEMI_AUTO_PLANNING_READY
    elif ready_manual:
        readiness = ProjectReadiness.MANUAL_PLANNING_READY
    elif configured_sites or configured_profiles:
        readiness = ProjectReadiness.SEMANTIC_CANDIDATES_READY
    else:
        readiness = ProjectReadiness.ROUTE_TABLE_ONLY
    return ProjectConfigurationReport(
        readiness=readiness,
        ready_for_route_table=True,
        ready_for_semantic_candidates=True,
        ready_for_manual_planning=ready_manual,
        ready_for_semi_auto_planning=ready_semi,
        ready_for_full_360_planning=ready_full,
        configured_sites=configured_sites,
        missing_sites=missing_sites,
        configured_unload_profiles=configured_profiles,
        missing_unload_profiles=missing_profiles,
        functional_hashes=hashes,
        warnings=tuple(warnings),
        errors=tuple(errors),
    )


def compute_project_functional_hashes(project: ProjectV40) -> dict[str, str]:
    values = {
        "site_config_hash": {
            "site_pose_provider": project.site_pose_provider,
            "sites": project.sites,
            "unload_profiles": project.unload_profiles,
        },
        "vehicle_config_hash": project.vehicle,
        "dynamics_config_hash": project.dynamics,
        "planner_config_hash": project.planner_profiles,
        "action_profile_hash": project.action_profiles,
        "topology_config_hash": project.topology_profiles,
    }
    hashes = {key: canonical_json_crc32_hex(_functional_value(value)) for key, value in values.items()}
    hashes.update(compute_collision_hashes(project))
    return {key: hashes[key] for key in FUNCTIONAL_HASH_KEYS}


def mark_dependents_stale(records: Iterable[dict[str, Any]], old_hashes: dict[str, str], new_hashes: dict[str, str]) -> tuple[dict[str, Any], ...]:
    changed = sorted(key for key in FUNCTIONAL_HASH_KEYS if old_hashes.get(key) != new_hashes.get(key))
    result: list[dict[str, Any]] = []
    for record in records:
        item = dict(record)
        dependencies = dict(item.get("dependency_hashes", item.get("hashes", {})))
        if any(dependencies.get(key) and dependencies.get(key) != new_hashes.get(key) for key in changed):
            if "state" in item:
                item["state"] = "STALE"
            review = dict(item.get("review", {}))
            review["state"] = "STALE"
            review["stale_reason"] = f"functional config changed: {', '.join(changed)}"
            item["review"] = review
        result.append(item)
    return tuple(result)


def _functional_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _functional_value(val) for key, val in value.items() if key not in NONFUNCTIONAL_KEYS}
    if isinstance(value, list):
        return [_functional_value(item) for item in value]
    if isinstance(value, tuple):
        return [_functional_value(item) for item in value]
    return value


def _vehicle_complete(vehicle: dict[str, Any]) -> bool:
    footprint = vehicle.get("footprint", {}) if isinstance(vehicle, dict) else {}
    wheel = vehicle.get("wheel", {}) if isinstance(vehicle, dict) else {}
    required_positive = (
        footprint.get("r_large_mm"),
        footprint.get("r_small_mm"),
        wheel.get("radius_mm"),
        wheel.get("rotation_radius_mm"),
        wheel.get("plan_limit_rpm"),
    )
    return all(isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0 for value in required_positive)


def _dynamics_complete(dynamics: dict[str, Any]) -> bool:
    required_positive = (
        dynamics.get("max_speed_mmps"),
        dynamics.get("linear_accel_mmps2"),
        dynamics.get("braking_accel_mmps2"),
        dynamics.get("lateral_accel_mmps2"),
        dynamics.get("max_wz_ddegps"),
        dynamics.get("angular_accel_moving_ddegps2"),
    )
    return all(isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0 for value in required_positive)
