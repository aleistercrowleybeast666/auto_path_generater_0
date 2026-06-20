"""Explicit V3.5 project conversion into MANUAL or SEMI_AUTO V4 Cases."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from hjmb_pathgen.py_domain.enums import GenerationMode, StorageMode
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
if TYPE_CHECKING:
    from hjmb_pathgen.py_legacy.v35_import.legacy_models import LegacyProjectV35


LOGICAL_KEYS = (
    "P_START",
    "P_PICK_1",
    "P_PICK_2L",
    "P_PICK_2R",
    "P_PICK_3",
    "P_DROP_1",
    "P_DROP_2",
    "P_DROP_3",
)


@dataclass(frozen=True)
class V35CaseMigrationResult:
    case: CaseManifestV40
    migrated_action_count: int
    warnings: tuple[str, ...]
    unsupported_actions: tuple[dict[str, Any], ...]


def migrate_v35_to_manual(source: LegacyProjectV35) -> V35CaseMigrationResult:
    points = []
    warnings: list[str] = []
    for index, raw in enumerate(source.points):
        point_type = str(raw.get("type", ""))
        if point_type not in {"START", "WAYPOINT", "ARRIVAL"}:
            raise ValueError(f"unsupported V3.5 point type at index {index}: {point_type}")
        point: dict[str, Any] = {
            "type": point_type,
            "x_mm": round(float(raw.get("x_mm", 0))),
            "y_mm": round(float(raw.get("y_mm", 0))),
        }
        if point_type in {"START", "ARRIVAL"}:
            yaw = int(raw.get("yaw_ddeg", 0))
            point["yaw_ddeg"] = 0 if yaw == 0xFF else yaw
        else:
            point["exact_pass"] = bool(raw.get("exact_pass", False))
            if int(raw.get("max_speed_mmps", 0)) > 0:
                point["max_speed_mmps"] = int(raw["max_speed_mmps"])
        points.append(point)
    actions, unsupported, action_warnings = _migrate_actions(source.actions)
    warnings.extend(action_warnings)
    case = CaseManifestV40.from_dict(
        _case_base(
            source,
            GenerationMode.MANUAL,
            selected_plan={
                "route_family": "MANUAL",
                "vehicle_bin_assignment": {},
                "drop_targets": [],
                "unload_sequence": [],
                "yaw_direction": "SHORTEST",
                "locked_by_user": True,
            },
            manual_path={"points": points},
            logical_points=[],
            actions=actions,
        )
    )
    return V35CaseMigrationResult(case, len(actions), tuple(warnings), tuple(unsupported))


def migrate_v35_to_semi_auto(source: LegacyProjectV35) -> V35CaseMigrationResult:
    by_key = {str(item.get("site_key")): item for item in source.fixed_sites}
    missing = [key for key in LOGICAL_KEYS if key not in by_key]
    if missing:
        raise ValueError(f"V3.5 fixed_sites cannot form eight logical anchors: missing {missing}")
    logical_points = [
        {
            "point_id": key,
            "type": "TASK_ANCHOR",
            "pose": {
                "x_mm": round(float(by_key[key].get("x_mm", 0))),
                "y_mm": round(float(by_key[key].get("y_mm", 0))),
                "yaw_ddeg": int(by_key[key].get("yaw_ddeg", 0)),
            },
        }
        for key in LOGICAL_KEYS
    ]
    actions, unsupported, warnings = _migrate_actions(source.actions)
    site_key_by_id = {
        int(item.get("site_id", -1)): str(item.get("site_key", ""))
        for item in source.fixed_sites
    }
    site_id_by_point_id = {
        int(item.get("point_id", index)): int(item.get("site_id", -1))
        for index, item in enumerate(source.points)
    }
    for action in actions:
        if "arrival_point_id" not in action:
            continue
        point_id = int(action.pop("arrival_point_id"))
        site_key = site_key_by_id.get(site_id_by_point_id.get(point_id, -1), "")
        if site_key:
            action["arrival_state_id"] = site_key
        else:
            warnings.append(
                f"action arrival_point_id={point_id} could not be bound to a logical anchor"
            )
    selected_plan = {
        "route_family": "MANUAL",
        "pickup_arrival_state_order": ["P_PICK_1", "P_PICK_2L", "P_PICK_3"],
        "vehicle_bin_assignment": {},
        "drop_targets": [],
        "unload_sequence": [],
        "yaw_direction": "SHORTEST",
        "locked_by_user": True,
        "selection_state": "MIGRATED_REVIEW_REQUIRED",
    }
    case = CaseManifestV40.from_dict(
        _case_base(
            source,
            GenerationMode.SEMI_AUTO,
            selected_plan=selected_plan,
            manual_path=None,
            logical_points=logical_points,
            actions=actions,
        )
    )
    return V35CaseMigrationResult(case, len(actions), tuple(warnings), tuple(unsupported))


def _migrate_actions(
    source_actions: tuple[dict[str, Any], ...],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[str]]:
    migrated: list[dict[str, Any]] = []
    unsupported: list[dict[str, Any]] = []
    warnings: list[str] = []
    supported_modes = {"STOP_AND_WAIT", "ASYNC", "KINEMATIC"}
    for raw in sorted(source_actions, key=lambda item: int(item.get("action_seq", 0))):
        mode = str(raw.get("mode", "STOP_AND_WAIT"))
        action = raw.get("action")
        if mode not in supported_modes or action is None:
            unsupported.append(dict(raw))
            continue
        item = {
            "action": action,
            "mode": mode,
            "timeout_ms": int(raw.get("timeout_ms", 1000)),
            "post_wait_ms": int(raw.get("post_wait_ms", 0)),
        }
        if mode == "STOP_AND_WAIT":
            if raw.get("arrival_point_id") is None:
                unsupported.append(dict(raw))
                continue
            item["arrival_point_id"] = int(raw["arrival_point_id"])
        if mode == "KINEMATIC":
            limits = dict(raw.get("limits", {}))
            for key in (
                "accel_limit_mmps2",
                "beta_limit_ddegps2",
                "wz_limit_ddegps",
                "speed_limit_mmps",
                "stable_time_ms",
            ):
                item[key] = int(limits.get(key, raw.get(key, 0)))
        migrated.append(item)
    if unsupported:
        warnings.append(f"{len(unsupported)} V3.5 actions require manual review and were not migrated")
    if migrated and (
        not str(migrated[-1].get("action", "")).startswith("DROP_")
        or migrated[-1].get("mode") != "STOP_AND_WAIT"
    ):
        warnings.append("migrated final action is not DROP_* STOP_AND_WAIT; formal export remains blocked")
    return migrated, unsupported, warnings


def _case_base(
    source: LegacyProjectV35,
    mode: GenerationMode,
    *,
    selected_plan: dict[str, Any],
    manual_path: dict[str, Any] | None,
    logical_points: list[dict[str, Any]],
    actions: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "format": "HJMB_ROUTE_CASE_JSON_V40",
        "storage_mode": StorageMode.REFERENCED.value,
        "generation_mode": mode.value,
        "traj_id": source.traj_id,
        "bean_code": source.traj_id // 60,
        "drop_code": source.traj_id % 60,
        "source_mapping": {"migration_source": "HJMB_PATH_EDITOR_JSON_V35"},
        "selected_plan": selected_plan,
        "manual_path": manual_path,
        "logical_points": logical_points,
        "arrival_states": [],
        "leg_refs": [],
        "actions": {"source": actions, "compiled": []},
        "finish": {"mode": "AT_FINAL_DROP"},
        "estimates": {},
        "hashes": {},
        "review": {
            "state": "STALE",
            "detached_from_library": mode == GenerationMode.MANUAL,
            "manual_override": True,
            "approved": False,
            "override_reason": "explicit V3.5 migration; operator review required",
        },
    }
