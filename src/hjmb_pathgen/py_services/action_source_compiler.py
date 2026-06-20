"""Source mechanical FIFO action generation for Phase 3 task candidates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode, UnloadMask
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import RouteCaseRowV40
from hjmb_pathgen.py_domain.task_plan import UnloadStep

PICK_PREP_ACTION_BY_STATE = {
    "P_PICK_1": ActionCode.PREP_PICK_1,
    "P_PICK_2L": ActionCode.PREP_PICK_2L,
    "P_PICK_2R": ActionCode.PREP_PICK_2R,
    "P_PICK_3": ActionCode.PREP_PICK_3,
}

STORE_PREP_ACTION_BY_BIN = {
    "BIN_1": ActionCode.PREP_STORE_1,
    "BIN_2": ActionCode.PREP_STORE_2,
    "BIN_3": ActionCode.PREP_STORE_3,
}

DROP_ACTION_BY_MASK = {
    UnloadMask.BIN_1: ActionCode.DROP_1,
    UnloadMask.BIN_2: ActionCode.DROP_2,
    UnloadMask.BIN_3: ActionCode.DROP_3,
    UnloadMask.BIN_12: ActionCode.DROP_12,
    UnloadMask.BIN_23: ActionCode.DROP_23,
}


@dataclass(frozen=True)
class SourceActionCompileResult:
    actions: tuple[dict[str, Any], ...]
    estimated_mechanism_time_ms: int


def compile_source_actions(
    project: ProjectV40,
    row: RouteCaseRowV40,
    *,
    pickup_position_order: tuple[str, ...],
    pickup_arrival_state_order: tuple[str, ...],
    vehicle_bin_assignment: dict[str, str],
    unload_sequence: tuple[UnloadStep, ...],
) -> SourceActionCompileResult:
    if len(pickup_position_order) != len(pickup_arrival_state_order):
        raise CompileError("pickup position order and arrival state order length mismatch")

    actions: list[dict[str, Any]] = []
    stored_by_bin: dict[str, str] = {}
    dropped_beans: list[str] = []

    for pickup_slot, arrival_state_id in zip(pickup_position_order, pickup_arrival_state_order, strict=True):
        bean_type = str(row.pick_assignment[pickup_slot])
        vehicle_bin = str(vehicle_bin_assignment[bean_type])
        if vehicle_bin in stored_by_bin:
            raise CompileError(f"{vehicle_bin} would be overwritten before unloading")
        prep_pick = PICK_PREP_ACTION_BY_STATE[arrival_state_id]
        actions.append(_source_action(project, prep_pick, arrival_state_id=arrival_state_id, pickup_slot=pickup_slot, bean_type=bean_type))
        actions.append(_source_action(project, ActionCode.PICK, arrival_state_id=arrival_state_id, pickup_slot=pickup_slot, bean_type=bean_type))
        actions.append(
            _source_action(
                project,
                STORE_PREP_ACTION_BY_BIN[vehicle_bin],
                arrival_state_id=arrival_state_id,
                pickup_slot=pickup_slot,
                bean_type=bean_type,
                vehicle_bin=vehicle_bin,
            )
        )
        actions.append(
            _source_action(
                project,
                ActionCode.STORE,
                arrival_state_id=arrival_state_id,
                pickup_slot=pickup_slot,
                bean_type=bean_type,
                vehicle_bin=vehicle_bin,
            )
        )
        stored_by_bin[vehicle_bin] = bean_type

    for step in unload_sequence:
        for vehicle_bin, bean_type in zip(step.vehicle_bins, step.bean_types, strict=True):
            stored = stored_by_bin.get(vehicle_bin)
            if stored is None:
                raise CompileError(f"{vehicle_bin} is empty before {step.unload_mask.value}")
            if stored != bean_type:
                raise CompileError(f"{vehicle_bin} contains {stored}, cannot unload {bean_type}")
        actions.append(
            _source_action(
                project,
                DROP_ACTION_BY_MASK[step.unload_mask],
                arrival_state_id=f"DROP_STEP_{step.step_index}",
                unload_mask=step.unload_mask.value,
                target_ranks=list(step.target_ranks),
                bean_types=list(step.bean_types),
                physical_sites=list(step.physical_sites),
                vehicle_bins=list(step.vehicle_bins),
                anchor_site=step.anchor_site,
                unload_profile_hash=canonical_json_crc32_hex(_unload_profile(project, step.unload_mask)),
                unload_estimated_action_time_ms=int(_unload_profile(project, step.unload_mask).get("estimated_action_time_ms", 0)),
            )
        )
        for vehicle_bin, bean_type in zip(step.vehicle_bins, step.bean_types, strict=True):
            del stored_by_bin[vehicle_bin]
            dropped_beans.append(bean_type)

    if stored_by_bin:
        raise CompileError(f"stored beans were not unloaded: {stored_by_bin}")
    if sorted(dropped_beans) != sorted(str(value) for value in row.pick_assignment.values()):
        raise CompileError(f"dropped beans do not match picked beans: {dropped_beans}")

    sequenced = tuple(dict(action, action_seq=index) for index, action in enumerate(actions))
    estimate = sum(int(action.get("estimated_time_ms", 0)) for action in sequenced)
    return SourceActionCompileResult(actions=sequenced, estimated_mechanism_time_ms=estimate)


def _source_action(project: ProjectV40, action_code: ActionCode, **semantic: Any) -> dict[str, Any]:
    profile = _action_profile(project, action_code.name)
    mode = _mode_name(profile.get("mode"))
    post_wait_ms = int(profile.get("post_wait_ms", 0))
    estimate = int(profile.get("estimated_time_ms", semantic.pop("unload_estimated_action_time_ms", 0))) + post_wait_ms
    action = {
        "action": action_code.name,
        "mode": mode,
        "profile_key": action_code.name,
        "profile_hash": canonical_json_crc32_hex(profile),
        "timeout_ms": int(profile.get("timeout_ms", 1000)),
        "post_wait_ms": post_wait_ms,
        "estimated_time_ms": estimate,
    }
    action.update(semantic)
    if "arrival_id" in action or "check_start_s_mm" in action:
        raise CompileError("source actions must not contain arrival_id or check_start_s_mm")
    return action


def _action_profile(project: ProjectV40, profile_key: str) -> dict[str, Any]:
    if profile_key not in project.action_profiles:
        raise CompileError(f"missing action_profile: {profile_key}")
    profile = project.action_profiles[profile_key]
    if not isinstance(profile, dict):
        raise CompileError(f"action_profile {profile_key} must be an object")
    if "mode" not in profile:
        raise CompileError(f"action_profile {profile_key} missing required field: mode")
    return dict(profile)


def _unload_profile(project: ProjectV40, unload_mask: UnloadMask) -> dict[str, Any]:
    if unload_mask.value not in project.unload_profiles:
        raise CompileError(f"missing unload_profile: {unload_mask.value}")
    profile = project.unload_profiles[unload_mask.value]
    if not isinstance(profile, dict):
        raise CompileError(f"unload_profile {unload_mask.value} must be an object")
    return dict(profile)


def _mode_name(value: object) -> str:
    if isinstance(value, ActionMode):
        return value.name
    if isinstance(value, int) and not isinstance(value, bool):
        return ActionMode(value).name
    if isinstance(value, str):
        name = value
        if name.startswith("ACTION_MODE_"):
            name = name[len("ACTION_MODE_") :]
        return ActionMode[name].name
    raise CompileError(f"unsupported action mode: {value!r}")
