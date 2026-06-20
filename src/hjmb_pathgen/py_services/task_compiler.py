"""Phase 3 task compiler from route-table rows to reviewable Case drafts."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_domain.enums import GenerationMode, RouteFamily, StorageMode, UnloadMask, YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, RouteCaseRowV40
from hjmb_pathgen.py_domain.task_mapping import DropTarget, drop_targets_from_label_positions
from hjmb_pathgen.py_domain.task_plan import CandidatePlan, TransitionRequirement, UnloadStep
from hjmb_pathgen.py_services.action_source_compiler import compile_source_actions
from hjmb_pathgen.py_planning.geometry.automatic_topology import topology_profile_for_transition
from hjmb_pathgen.py_utils.yaw_unwrap import unwrap_yaw_sequence

ROUTE_FAMILY_ORDER = (RouteFamily.PICK_1_TO_3, RouteFamily.PICK_3_TO_1)
SINGLE_UNLOAD_MASKS = (UnloadMask.BIN_1, UnloadMask.BIN_2, UnloadMask.BIN_3)
DUAL_UNLOAD_MASKS = (UnloadMask.BIN_12, UnloadMask.BIN_23)
ALLOWED_UNLOAD_MASKS = SINGLE_UNLOAD_MASKS + DUAL_UNLOAD_MASKS


@dataclass(frozen=True)
class TaskCandidateSet:
    row: RouteCaseRowV40
    drop_targets: tuple[DropTarget, ...]
    candidates: tuple[CandidatePlan, ...]
    unavailable_reasons: tuple[str, ...]


@dataclass(frozen=True)
class CaseDraftBuildResult:
    case: CaseManifestV40
    candidate_set: TaskCandidateSet
    selected_candidate: CandidatePlan
    transition_requirements: tuple[TransitionRequirement, ...]


@dataclass(frozen=True)
class _DropItem:
    target: DropTarget
    vehicle_bin: str


@dataclass(frozen=True)
class _UnloadSpec:
    steps: tuple[tuple[_DropItem, ...], ...]
    label: str


@dataclass(frozen=True)
class _RouteBinding:
    route_family: RouteFamily
    pickup_position_order: tuple[str, ...]
    pickup_arrival_state_order: tuple[str, ...]
    drop_target_rank_order: tuple[int, ...]
    sweep_vehicle_bins: tuple[str, ...]
    yaw_direction: YawPolicy


def compile_task_candidates(row: RouteCaseRowV40, project: ProjectV40) -> TaskCandidateSet:
    drop_targets = drop_targets_from_label_positions(row.label_positions)
    candidates: list[CandidatePlan] = []
    unavailable_reasons: list[str] = []
    for route_family in ROUTE_FAMILY_ORDER:
        binding = _route_binding(route_family)
        target_by_rank = {target.target_rank: target for target in drop_targets}
        vehicle_bin_assignment = _vehicle_bin_assignment(binding, target_by_rank)
        baseline = tuple(_DropItem(target=target_by_rank[rank], vehicle_bin=vehicle_bin) for rank, vehicle_bin in zip(binding.drop_target_rank_order, binding.sweep_vehicle_bins, strict=True))
        for unload_spec in _unload_specs(project, binding, baseline, unavailable_reasons):
            try:
                candidates.append(_candidate_from_unload_spec(project, row, drop_targets, binding, vehicle_bin_assignment, unload_spec))
            except CompileError as exc:
                unavailable_reasons.append(f"{route_family.name}/{unload_spec.label}: {exc}")
    candidates = sorted(candidates, key=_candidate_sort_key)
    return TaskCandidateSet(row=row, drop_targets=drop_targets, candidates=tuple(candidates), unavailable_reasons=tuple(unavailable_reasons))


def automatic_candidate_subset(candidates: tuple[CandidatePlan, ...] | list[CandidatePlan]) -> tuple[CandidatePlan, ...]:
    """Return only task-optimal candidates that obey the user's left/right rule.

    The unloading plan is decided before route geometry.  Fewer unloading stops
    are preferred.  For that best stop count, stops {1,2} use the right route
    (pickup sequence ending at PICK_1); stops {2,3} use the left route (ending
    at PICK_3); {1,2,3}, {1,3}, and all ties default to the left route.
    """

    items = tuple(candidates)
    if not items:
        return ()
    minimum_stops = min(item.stop_count for item in items)
    reduced = tuple(item for item in items if item.stop_count == minimum_stops)
    matched = tuple(
        item for item in reduced
        if item.route_family == preferred_route_family_for_candidate(item)
    )
    return tuple(sorted(matched or reduced, key=_candidate_sort_key))


def unload_stop_ranks(candidate: CandidatePlan) -> tuple[int, ...]:
    result: list[int] = []
    for step in candidate.unload_sequence:
        rank = None
        for index, site in enumerate(step.physical_sites):
            if site == step.anchor_site and index < len(step.target_ranks):
                rank = int(step.target_ranks[index])
                break
        if rank is None and step.target_ranks:
            rank = int(step.target_ranks[0])
        if rank is not None:
            result.append(rank)
    return tuple(sorted(set(result)))


def preferred_route_family_for_candidate(candidate: CandidatePlan) -> RouteFamily:
    ranks = set(unload_stop_ranks(candidate))
    if ranks == {1, 2}:
        # User naming: pickup-1 to drop-1 is the right-hand traversal.
        return RouteFamily.PICK_3_TO_1
    if ranks == {2, 3}:
        return RouteFamily.PICK_1_TO_3
    # {1,2,3}, {1,3}, and any ambiguous arrangement are equal; default left.
    return RouteFamily.PICK_1_TO_3


def route_selection_reason(candidate: CandidatePlan) -> str:
    ranks = set(unload_stop_ranks(candidate))
    if ranks == {1, 2}:
        return "RIGHT_ROUTE_FOR_UNLOAD_STOPS_1_2"
    if ranks == {2, 3}:
        return "LEFT_ROUTE_FOR_UNLOAD_STOPS_2_3"
    return "LEFT_ROUTE_TIE_DEFAULT"


def build_case_draft(
    row: RouteCaseRowV40,
    project: ProjectV40,
    *,
    existing_case: CaseManifestV40 | None = None,
    preferred_candidate_id: str | None = None,
    lock_selected: bool | None = None,
) -> CaseDraftBuildResult:
    candidate_set = compile_task_candidates(row, project)
    if not candidate_set.candidates:
        raise CompileError(f"P{row.traj_id:04d} has no legal Phase 3 candidate: {candidate_set.unavailable_reasons}")
    selected = _select_candidate(candidate_set.candidates, existing_case, preferred_candidate_id, lock_selected)
    raw_start_state = _start_state_from_project(project)
    raw_arrival_states = _arrival_states(project, selected)
    start_state, arrival_states = _resolve_unconstrained_route_yaws(raw_start_state, raw_arrival_states)
    logical_points = _logical_points(
        project,
        candidate_set.drop_targets,
        selected,
        resolved_start_state=start_state,
        resolved_arrival_states=arrival_states,
    )
    selected_plan = _selected_plan_dict(candidate_set, selected)
    selected_plan["unconstrained_yaw_policy"] = "KEEP_PREVIOUS_OR_NEXT_EXPLICIT"
    source_mapping = _source_mapping_dict(row, candidate_set.drop_targets)
    hashes = _case_hashes(row, project, selected, selected_plan, source_mapping)
    case = CaseManifestV40(
        storage_mode=StorageMode.REFERENCED,
        generation_mode=GenerationMode.FULL_AUTO,
        traj_id=row.traj_id,
        bean_code=row.bean_code,
        drop_code=row.drop_code,
        source_mapping=source_mapping,
        selected_plan=selected_plan,
        logical_points=logical_points,
        arrival_states=arrival_states,
        leg_refs=(),
        actions={"source": list(selected.source_actions), "compiled": []},
        finish=dict(project.finish_policy),
        estimates={
            "planned_motion_time_ms": 0,
            "planned_motion_time_state": "UNKNOWN_PHASE3_NO_OPTIMIZED_LEGS",
            "planned_mechanism_time_ms": selected.estimated_mechanism_time_ms,
            "planned_total_estimate_ms": selected.estimated_mechanism_time_ms,
            "planned_total_estimate_state": "MECHANISM_ONLY_PHASE3",
        },
        hashes=hashes,
        manual_path=None,
        review={
            "state": "DRAFT",
            "approved": False,
            "detached_from_library": False,
            "manual_override": False,
            "incomplete_reason": "INCOMPLETE_LEGS",
            "locked_conflict": False,
            "override_reason": "",
        },
    )
    requirements = transition_requirements_for_case(case, project)
    return CaseDraftBuildResult(case=case, candidate_set=candidate_set, selected_candidate=selected, transition_requirements=requirements)


def transition_requirements_for_case(case: CaseManifestV40, project: ProjectV40) -> tuple[TransitionRequirement, ...]:
    states = [_start_state_for_case(case, project), *case.arrival_states]
    route_family = str(case.selected_plan.get("route_family", ""))
    dependency_hashes = _transition_dependency_hashes(project)
    requirements: list[TransitionRequirement] = []
    for from_state, to_state in zip(states, states[1:], strict=False):
        topology_profile = topology_profile_for_transition(
            project,
            route_family,
            str(from_state["state_id"]),
            str(to_state["state_id"]),
        )
        semantic = {
            "from_state_id": from_state["state_id"],
            "to_state_id": to_state["state_id"],
            "route_family": route_family,
            "topology_profile": topology_profile,
            "from_pose": from_state["pose"],
            "to_pose": to_state["pose"],
            "dependency_hashes": dependency_hashes,
            "reason": "TASK_SEQUENCE",
        }
        semantic_hash = canonical_json_crc32_hex(semantic)
        requirements.append(
            TransitionRequirement(
                requirement_id=f"TR_{semantic_hash[:8].upper()}",
                semantic_hash=semantic_hash,
                from_state_id=str(from_state["state_id"]),
                to_state_id=str(to_state["state_id"]),
                route_family=route_family,
                topology_profile=topology_profile,
                from_pose=dict(from_state["pose"]),
                to_pose=dict(to_state["pose"]),
                dependency_hashes=dependency_hashes,
                reason="TASK_SEQUENCE",
            )
        )
    return tuple(requirements)


def candidate_review_dict(candidate: CandidatePlan) -> dict[str, Any]:
    return {
        "candidate_id": candidate.candidate_id,
        "semantic_hash": candidate.semantic_hash,
        "route_family": candidate.route_family.name,
        "pickup_position_order": list(candidate.pickup_position_order),
        "pickup_arrival_state_order": list(candidate.pickup_arrival_state_order),
        "drop_target_rank_order": list(candidate.drop_target_rank_order),
        "vehicle_bin_assignment": dict(candidate.vehicle_bin_assignment),
        "unload_sequence": [step.to_dict() for step in candidate.unload_sequence],
        "yaw_direction": candidate.yaw_direction.value,
        "yaw_sequence_ddeg": list(candidate.yaw_sequence_ddeg),
        "estimated_mechanism_time_ms": candidate.estimated_mechanism_time_ms,
        "stop_count": candidate.stop_count,
        "unload_stop_ranks": list(unload_stop_ranks(candidate)),
        "preferred_route_family": preferred_route_family_for_candidate(candidate).name,
        "route_selection_reason": route_selection_reason(candidate),
        "route_rule_match": candidate.route_family == preferred_route_family_for_candidate(candidate),
        "warnings": list(candidate.warnings),
        "unavailable_reasons": list(candidate.unavailable_reasons),
        "locked_by_user": candidate.locked_by_user,
    }


def _route_binding(route_family: RouteFamily) -> _RouteBinding:
    if route_family == RouteFamily.PICK_1_TO_3:
        return _RouteBinding(
            route_family=route_family,
            pickup_position_order=("PICK_1", "PICK_2", "PICK_3"),
            pickup_arrival_state_order=("P_PICK_1", "P_PICK_2L", "P_PICK_3"),
            drop_target_rank_order=(3, 2, 1),
            sweep_vehicle_bins=("BIN_1", "BIN_2", "BIN_3"),
            yaw_direction=YawPolicy.CW_ONLY,
        )
    if route_family == RouteFamily.PICK_3_TO_1:
        return _RouteBinding(
            route_family=route_family,
            pickup_position_order=("PICK_3", "PICK_2", "PICK_1"),
            pickup_arrival_state_order=("P_PICK_3", "P_PICK_2R", "P_PICK_1"),
            drop_target_rank_order=(1, 2, 3),
            sweep_vehicle_bins=("BIN_3", "BIN_2", "BIN_1"),
            yaw_direction=YawPolicy.CCW_ONLY,
        )
    raise CompileError(f"unsupported automatic route family: {route_family}")


def _vehicle_bin_assignment(binding: _RouteBinding, target_by_rank: dict[int, DropTarget]) -> dict[str, str]:
    assignment: dict[str, str] = {}
    for rank, vehicle_bin in zip(binding.drop_target_rank_order, binding.sweep_vehicle_bins, strict=True):
        bean_type = target_by_rank[rank].bean_type.value
        if bean_type in assignment:
            raise CompileError(f"bean {bean_type} is assigned to more than one vehicle bin")
        assignment[bean_type] = vehicle_bin
    if sorted(assignment) != ["GREEN", "WHITE", "YELLOW"] or sorted(assignment.values()) != ["BIN_1", "BIN_2", "BIN_3"]:
        raise CompileError(f"vehicle_bin_assignment must be a bean-to-bin bijection: {assignment}")
    return assignment


def _unload_specs(project: ProjectV40, binding: _RouteBinding, baseline: tuple[_DropItem, ...], unavailable_reasons: list[str]) -> tuple[_UnloadSpec, ...]:
    specs = [_UnloadSpec(steps=tuple((item,) for item in baseline), label="single")]
    for pair_index in (0, 1):
        first = baseline[pair_index]
        second = baseline[pair_index + 1]
        if abs(first.target.physical_order_index - second.target.physical_order_index) != 1:
            continue
        dual_mask = _dual_mask_for_bins((first.vehicle_bin, second.vehicle_bin))
        reason_prefix = f"{binding.route_family.name}/{dual_mask.value}/pair{pair_index + 1}"
        if not _unload_profile_available(project, dual_mask):
            unavailable_reasons.append(f"{reason_prefix}: missing or uncalibrated manual yaw profile")
            continue
        steps = list(tuple((item,) for item in baseline))
        steps[pair_index] = (first, second)
        del steps[pair_index + 1]
        specs.append(_UnloadSpec(steps=tuple(steps), label=f"dual_{dual_mask.value}_pair{pair_index + 1}"))
    return tuple(specs)


def _candidate_from_unload_spec(
    project: ProjectV40,
    row: RouteCaseRowV40,
    drop_targets: tuple[DropTarget, ...],
    binding: _RouteBinding,
    vehicle_bin_assignment: dict[str, str],
    unload_spec: _UnloadSpec,
) -> CandidatePlan:
    unload_sequence = _unload_sequence(project, unload_spec)
    yaw_sequence = unwrap_yaw_sequence(tuple(step.yaw_ddeg for step in unload_sequence), binding.yaw_direction)
    source_actions = compile_source_actions(
        project,
        row,
        pickup_position_order=binding.pickup_position_order,
        pickup_arrival_state_order=binding.pickup_arrival_state_order,
        vehicle_bin_assignment=vehicle_bin_assignment,
        unload_sequence=unload_sequence,
    )
    semantic = {
        "version": "HJMB_PHASE3_CANDIDATE_V1",
        "traj_id": row.traj_id,
        "bean_code": row.bean_code,
        "drop_code": row.drop_code,
        "source_row_hash": row.source_row_hash,
        "route_family": binding.route_family.name,
        "pickup_position_order": list(binding.pickup_position_order),
        "pickup_arrival_state_order": list(binding.pickup_arrival_state_order),
        "drop_targets": [target.to_dict() for target in drop_targets],
        "drop_target_rank_order": list(binding.drop_target_rank_order),
        "vehicle_bin_assignment": vehicle_bin_assignment,
        "unload_sequence": [step.to_dict() for step in unload_sequence],
        "yaw_direction": binding.yaw_direction.value,
        "yaw_sequence_ddeg": list(yaw_sequence),
        "source_actions": list(source_actions.actions),
    }
    semantic_hash = canonical_json_crc32_hex(semantic)
    return CandidatePlan(
        candidate_id=f"C_{binding.route_family.name}_{semantic_hash[:8].upper()}",
        semantic_hash=semantic_hash,
        traj_id=row.traj_id,
        route_family=binding.route_family,
        pickup_position_order=binding.pickup_position_order,
        pickup_arrival_state_order=binding.pickup_arrival_state_order,
        drop_target_rank_order=binding.drop_target_rank_order,
        vehicle_bin_assignment=dict(vehicle_bin_assignment),
        unload_sequence=unload_sequence,
        yaw_direction=binding.yaw_direction,
        yaw_sequence_ddeg=yaw_sequence,
        source_actions=source_actions.actions,
        estimated_mechanism_time_ms=source_actions.estimated_mechanism_time_ms,
    )


def _unload_sequence(project: ProjectV40, unload_spec: _UnloadSpec) -> tuple[UnloadStep, ...]:
    steps: list[UnloadStep] = []
    for step_index, items in enumerate(unload_spec.steps, start=1):
        vehicle_bins = tuple(item.vehicle_bin for item in items)
        unload_mask = _mask_for_step(vehicle_bins)
        if unload_mask not in ALLOWED_UNLOAD_MASKS:
            raise CompileError(f"illegal unload mask: {unload_mask}")
        if not _unload_profile_available(project, unload_mask):
            raise CompileError(f"missing or uncalibrated unload_profile: {unload_mask.value}")
        profile = _unload_profile(project, unload_mask)
        anchor_site = _anchor_site(unload_mask, items)
        steps.append(
            UnloadStep(
                step_index=step_index,
                unload_mask=unload_mask,
                target_ranks=tuple(item.target.target_rank for item in items),
                bean_types=tuple(item.target.bean_type.value for item in items),
                physical_sites=tuple(item.target.physical_site.value for item in items),
                vehicle_bins=vehicle_bins,
                anchor_site=anchor_site,
                yaw_ddeg=int(profile["yaw_ddeg"]),
            )
        )
    return tuple(steps)


def _mask_for_step(vehicle_bins: tuple[str, ...]) -> UnloadMask:
    if len(vehicle_bins) == 1:
        return UnloadMask(vehicle_bins[0])
    if len(vehicle_bins) == 2:
        return _dual_mask_for_bins(vehicle_bins)
    raise CompileError(f"unload step must contain one or two bins: {vehicle_bins}")


def _dual_mask_for_bins(vehicle_bins: tuple[str, ...]) -> UnloadMask:
    key = frozenset(vehicle_bins)
    if key == frozenset(("BIN_1", "BIN_2")):
        return UnloadMask.BIN_12
    if key == frozenset(("BIN_2", "BIN_3")):
        return UnloadMask.BIN_23
    raise CompileError(f"non-adjacent dual unload is forbidden: {vehicle_bins}")


def _anchor_site(unload_mask: UnloadMask, items: tuple[_DropItem, ...]) -> str:
    if unload_mask == UnloadMask.BIN_12:
        return _site_for_bin(items, "BIN_1")
    if unload_mask == UnloadMask.BIN_23:
        return _site_for_bin(items, "BIN_2")
    return items[0].target.physical_site.value


def _site_for_bin(items: tuple[_DropItem, ...], vehicle_bin: str) -> str:
    for item in items:
        if item.vehicle_bin == vehicle_bin:
            return item.target.physical_site.value
    raise CompileError(f"{vehicle_bin} is missing from dual unload step")


def _unload_profile_available(project: ProjectV40, unload_mask: UnloadMask) -> bool:
    try:
        profile = _unload_profile(project, unload_mask)
    except CompileError:
        return False
    if "yaw_ddeg" not in profile:
        return False
    if profile.get("configured", False) is False:
        return False
    int(profile["yaw_ddeg"])
    return True


def _unload_profile(project: ProjectV40, unload_mask: UnloadMask) -> dict[str, Any]:
    if unload_mask.value not in project.unload_profiles:
        raise CompileError(f"missing unload_profile: {unload_mask.value}")
    profile = project.unload_profiles[unload_mask.value]
    if not isinstance(profile, dict):
        raise CompileError(f"unload_profile {unload_mask.value} must be an object")
    return dict(profile)


def _candidate_sort_key(candidate: CandidatePlan) -> tuple[int, int, tuple[str, ...], str]:
    return (
        int(candidate.route_family),
        candidate.stop_count,
        tuple(step.unload_mask.value for step in candidate.unload_sequence),
        candidate.candidate_id,
    )


def _select_candidate(
    candidates: tuple[CandidatePlan, ...],
    existing_case: CaseManifestV40 | None,
    preferred_candidate_id: str | None,
    lock_selected: bool | None,
) -> CandidatePlan:
    by_id = {candidate.candidate_id: candidate for candidate in candidates}
    locked_by_user = False
    candidate_id = preferred_candidate_id
    expected_hash = None
    if existing_case is not None:
        if existing_case.review.get("detached_from_library") or existing_case.review.get("approved") or existing_case.review.get("manual_override"):
            raise CompileError(f"P{existing_case.traj_id:04d} is approved/detached/manual and must not be overwritten by Phase 3")
        selected_plan = existing_case.selected_plan
        if selected_plan.get("locked_by_user") and candidate_id is None:
            candidate_id = str(selected_plan.get("candidate_id", ""))
            expected_hash = str(selected_plan.get("semantic_hash", ""))
            locked_by_user = True
    if candidate_id is None:
        automatic = automatic_candidate_subset(candidates)
        selected = automatic[0] if automatic else candidates[0]
    else:
        selected = by_id.get(candidate_id)
        if selected is None:
            raise CompileError(f"selected candidate is not valid for current task semantics: {candidate_id}")
        if expected_hash and selected.semantic_hash != expected_hash:
            raise CompileError(f"locked candidate semantic hash changed for {candidate_id}")
    if lock_selected is not None:
        locked_by_user = bool(lock_selected)
    return replace(selected, locked_by_user=locked_by_user)


def _arrival_states(project: ProjectV40, selected: CandidatePlan) -> tuple[dict[str, Any], ...]:
    states: list[dict[str, Any]] = []
    for pickup_state, pickup_slot in zip(selected.pickup_arrival_state_order, selected.pickup_position_order, strict=True):
        states.append(
            {
                "state_id": pickup_state,
                "type": "PICK",
                "site_key": pickup_state,
                "pickup_slot": pickup_slot,
                "pose": _project_site_pose(project, pickup_state),
            }
        )
    for step, yaw_ddeg in zip(selected.unload_sequence, selected.yaw_sequence_ddeg, strict=True):
        states.append(
            {
                "state_id": f"DROP_STEP_{step.step_index}",
                "type": "DROP",
                "site_key": step.anchor_site,
                "physical_drop_sites": list(step.physical_sites),
                "target_ranks": list(step.target_ranks),
                "bean_types": list(step.bean_types),
                "vehicle_bins": list(step.vehicle_bins),
                "unload_mask": step.unload_mask.value,
                "pose": _drop_pose(project, step.anchor_site, step.unload_mask, yaw_ddeg),
            }
        )
    return tuple(states)


def _logical_points(
    project: ProjectV40,
    drop_targets: tuple[DropTarget, ...],
    selected: CandidatePlan,
    *,
    resolved_start_state: dict[str, Any] | None = None,
    resolved_arrival_states: tuple[dict[str, Any], ...] = (),
) -> tuple[dict[str, Any], ...]:
    resolved_pose_by_id = {
        str(item.get("state_id", "")): dict(item.get("pose", {}))
        for item in resolved_arrival_states
    }
    if resolved_start_state is not None:
        resolved_pose_by_id["P_START"] = dict(resolved_start_state.get("pose", {}))
    points = [
        {
            "point_id": site_key,
            "type": "TASK_ANCHOR",
            "pose": dict(resolved_pose_by_id.get(site_key, _project_site_pose(project, site_key))),
        }
        for site_key in ("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3")
    ]
    step_by_rank = {
        target_rank: step
        for step in selected.unload_sequence
        for target_rank in step.target_ranks
    }
    for target in sorted(drop_targets, key=lambda item: item.target_rank):
        step = step_by_rank[target.target_rank]
        points.append(
            {
                "point_id": f"P_DROP_{target.target_rank}",
                "type": "TASK_ANCHOR",
                "physical_drop_site": target.physical_site.value,
                "target_rank": target.target_rank,
                "bean_type": target.bean_type.value,
                "unload_mask": step.unload_mask.value,
                "pose": _drop_pose(project, step.anchor_site, step.unload_mask, step.yaw_ddeg),
            }
        )
    return tuple(points)


def _drop_pose(project: ProjectV40, anchor_site: str, unload_mask: UnloadMask, yaw_ddeg: int) -> dict[str, Any]:
    site = _physical_drop_box(project, anchor_site)
    profile = _unload_profile(project, unload_mask)
    base_x, base_y = _drop_approach_xy(project, site)
    return {
        "x_mm": round(base_x) + int(profile.get("dx_mm", 0)),
        "y_mm": round(base_y) + int(profile.get("dy_mm", 0)),
        "yaw_ddeg": int(yaw_ddeg),
        "anchor_site": anchor_site,
        "unload_mask": unload_mask.value,
    }


def _physical_drop_box(project: ProjectV40, physical_site: str) -> dict[str, Any]:
    for item in project.field_objects.get("drop_boxes", []):
        if str(item.get("physical_drop_site")) == physical_site:
            if not item.get("configured", False):
                raise CompileError(f"physical drop box is not configured: {physical_site}")
            return dict(item)
    raise CompileError(f"physical drop box is missing: {physical_site}")


def _drop_approach_xy(project: ProjectV40, target: dict[str, Any]) -> tuple[float, float]:
    boxes = [item for item in project.field_objects.get("drop_boxes", []) if item.get("configured", False)]
    if not boxes:
        raise CompileError("no configured physical drop boxes")
    centroid_x = sum(float(item["center_x_mm"]) for item in boxes) / len(boxes)
    centroid_y = sum(float(item["center_y_mm"]) for item in boxes) / len(boxes)
    x_span = max(float(item["center_x_mm"]) for item in boxes) - min(float(item["center_x_mm"]) for item in boxes)
    y_span = max(float(item["center_y_mm"]) for item in boxes) - min(float(item["center_y_mm"]) for item in boxes)
    if x_span >= y_span:
        normal_x, normal_y = 0.0, -1.0 if centroid_y > 0 else 1.0
    else:
        normal_x, normal_y = (-1.0 if centroid_x > 0 else 1.0), 0.0
    box_yaw = math.radians(float(target.get("yaw_ddeg", 0)) / 10.0)
    axis_u = (math.cos(box_yaw), math.sin(box_yaw))
    axis_v = (-axis_u[1], axis_u[0])
    support = (
        0.5 * float(target["length_mm"]) * abs(normal_x * axis_u[0] + normal_y * axis_u[1])
        + 0.5 * float(target["width_mm"]) * abs(normal_x * axis_v[0] + normal_y * axis_v[1])
    )
    footprint = project.vehicle.get("footprint", {})
    clearance = float(footprint.get("r_small_mm", 0)) + float(footprint.get("numerical_epsilon_mm", 0))
    distance = support + clearance
    return (
        float(target["center_x_mm"]) + normal_x * distance,
        float(target["center_y_mm"]) + normal_y * distance,
    )


def _project_site_pose(project: ProjectV40, site_key: str) -> dict[str, Any]:
    if site_key not in project.sites:
        raise CompileError(f"project site is missing: {site_key}")
    site = dict(project.sites[site_key])
    if not site.get("configured", False):
        raise CompileError(f"project site is not configured: {site_key}")
    return {
        "x_mm": int(site.get("x_mm", 0)),
        "y_mm": int(site.get("y_mm", 0)),
        "yaw_ddeg": int(site.get("yaw_ddeg", 0)),
        "site_key": site_key,
    }


def _selected_plan_dict(candidate_set: TaskCandidateSet, selected: CandidatePlan) -> dict[str, Any]:
    return {
        "candidate_id": selected.candidate_id,
        "semantic_hash": selected.semantic_hash,
        "route_family": selected.route_family.name,
        "pickup_position_order": list(selected.pickup_position_order),
        "pickup_arrival_state_order": list(selected.pickup_arrival_state_order),
        "drop_target_rank_order": list(selected.drop_target_rank_order),
        "drop_targets": [target.to_dict() for target in candidate_set.drop_targets],
        "vehicle_bin_assignment": dict(selected.vehicle_bin_assignment),
        "unload_sequence": [step.to_dict() for step in selected.unload_sequence],
        "yaw_direction": selected.yaw_direction.value,
        "yaw_sequence_ddeg": list(selected.yaw_sequence_ddeg),
        "estimated_mechanism_time_ms": selected.estimated_mechanism_time_ms,
        "unload_stop_ranks": list(unload_stop_ranks(selected)),
        "preferred_route_family": preferred_route_family_for_candidate(selected).name,
        "route_selection_reason": route_selection_reason(selected),
        "locked_by_user": selected.locked_by_user,
        "selection_state": "LOCKED" if selected.locked_by_user else "DETERMINISTIC_ROUTE_RULE",
        "candidates": [candidate_review_dict(candidate) for candidate in candidate_set.candidates],
        "unavailable_reasons": list(candidate_set.unavailable_reasons),
    }


def _source_mapping_dict(row: RouteCaseRowV40, drop_targets: tuple[DropTarget, ...]) -> dict[str, Any]:
    data = {
        "traj_id": row.traj_id,
        "file_name": row.file_name,
        "bean_code": row.bean_code,
        "drop_code": row.drop_code,
        "pick_assignment": dict(row.pick_assignment),
        "label_positions": dict(row.label_positions),
        "drop_targets": [target.to_dict() for target in drop_targets],
        "source_row_hash": row.source_row_hash,
    }
    if row.source_row_number is not None:
        data["source_row_number"] = row.source_row_number
    if row.raw_fields is not None:
        data["raw_fields"] = dict(row.raw_fields)
    return data


def _case_hashes(
    row: RouteCaseRowV40,
    project: ProjectV40,
    selected: CandidatePlan,
    selected_plan: dict[str, Any],
    source_mapping: dict[str, Any],
) -> dict[str, Any]:
    task_semantic = {
        "row": source_mapping,
        "selected_plan": {key: value for key, value in selected_plan.items() if key != "candidates"},
        "project_task_config_hash": _project_task_config_hash(project),
    }
    return {
        "source_row_hash": row.source_row_hash,
        "selected_candidate_semantic_hash": selected.semantic_hash,
        "project_task_config_hash": _project_task_config_hash(project),
        "task_semantic_hash": canonical_json_crc32_hex(task_semantic),
    }


def _project_task_config_hash(project: ProjectV40) -> str:
    return canonical_json_crc32_hex(
        {
            "project_id": project.project_id,
            "sites": project.sites,
            "unload_profiles": project.unload_profiles,
            "action_profiles": project.action_profiles,
            "finish_policy": project.finish_policy,
            "topology_profiles": project.topology_profiles,
        }
    )


def _start_state_from_project(project: ProjectV40) -> dict[str, Any]:
    return {"state_id": "P_START", "type": "START", "pose": _project_site_pose(project, "P_START")}


def _start_state_for_case(case: CaseManifestV40, project: ProjectV40) -> dict[str, Any]:
    for point in case.logical_points:
        if str(point.get("point_id", "")) == "P_START":
            pose = point.get("pose")
            if isinstance(pose, dict):
                return {"state_id": "P_START", "type": "START", "pose": dict(pose)}
    return _start_state_from_project(project)


def _resolve_unconstrained_route_yaws(
    start_state: dict[str, Any],
    arrival_states: tuple[dict[str, Any], ...],
) -> tuple[dict[str, Any], tuple[dict[str, Any], ...]]:
    """Resolve project yaw 0xFFFF only for planning; source JSON stays untouched.

    An unconstrained arrival inherits the preceding resolved yaw, which is the
    minimum-rotation solution.  When P_START itself is unconstrained, it uses
    the first later explicit yaw (or zero when the whole route is free).
    """

    states = [
        {**start_state, "pose": dict(start_state.get("pose", {}))},
        *({**item, "pose": dict(item.get("pose", {}))} for item in arrival_states),
    ]
    raw_yaws = [int(item["pose"].get("yaw_ddeg", 0)) for item in states]
    first_explicit = next((value for value in raw_yaws if value != YAW_UNSPECIFIED_DDEG), 0)
    current = first_explicit
    for item, raw_yaw in zip(states, raw_yaws, strict=True):
        if raw_yaw != YAW_UNSPECIFIED_DDEG:
            current = raw_yaw
        item["pose"]["yaw_ddeg"] = int(current)
        if raw_yaw == YAW_UNSPECIFIED_DDEG:
            item["pose"]["yaw_source"] = "UNCONSTRAINED_0XFFFF"
    return states[0], tuple(states[1:])


def _transition_dependency_hashes(project: ProjectV40) -> dict[str, Any]:
    dependencies = {
        "sites": project.sites,
        "vehicle": project.vehicle,
        "dynamics": project.dynamics,
        "field_objects": project.field_objects,
        "unload_profiles": project.unload_profiles,
        "topology_profiles": project.topology_profiles,
    }
    return {key: canonical_json_crc32_hex(value) for key, value in dependencies.items()}
