"""Phase 3 task compiler from route-table rows to reviewable Case drafts."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_domain.competition_task_config import CompetitionTaskConfigV40
from hjmb_pathgen.py_domain.enums import GenerationMode, RouteFamily, StorageMode, UnloadMask, YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, RouteCaseRowV40
from hjmb_pathgen.py_domain.task_mapping import DropTarget, drop_targets_from_label_positions
from hjmb_pathgen.py_domain.task_plan import CandidatePlan, TransitionRequirement, UnloadStep
from hjmb_pathgen.py_services.action_source_compiler import compile_source_actions
from hjmb_pathgen.py_services.competition_task_config_service import default_competition_task_config
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
class _RouteBinding:
    route_family: RouteFamily
    pickup_position_order: tuple[str, ...]
    pickup_arrival_state_order: tuple[str, ...]
    drop_station_order: tuple[str, ...]
    yaw_direction: YawPolicy


def compile_task_candidates(
    row: RouteCaseRowV40,
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40 | None = None,
) -> TaskCandidateSet:
    """Compile all physically legal task candidates for one draw result.

    The competition rule JSON is the source of truth.  The compiler enumerates
    the six bean-to-bin bijections, rejects assignments that cannot reach the
    target physical boxes, recognises only the two real dual-unload corners
    F4+F5/BIN_12 and F7+F8/BIN_23, and only then selects a left/right route.
    """

    config = task_config or default_competition_task_config()
    drop_targets = drop_targets_from_label_positions(row.label_positions)
    target_by_bean = {target.bean_type.value: target for target in drop_targets}
    candidates: list[CandidatePlan] = []
    unavailable_reasons: list[str] = []

    bean_order = ("YELLOW", "GREEN", "WHITE")
    for bins in itertools.permutations(("BIN_1", "BIN_2", "BIN_3")):
        vehicle_bin_assignment = dict(zip(bean_order, bins, strict=True))
        invalid = [
            f"{bean}->{target_by_bean[bean].physical_site.value} cannot use {vehicle_bin_assignment[bean]}"
            for bean in bean_order
            if vehicle_bin_assignment[bean]
            not in config.bin_reachability[target_by_bean[bean].physical_site.value]
        ]
        if invalid:
            continue
        physical_items = {
            target.physical_site.value: _DropItem(
                target=target,
                vehicle_bin=vehicle_bin_assignment[target.bean_type.value],
            )
            for target in drop_targets
        }
        for route_family in ROUTE_FAMILY_ORDER:
            binding = _route_binding(route_family, config)
            try:
                unload_sequence = _build_unload_sequence(
                    project,
                    config,
                    binding,
                    physical_items,
                )
                candidates.append(
                    _candidate_from_unload_sequence(
                        project,
                        row,
                        drop_targets,
                        binding,
                        vehicle_bin_assignment,
                        unload_sequence,
                    )
                )
            except CompileError as exc:
                unavailable_reasons.append(
                    f"{route_family.name}/{_assignment_label(vehicle_bin_assignment)}: {exc}"
                )

    # Different enumerations can theoretically collapse to the same semantic
    # candidate.  Deduplicate before presenting them in the GUI.
    by_hash = {candidate.semantic_hash: candidate for candidate in candidates}
    ordered = tuple(sorted(by_hash.values(), key=_candidate_sort_key))
    return TaskCandidateSet(
        row=row,
        drop_targets=drop_targets,
        candidates=ordered,
        unavailable_reasons=tuple(dict.fromkeys(unavailable_reasons)),
    )


def automatic_candidate_subset(
    candidates: tuple[CandidatePlan, ...] | list[CandidatePlan],
    task_config: CompetitionTaskConfigV40 | None = None,
) -> tuple[CandidatePlan, ...]:
    """Return all minimum-stop candidates for geometric comparison.

    The unloading plan is decided before geometry and fewer unloading stops are
    always preferred.  Left/right route rules are no longer a hard pre-filter:
    both route families must reach Phase 8 so their complete motion time can be
    compared.  The configured route rule is retained only as a deterministic
    tie-break preference.
    """

    items = tuple(candidates)
    if not items:
        return ()
    minimum_stops = min(item.stop_count for item in items)
    reduced = tuple(item for item in items if item.stop_count == minimum_stops)
    return tuple(sorted(reduced, key=_candidate_sort_key))


def unload_stop_ranks(candidate: CandidatePlan) -> tuple[int, ...]:
    """Return physical logical-station numbers (P_DROP_1/2/3)."""

    result: list[int] = []
    for step in candidate.unload_sequence:
        anchor = str(step.anchor_site)
        if anchor.startswith("P_DROP_"):
            result.append(int(anchor.rsplit("_", 1)[1]))
            continue
        # Compatibility for old in-memory test/candidate objects.  Newly
        # compiled cases always store P_DROP_n here.
        if step.target_ranks:
            result.append(int(step.target_ranks[0]))
            continue
        raise CompileError(f"invalid unload station anchor: {anchor}")
    return tuple(sorted(set(result)))


def _station_rule_key(candidate: CandidatePlan) -> str:
    return ",".join(str(value) for value in unload_stop_ranks(candidate))


def preferred_route_family_for_candidate(
    candidate: CandidatePlan,
    task_config: CompetitionTaskConfigV40 | None = None,
) -> RouteFamily:
    config = task_config or default_competition_task_config()
    key = _station_rule_key(candidate)
    rules = dict(config.automatic_selection.get("station_set_route_rules", {}))
    route_name = str(rules.get(key, config.automatic_selection.get("tie_default", "PICK_1_TO_3")))
    try:
        return RouteFamily[route_name]
    except KeyError as exc:
        raise CompileError(f"invalid automatic route rule for unload stations {key}: {route_name}") from exc


def route_selection_reason(
    candidate: CandidatePlan,
    task_config: CompetitionTaskConfigV40 | None = None,
) -> str:
    key = _station_rule_key(candidate)
    preferred = preferred_route_family_for_candidate(candidate, task_config)
    if key == "1,2" and preferred == RouteFamily.PICK_3_TO_1:
        return "RIGHT_ROUTE_FOR_UNLOAD_STOPS_1_2"
    if key == "2,3" and preferred == RouteFamily.PICK_1_TO_3:
        return "LEFT_ROUTE_FOR_UNLOAD_STOPS_2_3"
    return "LEFT_ROUTE_TIE_DEFAULT"

def build_case_draft(
    row: RouteCaseRowV40,
    project: ProjectV40,
    *,
    existing_case: CaseManifestV40 | None = None,
    preferred_candidate_id: str | None = None,
    lock_selected: bool | None = None,
    task_config: CompetitionTaskConfigV40 | None = None,
) -> CaseDraftBuildResult:
    config = task_config or default_competition_task_config()
    candidate_set = compile_task_candidates(row, project, config)
    if not candidate_set.candidates:
        raise CompileError(f"P{row.traj_id:04d} has no legal Phase 3 candidate: {candidate_set.unavailable_reasons}")
    selected = _select_candidate(
        candidate_set.candidates, existing_case, preferred_candidate_id, lock_selected, config
    )
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
    selected_plan = _selected_plan_dict(candidate_set, selected, config)
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


def candidate_review_dict(
    candidate: CandidatePlan,
    task_config: CompetitionTaskConfigV40 | None = None,
) -> dict[str, Any]:
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
        "preferred_route_family": preferred_route_family_for_candidate(candidate, task_config).name,
        "route_selection_reason": route_selection_reason(candidate, task_config),
        "route_rule_match": candidate.route_family == preferred_route_family_for_candidate(candidate, task_config),
        "warnings": list(candidate.warnings),
        "unavailable_reasons": list(candidate.unavailable_reasons),
        "locked_by_user": candidate.locked_by_user,
    }


def _route_binding(
    route_family: RouteFamily,
    task_config: CompetitionTaskConfigV40,
) -> _RouteBinding:
    try:
        raw = task_config.route_families[route_family.name]
    except KeyError as exc:
        raise CompileError(f"task config missing route family: {route_family.name}") from exc
    try:
        return _RouteBinding(
            route_family=route_family,
            pickup_position_order=tuple(str(item) for item in raw["pickup_position_order"]),
            pickup_arrival_state_order=tuple(str(item) for item in raw["pickup_arrival_state_order"]),
            drop_station_order=tuple(str(item) for item in raw["drop_station_order"]),
            yaw_direction=YawPolicy(str(raw["yaw_direction"])),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise CompileError(f"invalid route family config {route_family.name}: {exc}") from exc


def _build_unload_sequence(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    binding: _RouteBinding,
    physical_items: dict[str, _DropItem],
) -> tuple[UnloadStep, ...]:
    """Group target boxes into real physical unload operations.

    There are only two legal dual operations: F4+F5 at P_DROP_3 with
    BIN_1+BIN_2, and F7+F8 at P_DROP_1 with BIN_2+BIN_3.  Other "adjacent"
    targets are never merged.
    """

    steps: list[UnloadStep] = []
    for station in binding.drop_station_order:
        station_spec = task_config.drop_stations.get(station)
        if not isinstance(station_spec, dict):
            raise CompileError(f"task config missing drop station: {station}")
        station_sites = tuple(str(item) for item in station_spec.get("physical_sites", ()))
        items = tuple(physical_items[site] for site in station_sites if site in physical_items)
        if not items:
            continue

        assignments = {item.target.physical_site.value: item.vehicle_bin for item in items}
        profile_id = task_config.pose_profile_for_assignments(station, assignments)
        if profile_id is None:
            raise CompileError(
                f"no legal unload operation for {station}: {assignments}; "
                "only F4+F5/BIN_12 and F7+F8/BIN_23 may unload together"
            )
        profile = _unload_pose_profile(project, profile_id)
        if not bool(profile.get("configured", False)):
            raise CompileError(f"missing or uncalibrated unload pose profile: {profile_id}")
        raw_spec = task_config.unload_pose_catalog[profile_id]
        unload_mask = UnloadMask(str(raw_spec["unload_mask"]))

        ordered_items = tuple(sorted(items, key=lambda item: int(item.vehicle_bin.rsplit("_", 1)[1])))
        steps.append(
            UnloadStep(
                step_index=len(steps) + 1,
                unload_mask=unload_mask,
                target_ranks=tuple(item.target.target_rank for item in ordered_items),
                bean_types=tuple(item.target.bean_type.value for item in ordered_items),
                physical_sites=tuple(item.target.physical_site.value for item in ordered_items),
                vehicle_bins=tuple(item.vehicle_bin for item in ordered_items),
                anchor_site=station,
                yaw_ddeg=int(profile["yaw_ddeg"]),
                unload_pose_profile_id=profile_id,
            )
        )
    if not steps:
        raise CompileError("task has no unload steps")
    if sum(len(step.bean_types) for step in steps) != 3:
        raise CompileError(f"unload sequence does not cover all three beans: {steps}")
    return tuple(steps)


def _candidate_from_unload_sequence(
    project: ProjectV40,
    row: RouteCaseRowV40,
    drop_targets: tuple[DropTarget, ...],
    binding: _RouteBinding,
    vehicle_bin_assignment: dict[str, str],
    unload_sequence: tuple[UnloadStep, ...],
) -> CandidatePlan:
    yaw_sequence = unwrap_yaw_sequence(
        tuple(step.yaw_ddeg for step in unload_sequence),
        binding.yaw_direction,
    )
    source_actions = compile_source_actions(
        project,
        row,
        pickup_position_order=binding.pickup_position_order,
        pickup_arrival_state_order=binding.pickup_arrival_state_order,
        vehicle_bin_assignment=vehicle_bin_assignment,
        unload_sequence=unload_sequence,
    )
    flattened_ranks = (
        (3, 2, 1)
        if binding.route_family == RouteFamily.PICK_1_TO_3
        else (1, 2, 3)
    )
    semantic = {
        "version": "HJMB_TASK_CANDIDATE_V2",
        "traj_id": row.traj_id,
        "bean_code": row.bean_code,
        "drop_code": row.drop_code,
        "source_row_hash": row.source_row_hash,
        "route_family": binding.route_family.name,
        "pickup_position_order": list(binding.pickup_position_order),
        "pickup_arrival_state_order": list(binding.pickup_arrival_state_order),
        "drop_station_order": list(binding.drop_station_order),
        "drop_targets": [target.to_dict() for target in drop_targets],
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
        drop_target_rank_order=flattened_ranks,
        vehicle_bin_assignment=dict(vehicle_bin_assignment),
        unload_sequence=unload_sequence,
        yaw_direction=binding.yaw_direction,
        yaw_sequence_ddeg=yaw_sequence,
        source_actions=source_actions.actions,
        estimated_mechanism_time_ms=source_actions.estimated_mechanism_time_ms,
    )


def _assignment_label(assignment: dict[str, str]) -> str:
    return ",".join(f"{bean}={assignment[bean]}" for bean in ("YELLOW", "GREEN", "WHITE"))


def _unload_pose_profile(project: ProjectV40, profile_id: str) -> dict[str, Any]:
    profile = project.unload_pose_profiles.get(profile_id)
    if not isinstance(profile, dict):
        raise CompileError(f"missing unload pose profile: {profile_id}")
    if "yaw_ddeg" not in profile:
        raise CompileError(f"unload pose profile missing yaw_ddeg: {profile_id}")
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
    task_config: CompetitionTaskConfigV40,
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
        automatic = automatic_candidate_subset(candidates, task_config)
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
                "unload_pose_profile_id": step.unload_pose_profile_id,
                "pose": _drop_pose(project, step.anchor_site, step.unload_pose_profile_id, yaw_ddeg),
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

    target_by_site = {target.physical_site.value: target for target in drop_targets}
    step_by_station = {step.anchor_site: step for step in selected.unload_sequence}
    for station in ("P_DROP_1", "P_DROP_2", "P_DROP_3"):
        step = step_by_station.get(station)
        if step is None:
            pose = _project_site_pose(project, station)
            metadata: dict[str, Any] = {"active": False}
        else:
            pose = _drop_pose(project, station, step.unload_pose_profile_id, step.yaw_ddeg)
            metadata = {
                "active": True,
                "unload_mask": step.unload_mask.value,
                "unload_pose_profile_id": step.unload_pose_profile_id,
                "physical_drop_sites": list(step.physical_sites),
                "target_ranks": list(step.target_ranks),
                "bean_types": list(step.bean_types),
                "targets": [
                    target_by_site[site].to_dict()
                    for site in step.physical_sites
                    if site in target_by_site
                ],
            }
        points.append(
            {
                "point_id": station,
                "type": "TASK_ANCHOR",
                "pose": pose,
                **metadata,
            }
        )
    return tuple(points)


def _drop_pose(
    project: ProjectV40,
    anchor_site: str,
    unload_pose_profile_id: str,
    yaw_ddeg: int,
) -> dict[str, Any]:
    site = _project_site_pose(project, anchor_site)
    profile = _unload_pose_profile(project, unload_pose_profile_id)
    return {
        "x_mm": int(site["x_mm"]) + int(profile.get("dx_mm", 0)),
        "y_mm": int(site["y_mm"]) + int(profile.get("dy_mm", 0)),
        "yaw_ddeg": int(yaw_ddeg),
        "anchor_site": anchor_site,
        "unload_pose_profile_id": unload_pose_profile_id,
    }


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


def _selected_plan_dict(
    candidate_set: TaskCandidateSet,
    selected: CandidatePlan,
    task_config: CompetitionTaskConfigV40,
) -> dict[str, Any]:
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
        "preferred_route_family": preferred_route_family_for_candidate(selected, task_config).name,
        "route_selection_reason": route_selection_reason(selected, task_config),
        "locked_by_user": selected.locked_by_user,
        "selection_state": "LOCKED" if selected.locked_by_user else "DETERMINISTIC_ROUTE_RULE",
        "candidates": [
            candidate_review_dict(candidate, task_config)
            for candidate in candidate_set.candidates
        ],
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
            "unload_pose_profiles": project.unload_pose_profiles,
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
        "unload_pose_profiles": project.unload_pose_profiles,
        "topology_profiles": project.topology_profiles,
    }
    return {key: canonical_json_crc32_hex(value) for key, value in dependencies.items()}
