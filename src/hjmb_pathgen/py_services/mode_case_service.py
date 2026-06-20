"""Generation-mode conversion and explicit per-mode planning workflows."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_case,
    load_leg_library,
    load_project,
    save_case,
)

from .case_compiler import CaseCompileRequest, compile_case_to_trajectory
from .leg_optimization_service import optimize_current_case_leg
from .output_service import CaseOutputOptions, CaseOutputResult, write_case_outputs
from .path_validation_service import case_with_collision_result, validate_time_parameterized_trajectory
from .phase7_generation_service import _compiled_actions, _leg_refs_and_arrival_s
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from .task_compiler import transition_requirements_for_case


def convert_full_auto_to_semi_auto(
    layout: ProjectLayout,
    traj_id: int,
    *,
    overwrite: bool = False,
) -> CaseManifestV40:
    """Create an editable SEMI_AUTO copy without mutating the FULL_AUTO source."""

    source_path = layout.case_json_path_for_mode(traj_id, GenerationMode.FULL_AUTO)
    target_path = layout.case_json_path_for_mode(traj_id, GenerationMode.SEMI_AUTO)
    if not source_path.exists():
        raise CompileError(f"FULL_AUTO case does not exist: {source_path}")
    if target_path.exists() and not overwrite:
        raise CompileError(f"SEMI_AUTO case already exists: {target_path}")
    source = load_case(source_path)
    if source.generation_mode != GenerationMode.FULL_AUTO:
        raise CompileError(f"source case is not FULL_AUTO: {source.generation_mode.value}")
    source_hash = canonical_json_crc32_hex(source.to_dict())
    review = {
        **source.review,
        "state": "STALE",
        "approved": False,
        "detached_from_library": False,
        "manual_override": True,
        "override_reason": "converted from FULL_AUTO for assisted editing",
        "stale_reason": "SEMI_AUTO copy requires explicit regeneration",
    }
    selected_plan = {
        **source.selected_plan,
        "locked_by_user": True,
        "selection_state": "DERIVED_SEMI_AUTO",
        "initial_curve_leg_refs": [dict(item) for item in source.leg_refs],
    }
    converted = replace(
        source,
        generation_mode=GenerationMode.SEMI_AUTO,
        selected_plan=selected_plan,
        derived_from={
            "generation_mode": GenerationMode.FULL_AUTO.value,
            "traj_id": traj_id,
            "case_hash": source_hash,
        },
        review=review,
    )
    layout.ensure_directories()
    save_case(target_path, converted)
    return converted


def generate_semi_auto(
    layout: ProjectLayout,
    traj_id: int,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
    cancel_check: Callable[[], bool] | None = None,
) -> CaseOutputResult:
    """Optimize only between the eight fixed SEMI_AUTO anchors and assemble V4 output."""

    case_path = layout.case_json_path_for_mode(traj_id, GenerationMode.SEMI_AUTO)
    if not case_path.exists():
        raise CompileError(f"SEMI_AUTO case does not exist: {case_path}")
    case = load_case(case_path)
    if case.generation_mode != GenerationMode.SEMI_AUTO:
        raise CompileError(f"case is not SEMI_AUTO: {case.generation_mode.value}")
    project = load_project(layout.project_json)
    synced_case = replace(case, arrival_states=_semi_arrival_states(case))
    save_case(case_path, synced_case)
    requirements = transition_requirements_for_case(synced_case, project)
    for index, requirement in enumerate(requirements):
        if cancel_check is not None and cancel_check():
            raise RuntimeError("CANCELLED")
        result = optimize_current_case_leg(
            layout,
            case_path,
            requirement.requirement_id,
            profile_name=profile_name,
            seed=seed + index,
            replace_existing=True,
        )
        if not result.success or result.leg is None:
            raise CompileError(
                f"SEMI_AUTO transition {requirement.requirement_id} failed: {result.diagnostics}"
            )
        if progress_callback is not None:
            completed = index + 1
            progress_callback(
                {
                    "current_item": requirement.requirement_id,
                    "completed_count": completed,
                    "total_count": len(requirements),
                    "percent": round(90 * completed / max(len(requirements), 1)),
                    "optimized_count": completed,
                    "failed_count": 0,
                }
            )
    library = load_leg_library(layout.leg_library_json)
    leg_refs, arrival_s = _leg_refs_and_arrival_s(
        requirements,
        project,
        synced_case,
        library,
    )
    actions = {
        "source": list(synced_case.actions.get("source", [])),
        "compiled": _compiled_actions(synced_case, project, arrival_s),
    }
    ready_case = replace(
        synced_case,
        leg_refs=tuple(leg_refs),
        actions=actions,
        review={
            **synced_case.review,
            "state": "VALID",
            "approved": False,
            "stale_reason": "",
        },
    )
    trajectory = compile_case_to_trajectory(
        CaseCompileRequest(case=ready_case, leg_library=library, project=project)
    )
    collision = validate_time_parameterized_trajectory(tuple(trajectory.nodes), project, strict=True)
    if not collision.passed:
        raise CompileError(
            f"SEMI_AUTO collision validation failed: {collision.status.value}"
        )
    ready_case = case_with_collision_result(ready_case, collision)
    save_case(case_path, ready_case)
    return write_case_outputs(
        layout,
        CaseCompileRequest(case=ready_case, leg_library=library, project=project),
        CaseOutputOptions(
            generation_mode=GenerationMode.SEMI_AUTO,
            require_approval=False,
        ),
    )


def _semi_arrival_states(case: CaseManifestV40) -> tuple[dict, ...]:
    pose_by_id = {str(item["point_id"]): dict(item["pose"]) for item in case.logical_points}
    states: list[dict] = []
    for state_id in case.selected_plan.get("pickup_arrival_state_order", ()): 
        states.append(
            {
                "state_id": str(state_id),
                "type": "PICK",
                "site_key": str(state_id),
                "pose": pose_by_id[str(state_id)],
            }
        )
    for step in case.selected_plan.get("unload_sequence", ()):
        step_index = int(step["step_index"])
        target_ranks = tuple(int(value) for value in step.get("target_ranks", ()))
        if not target_ranks:
            raise CompileError(f"SEMI_AUTO unload step {step_index} has no target rank")
        point_id = f"P_DROP_{target_ranks[0]}"
        states.append(
            {
                "state_id": f"DROP_STEP_{step_index}",
                "type": "DROP",
                "physical_drop_sites": list(step.get("physical_sites", ())),
                "target_ranks": list(target_ranks),
                "vehicle_bins": list(step.get("vehicle_bins", ())),
                "unload_mask": str(step.get("unload_mask", "")),
                "pose": pose_by_id[point_id],
            }
        )
    if not states:
        raise CompileError("SEMI_AUTO case has no executable arrival sequence")
    return tuple(states)
