"""Generation-mode conversion and explicit per-mode planning workflows."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.semi_path import ROUTE_A_SITE_SEQUENCE, ROUTE_B_SITE_SEQUENCE
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_case
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


def _semi_path_from_full_case(source: CaseManifestV40) -> dict[str, Any]:
    route_family = str(source.selected_plan.get("route_family", ""))
    if route_family == "PICK_1_TO_3":
        sequence = ROUTE_A_SITE_SEQUENCE
    elif route_family == "PICK_3_TO_1":
        sequence = ROUTE_B_SITE_SEQUENCE
    else:
        raise CompileError(f"FULL_AUTO case has unsupported route family: {route_family}")

    drop_state_by_rank: dict[int, str] = {}
    for step in source.selected_plan.get("unload_sequence", ()):  # one stop or a legal dual stop
        state_id = f"DROP_STEP_{int(step.get('step_index', len(drop_state_by_rank) + 1))}"
        for rank in step.get("target_ranks", ()):
            drop_state_by_rank[int(rank)] = state_id

    points: list[dict[str, Any]] = []
    for index, site_key in enumerate(sequence):
        state_id = site_key
        if site_key.startswith("P_DROP_"):
            state_id = drop_state_by_rank.get(int(site_key.rsplit("_", 1)[1]), site_key)
        points.append(
            {
                "type": "START" if index == 0 else "ARRIVAL",
                "site_key": site_key,
                "state_id": state_id,
            }
        )
    return {
        "points": points,
        "notes": "derived from FULL_AUTO; add free WAYPOINT rows manually where required",
    }


def convert_full_auto_to_semi_auto(
    layout: ProjectLayout,
    traj_id: int,
    *,
    overwrite: bool = False,
) -> CaseManifestV40:
    """Create an editable ordered SEMI_AUTO copy without mutating FULL_AUTO."""

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
        "detached_from_library": True,
        "manual_override": True,
        "override_reason": "converted from FULL_AUTO for ordered semi-auto editing",
        "stale_reason": "SEMI_AUTO copy requires explicit regeneration",
    }
    selected_plan = {
        **source.selected_plan,
        "yaw_direction": "SHORTEST",
        "locked_by_user": True,
        "selection_state": "DERIVED_SEMI_AUTO",
    }
    converted = replace(
        source,
        generation_mode=GenerationMode.SEMI_AUTO,
        selected_plan=selected_plan,
        manual_path=None,
        semi_path=_semi_path_from_full_case(source),
        logical_points=(),
        auxiliary_points=(),
        leg_refs=(),
        derived_from={
            "generation_mode": GenerationMode.FULL_AUTO.value,
            "traj_id": traj_id,
            "case_hash": source_hash,
        },
        review=review,
    )
    # Re-parse to apply all SEMI_AUTO invariants before writing.
    converted = CaseManifestV40.from_dict(converted.to_dict())
    layout.ensure_directories()
    save_case(target_path, converted)
    return converted


def generate_semi_auto(
    layout: ProjectLayout,
    traj_id: int,
    *,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
    cancel_check: Callable[[], bool] | None = None,
):
    """Plan the user-drawn ordered SEMI_AUTO path without replacing its geometry."""

    if cancel_check is not None and cancel_check():
        raise RuntimeError("CANCELLED")
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_RESOLVE",
                "message": "解析固定点与人工途径点",
                "completed_count": 0,
                "total_count": 1,
                "percent": 10,
            }
        )
    case_path = layout.case_json_path_for_mode(traj_id, GenerationMode.SEMI_AUTO)
    if not case_path.exists():
        raise CompileError(f"SEMI_AUTO case does not exist: {case_path}")
    case = load_case(case_path)
    if case.generation_mode != GenerationMode.SEMI_AUTO:
        raise CompileError(f"case is not SEMI_AUTO: {case.generation_mode.value}")

    from .mode_output_service import write_semi_auto_outputs

    result = write_semi_auto_outputs(
        layout,
        case,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_DONE",
                "message": "半自动路径已按人工顺序生成",
                "completed_count": 1,
                "total_count": 1,
                "percent": 100,
            }
        )
    return result
