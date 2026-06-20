"""Candidate listing and user plan locking services."""

from __future__ import annotations

from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_project, load_route_case_table
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.task_plan import CandidatePlan
from hjmb_pathgen.py_services.case_draft_service import CaseDraftResult, generate_case_draft
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.task_compiler import compile_task_candidates
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table


def list_candidates(layout: ProjectLayout, traj_id: int) -> tuple[CandidatePlan, ...]:
    project = load_project(layout.project_json)
    table = _load_or_build_table(layout)
    row = _row_by_traj_id(table, traj_id)
    return compile_task_candidates(row, project).candidates


def select_candidate(layout: ProjectLayout, traj_id: int, candidate_id: str, *, lock: bool = False) -> CaseDraftResult:
    candidates = list_candidates(layout, traj_id)
    if candidate_id not in {candidate.candidate_id for candidate in candidates}:
        raise CompileError(f"candidate_id is not valid for P{traj_id:04d}: {candidate_id}")
    return generate_case_draft(layout, traj_id, preferred_candidate_id=candidate_id, lock_selected=lock)


def lock_candidate(layout: ProjectLayout, traj_id: int, candidate_id: str) -> CaseDraftResult:
    return select_candidate(layout, traj_id, candidate_id, lock=True)


def unlock_candidate(layout: ProjectLayout, traj_id: int) -> CaseDraftResult:
    current_id = None
    case_path = layout.case_json_path_for_mode(traj_id, GenerationMode.FULL_AUTO)
    if case_path.exists():
        current_id = str(load_case(case_path).selected_plan.get("candidate_id", "") or "")
    if current_id:
        return select_candidate(layout, traj_id, current_id, lock=False)
    return generate_case_draft(layout, traj_id, lock_selected=False)


def _load_or_build_table(layout: ProjectLayout):
    if not layout.route_case_table_json.exists():
        write_route_case_table(layout)
    return load_route_case_table(layout.route_case_table_json)


def _row_by_traj_id(table, traj_id: int):
    for row in table.cases:
        if row.traj_id == traj_id:
            return row
    raise CompileError(f"traj_id not found in route_case_table.json: {traj_id}")
