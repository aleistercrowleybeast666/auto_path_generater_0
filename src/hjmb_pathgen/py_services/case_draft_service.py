"""Phase 3 Case draft generation and deterministic reports."""

from __future__ import annotations

import csv
import io
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_project, load_route_case_table, save_case
from hjmb_pathgen.py_domain.errors import CompileError, WriteBackValidationError
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, RouteCaseRowV40, RouteCaseTableV40
from hjmb_pathgen.py_domain.task_plan import TransitionRequirement
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.task_compiler import CaseDraftBuildResult, build_case_draft
from hjmb_pathgen.py_services.competition_task_config_service import ensure_competition_task_config
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table

TASK_COMPILE_SUMMARY_CSV = "task_compile_summary.csv"
TASK_COMPILE_REPORT_JSON = "task_compile_report.json"
UNIQUE_TRANSITION_REQUIREMENTS_JSON = "unique_transition_requirements.json"


@dataclass(frozen=True)
class CaseDraftFailure:
    traj_id: int
    error: str


@dataclass(frozen=True)
class CaseDraftResult:
    traj_id: int
    case: CaseManifestV40
    selected_candidate_id: str
    candidate_count: int
    dual_candidate_count: int
    transition_requirement_count: int
    case_path: Path
    case_hash: str


@dataclass(frozen=True)
class CaseDraftBatchResult:
    results: tuple[CaseDraftResult, ...]
    failures: tuple[CaseDraftFailure, ...]
    summary_csv_path: Path
    report_json_path: Path
    unique_transition_requirements_path: Path
    unique_transition_requirement_count: int


def generate_case_draft(
    layout: ProjectLayout,
    traj_id: int,
    *,
    preferred_candidate_id: str | None = None,
    lock_selected: bool | None = None,
) -> CaseDraftResult:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    table = _ensure_route_case_table(layout)
    task_config = ensure_competition_task_config(layout.competition_task_config_json)
    row = _row_by_traj_id(table, traj_id)
    case_path = layout.case_json_path_for_mode(traj_id, GenerationMode.FULL_AUTO)
    existing_case = _load_existing_case(case_path)
    built = build_case_draft(
        row,
        project,
        existing_case=existing_case,
        preferred_candidate_id=preferred_candidate_id,
        lock_selected=lock_selected,
        task_config=task_config,
    )
    save_case(case_path, built.case)
    return _case_result(layout, built)


def generate_all_case_drafts(layout: ProjectLayout) -> CaseDraftBatchResult:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    table = _ensure_route_case_table(layout)
    task_config = ensure_competition_task_config(layout.competition_task_config_json)
    results: list[CaseDraftResult] = []
    failures: list[CaseDraftFailure] = []
    all_requirements: dict[str, TransitionRequirement] = {}

    for row in sorted(table.cases, key=lambda item: item.traj_id):
        try:
            case_path = layout.case_json_path_for_mode(row.traj_id, GenerationMode.FULL_AUTO)
            existing_case = _load_existing_case(case_path)
            built = build_case_draft(row, project, existing_case=existing_case, task_config=task_config)
            save_case(case_path, built.case)
            result = _case_result(layout, built)
            results.append(result)
            for requirement in built.transition_requirements:
                all_requirements.setdefault(requirement.requirement_id, requirement)
        except Exception as exc:  # noqa: BLE001 - per-case report should keep going.
            failures.append(CaseDraftFailure(traj_id=row.traj_id, error=str(exc)))

    summary_path = layout.reports_dir / TASK_COMPILE_SUMMARY_CSV
    report_path = layout.reports_dir / TASK_COMPILE_REPORT_JSON
    requirements_path = layout.reports_dir / UNIQUE_TRANSITION_REQUIREMENTS_JSON
    _write_summary_csv(summary_path, results, failures)
    _write_report_json(report_path, results, failures, table)
    _write_unique_requirements(requirements_path, all_requirements.values())
    return CaseDraftBatchResult(
        results=tuple(results),
        failures=tuple(failures),
        summary_csv_path=summary_path,
        report_json_path=report_path,
        unique_transition_requirements_path=requirements_path,
        unique_transition_requirement_count=len(all_requirements),
    )


def _ensure_route_case_table(layout: ProjectLayout) -> RouteCaseTableV40:
    if not layout.route_case_table_json.exists():
        write_route_case_table(layout)
    return load_route_case_table(layout.route_case_table_json)


def _row_by_traj_id(table: RouteCaseTableV40, traj_id: int) -> RouteCaseRowV40:
    for row in table.cases:
        if row.traj_id == traj_id:
            return row
    raise CompileError(f"traj_id not found in route_case_table.json: {traj_id}")


def _load_existing_case(path: Path) -> CaseManifestV40 | None:
    if not path.exists():
        return None
    return load_case(path)


def _case_result(layout: ProjectLayout, built: CaseDraftBuildResult) -> CaseDraftResult:
    case = built.case
    selected = built.selected_candidate
    dual_count = sum(1 for candidate in built.candidate_set.candidates if any(len(step.vehicle_bins) == 2 for step in candidate.unload_sequence))
    return CaseDraftResult(
        traj_id=case.traj_id,
        case=case,
        selected_candidate_id=selected.candidate_id,
        candidate_count=len(built.candidate_set.candidates),
        dual_candidate_count=dual_count,
        transition_requirement_count=len(built.transition_requirements),
        case_path=layout.case_json_path_for_mode(case.traj_id, GenerationMode.FULL_AUTO),
        case_hash=canonical_json_crc32_hex(case.to_dict()),
    )


def _write_summary_csv(path: Path, results: list[CaseDraftResult], failures: list[CaseDraftFailure]) -> None:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(
        [
            "traj_id",
            "status",
            "bean_code",
            "drop_code",
            "selected_candidate_id",
            "route_family",
            "unload_masks",
            "stop_count",
            "candidate_count",
            "dual_candidate_count",
            "transition_requirement_count",
            "case_hash",
            "error",
        ]
    )
    for result in sorted(results, key=lambda item: item.traj_id):
        selected_plan = result.case.selected_plan
        writer.writerow(
            [
                result.traj_id,
                "OK",
                result.case.bean_code,
                result.case.drop_code,
                result.selected_candidate_id,
                selected_plan.get("route_family", ""),
                " ".join(step.get("unload_mask", "") for step in selected_plan.get("unload_sequence", [])),
                len(selected_plan.get("unload_sequence", [])),
                result.candidate_count,
                result.dual_candidate_count,
                result.transition_requirement_count,
                result.case_hash,
                "",
            ]
        )
    for failure in sorted(failures, key=lambda item: item.traj_id):
        writer.writerow([failure.traj_id, "FAILED", "", "", "", "", "", "", "", "", "", "", failure.error])
    data = buffer.getvalue().encode("utf-8")

    def validator(temp_path: Path) -> None:
        if temp_path.read_bytes() != data:
            raise WriteBackValidationError(f"summary CSV write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)


def _write_report_json(path: Path, results: list[CaseDraftResult], failures: list[CaseDraftFailure], table: RouteCaseTableV40) -> None:
    candidate_counts = [result.candidate_count for result in results]
    report = {
        "format": "HJMB_PHASE3_TASK_COMPILE_REPORT",
        "development_phase": 3,
        "source_csv": table.source_csv,
        "source_csv_sha256": table.source_csv_sha256,
        "route_case_count": len(table.cases),
        "case_draft_count": len(results),
        "failure_count": len(failures),
        "generated_bin": False,
        "candidate_total": sum(candidate_counts),
        "candidate_count_min": min(candidate_counts) if candidate_counts else 0,
        "candidate_count_max": max(candidate_counts) if candidate_counts else 0,
        "dual_candidate_total": sum(result.dual_candidate_count for result in results),
        "results": [_result_report_dict(result) for result in sorted(results, key=lambda item: item.traj_id)],
        "failures": [{"traj_id": failure.traj_id, "error": failure.error} for failure in sorted(failures, key=lambda item: item.traj_id)],
    }
    _write_json(path, report)


def _write_unique_requirements(path: Path, requirements: Any) -> None:
    requirement_dicts = sorted((requirement.to_dict() for requirement in requirements), key=lambda item: item["requirement_id"])
    report = {
        "format": "HJMB_PHASE3_UNIQUE_TRANSITION_REQUIREMENTS",
        "development_phase": 3,
        "requirement_count": len(requirement_dicts),
        "requirements": requirement_dicts,
    }
    _write_json(path, report)


def _write_json(path: Path, value: dict[str, Any]) -> None:
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != value:
            raise WriteBackValidationError(f"JSON report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)


def _result_report_dict(result: CaseDraftResult) -> dict[str, Any]:
    selected_plan = result.case.selected_plan
    return {
        "traj_id": result.traj_id,
        "bean_code": result.case.bean_code,
        "drop_code": result.case.drop_code,
        "pick_assignment": result.case.source_mapping.get("pick_assignment", {}),
        "label_positions": result.case.source_mapping.get("label_positions", {}),
        "drop_targets": selected_plan.get("drop_targets", []),
        "candidate_count": result.candidate_count,
        "candidate_ids": [candidate.get("candidate_id", "") for candidate in selected_plan.get("candidates", [])],
        "selected_candidate_id": result.selected_candidate_id,
        "locked_by_user": bool(selected_plan.get("locked_by_user", False)),
        "route_family": selected_plan.get("route_family", ""),
        "unload_masks": [step.get("unload_mask", "") for step in selected_plan.get("unload_sequence", [])],
        "stop_count": len(selected_plan.get("unload_sequence", [])),
        "yaw_direction": selected_plan.get("yaw_direction", ""),
        "yaw_sequence_ddeg": selected_plan.get("yaw_sequence_ddeg", []),
        "estimated_mechanism_time_ms": selected_plan.get("estimated_mechanism_time_ms", 0),
        "transition_requirement_count": result.transition_requirement_count,
        "case_hash": result.case_hash,
    }
