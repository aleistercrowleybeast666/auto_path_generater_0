"""Phase 7 unique-leg collection and final case generation services."""

from __future__ import annotations

import csv
import io
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Iterable

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.bin_codec import decode_trajectory, encode_trajectory
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project, load_route_case_table
from hjmb_pathgen.py_domain.compiled import CompiledTrajectoryV40
from hjmb_pathgen.py_domain.enums import ActionMode, LegState, NodeFlag, GenerationMode, YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError, WriteBackValidationError
from hjmb_pathgen.py_domain.leg import LegLibraryV40, LegV40
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, RouteCaseRowV40, RouteCaseTableV40
from hjmb_pathgen.py_domain.task_plan import TransitionRequirement
from hjmb_pathgen.py_planning.optimization.leg_optimizer import leg_id_from_key, leg_key_from_request
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from hjmb_pathgen.py_services.case_compiler import CaseCompileRequest, compile_case_to_trajectory
from hjmb_pathgen.py_services.leg_library_service import load_or_create_leg_library, save_leg_library_checked, upsert_leg
from hjmb_pathgen.py_services.export_guard_service import check_formal_export_guard
from hjmb_pathgen.py_services.leg_optimization_service import leg_request_from_transition, optimize_transition_leg, validate_leg
from hjmb_pathgen.py_services.leg_stale_service import leg_stale_reasons
from hjmb_pathgen.py_services.output_service import CaseOutputOptions, CaseOutputResult, write_case_outputs
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.task_compiler import (
    automatic_candidate_subset,
    build_case_draft,
    compile_task_candidates,
    preferred_route_family_for_candidate,
)
from hjmb_pathgen.py_services.execution_time_estimator import estimate_fifo_execution
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table
from hjmb_pathgen.py_services.competition_task_config_service import ensure_competition_task_config

UNIQUE_LEG_REPORT_JSON = "phase7_unique_leg_report.json"
BATCH_REPORT_JSON = "phase7_batch_report.json"
BATCH_SUMMARY_CSV = "phase7_batch_summary.csv"

REUSABLE_LEG_STATES = {LegState.VALID, LegState.APPROVED, LegState.LOCKED}


@dataclass(frozen=True)
class UniqueLegUsage:
    traj_id: int
    candidate_id: str
    transition_index: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.traj_id,
            "candidate_id": self.candidate_id,
            "transition_index": self.transition_index,
        }


@dataclass(frozen=True)
class UniqueLegRequirement:
    requirement_id: str
    semantic_hash: str
    leg_id: str
    status: str
    reusable: bool
    transition: TransitionRequirement
    usage: tuple[UniqueLegUsage, ...]
    leg_state: str | None = None
    review: dict[str, Any] | None = None
    planned_time_ms: int | None = None

    def to_dict(self) -> dict[str, Any]:
        data = {
            "requirement_id": self.requirement_id,
            "semantic_hash": self.semantic_hash,
            "leg_id": self.leg_id,
            "status": self.status,
            "reusable": self.reusable,
            "transition": self.transition.to_dict(),
            "usage": [item.to_dict() for item in self.usage],
        }
        if self.leg_state is not None:
            data["leg_state"] = self.leg_state
        if self.review is not None:
            data["review"] = self.review
        if self.planned_time_ms is not None:
            data["planned_time_ms"] = self.planned_time_ms
        return data


@dataclass(frozen=True)
class UniqueLegCollectionResult:
    requirements: tuple[UniqueLegRequirement, ...]
    report_path: Path

    @property
    def counts_by_status(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for requirement in self.requirements:
            counts[requirement.status] = counts.get(requirement.status, 0) + 1
        return counts

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": "HJMB_PHASE7_UNIQUE_LEG_REPORT",
            "requirement_count": len(self.requirements),
            "counts_by_status": self.counts_by_status,
            "requirements": [item.to_dict() for item in self.requirements],
        }


@dataclass(frozen=True)
class CandidateTiming:
    candidate_id: str
    semantic_hash: str
    route_family: str
    complete: bool
    motion_time_ms: int
    mechanism_time_ms: int
    mechanism_busy_time_ms: int
    total_time_ms: int
    leg_ids: tuple[str, ...]
    missing_leg_ids: tuple[str, ...]
    failure_reason: str = ""
    route_rule_match: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "semantic_hash": self.semantic_hash,
            "route_family": self.route_family,
            "complete": self.complete,
            "motion_time_ms": self.motion_time_ms,
            "mechanism_time_ms": self.mechanism_time_ms,
            "mechanism_busy_time_ms": self.mechanism_busy_time_ms,
            "total_time_ms": self.total_time_ms,
            "leg_ids": list(self.leg_ids),
            "missing_leg_ids": list(self.missing_leg_ids),
            "failure_reason": self.failure_reason,
            "route_rule_match": self.route_rule_match,
        }


@dataclass(frozen=True)
class CaseCandidateEvaluationResult:
    traj_id: int
    timings: tuple[CandidateTiming, ...]
    selected_candidate_id: str | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.traj_id,
            "candidate_count": len(self.timings),
            "selected_candidate_id": self.selected_candidate_id,
            "timings": [item.to_dict() for item in self.timings],
        }


@dataclass(frozen=True)
class GeneratedCaseResult:
    traj_id: int
    selected_candidate_id: str
    case: CaseManifestV40
    timing: CandidateTiming
    output: CaseOutputResult

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.traj_id,
            "selected_candidate_id": self.selected_candidate_id,
            "timing": self.timing.to_dict(),
            "case_path": str(self.output.case_path) if self.output.case_path else None,
            "bin_path": str(self.output.bin_path) if self.output.bin_path else None,
            "portable_path": str(self.output.portable_path) if self.output.portable_path else None,
            "byte_size": self.output.byte_size,
            "hashes": self.output.hashes,
            "warnings": list(self.output.warnings),
        }


@dataclass(frozen=True)
class BatchGenerationFailure:
    traj_id: int
    error: str

    def to_dict(self) -> dict[str, Any]:
        return {"traj_id": self.traj_id, "error": self.error}


@dataclass(frozen=True)
class BatchGenerationResult:
    results: tuple[GeneratedCaseResult, ...]
    failures: tuple[BatchGenerationFailure, ...]
    report_path: Path
    summary_path: Path

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": "HJMB_PHASE7_BATCH_REPORT",
            "case_count": len(self.results),
            "failure_count": len(self.failures),
            "report_path": str(self.report_path),
            "summary_path": str(self.summary_path),
            "results": [item.to_dict() for item in self.results],
            "failures": [item.to_dict() for item in self.failures],
        }


@dataclass(frozen=True)
class OptimizeMissingLegsResult:
    attempted_count: int
    optimized_count: int
    failure_count: int
    skipped_count: int
    failures: tuple[dict[str, Any], ...]
    library_path: Path

    def to_dict(self) -> dict[str, Any]:
        return {
            "attempted_count": self.attempted_count,
            "optimized_count": self.optimized_count,
            "failure_count": self.failure_count,
            "skipped_count": self.skipped_count,
            "failures": list(self.failures),
            "library_path": str(self.library_path),
        }


@dataclass(frozen=True)
class _ReusableLegAudit:
    reusable: bool
    reason: str
    validation: dict[str, Any] | None = None


def audit_phase6(layout: ProjectLayout) -> dict[str, Any]:
    collection = collect_unique_legs(layout, write_report=False)
    return {
        "format": "HJMB_PHASE7_PHASE6_AUDIT",
        "unique_leg_requirement_count": len(collection.requirements),
        "counts_by_status": collection.counts_by_status,
        "final_drop_semantics": "AT_FINAL_DROP_NO_SAFE_END_NO_FINISH_CLEAR",
        "notes": [
            "approved/locked are review flags; stale marking may still set leg.state=STALE",
            "generation commands do not run optimization implicitly",
        ],
    }


def collect_unique_legs(layout: ProjectLayout, *, write_report: bool = True) -> UniqueLegCollectionResult:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    table = _ensure_route_case_table(layout)
    task_config = ensure_competition_task_config(layout.competition_task_config_json)
    library = load_or_create_leg_library(layout.leg_library_json, project)
    requirements: dict[str, tuple[str, TransitionRequirement, list[UniqueLegUsage]]] = {}

    for row in sorted(table.cases, key=lambda item: item.traj_id):
        candidate_set = compile_task_candidates(row, project, task_config)
        for candidate in automatic_candidate_subset(candidate_set.candidates, task_config):
            built = build_case_draft(row, project, preferred_candidate_id=candidate.candidate_id, task_config=task_config)
            for index, requirement in enumerate(built.transition_requirements):
                key = _leg_id_for_transition(requirement, project, built.case)
                entry = requirements.setdefault(key, (key, requirement, []))
                entry[2].append(UniqueLegUsage(row.traj_id, candidate.candidate_id, index))

    result = UniqueLegCollectionResult(
        requirements=tuple(
            sorted(
                (
                    _unique_requirement_from_library(leg_id, transition, usage, library, project)
                    for leg_id, transition, usage in requirements.values()
                ),
                key=lambda item: item.leg_id,
            )
        ),
        report_path=layout.reports_dir / UNIQUE_LEG_REPORT_JSON,
    )
    if write_report:
        _write_json(result.report_path, result.to_dict())
    return result


def show_leg_status(layout: ProjectLayout) -> dict[str, Any]:
    result = collect_unique_legs(layout)
    return {
        "requirement_count": len(result.requirements),
        "counts_by_status": result.counts_by_status,
        "report_path": str(result.report_path),
    }


def optimize_missing_legs(
    layout: ProjectLayout,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    include_stale: bool = True,
    max_count: int | None = None,
    force: bool = False,
    traj_id: int | None = None,
    candidate_id: str | None = None,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
    cancel_check: Callable[[], bool] | None = None,
) -> OptimizeMissingLegsResult:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    library = load_or_create_leg_library(layout.leg_library_json, project)
    if traj_id is None:
        collection = collect_unique_legs(layout)
        relevant_requirements = list(collection.requirements)
    else:
        # A single-ID GUI generation must not compile/audit all 360 cases.
        # Build only the deterministic route candidates for this row; this also
        # makes cancellation and progress feedback effectively immediate.
        table = _ensure_route_case_table(layout)
        task_config = ensure_competition_task_config(layout.competition_task_config_json)
        row = _row_by_traj_id(table, traj_id)
        candidate_set = compile_task_candidates(row, project, task_config)
        selected_candidates = automatic_candidate_subset(candidate_set.candidates, task_config)
        if candidate_id is not None:
            selected_candidates = tuple(
                item for item in selected_candidates if item.candidate_id == candidate_id
            )
        if not selected_candidates:
            raise CompileError(f"P{traj_id:04d} has no matching automatic candidate")
        by_leg: dict[str, tuple[TransitionRequirement, list[UniqueLegUsage]]] = {}
        for candidate in selected_candidates:
            built = build_case_draft(
                row, project, preferred_candidate_id=candidate.candidate_id,
                task_config=task_config,
            )
            for index, transition in enumerate(built.transition_requirements):
                leg_id = _leg_id_for_transition(transition, project, built.case)
                entry = by_leg.setdefault(leg_id, (transition, []))
                entry[1].append(UniqueLegUsage(traj_id, candidate.candidate_id, index))
        relevant_requirements = [
            _unique_requirement_from_library(leg_id, transition, usage, library, project)
            for leg_id, (transition, usage) in by_leg.items()
        ]
    targets = [
        item
        for item in relevant_requirements
        if item.status == "MISSING" or (include_stale and item.status == "STALE")
    ]
    if max_count is not None:
        targets = targets[: max(0, max_count)]

    optimized = 0
    attempted = 0
    failures: list[dict[str, Any]] = []
    for index, target in enumerate(targets):
        if cancel_check is not None and cancel_check():
            break
        attempted += 1
        warm_start = _leg_by_id(library, target.leg_id)
        try:
            result = optimize_transition_leg(
                target.transition,
                project,
                profile_name=profile_name,
                seed=seed + index,
                yaw_policy=_yaw_policy_from_usage(layout, target),
                warm_start_leg=warm_start,
                cancel_check=cancel_check,
                progress_callback=(
                    None
                    if progress_callback is None
                    else lambda diagnostic, leg_id=target.leg_id: progress_callback(
                        {
                            "current_item": leg_id,
                            "optimizer_stage": diagnostic.get("stage", ""),
                            "optimizer_message": diagnostic.get("message", ""),
                        }
                    )
                ),
            )
            if not result.success or result.leg is None:
                failures.append(
                    {
                        "leg_id": target.leg_id,
                        "reason": result.reason,
                        "state": result.state.value,
                        "from_state_id": target.transition.from_state_id,
                        "to_state_id": target.transition.to_state_id,
                        "evaluations": [item.to_dict() for item in result.evaluations],
                    }
                )
                continue
            library = upsert_leg(
                library,
                result.leg,
                replace_existing=target.status != "MISSING",
                force=force or target.status == "STALE",
            )
            optimized += 1
        except Exception as exc:  # noqa: BLE001 - per-leg report should keep going.
            failures.append({"leg_id": target.leg_id, "reason": str(exc), "state": target.status})
        if progress_callback is not None:
            completed = index + 1
            progress_callback(
                {
                    "current_item": target.leg_id,
                    "completed_count": completed,
                    "total_count": len(targets),
                    "percent": round(100 * completed / max(len(targets), 1)),
                    "optimized_count": optimized,
                    "failed_count": len(failures),
                }
            )

    if optimized:
        save_leg_library_checked(layout.leg_library_json, library)
    return OptimizeMissingLegsResult(
        attempted_count=attempted,
        optimized_count=optimized,
        failure_count=len(failures),
        skipped_count=len(relevant_requirements) - attempted,
        failures=tuple(failures),
        library_path=layout.leg_library_json,
    )


def optimize_leg_by_id(
    layout: ProjectLayout,
    leg_id: str,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    force: bool = False,
) -> dict[str, Any]:
    """Explicitly optimize one directed leg selected in the shared library view."""

    project = load_project(layout.project_json)
    collection = collect_unique_legs(layout)
    target = next((item for item in collection.requirements if item.leg_id == leg_id), None)
    if target is None:
        raise CompileError(f"leg is not referenced by the current route table: {leg_id}")
    library = load_or_create_leg_library(layout.leg_library_json, project)
    warm_start = _leg_by_id(library, leg_id)
    result = optimize_transition_leg(
        target.transition,
        project,
        profile_name=profile_name,
        seed=seed,
        yaw_policy=_yaw_policy_from_usage(layout, target),
        warm_start_leg=warm_start,
    )
    if not result.success or result.leg is None:
        return {
            "leg_id": leg_id,
            "success": False,
            "state": result.state.value,
            "reason": result.reason,
        }
    updated = upsert_leg(
        library,
        result.leg,
        replace_existing=warm_start is not None,
        force=force,
    )
    save_leg_library_checked(layout.leg_library_json, updated)
    return {
        "leg_id": leg_id,
        "success": True,
        "state": result.state.value,
        "planned_time_ms": result.leg.analysis.get("planned_time_ms", 0),
    }


def evaluate_case_candidates(layout: ProjectLayout, traj_id: int) -> CaseCandidateEvaluationResult:
    project = load_project(layout.project_json)
    table = _ensure_route_case_table(layout)
    library = load_or_create_leg_library(layout.leg_library_json, project)
    row = _row_by_traj_id(table, traj_id)
    timings = _candidate_timings(row, project, library)
    complete = [item for item in timings if item.complete]
    selected = min(complete, key=_candidate_timing_sort_key).candidate_id if complete else None
    return CaseCandidateEvaluationResult(traj_id=traj_id, timings=timings, selected_candidate_id=selected)


def generate_one(
    layout: ProjectLayout,
    traj_id: int,
    *,
    write_portable: bool = False,
    dry_run: bool = False,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
) -> GeneratedCaseResult:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    table = _ensure_route_case_table(layout)
    library = load_leg_library(layout.leg_library_json)
    row = _row_by_traj_id(table, traj_id)
    case, timing = _best_complete_case(layout, row, project, library, leg_audit_cache=leg_audit_cache)
    output = write_case_outputs(
        layout,
        CaseCompileRequest(case=case, leg_library=library, project=project),
        CaseOutputOptions(
            write_case_json=True,
            write_bin=True,
            write_portable=write_portable,
            write_report=True,
            dry_run=dry_run,
            formal_competition=True,
            require_approval=False,
            generation_mode=GenerationMode.FULL_AUTO,
        ),
    )
    return GeneratedCaseResult(traj_id=traj_id, selected_candidate_id=timing.candidate_id, case=case, timing=timing, output=output)


def generate_all(
    layout: ProjectLayout,
    *,
    write_portable: bool = False,
    dry_run: bool = False,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
    cancel_check: Callable[[], bool] | None = None,
) -> BatchGenerationResult:
    layout.ensure_directories()
    table = _ensure_route_case_table(layout)
    results: list[GeneratedCaseResult] = []
    failures: list[BatchGenerationFailure] = []
    leg_audit_cache: dict[str, _ReusableLegAudit] = {}
    rows = sorted(table.cases, key=lambda item: item.traj_id)
    for index, row in enumerate(rows):
        if cancel_check is not None and cancel_check():
            break
        try:
            results.append(
                generate_one(
                    layout,
                    row.traj_id,
                    write_portable=write_portable,
                    dry_run=dry_run,
                    leg_audit_cache=leg_audit_cache,
                )
            )
        except Exception as exc:  # noqa: BLE001 - batch should continue and report per case.
            failures.append(BatchGenerationFailure(row.traj_id, str(exc)))
        if progress_callback is not None:
            completed = index + 1
            progress_callback(
                {
                    "current_item": f"P{row.traj_id:04d}",
                    "completed_count": completed,
                    "total_count": len(rows),
                    "percent": round(100 * completed / max(len(rows), 1)),
                    "generated_count": len(results),
                    "failed_count": len(failures),
                }
            )
    report_path = layout.reports_dir / BATCH_REPORT_JSON
    summary_path = layout.reports_dir / BATCH_SUMMARY_CSV
    result = BatchGenerationResult(tuple(results), tuple(failures), report_path, summary_path)
    _write_json(report_path, result.to_dict())
    _write_batch_summary(summary_path, results, failures)
    return result


def validate_one(
    layout: ProjectLayout,
    traj_id: int,
    *,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
) -> dict[str, Any]:
    project = load_project(layout.project_json)
    library = load_leg_library(layout.leg_library_json)
    case_path = _existing_case_path(layout, traj_id, GenerationMode.FULL_AUTO)
    case = load_case(case_path)
    dependency_failures = _case_dependency_failures(case, project, library, leg_audit_cache=leg_audit_cache)
    trajectory = compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library, project=project))
    data = encode_trajectory(trajectory)
    decoded = decode_trajectory(data)
    roundtrip_ok = encode_trajectory(decoded) == data
    export_guard = check_formal_export_guard(case, require_collision_passed=True, require_approval=True)
    validation_errors: list[str] = []
    if dependency_failures:
        validation_errors.extend(dependency_failures)
    if not roundtrip_ok:
        validation_errors.append("BIN round-trip mismatch")
    if trajectory.nodes[-1].flags & int(NodeFlag.SAFE_END):
        validation_errors.append("SAFE_END is reserved and must be zero")
    if any(segment.flags & 0x02 for segment in trajectory.segments):
        validation_errors.append("FINISH_CLEAR is reserved and must be zero")
    return {
        "traj_id": traj_id,
        "valid": not validation_errors,
        "case_path": str(case_path),
        "node_count": len(trajectory.nodes),
        "segment_count": len(trajectory.segments),
        "action_count": len(trajectory.actions),
        "planned_motion_time_ms": trajectory.header.planned_motion_time_ms,
        "finish_mode": trajectory.header.finish_mode,
        "last_node_flags": trajectory.nodes[-1].flags,
        "bin_roundtrip": roundtrip_ok,
        "dependency_failure_count": len(dependency_failures),
        "dependency_failures": dependency_failures,
        "final_export_allowed": export_guard.allowed,
        "final_export_blockers": list(export_guard.reasons),
        "errors": validation_errors,
    }


def validate_all(layout: ProjectLayout) -> dict[str, Any]:
    table = _ensure_route_case_table(layout)
    results = []
    failures = []
    leg_audit_cache: dict[str, _ReusableLegAudit] = {}
    for row in sorted(table.cases, key=lambda item: item.traj_id):
        try:
            results.append(validate_one(layout, row.traj_id, leg_audit_cache=leg_audit_cache))
        except Exception as exc:  # noqa: BLE001
            failures.append({"traj_id": row.traj_id, "error": str(exc)})
    return {"case_count": len(results), "failure_count": len(failures), "results": results, "failures": failures}


def export_portable(layout: ProjectLayout, traj_id: int) -> GeneratedCaseResult:
    return generate_one(layout, traj_id, write_portable=True)


def show_batch_report(layout: ProjectLayout) -> dict[str, Any]:
    report_path = layout.reports_dir / BATCH_REPORT_JSON
    if not report_path.exists():
        raise CompileError(f"batch report not found: {report_path}")
    return json.loads(report_path.read_text(encoding="utf-8"))


def _ensure_route_case_table(layout: ProjectLayout) -> RouteCaseTableV40:
    if not layout.route_case_table_json.exists():
        write_route_case_table(layout)
    return load_route_case_table(layout.route_case_table_json)


def _row_by_traj_id(table: RouteCaseTableV40, traj_id: int) -> RouteCaseRowV40:
    for row in table.cases:
        if row.traj_id == traj_id:
            return row
    raise CompileError(f"traj_id not found in route_case_table.json: {traj_id}")


def _unique_requirement_from_library(
    leg_id: str,
    transition: TransitionRequirement,
    usage: list[UniqueLegUsage],
    library: LegLibraryV40,
    project: ProjectV40,
) -> UniqueLegRequirement:
    leg = _leg_by_id(library, leg_id)
    if leg is None:
        return UniqueLegRequirement(
            requirement_id=transition.requirement_id,
            semantic_hash=transition.semantic_hash,
            leg_id=leg_id,
            status="MISSING",
            reusable=False,
            transition=transition,
            usage=tuple(usage),
        )
    audit = _active_leg_audit(project, leg)
    reusable = audit.reusable
    status = "REUSABLE" if reusable else ("STALE" if leg.state in REUSABLE_LEG_STATES else leg.state.value)
    review = dict(leg.review)
    if not reusable:
        review["active_audit_reason"] = audit.reason
    return UniqueLegRequirement(
        requirement_id=transition.requirement_id,
        semantic_hash=transition.semantic_hash,
        leg_id=leg_id,
        status=status,
        reusable=reusable,
        transition=transition,
        usage=tuple(usage),
        leg_state=leg.state.value,
        review=review,
        planned_time_ms=int(leg.analysis.get("planned_time_ms", 0)),
    )


def _candidate_timings(
    row: RouteCaseRowV40,
    project: ProjectV40,
    library: LegLibraryV40,
    *,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
    task_config=None,
) -> tuple[CandidateTiming, ...]:
    candidate_set = compile_task_candidates(row, project, task_config)
    timings: list[CandidateTiming] = []
    audit_cache = leg_audit_cache if leg_audit_cache is not None else {}
    for candidate in automatic_candidate_subset(candidate_set.candidates, task_config):
        built = build_case_draft(row, project, preferred_candidate_id=candidate.candidate_id, task_config=task_config)
        leg_ids: list[str] = []
        missing: list[str] = []
        motion_time = 0
        arrival_release: dict[object, int] = {}
        for requirement in built.transition_requirements:
            leg_id = _leg_id_for_transition(requirement, project, built.case)
            leg_ids.append(leg_id)
            leg = _leg_by_id(library, leg_id)
            audit = _cached_leg_audit(project, leg, audit_cache)
            if leg is None or not audit.reusable:
                missing.append(leg_id)
            else:
                motion_time += int(leg.analysis.get("planned_time_ms", 0))
                arrival_release[requirement.to_state_id] = motion_time
        execution = estimate_fifo_execution(
            project,
            built.case.actions.get("source", []),
            motion_time_ms=motion_time,
            arrival_release_ms=arrival_release,
        )
        timings.append(
            CandidateTiming(
                candidate_id=candidate.candidate_id,
                semantic_hash=candidate.semantic_hash,
                route_family=candidate.route_family.name,
                complete=not missing,
                motion_time_ms=motion_time,
                mechanism_time_ms=execution.added_wait_time_ms,
                mechanism_busy_time_ms=execution.mechanism_busy_time_ms,
                total_time_ms=execution.total_time_ms,
                leg_ids=tuple(leg_ids),
                missing_leg_ids=tuple(missing),
                failure_reason="" if not missing else "missing, stale, failed, preview, or invalid legs",
                route_rule_match=(
                    candidate.route_family
                    == preferred_route_family_for_candidate(candidate, task_config)
                ),
            )
        )
    return tuple(sorted(timings, key=_candidate_timing_sort_key))


def _best_complete_case(
    layout: ProjectLayout,
    row: RouteCaseRowV40,
    project: ProjectV40,
    library: LegLibraryV40,
    *,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
) -> tuple[CaseManifestV40, CandidateTiming]:
    task_config = ensure_competition_task_config(layout.competition_task_config_json)
    timings = _candidate_timings(
        row, project, library, leg_audit_cache=leg_audit_cache, task_config=task_config
    )
    complete = [item for item in timings if item.complete]
    if not complete:
        raise CompileError(f"P{row.traj_id:04d} has no complete candidate; optimize missing legs first")
    existing_case = _load_existing_task_case(layout, row.traj_id)
    selected, selection_state, locked_by_user = _select_phase8_candidate(row, existing_case, timings, complete)
    built = build_case_draft(
        row, project, preferred_candidate_id=selected.candidate_id,
        lock_selected=locked_by_user, task_config=task_config,
    )
    leg_refs, arrival_s = _leg_refs_and_arrival_s(built.transition_requirements, project, built.case, library, leg_audit_cache=leg_audit_cache)
    compiled_actions = _compiled_actions(built.case, project, arrival_s)
    review = dict(built.case.review)
    review.update(
        {
            "state": "VALID",
            "approved": False,
            "collision_status": "PASSED",
            "incomplete_reason": "",
            "phase7_generated": True,
            "approval_required_reason": "generated working output requires explicit user approval before final export",
        }
    )
    selected_plan = dict(built.case.selected_plan)
    selected_plan["selection_state"] = selection_state
    selected_plan["route_selection_reason"] = (
        "LOCKED_BY_USER"
        if locked_by_user
        else "FASTEST_COMPLETE_GEOMETRY"
    )
    selected_plan["route_rule_match"] = bool(selected.route_rule_match)
    estimates = dict(built.case.estimates)
    estimates.update(
        {
            "planned_motion_time_ms": selected.motion_time_ms,
            "planned_motion_time_state": "LEG_LIBRARY_SUM",
            "planned_mechanism_time_ms": selected.mechanism_time_ms,
            "planned_mechanism_busy_time_ms": selected.mechanism_busy_time_ms,
            "planned_total_estimate_ms": selected.total_time_ms,
            "planned_total_estimate_state": "FIFO_OVERLAP_WITH_STOP_CARRY_FORWARD",
        }
    )
    hashes = dict(built.case.hashes)
    hashes["phase7_selected_candidate_hash"] = canonical_json_crc32_hex(selected.to_dict())
    case = replace(
        built.case,
        selected_plan=selected_plan,
        leg_refs=tuple(leg_refs),
        actions={"source": list(built.case.actions.get("source", [])), "compiled": compiled_actions},
        finish=_finish_policy(project),
        estimates=estimates,
        hashes=hashes,
        review=review,
    )
    trajectory = compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library, project=project))
    scan = _kinematic_check_starts_from_trajectory(case, project, trajectory, arrival_s)
    if scan:
        compiled_actions = _compiled_actions(built.case, project, arrival_s, kinematic_check_starts=scan)
        case = replace(case, actions={"source": list(built.case.actions.get("source", [])), "compiled": compiled_actions})
        compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library, project=project))
    return case, selected


def _leg_refs_and_arrival_s(
    requirements: Iterable[TransitionRequirement],
    project: ProjectV40,
    case: CaseManifestV40,
    library: LegLibraryV40,
    *,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    refs: list[dict[str, Any]] = []
    arrival_s: dict[str, int] = {}
    global_s = 0
    audit_cache = leg_audit_cache if leg_audit_cache is not None else {}
    for requirement in requirements:
        leg_id = _leg_id_for_transition(requirement, project, case)
        leg = _leg_by_id(library, leg_id)
        audit = _cached_leg_audit(project, leg, audit_cache)
        if leg is None or not audit.reusable:
            raise CompileError(f"missing or non-reusable leg for {requirement.requirement_id}: {leg_id}: {audit.reason}")
        refs.append({"leg_id": leg.leg_id, "expected_leg_hash32": str(leg.hashes.get("self_hash32", ""))})
        if len(leg.nodes) < 2:
            raise CompileError(f"leg {leg.leg_id} has fewer than two nodes")
        start_s = int(leg.nodes[0].get("local_s_mm", leg.nodes[0].get("s_mm", 0)))
        end_s = int(leg.nodes[-1].get("local_s_mm", leg.nodes[-1].get("s_mm", 0)))
        global_s += end_s - start_s
        arrival_s[requirement.to_state_id] = global_s
    return refs, arrival_s


def _compiled_actions(
    case: CaseManifestV40,
    project: ProjectV40,
    arrival_s: dict[str, int],
    *,
    kinematic_check_starts: dict[int, int] | None = None,
) -> list[dict[str, Any]]:
    compiled: list[dict[str, Any]] = []
    for item in case.actions.get("source", []):
        action = dict(item)
        if "check_start_s_mm" in action:
            raise CompileError("source actions must not contain check_start_s_mm; it is generated from the assembled trajectory")
        profile = dict(project.action_profiles.get(str(action.get("profile_key", action.get("action", ""))), {}))
        mode = _action_mode(action.get("mode", profile.get("mode", ActionMode.STOP_AND_WAIT.name)))
        merged = {**profile, **action}
        out: dict[str, Any] = {
            "action": merged.get("action"),
            "mode": mode.name,
            "timeout_ms": int(merged.get("timeout_ms", 1000)),
            "post_wait_ms": int(merged.get("post_wait_ms", 0)),
        }
        if mode == ActionMode.STOP_AND_WAIT:
            if "arrival_state_id" in merged:
                out["arrival_state_id"] = str(merged["arrival_state_id"])
            out["check_start_s_mm"] = 0xFFFF
        elif mode == ActionMode.ASYNC:
            out["check_start_s_mm"] = 0xFFFF
        elif mode == ActionMode.KINEMATIC:
            out["check_start_s_mm"] = int(
                kinematic_check_starts.get(len(compiled), _generated_check_start(merged, arrival_s))
                if kinematic_check_starts is not None
                else _generated_check_start(merged, arrival_s)
            )
            for key in ("accel_limit_mmps2", "beta_limit_ddegps2", "wz_limit_ddegps", "speed_limit_mmps", "stable_time_ms"):
                if key in merged:
                    out[key] = int(merged[key])
            if "stable_time_ms" not in out:
                out["stable_time_ms"] = 1
        compiled.append(out)
    if not compiled or not str(compiled[-1].get("action", "")).startswith("DROP_"):
        raise CompileError("final compiled action must be DROP_* for Phase 7 formal generation")
    return compiled


def _generated_check_start(action: dict[str, Any], arrival_s: dict[str, int]) -> int:
    state_id = str(action.get("arrival_state_id", ""))
    anchor_s = arrival_s.get(state_id, 0)
    lead = int(action.get("check_lead_s_mm", 0))
    return max(0, anchor_s - lead)


def _kinematic_check_starts_from_trajectory(
    case: CaseManifestV40,
    project: ProjectV40,
    trajectory: CompiledTrajectoryV40,
    arrival_s: dict[str, int],
) -> dict[int, int]:
    check_starts: dict[int, int] = {}
    source_actions = list(case.actions.get("source", []))
    compiled_actions = list(case.actions.get("compiled", []))
    metrics = _trajectory_node_metrics(trajectory)
    for index, item in enumerate(compiled_actions):
        mode = _action_mode(item.get("mode", ActionMode.STOP_AND_WAIT.name))
        if mode != ActionMode.KINEMATIC:
            continue
        source = dict(source_actions[index]) if index < len(source_actions) else {}
        profile = dict(project.action_profiles.get(str(source.get("profile_key", source.get("action", ""))), {}))
        merged = {**profile, **source, **item}
        anchor_s = arrival_s.get(str(source.get("arrival_state_id", "")), trajectory.header.total_length_mm)
        lead = int(merged.get("check_lead_s_mm", 0))
        lower_s = max(0, anchor_s - lead)
        check_starts[index] = _scan_kinematic_check_start(metrics, lower_s, anchor_s, merged)
    return check_starts


def _trajectory_node_metrics(trajectory: CompiledTrajectoryV40) -> tuple[dict[str, float], ...]:
    nodes = trajectory.nodes
    segment_by_node: dict[int, float] = {}
    for segment in trajectory.segments:
        span = max(1, segment.end_node_index - segment.start_node_index)
        per_step_ms = float(segment.planned_time_ms) / span if segment.planned_time_ms > 0 else 0.0
        for index in range(segment.start_node_index + 1, segment.end_node_index + 1):
            segment_by_node[index] = max(segment_by_node.get(index, 0.0), per_step_ms)
    metrics: list[dict[str, float]] = []
    previous_speed = 0.0
    previous_wz = 0.0
    for index, node in enumerate(nodes):
        speed = math.hypot(float(node.vx_mmps), float(node.vy_mmps))
        wz = abs(float(node.wz_ddegps))
        dt_s = max(segment_by_node.get(index, 0.0) / 1000.0, 1.0e-9)
        accel = abs(speed - previous_speed) / dt_s if index else 0.0
        beta = abs(wz - previous_wz) / dt_s if index else 0.0
        metrics.append({"s_mm": float(node.s_mm), "speed": speed, "wz": wz, "accel": accel, "beta": beta})
        previous_speed = speed
        previous_wz = wz
    return tuple(metrics)


def _scan_kinematic_check_start(metrics: tuple[dict[str, float], ...], lower_s: int, anchor_s: int, action: dict[str, Any]) -> int:
    speed_limit = int(action.get("speed_limit_mmps", 0))
    wz_limit = int(action.get("wz_limit_ddegps", 0))
    accel_limit = int(action.get("accel_limit_mmps2", 0))
    beta_limit = int(action.get("beta_limit_ddegps2", 0))
    candidates = [item for item in metrics if lower_s <= item["s_mm"] <= anchor_s]
    if not candidates:
        return max(0, int(lower_s))
    for item in candidates:
        if speed_limit and item["speed"] > speed_limit:
            continue
        if wz_limit and item["wz"] > wz_limit:
            continue
        if accel_limit and item["accel"] > accel_limit:
            continue
        if beta_limit and item["beta"] > beta_limit:
            continue
        return max(0, round(item["s_mm"]))
    return max(0, round(candidates[-1]["s_mm"]))


def _action_mode(value: object) -> ActionMode:
    if isinstance(value, ActionMode):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return ActionMode(value)
    text = str(value)
    if text.startswith("ACTION_MODE_"):
        text = text[len("ACTION_MODE_") :]
    return ActionMode[text]


def _candidate_timing_sort_key(timing: CandidateTiming) -> tuple[int, int, int, str]:
    return (
        0 if timing.complete else 1,
        timing.total_time_ms,
        0 if timing.route_rule_match else 1,
        timing.candidate_id,
    )


def _select_phase8_candidate(
    row: RouteCaseRowV40,
    existing_case: CaseManifestV40 | None,
    timings: tuple[CandidateTiming, ...],
    complete: list[CandidateTiming],
) -> tuple[CandidateTiming, str, bool]:
    locked_candidate_id = ""
    locked_semantic_hash = ""
    if existing_case is not None and existing_case.selected_plan.get("locked_by_user"):
        locked_candidate_id = str(existing_case.selected_plan.get("candidate_id", ""))
        locked_semantic_hash = str(existing_case.selected_plan.get("semantic_hash", ""))
    if locked_candidate_id:
        by_id = {item.candidate_id: item for item in timings}
        locked = by_id.get(locked_candidate_id)
        if locked is None:
            raise CompileError(f"LOCK_CONFLICT P{row.traj_id:04d}: locked candidate is not available: {locked_candidate_id}")
        if locked_semantic_hash and locked.semantic_hash != locked_semantic_hash:
            raise CompileError(f"LOCK_CONFLICT P{row.traj_id:04d}: locked candidate semantic_hash changed: {locked_candidate_id}")
        if not locked.complete:
            raise CompileError(f"LOCK_CONFLICT P{row.traj_id:04d}: locked candidate is incomplete: {locked_candidate_id}")
        return locked, "LOCKED_PRESERVED_PHASE8", True
    selected = min(complete, key=_candidate_timing_sort_key)
    return selected, "FASTEST_COMPLETE_PHASE8", False


def _load_existing_task_case(layout: ProjectLayout, traj_id: int) -> CaseManifestV40 | None:
    path = layout.case_json_path_for_mode(traj_id, GenerationMode.FULL_AUTO)
    if path.exists():
        case = load_case(path)
        if case.generation_mode == GenerationMode.FULL_AUTO:
            return case
    return None


def _existing_case_path(layout: ProjectLayout, traj_id: int, generation_mode: GenerationMode) -> Path:
    mode_path = layout.case_json_path_for_mode(traj_id, generation_mode)
    if mode_path.exists():
        return mode_path
    raise CompileError(f"P{traj_id:04d} case JSON not found for {generation_mode.value}")


def _case_dependency_failures(
    case: CaseManifestV40,
    project: ProjectV40,
    library: LegLibraryV40,
    *,
    leg_audit_cache: dict[str, _ReusableLegAudit] | None = None,
) -> list[str]:
    failures: list[str] = []
    cache = leg_audit_cache if leg_audit_cache is not None else {}
    for ref in case.leg_refs:
        leg_id = str(ref.get("leg_id", ""))
        leg = _leg_by_id(library, leg_id)
        audit = _cached_leg_audit(project, leg, cache)
        if not audit.reusable:
            failures.append(f"{leg_id}: {audit.reason}")
    return failures


def _cached_leg_audit(
    project: ProjectV40,
    leg: LegV40 | None,
    cache: dict[str, _ReusableLegAudit],
) -> _ReusableLegAudit:
    if leg is None:
        return _ReusableLegAudit(False, "missing leg")
    cached = cache.get(leg.leg_id)
    if cached is not None:
        return cached
    audit = _active_leg_audit(project, leg)
    cache[leg.leg_id] = audit
    return audit


def _active_leg_audit(project: ProjectV40, leg: LegV40) -> _ReusableLegAudit:
    if leg.state not in REUSABLE_LEG_STATES:
        return _ReusableLegAudit(False, f"leg.state={leg.state.value}")
    review_state = str(leg.review.get("state", leg.state.value)).upper()
    if review_state in {"PREVIEW_VALID", "STALE", "FAILED", "CANCELLED", "TIMEOUT", "MISSING"}:
        return _ReusableLegAudit(False, f"review.state={review_state}")
    stale = leg_stale_reasons(leg, project)
    if stale:
        return _ReusableLegAudit(False, "stale dependency: " + "; ".join(stale))
    validation = validate_leg(project, leg)
    if not validation.get("valid", False):
        return _ReusableLegAudit(False, "validation failed", validation)
    return _ReusableLegAudit(True, "", validation)


def _leg_id_for_transition(transition: TransitionRequirement, project: ProjectV40, case: CaseManifestV40 | None) -> str:
    request = leg_request_from_transition(
        transition,
        project,
        yaw_policy=_case_yaw_policy(case, transition) if case is not None else YawPolicy.SHORTEST,
    )
    return leg_id_from_key(leg_key_from_request(request))


def _leg_by_id(library: LegLibraryV40, leg_id: str) -> LegV40 | None:
    for leg in library.legs:
        if leg.leg_id == leg_id:
            return leg
    return None


def _case_yaw_policy(
    case: CaseManifestV40 | None,
    transition: TransitionRequirement | None = None,
) -> YawPolicy:
    if case is None:
        return YawPolicy.SHORTEST
    if transition is not None and not (
        transition.from_state_id.startswith("DROP_STEP_")
        and transition.to_state_id.startswith("DROP_STEP_")
    ):
        return YawPolicy.SHORTEST
    try:
        return YawPolicy(str(case.selected_plan.get("yaw_direction", YawPolicy.SHORTEST.value)))
    except ValueError:
        return YawPolicy.SHORTEST


def _yaw_policy_from_usage(layout: ProjectLayout, target: UniqueLegRequirement) -> YawPolicy:
    if not target.usage:
        return YawPolicy.SHORTEST
    first = target.usage[0]
    table = _ensure_route_case_table(layout)
    row = _row_by_traj_id(table, first.traj_id)
    project = load_project(layout.project_json)
    task_config = ensure_competition_task_config(layout.competition_task_config_json)
    case = build_case_draft(
        row, project, preferred_candidate_id=first.candidate_id, task_config=task_config
    ).case
    return _case_yaw_policy(case, target.transition)


def _finish_policy(project: ProjectV40) -> dict[str, Any]:
    source = dict(project.finish_policy)
    return {"mode": "AT_FINAL_DROP", "signal_flags": int(source.get("signal_flags", source.get("finish_signal_flags", 0)))}


def _write_json(path: Path, value: dict[str, Any]) -> None:
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != value:
            raise WriteBackValidationError(f"JSON report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)


def _write_batch_summary(path: Path, results: list[GeneratedCaseResult], failures: list[BatchGenerationFailure]) -> None:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(["traj_id", "status", "candidate_id", "motion_time_ms", "mechanism_time_ms", "total_time_ms", "bin_size", "bin_crc32", "error"])
    for result in sorted(results, key=lambda item: item.traj_id):
        writer.writerow(
            [
                result.traj_id,
                "OK",
                result.selected_candidate_id,
                result.timing.motion_time_ms,
                result.timing.mechanism_time_ms,
                result.timing.total_time_ms,
                result.output.byte_size,
                result.output.hashes.get("bin_crc32", ""),
                "",
            ]
        )
    for failure in sorted(failures, key=lambda item: item.traj_id):
        writer.writerow([failure.traj_id, "FAILED", "", "", "", "", 0, "", failure.error])
    data = buffer.getvalue().encode("utf-8")

    def validator(temp_path: Path) -> None:
        if temp_path.read_bytes() != data:
            raise WriteBackValidationError(f"batch summary write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
