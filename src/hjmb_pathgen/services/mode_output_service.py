"""Phase 8 mode-aware working outputs and final BIN export."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from hjmb_pathgen.codec.bin_codec import encode_trajectory, save_bin
from hjmb_pathgen.codec.crc32 import crc32_ieee
from hjmb_pathgen.codec.json_codec import load_case, load_leg_library, load_project, save_case
from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.models.errors import CompileError
from hjmb_pathgen.models.route_case import CaseManifestV40

from .atomic_writer import atomic_write_bytes
from .case_compiler import CaseCompileRequest
from .export_guard_service import check_formal_export_guard
from .manual_path_service import plan_manual_case
from .output_service import CaseOutputOptions, CaseOutputResult, write_case_outputs
from .project_service import ProjectLayout


def write_manual_free_outputs(
    layout: ProjectLayout,
    case: CaseManifestV40,
    *,
    profile_name: str = "default",
    write_case_json: bool = True,
    write_bin: bool = True,
    write_report: bool = True,
    dry_run: bool = False,
) -> CaseOutputResult:
    if case.path_source != PathSource.MANUAL_FREE:
        raise CompileError("write_manual_free_outputs requires a MANUAL_FREE case")
    layout.ensure_directories()
    project = load_project(layout.project_json)
    result = plan_manual_case(case, project, profile_name=profile_name)
    if result.trajectory is None:
        raise CompileError(f"P{case.traj_id:04d} manual planning failed: {result.timing.reason}")
    bin_bytes = encode_trajectory(result.trajectory)
    hashes = {"bin_crc32": f"{crc32_ieee(bin_bytes):08x}"}
    case_path = layout.case_json_path_for_source(case.traj_id, PathSource.MANUAL_FREE) if write_case_json else None
    bin_path = layout.bin_path_for_source(case.traj_id, PathSource.MANUAL_FREE) if write_bin else None
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.manual_free_report.json" if write_report else None
    if not dry_run:
        if case_path is not None:
            save_case(case_path, case)
        if bin_path is not None:
            save_bin(bin_path, result.trajectory)
        if report_path is not None:
            _write_report(
                report_path,
                {
                    "format": "HJMB_PHASE8_MANUAL_FREE_OUTPUT_REPORT",
                    "traj_id": case.traj_id,
                    "success": True,
                    "profile_name": profile_name,
                    "timing": result.timing.to_dict(),
                    "hashes": hashes,
                },
            )
    return CaseOutputResult(
        traj_id=case.traj_id,
        case_path=case_path,
        bin_path=bin_path,
        portable_path=None,
        validation_report_path=report_path,
        hashes=hashes,
        byte_size=len(bin_bytes),
        warnings=(),
        bin_bytes=bin_bytes,
    )


def export_final_bin(
    layout: ProjectLayout,
    traj_id: int,
    *,
    path_source: PathSource | str,
    profile_name: str = "default",
    dry_run: bool = False,
) -> CaseOutputResult:
    source = path_source if isinstance(path_source, PathSource) else PathSource(str(path_source))
    case_path = layout.case_json_path_for_source(traj_id, source)
    if not case_path.exists():
        raise CompileError(f"P{traj_id:04d} {source.value} case does not exist: {case_path}")
    case = load_case(case_path)
    guard = check_formal_export_guard(case, require_collision_passed=True, require_approval=True)
    if not guard.allowed:
        raise CompileError(f"P{traj_id:04d} final export blocked: {'; '.join(guard.reasons)}")
    if source == PathSource.TASK_COMPILED:
        return write_case_outputs(
            layout,
            CaseCompileRequest(case=case, leg_library=load_leg_library(layout.leg_library_json), project=load_project(layout.project_json)),
            CaseOutputOptions(
                write_case_json=False,
                write_bin=True,
                write_portable=False,
                write_report=True,
                dry_run=dry_run,
                formal_competition=True,
                require_approval=True,
                final_bin=True,
                path_source=source,
            ),
        )
    if source == PathSource.MANUAL_FREE:
        return _export_manual_final(layout, case, profile_name=profile_name, dry_run=dry_run)
    raise CompileError(f"unsupported final export source: {source.value}")


def _export_manual_final(
    layout: ProjectLayout,
    case: CaseManifestV40,
    *,
    profile_name: str,
    dry_run: bool,
) -> CaseOutputResult:
    project = load_project(layout.project_json)
    result = plan_manual_case(case, project, profile_name=profile_name)
    if result.trajectory is None:
        raise CompileError(f"P{case.traj_id:04d} manual planning failed: {result.timing.reason}")
    bin_bytes = encode_trajectory(result.trajectory)
    hashes = {"bin_crc32": f"{crc32_ieee(bin_bytes):08x}"}
    bin_path = layout.final_bin_path(case.traj_id)
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.final_export_report.json"
    if not dry_run:
        save_bin(bin_path, result.trajectory)
        _write_report(
            report_path,
            {
                "format": "HJMB_PHASE8_FINAL_EXPORT_REPORT",
                "traj_id": case.traj_id,
                "path_source": case.path_source.value,
                "profile_name": profile_name,
                "hashes": hashes,
                "timing": result.timing.to_dict(),
            },
        )
    return CaseOutputResult(
        traj_id=case.traj_id,
        case_path=None,
        bin_path=bin_path,
        portable_path=None,
        validation_report_path=report_path,
        hashes=hashes,
        byte_size=len(bin_bytes),
        warnings=(),
        bin_bytes=bin_bytes,
    )


def _write_report(path: Path, value: dict[str, Any]) -> None:
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != value:
            raise CompileError(f"report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
