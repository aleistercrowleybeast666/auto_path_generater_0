"""Unified V4.0 single-case output service."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from hjmb_pathgen.py_io.codecs.bin_codec import encode_trajectory, save_bin
from hjmb_pathgen.py_io.codecs.crc32 import crc32_ieee
from hjmb_pathgen.py_io.codecs.json_codec import load_leg_library, load_project, save_case, save_portable_case
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, PortableCaseV40

from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from .case_compiler import CaseCompileRequest, compile_case_to_trajectory
from .export_guard_service import check_formal_export_guard
from .full_auto_leg_source_service import effective_library_for_case_refs
from .portable_service import export_portable_case
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


@dataclass(frozen=True)
class CaseOutputOptions:
    write_case_json: bool = True
    write_bin: bool = True
    write_portable: bool = False
    write_report: bool = True
    dry_run: bool = False
    formal_competition: bool = False
    require_approval: bool = True
    generation_mode: GenerationMode | None = None
    final_bin: bool = False


@dataclass(frozen=True)
class CaseOutputResult:
    traj_id: int
    case_path: Path | None
    bin_path: Path | None
    portable_path: Path | None
    validation_report_path: Path | None
    hashes: dict[str, str]
    byte_size: int
    warnings: tuple[str, ...]
    bin_bytes: bytes | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "traj_id": self.traj_id,
            "case_path": str(self.case_path) if self.case_path else None,
            "bin_path": str(self.bin_path) if self.bin_path else None,
            "portable_path": str(self.portable_path) if self.portable_path else None,
            "validation_report_path": str(self.validation_report_path) if self.validation_report_path else None,
            "hashes": dict(self.hashes),
            "byte_size": self.byte_size,
            "warnings": list(self.warnings),
        }


def write_case_outputs(
    layout: ProjectLayout,
    request: CaseCompileRequest,
    options: CaseOutputOptions | None = None,
) -> CaseOutputResult:
    options = options or CaseOutputOptions()
    layout.ensure_directories()
    case = request.case
    _ensure_case_exportable(
        case,
        require_collision_passed=options.formal_competition,
        require_approval=options.require_approval,
    )
    leg_library = request.leg_library or _load_library_if_present(layout)
    project = request.project or _load_project_if_present(layout)
    if (
        case.generation_mode == GenerationMode.FULL_AUTO
        and leg_library is not None
        and project is not None
        and any(str(ref.get("selected_source", "")).upper() == "MANUAL_TEMPLATE" for ref in case.leg_refs)
    ):
        leg_library = effective_library_for_case_refs(layout, project, leg_library, case)
    effective_request = CaseCompileRequest(case=case, leg_library=leg_library, project=project)

    trajectory = None
    bin_bytes = None
    if options.write_bin or options.dry_run or options.write_report:
        trajectory = compile_case_to_trajectory(effective_request)
        bin_bytes = encode_trajectory(trajectory)

    generation_mode = options.generation_mode or case.generation_mode
    case_path = _case_path(layout, case.traj_id, generation_mode, options) if options.write_case_json else None
    bin_path = _bin_path(layout, case.traj_id, generation_mode, options) if options.write_bin else None
    portable_path = _portable_path(layout, case.traj_id, generation_mode, options) if options.write_portable else None
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.validation_report.json" if options.write_report else None

    portable_case: PortableCaseV40 | None = None
    if options.write_portable:
        if leg_library is None:
            raise CompileError("portable output requires a leg library for REFERENCED cases")
        portable_case = export_portable_case(case, leg_library)

    hashes = {}
    if bin_bytes is not None:
        hashes["bin_crc32"] = f"{crc32_ieee(bin_bytes):08x}"
    warnings: tuple[str, ...] = ()

    if not options.dry_run:
        if case_path is not None:
            save_case(case_path, case)
        if bin_path is not None:
            if trajectory is None:
                trajectory = compile_case_to_trajectory(effective_request)
            save_bin(bin_path, trajectory)
        if portable_path is not None:
            if portable_case is None:
                raise CompileError("portable output was requested but portable case was not built")
            save_portable_case(portable_path, portable_case)
        if report_path is not None:
            _write_case_report(report_path, _case_report_dict(case, bin_bytes, hashes, warnings))

    return CaseOutputResult(
        traj_id=case.traj_id,
        case_path=case_path,
        bin_path=bin_path,
        portable_path=portable_path,
        validation_report_path=report_path,
        hashes=hashes,
        byte_size=len(bin_bytes or b""),
        warnings=warnings,
        bin_bytes=bin_bytes,
    )


def _ensure_case_exportable(
    case: CaseManifestV40,
    *,
    require_collision_passed: bool = False,
    require_approval: bool = True,
) -> None:
    state = str(case.review.get("state", "VALID")).upper()
    if state in {"STALE", "FAILED"}:
        raise CompileError(f"P{case.traj_id:04d} cannot be exported while review.state={state}")
    if require_approval and case.review.get("approved", True) is False:
        raise CompileError(f"P{case.traj_id:04d} is not approved for export")
    if require_collision_passed:
        guard = check_formal_export_guard(
            case,
            require_collision_passed=True,
            require_approval=require_approval,
        )
        if not guard.allowed:
            raise CompileError(f"P{case.traj_id:04d} formal export blocked: {'; '.join(guard.reasons)}")


def _case_path(layout: ProjectLayout, traj_id: int, generation_mode: GenerationMode, options: CaseOutputOptions) -> Path:
    return layout.case_json_path_for_mode(traj_id, generation_mode)


def _bin_path(layout: ProjectLayout, traj_id: int, generation_mode: GenerationMode, options: CaseOutputOptions) -> Path:
    if options.final_bin:
        return layout.final_bin_path(traj_id)
    return layout.bin_path_for_mode(traj_id, generation_mode)


def _portable_path(layout: ProjectLayout, traj_id: int, generation_mode: GenerationMode, options: CaseOutputOptions) -> Path:
    return layout.portable_path_for_mode(traj_id, generation_mode)


def _load_library_if_present(layout: ProjectLayout) -> LegLibraryV40 | None:
    if layout.leg_library_json.exists():
        return load_leg_library(layout.leg_library_json)
    return None


def _load_project_if_present(layout: ProjectLayout) -> ProjectV40 | None:
    if layout.project_json.exists():
        return load_project(layout.project_json)
    return None


def _case_report_dict(case: CaseManifestV40, bin_bytes: bytes | None, hashes: dict[str, str], warnings: tuple[str, ...]) -> dict:
    return {
        "format": "HJMB_PHASE2_CASE_VALIDATION_REPORT",
        "development_phase": 2,
        "traj_id": case.traj_id,
        "case_name": f"P{case.traj_id:04d}",
        "bin_size": len(bin_bytes or b""),
        "hashes": hashes,
        "warnings": list(warnings),
        "phase3_started": False,
    }


def _write_case_report(path: Path, report: dict) -> None:
    data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != report:
            raise CompileError(f"report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
