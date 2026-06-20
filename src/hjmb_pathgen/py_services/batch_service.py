"""Phase 2 batch orchestration skeleton using the single-case output core."""

from __future__ import annotations

import csv
import io
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from .case_compiler import CaseCompileRequest
from .output_service import CaseOutputOptions, CaseOutputResult, write_case_outputs
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


@dataclass(frozen=True)
class BatchOutputFailure:
    traj_id: int | None
    error: str


@dataclass(frozen=True)
class BatchOutputResult:
    results: tuple[CaseOutputResult, ...]
    failures: tuple[BatchOutputFailure, ...]
    validation_report_path: Path
    batch_summary_path: Path


def write_batch_outputs(
    layout: ProjectLayout,
    requests: Iterable[CaseCompileRequest],
    options: CaseOutputOptions | None = None,
) -> BatchOutputResult:
    options = options or CaseOutputOptions(write_case_json=True, write_bin=True, write_portable=False, write_report=False)
    layout.ensure_directories()
    results: list[CaseOutputResult] = []
    failures: list[BatchOutputFailure] = []
    for request in list(requests):
        try:
            results.append(write_case_outputs(layout, request, options))
        except Exception as exc:  # noqa: BLE001 - batch records per-case failure and continues.
            traj_id = getattr(request.case, "traj_id", None)
            failures.append(BatchOutputFailure(traj_id=traj_id, error=str(exc)))

    validation_report_path = layout.reports_dir / "validation_report.json"
    batch_summary_path = layout.reports_dir / "batch_summary.csv"
    _write_validation_report(validation_report_path, results, failures)
    _write_batch_summary(batch_summary_path, results, failures)
    return BatchOutputResult(
        results=tuple(results),
        failures=tuple(failures),
        validation_report_path=validation_report_path,
        batch_summary_path=batch_summary_path,
    )


def _write_validation_report(path: Path, results: list[CaseOutputResult], failures: list[BatchOutputFailure]) -> None:
    report = {
        "format": "HJMB_PHASE2_BATCH_VALIDATION_REPORT",
        "development_phase": 2,
        "development_batch": True,
        "phase3_started": False,
        "case_count": len(results),
        "failure_count": len(failures),
        "generated_360": False,
        "results": [
            {
                "traj_id": result.traj_id,
                "bin_size": result.byte_size,
                "hashes": result.hashes,
            }
            for result in results
        ],
        "failures": [{"traj_id": failure.traj_id, "error": failure.error} for failure in failures],
    }
    data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != report:
            raise ValueError("batch validation report write-back mismatch")

    atomic_write_bytes(path, data, validator=validator)


def _write_batch_summary(path: Path, results: list[CaseOutputResult], failures: list[BatchOutputFailure]) -> None:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(["traj_id", "status", "bin_size", "bin_crc32", "error"])
    for result in results:
        writer.writerow([result.traj_id, "OK", result.byte_size, result.hashes.get("bin_crc32", ""), ""])
    for failure in failures:
        writer.writerow(["" if failure.traj_id is None else failure.traj_id, "FAILED", 0, "", failure.error])
    data = buffer.getvalue().encode("utf-8")

    def validator(temp_path: Path) -> None:
        if temp_path.read_bytes() != data:
            raise ValueError("batch summary write-back mismatch")

    atomic_write_bytes(path, data, validator=validator)
