"""Phase 9 delivery validation, manifests, and profiling helpers."""

from __future__ import annotations

import hashlib
import json
import platform
import struct
import sys
import time
import tracemalloc
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from hjmb_pathgen.py_io.codecs.bin_codec import decode_trajectory, encode_trajectory, load_bin
from hjmb_pathgen.py_io.codecs.binary_layout import ACTION_FMT, HEADER_FMT, NODE_FMT, SEGMENT_FMT
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.crc32 import crc32_ieee
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_project, load_route_case_table
from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode, FinishMode, NodeFlag, GenerationMode, SegmentFlag
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.protocol import (
    ACTION_SIZE,
    BIN_VERSION,
    HEADER_SIZE,
    NODE_SIZE,
    SEGMENT_SIZE,
)

from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout

DROP_ACTIONS = {
    int(ActionCode.DROP_1),
    int(ActionCode.DROP_2),
    int(ActionCode.DROP_3),
    int(ActionCode.DROP_12),
    int(ActionCode.DROP_23),
}


@dataclass(frozen=True)
class Phase9CheckResult:
    name: str
    passed: bool
    details: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "passed": self.passed, "details": dict(self.details)}


def protocol_conformance_report(protocol_path: str | Path | None = None) -> dict[str, Any]:
    calcsize = {
        "HeaderV40": struct.calcsize(HEADER_FMT),
        "NodeV40": struct.calcsize(NODE_FMT),
        "SegmentV40": struct.calcsize(SEGMENT_FMT),
        "ActionV40": struct.calcsize(ACTION_FMT),
    }
    expected = {
        "HeaderV40": HEADER_SIZE,
        "NodeV40": NODE_SIZE,
        "SegmentV40": SEGMENT_SIZE,
        "ActionV40": ACTION_SIZE,
    }
    checks = [
        Phase9CheckResult("bin_version", BIN_VERSION == 40, {"actual": BIN_VERSION, "expected": 40}),
        Phase9CheckResult("struct_sizes", calcsize == expected, {"actual": calcsize, "expected": expected}),
        Phase9CheckResult("crc32_ieee_vector", crc32_ieee(b"123456789") == 0xCBF43926, {"actual": f"0x{crc32_ieee(b'123456789'):08X}", "expected": "0xCBF43926"}),
    ]
    if protocol_path is not None and Path(protocol_path).exists():
        text = Path(protocol_path).read_text(encoding="utf-8", errors="replace")
        required_fragments = (HEADER_FMT, NODE_FMT, SEGMENT_FMT, ACTION_FMT, "FINISH_MODE_AT_FINAL_DROP", "SAFE_END", "FINISH_CLEAR")
        missing = [fragment for fragment in required_fragments if fragment not in text]
        checks.append(Phase9CheckResult("protocol_text_fragments", not missing, {"missing": missing}))
    return {
        "format": "HJMB_PHASE9_PROTOCOL_CONFORMANCE_REPORT",
        "python": sys.version,
        "platform": platform.platform(),
        "checks": [check.to_dict() for check in checks],
        "passed": all(check.passed for check in checks),
    }


def output_layout_report(layout: ProjectLayout) -> dict[str, Any]:
    layout.ensure_directories()
    expected_dirs = {
        "cases/full_auto": layout.full_auto_cases_dir,
        "cases/semi_auto": layout.semi_auto_cases_dir,
        "cases/manual": layout.manual_cases_dir,
        "bin/full_auto": layout.full_auto_bin_dir,
        "bin/semi_auto": layout.semi_auto_bin_dir,
        "bin/manual": layout.manual_bin_dir,
        "bin/final": layout.final_bin_dir,
        "portable/full_auto": layout.full_auto_portable_dir,
        "portable/semi_auto": layout.semi_auto_portable_dir,
        "portable/manual": layout.manual_portable_dir,
    }
    mismatches: list[dict[str, Any]] = []
    mismatches.extend(_case_dir_mismatches(layout.full_auto_cases_dir, GenerationMode.FULL_AUTO))
    mismatches.extend(_case_dir_mismatches(layout.semi_auto_cases_dir, GenerationMode.SEMI_AUTO))
    mismatches.extend(_case_dir_mismatches(layout.manual_cases_dir, GenerationMode.MANUAL))
    return {
        "format": "HJMB_PHASE9_OUTPUT_LAYOUT_REPORT",
        "root": str(layout.root),
        "directories": {name: {"path": str(path), "exists": path.is_dir()} for name, path in expected_dirs.items()},
        "generation_mode_mismatch_count": len(mismatches),
        "generation_mode_mismatches": mismatches,
        "passed": all(path.is_dir() for path in expected_dirs.values()) and not mismatches,
    }


def generate_golden_manifest(layout: ProjectLayout, *, include_final: bool = True) -> dict[str, Any]:
    project_hash = canonical_json_crc32_hex(load_project(layout.project_json).to_dict()) if layout.project_json.exists() else ""
    table_hash = canonical_json_crc32_hex(load_route_case_table(layout.route_case_table_json).to_dict()) if layout.route_case_table_json.exists() else ""
    entries = []
    for source in GenerationMode:
        case_dir = layout.case_json_path_for_mode(0, source).parent
        bin_dir = layout.bin_path_for_mode(0, source).parent
        for case_path in sorted(case_dir.glob("P*.json")):
            case = load_case(case_path)
            bin_path = bin_dir / f"P{case.traj_id:04d}.BIN"
            entries.append(_case_manifest_entry(case_path, bin_path, source, project_hash, table_hash))
    final_entries = []
    if include_final:
        for bin_path in sorted(layout.final_bin_dir.glob("P*.BIN")):
            final_entries.append(_bin_entry(bin_path))
    payload = {
        "format": "HJMB_PHASE9_GOLDEN_MANIFEST_V40",
        "protocol_version": BIN_VERSION,
        "project_hash32": project_hash,
        "route_case_table_hash32": table_hash,
        "case_count": len(entries),
        "final_bin_count": len(final_entries),
        "entries": entries,
        "final_bins": final_entries,
    }
    payload["manifest_sha256"] = _stable_sha256({key: value for key, value in payload.items() if key != "manifest_sha256"})
    return payload


def final_drop_audit_from_bin(path: str | Path) -> dict[str, Any]:
    trajectory = load_bin(path)
    finish_indexes = [index for index, node in enumerate(trajectory.nodes) if node.flags & int(NodeFlag.FINISH_ARM)]
    safe_end_indexes = [index for index, node in enumerate(trajectory.nodes) if node.flags & int(NodeFlag.SAFE_END)]
    finish_clear_segments = [index for index, segment in enumerate(trajectory.segments) if segment.flags & int(SegmentFlag.FINISH_CLEAR)]
    final_node = trajectory.nodes[-1]
    final_action = trajectory.actions[-1] if trajectory.actions else None
    legacy_finish_fields = {
        "finish_axis": trajectory.header.finish_axis,
        "finish_direction": trajectory.header.finish_direction,
        "finish_line_mm": trajectory.header.finish_line_mm,
        "finish_envelope_margin_mm": trajectory.header.finish_envelope_margin_mm,
        "finish_stable_time_ms": trajectory.header.finish_stable_time_ms,
        "finish_brake_accel_mmps2": trajectory.header.finish_brake_accel_mmps2,
        "finish_max_runout_mm": trajectory.header.finish_max_runout_mm,
        "finish_hard_timeout_ms": trajectory.header.finish_hard_timeout_ms,
    }
    final_action_ok = (
        final_action is not None
        and final_action.action in DROP_ACTIONS
        and final_action.mode == int(ActionMode.STOP_AND_WAIT)
        and final_action.arrival_id == final_node.arrival_id
    )
    checks = {
        "finish_mode_at_final_drop": trajectory.header.finish_mode == int(FinishMode.AT_FINAL_DROP),
        "unique_finish_arm": finish_indexes == [len(trajectory.nodes) - 1],
        "final_node_is_arrival_exact_stop": bool(final_node.flags & int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS)) and final_node.vx_mmps == 0 and final_node.vy_mmps == 0 and final_node.wz_ddegps == 0,
        "final_action_drop_stop_and_wait": final_action_ok,
        "safe_end_reserved_zero": not safe_end_indexes,
        "finish_clear_reserved_zero": not finish_clear_segments,
        "legacy_finish_fields_zero": not any(legacy_finish_fields.values()),
    }
    return {
        "format": "HJMB_PHASE9_FINAL_DROP_AUDIT",
        "bin_path": str(path),
        "checks": checks,
        "passed": all(checks.values()),
        "finish_arm_indexes": finish_indexes,
        "safe_end_indexes": safe_end_indexes,
        "finish_clear_segments": finish_clear_segments,
        "legacy_finish_fields": legacy_finish_fields,
        "completion_contract": "final DROP_* DONE -> post_wait_ms elapsed -> FIFO empty -> chassis stopped at final ARRIVAL",
    }


def performance_profile(name: str, func: Callable[[], Any]) -> dict[str, Any]:
    tracemalloc.start()
    start = time.perf_counter()
    result = func()
    elapsed_ms = round((time.perf_counter() - start) * 1000.0)
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return {
        "format": "HJMB_PHASE9_PERFORMANCE_PROFILE",
        "name": name,
        "elapsed_ms": elapsed_ms,
        "peak_memory_bytes": peak,
        "current_memory_bytes": current,
        "result_summary": _result_summary(result),
    }


def release_manifest(root: str | Path) -> dict[str, Any]:
    root = Path(root).resolve(strict=False)
    files = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        rel = path.relative_to(root).as_posix()
        if _excluded_from_release(rel):
            continue
        data = path.read_bytes()
        files.append({"path": rel, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    payload = {
        "format": "HJMB_PHASE9_RELEASE_MANIFEST",
        "root": str(root),
        "file_count": len(files),
        "files": files,
    }
    payload["manifest_sha256"] = _stable_sha256({key: value for key, value in payload.items() if key != "manifest_sha256"})
    return payload


def write_json_report(path: str | Path, report: dict[str, Any]) -> Path:
    from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes

    path = Path(path)
    data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != report:
            raise CompileError(f"report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
    return path


def _case_dir_mismatches(case_dir: Path, expected_source: GenerationMode) -> list[dict[str, Any]]:
    mismatches: list[dict[str, Any]] = []
    for path in sorted(case_dir.glob("P*.json")):
        try:
            case = load_case(path)
        except Exception as exc:  # noqa: BLE001 - report boundary.
            mismatches.append({"path": str(path), "error": str(exc)})
            continue
        if case.generation_mode != expected_source:
            mismatches.append({"path": str(path), "actual": case.generation_mode.value, "expected": expected_source.value})
    return mismatches


def _case_manifest_entry(case_path: Path, bin_path: Path, source: GenerationMode, project_hash: str, table_hash: str) -> dict[str, Any]:
    case = load_case(case_path)
    entry = {
        "traj_id": case.traj_id,
        "generation_mode": source.value,
        "case_path": str(case_path),
        "case_hash32": canonical_json_crc32_hex(case.to_dict()),
        "selected_candidate_id": str(case.selected_plan.get("candidate_id", "")),
        "route_family": str(case.selected_plan.get("route_family", "")),
        "leg_ids": [str(ref.get("leg_id", "")) for ref in case.leg_refs],
        "leg_hashes": [str(ref.get("expected_leg_hash32", "")) for ref in case.leg_refs],
        "project_hash32": project_hash,
        "route_case_table_hash32": table_hash,
    }
    if bin_path.exists():
        entry.update(_bin_entry(bin_path))
    else:
        entry["bin_missing"] = True
    return entry


def _bin_entry(bin_path: Path) -> dict[str, Any]:
    data = bin_path.read_bytes()
    trajectory = decode_trajectory(data, expected_filename=bin_path)
    reencoded = encode_trajectory(trajectory)
    return {
        "bin_path": str(bin_path),
        "bin_sha256": hashlib.sha256(data).hexdigest(),
        "bin_crc32": f"{trajectory.header.file_crc32:08x}",
        "bin_size": len(data),
        "node_count": len(trajectory.nodes),
        "segment_count": len(trajectory.segments),
        "action_count": len(trajectory.actions),
        "planned_motion_time_ms": trajectory.header.planned_motion_time_ms,
        "planned_total_estimate_ms": trajectory.header.planned_total_estimate_ms,
        "roundtrip_byte_identical": reencoded == data,
        "final_drop": final_drop_audit_from_bin(bin_path)["passed"],
    }


def _stable_sha256(value: dict[str, Any]) -> str:
    data = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _result_summary(result: Any) -> dict[str, Any]:
    if isinstance(result, dict):
        keys = ("case_count", "failure_count", "passed", "manifest_sha256", "elapsed_ms")
        return {key: result[key] for key in keys if key in result}
    return {"type": type(result).__name__}


def _excluded_from_release(relative_path: str) -> bool:
    parts = relative_path.split("/")
    if any(part in {".git", ".venv", "__pycache__", "build", "dist", "release", ".pytest_cache", ".mypy_cache", ".ruff_cache"} for part in parts):
        return True
    if relative_path.endswith((".pyc", ".pyo", ".log", ".tmp", ".bak")):
        return True
    if relative_path.startswith(("cases/", "bin/", "portable/", "reports/", "cache/")):
        return True
    return False
