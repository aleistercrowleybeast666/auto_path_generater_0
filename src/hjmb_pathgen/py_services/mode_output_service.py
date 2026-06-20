"""Phase 8 mode-aware working outputs and final BIN export."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from hjmb_pathgen.py_io.codecs.bin_codec import encode_trajectory, save_bin
from hjmb_pathgen.py_io.codecs.crc32 import crc32_ieee
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project, save_case
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.route_case import CaseManifestV40

from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from .case_compiler import CaseCompileRequest
from .export_guard_service import check_formal_export_guard
from .manual_path_service import plan_manual_case
from .semi_auto_path_service import plan_semi_auto_case, semi_case_with_derived_arrivals
from .output_service import CaseOutputOptions, CaseOutputResult, write_case_outputs
from .path_validation_service import case_with_collision_result, validate_case_collision
from .execution_time_estimator import (
    arrival_release_from_segments,
    estimate_fifo_execution,
    time_at_s_from_segments,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


def write_manual_outputs(
    layout: ProjectLayout,
    case: CaseManifestV40,
    *,
    profile_name: str = "default",
    write_case_json: bool = True,
    write_bin: bool = True,
    write_report: bool = True,
    dry_run: bool = False,
) -> CaseOutputResult:
    if case.generation_mode != GenerationMode.MANUAL:
        raise CompileError("write_manual_outputs requires a MANUAL case")
    layout.ensure_directories()
    project = load_project(layout.project_json)
    result = plan_manual_case(case, project, profile_name=profile_name)
    if result.trajectory is None:
        raise CompileError(f"P{case.traj_id:04d} manual planning failed: {result.timing.reason}")
    collision = validate_case_collision(case, project, samples=tuple(result.timing.samples), strict=True)
    if not collision.passed:
        raise CompileError(
            f"P{case.traj_id:04d} manual collision validation failed: {collision.status.value}"
        )
    execution = _trajectory_execution_estimate(case, project, result.trajectory)
    case = case_with_collision_result(case, collision)
    case = replace(
        case,
        estimates={
            **case.estimates,
            "planned_motion_time_ms": execution.motion_time_ms,
            "planned_mechanism_time_ms": execution.added_wait_time_ms,
            "planned_mechanism_busy_time_ms": execution.mechanism_busy_time_ms,
            "planned_total_estimate_ms": execution.total_time_ms,
            "planned_total_estimate_state": "FIFO_OVERLAP_WITH_STOP_CARRY_FORWARD",
            "action_timeline": list(execution.action_timeline),
        },
        review={**case.review, "state": "VALID"},
    )
    trajectory = _trajectory_with_estimate(result.trajectory, case, execution.total_time_ms)
    bin_bytes = encode_trajectory(trajectory)
    hashes = {"bin_crc32": f"{crc32_ieee(bin_bytes):08x}"}
    case_path = layout.case_json_path_for_mode(case.traj_id, GenerationMode.MANUAL) if write_case_json else None
    bin_path = layout.bin_path_for_mode(case.traj_id, GenerationMode.MANUAL) if write_bin else None
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.manual_report.json" if write_report else None
    if not dry_run:
        if case_path is not None:
            save_case(case_path, case)
        if bin_path is not None:
            save_bin(bin_path, trajectory)
        if report_path is not None:
            _write_report(
                report_path,
                {
                    "format": "HJMB_PHASE8_MANUAL_OUTPUT_REPORT",
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


def write_semi_auto_outputs(
    layout: ProjectLayout,
    case: CaseManifestV40,
    *,
    write_case_json: bool = True,
    write_bin: bool = True,
    write_report: bool = True,
    dry_run: bool = False,
    cancel_check: Callable[[], bool] | None = None,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
) -> CaseOutputResult:
    """Generate a SEMI_AUTO path from its exact ordered sparse point list."""

    if case.generation_mode != GenerationMode.SEMI_AUTO:
        raise CompileError("write_semi_auto_outputs requires a SEMI_AUTO case")
    layout.ensure_directories()
    project = load_project(layout.project_json)
    case = semi_case_with_derived_arrivals(case)
    planned = plan_semi_auto_case(
        case,
        project,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )
    if planned.trajectory is None:
        raise CompileError(f"P{case.traj_id:04d} semi-auto planning failed: {planned.timing.reason}")
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_COLLISION",
                "message": "执行严格连续碰撞校验",
                "percent": 84,
                "completed_count": 0,
                "total_count": 1,
            }
        )
    collision = validate_case_collision(case, project, samples=tuple(planned.timing.samples), strict=True)
    if not collision.passed:
        raise CompileError(_semi_collision_error(case.traj_id, collision))
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_COMPILE",
                "message": "碰撞通过，编译V4轨迹",
                "percent": 90,
                "completed_count": 1,
                "total_count": 1,
            }
        )
    execution = _trajectory_execution_estimate(case, project, planned.trajectory)
    checked_case = case_with_collision_result(case, collision)
    case = replace(
        checked_case,
        estimates={
            **checked_case.estimates,
            "planned_motion_time_ms": execution.motion_time_ms,
            "planned_mechanism_time_ms": execution.added_wait_time_ms,
            "planned_mechanism_busy_time_ms": execution.mechanism_busy_time_ms,
            "planned_total_estimate_ms": execution.total_time_ms,
            "planned_total_estimate_state": "FIFO_OVERLAP_WITH_STOP_CARRY_FORWARD",
            "action_timeline": list(execution.action_timeline),
        },
        review={
            **checked_case.review,
            "state": "VALID",
            "approved": False,
            "detached_from_library": True,
            "manual_override": True,
            "stale_reason": "",
        },
    )
    if cancel_check is not None and cancel_check():
        raise RuntimeError("CANCELLED")
    # Geometry/timing did not change when only estimates, review and collision
    # diagnostics were attached.  Refresh the source hash in-place instead of
    # running all nine yaw-window candidates a second time.
    trajectory = _trajectory_with_estimate(planned.trajectory, case, execution.total_time_ms)
    trajectory.validate()
    if progress_callback is not None:
        progress_callback(
            {
                "stage": "SEMI_WRITE",
                "message": "编码并原子写入Case/BIN",
                "percent": 92,
                "completed_count": 1,
                "total_count": 1,
            }
        )
    bin_bytes = encode_trajectory(trajectory)
    hashes = {"bin_crc32": f"{crc32_ieee(bin_bytes):08x}"}
    case_path = layout.case_json_path_for_mode(case.traj_id, GenerationMode.SEMI_AUTO) if write_case_json else None
    bin_path = layout.bin_path_for_mode(case.traj_id, GenerationMode.SEMI_AUTO) if write_bin else None
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.semi_auto_report.json" if write_report else None
    warnings = tuple(planned.timing.warnings)
    if not dry_run:
        if case_path is not None:
            save_case(case_path, case)
        if bin_path is not None:
            save_bin(bin_path, trajectory)
        if report_path is not None:
            _write_report(
                report_path,
                {
                    "format": "HJMB_SEMI_AUTO_OUTPUT_REPORT_V40",
                    "traj_id": case.traj_id,
                    "success": True,
                    "optimization": "FASTEST_FEASIBLE",
                    "route_family": case.selected_plan.get("route_family"),
                    "timing": planned.timing.to_dict(),
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
        warnings=warnings,
        bin_bytes=bin_bytes,
    )


def _trajectory_execution_estimate(case: CaseManifestV40, project: Any, trajectory: Any):
    releases = arrival_release_from_segments(trajectory.segments)
    return estimate_fifo_execution(
        project,
        case.actions.get("source", ()),
        motion_time_ms=int(trajectory.header.planned_motion_time_ms),
        arrival_release_ms=releases,
        compiled_actions=trajectory.actions,
        time_at_s_mm=time_at_s_from_segments(trajectory.segments),
    )


def _trajectory_with_estimate(trajectory: Any, case: CaseManifestV40, total_time_ms: int):
    updated = replace(
        trajectory,
        header=replace(
            trajectory.header,
            source_case_hash32=canonical_json_crc32(case.to_dict()),
            planned_total_estimate_ms=int(total_time_ms),
        ),
    ).normalized()
    updated.validate()
    return updated


def _semi_collision_error(traj_id: int, collision: Any) -> str:
    """Describe a collision failure without labelling geometry planning as failed."""

    first = collision.first_collision
    obstacle = (
        getattr(first, "obstacle_id", None)
        or collision.min_clearance_obstacle
        or "unknown obstacle"
    )
    pose = getattr(first, "pose", None) or collision.min_clearance_pose
    parts = [
        f"P{traj_id:04d} 半自动几何与速度规划已完成，但碰撞校验失败",
        f"status={collision.status.value}",
        f"obstacle={obstacle}",
    ]
    if pose is not None:
        parts.append(f"pose=({float(pose.x_mm):.1f}, {float(pose.y_mm):.1f}) mm")
    if collision.min_clearance_mm is not None:
        parts.append(f"min_clearance={float(collision.min_clearance_mm):.1f} mm")
    parts.append("请在该障碍附近补充或移动人工途径点后重新生成")
    return "; ".join(parts)


def export_final_bin(
    layout: ProjectLayout,
    traj_id: int,
    *,
    generation_mode: GenerationMode | str,
    profile_name: str = "default",
    dry_run: bool = False,
    approve: bool = False,
) -> CaseOutputResult:
    mode = generation_mode if isinstance(generation_mode, GenerationMode) else GenerationMode(str(generation_mode))
    case_path = layout.case_json_path_for_mode(traj_id, mode)
    if not case_path.exists():
        raise CompileError(f"P{traj_id:04d} {mode.value} case does not exist: {case_path}")
    case = load_case(case_path)
    if approve:
        if str(case.review.get("state", "STALE")) == "STALE":
            raise CompileError(f"P{traj_id:04d} final export blocked: Case is STALE")
        case = replace(case, review={**case.review, "approved": True})
    guard = check_formal_export_guard(case, require_collision_passed=True, require_approval=True)
    if not guard.allowed:
        raise CompileError(f"P{traj_id:04d} final export blocked: {'; '.join(guard.reasons)}")
    if mode == GenerationMode.FULL_AUTO:
        result = write_case_outputs(
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
                generation_mode=mode,
            ),
        )
    elif mode == GenerationMode.SEMI_AUTO:
        result = _export_semi_final(layout, case, dry_run=dry_run)
    elif mode == GenerationMode.MANUAL:
        result = _export_manual_final(layout, case, profile_name=profile_name, dry_run=dry_run)
    else:
        raise CompileError(f"unsupported final export mode: {mode.value}")
    if approve and not dry_run:
        save_case(case_path, case)
    return result


def _export_semi_final(
    layout: ProjectLayout,
    case: CaseManifestV40,
    *,
    dry_run: bool,
) -> CaseOutputResult:
    project = load_project(layout.project_json)
    planned = plan_semi_auto_case(case, project)
    if planned.trajectory is None:
        raise CompileError(f"P{case.traj_id:04d} semi-auto planning failed: {planned.timing.reason}")
    _require_final_drop(planned.trajectory, mode_name="SEMI_AUTO")
    bin_bytes = encode_trajectory(planned.trajectory)
    hashes = {"bin_crc32": f"{crc32_ieee(bin_bytes):08x}"}
    bin_path = layout.final_bin_path(case.traj_id)
    report_path = layout.reports_dir / f"P{case.traj_id:04d}.final_export_report.json"
    if not dry_run:
        save_bin(bin_path, planned.trajectory)
        _write_report(
            report_path,
            {
                "format": "HJMB_PHASE8_FINAL_EXPORT_REPORT",
                "traj_id": case.traj_id,
                "generation_mode": case.generation_mode.value,
                "optimization": "FASTEST_FEASIBLE",
                "hashes": hashes,
                "timing": planned.timing.to_dict(),
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
        warnings=tuple(planned.timing.warnings),
        bin_bytes=bin_bytes,
    )


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
    _require_final_drop(result.trajectory, mode_name="MANUAL")
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
                "generation_mode": case.generation_mode.value,
                "optimization": "FASTEST_FEASIBLE",
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


def _require_final_drop(trajectory, *, mode_name: str) -> None:
    from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode

    if not trajectory.actions:
        raise CompileError(f"{mode_name} final export requires a final DROP_* STOP_AND_WAIT action")
    action = trajectory.actions[-1]
    drop_codes = {
        int(ActionCode.DROP_1),
        int(ActionCode.DROP_2),
        int(ActionCode.DROP_3),
        int(ActionCode.DROP_12),
        int(ActionCode.DROP_23),
    }
    final_arrival = trajectory.header.arrival_count - 1
    if (
        action.action not in drop_codes
        or action.mode != int(ActionMode.STOP_AND_WAIT)
        or action.arrival_id != final_arrival
    ):
        raise CompileError(
            f"{mode_name} final export requires the final DROP_* STOP_AND_WAIT action bound to the final ARRIVAL"
        )


def _write_report(path: Path, value: dict[str, Any]) -> None:
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != value:
            raise CompileError(f"report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
