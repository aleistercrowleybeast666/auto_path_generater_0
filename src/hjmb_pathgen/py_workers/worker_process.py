"""Phase 8 process worker for long-running path generation jobs."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project

from hjmb_pathgen.py_services.mode_case_service import generate_semi_auto
from hjmb_pathgen.py_services.mode_output_service import export_final_bin, write_manual_outputs
from hjmb_pathgen.py_services.case_compiler import CaseCompileRequest
from hjmb_pathgen.py_services.output_service import CaseOutputOptions, write_case_outputs
from hjmb_pathgen.py_services.phase7_generation_service import (
    generate_all,
    generate_one,
    optimize_leg_by_id,
    optimize_missing_legs,
    validate_one,
    validate_all,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


@dataclass(frozen=True)
class WorkerMessage:
    kind: str
    payload: dict[str, Any]


class WorkerJobHandle:
    def __init__(self, process: mp.Process, messages: mp.Queue, cancel_event: mp.Event) -> None:
        self._process = process
        self._messages = messages
        self._cancel_event = cancel_event

    def cancel(self) -> None:
        self._cancel_event.set()

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def poll(self) -> list[WorkerMessage]:
        messages: list[WorkerMessage] = []
        while True:
            try:
                item = self._messages.get_nowait()
            except queue.Empty:
                break
            messages.append(WorkerMessage(kind=str(item.get("kind", "")), payload=dict(item.get("payload", {}))))
        return messages

    def join(self, timeout: float | None = None) -> int | None:
        self._process.join(timeout)
        return self._process.exitcode


def start_worker_job(project_root: str | Path, job: str, params: dict[str, Any] | None = None) -> WorkerJobHandle:
    context = mp.get_context("spawn")
    messages: mp.Queue = context.Queue()
    cancel_event = context.Event()
    process = context.Process(
        target=_worker_main,
        args=(str(project_root), job, dict(params or {}), messages, cancel_event),
        daemon=True,
    )
    process.start()
    return WorkerJobHandle(process, messages, cancel_event)


def _worker_main(project_root: str, job: str, params: dict[str, Any], messages: mp.Queue, cancel_event: mp.Event) -> None:
    start = time.perf_counter()

    def emit(stage: str, message: str, **extra: Any) -> None:
        messages.put(
            {
                "kind": "progress",
                "payload": {
                    "stage": stage,
                    "message": message,
                    "elapsed_ms": round((time.perf_counter() - start) * 1000.0),
                    **extra,
                },
            }
        )

    try:
        emit("STARTED", job)
        if cancel_event.is_set():
            emit("CANCELLED", "cancelled before job start")
            messages.put({"kind": "cancelled", "payload": {"job": job}})
            return
        result = _run_job(ProjectLayout.open(project_root), job, params, cancel_event, emit)
        if cancel_event.is_set():
            emit("CANCELLED", "job completed after cancellation was requested")
            messages.put({"kind": "cancelled", "payload": {"job": job, "result": result}})
            return
        emit("COMPLETED", job)
        messages.put({"kind": "result", "payload": result})
    except Exception as exc:  # noqa: BLE001 - worker boundary serializes failures.
        if cancel_event.is_set() or str(exc) == "CANCELLED":
            emit("CANCELLED", "worker stopped by user")
            messages.put({"kind": "cancelled", "payload": {"job": job}})
            return
        emit("FAILED", str(exc), errors=(str(exc),))
        messages.put({"kind": "error", "payload": {"job": job, "error": str(exc)}})


def _run_job(
    layout: ProjectLayout,
    job: str,
    params: dict[str, Any],
    cancel_event: mp.Event,
    emit: Any,
) -> dict[str, Any]:
    if job in {"generate-one", "generate-full-auto-one"}:
        _raise_if_cancelled(cancel_event)
        emit("OPTIMIZING", "collecting and optimizing current Case dependencies", percent=1)
        optimization = optimize_missing_legs(
            layout,
            profile_name=LegOptimizationProfileName.STANDARD,
            seed=int(params.get("seed", 0)),
            traj_id=int(params["traj_id"]),
            cancel_check=cancel_event.is_set,
            progress_callback=lambda item: emit(
                "OPTIMIZING",
                "optimizing current Case legs",
                **{**item, "percent": max(1, round(int(item["percent"]) * 0.8))},
            ),
        ).to_dict()
        emit(
            "OPTIMIZED",
            "current Case dependencies ready",
            percent=80,
            optimized_count=optimization["optimized_count"],
            reused_count=optimization["skipped_count"],
            failed_count=optimization["failure_count"],
        )
        if cancel_event.is_set():
            return {"optimization": optimization}
        generation = generate_one(
            layout,
            int(params["traj_id"]),
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
        ).to_dict()
        emit("GENERATED", f"P{int(params['traj_id']):04d}", percent=100)
        return {"optimization": optimization, "generation": generation}
    if job in {"generate-all", "generate-full-auto-all"}:
        _raise_if_cancelled(cancel_event)
        emit("OPTIMIZING", "optimizing all missing/stale unique legs", percent=1)
        optimization = optimize_missing_legs(
            layout,
            profile_name=LegOptimizationProfileName.STANDARD,
            seed=int(params.get("seed", 0)),
            cancel_check=cancel_event.is_set,
            progress_callback=lambda item: emit(
                "OPTIMIZING",
                "optimizing unique legs",
                **{**item, "percent": max(1, round(int(item["percent"]) * 0.5))},
            ),
        ).to_dict()
        emit(
            "OPTIMIZED",
            "unique leg pass complete",
            percent=50,
            optimized_count=optimization["optimized_count"],
            reused_count=optimization["skipped_count"],
            failed_count=optimization["failure_count"],
        )
        if cancel_event.is_set():
            return {"optimization": optimization}
        generation = generate_all(
            layout,
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
            cancel_check=cancel_event.is_set,
            progress_callback=lambda item: emit(
                "GENERATING",
                "generating full-auto Cases",
                **{**item, "percent": 50 + round(int(item["percent"]) * 0.5)},
            ),
        ).to_dict()
        return {"optimization": optimization, "generation": generation}
    if job == "generate-semi-auto":
        _raise_if_cancelled(cancel_event)
        return generate_semi_auto(
            layout,
            int(params["traj_id"]),
            profile_name=LegOptimizationProfileName(
                str(params.get("profile", LegOptimizationProfileName.STANDARD.value))
            ),
            seed=int(params.get("seed", 0)),
            cancel_check=cancel_event.is_set,
            progress_callback=lambda item: emit(
                "OPTIMIZING",
                "optimizing semi-auto anchor legs",
                **item,
            ),
        ).to_dict()
    if job == "generate-manual":
        _raise_if_cancelled(cancel_event)
        traj_id = int(params["traj_id"])
        case = load_case(layout.case_json_path_for_mode(traj_id, GenerationMode.MANUAL))
        return write_manual_outputs(
            layout,
            case,
            profile_name=str(params.get("profile", "default")),
        ).to_dict()
    if job == "optimize-missing-legs":
        _raise_if_cancelled(cancel_event)
        emit("OPTIMIZING", "optimizing missing directed legs")
        return optimize_missing_legs(
            layout,
            profile_name=LegOptimizationProfileName(str(params.get("profile", LegOptimizationProfileName.STANDARD.value))),
            seed=int(params.get("seed", 0)),
            include_stale=bool(params.get("include_stale", True)),
            max_count=params.get("max_count"),
            force=bool(params.get("force", False)),
        ).to_dict()
    if job in {"optimize-current-leg", "reoptimize-current-leg"}:
        _raise_if_cancelled(cancel_event)
        emit("OPTIMIZING", f"optimizing selected directed leg {params['leg_id']}")
        return optimize_leg_by_id(
            layout,
            str(params["leg_id"]),
            profile_name=LegOptimizationProfileName(
                str(params.get("profile", LegOptimizationProfileName.STANDARD.value))
            ),
            seed=int(params.get("seed", 0)),
            force=job == "reoptimize-current-leg",
        )
    if job == "validate-all":
        _raise_if_cancelled(cancel_event)
        return validate_all(layout)
    if job == "validate-current":
        _raise_if_cancelled(cancel_event)
        traj_id = int(params["traj_id"])
        mode = GenerationMode(str(params["generation_mode"]))
        case = load_case(layout.case_json_path_for_mode(traj_id, mode))
        if mode == GenerationMode.MANUAL:
            return write_manual_outputs(
                layout,
                case,
                write_case_json=False,
                write_bin=False,
                write_report=False,
                dry_run=True,
            ).to_dict()
        if mode == GenerationMode.FULL_AUTO:
            return validate_one(layout, traj_id)
        return write_case_outputs(
            layout,
            CaseCompileRequest(
                case=case,
                leg_library=load_leg_library(layout.leg_library_json),
                project=load_project(layout.project_json),
            ),
            CaseOutputOptions(
                write_case_json=False,
                write_bin=False,
                write_portable=False,
                write_report=False,
                dry_run=True,
                require_approval=False,
                generation_mode=mode,
            ),
        ).to_dict()
    if job == "export-final":
        _raise_if_cancelled(cancel_event)
        return export_final_bin(
            layout,
            int(params["traj_id"]),
            generation_mode=GenerationMode(str(params["generation_mode"])),
            profile_name=str(params.get("profile", "default")),
            dry_run=bool(params.get("dry_run", False)),
            approve=bool(params.get("approve", False)),
        ).to_dict()
    raise ValueError(f"unsupported worker job: {job}")


def _raise_if_cancelled(cancel_event: mp.Event) -> None:
    if cancel_event.is_set():
        raise RuntimeError("CANCELLED")
