"""Phase 8 process worker for long-running path generation jobs."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.models.leg_optimization import LegOptimizationProfileName

from .mode_output_service import export_final_bin
from .phase7_generation_service import generate_all, generate_one, optimize_missing_legs, validate_all
from .project_service import ProjectLayout


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
        emit("FAILED", str(exc), errors=(str(exc),))
        messages.put({"kind": "error", "payload": {"job": job, "error": str(exc)}})


def _run_job(
    layout: ProjectLayout,
    job: str,
    params: dict[str, Any],
    cancel_event: mp.Event,
    emit: Any,
) -> dict[str, Any]:
    if job == "generate-one":
        _raise_if_cancelled(cancel_event)
        return generate_one(
            layout,
            int(params["traj_id"]),
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
            replace_manual=bool(params.get("replace_manual", False)),
        ).to_dict()
    if job == "generate-all":
        _raise_if_cancelled(cancel_event)
        return generate_all(
            layout,
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
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
    if job == "validate-all":
        _raise_if_cancelled(cancel_event)
        return validate_all(layout)
    if job == "export-final":
        _raise_if_cancelled(cancel_event)
        return export_final_bin(
            layout,
            int(params["traj_id"]),
            path_source=PathSource(str(params["path_source"])),
            profile_name=str(params.get("profile", "default")),
            dry_run=bool(params.get("dry_run", False)),
        ).to_dict()
    raise ValueError(f"unsupported worker job: {job}")


def _raise_if_cancelled(cancel_event: mp.Event) -> None:
    if cancel_event.is_set():
        raise RuntimeError("CANCELLED")
