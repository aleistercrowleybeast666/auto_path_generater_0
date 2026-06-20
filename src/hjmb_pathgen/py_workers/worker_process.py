"""Phase 8 process worker for long-running path generation jobs."""

from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project

from hjmb_pathgen.py_services.mode_case_service import generate_semi_auto
from hjmb_pathgen.py_services.mode_output_service import export_final_bin, write_manual_outputs, write_semi_auto_outputs
from hjmb_pathgen.py_services.case_compiler import CaseCompileRequest
from hjmb_pathgen.py_services.output_service import CaseOutputOptions, write_case_outputs
from hjmb_pathgen.py_services.phase7_generation_service import (
    generate_all,
    generate_one,
    evaluate_case_candidates,
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
        # Long numerical optimizers cannot always return to Python frequently
        # enough for cooperative cancellation.  Formal writes are atomic, so a
        # forced process stop is safer and gives the UI the immediate response
        # expected by the operator.
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(0.20)
        if self._process.is_alive() and hasattr(self._process, "kill"):
            self._process.kill()
            self._process.join(0.20)

    def is_alive(self) -> bool:
        return self._process.is_alive()

    def poll(self) -> list[WorkerMessage]:
        messages: list[WorkerMessage] = []
        while True:
            try:
                item = self._messages.get_nowait()
            except (queue.Empty, EOFError, OSError):
                break
            messages.append(WorkerMessage(kind=str(item.get("kind", "")), payload=dict(item.get("payload", {}))))
        return messages

    def join(self, timeout: float | None = None) -> int | None:
        self._process.join(timeout)
        return self._process.exitcode

    def close(self) -> None:
        """Release multiprocessing handles after the child has exited.

        The GUI starts a second clean process after FULL_AUTO optimization.
        Explicitly closing the first Queue/Process prevents Windows semaphore
        and resource-tracker handles from leaking into that continuation.
        """

        if self._process.is_alive():
            return
        try:
            self._messages.close()
            self._messages.join_thread()
        except (AttributeError, OSError, ValueError):
            pass
        try:
            self._process.close()
        except (AttributeError, OSError, ValueError):
            pass



class IsolatedCompileJobHandle:
    """Subprocess-backed handle for final FULL_AUTO assembly.

    It is launched directly by the GUI process rather than by another
    ``multiprocessing`` child.  This avoids inheriting the numerical runtime
    state left by the preceding optimizer worker.
    """

    def __init__(self, project_root: str | Path, params: dict[str, Any]) -> None:
        self._start = time.perf_counter()
        self._messages: list[WorkerMessage] = []
        self._finished_collected = False
        self._cancelled = False
        layout = ProjectLayout.open(project_root, create_dirs=True)
        layout.ensure_directories()
        traj_id = int(params["traj_id"])
        self._traj_id = traj_id
        self._result_path = layout.cache_dir / (
            f"full_auto_compile_{traj_id:04d}_{uuid.uuid4().hex}.json"
        )
        command = [
            sys.executable,
            "-m",
            "hjmb_pathgen.py_workers.full_auto_compile_entry",
            "--project-root",
            str(layout.root),
            "--traj-id",
            str(traj_id),
            "--result-file",
            str(self._result_path),
        ]
        if bool(params.get("write_portable", False)):
            command.append("--write-portable")
        if bool(params.get("dry_run", False)):
            command.append("--dry-run")
        environment = _subprocess_python_environment()
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=environment,
        )
        self._emit_progress("STARTED", "compile-full-auto-one")
        self._emit_progress(
            "ASSEMBLING",
            f"isolated evaluation and assembly for P{traj_id:04d}",
            percent=82,
        )

    def _elapsed_ms(self) -> int:
        return round((time.perf_counter() - self._start) * 1000.0)

    def _emit_progress(self, stage: str, message: str, **extra: Any) -> None:
        self._messages.append(
            WorkerMessage(
                kind="progress",
                payload={
                    "stage": stage,
                    "message": message,
                    "elapsed_ms": self._elapsed_ms(),
                    **extra,
                },
            )
        )

    def _collect_if_finished(self) -> None:
        if self._finished_collected or self._process.poll() is None:
            return
        self._finished_collected = True
        stdout, stderr = self._process.communicate()
        if self._cancelled:
            self._emit_progress("CANCELLED", "worker stopped by user")
            self._messages.append(
                WorkerMessage(kind="cancelled", payload={"job": "compile-full-auto-one"})
            )
            return
        try:
            if not self._result_path.exists():
                detail = (stderr or stdout or f"exit code {self._process.returncode}").strip()
                raise RuntimeError(
                    f"isolated FULL_AUTO assembly did not return a result: {detail}"
                )
            payload = json.loads(self._result_path.read_text(encoding="utf-8"))
            if not bool(payload.get("ok")):
                raise RuntimeError(
                    str(payload.get("error") or stderr or stdout or "unknown isolated assembly failure")
                )
            result = dict(payload["result"])
            self._emit_progress("GENERATED", f"P{self._traj_id:04d}", percent=100)
            self._emit_progress("COMPLETED", "compile-full-auto-one")
            self._messages.append(WorkerMessage(kind="result", payload=result))
        except Exception as exc:  # noqa: BLE001 - subprocess boundary.
            self._emit_progress("FAILED", str(exc), errors=(str(exc),))
            self._messages.append(
                WorkerMessage(
                    kind="error",
                    payload={"job": "compile-full-auto-one", "error": str(exc)},
                )
            )

    def cancel(self) -> None:
        if self._process.poll() is not None:
            return
        self._cancelled = True
        self._process.terminate()
        try:
            self._process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=0.5)
        self._collect_if_finished()

    def is_alive(self) -> bool:
        return self._process.poll() is None

    def poll(self) -> list[WorkerMessage]:
        self._collect_if_finished()
        messages = list(self._messages)
        self._messages.clear()
        return messages

    def join(self, timeout: float | None = None) -> int | None:
        try:
            self._process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None
        self._collect_if_finished()
        return self._process.returncode

    def close(self) -> None:
        if self._process.poll() is None:
            return
        try:
            if self._process.stdout is not None:
                self._process.stdout.close()
            if self._process.stderr is not None:
                self._process.stderr.close()
        finally:
            try:
                self._result_path.unlink(missing_ok=True)
            except OSError:
                pass


def _subprocess_python_environment() -> dict[str, str]:
    environment = os.environ.copy()
    runtime_paths = [str(item) for item in sys.path if item]
    existing = environment.get("PYTHONPATH", "")
    if existing:
        runtime_paths.append(existing)
    environment["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(runtime_paths))
    return environment

def start_worker_job(
    project_root: str | Path,
    job: str,
    params: dict[str, Any] | None = None,
) -> WorkerJobHandle | IsolatedCompileJobHandle:
    normalized_params = dict(params or {})
    if job == "compile-full-auto-one":
        return IsolatedCompileJobHandle(project_root, normalized_params)
    context = mp.get_context("spawn")
    messages: mp.Queue = context.Queue()
    cancel_event = context.Event()
    process = context.Process(
        target=_worker_main,
        args=(str(project_root), job, normalized_params, messages, cancel_event),
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
    if job == "generate-full-auto-one":
        _raise_if_cancelled(cancel_event)
        traj_id = int(params["traj_id"])

        # Keep optimization and final Case assembly in separate worker
        # processes.  SciPy/numerical backends may retain large native state
        # after an optimization pass; evaluating/assembling in the same child
        # used to stall indefinitely on some Windows installations.  Returning
        # a follow-up job lets the GUI launch a clean process immediately.
        initial = evaluate_case_candidates(layout, traj_id)
        ordered = sorted(
            initial.timings,
            key=lambda item: (
                0 if item.complete else 1,
                len(item.missing_leg_ids),
                item.total_time_ms,
                item.candidate_id,
            ),
        )
        compile_params = {
            "traj_id": traj_id,
            "write_portable": bool(params.get("write_portable", False)),
            "dry_run": bool(params.get("dry_run", False)),
        }
        if any(item.complete for item in ordered):
            emit("PREPARED", f"P{traj_id:04d} already has a complete candidate", percent=78)
            return {
                "phase": "PREPARED",
                "optimization": [],
                "candidate_evaluation": initial.to_dict(),
                "followup": {"job": "compile-full-auto-one", "params": compile_params},
            }

        optimization_passes: list[dict[str, Any]] = []
        for candidate_index, timing in enumerate(ordered):
            _raise_if_cancelled(cancel_event)
            emit(
                "OPTIMIZING",
                f"optimizing candidate {timing.candidate_id}",
                percent=max(1, round(72 * candidate_index / max(len(ordered), 1))),
                candidate_id=timing.candidate_id,
                missing_leg_count=len(timing.missing_leg_ids),
            )
            optimization_result = optimize_missing_legs(
                layout,
                profile_name=LegOptimizationProfileName.AUTOMATIC,
                seed=int(params.get("seed", 0)) + candidate_index * 100,
                traj_id=traj_id,
                candidate_id=timing.candidate_id,
                cancel_check=cancel_event.is_set,
                progress_callback=lambda item, candidate_id=timing.candidate_id: emit(
                    "OPTIMIZING",
                    str(item.get("optimizer_message") or "optimizing current candidate legs"),
                    **{
                        **item,
                        "candidate_id": candidate_id,
                        "percent": max(1, min(72, round(int(item.get("percent", 0)) * 0.72))),
                    },
                ),
            )
            optimization = optimization_result.to_dict()
            optimization_passes.append({"candidate_id": timing.candidate_id, **optimization})
            emit(
                "OPTIMIZED",
                f"candidate {timing.candidate_id} optimization pass complete",
                percent=75,
                candidate_id=timing.candidate_id,
                optimized_count=optimization["optimized_count"],
                reused_count=optimization["skipped_count"],
                failed_count=optimization["failure_count"],
                failures=optimization["failures"],
            )
            _raise_if_cancelled(cancel_event)

            # All requirements belonging to this candidate were either
            # reusable or optimized successfully.  Do not re-evaluate in this
            # process; launch the clean compile process instead.
            if optimization_result.failure_count == 0:
                emit(
                    "PREPARED",
                    f"candidate {timing.candidate_id} is ready for clean-process assembly",
                    percent=78,
                    candidate_id=timing.candidate_id,
                )
                return {
                    "phase": "PREPARED",
                    "optimization": optimization_passes,
                    "candidate_evaluation": initial.to_dict(),
                    "prepared_candidate_id": timing.candidate_id,
                    "followup": {"job": "compile-full-auto-one", "params": compile_params},
                }

        failure_reasons: list[str] = []
        for item in optimization_passes:
            for failure in item.get("failures", []):
                failure_reasons.append(
                    f"{failure.get('from_state_id', '?')}->{failure.get('to_state_id', '?')}: "
                    f"{failure.get('reason', 'optimization failed')}"
                )
        details = [
            f"{item.candidate_id}: missing {', '.join(item.missing_leg_ids) or 'none'}"
            for item in ordered
        ]
        message = (
            f"P{traj_id:04d} automatic optimization could not prepare a complete candidate: "
            + " | ".join(details)
        )
        if failure_reasons:
            message += "; failures: " + "; ".join(failure_reasons[-8:])
        raise RuntimeError(message)

    if job == "compile-full-auto-one":
        _raise_if_cancelled(cancel_event)
        traj_id = int(params["traj_id"])
        emit("ASSEMBLING", f"isolated evaluation and assembly for P{traj_id:04d}", percent=82)
        result = _compile_full_auto_in_isolated_interpreter(
            layout,
            traj_id,
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
            cancel_event=cancel_event,
        )
        emit("GENERATED", f"P{traj_id:04d}", percent=100)
        return result

    if job == "generate-one":
        _raise_if_cancelled(cancel_event)
        traj_id = int(params["traj_id"])
        emit("ASSEMBLING", f"evaluating complete candidates for P{traj_id:04d}", percent=82)
        evaluation = evaluate_case_candidates(layout, traj_id)
        if not any(item.complete for item in evaluation.timings):
            details = " | ".join(
                f"{item.candidate_id}: missing {', '.join(item.missing_leg_ids) or 'none'}"
                for item in evaluation.timings
            )
            raise RuntimeError(
                f"P{traj_id:04d} has no complete candidate: {details}"
            )
        generation = generate_one(
            layout,
            traj_id,
            write_portable=bool(params.get("write_portable", False)),
            dry_run=bool(params.get("dry_run", False)),
        ).to_dict()
        emit("GENERATED", f"P{traj_id:04d}", percent=100)
        return {
            "phase": "GENERATED",
            "candidate_evaluation": evaluation.to_dict(),
            "generation": generation,
        }
    if job in {"generate-all", "generate-full-auto-all"}:
        _raise_if_cancelled(cancel_event)
        emit("OPTIMIZING", "optimizing all missing/stale unique legs", percent=1)
        optimization = optimize_missing_legs(
            layout,
            profile_name=LegOptimizationProfileName.AUTOMATIC,
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
            cancel_check=cancel_event.is_set,
            progress_callback=lambda item: emit(
                str(item.get("stage", "PLANNING")),
                str(item.get("message", "planning ordered semi-auto path")),
                **{key: value for key, value in item.items() if key not in {"stage", "message"}},
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
            profile_name=_leg_profile(params),
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
            profile_name=_leg_profile(params),
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
        if mode == GenerationMode.SEMI_AUTO:
            return write_semi_auto_outputs(
                layout,
                case,
                write_case_json=False,
                write_bin=False,
                write_report=False,
                dry_run=True,
            ).to_dict()
        raise ValueError(f"unsupported validation mode: {mode.value}")
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


def _compile_full_auto_in_isolated_interpreter(
    layout: ProjectLayout,
    traj_id: int,
    *,
    write_portable: bool,
    dry_run: bool,
    cancel_event: mp.Event,
) -> dict[str, Any]:
    """Run final FULL_AUTO assembly in a pristine Python interpreter."""

    layout.ensure_directories()
    result_path = layout.cache_dir / f"full_auto_compile_{traj_id:04d}_{uuid.uuid4().hex}.json"
    command = [
        sys.executable,
        "-m",
        "hjmb_pathgen.py_workers.full_auto_compile_entry",
        "--project-root",
        str(layout.root),
        "--traj-id",
        str(traj_id),
        "--result-file",
        str(result_path),
    ]
    if write_portable:
        command.append("--write-portable")
    if dry_run:
        command.append("--dry-run")

    environment = _subprocess_python_environment()

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=environment,
    )
    try:
        while process.poll() is None:
            if cancel_event.is_set():
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=0.5)
                raise RuntimeError("CANCELLED")
            time.sleep(0.05)
        stdout, stderr = process.communicate()
        if not result_path.exists():
            detail = (stderr or stdout or f"exit code {process.returncode}").strip()
            raise RuntimeError(f"isolated FULL_AUTO assembly did not return a result: {detail}")
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        if not bool(payload.get("ok")):
            detail = str(payload.get("error") or stderr or stdout or "unknown isolated assembly failure")
            raise RuntimeError(detail)
        return dict(payload["result"])
    finally:
        try:
            result_path.unlink(missing_ok=True)
        except OSError:
            pass


def _raise_if_cancelled(cancel_event: mp.Event) -> None:
    if cancel_event.is_set():
        raise RuntimeError("CANCELLED")


def _leg_profile(params: dict[str, Any]) -> LegOptimizationProfileName:
    """Normalize old UI labels while keeping one production optimization mode."""

    raw = str(params.get("profile", LegOptimizationProfileName.STANDARD.value)).upper()
    aliases = {
        "QUICK": LegOptimizationProfileName.QUICK_PREVIEW.value,
        "DEFAULT": LegOptimizationProfileName.STANDARD.value,
        "OPTIMAL": LegOptimizationProfileName.STANDARD.value,
    }
    return LegOptimizationProfileName(aliases.get(raw, raw))
