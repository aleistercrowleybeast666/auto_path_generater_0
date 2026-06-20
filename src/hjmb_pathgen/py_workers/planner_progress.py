"""Progress and cancellation primitives for long-running planners."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from threading import Event
from typing import Any, Callable


@dataclass(frozen=True)
class PlannerProgressEvent:
    stage: str
    message: str
    current_item: str = ""
    best_time_ms: int | None = None
    elapsed_ms: int = 0
    warnings: tuple[str, ...] = ()
    errors: tuple[str, ...] = ()
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "stage": self.stage,
            "message": self.message,
            "current_item": self.current_item,
            "best_time_ms": self.best_time_ms,
            "elapsed_ms": self.elapsed_ms,
            "warnings": list(self.warnings),
            "errors": list(self.errors),
            "data": dict(self.data),
        }


class CancellationToken:
    def __init__(self) -> None:
        self._event = Event()

    def cancel(self) -> None:
        self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()


ProgressSink = Callable[[PlannerProgressEvent], None]


class PlannerProgressReporter:
    def __init__(self, sink: ProgressSink | None = None) -> None:
        self._sink = sink
        self._start = time.perf_counter()

    def emit(self, stage: str, message: str, **kwargs: Any) -> None:
        if self._sink is None:
            return
        self._sink(
            PlannerProgressEvent(
                stage=stage,
                message=message,
                elapsed_ms=round((time.perf_counter() - self._start) * 1000.0),
                **kwargs,
            )
        )
