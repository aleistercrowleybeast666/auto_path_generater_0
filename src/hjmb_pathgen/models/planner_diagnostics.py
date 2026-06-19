"""Phase 6 optimizer diagnostic records."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class PlannerStage(StrEnum):
    INITIALIZING = "INITIALIZING"
    INITIAL_GUESS = "INITIAL_GUESS"
    TOPOLOGY = "TOPOLOGY"
    COLLISION = "COLLISION"
    TIME_PARAMETERIZATION = "TIME_PARAMETERIZATION"
    REFINEMENT = "REFINEMENT"
    WRITING_LIBRARY = "WRITING_LIBRARY"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class PlannerDiagnostic:
    stage: PlannerStage
    message: str
    candidate_id: str = ""
    data: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "stage": self.stage.value,
            "message": self.message,
            "candidate_id": self.candidate_id,
        }
        if self.data is not None:
            payload["data"] = dict(self.data)
        return payload
