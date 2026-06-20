"""Phase 6 directed leg optimization request/result models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Callable

from hjmb_pathgen.py_domain.enums import LegState, YawPolicy
from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_domain.planner_diagnostics import PlannerDiagnostic
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.topology import TopologyGate


class LegOptimizationProfileName(StrEnum):
    QUICK_PREVIEW = "QUICK_PREVIEW"
    STANDARD = "STANDARD"
    FINAL = "FINAL"


class LegFailureCategory(StrEnum):
    INVALID_REQUEST = "INVALID_REQUEST"
    TOPOLOGY_FAILED = "TOPOLOGY_FAILED"
    COLLISION_FAILED = "COLLISION_FAILED"
    TIME_PARAMETERIZATION_FAILED = "TIME_PARAMETERIZATION_FAILED"
    NO_VALID_CANDIDATE = "NO_VALID_CANDIDATE"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class Pose2D:
    x_mm: float
    y_mm: float
    yaw_ddeg: float

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, field_name: str) -> "Pose2D":
        missing = [key for key in ("x_mm", "y_mm", "yaw_ddeg") if key not in data]
        if missing:
            raise ValueError(f"{field_name} is missing pose fields: {', '.join(missing)}")
        return cls(x_mm=float(data["x_mm"]), y_mm=float(data["y_mm"]), yaw_ddeg=float(data["yaw_ddeg"]))

    def to_dict(self) -> dict[str, float]:
        return {"x_mm": self.x_mm, "y_mm": self.y_mm, "yaw_ddeg": self.yaw_ddeg}


ProgressCallback = Callable[[dict[str, Any]], None]


@dataclass(frozen=True)
class LegOptimizationRequest:
    project: ProjectV40
    from_state_id: str
    to_state_id: str
    from_pose: Pose2D
    to_pose: Pose2D
    route_family: str
    topology_profile: str
    topology_gates: tuple[TopologyGate, ...] = ()
    footprint_state: dict[str, Any] | None = None
    load_state: dict[str, Any] | None = None
    mechanism_state: dict[str, Any] | None = None
    unload_state: dict[str, Any] | None = None
    dependency_hashes: dict[str, Any] | None = None
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD
    seed: int = 0
    time_budget_ms: int | None = None
    initial_control_points: tuple[dict[str, Any], ...] = ()
    yaw_policy: YawPolicy = YawPolicy.SHORTEST
    warm_start_leg: LegV40 | None = None
    progress_callback: ProgressCallback | None = None
    cancel_check: Callable[[], bool] | None = None

    @property
    def dependency_payload(self) -> dict[str, Any]:
        return dict(self.dependency_hashes or {})


@dataclass(frozen=True)
class CandidateEvaluation:
    candidate_id: str
    source: str
    success: bool
    planned_time_ms: int = 0
    total_length_mm: float = 0.0
    min_clearance_mm: float | None = None
    failure_category: LegFailureCategory | None = None
    failure_reason: str = ""
    topology: dict[str, Any] | None = None
    collision: dict[str, Any] | None = None
    time_parameterization: dict[str, Any] | None = None
    max_metrics: dict[str, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "source": self.source,
            "success": self.success,
            "planned_time_ms": self.planned_time_ms,
            "total_length_mm": self.total_length_mm,
            "min_clearance_mm": self.min_clearance_mm,
            "failure_category": self.failure_category.value if self.failure_category else None,
            "failure_reason": self.failure_reason,
            "topology": self.topology,
            "collision": self.collision,
            "time_parameterization": self.time_parameterization,
            "max_metrics": self.max_metrics,
        }


@dataclass(frozen=True)
class LegOptimizationResult:
    success: bool
    state: LegState
    leg: LegV40 | None
    reason: str
    evaluations: tuple[CandidateEvaluation, ...]
    diagnostics: tuple[PlannerDiagnostic, ...] = ()
    elapsed_ms: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "state": self.state.value,
            "leg": self.leg.to_dict() if self.leg else None,
            "reason": self.reason,
            "evaluations": [evaluation.to_dict() for evaluation in self.evaluations],
            "diagnostics": [diagnostic.to_dict() for diagnostic in self.diagnostics],
            "elapsed_ms": self.elapsed_ms,
        }
