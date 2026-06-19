"""Formal export guard for Phase 5 collision validation state."""

from __future__ import annotations

from hjmb_pathgen.models.collision import CollisionStatus, PathCollisionResult
from hjmb_pathgen.models.route_case import CaseManifestV40
from hjmb_pathgen.models.validation import ExportGuardResult


def check_formal_export_guard(
    case: CaseManifestV40,
    *,
    collision_result: PathCollisionResult | None = None,
    require_collision_passed: bool = True,
    require_approval: bool = True,
) -> ExportGuardResult:
    reasons: list[str] = []
    state = str(case.review.get("state", "VALID")).upper()
    if state in {"STALE", "FAILED"}:
        reasons.append(f"review.state={state}")
    if require_approval and case.review.get("approved", True) is False:
        reasons.append("review.approved is false")
    if require_collision_passed:
        status = _collision_status(case, collision_result)
        if status != CollisionStatus.PASSED:
            reasons.append(f"collision_status={status.value}")
        if collision_result is not None and collision_result.errors:
            reasons.extend(f"collision_error: {error}" for error in collision_result.errors)
    return ExportGuardResult(allowed=not reasons, reasons=tuple(reasons))


def _collision_status(case: CaseManifestV40, collision_result: PathCollisionResult | None) -> CollisionStatus:
    if collision_result is not None:
        return collision_result.status
    value = case.review.get("collision_status", CollisionStatus.NOT_CHECKED.value)
    try:
        return CollisionStatus(str(value))
    except ValueError:
        return CollisionStatus.NOT_CHECKED
