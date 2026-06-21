"""Phase 6 optimizer objective helpers."""

from __future__ import annotations

from hjmb_pathgen.py_domain.leg_optimization import CandidateEvaluation


# The chassis is omnidirectional.  Curvature is therefore a tracking-quality
# constraint, not a steering geometry objective.  The time parameterizer
# already slows genuinely tight curves through the lateral-acceleration limit.
# Keep only a small safety penalty for near-cusps; otherwise the measured
# planned time must decide so a longer, overly broad S curve cannot beat a
# shorter legal route merely because it has a larger radius.
_SOFT_MIN_RADIUS_MM = 160.0
_SEVERE_MIN_RADIUS_MM = 110.0
_SOFT_MAX_CURVATURE_JUMP = 0.0040


def candidate_objective_key(evaluation: CandidateEvaluation) -> tuple[int, float, float, str]:
    if not evaluation.success:
        return (1, float("inf"), float("inf"), evaluation.candidate_id)

    metrics = evaluation.max_metrics or {}
    curvature = abs(float(metrics.get("max_abs_curvature_1_per_mm", 0.0)))
    curvature_jump = abs(float(metrics.get("max_curvature_jump_1_per_mm", 0.0)))
    radius = (1.0 / curvature) if curvature > 1.0e-12 else float("inf")

    quality_penalty_ms = 0.0
    if radius < _SEVERE_MIN_RADIUS_MM:
        quality_penalty_ms += 10_000.0 + (_SEVERE_MIN_RADIUS_MM - radius) * 20.0
    elif radius < _SOFT_MIN_RADIUS_MM:
        quality_penalty_ms += (_SOFT_MIN_RADIUS_MM - radius) * 0.75
    quality_penalty_ms += max(0.0, curvature_jump - _SOFT_MAX_CURVATURE_JUMP) * 5_000.0

    score = float(evaluation.planned_time_ms) + quality_penalty_ms
    # Smoothness is only a deterministic tie-break after the time-dominant
    # score.  A tiny clearance preference avoids unstable exact ties without
    # changing any whole-millisecond decision.
    tie_break = curvature + curvature_jump
    if evaluation.min_clearance_mm is not None:
        tie_break -= min(float(evaluation.min_clearance_mm), 1000.0) * 1.0e-12
    return (0, score, tie_break, evaluation.candidate_id)
