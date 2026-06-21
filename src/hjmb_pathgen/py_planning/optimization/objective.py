"""Phase 6 optimizer objective helpers."""

from __future__ import annotations

from hjmb_pathgen.py_domain.leg_optimization import CandidateEvaluation


def candidate_objective_key(evaluation: CandidateEvaluation) -> tuple[int, float, str]:
    if not evaluation.success:
        return (1, float("inf"), evaluation.candidate_id)
    metrics = evaluation.max_metrics or {}
    curvature = float(metrics.get("max_abs_curvature_1_per_mm", 0.0))
    curvature_jump = float(metrics.get("max_curvature_jump_1_per_mm", 0.0))
    # Time remains the principal objective.  These soft penalties prevent a
    # marginally shorter A* polyline from winning by introducing a near-cusp
    # that forces the real chassis to brake hard.  Values below a roughly
    # 250 mm radius and normal sampled curvature variation are unpenalized.
    curvature_penalty = max(0.0, curvature - 1.0 / 250.0) * 45_000.0
    jump_penalty = max(0.0, curvature_jump - 0.0015) * 30_000.0
    clearance_penalty = (
        0.0
        if evaluation.min_clearance_mm is None
        else -float(evaluation.min_clearance_mm) * 1.0e-6
    )
    score = float(evaluation.planned_time_ms) + curvature_penalty + jump_penalty + clearance_penalty
    return (0, score, evaluation.candidate_id)
