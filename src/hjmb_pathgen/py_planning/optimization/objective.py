"""Phase 6 optimizer objective helpers."""

from __future__ import annotations

from hjmb_pathgen.py_domain.leg_optimization import CandidateEvaluation


def candidate_objective_key(evaluation: CandidateEvaluation) -> tuple[int, float, str]:
    if not evaluation.success:
        return (1, float("inf"), evaluation.candidate_id)
    clearance_penalty = 0.0 if evaluation.min_clearance_mm is None else -float(evaluation.min_clearance_mm) * 1.0e-6
    return (0, float(evaluation.planned_time_ms) + clearance_penalty, evaluation.candidate_id)
