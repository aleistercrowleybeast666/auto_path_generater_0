"""Phase 6 leg stale checks against project hashes and planner version."""

from __future__ import annotations

from dataclasses import replace

from hjmb_pathgen.py_domain.enums import LegState
from hjmb_pathgen.py_domain.leg import LegLibraryV40, LegV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_planning.optimization.leg_optimizer import PLANNER_ALGORITHM_VERSION
from hjmb_pathgen.py_services.project_config_service import compute_project_functional_hashes


def leg_stale_reasons(leg: LegV40, project: ProjectV40) -> tuple[str, ...]:
    reasons: list[str] = []
    current_hashes = compute_project_functional_hashes(project)
    stored_hashes = dict(leg.hashes.get("dependency_hashes", {}))
    for key, stored in stored_hashes.items():
        if key in current_hashes and str(stored) != str(current_hashes[key]):
            reasons.append(f"{key} changed")
    if str(leg.hashes.get("planner_algorithm_version", "")) != PLANNER_ALGORITHM_VERSION:
        reasons.append("planner_algorithm_version changed")
    return tuple(reasons)


def mark_stale_legs(library: LegLibraryV40, project: ProjectV40) -> LegLibraryV40:
    legs = []
    for leg in library.legs:
        reasons = leg_stale_reasons(leg, project)
        if not reasons:
            legs.append(leg)
            continue
        review = dict(leg.review)
        review["stale_previous_state"] = leg.state.value
        review["state"] = LegState.STALE.value
        review["stale_reason"] = "; ".join(reasons)
        legs.append(replace(leg, state=LegState.STALE, review=review))
    return replace(library, legs=tuple(legs))
