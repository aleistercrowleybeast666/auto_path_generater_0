"""Phase 8 helpers for clearing optimized leg results without editing project config."""

from __future__ import annotations

from dataclasses import dataclass, replace

from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, save_case
from hjmb_pathgen.py_domain.enums import LegState
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.leg import LegLibraryV40, LegV40

from .leg_library_service import save_leg_library_checked
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


@dataclass(frozen=True)
class ClearLegResult:
    leg_id: str
    previous_state: str
    new_state: str
    library_path: str
    stale_case_paths: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "leg_id": self.leg_id,
            "previous_state": self.previous_state,
            "new_state": self.new_state,
            "library_path": self.library_path,
            "stale_case_paths": list(self.stale_case_paths),
        }


def clear_optimized_leg_result(
    layout: ProjectLayout,
    leg_id: str,
    *,
    confirm_leg_id: str | None = None,
) -> ClearLegResult:
    library = load_leg_library(layout.leg_library_json)
    legs: list[LegV40] = []
    found: LegV40 | None = None
    for leg in library.legs:
        if leg.leg_id != leg_id:
            legs.append(leg)
            continue
        found = leg
        _ensure_clear_allowed(leg, confirm_leg_id=confirm_leg_id)
        legs.append(_cleared_leg(leg))
    if found is None:
        raise CompileError(f"leg not found: {leg_id}")
    updated = replace(library, legs=tuple(legs))
    save_leg_library_checked(layout.leg_library_json, updated)
    stale_case_paths = _mark_referencing_cases_stale(layout, leg_id)
    return ClearLegResult(
        leg_id=leg_id,
        previous_state=found.state.value,
        new_state=LegState.MISSING.value,
        library_path=str(layout.leg_library_json),
        stale_case_paths=stale_case_paths,
    )


def _ensure_clear_allowed(leg: LegV40, *, confirm_leg_id: str | None) -> None:
    guarded = bool(leg.review.get("approved")) or bool(leg.review.get("locked")) or leg.state in {LegState.APPROVED, LegState.LOCKED}
    if guarded and confirm_leg_id != leg.leg_id:
        raise CompileError(f"clearing approved/locked leg {leg.leg_id} requires --confirm-leg-id {leg.leg_id}")


def _cleared_leg(leg: LegV40) -> LegV40:
    review = dict(leg.review)
    review.update(
        {
            "approved": False,
            "locked": False,
            "state": LegState.MISSING.value,
            "cleared_reason": "PHASE8_CLEAR_OPTIMIZED_LEG_RESULT",
            "cleared_previous_state": leg.state.value,
        }
    )
    hashes = {
        key: value
        for key, value in leg.hashes.items()
        if key in {"dependency_hashes", "planner_algorithm_version"}
    }
    return replace(
        leg,
        state=LegState.MISSING,
        source="CLEARED_PHASE8",
        control_points=(),
        yaw_profile={},
        nodes=(),
        analysis={},
        hashes=hashes,
        review=review,
    )


def _mark_referencing_cases_stale(layout: ProjectLayout, leg_id: str) -> tuple[str, ...]:
    changed: list[str] = []
    for directory in (layout.semi_auto_cases_dir, layout.full_auto_cases_dir):
        for path in sorted(directory.glob("P*.json")):
            case = load_case(path)
            if not any(str(ref.get("leg_id", "")) == leg_id for ref in case.leg_refs):
                continue
            review = {
                **case.review,
                "state": "STALE",
                "approved": False,
                "stale_reason": f"referenced leg cleared: {leg_id}",
            }
            save_case(path, replace(case, review=review))
            changed.append(str(path))
    return tuple(changed)
