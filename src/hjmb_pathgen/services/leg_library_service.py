"""Phase 6 leg library mutation helpers."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.codec.json_codec import load_leg_library, save_leg_library
from hjmb_pathgen.models.enums import LegState
from hjmb_pathgen.models.errors import CompileError
from hjmb_pathgen.models.leg import LegLibraryV40, LegV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.planning.leg_optimizer import PLANNER_ALGORITHM_VERSION

REUSABLE_LEG_STATES = {LegState.VALID, LegState.APPROVED, LegState.LOCKED}


def load_or_create_leg_library(path: str | Path, project: ProjectV40) -> LegLibraryV40:
    path = Path(path)
    if path.exists():
        return load_leg_library(path)
    return LegLibraryV40(
        planner_version=PLANNER_ALGORITHM_VERSION,
        project_hash=canonical_json_crc32_hex(project.to_dict()),
        legs=(),
    )


def save_leg_library_checked(path: str | Path, library: LegLibraryV40) -> None:
    save_leg_library(path, library)


def upsert_leg(library: LegLibraryV40, leg: LegV40, *, replace_existing: bool = False, force: bool = False) -> LegLibraryV40:
    legs = list(library.legs)
    for index, existing in enumerate(legs):
        if existing.leg_id != leg.leg_id:
            continue
        _ensure_can_overwrite(existing, replace_existing=replace_existing, force=force)
        legs[index] = _review_for_replacement(existing, leg)
        return replace(library, planner_version=PLANNER_ALGORITHM_VERSION, legs=tuple(legs))
    legs.append(leg)
    legs.sort(key=lambda item: item.leg_id)
    return replace(library, planner_version=PLANNER_ALGORITHM_VERSION, legs=tuple(legs))


def approve_leg(library: LegLibraryV40, leg_id: str, *, notes: str = "") -> LegLibraryV40:
    return _update_leg_review(library, leg_id, approved=True, locked=None, notes=notes)


def lock_leg(library: LegLibraryV40, leg_id: str, *, notes: str = "") -> LegLibraryV40:
    return _update_leg_review(library, leg_id, approved=None, locked=True, notes=notes)


def unlock_leg(library: LegLibraryV40, leg_id: str) -> LegLibraryV40:
    return _update_leg_review(library, leg_id, approved=None, locked=False, notes=None)


def show_leg(library: LegLibraryV40, leg_id: str) -> LegV40:
    for leg in library.legs:
        if leg.leg_id == leg_id:
            return leg
    raise CompileError(f"leg not found: {leg_id}")


def _ensure_can_overwrite(existing: LegV40, *, replace_existing: bool, force: bool) -> None:
    review = existing.review
    guarded = bool(review.get("approved")) or bool(review.get("locked")) or existing.state in {LegState.APPROVED, LegState.LOCKED}
    if guarded and not force:
        raise CompileError(f"leg {existing.leg_id} is approved/locked and will not be overwritten silently")
    if not replace_existing and not force:
        raise CompileError(f"leg already exists: {existing.leg_id}")


def _review_for_replacement(existing: LegV40, replacement: LegV40) -> LegV40:
    existing_hash = str(existing.hashes.get("validity_hash", ""))
    replacement_hash = str(replacement.hashes.get("validity_hash", ""))
    if existing_hash and existing_hash == replacement_hash:
        review = dict(replacement.review)
        review["approved"] = bool(existing.review.get("approved"))
        review["locked"] = bool(existing.review.get("locked"))
        review["notes"] = existing.review.get("notes", review.get("notes", ""))
        review["state"] = replacement.state.value
        return replace(replacement, review=review)
    review = dict(replacement.review)
    review["approved"] = False
    review["locked"] = False
    review["notes"] = ""
    review["approval_cleared_reason"] = "replacement validity_hash changed"
    return replace(replacement, review=review)


def _update_leg_review(
    library: LegLibraryV40,
    leg_id: str,
    *,
    approved: bool | None,
    locked: bool | None,
    notes: str | None,
) -> LegLibraryV40:
    legs = []
    found = False
    for leg in library.legs:
        if leg.leg_id != leg_id:
            legs.append(leg)
            continue
        found = True
        if leg.state not in REUSABLE_LEG_STATES:
            raise CompileError(f"leg {leg_id} is not reusable: {leg.state.value}")
        if not leg.hashes.get("validity_hash"):
            raise CompileError(f"leg {leg_id} has no validity_hash")
        review = dict(leg.review)
        if approved is not None:
            review["approved"] = approved
        if locked is not None:
            review["locked"] = locked
        if notes is not None:
            review["notes"] = notes
        review["state"] = leg.state.value
        legs.append(replace(leg, review=review))
    if not found:
        raise CompileError(f"leg not found: {leg_id}")
    return replace(library, legs=tuple(legs))
