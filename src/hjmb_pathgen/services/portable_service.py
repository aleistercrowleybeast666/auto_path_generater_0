"""Portable V4.0 case export/import helpers."""

from __future__ import annotations

from dataclasses import replace

from hjmb_pathgen.models.enums import LegState, StorageMode
from hjmb_pathgen.models.errors import CompileError, MissingDependencyError, StaleDependencyError
from hjmb_pathgen.models.leg import LegLibraryV40, LegV40
from hjmb_pathgen.models.route_case import CaseManifestV40, PortableCaseV40

from .case_compiler import CaseCompileRequest, compile_case_to_trajectory

REUSABLE_LEG_STATES = {LegState.VALID, LegState.APPROVED, LegState.LOCKED}


def export_portable_case(case: CaseManifestV40, leg_library: LegLibraryV40) -> PortableCaseV40:
    if case.storage_mode != StorageMode.REFERENCED:
        raise CompileError(f"P{case.traj_id:04d} portable export requires a REFERENCED case")
    compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=leg_library))
    legs_by_id = {leg.leg_id: leg for leg in leg_library.legs}
    embedded: list[dict] = []
    seen: set[str] = set()
    for ref in case.leg_refs:
        leg_id = str(ref.get("leg_id", ""))
        if leg_id in seen:
            continue
        leg = legs_by_id.get(leg_id)
        if leg is None:
            raise MissingDependencyError(f"missing referenced leg: {leg_id}")
        _validate_portable_leg(leg)
        embedded.append(leg.to_dict())
        seen.add(leg_id)
    portable = replace(
        case,
        storage_mode=StorageMode.EMBEDDED,
        embedded_legs=tuple(embedded),
    )
    return PortableCaseV40(**portable.__dict__)


def _validate_portable_leg(leg: LegV40) -> None:
    if leg.state not in REUSABLE_LEG_STATES:
        raise StaleDependencyError(f"leg {leg.leg_id} is not reusable: {leg.state.value}")
