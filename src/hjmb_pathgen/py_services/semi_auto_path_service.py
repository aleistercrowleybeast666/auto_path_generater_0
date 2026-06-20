"""Deterministic SEMI_AUTO path resolution and planning.

SEMI_AUTO keeps one ordered sparse path.  Fixed START/ARRIVAL rows reference
project.json, while free WAYPOINT rows remain exactly where the user drew them.
No geometry search or leg-library substitution is performed.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Callable

from hjmb_pathgen.py_domain.compiled import CompiledTrajectoryV40
from hjmb_pathgen.py_domain.enums import GenerationMode, ManualPathPointType, SegmentFlag, YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.semi_path import SemiPathV40
from hjmb_pathgen.py_planning.dynamics.time_parameterization import TimeParameterizationResult

from .manual_path_service import retime_path, trajectory_from_deterministic_timing
from .competition_task_config_service import default_competition_task_config


@dataclass(frozen=True)
class SemiAutoCasePlanResult:
    case: CaseManifestV40
    resolved_manual_path: dict[str, Any]
    trajectory: CompiledTrajectoryV40 | None
    timing: TimeParameterizationResult

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.case.traj_id,
            "success": self.timing.success,
            "route_family": self.case.selected_plan.get("route_family"),
            "timing": self.timing.to_dict(),
            "node_count": len(self.trajectory.nodes) if self.trajectory else 0,
        }


def resolve_semi_path(case: CaseManifestV40, project: ProjectV40) -> dict[str, Any]:
    """Resolve fixed site references without copying them back into the Case JSON."""

    if case.generation_mode != GenerationMode.SEMI_AUTO:
        raise CompileError("resolve_semi_path requires a SEMI_AUTO case")
    if case.semi_path is None:
        raise CompileError("SEMI_AUTO case has no ordered semi_path")
    path = SemiPathV40.from_dict(case.semi_path, require_complete=True)
    result: list[dict[str, Any]] = []
    for index, point in enumerate(path.points):
        if point.point_type in (ManualPathPointType.START, ManualPathPointType.ARRIVAL):
            assert point.site_key is not None
            raw = project.sites.get(point.site_key)
            if not isinstance(raw, dict):
                raise CompileError(f"SEMI_AUTO point {index} references missing project site {point.site_key}")
            if not bool(raw.get("configured", False)):
                raise CompileError(f"SEMI_AUTO point {index} references unconfigured project site {point.site_key}")
            x_mm = int(raw["x_mm"])
            y_mm = int(raw["y_mm"])
            yaw_ddeg = int(raw["yaw_ddeg"])
            hints: dict[str, Any] = {
                "site_key": point.site_key,
                "state_id": point.state_id or point.site_key,
            }
            if point.point_type == ManualPathPointType.ARRIVAL and point.site_key.startswith("P_DROP_"):
                use_profiles = bool(
                    project.planner_profiles.get("default", {}).get(
                        "use_unload_pose_profiles", False
                    )
                )
                if point.unload_pose_profile_id:
                    config = default_competition_task_config()
                    spec = config.unload_pose_catalog.get(point.unload_pose_profile_id)
                    if not isinstance(spec, dict):
                        raise CompileError(
                            f"SEMI_AUTO point {index} has unknown unload pose profile "
                            f"{point.unload_pose_profile_id}"
                        )
                    if str(spec.get("station_site")) != point.site_key:
                        raise CompileError(
                            f"SEMI_AUTO point {index} profile {point.unload_pose_profile_id} "
                            f"belongs to {spec.get('station_site')}, not {point.site_key}"
                        )
                    profile = project.unload_pose_profiles.get(point.unload_pose_profile_id)
                    if not isinstance(profile, dict) or not bool(profile.get("configured", False)):
                        raise CompileError(
                            f"SEMI_AUTO point {index} references unconfigured unload pose "
                            f"{point.unload_pose_profile_id}"
                        )
                    x_mm += int(profile.get("dx_mm", 0))
                    y_mm += int(profile.get("dy_mm", 0))
                    yaw_ddeg = int(profile["yaw_ddeg"])
                    hints["unload_pose_profile_id"] = point.unload_pose_profile_id
                elif use_profiles:
                    raise CompileError(
                        f"SEMI_AUTO point {index} ({point.site_key}) must select an unload "
                        "pose profile while unload-angle selection is enabled"
                    )
            item = {
                "type": point.point_type.value,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "yaw_ddeg": yaw_ddeg,
                "exact_pass": True,
                "hints": hints,
            }
        else:
            item = {
                "type": ManualPathPointType.WAYPOINT.value,
                "x_mm": int(point.x_mm),
                "y_mm": int(point.y_mm),
                "exact_pass": bool(point.exact_pass),
                "corner_trim_mm": float(point.corner_trim_mm),
            }
            if point.max_speed_mmps is not None:
                item["max_speed_mmps"] = int(point.max_speed_mmps)
            if point.state_id:
                item["hints"] = {"state_id": point.state_id}
        result.append(item)
    result = _remove_redundant_waypoints(result)
    value: dict[str, Any] = {"points": result}
    if path.notes:
        value["notes"] = path.notes
    return value


def plan_semi_auto_case(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    cancel_check: Callable[[], bool] | None = None,
    progress_callback: Callable[[dict[str, Any]], None] | None = None,
) -> SemiAutoCasePlanResult:
    """Plan the fastest feasible deterministic path for the authored points.

    SEMI_AUTO intentionally has no user-selectable optimization profile.  The
    project default dynamics are always used and all deterministic low-speed
    yaw-window candidates are compared; this prevents old labels such as
    ``QUICK`` from leaking into the leg-optimizer enum.
    """
    if case.generation_mode != GenerationMode.SEMI_AUTO:
        raise CompileError("plan_semi_auto_case requires a SEMI_AUTO case")
    path = SemiPathV40.from_dict(case.semi_path, require_complete=True)
    resolved = resolve_semi_path(case, project)
    raw_policy = str(case.selected_plan.get("yaw_direction", YawPolicy.SHORTEST.value))
    try:
        yaw_policy = YawPolicy(raw_policy)
    except ValueError:
        yaw_policy = YawPolicy.SHORTEST
    timing = retime_path(
        resolved,
        project,
        profile_name="default",
        yaw_policy=yaw_policy,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )
    trajectory = None
    if timing.success:
        trajectory = trajectory_from_deterministic_timing(
            case,
            project,
            timing,
            route_family=path.route_family,
            segment_flags=int(SegmentFlag.NORMAL | SegmentFlag.MANUAL_OVERRIDE),
            header_manual_override=True,
        )
    return SemiAutoCasePlanResult(
        case=case,
        resolved_manual_path=resolved,
        trajectory=trajectory,
        timing=timing,
    )


def _remove_redundant_waypoints(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Drop only zero-length free WAYPOINTs adjacent to a fixed stop.

    A double-click on a configured fixed point can leave a free WAYPOINT at the
    same coordinate immediately before the ARRIVAL selected from the combo box.
    Removing that one point does not alter the authored polyline, while keeping
    it would create an undefined zero-length leg and make every planning profile
    fail.  Fixed START/ARRIVAL rows are never removed.
    """

    cleaned = list(points)
    changed = True
    while changed and len(cleaned) >= 2:
        changed = False
        for index, point in enumerate(cleaned):
            if str(point.get("type")) != ManualPathPointType.WAYPOINT.value:
                continue
            x = float(point["x_mm"])
            y = float(point["y_mm"])
            neighbours = []
            if index > 0:
                neighbours.append(cleaned[index - 1])
            if index + 1 < len(cleaned):
                neighbours.append(cleaned[index + 1])
            if any(
                (float(other["x_mm"]) - x) ** 2 + (float(other["y_mm"]) - y) ** 2 < 1.0
                and str(other.get("type")) in {
                    ManualPathPointType.START.value,
                    ManualPathPointType.ARRIVAL.value,
                }
                for other in neighbours
            ):
                del cleaned[index]
                changed = True
                break
    return cleaned


def semi_case_with_derived_arrivals(case: CaseManifestV40) -> CaseManifestV40:
    """Refresh diagnostic arrival states from the ordered path, not from old legs."""

    if case.semi_path is None:
        return case
    path = SemiPathV40.from_dict(case.semi_path, require_complete=True)
    arrivals: list[dict[str, Any]] = []
    for point in path.points:
        if point.point_type != ManualPathPointType.ARRIVAL:
            continue
        assert point.site_key is not None
        item: dict[str, Any] = {
            "state_id": point.state_id or point.site_key,
            "type": "PICK" if point.site_key.startswith("P_PICK_") else "DROP",
            "site_key": point.site_key,
        }
        if point.unload_pose_profile_id:
            item["unload_pose_profile_id"] = point.unload_pose_profile_id
        arrivals.append(item)
    return replace(case, arrival_states=tuple(arrivals), leg_refs=())
