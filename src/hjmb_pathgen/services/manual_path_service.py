"""Manual free-path planning services for Phase 4."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32
from hjmb_pathgen.models.compiled import CompiledTrajectoryV40, HeaderV40, SegmentV40
from hjmb_pathgen.models.enums import HeaderFlag, ManualPathPointType, NodeFlag, PathSource, RouteFamily, SegmentFlag
from hjmb_pathgen.models.errors import CompileError
from hjmb_pathgen.models.manual_path import ManualPathV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.models.route_case import CaseManifestV40
from hjmb_pathgen.models.protocol import REQUIRED_HEADER_FLAGS
from hjmb_pathgen.planning.time_parameterization import (
    GeometrySample,
    TimeParameterizationLimits,
    TimeParameterizationRequest,
    TimeParameterizationResult,
    time_parameterize,
)


@dataclass(frozen=True)
class ManualCasePlanResult:
    case: CaseManifestV40
    trajectory: CompiledTrajectoryV40 | None
    timing: TimeParameterizationResult

    def to_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.case.traj_id,
            "success": self.timing.success,
            "timing": self.timing.to_dict(),
            "node_count": len(self.trajectory.nodes) if self.trajectory else 0,
        }


def build_manual_spatial_path(manual_path: dict[str, Any] | ManualPathV40) -> tuple[GeometrySample, ...]:
    path = manual_path if isinstance(manual_path, ManualPathV40) else ManualPathV40.from_dict(manual_path)
    points = path.points
    cumulative = [0.0]
    for left, right in zip(points, points[1:]):
        distance = math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm)
        if distance <= 0.0:
            raise CompileError("manual_path contains a zero-length interval")
        cumulative.append(cumulative[-1] + distance)
    yaws = _interpolated_point_yaws(path, cumulative)
    samples: list[GeometrySample] = []
    for index, point in enumerate(points):
        if index < len(points) - 1:
            dx = points[index + 1].x_mm - point.x_mm
            dy = points[index + 1].y_mm - point.y_mm
            ds = cumulative[index + 1] - cumulative[index]
            yaw_rate = (yaws[index + 1] - yaws[index]) / ds
        else:
            dx = point.x_mm - points[index - 1].x_mm
            dy = point.y_mm - points[index - 1].y_mm
            ds = cumulative[index] - cumulative[index - 1]
            yaw_rate = (yaws[index] - yaws[index - 1]) / ds
        distance = max(math.hypot(dx, dy), 1.0e-9)
        flags = 0
        if point.point_type == ManualPathPointType.START:
            flags |= int(NodeFlag.START)
        if point.point_type == ManualPathPointType.ARRIVAL:
            flags |= int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS)
        if point.exact_pass:
            flags |= int(NodeFlag.EXACT_PASS)
        samples.append(
            GeometrySample(
                s_mm=cumulative[index],
                x_mm=point.x_mm,
                y_mm=point.y_mm,
                yaw_ddeg=yaws[index],
                tangent_x=dx / distance,
                tangent_y=dy / distance,
                yaw_ddeg_per_mm=yaw_rate,
                flags=flags,
                arrival_state_id=f"MANUAL_ARRIVAL_{index}" if point.point_type == ManualPathPointType.ARRIVAL else "",
                max_speed_mmps=float(point.max_speed_mmps) if point.max_speed_mmps is not None else None,
            )
        )
    return tuple(samples)


def retime_case(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    profile_name: str = "default",
) -> TimeParameterizationResult:
    if case.path_source != PathSource.MANUAL_FREE:
        raise CompileError("retime_case currently supports MANUAL_FREE cases only in Phase 4")
    if case.manual_path is None:
        raise CompileError("MANUAL_FREE case has no manual_path")
    samples = build_manual_spatial_path(case.manual_path)
    limits = TimeParameterizationLimits.from_project(project, profile_name=profile_name)
    return time_parameterize(TimeParameterizationRequest(samples=samples, limits=limits))


def plan_manual_case(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    profile_name: str = "default",
) -> ManualCasePlanResult:
    timing = retime_case(case, project, profile_name=profile_name)
    trajectory = _trajectory_from_timing(case, project, timing) if timing.success else None
    return ManualCasePlanResult(case=case, trajectory=trajectory, timing=timing)


def _trajectory_from_timing(case: CaseManifestV40, project: ProjectV40, timing: TimeParameterizationResult) -> CompiledTrajectoryV40:
    if not timing.nodes:
        raise CompileError("successful timing result did not produce nodes")
    nodes = list(timing.nodes)
    nodes[-1] = replace(nodes[-1], flags=(nodes[-1].flags & ~int(NodeFlag.SAFE_END)) | int(NodeFlag.FINISH_ARM | NodeFlag.EXACT_PASS))
    segment = SegmentV40(
        segment_id=0,
        start_node_index=0,
        end_node_index=len(nodes) - 1,
        start_s_mm=nodes[0].s_mm,
        end_s_mm=nodes[-1].s_mm,
        start_arrival_id=nodes[0].arrival_id,
        end_arrival_id=nodes[-1].arrival_id,
        flags=int(SegmentFlag.NORMAL | SegmentFlag.MANUAL_OVERRIDE),
        planned_time_ms=timing.planned_time_ms,
        source_leg_hash32=0,
    )
    header = HeaderV40(
        traj_id=case.traj_id,
        bean_code=case.bean_code,
        drop_code=case.drop_code,
        route_family=int(RouteFamily.MANUAL_FREE),
        flags=int(REQUIRED_HEADER_FLAGS | HeaderFlag.MANUAL_OVERRIDE),
        planned_motion_time_ms=timing.planned_time_ms,
        planned_total_estimate_ms=timing.planned_time_ms,
        source_case_hash32=canonical_json_crc32(case.to_dict()),
        source_project_hash32=canonical_json_crc32(project.to_dict()),
    )
    trajectory = CompiledTrajectoryV40(header=header, nodes=tuple(nodes), segments=(segment,), actions=()).normalized()
    trajectory.validate()
    return trajectory


def _interpolated_point_yaws(path: ManualPathV40, cumulative: list[float]) -> list[float]:
    points = path.points
    known = [index for index, point in enumerate(points) if point.yaw_ddeg is not None]
    yaws = [0.0 for _ in points]
    for left_index, right_index in zip(known, known[1:]):
        left_yaw = float(points[left_index].yaw_ddeg)
        right_yaw = float(points[right_index].yaw_ddeg)
        left_s = cumulative[left_index]
        right_s = cumulative[right_index]
        span = max(right_s - left_s, 1.0e-9)
        for index in range(left_index, right_index + 1):
            ratio = (cumulative[index] - left_s) / span
            yaws[index] = left_yaw + (right_yaw - left_yaw) * ratio
    first = known[0]
    for index in range(0, first):
        yaws[index] = float(points[first].yaw_ddeg)
    last = known[-1]
    for index in range(last, len(points)):
        yaws[index] = float(points[last].yaw_ddeg)
    return yaws
