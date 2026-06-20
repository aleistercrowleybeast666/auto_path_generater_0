"""Geometry sample conversion for Phase 6 leg candidates."""

from __future__ import annotations

from hjmb_pathgen.py_planning.geometry.bezier import BezierPath
from hjmb_pathgen.py_domain.enums import NodeFlag
from hjmb_pathgen.py_planning.dynamics.time_parameterization import GeometrySample
from hjmb_pathgen.py_planning.optimization.yaw_windows import YawWindowProfile


def geometry_samples_from_bezier(
    path: BezierPath,
    yaw_profile: YawWindowProfile,
    *,
    max_spacing_mm: float,
    oversample_per_segment: int = 48,
    arrival_state_id: str = "",
) -> tuple[GeometrySample, ...]:
    xy_samples = path.sample_arclength(max_spacing_mm=max_spacing_mm, oversample_per_segment=oversample_per_segment)
    total_length = xy_samples[-1].s_mm
    result: list[GeometrySample] = []
    for index, sample in enumerate(xy_samples):
        yaw = yaw_profile.evaluate(sample.s_mm, total_length)
        if index == 0:
            flags = int(NodeFlag.START)
            arrival = ""
        elif index == len(xy_samples) - 1:
            flags = int(NodeFlag.ARRIVAL)
            arrival = arrival_state_id
        else:
            flags = 0
            arrival = ""
        result.append(
            GeometrySample(
                s_mm=sample.s_mm,
                x_mm=sample.x_mm,
                y_mm=sample.y_mm,
                yaw_ddeg=yaw.yaw_ddeg,
                tangent_x=sample.tangent_x,
                tangent_y=sample.tangent_y,
                curvature_1_per_mm=sample.curvature_1_per_mm,
                yaw_ddeg_per_mm=yaw.yaw_ddeg_per_mm,
                yaw_ddeg_per_mm2=yaw.yaw_ddeg_per_mm2,
                flags=flags,
                arrival_state_id=arrival,
            )
        )
    return tuple(result)
