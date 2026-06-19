# -*- coding: utf-8 -*-
"""Pure-Python V3.5 batch project assembly helpers."""
from __future__ import annotations

from copy import deepcopy
from typing import Dict, Tuple

from .batch_models import LegTemplate, RouteCase
from .path_models import (
    EditPoint,
    MechanicalAction,
    PATH_MODE_FIXED_8,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    PathProject,
    SITE_ID_FREE,
    YAW_UNSPECIFIED_DDEG,
)


def assemble_route_case(
    common_project: PathProject,
    route_case: RouteCase,
    leg_templates: Dict[Tuple[int, int], LegTemplate],
    allow_direct_missing_leg: bool = False,
) -> PathProject:
    """Build one independent V3.5 project from a centralized route case."""
    project = deepcopy(common_project)
    project.traj_id = route_case.traj_id
    project.path_mode = PATH_MODE_FIXED_8
    project.planner.yaw_rotation_policy = route_case.yaw_rotation_policy
    project.route_meta = {
        "pickup_order": list(route_case.pickup_order),
        "drop_order": list(route_case.drop_order),
        "sweep_direction": route_case.sweep_direction,
        "action_template_name": route_case.action_template_name,
    }
    project.points = [
        EditPoint(
            point_id=0,
            type=POINT_TYPE_START,
            site_id=0,
            yaw_ddeg=0,
            exact_pass=True,
            corner_trim_mm=0,
        )
    ]
    current_site = 0
    for next_site in route_case.ordered_sites:
        template = leg_templates.get((current_site, next_site))
        if template is None and not allow_direct_missing_leg:
            raise ValueError(f"缺少有向路段模板 {current_site}->{next_site}")
        if template is not None:
            for waypoint_data in template.waypoints:
                project.points.append(
                    EditPoint(
                        point_id=len(project.points),
                        type=POINT_TYPE_WAYPOINT,
                        site_id=SITE_ID_FREE,
                        x_mm=float(waypoint_data.get("x_mm", 0)),
                        y_mm=float(waypoint_data.get("y_mm", 0)),
                        yaw_ddeg=YAW_UNSPECIFIED_DDEG,
                        max_speed_mmps=int(waypoint_data.get("max_speed_mmps", 0)),
                        corner_trim_mm=float(waypoint_data.get("corner_trim_mm", 200)),
                        exact_pass=bool(waypoint_data.get("exact_pass", False)),
                    )
                )
        project.points.append(
            EditPoint(
                point_id=len(project.points),
                type=POINT_TYPE_ARRIVAL,
                site_id=next_site,
                yaw_ddeg=0,
            )
        )
        current_site = next_site
    project.actions = [deepcopy(action) for action in common_project.actions]
    for index, point in enumerate(project.points):
        point.point_id = index
    for index, action in enumerate(project.actions):
        if not isinstance(action, MechanicalAction):
            continue
        action.action_seq = index
    return project
