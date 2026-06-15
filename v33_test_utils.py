# -*- coding: utf-8 -*-
"""Shared builders for V3.3 unit tests."""
from __future__ import annotations

from path_models import (
    EditPoint,
    PathProject,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_CUT_IN,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)


def make_straight_project(length_mm: float = 3000.0) -> PathProject:
    project = PathProject()
    project.cut_in.target_speed_mmps = 400
    project.cut_in.approach_max_speed_mmps = 800
    project.cut_in.straight_length_mm = 300
    project.preview_initial_pose.enabled = False
    project.points = [
        EditPoint(
            point_id=0,
            type=POINT_TYPE_CUT_IN,
            x_mm=0,
            y_mm=0,
            yaw_ddeg=0,
            exact_pass=True,
        ),
        EditPoint(
            point_id=1,
            type=POINT_TYPE_ARRIVAL,
            x_mm=length_mm,
            y_mm=0,
            yaw_ddeg=0,
            stop_required=True,
            is_end=True,
        ),
    ]
    return project


def make_curve_project() -> PathProject:
    project = PathProject()
    project.cut_in.target_speed_mmps = 350
    project.cut_in.approach_max_speed_mmps = 700
    project.cut_in.straight_length_mm = 250
    project.points = [
        EditPoint(
            point_id=0,
            type=POINT_TYPE_CUT_IN,
            x_mm=0,
            y_mm=0,
            yaw_ddeg=0,
            exact_pass=True,
        ),
        EditPoint(
            point_id=1,
            type=POINT_TYPE_WAYPOINT,
            x_mm=700,
            y_mm=0,
            yaw_ddeg=YAW_UNSPECIFIED_DDEG,
            corner_trim_mm=180,
        ),
        EditPoint(
            point_id=2,
            type=POINT_TYPE_WAYPOINT,
            x_mm=1200,
            y_mm=500,
            yaw_ddeg=YAW_UNSPECIFIED_DDEG,
            corner_trim_mm=180,
        ),
        EditPoint(
            point_id=3,
            type=POINT_TYPE_ARRIVAL,
            x_mm=2000,
            y_mm=500,
            yaw_ddeg=900,
            stop_required=True,
            is_end=True,
        ),
    ]
    return project
