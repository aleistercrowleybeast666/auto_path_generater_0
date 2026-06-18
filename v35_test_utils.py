# -*- coding: utf-8 -*-
"""Shared builders for V3.5 unit tests."""
from __future__ import annotations

from path_models import (
    ACTION_MODE_STOP_AND_WAIT,
    EditPoint,
    MechanicalAction,
    PATH_ACT_PICK,
    PathProject,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)


def make_straight_project(length_mm: float = 3000.0) -> PathProject:
    project = PathProject()
    project.planner.max_speed_mmps = 1200
    project.points = [
        EditPoint(
            point_id=0,
            type=POINT_TYPE_START,
            x_mm=0,
            y_mm=0,
            yaw_ddeg=0,
            exact_pass=True,
            corner_trim_mm=0,
        ),
        EditPoint(
            point_id=1,
            type=POINT_TYPE_ARRIVAL,
            x_mm=length_mm,
            y_mm=0,
            yaw_ddeg=0,
        ),
    ]
    return project


def make_curve_project() -> PathProject:
    project = PathProject()
    project.planner.max_speed_mmps = 1200
    project.points = [
        EditPoint(
            point_id=0,
            type=POINT_TYPE_START,
            x_mm=0,
            y_mm=0,
            yaw_ddeg=0,
            exact_pass=True,
            corner_trim_mm=0,
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
        ),
    ]
    return project


def add_stop_action(project: PathProject, arrival_point_id: int = 1) -> None:
    project.actions = [
        MechanicalAction(
            action_seq=0,
            action=PATH_ACT_PICK,
            mode=ACTION_MODE_STOP_AND_WAIT,
            arrival_point_id=arrival_point_id,
            timeout_ms=3000,
        )
    ]
