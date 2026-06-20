"""Create a usable V4 workspace from the V3.5-style editor configuration.

This module deliberately contains no Qt imports.  The GUI passes plain dictionaries,
so project creation can also be tested and used by CLI tools.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import REQUIRED_SITE_KEYS, YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table
from hjmb_pathgen.py_services.competition_task_config_service import ensure_competition_task_config

ACTION_PROFILE_KEYS = (
    "PREP_PICK_1",
    "PREP_PICK_2L",
    "PREP_PICK_2R",
    "PREP_PICK_3",
    "PICK",
    "PREP_STORE_1",
    "PREP_STORE_2",
    "PREP_STORE_3",
    "STORE",
    "DROP_1",
    "DROP_2",
    "DROP_3",
    "DROP_12",
    "DROP_23",
)


@dataclass(frozen=True)
class BootstrapResult:
    layout: ProjectLayout
    created_project: bool
    created_route_table: bool
    warnings: tuple[str, ...]


def bootstrap_v4_workspace(
    root: str | Path,
    *,
    project_id: str,
    common_sites: Mapping[str, Mapping[str, Any]],
    vehicle: Mapping[str, Any],
    dynamics: Mapping[str, Any],
    start_check: Mapping[str, Any],
    arrival_check: Mapping[str, Any],
    action_durations_ms: Mapping[str, int],
    source_traj_csv: str | Path | None = None,
) -> BootstrapResult:
    """Create ``project.json`` and the V4 directory tree when they are absent.

    Existing project.json is never overwritten.  The English-only deterministic
    task_config/competition_task_config.json is created when absent and is the
    primary source for route_case_table.json.  ``source_traj_csv`` is accepted only
    for API compatibility and is not required by the normal workflow.
    """

    root_path = Path(root).resolve(strict=False)
    root_path.mkdir(parents=True, exist_ok=True)
    layout = ProjectLayout.open(root_path, create_dirs=True)
    warnings: list[str] = []
    created_project = False

    if not layout.project_json.exists():
        project = build_default_v4_project(
            project_id=project_id,
            common_sites=common_sites,
            vehicle=vehicle,
            dynamics=dynamics,
            start_check=start_check,
            arrival_check=arrival_check,
            action_durations_ms=action_durations_ms,
        )
        layout = ProjectLayout.create(root_path, project)
        created_project = True
        warnings.extend(
            (
                "已按当前GUI配置创建project.json。",
                "碰撞包络、11项倒货姿态和场地物体尺寸使用初始值，正式比赛前必须核对。",
            )
        )
    else:
        layout.ensure_directories()

    # The normal V4 workflow is independent of the legacy Chinese CSV.
    ensure_competition_task_config(layout.competition_task_config_json)

    created_route_table = not layout.route_case_table_json.exists()
    # Always rebuild the small 360-row mapping from the English task JSON.
    # This removes hidden dependence on an old CSV-generated table and makes
    # task-config edits take effect on the next project load.
    write_route_case_table(layout)

    return BootstrapResult(
        layout=layout,
        created_project=created_project,
        created_route_table=created_route_table,
        warnings=tuple(warnings),
    )


def build_default_v4_project(
    *,
    project_id: str,
    common_sites: Mapping[str, Mapping[str, Any]],
    vehicle: Mapping[str, Any],
    dynamics: Mapping[str, Any],
    start_check: Mapping[str, Any],
    arrival_check: Mapping[str, Any],
    action_durations_ms: Mapping[str, int],
) -> ProjectV40:
    """Build a complete, writable V4 project template.

    The field object geometry mirrors the proven V3.5 editor drawing.  It is an
    editable starting configuration, not a claim that the real competition field has
    already been measured.
    """

    sites: dict[str, dict[str, Any]] = {}
    for key in REQUIRED_SITE_KEYS:
        value = common_sites.get(key)
        if value is None:
            sites[key] = {
                "configured": False,
                "x_mm": 0,
                "y_mm": 0,
                "yaw_ddeg": YAW_UNSPECIFIED_DDEG if key.startswith("P_DROP_") else 0,
            }
            continue
        sites[key] = {
            "configured": bool(value.get("configured", True)),
            "x_mm": int(round(float(value.get("x_mm", 0)))),
            "y_mm": int(round(float(value.get("y_mm", 0)))),
            # Keep 0xFFFF exactly: it means the arrival yaw is unconstrained.
            "yaw_ddeg": int(value.get("yaw_ddeg", 0)),
        }

    action_profiles: dict[str, dict[str, Any]] = {}
    for key in ACTION_PROFILE_KEYS:
        mode = "STOP_AND_WAIT"
        if key.startswith("PREP_"):
            mode = "ASYNC"
        duration = max(0, int(action_durations_ms.get(key, 500)))
        action_profiles[key] = {
            "mode": mode,
            "timeout_ms": max(1000, duration + 2000),
            "post_wait_ms": 0,
            "estimated_time_ms": duration,
        }

    wheel = dict(vehicle.get("wheel", {}))
    footprint = dict(vehicle.get("footprint", {}))
    data: dict[str, Any] = {
        "format": "HJMB_PATH_PROJECT_JSON_V40",
        "project_id": project_id or "HJMB_V40_PROJECT",
        "protocol_version": 40,
        "nominal_field": {"length_mm": 4000, "width_mm": 2000},
        "coordinate_system": {
            "origin": "FIELD_CENTER",
            "yaw_zero": "ROBOT_FRONT_TO_PICK_AREA",
            "yaw_positive": "RIGHT_HAND_Z",
        },
        "site_pose_provider": {"type": "MANUAL"},
        "sites": sites,
        "field_objects": {
            "cylinders": [
                {"obstacle_id": "CYLINDER_1", "center_x_mm": 1000, "center_y_mm": 0, "radius_mm": 51, "configured": True, "enabled": True},
                {"obstacle_id": "CYLINDER_2", "center_x_mm": -1000, "center_y_mm": 0, "radius_mm": 51, "configured": True, "enabled": True},
            ],
            "pickup_boxes": [
                {"obstacle_id": "PICKUP_BOX_1", "physical_pick_site": "PICK_1", "center_x_mm": 1800, "center_y_mm": 500, "length_mm": 210, "width_mm": 300, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "PICKUP_BOX_2", "physical_pick_site": "PICK_2", "center_x_mm": 1500, "center_y_mm": 0, "length_mm": 210, "width_mm": 300, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "PICKUP_BOX_3", "physical_pick_site": "PICK_3", "center_x_mm": 1800, "center_y_mm": -500, "length_mm": 210, "width_mm": 300, "yaw_ddeg": 0, "configured": True, "enabled": True},
            ],
            "drop_boxes": [
                {"obstacle_id": "DROP_BOX_4", "physical_drop_site": "F_DROP_4", "center_x_mm": -1500, "center_y_mm": 800, "length_mm": 280, "width_mm": 200, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_5", "physical_drop_site": "F_DROP_5", "center_x_mm": -1700, "center_y_mm": 400, "length_mm": 200, "width_mm": 280, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_6", "physical_drop_site": "F_DROP_6", "center_x_mm": -1700, "center_y_mm": 0, "length_mm": 200, "width_mm": 280, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_7", "physical_drop_site": "F_DROP_7", "center_x_mm": -1700, "center_y_mm": -400, "length_mm": 200, "width_mm": 280, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_8", "physical_drop_site": "F_DROP_8", "center_x_mm": -1500, "center_y_mm": -800, "length_mm": 280, "width_mm": 200, "yaw_ddeg": 0, "configured": True, "enabled": True},
            ],
            "field_boundary": {
                "enabled": True,
                "x_min_mm": -2000,
                "x_max_mm": 2000,
                "y_min_mm": -1000,
                "y_max_mm": 1000,
                "footprint_profile": "LARGE_CIRCLE",
            },
        },
        "vehicle": {
            "footprint": {
                "r_large_mm": float(footprint.get("r_large_mm", 120.0)),
                "r_small_mm": float(footprint.get("r_small_mm", 70.0)),
                "collision_resolution_mm": float(footprint.get("collision_resolution_mm", 10.0)),
                "strict_validation_resolution_mm": float(footprint.get("strict_validation_resolution_mm", 5.0)),
                "numerical_epsilon_mm": float(footprint.get("numerical_epsilon_mm", 0.000001)),
                "pickup_arc_segments": int(footprint.get("pickup_arc_segments", 64)),
                "field_boundary_footprint_profile": "LARGE_CIRCLE",
            },
            "wheel": {
                "radius_mm": float(wheel.get("radius_mm", 76.0)),
                "rotation_radius_mm": float(wheel.get("rotation_radius_mm", 260.0)),
                "plan_limit_rpm": int(wheel.get("plan_limit_rpm", 420)),
                "hard_limit_rpm": int(wheel.get("hard_limit_rpm", 450)),
            },
        },
        "dynamics": dict(dynamics),
        "unload_profiles": {
            "BIN_1": {"configured": True, "yaw_ddeg": 900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_1", 700))},
            "BIN_2": {"configured": True, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_2", 700))},
            "BIN_3": {"configured": True, "yaw_ddeg": -900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_3", 700))},
            "BIN_12": {"configured": False, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_12", 700))},
            "BIN_23": {"configured": False, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_23", 700))},
        },
        "unload_pose_profiles": {
            "DROP_F4_BIN_1": {"configured": True, "yaw_ddeg": 900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_1", 700))},
            "DROP_F5_BIN_1": {"configured": True, "yaw_ddeg": 900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_1", 700))},
            "DROP_F5_BIN_2": {"configured": True, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_2", 700))},
            "DROP_F6_BIN_1": {"configured": True, "yaw_ddeg": 900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_1", 700))},
            "DROP_F6_BIN_2": {"configured": True, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_2", 700))},
            "DROP_F6_BIN_3": {"configured": True, "yaw_ddeg": -900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_3", 700))},
            "DROP_F7_BIN_2": {"configured": True, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_2", 700))},
            "DROP_F7_BIN_3": {"configured": True, "yaw_ddeg": -900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_3", 700))},
            "DROP_F8_BIN_3": {"configured": True, "yaw_ddeg": -900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_3", 700))},
            "DROP_F45_BIN_12": {"configured": False, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_12", 700))},
            "DROP_F78_BIN_23": {"configured": False, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": int(action_durations_ms.get("DROP_23", 700))},
        },
        "topology_profiles": {
            "PICK_1_TO_3": {
                "profile_id": "S_LEFT_DEFAULT",
                "transfer_profile_id": "S_LEFT_TRANSFER",
                "auto_generate_transfer_gates": True,
                "gate_clearance_mm": 65,
                "gates": [],
            },
            "PICK_3_TO_1": {
                "profile_id": "S_RIGHT_DEFAULT",
                "transfer_profile_id": "S_RIGHT_TRANSFER",
                "auto_generate_transfer_gates": True,
                "gate_clearance_mm": 65,
                "gates": [],
            },
        },
        "action_profiles": action_profiles,
        "planner_profiles": {
            "default": {"max_spacing_mm": 25, "max_yaw_step_ddeg": 30, "use_unload_pose_profiles": True},
            "QUICK": {"max_spacing_mm": 35, "max_yaw_step_ddeg": 40},
            "STANDARD": {"max_spacing_mm": 25, "max_yaw_step_ddeg": 30},
            "FINAL": {"max_spacing_mm": 20, "max_yaw_step_ddeg": 20},
        },
        "start_check": dict(start_check),
        "arrival_check": dict(arrival_check),
        "finish_policy": {"mode": "AT_FINAL_DROP"},
        "output": {"case_dir": "cases", "bin_dir": "bin"},
        "traj_table": {
            "source_type": "TASK_CONFIG_JSON",
            "source_path": "task_config/competition_task_config.json",
            "expected_case_count": 360,
        },
    }
    return ProjectV40.from_dict(data)
