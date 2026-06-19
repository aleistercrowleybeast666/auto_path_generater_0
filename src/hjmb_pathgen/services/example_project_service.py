"""Create a reproducible synthetic V4.0 example project for Phase 9 delivery."""

from __future__ import annotations

import hashlib
import math
import shutil
from dataclasses import replace
from pathlib import Path
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_bytes, canonical_json_crc32_hex
from hjmb_pathgen.codec.json_codec import save_leg_library
from hjmb_pathgen.models.enums import LegState, NodeFlag
from hjmb_pathgen.models.leg import LegLibraryV40, LegV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.models.protocol import DIR_BIN, DIR_CASES, DIR_PORTABLE
from hjmb_pathgen.planning.leg_optimizer import PLANNER_ALGORITHM_VERSION
from hjmb_pathgen.services.phase7_generation_service import collect_unique_legs, generate_all
from hjmb_pathgen.services.project_service import ProjectLayout
from hjmb_pathgen.services.traj_table_service import write_route_case_table


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


def create_synthetic_example_project(
    root: str | Path,
    *,
    source_traj_csv: str | Path,
    generate_outputs: bool = False,
) -> dict[str, Any]:
    root = Path(root)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"example project root must be empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    project = _example_project()
    layout = ProjectLayout.create(root, project)
    shutil.copyfile(source_traj_csv, layout.traj_id_csv)
    table_result = write_route_case_table(layout)
    collection = collect_unique_legs(layout)
    library = _synthetic_leg_library(project, collection.to_dict()["requirements"])
    save_leg_library(layout.leg_library_json, library)
    generation = generate_all(layout) if generate_outputs else None
    return {
        "format": "HJMB_PHASE9_SYNTHETIC_EXAMPLE_PROJECT",
        "root": str(layout.root),
        "synthetic": True,
        "warning": "Synthetic geometry is for software reproducibility only; measure and validate real competition sites before use.",
        "route_case_count": len(table_result.route_case_table.cases),
        "unique_leg_count": len(library.legs),
        "generated_outputs": generation is not None,
        "generation": generation.to_dict() if generation is not None else None,
        "directories": [DIR_CASES, DIR_BIN, DIR_PORTABLE],
    }


def _example_project() -> ProjectV40:
    data = _base_project_dict()
    data["project_id"] = "HJMB_SYNTHETIC_EXAMPLE_V40"
    data["action_profiles"] = {
        key: {
            "mode": "STOP_AND_WAIT",
            "timeout_ms": 1000,
            "post_wait_ms": 0,
            "estimated_time_ms": _profile_estimate(key),
        }
        for key in ACTION_PROFILE_KEYS
    }
    data["topology_profiles"] = {
        "PICK_1_TO_3": {"profile_id": "S_LTR_SYNTHETIC", "gates": []},
        "PICK_3_TO_1": {"profile_id": "S_RTL_SYNTHETIC", "gates": []},
    }
    data["planner_profiles"] = {
        "default": {"max_spacing_mm": 25, "max_yaw_step_ddeg": 30},
        "STANDARD": {"max_spacing_mm": 25, "max_yaw_step_ddeg": 30},
        "FINAL": {"max_spacing_mm": 20, "max_yaw_step_ddeg": 20},
    }
    data["output"] = {"case_dir": "cases", "bin_dir": "bin", "synthetic_example": True}
    return ProjectV40.from_dict(data)


def _base_project_dict() -> dict[str, Any]:
    return {
        "format": "HJMB_PATH_PROJECT_JSON_V40",
        "project_id": "minimal_v40",
        "protocol_version": 40,
        "nominal_field": {"length_mm": 4000, "width_mm": 2000},
        "coordinate_system": {
            "origin": "FIELD_CENTER",
            "yaw_zero": "ROBOT_FRONT_TO_PICK_AREA",
            "yaw_positive": "RIGHT_HAND_Z",
        },
        "site_pose_provider": {"type": "MANUAL"},
        "sites": {
            "P_START": {"configured": True, "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
            "P_PICK_1": {"configured": True, "x_mm": 100, "y_mm": 0, "yaw_ddeg": 0},
            "P_PICK_2L": {"configured": True, "x_mm": 200, "y_mm": 0, "yaw_ddeg": 0},
            "P_PICK_2R": {"configured": True, "x_mm": 300, "y_mm": 0, "yaw_ddeg": 0},
            "P_PICK_3": {"configured": True, "x_mm": 400, "y_mm": 0, "yaw_ddeg": 0},
            "F_DROP_4": {"configured": True, "x_mm": 0, "y_mm": 500},
            "F_DROP_5": {"configured": True, "x_mm": 100, "y_mm": 500},
            "F_DROP_6": {"configured": True, "x_mm": 200, "y_mm": 500},
            "F_DROP_7": {"configured": True, "x_mm": 300, "y_mm": 500},
            "F_DROP_8": {"configured": True, "x_mm": 400, "y_mm": 500},
        },
        "field_objects": {
            "cylinders": [
                {"obstacle_id": "CYLINDER_1", "center_x_mm": -1200, "center_y_mm": 350, "radius_mm": 51, "configured": True, "enabled": True},
                {"obstacle_id": "CYLINDER_2", "center_x_mm": -1200, "center_y_mm": -350, "radius_mm": 51, "configured": True, "enabled": True},
            ],
            "pickup_boxes": [
                {"obstacle_id": "PICKUP_BOX_1", "physical_pick_site": "PICK_1", "center_x_mm": 1500, "center_y_mm": 700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "PICKUP_BOX_2", "physical_pick_site": "PICK_2", "center_x_mm": 1650, "center_y_mm": 700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "PICKUP_BOX_3", "physical_pick_site": "PICK_3", "center_x_mm": 1800, "center_y_mm": 700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
            ],
            "drop_boxes": [
                {"obstacle_id": "DROP_BOX_4", "physical_drop_site": "F_DROP_4", "center_x_mm": -1600, "center_y_mm": -700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_5", "physical_drop_site": "F_DROP_5", "center_x_mm": -1450, "center_y_mm": -700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_6", "physical_drop_site": "F_DROP_6", "center_x_mm": -1300, "center_y_mm": -700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_7", "physical_drop_site": "F_DROP_7", "center_x_mm": -1150, "center_y_mm": -700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
                {"obstacle_id": "DROP_BOX_8", "physical_drop_site": "F_DROP_8", "center_x_mm": -1000, "center_y_mm": -700, "length_mm": 140, "width_mm": 120, "yaw_ddeg": 0, "configured": True, "enabled": True},
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
                "r_large_mm": 120,
                "r_small_mm": 70,
                "collision_resolution_mm": 10,
                "strict_validation_resolution_mm": 5,
                "numerical_epsilon_mm": 0.000001,
                "pickup_arc_segments": 64,
                "field_boundary_footprint_profile": "LARGE_CIRCLE",
            },
            "wheel": {"radius_mm": 50, "rotation_radius_mm": 200, "plan_limit_rpm": 300, "hard_limit_rpm": 400},
        },
        "dynamics": {
            "max_speed_mmps": 1000,
            "linear_accel_mmps2": 800,
            "braking_accel_mmps2": 800,
            "lateral_accel_mmps2": 800,
            "max_wz_ddegps": 900,
            "angular_accel_moving_ddegps2": 1000,
            "angular_accel_rotate_ddegps2": 1500,
            "dynamic_margin_ratio": 0.1,
        },
        "unload_profiles": {
            "BIN_1": {"configured": True, "yaw_ddeg": 900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": 1000},
            "BIN_2": {"configured": True, "yaw_ddeg": 0, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": 1000},
            "BIN_3": {"configured": True, "yaw_ddeg": -900, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": 1000},
            "BIN_12": {"configured": True, "yaw_ddeg": 450, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": 1200},
            "BIN_23": {"configured": True, "yaw_ddeg": -450, "dx_mm": 0, "dy_mm": 0, "estimated_action_time_ms": 1200},
        },
        "topology_profiles": {},
        "action_profiles": {},
        "planner_profiles": {},
        "start_check": {"position_tolerance_mm": 20, "yaw_tolerance_ddeg": 50, "stable_time_ms": 100},
        "arrival_check": {"position_tolerance_mm": 20, "yaw_tolerance_ddeg": 50, "speed_tolerance_mmps": 10, "wz_tolerance_ddegps": 10, "stable_time_ms": 100},
        "finish_policy": {"mode": "AT_FINAL_DROP"},
        "output": {"case_dir": "cases", "bin_dir": "bin"},
        "traj_table": {"source_csv": "traj_id.csv", "expected_case_count": 360},
    }


def _profile_estimate(key: str) -> int:
    if key.startswith("DROP_"):
        return 20
    if key in {"PICK", "STORE"}:
        return 15
    return 5


def _synthetic_leg_library(project: ProjectV40, requirements: list[dict[str, Any]]) -> LegLibraryV40:
    legs = tuple(_synthetic_leg(requirement, int(index)) for index, requirement in enumerate(requirements))
    return LegLibraryV40(
        planner_version=PLANNER_ALGORITHM_VERSION,
        project_hash=canonical_json_crc32_hex(project.to_dict()),
        legs=legs,
    )


def _synthetic_leg(requirement: dict[str, Any], index: int) -> LegV40:
    transition = requirement["transition"]
    start = transition["from_pose"]
    end = transition["to_pose"]
    distance = max(1, round(math.hypot(float(end["x_mm"]) - float(start["x_mm"]), float(end["y_mm"]) - float(start["y_mm"]))))
    first_flags = int(NodeFlag.START) if transition["from_state_id"] == "P_START" else int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS)
    first = {
        "local_s_mm": 0,
        "x_mm": round(float(start["x_mm"])),
        "y_mm": round(float(start["y_mm"])),
        "yaw_ddeg": round(float(start["yaw_ddeg"])),
        "vx_mmps": 0,
        "vy_mmps": 0,
        "wz_ddegps": 0,
        "flags": first_flags,
    }
    if transition["from_state_id"] != "P_START":
        first["arrival_state_id"] = transition["from_state_id"]
    last = {
        "local_s_mm": distance,
        "x_mm": round(float(end["x_mm"])),
        "y_mm": round(float(end["y_mm"])),
        "yaw_ddeg": round(float(end["yaw_ddeg"])),
        "vx_mmps": 0,
        "vy_mmps": 0,
        "wz_ddegps": 0,
        "flags": int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS),
        "arrival_state_id": transition["to_state_id"],
    }
    leg = LegV40(
        leg_id=str(requirement["leg_id"]),
        key={
            "from_state_id": transition["from_state_id"],
            "to_state_id": transition["to_state_id"],
            "route_family": transition["route_family"],
            "topology_profile": transition["topology_profile"],
        },
        state=LegState.VALID,
        source="PHASE9_SYNTHETIC_EXAMPLE",
        topology_profile=str(transition["topology_profile"]),
        control_points=(),
        yaw_profile={"model": "SYNTHETIC_STRAIGHT", "index": index},
        nodes=(first, last),
        analysis={"planned_time_ms": max(1, distance), "total_length_mm": distance, "max_metrics": {"max_wheel_rpm": 0.0}},
        hashes={"dependency_hashes": {}, "planner_algorithm_version": PLANNER_ALGORITHM_VERSION},
        review={"approved": True, "locked": False, "state": "VALID", "notes": "synthetic example leg"},
    )
    return replace(leg, hashes=_refreshed_hashes(leg))


def _refreshed_hashes(leg: LegV40) -> dict[str, Any]:
    payload = _leg_validity_payload(leg)
    hashes = dict(leg.hashes)
    hashes["self_hash32"] = f"0x{canonical_json_crc32_hex(payload).upper()}"
    hashes["validity_hash"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    return hashes


def _leg_validity_payload(leg: LegV40) -> dict[str, Any]:
    return {
        "planner_algorithm_version": str(leg.hashes.get("planner_algorithm_version", PLANNER_ALGORITHM_VERSION)),
        "key": dict(leg.key),
        "control_points": list(leg.control_points),
        "yaw_profile": dict(leg.yaw_profile),
        "nodes": list(leg.nodes),
        "analysis_semantic": {
            "planned_time_ms": leg.analysis.get("planned_time_ms", 0),
            "total_length_mm": leg.analysis.get("total_length_mm", 0),
            "max_metrics": dict(leg.analysis.get("max_metrics", {})),
            "min_clearance_mm": leg.analysis.get("min_clearance_mm"),
        },
    }
