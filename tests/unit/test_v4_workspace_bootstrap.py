from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_io.codecs.json_codec import load_leg_library, load_project, load_route_case_table
from hjmb_pathgen.py_services.case_draft_service import generate_case_draft
from hjmb_pathgen.py_services.mode_case_service import convert_full_auto_to_semi_auto
from hjmb_pathgen.py_services.project_bootstrap_service import bootstrap_v4_workspace


class V4WorkspaceBootstrapTests(unittest.TestCase):
    def _kwargs(self) -> dict:
        keys = ("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3", "P_DROP_1", "P_DROP_2", "P_DROP_3")
        common_sites = {
            key: {
                "configured": True,
                "x_mm": index * 100,
                "y_mm": 0,
                "yaw_ddeg": YAW_UNSPECIFIED_DDEG if key.startswith("P_DROP_") else 0,
            }
            for index, key in enumerate(keys)
        }
        action_names = (
            "PREP_PICK_1", "PREP_PICK_2L", "PREP_PICK_2R", "PREP_PICK_3",
            "PICK", "PREP_STORE_1", "PREP_STORE_2", "PREP_STORE_3",
            "STORE", "DROP_1", "DROP_2", "DROP_3", "DROP_12", "DROP_23",
        )
        return {
            "project_id": "bootstrap_test",
            "common_sites": common_sites,
            "vehicle": {
                "wheel": {
                    "radius_mm": 76,
                    "rotation_radius_mm": 260,
                    "plan_limit_rpm": 420,
                    "hard_limit_rpm": 450,
                },
                "footprint": {},
            },
            "dynamics": {
                "max_speed_mmps": 2000,
                "linear_accel_mmps2": 1200,
                "braking_accel_mmps2": 1200,
                "lateral_accel_mmps2": 1200,
                "max_wz_ddegps": 2292,
                "angular_accel_moving_ddegps2": 1146,
                "angular_accel_rotate_ddegps2": 2865,
                "dynamic_margin_ratio": 0.1,
            },
            "start_check": {
                "position_tolerance_mm": 30,
                "yaw_tolerance_ddeg": 50,
                "stable_time_ms": 100,
            },
            "arrival_check": {
                "position_tolerance_mm": 20,
                "yaw_tolerance_ddeg": 30,
                "speed_tolerance_mmps": 60,
                "wz_tolerance_ddegps": 50,
                "stable_time_ms": 100,
            },
            "action_durations_ms": {name: 500 for name in action_names},
        }

    def test_creates_project_tree_and_mapping(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            result = bootstrap_v4_workspace(
                directory,
                **self._kwargs(),
                source_traj_csv=repository_root / "traj_id.csv",
            )
            self.assertTrue(result.created_project)
            self.assertTrue(result.created_route_table)
            self.assertEqual(len(load_project(result.layout.project_json).sites), 8)
            self.assertEqual(len(load_route_case_table(result.layout.route_case_table_json).cases), 360)
            self.assertEqual(len(load_leg_library(result.layout.leg_library_json).legs), 0)
            self.assertTrue(result.layout.semi_auto_cases_dir.is_dir())
            self.assertTrue(result.layout.full_auto_bin_dir.is_dir())

    def test_existing_project_is_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            first = bootstrap_v4_workspace(directory, **self._kwargs())
            original = first.layout.project_json.read_bytes()
            changed = self._kwargs()
            changed["project_id"] = "must_not_replace"
            second = bootstrap_v4_workspace(directory, **changed)
            self.assertFalse(second.created_project)
            self.assertEqual(first.layout.project_json.read_bytes(), original)

    def test_bootstrap_field_objects_use_current_field_centers(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = bootstrap_v4_workspace(directory, **self._kwargs())
            field_objects = load_project(result.layout.project_json).field_objects

        drop_boxes = {item["obstacle_id"]: item for item in field_objects["drop_boxes"]}
        self.assertEqual((drop_boxes["DROP_BOX_4"]["center_x_mm"], drop_boxes["DROP_BOX_4"]["center_y_mm"]), (-1640, 875))
        self.assertEqual((drop_boxes["DROP_BOX_5"]["center_x_mm"], drop_boxes["DROP_BOX_5"]["center_y_mm"]), (-1875, 400))
        self.assertEqual((drop_boxes["DROP_BOX_6"]["center_x_mm"], drop_boxes["DROP_BOX_6"]["center_y_mm"]), (-1875, 0))
        self.assertEqual((drop_boxes["DROP_BOX_7"]["center_x_mm"], drop_boxes["DROP_BOX_7"]["center_y_mm"]), (-1875, -400))
        self.assertEqual((drop_boxes["DROP_BOX_8"]["center_x_mm"], drop_boxes["DROP_BOX_8"]["center_y_mm"]), (-1640, -875))
        self.assertEqual((drop_boxes["DROP_BOX_4"]["length_mm"], drop_boxes["DROP_BOX_4"]["width_mm"]), (280, 200))
        self.assertEqual((drop_boxes["DROP_BOX_8"]["length_mm"], drop_boxes["DROP_BOX_8"]["width_mm"]), (280, 200))
        for key in ("DROP_BOX_5", "DROP_BOX_6", "DROP_BOX_7"):
            self.assertEqual((drop_boxes[key]["length_mm"], drop_boxes[key]["width_mm"]), (200, 280))
            self.assertEqual(drop_boxes[key]["yaw_ddeg"], 0)

        pickup_boxes = {item["obstacle_id"]: item for item in field_objects["pickup_boxes"]}
        self.assertEqual((pickup_boxes["PICKUP_BOX_1"]["center_x_mm"], pickup_boxes["PICKUP_BOX_1"]["center_y_mm"]), (1855, 500))
        self.assertEqual((pickup_boxes["PICKUP_BOX_2"]["center_x_mm"], pickup_boxes["PICKUP_BOX_2"]["center_y_mm"]), (1600, 0))
        self.assertEqual((pickup_boxes["PICKUP_BOX_3"]["center_x_mm"], pickup_boxes["PICKUP_BOX_3"]["center_y_mm"]), (1855, -500))
        for item in pickup_boxes.values():
            self.assertEqual((item["length_mm"], item["width_mm"], item["yaw_ddeg"]), (210, 300, 0))

        cylinders = {item["obstacle_id"]: item for item in field_objects["cylinders"]}
        self.assertEqual((cylinders["CYLINDER_1"]["center_x_mm"], cylinders["CYLINDER_1"]["center_y_mm"], cylinders["CYLINDER_1"]["radius_mm"]), (1000, 0, 51))
        self.assertEqual((cylinders["CYLINDER_2"]["center_x_mm"], cylinders["CYLINDER_2"]["center_y_mm"], cylinders["CYLINDER_2"]["radius_mm"]), (-1000, 0, 51))
        self.assertEqual(
            field_objects["field_boundary"],
            {
                "enabled": True,
                "x_min_mm": -2000,
                "x_max_mm": 2000,
                "y_min_mm": -1000,
                "y_max_mm": 1000,
                "footprint_profile": "LARGE_CIRCLE",
            },
        )

    def test_bootstrap_can_supply_semi_auto_task_template(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            result = bootstrap_v4_workspace(
                directory,
                **self._kwargs(),
                source_traj_csv=repository_root / "traj_id.csv",
            )
            draft = generate_case_draft(result.layout, 4)
            semi = convert_full_auto_to_semi_auto(result.layout, 4)
            self.assertEqual(len(draft.case.logical_points), 8)
            self.assertEqual(semi.generation_mode.value, "SEMI_AUTO")
            self.assertEqual(len(semi.logical_points), 0)
            self.assertEqual(len(semi.semi_path["points"]), 7)


if __name__ == "__main__":
    unittest.main()
