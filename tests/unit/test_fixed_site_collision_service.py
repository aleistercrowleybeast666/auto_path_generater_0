from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_services.fixed_site_collision_service import (
    FixedSiteCollisionResult,
    check_fixed_site_collisions,
)

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"

SAFE_SITE_POSES = {
    "P_START": (0, 0, 0),
    "P_PICK_1": (250, 0, 0),
    "P_PICK_2L": (500, 0, 0),
    "P_PICK_2R": (750, 0, 0),
    "P_PICK_3": (1000, 0, 0),
    "P_DROP_1": (0, 300, 0),
    "P_DROP_2": (300, 300, 0),
    "P_DROP_3": (600, 300, 0),
}


def project_dict() -> dict:
    return json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))


def safe_project_dict() -> dict:
    data = project_dict()
    for site_key, (x_mm, y_mm, yaw_ddeg) in SAFE_SITE_POSES.items():
        data["sites"][site_key] = {
            "configured": True,
            "x_mm": x_mm,
            "y_mm": y_mm,
            "yaw_ddeg": yaw_ddeg,
        }
    return data


def project_from_data(data: dict) -> ProjectV40:
    return ProjectV40.from_dict(copy.deepcopy(data))


class FixedSiteCollisionServiceTest(unittest.TestCase):
    def test_all_eight_fixed_sites_safe_returns_passed(self) -> None:
        report = check_fixed_site_collisions(project_from_data(safe_project_dict()))

        self.assertEqual(report.result, FixedSiteCollisionResult.PASSED)
        self.assertEqual(report.passed_count, 8)
        self.assertEqual(report.collision_count, 0)
        self.assertEqual(report.incomplete_count, 0)

    def test_penetrating_fixed_site_returns_failed_with_obstacle_detail(self) -> None:
        data = safe_project_dict()
        data["sites"]["P_START"]["x_mm"] = -1200
        data["sites"]["P_START"]["y_mm"] = 350

        report = check_fixed_site_collisions(project_from_data(data))

        self.assertEqual(report.result, FixedSiteCollisionResult.FAILED)
        start_entry = next(entry for entry in report.entries if entry.site_key == "P_START")
        self.assertFalse(start_entry.passed)
        self.assertTrue(start_entry.collisions)
        self.assertEqual(start_entry.collisions[0].obstacle_id, "CYLINDER_1")
        self.assertEqual(start_entry.collisions[0].obstacle_type, "CYLINDER")
        self.assertGreater(start_entry.collisions[0].penetration_depth_mm, 0.0)

    def test_drop_site_uses_configured_profile_offset_and_yaw(self) -> None:
        data = safe_project_dict()
        data["planner_profiles"] = {"default": {"use_unload_pose_profiles": True}}
        data["sites"]["P_DROP_1"]["yaw_ddeg"] = YAW_UNSPECIFIED_DDEG
        data["unload_pose_profiles"] = {
            profile_id: {
                "configured": False,
                "yaw_ddeg": 0,
                "dx_mm": 0,
                "dy_mm": 0,
                "estimated_action_time_ms": 700,
            }
            for profile_id in (
                "DROP_F4_BIN_1",
                "DROP_F5_BIN_1",
                "DROP_F5_BIN_2",
                "DROP_F6_BIN_1",
                "DROP_F6_BIN_2",
                "DROP_F6_BIN_3",
                "DROP_F7_BIN_2",
                "DROP_F7_BIN_3",
                "DROP_F8_BIN_3",
                "DROP_F45_BIN_12",
                "DROP_F78_BIN_23",
            )
        }
        data["unload_pose_profiles"]["DROP_F7_BIN_2"].update(
            {"configured": True, "yaw_ddeg": 321, "dx_mm": 123, "dy_mm": -45}
        )

        report = check_fixed_site_collisions(project_from_data(data))

        self.assertEqual(report.result, FixedSiteCollisionResult.PASSED)
        profile_entry = next(
            entry
            for entry in report.entries
            if entry.site_key == "P_DROP_1" and entry.profile_id == "DROP_F7_BIN_2"
        )
        self.assertEqual(profile_entry.x_mm, 123.0)
        self.assertEqual(profile_entry.y_mm, 255.0)
        self.assertEqual(profile_entry.yaw_ddeg, 321.0)

    def test_unspecified_yaw_without_actual_pose_is_incomplete_not_zero_yaw(self) -> None:
        data = safe_project_dict()
        data["sites"]["P_PICK_1"].update(
            {"x_mm": -1200, "y_mm": 350, "yaw_ddeg": YAW_UNSPECIFIED_DDEG}
        )

        report = check_fixed_site_collisions(project_from_data(data))

        self.assertEqual(report.result, FixedSiteCollisionResult.INCOMPLETE)
        self.assertEqual(report.collision_count, 0)
        pick_entry = next(entry for entry in report.entries if entry.site_key == "P_PICK_1")
        self.assertFalse(pick_entry.checked)
        self.assertEqual(pick_entry.incomplete_reason, "未指定 yaw")


if __name__ == "__main__":
    unittest.main()
