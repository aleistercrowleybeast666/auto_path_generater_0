from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_services.project_config_service import compute_project_functional_hashes, validate_project_site_configuration

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def project_dict() -> dict:
    return json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))


class Phase4ProjectConfigTest(unittest.TestCase):
    def test_minimal_project_has_manual_planning_readiness(self):
        project = ProjectV40.from_dict(project_dict())
        report = validate_project_site_configuration(project)
        self.assertTrue(report.ready_for_route_table)
        self.assertTrue(report.ready_for_semantic_candidates)
        self.assertTrue(report.ready_for_manual_planning)
        self.assertFalse(report.ready_for_full_360_planning)
        self.assertEqual(report.missing_sites, ())
        self.assertEqual(report.missing_unload_profiles, ())
        self.assertIn("site_config_hash", report.functional_hashes)

    def test_sites_are_exact_eight_shared_poses_and_drop_boxes_are_objects(self):
        data = project_dict()
        data["sites"]["EXTRA"] = {"configured": True, "x_mm": 0, "y_mm": 0}
        with self.assertRaisesRegex(V40ValidationError, "exactly the eight"):
            ProjectV40.from_dict(data)

        data = project_dict()
        data["sites"]["F_DROP_4"] = {"configured": True, "x_mm": 0, "y_mm": 0}
        with self.assertRaisesRegex(V40ValidationError, "exactly the eight"):
            ProjectV40.from_dict(data)

        project = ProjectV40.from_dict(project_dict())
        self.assertEqual(len(project.sites), 8)
        self.assertEqual(len(project.field_objects["pickup_boxes"]), 3)
        self.assertEqual(len(project.field_objects["drop_boxes"]), 5)
        self.assertEqual(
            [item["physical_pick_site"] for item in project.field_objects["pickup_boxes"]],
            ["PICK_1", "PICK_2", "PICK_3"],
        )

    def test_configured_false_is_explicit_not_zero_sentinel(self):
        data = project_dict()
        data["sites"]["P_PICK_1"]["configured"] = False
        data["sites"]["P_PICK_1"]["x_mm"] = 0
        data["sites"]["P_PICK_1"]["y_mm"] = 0
        project = ProjectV40.from_dict(data)
        report = validate_project_site_configuration(project)
        self.assertIn("P_PICK_1", report.missing_sites)
        self.assertTrue(report.ready_for_manual_planning)
        self.assertFalse(report.ready_for_full_360_planning)

    def test_functional_hash_ignores_notes(self):
        first = ProjectV40.from_dict(project_dict())
        data = project_dict()
        data["planner_profiles"] = {"default": {"max_spacing_mm": 25, "notes": "operator note"}}
        second = ProjectV40.from_dict(data)
        data["planner_profiles"]["default"]["notes"] = "changed UI note"
        third = ProjectV40.from_dict(data)
        self.assertNotEqual(
            compute_project_functional_hashes(first)["planner_config_hash"],
            compute_project_functional_hashes(second)["planner_config_hash"],
        )
        self.assertEqual(
            compute_project_functional_hashes(second)["planner_config_hash"],
            compute_project_functional_hashes(third)["planner_config_hash"],
        )


if __name__ == "__main__":
    unittest.main()
