from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.collision import CollisionStatus
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_planning.dynamics.time_parameterization import GeometrySample
from hjmb_pathgen.py_services.export_guard_service import check_formal_export_guard
from hjmb_pathgen.py_services.manual_path_service import build_manual_spatial_path
from hjmb_pathgen.py_services.path_validation_service import (
    case_with_collision_result,
    collision_result_is_stale,
    validate_case_collision,
    validate_spatial_path_collision,
)

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def project_dict() -> dict:
    return json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))


def manual_case_dict() -> dict:
    return {
        "format": "HJMB_ROUTE_CASE_JSON_V40",
        "storage_mode": "REFERENCED",
        "generation_mode": "MANUAL",
        "traj_id": 0,
        "bean_code": 0,
        "drop_code": 0,
        "source_mapping": {"manual": True},
        "selected_plan": {
            "route_family": "MANUAL",
            "vehicle_bin_assignment": {},
            "drop_targets": [],
            "unload_sequence": [],
            "yaw_direction": "SHORTEST",
            "locked_by_user": True,
        },
        "manual_path": {
            "points": [
                {"type": "START", "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
                {"type": "WAYPOINT", "x_mm": 500, "y_mm": 0},
                {"type": "ARRIVAL", "x_mm": 1000, "y_mm": 0, "yaw_ddeg": 0},
            ]
        },
        "logical_points": [],
        "arrival_states": [],
        "leg_refs": [],
        "actions": {"source": [], "compiled": []},
        "finish": {"mode": "AT_FINAL_DROP"},
        "estimates": {},
        "hashes": {},
        "review": {
            "detached_from_library": True,
            "manual_override": True,
            "approved": True,
            "override_reason": "phase5 manual free test",
        },
    }


def project_with_center_cylinder() -> ProjectV40:
    data = project_dict()
    data["field_objects"]["cylinders"][0]["center_x_mm"] = 500
    data["field_objects"]["cylinders"][0]["center_y_mm"] = 200
    data["field_objects"]["cylinders"][0]["radius_mm"] = 30
    data["field_objects"]["cylinders"][1]["enabled"] = False
    for group in ("pickup_boxes", "drop_boxes"):
        for item in data["field_objects"][group]:
            item["enabled"] = False
    return ProjectV40.from_dict(data)


class Phase5PathValidationTest(unittest.TestCase):
    def test_manual_case_validation_passes_and_writes_report(self):
        project = ProjectV40.from_dict(project_dict())
        case = CaseManifestV40.from_dict(manual_case_dict())
        with tempfile.TemporaryDirectory() as tmp:
            report_path = Path(tmp) / "P0000_collision.json"
            result = validate_case_collision(case, project, report_path=report_path)
            self.assertEqual(result.status, CollisionStatus.PASSED, result.to_dict())
            self.assertGreaterEqual(result.min_clearance_mm, 0)
            self.assertTrue(report_path.exists())
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["result"]["status"], "PASSED")
            updated = case_with_collision_result(case, result)
            self.assertEqual(updated.review["collision_status"], "PASSED")

    def test_continuous_check_finds_collision_between_safe_endpoints(self):
        project = project_with_center_cylinder()
        samples = (
            GeometrySample(0, 0, 200, 0, 1, 0),
            GeometrySample(1000, 1000, 200, 0, 1, 0),
        )
        result = validate_spatial_path_collision(samples, project)
        self.assertEqual(result.status, CollisionStatus.FAILED, result.to_dict())
        self.assertIsNotNone(result.first_collision)
        assert result.first_collision is not None
        self.assertEqual(result.first_collision.source["kind"], "midpoint")
        self.assertGreater(result.subdivision_count, 0)

    def test_pure_rotation_can_cause_pickup_clipped_disk_collision(self):
        data = project_dict()
        for group in ("cylinders", "drop_boxes"):
            for item in data["field_objects"][group]:
                item["enabled"] = False
        data["field_objects"]["pickup_boxes"] = [
            {"obstacle_id": "PICKUP_BOX_1", "physical_pick_site": "PICK_1", "center_x_mm": 105, "center_y_mm": 0, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": True},
            {"obstacle_id": "PICKUP_BOX_2", "physical_pick_site": "PICK_2", "center_x_mm": 1800, "center_y_mm": 700, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": False},
            {"obstacle_id": "PICKUP_BOX_3", "physical_pick_site": "PICK_3", "center_x_mm": 1800, "center_y_mm": 800, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": False},
        ]
        project = ProjectV40.from_dict(data)
        samples = (
            GeometrySample(0, 0, 0, 0, 1, 0),
            GeometrySample(0, 0, 0, 900, 1, 0),
        )
        result = validate_spatial_path_collision(samples, project)
        self.assertEqual(result.status, CollisionStatus.FAILED, result.to_dict())
        self.assertGreater(result.checked_pose_count, 1)

    def test_hash_stale_and_formal_export_guard(self):
        project = ProjectV40.from_dict(project_dict())
        case = CaseManifestV40.from_dict(manual_case_dict())
        result = validate_case_collision(case, project)
        updated = case_with_collision_result(case, result)
        guard = check_formal_export_guard(updated)
        self.assertTrue(guard.allowed, guard.to_dict())

        changed = project_dict()
        changed["vehicle"]["footprint"]["r_large_mm"] = 130
        changed_project = ProjectV40.from_dict(changed)
        samples = build_manual_spatial_path(case.manual_path)
        self.assertTrue(collision_result_is_stale(result, samples, changed_project))

        unchecked = CaseManifestV40.from_dict(manual_case_dict())
        unchecked_guard = check_formal_export_guard(unchecked)
        self.assertFalse(unchecked_guard.allowed)
        self.assertIn("collision_status=NOT_CHECKED", unchecked_guard.reasons)

    def test_full_auto_case_without_geometry_reports_no_geometry(self):
        project = ProjectV40.from_dict(project_dict())
        case = CaseManifestV40.from_dict(json.loads((FIXTURE_ROOT / "minimal_case.json").read_text(encoding="utf-8")))
        result = validate_case_collision(case, project)
        self.assertEqual(result.status, CollisionStatus.NO_GEOMETRY)

    def test_full_auto_case_with_manual_geometry_can_be_checked_without_optimization(self):
        project = ProjectV40.from_dict(project_dict())
        case = CaseManifestV40.from_dict(json.loads((FIXTURE_ROOT / "minimal_case.json").read_text(encoding="utf-8")))
        samples = (
            GeometrySample(0, 0, 0, 0, 1, 0),
            GeometrySample(1000, 1000, 0, 0, 1, 0),
        )
        result = validate_case_collision(case, project, samples=samples)
        self.assertEqual(result.status, CollisionStatus.PASSED, result.to_dict())


if __name__ == "__main__":
    unittest.main()
