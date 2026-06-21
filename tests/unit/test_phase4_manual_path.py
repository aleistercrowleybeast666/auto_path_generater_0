from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.enums import HeaderFlag, NodeFlag
from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_services.manual_path_service import build_manual_spatial_path, plan_manual_case

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def project() -> ProjectV40:
    data = json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))
    data["planner_profiles"] = {"default": {"max_spacing_mm": 25}}
    return ProjectV40.from_dict(data)


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
                {"type": "WAYPOINT", "x_mm": 500, "y_mm": 0, "max_speed_mmps": 500},
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
            "approved": False,
            "override_reason": "phase4 manual free test",
        },
    }


class Phase4ManualPathTest(unittest.TestCase):
    def test_manual_path_schema_and_planning(self):
        case = CaseManifestV40.from_dict(manual_case_dict())
        samples = build_manual_spatial_path(case.manual_path)
        self.assertEqual(samples[0].s_mm, 0)
        self.assertTrue(samples[0].flags & int(NodeFlag.START))
        self.assertTrue(samples[-1].flags & int(NodeFlag.ARRIVAL))

        result = plan_manual_case(case, project())
        self.assertTrue(result.timing.success, result.timing.reason)
        self.assertIsNotNone(result.trajectory)
        assert result.trajectory is not None
        self.assertTrue(result.trajectory.header.flags & int(HeaderFlag.MANUAL_OVERRIDE))
        self.assertEqual(result.trajectory.nodes[0].vx_mmps, 0)
        self.assertEqual(result.trajectory.nodes[0].wz_ddegps, 0)
        self.assertEqual(result.trajectory.nodes[-1].vx_mmps, 0)
        self.assertEqual(result.trajectory.nodes[-1].wz_ddegps, 0)
        self.assertFalse(result.trajectory.nodes[-1].flags & int(NodeFlag.SAFE_END))


    def test_manual_yaw_is_distributed_across_complete_stop_interval(self):
        data = manual_case_dict()
        data["manual_path"]["points"][-1]["yaw_ddeg"] = 900
        case = CaseManifestV40.from_dict(data)
        samples = build_manual_spatial_path(case.manual_path)
        moving = [sample for sample in samples if 0.0 < sample.s_mm < samples[-1].s_mm]
        self.assertTrue(moving)
        q_values = {round(sample.yaw_ddeg_per_mm, 9) for sample in moving}
        self.assertEqual(len(q_values), 1)
        self.assertTrue(all(sample.yaw_ddeg_per_mm2 == 0.0 for sample in moving))
        result = plan_manual_case(case, project())
        self.assertTrue(result.timing.success, result.timing.reason)
        self.assertGreater(result.timing.max_metrics["max_wz_ddegps"], 0.0)

    def test_full_auto_case_rejects_manual_path_payload(self):
        data = json.loads((FIXTURE_ROOT / "minimal_case.json").read_text(encoding="utf-8"))
        data["manual_path"] = manual_case_dict()["manual_path"]
        with self.assertRaisesRegex(V40ValidationError, "FULL_AUTO"):
            CaseManifestV40.from_dict(data)

    def test_manual_requires_override_reason(self):
        data = manual_case_dict()
        data["review"]["override_reason"] = ""
        with self.assertRaisesRegex(V40ValidationError, "override_reason"):
            CaseManifestV40.from_dict(data)


if __name__ == "__main__":
    unittest.main()
