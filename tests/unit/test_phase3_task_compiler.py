from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.route_case import RouteCaseRowV40
from hjmb_pathgen.py_domain.task_mapping import drop_targets_from_label_positions
from hjmb_pathgen.py_services.task_compiler import build_case_draft, compile_task_candidates
from hjmb_pathgen.py_utils.yaw_unwrap import unwrap_yaw_sequence

from phase3_helpers import phase3_project, phase3_project_dict


def route_row(
    *,
    traj_id: int = 0,
    pick_assignment: dict[str, str] | None = None,
    label_positions: dict[str, str] | None = None,
) -> RouteCaseRowV40:
    return RouteCaseRowV40.from_dict(
        {
            "traj_id": traj_id,
            "file_name": f"P{traj_id:04d}.BIN",
            "bean_code": traj_id // 60,
            "drop_code": traj_id % 60,
            "pick_assignment": pick_assignment or {"PICK_1": "YELLOW", "PICK_2": "GREEN", "PICK_3": "WHITE"},
            "label_positions": label_positions
            or {"1": "F_DROP_4", "2": "F_DROP_5", "3": "F_DROP_6", "4": "F_DROP_7", "5": "F_DROP_8"},
            "source_row_hash": "rowhash",
            "source_row_number": 2,
            "raw_fields": {},
        }
    )


class Phase3TaskCompilerTest(unittest.TestCase):
    def test_drop_target_spatial_order_and_empty_boxes(self):
        labels = {"1": "F_DROP_8", "2": "F_DROP_4", "3": "F_DROP_6", "4": "F_DROP_5", "5": "F_DROP_7"}
        targets = drop_targets_from_label_positions(labels)
        self.assertEqual([target.target_rank for target in targets], [1, 2, 3])
        self.assertEqual([target.bean_type.value for target in targets], ["GREEN", "WHITE", "YELLOW"])
        self.assertEqual([target.label_number for target in targets], [2, 3, 1])
        self.assertNotIn("F_DROP_5", [target.physical_site.value for target in targets])

        swapped = dict(labels, **{"4": "F_DROP_7", "5": "F_DROP_5"})
        self.assertEqual([target.to_dict() for target in targets], [target.to_dict() for target in drop_targets_from_label_positions(swapped)])

    def test_two_route_families_pick2_side_and_bin_assignment(self):
        candidates = compile_task_candidates(route_row(), phase3_project()).candidates
        route_families = {candidate.route_family.name for candidate in candidates}
        self.assertEqual(route_families, {"PICK_1_TO_3", "PICK_3_TO_1"})
        pick_1_to_3 = [candidate for candidate in candidates if candidate.route_family.name == "PICK_1_TO_3"][0]
        pick_3_to_1 = [candidate for candidate in candidates if candidate.route_family.name == "PICK_3_TO_1"][0]

        self.assertEqual(pick_1_to_3.pickup_arrival_state_order, ("P_PICK_1", "P_PICK_2L", "P_PICK_3"))
        self.assertEqual(pick_3_to_1.pickup_arrival_state_order, ("P_PICK_3", "P_PICK_2R", "P_PICK_1"))
        self.assertEqual(pick_1_to_3.drop_target_rank_order, (3, 2, 1))
        self.assertEqual(pick_3_to_1.drop_target_rank_order, (1, 2, 3))
        self.assertEqual(sorted(pick_1_to_3.vehicle_bin_assignment), ["GREEN", "WHITE", "YELLOW"])
        self.assertEqual(sorted(pick_1_to_3.vehicle_bin_assignment.values()), ["BIN_1", "BIN_2", "BIN_3"])

    def test_dual_unload_candidates_are_adjacent_only_and_never_forbidden_masks(self):
        separated = route_row(label_positions={"1": "F_DROP_4", "2": "F_DROP_6", "3": "F_DROP_8", "4": "F_DROP_5", "5": "F_DROP_7"})
        separated_candidates = compile_task_candidates(separated, phase3_project()).candidates
        self.assertEqual(len(separated_candidates), 2)
        self.assertTrue(all(len(candidate.unload_sequence) == 3 for candidate in separated_candidates))

        consecutive_candidates = compile_task_candidates(route_row(), phase3_project()).candidates
        self.assertEqual(len(consecutive_candidates), 6)
        masks = {step.unload_mask.value for candidate in consecutive_candidates for step in candidate.unload_sequence}
        self.assertNotIn("BIN_13", masks)
        self.assertNotIn("BIN_123", masks)
        self.assertTrue({"BIN_12", "BIN_23"}.issubset(masks))

    def test_dual_unload_requires_manual_yaw_profile(self):
        data = phase3_project_dict()
        data["unload_profiles"]["BIN_12"]["configured"] = False
        project = phase3_project().__class__.from_dict(data)
        candidate_set = compile_task_candidates(route_row(), project)
        masks = {step.unload_mask.value for candidate in candidate_set.candidates for step in candidate.unload_sequence}
        self.assertNotIn("BIN_12", masks)
        self.assertIn("missing or uncalibrated", " ".join(candidate_set.unavailable_reasons))

    def test_source_actions_preserve_pick_store_unload_bin_mapping(self):
        candidate = compile_task_candidates(route_row(), phase3_project()).candidates[0]
        stored: dict[str, str] = {}
        dropped: list[str] = []
        for action in candidate.source_actions:
            self.assertNotIn("arrival_id", action)
            self.assertNotIn("check_start_s_mm", action)
            if action["action"] == "STORE":
                stored[action["vehicle_bin"]] = action["bean_type"]
            if action["action"].startswith("DROP_"):
                for vehicle_bin, bean_type in zip(action["vehicle_bins"], action["bean_types"], strict=True):
                    self.assertEqual(stored.pop(vehicle_bin), bean_type)
                    dropped.append(bean_type)
        self.assertEqual(stored, {})
        self.assertEqual(sorted(dropped), ["GREEN", "WHITE", "YELLOW"])

    def test_missing_action_profile_makes_candidate_unavailable(self):
        data = phase3_project_dict()
        del data["action_profiles"]["STORE"]
        project = phase3_project().__class__.from_dict(data)
        candidate_set = compile_task_candidates(route_row(), project)
        self.assertEqual(candidate_set.candidates, ())
        self.assertIn("missing action_profile: STORE", " ".join(candidate_set.unavailable_reasons))

    def test_yaw_unwrap_monotonic_and_same_angle(self):
        self.assertEqual(unwrap_yaw_sequence((900, 0, -900), "CW_ONLY"), (900, 0, -900))
        self.assertEqual(unwrap_yaw_sequence((-900, 0, 900), "CCW_ONLY"), (-900, 0, 900))
        self.assertEqual(unwrap_yaw_sequence((0, 0, 0), "CW_ONLY"), (0, 0, 0))
        with self.assertRaises(CompileError):
            unwrap_yaw_sequence(tuple(900 if index % 2 == 0 else -900 for index in range(20)), "CCW_ONLY")

    def test_case_draft_and_lock_preservation(self):
        row = route_row()
        project = phase3_project()
        first = build_case_draft(row, project)
        self.assertEqual(first.case.leg_refs, ())
        self.assertEqual(first.case.actions["compiled"], [])
        self.assertFalse(first.case.selected_plan["locked_by_user"])
        candidate_id = first.candidate_set.candidates[-1].candidate_id

        locked = build_case_draft(row, project, preferred_candidate_id=candidate_id, lock_selected=True)
        self.assertTrue(locked.case.selected_plan["locked_by_user"])
        self.assertEqual(
            {item["point_id"] for item in locked.case.logical_points},
            {"P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3", "P_DROP_1", "P_DROP_2", "P_DROP_3"},
        )
        self.assertEqual(len(locked.case.logical_points), 8)
        drop_1 = next(item for item in locked.case.logical_points if item["point_id"] == "P_DROP_1")
        drop_box = next(
            item for item in project.field_objects["drop_boxes"]
            if item["physical_drop_site"] == drop_1["physical_drop_site"]
        )
        self.assertNotEqual(
            (drop_1["pose"]["x_mm"], drop_1["pose"]["y_mm"]),
            (drop_box["center_x_mm"], drop_box["center_y_mm"]),
        )
        rebuilt = build_case_draft(row, project, existing_case=locked.case)
        self.assertEqual(rebuilt.case.selected_plan["candidate_id"], candidate_id)
        self.assertTrue(rebuilt.case.selected_plan["locked_by_user"])

        broken = dict(locked.case.selected_plan)
        broken["candidate_id"] = "C_MISSING"
        broken_case = locked.case.__class__(**{**locked.case.__dict__, "selected_plan": broken})
        with self.assertRaises(CompileError):
            build_case_draft(row, project, existing_case=broken_case)


if __name__ == "__main__":
    unittest.main()
