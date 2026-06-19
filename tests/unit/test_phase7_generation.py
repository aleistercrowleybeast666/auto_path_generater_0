from __future__ import annotations

import math
import hashlib
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.codec.canonical_json import canonical_json_bytes, canonical_json_crc32_hex
from hjmb_pathgen.codec.json_codec import load_case, load_leg_library, save_case, save_leg_library, save_route_case_table
from hjmb_pathgen.models.enums import LegState, NodeFlag, PathSource
from hjmb_pathgen.models.leg import LegLibraryV40, LegV40
from hjmb_pathgen.models.route_case import CaseManifestV40, RouteCaseRowV40, RouteCaseTableV40
from hjmb_pathgen.planning.leg_optimizer import PLANNER_ALGORITHM_VERSION
from hjmb_pathgen.services.leg_clear_service import clear_optimized_leg_result
from hjmb_pathgen.services.leg_library_service import show_leg
from hjmb_pathgen.services.mode_output_service import write_manual_free_outputs
from hjmb_pathgen.services.phase7_generation_service import collect_unique_legs, evaluate_case_candidates, generate_all, generate_one, validate_one
from hjmb_pathgen.services.project_service import ProjectLayout

from phase3_helpers import phase3_project


def route_row(traj_id: int = 0) -> RouteCaseRowV40:
    return RouteCaseRowV40.from_dict(
        {
            "traj_id": traj_id,
            "file_name": f"P{traj_id:04d}.BIN",
            "bean_code": 0,
            "drop_code": traj_id,
            "pick_assignment": {"PICK_1": "YELLOW", "PICK_2": "GREEN", "PICK_3": "WHITE"},
            "label_positions": {"1": "F_DROP_4", "2": "F_DROP_5", "3": "F_DROP_6", "4": "F_DROP_7", "5": "F_DROP_8"},
            "source_row_hash": f"rowhash{traj_id}",
        }
    )


def write_one_row_project(root: Path) -> ProjectLayout:
    layout = ProjectLayout.create(root, phase3_project())
    table = RouteCaseTableV40(source_csv="fixture", source_csv_sha256="fixture", cases=(route_row(),))
    save_route_case_table(layout.route_case_table_json, table)
    return layout


def synthetic_leg(requirement: dict, leg_id: str, planned_time_ms: int = 100) -> LegV40:
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
        leg_id=leg_id,
        key={"test_requirement_id": requirement["requirement_id"]},
        state=LegState.VALID,
        source="PHASE7_TEST_SYNTHETIC",
        topology_profile=transition["topology_profile"],
        control_points=(),
        yaw_profile={},
        nodes=(first, last),
        analysis={"planned_time_ms": planned_time_ms, "total_length_mm": distance},
        hashes={"dependency_hashes": {}, "planner_algorithm_version": PLANNER_ALGORITHM_VERSION},
        review={"approved": True, "locked": False, "state": "VALID", "notes": ""},
    )
    payload = _leg_validity_payload(leg)
    hashes = dict(leg.hashes)
    hashes["self_hash32"] = f"0x{canonical_json_crc32_hex(payload).upper()}"
    hashes["validity_hash"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    return replace(leg, hashes=hashes)


def _leg_validity_payload(leg: LegV40) -> dict:
    metrics = dict(leg.analysis.get("max_metrics", {}))
    return {
        "planner_algorithm_version": str(leg.hashes.get("planner_algorithm_version", PLANNER_ALGORITHM_VERSION)),
        "key": dict(leg.key),
        "control_points": list(leg.control_points),
        "yaw_profile": dict(leg.yaw_profile),
        "nodes": list(leg.nodes),
        "analysis_semantic": {
            "planned_time_ms": leg.analysis.get("planned_time_ms", 0),
            "total_length_mm": leg.analysis.get("total_length_mm", 0),
            "max_metrics": metrics,
            "min_clearance_mm": leg.analysis.get("min_clearance_mm"),
        },
    }


def populate_library_for_collection(layout: ProjectLayout, collection: dict) -> LegLibraryV40:
    project = phase3_project()
    legs = tuple(synthetic_leg(item, item["leg_id"]) for item in collection["requirements"])
    library = LegLibraryV40(planner_version="test", project_hash=canonical_json_crc32_hex(project.to_dict()), legs=legs)
    save_leg_library(layout.leg_library_json, library)
    return library


def manual_case_dict(traj_id: int = 0) -> dict:
    return {
        "format": "HJMB_ROUTE_CASE_JSON_V40",
        "storage_mode": "REFERENCED",
        "path_source": "MANUAL_FREE",
        "traj_id": traj_id,
        "bean_code": 0,
        "drop_code": traj_id,
        "source_mapping": {"manual": True},
        "selected_plan": {
            "route_family": "MANUAL_FREE",
            "vehicle_bin_assignment": {},
            "drop_targets": [],
            "unload_sequence": [],
            "yaw_direction": "SHORTEST",
            "locked_by_user": True,
        },
        "manual_path": {
            "points": [
                {"type": "START", "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
                {"type": "WAYPOINT", "x_mm": 250, "y_mm": 0, "max_speed_mmps": 300},
                {"type": "ARRIVAL", "x_mm": 500, "y_mm": 0, "yaw_ddeg": 0},
            ]
        },
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
            "override_reason": "phase8 manual coexist test",
        },
    }


def load_case_dict(data: dict) -> CaseManifestV40:
    return CaseManifestV40.from_dict(data)


def load_case_safe_leg_library(layout: ProjectLayout, leg_id: str) -> LegV40:
    return show_leg(load_leg_library(layout.leg_library_json), leg_id)


class Phase7GenerationTest(unittest.TestCase):
    def test_collect_unique_legs_and_generate_byte_identical_single_vs_batch(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            empty_collection = collect_unique_legs(layout)
            self.assertGreater(len(empty_collection.requirements), 0)
            self.assertEqual(empty_collection.counts_by_status, {"MISSING": len(empty_collection.requirements)})

            populate_library_for_collection(layout, empty_collection.to_dict())
            ready_collection = collect_unique_legs(layout)
            self.assertEqual(ready_collection.counts_by_status, {"REUSABLE": len(ready_collection.requirements)})

            single = generate_one(layout, 0)
            task_bin = layout.bin_path_for_source(0, PathSource.TASK_COMPILED)
            task_case = layout.case_json_path_for_source(0, PathSource.TASK_COMPILED)
            first_bytes = task_bin.read_bytes()
            case = load_case(task_case)
            self.assertFalse(case.review["approved"])
            self.assertTrue(case.actions["compiled"][-1]["action"].startswith("DROP_"))
            self.assertFalse(single.case.to_dict()["leg_refs"] == [])
            self.assertFalse(layout.bin_path(0).exists())

            validation = validate_one(layout, 0)
            self.assertTrue(validation["valid"])
            self.assertFalse(validation["final_export_allowed"])
            self.assertIn("review.approved is false", validation["final_export_blockers"])
            self.assertFalse(validation["last_node_flags"] & int(NodeFlag.SAFE_END))
            self.assertTrue(validation["last_node_flags"] & int(NodeFlag.FINISH_ARM))

            batch = generate_all(layout)
            self.assertEqual(batch.failures, ())
            self.assertEqual(task_bin.read_bytes(), first_bytes)

    def test_manual_and_task_outputs_coexist_in_mode_directories(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())

            manual_case = load_case_dict(manual_case_dict())
            manual = write_manual_free_outputs(layout, manual_case)
            task = generate_one(layout, 0)

            self.assertEqual(manual.case_path, layout.case_json_path_for_source(0, PathSource.MANUAL_FREE))
            self.assertEqual(task.output.case_path, layout.case_json_path_for_source(0, PathSource.TASK_COMPILED))
            self.assertTrue(layout.bin_path_for_source(0, PathSource.MANUAL_FREE).exists())
            self.assertTrue(layout.bin_path_for_source(0, PathSource.TASK_COMPILED).exists())

    def test_legacy_flat_manual_blocks_task_generation_without_replace_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())
            save_case(layout.case_json_path(0), load_case_dict(manual_case_dict()))

            with self.assertRaisesRegex(Exception, "replace-manual"):
                generate_one(layout, 0)

            result = generate_one(layout, 0, replace_manual=True)
            self.assertEqual(result.output.case_path, layout.case_json_path_for_source(0, PathSource.TASK_COMPILED))

    def test_locked_candidate_is_preserved_when_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())
            evaluation = evaluate_case_candidates(layout, 0)
            self.assertGreaterEqual(len(evaluation.timings), 1)
            locked = evaluation.timings[-1]
            generate_one(layout, 0)
            task_path = layout.case_json_path_for_source(0, PathSource.TASK_COMPILED)
            case = load_case(task_path)
            selected_plan = dict(case.selected_plan)
            selected_plan["candidate_id"] = locked.candidate_id
            selected_plan["semantic_hash"] = locked.semantic_hash
            selected_plan["locked_by_user"] = True
            save_case(task_path, replace(case, selected_plan=selected_plan))

            regenerated = generate_one(layout, 0)
            self.assertEqual(regenerated.selected_candidate_id, locked.candidate_id)
            self.assertEqual(regenerated.case.selected_plan["selection_state"], "LOCKED_PRESERVED_PHASE8")

    def test_clear_optimized_leg_result_requires_confirmation_for_approved_leg(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            library = populate_library_for_collection(layout, collection.to_dict())
            leg_id = library.legs[0].leg_id

            with self.assertRaisesRegex(Exception, "confirm-leg-id"):
                clear_optimized_leg_result(layout, leg_id)

            cleared = clear_optimized_leg_result(layout, leg_id, confirm_leg_id=leg_id)
            self.assertEqual(cleared.new_state, LegState.MISSING.value)
            loaded = load_case_safe_leg_library(layout, leg_id)
            self.assertEqual(loaded.state, LegState.MISSING)
            self.assertEqual(loaded.nodes, ())


if __name__ == "__main__":
    unittest.main()
