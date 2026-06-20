from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.enums import LegState, YawPolicy
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName, LegOptimizationRequest, Pose2D
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.task_plan import TransitionRequirement
from hjmb_pathgen.py_domain.topology import TopologyGate, TopologyGateDirection
from hjmb_pathgen.py_planning.optimization.leg_optimizer import optimize_leg
from hjmb_pathgen.py_services.leg_optimization_service import leg_request_from_transition

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def minimal_project() -> ProjectV40:
    return ProjectV40.from_dict(json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8")))


class Phase6LegOptimizerTest(unittest.TestCase):
    def test_straight_leg_optimizes_with_zero_boundaries(self):
        project = minimal_project()
        request = LegOptimizationRequest(
            project=project,
            from_state_id="P_START",
            to_state_id="P_PICK_1",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="PICK_1_TO_3",
            topology_profile="NONE",
            profile_name=LegOptimizationProfileName.QUICK_PREVIEW,
        )
        result = optimize_leg(request)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.state, LegState.PREVIEW_VALID)
        self.assertIsNotNone(result.leg)
        leg = result.leg
        self.assertGreaterEqual(len(leg.nodes), 2)
        self.assertEqual(leg.nodes[0]["local_s_mm"], 0)
        self.assertEqual(leg.nodes[0]["vx_mmps"], 0)
        self.assertEqual(leg.nodes[-1]["vx_mmps"], 0)
        self.assertNotIn("arrival_id", leg.nodes[-1])
        self.assertIn("validity_hash", leg.hashes)
        self.assertIn("self_hash32", leg.hashes)

    def test_topology_gate_failure_is_a_candidate_failure(self):
        project = minimal_project()
        gate = TopologyGate("G_NEG", 50, -20, 50, 20, TopologyGateDirection.POSITIVE)
        request = LegOptimizationRequest(
            project=project,
            from_state_id="A",
            to_state_id="B",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="MANUAL",
            topology_profile="TEST",
            topology_gates=(gate,),
            profile_name=LegOptimizationProfileName.QUICK_PREVIEW,
        )
        result = optimize_leg(request)
        self.assertFalse(result.success)
        self.assertTrue(result.evaluations)
        self.assertTrue(all(not item.success for item in result.evaluations))

    def test_optimizer_hash_is_deterministic_for_same_request(self):
        project = minimal_project()
        gate = TopologyGate("G_OK", 50, -20, 50, 20, TopologyGateDirection.NEGATIVE)
        request = LegOptimizationRequest(
            project=project,
            from_state_id="A",
            to_state_id="B",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="MANUAL",
            topology_profile="TEST",
            topology_gates=(gate,),
            profile_name=LegOptimizationProfileName.QUICK_PREVIEW,
            yaw_policy=YawPolicy.SHORTEST,
        )
        first = optimize_leg(request)
        second = optimize_leg(request)
        self.assertTrue(first.success)
        self.assertTrue(second.success)
        self.assertEqual(first.leg.hashes["validity_hash"], second.leg.hashes["validity_hash"])
        self.assertEqual(first.leg.nodes, second.leg.nodes)

    def test_transition_request_rejects_missing_target_pose(self):
        project = minimal_project()
        transition = TransitionRequirement(
            requirement_id="TR_BAD",
            semantic_hash="bad",
            from_state_id="P_START",
            to_state_id="DROP_STEP_1",
            route_family="PICK_1_TO_3",
            topology_profile="PICK_1_TO_3",
            from_pose={"x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
            to_pose={"finish_policy": {"mode": "AT_FINAL_DROP"}},
            dependency_hashes={},
            reason="TEST",
        )
        with self.assertRaisesRegex(ValueError, "missing pose fields"):
            leg_request_from_transition(transition, project)

    def test_cancel_after_valid_candidate_returns_best_leg(self):
        project = minimal_project()
        checks = {"count": 0}

        def cancel_after_first_batch_check() -> bool:
            checks["count"] += 1
            return checks["count"] > 1

        request = LegOptimizationRequest(
            project=project,
            from_state_id="A",
            to_state_id="B",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="MANUAL",
            topology_profile="NONE",
            profile_name=LegOptimizationProfileName.STANDARD,
            cancel_check=cancel_after_first_batch_check,
        )
        result = optimize_leg(request)
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.state, LegState.CANCELLED_WITH_BEST)
        self.assertIsNotNone(result.leg)

    def test_invalid_profile_is_rejected_before_search(self):
        data = json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))
        data["planner_profiles"] = {"STANDARD": {"oversample_per_segment": 4}}
        project = ProjectV40.from_dict(data)
        request = LegOptimizationRequest(
            project=project,
            from_state_id="A",
            to_state_id="B",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="MANUAL",
            topology_profile="NONE",
            profile_name=LegOptimizationProfileName.STANDARD,
        )
        result = optimize_leg(request)
        self.assertFalse(result.success)
        self.assertIn("oversample_per_segment", result.reason)


if __name__ == "__main__":
    unittest.main()
