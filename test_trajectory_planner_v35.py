# -*- coding: utf-8 -*-
import math
import unittest

from path_models import (
    ACTION_MODE_ASYNC,
    ACTION_MODE_KINEMATIC,
    ACTION_MODE_STOP_AND_WAIT,
    EditPoint,
    MechanicalAction,
    PATH_ACT_PICK,
    PATH_ACT_PREP_STORE_1,
    PATH_ACT_STORE,
    POINT_TYPE_ARRIVAL,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_END,
    TRAJ_FLAG_START,
    YAW_ROTATION_CCW_ONLY,
    YAW_ROTATION_CW_ONLY,
    YAW_ROTATION_SHORTEST,
    YAW_UNSPECIFIED_DDEG,
)
from trajectory_planner import plan_project
from v35_test_utils import make_curve_project, make_straight_project


class TrajectoryPlannerV35Test(unittest.TestCase):
    def test_start_and_arrivals_are_zero_speed_boundaries(self):
        project = make_straight_project(5000)
        project.points.insert(
            1,
            EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL, x_mm=2200, y_mm=0, yaw_ddeg=900),
        )
        project.points[-1].point_id = 2
        result = plan_project(project)
        self.assertTrue(result.nodes[0].flags & TRAJ_FLAG_START)
        self.assertEqual(result.nodes[0].speed_mmps, 0.0)
        arrivals = [node for node in result.nodes if node.flags & TRAJ_FLAG_ARRIVAL]
        self.assertEqual([node.arrival_id for node in arrivals], [0, 1])
        self.assertTrue(arrivals[-1].flags & TRAJ_FLAG_END)
        self.assertTrue(all(node.speed_mmps == 0.0 for node in arrivals))

    def test_curve_has_accel_beta_and_wheel_limits(self):
        project = make_curve_project()
        project.vehicle_profile.wheel_plan_limit_rpm = 160
        result = plan_project(project)
        curve_nodes = [node for node in result.nodes if abs(node.curvature_kappa_per_m) > 0.05]
        self.assertTrue(curve_nodes)
        self.assertLessEqual(result.summary.max_wheel_rpm, 160.75)
        for node in result.nodes:
            expected_beta = (
                node.q_rad_per_mm * node.a_t_mmps2
                + node.q_prime_rad_per_mm2 * node.speed_mmps * node.speed_mmps
            )
            self.assertAlmostEqual(node.beta_radps2, expected_beta, places=8)

    def test_waypoint_does_not_anchor_yaw(self):
        project = make_curve_project()
        project.points[1].exact_pass = True
        project.points[1].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        result = plan_project(project)
        waypoint = next(node for node in result.nodes if node.source_point == 1)
        self.assertNotAlmostEqual(math.degrees(waypoint.yaw_rad), 25.5, places=3)

    def _yaw_delta_for_policy(self, policy: str) -> float:
        project = make_straight_project(3000)
        project.planner.yaw_rotation_policy = policy
        project.points[0].yaw_ddeg = 1700
        project.points[1].yaw_ddeg = -1700
        result = plan_project(project)
        return math.degrees(result.nodes[-1].yaw_rad - result.nodes[0].yaw_rad)

    def test_yaw_rotation_policies(self):
        self.assertAlmostEqual(self._yaw_delta_for_policy(YAW_ROTATION_SHORTEST), 20.0, delta=0.5)
        self.assertAlmostEqual(self._yaw_delta_for_policy(YAW_ROTATION_CCW_ONLY), 20.0, delta=0.5)
        self.assertAlmostEqual(self._yaw_delta_for_policy(YAW_ROTATION_CW_ONLY), -340.0, delta=0.5)
        project = make_straight_project(3000)
        project.planner.yaw_rotation_policy = YAW_ROTATION_CCW_ONLY
        project.points[0].yaw_ddeg = 0
        project.points[1].yaw_ddeg = 0
        result = plan_project(project)
        self.assertAlmostEqual(result.nodes[-1].yaw_rad - result.nodes[0].yaw_rad, 0.0, places=9)

    def test_action_validation(self):
        project = make_straight_project(3000)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PICK,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
                timeout_ms=0,
            )
        ]
        with self.assertRaisesRegex(ValueError, "timeout"):
            plan_project(project)
        project.actions[0].timeout_ms = 1000
        project.actions.append(
            MechanicalAction(
                action_seq=2,
                action=PATH_ACT_PICK,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
            )
        )
        with self.assertRaisesRegex(ValueError, "action_seq"):
            plan_project(project)

    def test_kinematic_requires_limit_and_stable_time(self):
        project = make_straight_project(3000)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PICK,
                mode=ACTION_MODE_KINEMATIC,
                timeout_ms=1000,
            )
        ]
        with self.assertRaisesRegex(ValueError, "至少需要一个运动限制"):
            plan_project(project)
        project.actions[0].speed_limit_mmps = 800
        with self.assertRaisesRegex(ValueError, "stable_time"):
            plan_project(project)
        project.actions[0].stable_time_ms = 100
        result = plan_project(project)
        self.assertNotEqual(result.actions[0].check_start_s_mm, 0xFFFF)
        self.assertEqual(result.actions[0].execution_hint, "MOVING")

    def test_prep_store_pairing(self):
        project = make_straight_project(3000)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PREP_STORE_1,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
            ),
            MechanicalAction(
                action_seq=1,
                action=PATH_ACT_STORE,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
            ),
        ]
        plan_project(project)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_STORE,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
            )
        ]
        with self.assertRaisesRegex(ValueError, "PREP_STORE"):
            plan_project(project)

    def test_departure_lock_uses_max_bound_stop_action_seq(self):
        project = make_straight_project(3000)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PREP_STORE_1,
                mode=ACTION_MODE_ASYNC,
                timeout_ms=1000,
            ),
            MechanicalAction(
                action_seq=1,
                action=PATH_ACT_PICK,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
                timeout_ms=1000,
            ),
            MechanicalAction(
                action_seq=2,
                action=PATH_ACT_STORE,
                mode=ACTION_MODE_STOP_AND_WAIT,
                arrival_point_id=1,
                timeout_ms=1000,
            ),
        ]
        result = plan_project(project)
        self.assertEqual(len(result.departure_locks), 1)
        self.assertEqual(result.departure_locks[0].arrival_id, 0)
        self.assertEqual(result.departure_locks[0].departure_action_seq, 2)
        self.assertEqual(result.departure_locks[0].bound_action_seqs, [1, 2])


if __name__ == "__main__":
    unittest.main()
