# -*- coding: utf-8 -*-
import math
import unittest

from path_models import (
    EditPoint,
    PreviewInitialPose,
    POINT_TYPE_ARRIVAL,
    YAW_UNSPECIFIED_DDEG,
)
from trajectory_planner import estimate_cut_in_preview, plan_project
from v33_test_utils import make_curve_project, make_straight_project


class TrajectoryPlannerV33Test(unittest.TestCase):
    def test_long_straight_uses_nonzero_start_and_near_max_accel(self):
        project = make_straight_project(6000)
        result = plan_project(project)
        self.assertAlmostEqual(
            result.nodes[0].speed_mmps, project.cut_in.target_speed_mmps, delta=1e-6
        )
        self.assertEqual(result.nodes[-1].speed_mmps, 0.0)
        self.assertLessEqual(
            result.summary.max_speed_mmps, project.planner.max_speed_mmps + 1e-6
        )
        positive_accels = [
            node.a_t_mmps2 for node in result.nodes if node.a_t_mmps2 > 0
        ]
        self.assertGreater(max(positive_accels), 0.95 * project.planner.linear_accel_mmps2)
        self.assertGreater(result.summary.formal_time_ms, 0)
        self.assertTrue(any("正反向扫描" in warning for warning in result.warnings))
        self.assertTrue(
            all(
                node.speed_mmps + 2.0 >= project.cut_in.target_speed_mmps
                for node in result.nodes
                if node.s_mm <= project.cut_in.straight_length_mm
            )
        )

    def test_curve_has_centripetal_acceleration_and_combined_limit(self):
        project = make_curve_project()
        result = plan_project(project)
        curve_nodes = [
            node for node in result.nodes if abs(node.curvature_kappa_per_m) > 0.05
        ]
        self.assertTrue(curve_nodes)
        self.assertGreater(max(node.a_n_mmps2 for node in curve_nodes), 1.0)
        self.assertLessEqual(
            max(node.a_total_mmps2 for node in result.nodes),
            project.planner.linear_accel_mmps2 + 8.0,
        )

    def test_beta_uses_q_prime_term_and_respects_limit(self):
        project = make_curve_project()
        result = plan_project(project)
        for node in result.nodes:
            expected = (
                node.q_rad_per_mm * node.a_t_mmps2
                + node.q_prime_rad_per_mm2 * node.speed_mmps * node.speed_mmps
            )
            self.assertAlmostEqual(node.beta_radps2, expected, places=8)
        self.assertLessEqual(
            max(abs(node.beta_radps2) for node in result.nodes),
            project.planner.angular_accel_moving_radps2 + 0.02,
        )

    def test_wheel_soft_limit_and_finite_values(self):
        project = make_curve_project()
        project.vehicle_profile.wheel_plan_limit_rpm = 120
        result = plan_project(project)
        self.assertLessEqual(result.summary.max_wheel_rpm, 120.75)
        for node in result.nodes:
            for value in (
                node.speed_mmps,
                node.a_t_mmps2,
                node.a_n_mmps2,
                node.a_total_mmps2,
                node.beta_radps2,
                node.max_wheel_rpm,
            ):
                self.assertTrue(math.isfinite(value))

    def test_infeasible_cut_in_speed_is_rejected(self):
        project = make_straight_project(1000)
        project.cut_in.target_speed_mmps = 1800
        project.cut_in.approach_max_speed_mmps = 1800
        project.vehicle_profile.wheel_plan_limit_rpm = 50
        with self.assertRaisesRegex(ValueError, "CUT_IN"):
            plan_project(project)

    def test_cut_in_preview_nonzero_terminal_speed(self):
        project = make_straight_project(3000)
        project.preview_initial_pose = PreviewInitialPose(
            enabled=True,
            x_mm=-1000,
            y_mm=0,
            yaw_ddeg=0,
            initial_speed_mmps=0,
        )
        project.cut_in.target_speed_mmps = 500
        project.cut_in.approach_max_speed_mmps = 500
        preview = estimate_cut_in_preview(project)
        self.assertTrue(preview.reachable)
        self.assertEqual(preview.peak_speed_mmps, 500)
        self.assertGreater(preview.time_ms, 0)

    def test_cut_in_preview_can_accelerate_then_decelerate(self):
        project = make_straight_project(3000)
        project.preview_initial_pose = PreviewInitialPose(
            enabled=True,
            x_mm=-3000,
            y_mm=0,
            yaw_ddeg=0,
            initial_speed_mmps=100,
        )
        project.cut_in.target_speed_mmps = 500
        project.cut_in.approach_max_speed_mmps = 1200
        preview = estimate_cut_in_preview(project)
        self.assertTrue(preview.reachable)
        self.assertGreater(preview.peak_speed_mmps, 500)

    def test_cut_in_preview_reports_short_distance(self):
        project = make_straight_project(3000)
        project.preview_initial_pose = PreviewInitialPose(
            enabled=True,
            x_mm=-10,
            y_mm=0,
            yaw_ddeg=0,
            initial_speed_mmps=0,
        )
        project.cut_in.target_speed_mmps = 1000
        project.cut_in.approach_max_speed_mmps = 1000
        preview = estimate_cut_in_preview(project)
        self.assertFalse(preview.reachable)
        self.assertIn("距离不足", preview.warning)

    def test_waypoint_does_not_anchor_yaw(self):
        project = make_curve_project()
        project.points[1].exact_pass = True
        project.points[1].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        result = plan_project(project)
        waypoint = next(node for node in result.nodes if node.source_point == 1)
        t = waypoint.s_mm / result.nodes[-1].s_mm
        blend = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
        self.assertAlmostEqual(waypoint.yaw_rad, math.radians(90.0) * blend)
        self.assertNotAlmostEqual(waypoint.yaw_rad, math.radians(25.5), places=3)

    def test_arrival_yaw_is_exact_anchor_with_zero_wz(self):
        project = make_straight_project(4000)
        project.points.insert(
            1,
            EditPoint(
                point_id=1,
                type=POINT_TYPE_ARRIVAL,
                x_mm=2200,
                y_mm=0,
                yaw_ddeg=900,
            ),
        )
        project.points[-1].point_id = 2
        project.points[-1].yaw_ddeg = 0
        result = plan_project(project)
        arrival = next(node for node in result.nodes if node.source_point == 1)
        self.assertAlmostEqual(math.degrees(arrival.yaw_rad), 90.0, places=6)
        self.assertAlmostEqual(arrival.wz_radps, 0.0, places=9)
        self.assertLessEqual(
            result.summary.max_wz_radps,
            project.planner.max_wz_radps + 1e-9,
        )
        self.assertLessEqual(
            result.summary.max_beta_radps2,
            project.planner.angular_accel_moving_radps2 + 0.02,
        )


if __name__ == "__main__":
    unittest.main()
