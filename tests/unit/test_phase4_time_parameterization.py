from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.enums import NodeFlag
from hjmb_pathgen.py_planning.dynamics.time_parameterization import (
    GeometrySample,
    SpeedFailureCategory,
    TimeParameterizationLimits,
    TimeParameterizationRequest,
    samples_from_points,
    time_parameterize,
)


def limits(**overrides) -> TimeParameterizationLimits:
    values = {
        "max_speed_mmps": 2000.0,
        "linear_accel_mmps2": 1000.0,
        "braking_accel_mmps2": 1000.0,
        "lateral_accel_mmps2": 800.0,
        "max_wz_ddegps": 900.0,
        "angular_accel_moving_ddegps2": 1200.0,
        "wheel_radius_mm": 50.0,
        "wheel_rotation_radius_mm": 200.0,
        "wheel_plan_limit_rpm": 1000.0,
        "max_spacing_mm": 25.0,
    }
    values.update(overrides)
    return TimeParameterizationLimits(**values)


def request(samples, **limit_overrides) -> TimeParameterizationRequest:
    return TimeParameterizationRequest(samples=tuple(samples), limits=limits(**limit_overrides))


class Phase4TimeParameterizationTest(unittest.TestCase):
    def test_straight_high_candidate_speed_reduces_instead_of_failing(self):
        samples = samples_from_points(((0, 0, 0), (1000, 0, 0)))
        result = time_parameterize(request(samples, max_speed_mmps=1_000_000.0))
        self.assertTrue(result.success, result.reason)
        self.assertAlmostEqual(result.planned_time_ms, 2000, delta=80)
        self.assertEqual(result.nodes[0].vx_mmps, 0)
        self.assertEqual(result.nodes[-1].vx_mmps, 0)

    def test_multiple_arrivals_are_full_stop_boundaries(self):
        samples = (
            GeometrySample(0, 0, 0, 0, 1, 0, flags=int(NodeFlag.START)),
            GeometrySample(500, 500, 0, 0, 1, 0, flags=int(NodeFlag.ARRIVAL)),
            GeometrySample(1000, 1000, 0, 0, 1, 0, flags=int(NodeFlag.ARRIVAL)),
        )
        result = time_parameterize(request(samples))
        self.assertTrue(result.success, result.reason)
        arrival_nodes = [node for node in result.nodes if node.flags & int(NodeFlag.ARRIVAL)]
        self.assertEqual(len(arrival_nodes), 2)
        self.assertTrue(all(node.vx_mmps == 0 and node.vy_mmps == 0 and node.wz_ddegps == 0 for node in arrival_nodes))

    def test_curvature_lateral_accel_cap_slows_down(self):
        curvature = 0.01
        samples = (
            GeometrySample(0, 0, 0, 0, 1, 0, curvature_1_per_mm=curvature, flags=int(NodeFlag.START)),
            GeometrySample(500, 500, 0, 0, 1, 0, curvature_1_per_mm=curvature),
            GeometrySample(1000, 1000, 0, 0, 1, 0, curvature_1_per_mm=curvature, flags=int(NodeFlag.ARRIVAL)),
        )
        result = time_parameterize(request(samples, max_speed_mmps=2000.0))
        self.assertTrue(result.success, result.reason)
        self.assertLessEqual(result.max_metrics["max_speed_mmps"], math.sqrt(800.0 / curvature) + 1.0)
        self.assertLessEqual(result.max_metrics["max_lateral_accel_mmps2"], 800.0 + 1.0e-6)

    def test_yaw_metrics_are_diagnostic_and_combined_wheel_rpm_is_enforced(self):
        yaw_samples = (
            GeometrySample(0, 0, 0, 0, 1, 0, yaw_ddeg_per_mm=1.0, flags=int(NodeFlag.START)),
            GeometrySample(500, 500, 0, 500, 1, 0, yaw_ddeg_per_mm=1.0),
            GeometrySample(1000, 1000, 0, 1000, 1, 0, yaw_ddeg_per_mm=1.0, flags=int(NodeFlag.ARRIVAL)),
        )
        # A deliberately tiny standalone wz limit must no longer slow the path.
        # The reported wz can exceed it; rotational feasibility is enforced by
        # the combined four-wheel RPM limit instead.
        yaw_result = time_parameterize(request(yaw_samples, max_wz_ddegps=100.0, wheel_plan_limit_rpm=1000.0))
        self.assertTrue(yaw_result.success, yaw_result.reason)
        self.assertGreater(yaw_result.max_metrics["max_wz_ddegps"], 100.0)

        wheel_result = time_parameterize(request(yaw_samples, max_wz_ddegps=10_000.0, wheel_plan_limit_rpm=60.0))
        self.assertTrue(wheel_result.success, wheel_result.reason)
        self.assertLessEqual(wheel_result.max_metrics["max_wheel_rpm"], 60.0 + 1.0e-6)

    def test_dynamic_margin_does_not_reduce_direct_wheel_rpm_limit(self):
        configured = limits(wheel_plan_limit_rpm=450.0, constraint_margin_ratio=0.10)
        validated = configured.validated()
        self.assertEqual(validated.wheel_plan_limit_rpm, 450.0)
        self.assertEqual(validated.max_speed_mmps, 1800.0)

    def test_zero_yaw_path_is_not_capped_by_angular_acceleration_setting(self):
        straight = samples_from_points(((0, 0, 0), (1000, 0, 0)))
        normal = time_parameterize(request(straight, angular_accel_moving_ddegps2=1200.0))
        tiny_beta_limit = time_parameterize(request(straight, angular_accel_moving_ddegps2=1.0))
        self.assertTrue(normal.success, normal.reason)
        self.assertTrue(tiny_beta_limit.success, tiny_beta_limit.reason)
        self.assertEqual(tiny_beta_limit.planned_time_ms, normal.planned_time_ms)

    def test_invalid_limits_and_structural_geometry_are_categorized(self):
        straight = samples_from_points(((0, 0, 0), (1000, 0, 0)))
        bad_limits = time_parameterize(request(straight, max_speed_mmps=0.0))
        self.assertFalse(bad_limits.success)
        self.assertEqual(bad_limits.failure_category, SpeedFailureCategory.INVALID_LIMITS)

        duplicate_s = (
            GeometrySample(0, 0, 0, 0, 1, 0, flags=int(NodeFlag.START)),
            GeometrySample(0, 0, 0, 0, 1, 0, flags=int(NodeFlag.ARRIVAL)),
        )
        bad_geometry = time_parameterize(request(duplicate_s))
        self.assertFalse(bad_geometry.success)
        self.assertEqual(bad_geometry.failure_category, SpeedFailureCategory.STRUCTURAL_GEOMETRY_ERROR)


if __name__ == "__main__":
    unittest.main()
