from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_planning.geometry.bezier import BezierPath, Point2D
from hjmb_pathgen.py_planning.geometry.topology_gates import validate_ordered_topology_gates
from hjmb_pathgen.py_domain.enums import YawPolicy
from hjmb_pathgen.py_domain.topology import TopologyGate, TopologyGateDirection
from hjmb_pathgen.py_planning.optimization.yaw_windows import YawWindowProfile, resolve_yaw_delta


class Phase6GeometryYawTopologyTest(unittest.TestCase):
    def test_bezier_path_is_c1_and_arc_length_sampled(self):
        path = BezierPath.from_waypoints((Point2D(0, 0), Point2D(100, 50), Point2D(200, 0)))
        samples = path.sample_arclength(max_spacing_mm=20)
        self.assertGreater(len(samples), 5)
        self.assertAlmostEqual(samples[0].x_mm, 0.0, places=6)
        self.assertAlmostEqual(samples[-1].x_mm, 200.0, places=6)
        self.assertTrue(all(right.s_mm > left.s_mm for left, right in zip(samples, samples[1:])))
        self.assertTrue(all(math.isfinite(sample.curvature_1_per_mm) for sample in samples))

        first_end = path.segments[0].tangent(1.0)
        second_start = path.segments[1].tangent(0.0)
        self.assertAlmostEqual(first_end[0], second_start[0], places=6)
        self.assertAlmostEqual(first_end[1], second_start[1], places=6)

    def test_two_low_speed_yaw_windows_are_policy_aware(self):
        self.assertEqual(resolve_yaw_delta(0, 900, YawPolicy.CW_ONLY), -2700)
        self.assertEqual(resolve_yaw_delta(0, -900, YawPolicy.CCW_ONLY), 2700)
        self.assertEqual(resolve_yaw_delta(0, 1900, YawPolicy.SHORTEST), -1700)

        profile = YawWindowProfile(0, 900, policy=YawPolicy.CCW_ONLY, alpha=0.5)
        values = [profile.evaluate(s, 100.0).yaw_ddeg for s in (0.0, 25.0, 50.0, 75.0, 100.0)]
        self.assertEqual(values[0], 0)
        self.assertAlmostEqual(values[-1], 900)
        self.assertTrue(all(right >= left for left, right in zip(values, values[1:])))
        self.assertAlmostEqual(profile.evaluate(50.0, 100.0).yaw_ddeg_per_mm, 0.0)

    def test_ordered_topology_gate_direction(self):
        points = [{"x_mm": 0, "y_mm": 0}, {"x_mm": 100, "y_mm": 0}]
        gate = TopologyGate("G1", 50, -20, 50, 20, TopologyGateDirection.NEGATIVE)
        result = validate_ordered_topology_gates(points, (gate,))
        self.assertTrue(result.success)
        self.assertEqual(result.crossings[0].gate_id, "G1")

        wrong = TopologyGate("G1", 50, -20, 50, 20, TopologyGateDirection.POSITIVE)
        self.assertFalse(validate_ordered_topology_gates(points, (wrong,)).success)

    def test_topology_gate_order_uses_same_segment_ratio(self):
        points = [{"x_mm": 0, "y_mm": 0}, {"x_mm": 100, "y_mm": 0}]
        early = TopologyGate("G20", 20, -20, 20, 20, TopologyGateDirection.NEGATIVE)
        late = TopologyGate("G80", 80, -20, 80, 20, TopologyGateDirection.NEGATIVE)

        ordered = validate_ordered_topology_gates(points, (early, late))
        self.assertTrue(ordered.success, ordered.to_dict())
        self.assertLess(ordered.crossings[0].global_path_parameter, ordered.crossings[1].global_path_parameter)

        reversed_order = validate_ordered_topology_gates(points, (late, early))
        self.assertFalse(reversed_order.success)
        self.assertIn("G20", " ".join(reversed_order.errors))


if __name__ == "__main__":
    unittest.main()
