# -*- coding: utf-8 -*-
import math
import unittest

from path_geometry import generate_geometry, validate_cut_in_straight
from path_models import (
    EditPoint,
    PathProject,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_CUT_IN,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)
from v33_test_utils import make_curve_project, make_straight_project


class PathGeometryV33Test(unittest.TestCase):
    def test_straight_length_and_curvature(self):
        project = make_straight_project(2000)
        geometry = generate_geometry(project.points, project.planner)
        self.assertAlmostEqual(geometry.samples[-1].s_mm, 2000.0, places=4)
        self.assertLess(
            max(abs(sample.curvature_kappa_per_m) for sample in geometry.samples),
            1e-7,
        )

    def test_rounded_waypoint_does_not_force_corner_pass(self):
        project = PathProject()
        project.points = [
            EditPoint(
                point_id=0,
                type=POINT_TYPE_CUT_IN,
                x_mm=0,
                y_mm=0,
                yaw_ddeg=0,
                exact_pass=True,
            ),
            EditPoint(
                point_id=1,
                type=POINT_TYPE_WAYPOINT,
                x_mm=500,
                y_mm=500,
                yaw_ddeg=YAW_UNSPECIFIED_DDEG,
                corner_trim_mm=250,
            ),
            EditPoint(
                point_id=2,
                type=POINT_TYPE_ARRIVAL,
                x_mm=1000,
                y_mm=0,
                yaw_ddeg=0,
                stop_required=True,
                is_end=True,
            ),
        ]
        geometry = generate_geometry(project.points, project.planner)
        closest = min(
            math.hypot(sample.x_mm - 500, sample.y_mm - 500)
            for sample in geometry.samples
        )
        self.assertGreater(closest, 10.0)
        self.assertNotIn(1, [sample.source_point for sample in geometry.samples])

    def test_arrival_and_exact_waypoint_are_export_samples(self):
        project = make_curve_project()
        project.points[1].exact_pass = True
        geometry = generate_geometry(project.points, project.planner)
        source_points = {sample.source_point for sample in geometry.samples}
        self.assertIn(1, source_points)
        self.assertIn(3, source_points)
        exact = next(sample for sample in geometry.samples if sample.source_point == 1)
        self.assertAlmostEqual(exact.x_mm, project.points[1].x_mm)
        self.assertAlmostEqual(exact.y_mm, project.points[1].y_mm)

    def test_arc_length_is_monotonic_and_spacing_is_bounded(self):
        project = make_curve_project()
        geometry = generate_geometry(project.points, project.planner)
        distances = [
            current.s_mm - previous.s_mm
            for previous, current in zip(geometry.samples[:-1], geometry.samples[1:])
        ]
        self.assertTrue(all(distance > 0 for distance in distances))
        self.assertLessEqual(max(distances), project.planner.max_spacing_mm + 1e-6)
        self.assertGreater(
            max(abs(sample.curvature_kappa_per_m) for sample in geometry.samples),
            0.05,
        )

    def test_cut_in_straight_validation_rejects_early_turn(self):
        project = make_curve_project()
        project.cut_in.straight_length_mm = 900
        geometry = generate_geometry(project.points, project.planner)
        errors = validate_cut_in_straight(
            geometry, project.points, project.cut_in.straight_length_mm
        )
        self.assertTrue(errors)

    def test_waypoint_yaw_must_be_unspecified(self):
        project = make_curve_project()
        project.points[1].yaw_ddeg = 0
        with self.assertRaisesRegex(ValueError, "0xFF"):
            generate_geometry(project.points, project.planner)


if __name__ == "__main__":
    unittest.main()
