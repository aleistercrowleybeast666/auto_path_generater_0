# -*- coding: utf-8 -*-
import unittest

from path_geometry import generate_geometry
from path_models import (
    EditPoint,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
)
from v35_test_utils import make_curve_project, make_straight_project


class PathGeometryV35Test(unittest.TestCase):
    def test_start_must_be_first_and_last_must_be_arrival(self):
        project = make_straight_project()
        project.points[0].type = POINT_TYPE_WAYPOINT
        with self.assertRaisesRegex(ValueError, "START"):
            generate_geometry(project.points, project.planner)
        project = make_straight_project()
        project.points[-1].type = POINT_TYPE_WAYPOINT
        with self.assertRaisesRegex(ValueError, "最后一行必须是 ARRIVAL"):
            generate_geometry(project.points, project.planner)

    def test_waypoint_yaw_must_be_unspecified(self):
        project = make_curve_project()
        project.points[1].yaw_ddeg = 0
        with self.assertRaisesRegex(ValueError, "0xFF"):
            generate_geometry(project.points, project.planner)

    def test_exact_waypoint_is_kept_as_source(self):
        project = make_curve_project()
        project.points[1].exact_pass = True
        result = generate_geometry(project.points, project.planner)
        self.assertIn(1, {sample.source_point for sample in result.samples})

    def test_duplicate_point_id_is_rejected(self):
        project = make_straight_project()
        project.points.append(
            EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL, x_mm=3500, y_mm=0, yaw_ddeg=0)
        )
        with self.assertRaisesRegex(ValueError, "point_id"):
            generate_geometry(project.points, project.planner)


if __name__ == "__main__":
    unittest.main()
