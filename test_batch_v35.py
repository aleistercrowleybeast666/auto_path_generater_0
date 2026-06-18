# -*- coding: utf-8 -*-
import unittest

from batch_generator import assemble_route_case
from batch_models import LegTemplate, RouteCase, validate_route_case_coverage
from path_models import POINT_TYPE_WAYPOINT
from v35_test_utils import make_straight_project


class BatchV35Test(unittest.TestCase):
    def test_route_case_duplicate_and_missing_detection(self):
        cases = [RouteCase(traj_id=0), RouteCase(traj_id=0)]
        errors = validate_route_case_coverage(cases, require_full_360=False)
        self.assertTrue(any("重复" in error for error in errors))
        errors = validate_route_case_coverage([RouteCase(traj_id=0)], require_full_360=True)
        self.assertTrue(any("缺少" in error for error in errors))

    def test_leg_template_direction_is_not_reversed(self):
        common = make_straight_project()
        template = LegTemplate(
            from_site_id=0,
            to_site_id=1,
            waypoints=[{"x_mm": 100, "y_mm": 50}],
        )
        with self.assertRaisesRegex(ValueError, "缺少有向路段模板"):
            assemble_route_case(common, RouteCase(traj_id=1, pickup_order=[0]), {(1, 0): template})

    def test_assembled_cases_do_not_share_mutable_lists(self):
        common = make_straight_project()
        template = LegTemplate(
            from_site_id=0,
            to_site_id=1,
            waypoints=[{"x_mm": 100, "y_mm": 50}],
        )
        first = assemble_route_case(common, RouteCase(traj_id=1, pickup_order=[1]), {(0, 1): template})
        second = assemble_route_case(common, RouteCase(traj_id=2, pickup_order=[1]), {(0, 1): template})
        first.points[1].x_mm = 999
        self.assertNotEqual(first.points[1].x_mm, second.points[1].x_mm)
        self.assertEqual(first.points[1].type, POINT_TYPE_WAYPOINT)


if __name__ == "__main__":
    unittest.main()
