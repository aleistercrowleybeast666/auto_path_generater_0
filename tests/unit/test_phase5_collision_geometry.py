from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.collision.circle_rect import circle_rect_signed_distance
from hjmb_pathgen.collision.diagnostics import classify_clearance
from hjmb_pathgen.collision.footprints import clipped_disk_body_vertices, clipped_disk_chord_half_height, clipped_disk_world_vertices, is_convex_ccw
from hjmb_pathgen.collision.primitives import Circle, OrientedRect, Point2
from hjmb_pathgen.collision.transforms import body_to_world, rect_vertices, world_to_body
from hjmb_pathgen.collision.validator import check_pose_collision
from hjmb_pathgen.models.collision import ClearanceClass, CollisionStatus, RobotPose
from hjmb_pathgen.models.errors import V40ValidationError
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.services.collision_config_service import build_collision_world

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def project_dict() -> dict:
    return json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))


def project_with_objects(**updates) -> ProjectV40:
    data = project_dict()
    for key, value in updates.items():
        data[key] = value
    return ProjectV40.from_dict(data)


class Phase5CollisionGeometryTest(unittest.TestCase):
    def test_transform_round_trip_and_yaw_equivalence(self):
        origin = Point2(10, -20)
        body = Point2(30, 40)
        for yaw in (0, 900, 1800, -900, 4500):
            world = body_to_world(body, origin, yaw)
            round_trip = world_to_body(world, origin, yaw)
            self.assertAlmostEqual(round_trip.x, body.x, places=9)
            self.assertAlmostEqual(round_trip.y, body.y, places=9)
        p0 = body_to_world(body, origin, 900)
        p360 = body_to_world(body, origin, 4500)
        self.assertAlmostEqual(p0.x, p360.x, places=9)
        self.assertAlmostEqual(p0.y, p360.y, places=9)
        with self.assertRaises(ValueError):
            Point2(float("nan"), 0)

    def test_clipped_disk_shape_and_rotation(self):
        r_large = 120.0
        r_small = 70.0
        half_height = clipped_disk_chord_half_height(r_large, r_small)
        vertices = clipped_disk_body_vertices(r_large, r_small, 16)
        self.assertTrue(is_convex_ccw(vertices))
        self.assertAlmostEqual(vertices[0].x, r_small, places=9)
        self.assertAlmostEqual(vertices[0].y, half_height, places=9)
        self.assertAlmostEqual(vertices[-1].x, r_small, places=9)
        self.assertAlmostEqual(vertices[-1].y, -half_height, places=9)
        for vertex in vertices:
            self.assertLessEqual(vertex.x, r_small + 1.0e-9)
            self.assertLessEqual(math.hypot(vertex.x, vertex.y), r_large + 1.0e-9)
        self.assertFalse(any(vertex.x > r_small for vertex in vertices))
        world = clipped_disk_world_vertices(Point2(0, 0), 900, r_large, r_small, 16)
        self.assertAlmostEqual(world[0].x, -half_height, places=9)
        self.assertAlmostEqual(world[0].y, r_small, places=9)
        with self.assertRaises(ValueError):
            clipped_disk_body_vertices(100, 100, 16)
        with self.assertRaises(ValueError):
            clipped_disk_body_vertices(120, 70, 2)

    def test_circle_rect_tangency_and_inside_penetration(self):
        rect = OrientedRect(Point2(0, 0), 100, 100, 300)
        tangent = circle_rect_signed_distance(Circle(body_to_world(Point2(60, 0), rect.center, rect.yaw_ddeg), 10), rect)
        self.assertAlmostEqual(tangent.signed_clearance_mm, 0.0, places=7)
        corner_local = Point2(50 + 10 / math.sqrt(2), 50 + 10 / math.sqrt(2))
        corner = circle_rect_signed_distance(Circle(body_to_world(corner_local, rect.center, rect.yaw_ddeg), 10), rect)
        self.assertAlmostEqual(corner.signed_clearance_mm, 0.0, places=7)
        inside = circle_rect_signed_distance(Circle(Point2(0, 0), 10), rect)
        self.assertLess(inside.signed_clearance_mm, 0)
        self.assertEqual(inside.feature, "INTERIOR")

    def test_project_collision_schema_rejects_missing_phase5_fields(self):
        data = project_dict()
        del data["field_objects"]["cylinders"]
        with self.assertRaisesRegex(V40ValidationError, "required"):
            ProjectV40.from_dict(data)
        data = project_dict()
        del data["vehicle"]["footprint"]["strict_validation_resolution_mm"]
        with self.assertRaisesRegex(V40ValidationError, "Phase 5"):
            ProjectV40.from_dict(data)

    def test_discrete_obstacle_mapping_and_tangency(self):
        world = build_collision_world(project_with_objects())
        self.assertEqual(len(world.cylinders), 2)
        self.assertEqual([box.physical_site for box in world.pickup_boxes], ["PICK_1", "PICK_2", "PICK_3"])
        pose = RobotPose(x_mm=-1029, y_mm=350, yaw_ddeg=0)
        result = check_pose_collision(pose, world, {})
        cylinder = next(contact for contact in result.contacts if contact.obstacle_id == "CYLINDER_1")
        self.assertEqual(cylinder.clearance_class, ClearanceClass.TOUCHING)
        self.assertTrue(result.is_valid)

    def test_pickup_clipped_disk_semantics_and_yaw_sensitivity(self):
        data = project_dict()
        for group in ("cylinders", "drop_boxes"):
            for item in data["field_objects"][group]:
                item["enabled"] = False
        data["field_objects"]["pickup_boxes"] = [
            {"obstacle_id": "PICKUP_BOX_1", "physical_pick_site": "PICK_1", "center_x_mm": 105, "center_y_mm": 0, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": True},
            {"obstacle_id": "PICKUP_BOX_2", "physical_pick_site": "PICK_2", "center_x_mm": 1800, "center_y_mm": 700, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": False},
            {"obstacle_id": "PICKUP_BOX_3", "physical_pick_site": "PICK_3", "center_x_mm": 1800, "center_y_mm": 800, "length_mm": 20, "width_mm": 20, "yaw_ddeg": 0, "configured": True, "enabled": False},
        ]
        world = build_collision_world(ProjectV40.from_dict(data))
        legal_front_cap = check_pose_collision(RobotPose(0, 0, 0), world, {})
        self.assertTrue(legal_front_cap.is_valid, legal_front_cap.to_dict())
        yaw_collision = check_pose_collision(RobotPose(0, 0, 900), world, {})
        self.assertFalse(yaw_collision.is_valid)
        self.assertEqual(yaw_collision.violations[0].obstacle_type.value, "PICKUP_BOX")

    def test_field_boundary_uses_nominal_large_circle(self):
        world = build_collision_world(project_with_objects())
        touching = check_pose_collision(RobotPose(1880, 0, 0), world, {})
        boundary = next(contact for contact in touching.contacts if contact.obstacle_id == "FIELD_BOUNDARY")
        self.assertEqual(boundary.clearance_class, ClearanceClass.TOUCHING)
        penetrating = check_pose_collision(RobotPose(1880.001, 0, 0), world, {})
        boundary = next(contact for contact in penetrating.contacts if contact.obstacle_id == "FIELD_BOUNDARY")
        self.assertEqual(boundary.clearance_class, ClearanceClass.PENETRATING)

    def test_clearance_classification_is_shared(self):
        self.assertEqual(classify_clearance(0.1, 0.01), ClearanceClass.CLEAR)
        self.assertEqual(classify_clearance(0.0, 0.01), ClearanceClass.TOUCHING)
        self.assertEqual(classify_clearance(-0.1, 0.01), ClearanceClass.PENETRATING)


if __name__ == "__main__":
    unittest.main()
