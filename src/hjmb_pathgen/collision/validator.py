"""Discrete-pose collision validation."""

from __future__ import annotations

import math
from typing import Any

from hjmb_pathgen.collision.circle_rect import circle_rect_signed_distance
from hjmb_pathgen.collision.convex_sat import convex_polygon_signed_distance
from hjmb_pathgen.collision.diagnostics import classify_clearance
from hjmb_pathgen.collision.footprints import clipped_disk_world_vertices
from hjmb_pathgen.collision.obstacles import CollisionWorld, CylinderObstacle, FieldBoundary, RectObstacle
from hjmb_pathgen.collision.primitives import Circle, Point2
from hjmb_pathgen.collision.transforms import rect_vertices
from hjmb_pathgen.models.collision import (
    CollisionContact,
    FootprintProfile,
    ObstacleType,
    PoseCollisionResult,
    RobotPose,
)


def check_pose_collision(
    pose: RobotPose,
    collision_world: CollisionWorld,
    context: dict[str, Any] | None = None,
    *,
    collect_all: bool = True,
    source: dict[str, Any] | None = None,
) -> PoseCollisionResult:
    del context
    source = dict(source or {})
    origin = Point2(pose.x_mm, pose.y_mm)
    contacts: list[CollisionContact] = []
    pickup_footprint: tuple[Point2, ...] | None = None
    for obstacle in collision_world.obstacles:
        if isinstance(obstacle, FieldBoundary):
            contact = _check_field_boundary(pose, obstacle, collision_world, source)
        elif isinstance(obstacle, CylinderObstacle):
            contact = _check_cylinder(pose, obstacle, collision_world, source)
        elif isinstance(obstacle, RectObstacle) and obstacle.obstacle_type == ObstacleType.DROP_BOX:
            contact = _check_drop_box(pose, origin, obstacle, collision_world, source)
        elif isinstance(obstacle, RectObstacle) and obstacle.obstacle_type == ObstacleType.PICKUP_BOX:
            if pickup_footprint is None:
                pickup_footprint = clipped_disk_world_vertices(origin, pose.yaw_ddeg, collision_world.r_large_mm, collision_world.r_small_mm, collision_world.pickup_arc_segments)
            contact = _check_pickup_box(pose, pickup_footprint, obstacle, collision_world, source)
        else:
            raise ValueError(f"unsupported obstacle type: {obstacle!r}")
        contacts.append(contact)
        if contact.is_collision and not collect_all:
            break
    if contacts:
        closest = min(contacts, key=lambda item: item.signed_clearance_mm)
        min_clearance = closest.signed_clearance_mm
        closest_id = closest.obstacle_id
    else:
        min_clearance = math.inf
        closest_id = None
    violations = tuple(contact for contact in contacts if contact.is_collision)
    return PoseCollisionResult(
        is_valid=not violations,
        pose=pose,
        min_signed_clearance_mm=min_clearance,
        closest_obstacle_id=closest_id,
        contacts=tuple(contacts),
        violations=violations,
        numerical_epsilon_mm=collision_world.numerical_epsilon_mm,
    )


def _check_field_boundary(
    pose: RobotPose,
    boundary: FieldBoundary,
    world: CollisionWorld,
    source: dict[str, Any],
) -> CollisionContact:
    del boundary
    margins = {
        "LEFT": pose.x_mm - world.r_large_mm - world.field_boundary.x_min_mm,
        "RIGHT": world.field_boundary.x_max_mm - (pose.x_mm + world.r_large_mm),
        "BOTTOM": pose.y_mm - world.r_large_mm - world.field_boundary.y_min_mm,
        "TOP": world.field_boundary.y_max_mm - (pose.y_mm + world.r_large_mm),
    }
    side = min(margins, key=margins.get)
    clearance = float(margins[side])
    return _contact(
        pose=pose,
        obstacle_id=world.field_boundary.obstacle_id,
        obstacle_type=ObstacleType.FIELD_BOUNDARY,
        footprint_profile=FootprintProfile.LARGE_CIRCLE,
        clearance=clearance,
        epsilon=world.numerical_epsilon_mm,
        source=source,
        diagnostic={"boundary_side": side, "distance_semantics": "large_circle_inside_nominal_field"},
    )


def _check_cylinder(
    pose: RobotPose,
    obstacle: CylinderObstacle,
    world: CollisionWorld,
    source: dict[str, Any],
) -> CollisionContact:
    dx = pose.x_mm - obstacle.center.x
    dy = pose.y_mm - obstacle.center.y
    center_distance = math.hypot(dx, dy)
    clearance = center_distance - (world.r_large_mm + obstacle.radius_mm)
    if center_distance > 1.0e-12:
        normal = {"x": dx / center_distance, "y": dy / center_distance}
    else:
        normal = {"x": 1.0, "y": 0.0}
    return _contact(
        pose=pose,
        obstacle_id=obstacle.obstacle_id,
        obstacle_type=ObstacleType.CYLINDER,
        footprint_profile=FootprintProfile.LARGE_CIRCLE,
        clearance=clearance,
        epsilon=world.numerical_epsilon_mm,
        source=source,
        diagnostic={
            "center_distance_mm": center_distance,
            "normal": normal,
            "distance_semantics": "exact_circle_circle",
        },
    )


def _check_drop_box(
    pose: RobotPose,
    origin: Point2,
    obstacle: RectObstacle,
    world: CollisionWorld,
    source: dict[str, Any],
) -> CollisionContact:
    distance = circle_rect_signed_distance(Circle(origin, world.r_small_mm), obstacle.rect)
    return _contact(
        pose=pose,
        obstacle_id=obstacle.obstacle_id,
        obstacle_type=ObstacleType.DROP_BOX,
        footprint_profile=FootprintProfile.SMALL_CIRCLE,
        clearance=distance.signed_clearance_mm,
        epsilon=world.numerical_epsilon_mm,
        source=source,
        diagnostic={
            "closest_world": distance.closest_world.to_tuple(),
            "rect_feature": distance.feature,
            "physical_site": obstacle.physical_site,
            "distance_semantics": "exact_circle_oriented_rect",
        },
    )


def _check_pickup_box(
    pose: RobotPose,
    footprint: tuple[Point2, ...],
    obstacle: RectObstacle,
    world: CollisionWorld,
    source: dict[str, Any],
) -> CollisionContact:
    rect = rect_vertices(obstacle.rect)
    distance = convex_polygon_signed_distance(footprint, rect, epsilon=world.numerical_epsilon_mm)
    return _contact(
        pose=pose,
        obstacle_id=obstacle.obstacle_id,
        obstacle_type=ObstacleType.PICKUP_BOX,
        footprint_profile=FootprintProfile.PICKUP_CLIPPED_DISK,
        clearance=distance.signed_clearance_mm,
        epsilon=world.numerical_epsilon_mm,
        source=source,
        diagnostic={
            "physical_site": obstacle.physical_site,
            "intersects": distance.intersects,
            "min_overlap_mm": distance.min_overlap_mm,
            "distance_semantics": "sat_intersection_exact_segment_distance_when_separated",
        },
    )


def _contact(
    *,
    pose: RobotPose,
    obstacle_id: str,
    obstacle_type: ObstacleType,
    footprint_profile: FootprintProfile,
    clearance: float,
    epsilon: float,
    source: dict[str, Any],
    diagnostic: dict[str, Any],
) -> CollisionContact:
    clearance_class = classify_clearance(clearance, epsilon)
    return CollisionContact(
        obstacle_id=obstacle_id,
        obstacle_type=obstacle_type,
        footprint_profile=footprint_profile,
        clearance_class=clearance_class,
        signed_clearance_mm=clearance,
        penetration_depth_mm=max(0.0, -clearance),
        pose=pose,
        source=source,
        diagnostic=diagnostic,
    )
