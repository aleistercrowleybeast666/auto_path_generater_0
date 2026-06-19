"""Continuous trajectory collision validation by conservative subdivision."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.collision.obstacles import CollisionWorld
from hjmb_pathgen.collision.transforms import DDEG_TO_RAD
from hjmb_pathgen.collision.validator import check_pose_collision
from hjmb_pathgen.models.collision import CollisionContact, CollisionStatus, PathCollisionResult, RobotPose


@dataclass(frozen=True)
class ContinuousCheckOptions:
    resolution_mm: float
    max_depth: int = 24
    min_interval_s_mm: float = 1.0e-6
    max_checked_poses: int = 200_000
    max_recorded_collisions: int = 256
    collect_all: bool = True


@dataclass(frozen=True)
class SpatialPoseSample:
    s_mm: float
    x_mm: float
    y_mm: float
    yaw_ddeg: float

    def to_pose(self) -> RobotPose:
        return RobotPose(x_mm=self.x_mm, y_mm=self.y_mm, yaw_ddeg=self.yaw_ddeg, s_mm=self.s_mm)

    def to_dict(self) -> dict[str, float]:
        return {
            "s_mm": self.s_mm,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "yaw_ddeg": self.yaw_ddeg,
        }


@dataclass
class _Accumulator:
    checked_pose_count: int = 0
    subdivision_count: int = 0
    min_contact: CollisionContact | None = None
    first_collision: CollisionContact | None = None
    collisions: list[CollisionContact] | None = None
    errors: list[str] | None = None

    def __post_init__(self) -> None:
        if self.collisions is None:
            self.collisions = []
        if self.errors is None:
            self.errors = []


def validate_spatial_samples_continuous(
    samples: tuple[object, ...],
    world: CollisionWorld,
    options: ContinuousCheckOptions,
    *,
    path_hash: str | None = None,
) -> PathCollisionResult:
    started = time.perf_counter()
    normalized = tuple(_sample_from_object(sample) for sample in samples)
    computed_path_hash = path_hash or canonical_json_crc32_hex([sample.to_dict() for sample in normalized])
    config_hash = world.collision_config_hash
    if len(normalized) < 2:
        return _result(
            status=CollisionStatus.NO_GEOMETRY,
            world=world,
            options=options,
            path_hash=computed_path_hash,
            started=started,
            acc=_Accumulator(errors=["at least two spatial samples are required"]),
        )
    try:
        _validate_options(options)
        _validate_samples(normalized)
        acc = _Accumulator()
        _check_sample(normalized[0], 0, world, options, acc, "node")
        for segment_index, (left, right) in enumerate(zip(normalized, normalized[1:])):
            _check_interval(left, right, segment_index, world, options, acc, depth=0)
            _check_sample(right, segment_index + 1, world, options, acc, "node")
            if acc.errors and not options.collect_all:
                break
        status = CollisionStatus.NUMERICAL_ERROR if acc.errors else (
            CollisionStatus.FAILED if acc.first_collision else CollisionStatus.PASSED
        )
        return _result(status=status, world=world, options=options, path_hash=computed_path_hash, started=started, acc=acc)
    except ValueError as exc:
        return _result(
            status=CollisionStatus.NUMERICAL_ERROR,
            world=world,
            options=options,
            path_hash=computed_path_hash,
            started=started,
            acc=_Accumulator(errors=[str(exc)]),
        )
    finally:
        del config_hash


def _check_interval(
    left: SpatialPoseSample,
    right: SpatialPoseSample,
    segment_index: int,
    world: CollisionWorld,
    options: ContinuousCheckOptions,
    acc: _Accumulator,
    *,
    depth: int,
) -> None:
    if acc.errors and not options.collect_all:
        return
    motion_bound = math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm)
    motion_bound += world.r_large_mm * abs((right.yaw_ddeg - left.yaw_ddeg) * DDEG_TO_RAD)
    if motion_bound <= options.resolution_mm:
        return
    if depth >= options.max_depth:
        acc.errors.append(f"continuous collision recursion depth exceeded at segment {segment_index}")
        return
    if (
        abs(right.s_mm - left.s_mm) <= options.min_interval_s_mm
        and math.hypot(right.x_mm - left.x_mm, right.y_mm - left.y_mm) <= 1.0e-12
        and abs(right.yaw_ddeg - left.yaw_ddeg) <= 1.0e-12
    ):
        acc.errors.append(f"continuous collision minimum interval exceeded at segment {segment_index}")
        return
    middle = _interpolate(left, right, 0.5)
    acc.subdivision_count += 1
    _check_sample(middle, segment_index, world, options, acc, "midpoint")
    _check_interval(left, middle, segment_index, world, options, acc, depth=depth + 1)
    _check_interval(middle, right, segment_index, world, options, acc, depth=depth + 1)


def _check_sample(
    sample: SpatialPoseSample,
    index: int,
    world: CollisionWorld,
    options: ContinuousCheckOptions,
    acc: _Accumulator,
    source_kind: str,
) -> None:
    if acc.checked_pose_count >= options.max_checked_poses:
        acc.errors.append("continuous collision checked-pose limit exceeded")
        return
    result = check_pose_collision(
        sample.to_pose(),
        world,
        {},
        collect_all=options.collect_all,
        source={"kind": source_kind, "index": index, "s_mm": sample.s_mm},
    )
    acc.checked_pose_count += 1
    if result.contacts:
        closest = min(result.contacts, key=lambda contact: contact.signed_clearance_mm)
        if acc.min_contact is None or closest.signed_clearance_mm < acc.min_contact.signed_clearance_mm:
            acc.min_contact = closest
    if result.violations:
        if acc.first_collision is None:
            acc.first_collision = result.violations[0]
        assert acc.collisions is not None
        for violation in result.violations:
            if len(acc.collisions) < options.max_recorded_collisions:
                acc.collisions.append(violation)


def _interpolate(left: SpatialPoseSample, right: SpatialPoseSample, ratio: float) -> SpatialPoseSample:
    return SpatialPoseSample(
        s_mm=_lerp(left.s_mm, right.s_mm, ratio),
        x_mm=_lerp(left.x_mm, right.x_mm, ratio),
        y_mm=_lerp(left.y_mm, right.y_mm, ratio),
        yaw_ddeg=_lerp(left.yaw_ddeg, right.yaw_ddeg, ratio),
    )


def _sample_from_object(value: object) -> SpatialPoseSample:
    try:
        s_mm = float(getattr(value, "s_mm"))
        x_mm = float(getattr(value, "x_mm"))
        y_mm = float(getattr(value, "y_mm"))
        yaw_ddeg = float(getattr(value, "yaw_ddeg"))
    except AttributeError:
        if not isinstance(value, dict):
            raise ValueError(f"spatial sample must be object or dict, got {type(value).__name__}")
        s_mm = float(value.get("s_mm", value.get("local_s_mm")))
        x_mm = float(value["x_mm"])
        y_mm = float(value["y_mm"])
        yaw_ddeg = float(value["yaw_ddeg"])
    return SpatialPoseSample(s_mm=s_mm, x_mm=x_mm, y_mm=y_mm, yaw_ddeg=yaw_ddeg)


def _validate_options(options: ContinuousCheckOptions) -> None:
    if not math.isfinite(options.resolution_mm) or options.resolution_mm <= 0.0:
        raise ValueError("collision resolution must be positive")
    if options.max_depth <= 0 or options.max_checked_poses <= 0:
        raise ValueError("continuous collision limits must be positive")


def _validate_samples(samples: tuple[SpatialPoseSample, ...]) -> None:
    previous_s = samples[0].s_mm
    for index, sample in enumerate(samples):
        values = (sample.s_mm, sample.x_mm, sample.y_mm, sample.yaw_ddeg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"sample {index} contains non-finite values")
        if index > 0 and sample.s_mm < previous_s:
            raise ValueError(f"samples must be monotonically increasing at index {index}")
        previous_s = sample.s_mm


def _result(
    *,
    status: CollisionStatus,
    world: CollisionWorld,
    options: ContinuousCheckOptions,
    path_hash: str,
    started: float,
    acc: _Accumulator,
) -> PathCollisionResult:
    errors = tuple(acc.errors or ())
    collisions = tuple(acc.collisions or ())
    min_contact = acc.min_contact
    return PathCollisionResult(
        status=status,
        checked_config_hash=world.collision_config_hash,
        checked_path_hash=path_hash,
        validation_resolution_mm=options.resolution_mm,
        min_clearance_mm=min_contact.signed_clearance_mm if min_contact else None,
        min_clearance_pose=min_contact.pose if min_contact else None,
        min_clearance_obstacle=min_contact.obstacle_id if min_contact else None,
        collision_count=len(collisions),
        first_collision=acc.first_collision,
        collisions=collisions,
        checked_pose_count=acc.checked_pose_count,
        subdivision_count=acc.subdivision_count,
        elapsed_ms=(time.perf_counter() - started) * 1000.0,
        warnings=(),
        errors=errors,
    )


def _lerp(left: float, right: float, ratio: float) -> float:
    return left + (right - left) * ratio
