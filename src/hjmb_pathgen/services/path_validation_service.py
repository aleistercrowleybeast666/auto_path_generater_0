"""Phase 5 path/case/leg collision validation services."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.collision.continuous import ContinuousCheckOptions, validate_spatial_samples_continuous
from hjmb_pathgen.models.collision import CollisionStatus, PathCollisionResult
from hjmb_pathgen.models.errors import CompileError
from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.models.leg import LegV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.models.route_case import CaseManifestV40
from hjmb_pathgen.services.atomic_writer import atomic_write_bytes
from hjmb_pathgen.services.collision_config_service import build_collision_world
from hjmb_pathgen.services.manual_path_service import build_manual_spatial_path
from hjmb_pathgen.planning.time_parameterization import GeometrySample


def validate_spatial_path_collision(
    samples: tuple[object, ...],
    project: ProjectV40,
    *,
    strict: bool = True,
    report_path: str | Path | None = None,
) -> PathCollisionResult:
    world = build_collision_world(project)
    resolution = world.strict_validation_resolution_mm if strict else world.collision_resolution_mm
    path_hash = spatial_path_hash(samples)
    result = validate_spatial_samples_continuous(
        samples,
        world,
        ContinuousCheckOptions(resolution_mm=resolution),
        path_hash=path_hash,
    )
    if report_path is not None:
        write_collision_report(report_path, result, case_or_leg_id=None)
    return result


def validate_time_parameterized_trajectory(
    trajectory_or_samples: object,
    project: ProjectV40,
    *,
    strict: bool = True,
    report_path: str | Path | None = None,
) -> PathCollisionResult:
    samples = tuple(getattr(trajectory_or_samples, "samples", trajectory_or_samples))
    return validate_spatial_path_collision(samples, project, strict=strict, report_path=report_path)


def validate_case_collision(
    case: CaseManifestV40,
    project: ProjectV40,
    *,
    samples: tuple[object, ...] | None = None,
    strict: bool = True,
    report_path: str | Path | None = None,
) -> PathCollisionResult:
    if samples is None:
        samples = _case_samples(case)
    if not samples:
        return _no_geometry_result(project, case_hash=canonical_json_crc32_hex(case.to_dict()))
    result = validate_spatial_path_collision(samples, project, strict=strict, report_path=None)
    if report_path is not None:
        write_collision_report(report_path, result, case_or_leg_id=f"P{case.traj_id:04d}")
    return result


def validate_leg_collision(
    leg: LegV40,
    project: ProjectV40,
    *,
    strict: bool = True,
    report_path: str | Path | None = None,
) -> PathCollisionResult:
    samples = _leg_samples(leg)
    if not samples:
        return _no_geometry_result(project, case_hash=canonical_json_crc32_hex(leg.to_dict()))
    result = validate_spatial_path_collision(samples, project, strict=strict, report_path=None)
    if report_path is not None:
        write_collision_report(report_path, result, case_or_leg_id=leg.leg_id)
    return result


def case_with_collision_result(case: CaseManifestV40, result: PathCollisionResult) -> CaseManifestV40:
    hashes = dict(case.hashes)
    hashes["collision_config_hash"] = result.checked_config_hash
    hashes["path_geometry_hash"] = result.checked_path_hash
    review = dict(case.review)
    review["collision_status"] = result.status.value
    review["collision_min_clearance_mm"] = result.min_clearance_mm
    review["collision_checked_config_hash"] = result.checked_config_hash
    review["collision_checked_path_hash"] = result.checked_path_hash
    if result.status != CollisionStatus.PASSED:
        review["approved"] = False
    return replace(case, hashes=hashes, review=review)


def collision_result_is_stale(result: PathCollisionResult, samples: tuple[object, ...], project: ProjectV40) -> bool:
    world = build_collision_world(project)
    return result.checked_config_hash != world.collision_config_hash or result.checked_path_hash != spatial_path_hash(samples)


def spatial_path_hash(samples: tuple[object, ...]) -> str:
    payload = []
    for sample in samples:
        s_mm, x_mm, y_mm, yaw_ddeg = _sample_values(sample)
        payload.append(
            {
                "s_mm": s_mm,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "yaw_ddeg": yaw_ddeg,
            }
        )
    return canonical_json_crc32_hex(payload)


def _sample_values(sample: object) -> tuple[float, float, float, float]:
    if isinstance(sample, dict):
        return (
            float(sample.get("s_mm", sample.get("local_s_mm", 0.0))),
            float(sample["x_mm"]),
            float(sample["y_mm"]),
            float(sample["yaw_ddeg"]),
        )
    return (
        float(getattr(sample, "s_mm")),
        float(getattr(sample, "x_mm")),
        float(getattr(sample, "y_mm")),
        float(getattr(sample, "yaw_ddeg")),
    )


def write_collision_report(path: str | Path, result: PathCollisionResult, *, case_or_leg_id: str | None) -> None:
    report = {
        "format": "HJMB_COLLISION_REPORT_JSON_V40",
        "case_or_leg_id": case_or_leg_id,
        "result": result.to_dict(),
    }
    data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        loaded = json.loads(temp_path.read_text(encoding="utf-8"))
        if loaded != report:
            raise CompileError(f"collision report write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)


def _case_samples(case: CaseManifestV40) -> tuple[object, ...]:
    if case.path_source == PathSource.MANUAL_FREE:
        if case.manual_path is None:
            raise CompileError("MANUAL_FREE case has no manual_path")
        return build_manual_spatial_path(case.manual_path)
    if case.embedded_legs:
        samples: list[dict[str, Any]] = []
        s_offset = 0.0
        for leg in case.embedded_legs:
            nodes = leg.get("nodes", [])
            for index, node in enumerate(nodes):
                if samples and index == 0:
                    continue
                local_s = float(node.get("local_s_mm", node.get("s_mm", 0.0)))
                samples.append(
                    {
                        "s_mm": s_offset + local_s,
                        "x_mm": node["x_mm"],
                        "y_mm": node["y_mm"],
                        "yaw_ddeg": node["yaw_ddeg"],
                    }
                )
            if nodes:
                s_offset = samples[-1]["s_mm"]
        return tuple(samples)
    return ()


def _leg_samples(leg: LegV40) -> tuple[dict[str, Any], ...]:
    return tuple(
        {
            "s_mm": float(node.get("local_s_mm", node.get("s_mm", 0.0))),
            "x_mm": node["x_mm"],
            "y_mm": node["y_mm"],
            "yaw_ddeg": node["yaw_ddeg"],
        }
        for node in leg.nodes
    )


def _no_geometry_result(project: ProjectV40, *, case_hash: str) -> PathCollisionResult:
    world = build_collision_world(project)
    return PathCollisionResult(
        status=CollisionStatus.NO_GEOMETRY,
        checked_config_hash=world.collision_config_hash,
        checked_path_hash=case_hash,
        validation_resolution_mm=world.strict_validation_resolution_mm,
        min_clearance_mm=None,
        min_clearance_pose=None,
        min_clearance_obstacle=None,
        collision_count=0,
        first_collision=None,
        collisions=(),
        checked_pose_count=0,
        subdivision_count=0,
        elapsed_ms=0.0,
        warnings=(),
        errors=("NO_GEOMETRY",),
    )
