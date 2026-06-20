"""Phase 6 service API for directed leg optimization and library updates."""

from __future__ import annotations

import hashlib
from dataclasses import replace
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_bytes, canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project
from hjmb_pathgen.py_planning.geometry.bezier import BezierPath, Point2D
from hjmb_pathgen.py_domain.enums import LegState, YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_domain.leg import LegLibraryV40, LegV40
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName, LegOptimizationRequest, LegOptimizationResult, Pose2D
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.task_plan import TransitionRequirement
from hjmb_pathgen.py_domain.topology import topology_gates_from_profile
from hjmb_pathgen.py_planning.geometry.automatic_topology import topology_profile_object
from hjmb_pathgen.py_planning.optimization.leg_optimizer import PLANNER_ALGORITHM_VERSION, optimize_leg
from hjmb_pathgen.py_planning.dynamics.time_parameterization import TimeParameterizationLimits, TimeParameterizationRequest, samples_from_points, time_parameterize
from hjmb_pathgen.py_services.leg_library_service import (
    approve_leg as approve_leg_in_library,
    load_or_create_leg_library,
    lock_leg as lock_leg_in_library,
    save_leg_library_checked,
    show_leg,
    unlock_leg as unlock_leg_in_library,
    upsert_leg,
)
from hjmb_pathgen.py_services.path_validation_service import validate_leg_collision
from hjmb_pathgen.py_planning.geometry.topology_gates import validate_ordered_topology_gates
from hjmb_pathgen.py_services.project_config_service import compute_project_functional_hashes
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.route_assembler import transition_requirements_from_case


def leg_request_from_transition(
    transition: TransitionRequirement,
    project: ProjectV40,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    yaw_policy: YawPolicy = YawPolicy.SHORTEST,
    warm_start_leg: LegV40 | None = None,
    progress_callback: Any | None = None,
    cancel_check: Any | None = None,
) -> LegOptimizationRequest:
    from_pose = Pose2D.from_dict(transition.from_pose, field_name="from_pose")
    to_pose = Pose2D.from_dict(transition.to_pose, field_name="to_pose")
    topology_profile = topology_profile_object(project, transition.topology_profile, route_family=transition.route_family)
    dependencies = dict(transition.dependency_hashes)
    dependencies.update(compute_project_functional_hashes(project))
    return LegOptimizationRequest(
        project=project,
        from_state_id=transition.from_state_id,
        to_state_id=transition.to_state_id,
        from_pose=from_pose,
        to_pose=to_pose,
        route_family=transition.route_family,
        topology_profile=transition.topology_profile,
        topology_gates=topology_gates_from_profile(topology_profile),
        dependency_hashes=dependencies,
        profile_name=profile_name,
        seed=seed,
        yaw_policy=yaw_policy,
        warm_start_leg=warm_start_leg,
        progress_callback=progress_callback,
        cancel_check=cancel_check,
    )


def optimize_transition_leg(
    transition: TransitionRequirement,
    project: ProjectV40,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    yaw_policy: YawPolicy = YawPolicy.SHORTEST,
    warm_start_leg: LegV40 | None = None,
    progress_callback: Any | None = None,
    cancel_check: Any | None = None,
) -> LegOptimizationResult:
    return optimize_leg(
        leg_request_from_transition(
            transition,
            project,
            profile_name=profile_name,
            seed=seed,
            yaw_policy=yaw_policy,
            warm_start_leg=warm_start_leg,
            progress_callback=progress_callback,
            cancel_check=cancel_check,
        )
    )


def optimize_current_case_leg(
    layout: ProjectLayout,
    case_path: str | Path,
    transition_id: str,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    seed: int = 0,
    replace_existing: bool = False,
) -> LegOptimizationResult:
    project = load_project(layout.project_json)
    case = load_case(case_path, enforce_filename=False)
    transition = _find_transition(case, project, transition_id)
    library = load_or_create_leg_library(layout.leg_library_json, project)
    warm_start = _warm_start_for_transition(library, transition)
    result = optimize_transition_leg(
        transition,
        project,
        profile_name=profile_name,
        seed=seed,
        yaw_policy=_case_yaw_policy(case, transition),
        warm_start_leg=warm_start,
    )
    if result.success and result.leg is not None:
        updated = upsert_leg(library, result.leg, replace_existing=replace_existing)
        save_leg_library_checked(layout.leg_library_json, updated)
    return result


def retime_leg(project: ProjectV40, leg: LegV40, *, profile_name: str = "default") -> LegV40:
    if len(leg.nodes) < 2:
        raise CompileError(f"leg {leg.leg_id} has fewer than two nodes")
    points = tuple((float(node["x_mm"]), float(node["y_mm"]), float(node.get("yaw_ddeg", 0))) for node in leg.nodes)
    samples = samples_from_points(points)
    limits = TimeParameterizationLimits.from_project(project, profile_name=profile_name)
    result = time_parameterize(TimeParameterizationRequest(samples=samples, limits=limits))
    if not result.success:
        raise CompileError(f"retime failed for {leg.leg_id}: {result.reason}")
    nodes = []
    for sample in result.samples:
        nodes.append(
            {
                "local_s_mm": round(sample.s_mm),
                "x_mm": round(sample.x_mm),
                "y_mm": round(sample.y_mm),
                "yaw_ddeg": round(sample.yaw_ddeg),
                "speed_mmps": round(sample.speed_mmps),
                "vx_mmps": round(sample.vx_mmps),
                "vy_mmps": round(sample.vy_mmps),
                "wz_ddegps": round(sample.wz_ddegps),
                "flags": sample.flags,
            }
        )
    analysis = dict(leg.analysis)
    analysis["planned_time_ms"] = result.planned_time_ms
    analysis["max_metrics"] = dict(result.max_metrics)
    analysis["retime_profile"] = profile_name
    analysis["retime_validation"] = {
        "time_parameterization": result.to_dict(),
    }
    hashes = dict(leg.hashes)
    retimed = replace(leg, nodes=tuple(nodes), analysis=analysis, hashes=hashes, state=LegState.VALID)
    retimed = replace(retimed, hashes=_refreshed_leg_hashes(retimed))
    validation = validate_leg(project, retimed)
    if not validation["valid"]:
        return replace(retimed, state=LegState.FAILED, review={**retimed.review, "state": LegState.FAILED.value, "validation": validation})
    return replace(retimed, review={**retimed.review, "state": LegState.VALID.value})


def validate_leg(project: ProjectV40, leg: LegV40) -> dict[str, Any]:
    endpoint = _validate_leg_endpoint_and_nodes(leg)
    topology = _validate_leg_topology(leg)
    collision = validate_leg_collision(leg, project)
    dynamics = _validate_leg_dynamics(project, leg)
    hash_report = _validate_leg_hash(leg)
    valid = endpoint["success"] and topology["success"] and collision.passed and dynamics["success"] and hash_report["success"]
    return {
        "leg_id": leg.leg_id,
        "state": leg.state.value,
        "valid": valid,
        "endpoint": endpoint,
        "topology": topology,
        "collision": collision.to_dict(),
        "dynamics": dynamics,
        "quantization": dynamics.get("quantization", {}),
        "hash": hash_report,
        "node_count": len(leg.nodes),
        "planned_time_ms": leg.analysis.get("planned_time_ms"),
    }


def approve_leg(layout: ProjectLayout, leg_id: str, *, notes: str = "") -> LegLibraryV40:
    library = load_leg_library(layout.leg_library_json)
    updated = approve_leg_in_library(library, leg_id, notes=notes)
    save_leg_library_checked(layout.leg_library_json, updated)
    return updated


def lock_leg(layout: ProjectLayout, leg_id: str, *, notes: str = "") -> LegLibraryV40:
    library = load_leg_library(layout.leg_library_json)
    updated = lock_leg_in_library(library, leg_id, notes=notes)
    save_leg_library_checked(layout.leg_library_json, updated)
    return updated


def unlock_leg(layout: ProjectLayout, leg_id: str) -> LegLibraryV40:
    library = load_leg_library(layout.leg_library_json)
    updated = unlock_leg_in_library(library, leg_id)
    save_leg_library_checked(layout.leg_library_json, updated)
    return updated


def show_leg_from_layout(layout: ProjectLayout, leg_id: str) -> LegV40:
    return show_leg(load_leg_library(layout.leg_library_json), leg_id)


def control_points_from_nodes(leg: LegV40) -> tuple[dict[str, float], ...]:
    points = tuple(Point2D(float(node["x_mm"]), float(node["y_mm"])) for node in leg.nodes)
    return BezierPath.from_waypoints(points).control_points_dicts()


def _find_transition(case: CaseManifestV40, project: ProjectV40, transition_id: str) -> TransitionRequirement:
    for transition in transition_requirements_from_case(case, project):
        if transition.requirement_id == transition_id or transition.semantic_hash == transition_id:
            return transition
    raise CompileError(f"transition requirement not found: {transition_id}")


def _case_yaw_policy(case: CaseManifestV40, transition: TransitionRequirement | None = None) -> YawPolicy:
    # Only the drop-to-drop sweep benefits from the selected one-way yaw policy.
    # Pickup and transfer legs use the time-minimising shortest rotation.
    if transition is not None and not (
        transition.from_state_id.startswith("DROP_STEP_")
        and transition.to_state_id.startswith("DROP_STEP_")
    ):
        return YawPolicy.SHORTEST
    try:
        return YawPolicy(str(case.selected_plan.get("yaw_direction", YawPolicy.SHORTEST.value)))
    except ValueError:
        return YawPolicy.SHORTEST


def _warm_start_for_transition(library: LegLibraryV40, transition: TransitionRequirement) -> LegV40 | None:
    for leg in library.legs:
        key = leg.key
        if _warm_start_key_matches(key, transition):
            return leg
    return None


def _warm_start_key_matches(key: dict[str, Any], transition: TransitionRequirement) -> bool:
    if key.get("from_state_id") != transition.from_state_id or key.get("to_state_id") != transition.to_state_id:
        return False
    if key.get("route_family") != transition.route_family:
        return False
    if key.get("topology_profile") != transition.topology_profile:
        return False
    if dict(key.get("from_pose", {})) != dict(transition.from_pose):
        return False
    if dict(key.get("to_pose", {})) != dict(transition.to_pose):
        return False
    return True


def _validate_leg_endpoint_and_nodes(leg: LegV40) -> dict[str, Any]:
    errors: list[str] = []
    if len(leg.nodes) < 2:
        errors.append("leg must contain at least two nodes")
    previous_s = -1
    for index, node in enumerate(leg.nodes):
        local_s = int(node.get("local_s_mm", node.get("s_mm", 0)))
        if local_s < previous_s:
            errors.append(f"node {index} local_s_mm is not monotonic")
        previous_s = local_s
    if leg.nodes:
        first = leg.nodes[0]
        last = leg.nodes[-1]
        if int(first.get("local_s_mm", first.get("s_mm", -1))) != 0:
            errors.append("first local_s_mm must be 0")
        for label, node in (("first", first), ("last", last)):
            if any(int(node.get(key, 0)) != 0 for key in ("vx_mmps", "vy_mmps", "wz_ddegps")):
                errors.append(f"{label} node velocity must be zero")
        from_pose = dict(leg.key.get("from_pose", {}))
        to_pose = dict(leg.key.get("to_pose", {}))
        if from_pose and (round(float(from_pose.get("x_mm", 0))) != int(first["x_mm"]) or round(float(from_pose.get("y_mm", 0))) != int(first["y_mm"])):
            errors.append("first node does not match from_pose")
        if to_pose and (round(float(to_pose.get("x_mm", 0))) != int(last["x_mm"]) or round(float(to_pose.get("y_mm", 0))) != int(last["y_mm"])):
            errors.append("last node does not match to_pose")
    return {"success": not errors, "errors": errors}


def _validate_leg_topology(leg: LegV40) -> dict[str, Any]:
    gates = tuple(topology_gates_from_profile({"gates": list(leg.key.get("topology_gates", []))}))
    result = validate_ordered_topology_gates(leg.nodes, gates)
    return result.to_dict()


def _validate_leg_dynamics(project: ProjectV40, leg: LegV40) -> dict[str, Any]:
    try:
        points = tuple((float(node["x_mm"]), float(node["y_mm"]), float(node.get("yaw_ddeg", 0))) for node in leg.nodes)
        samples = samples_from_points(points)
        limits = TimeParameterizationLimits.from_project(project, profile_name=str(leg.analysis.get("optimizer_profile", "default")))
        result = time_parameterize(TimeParameterizationRequest(samples=samples, limits=limits))
        return {
            "success": result.success,
            "time_parameterization": result.to_dict(),
            "quantization": result.quantization_margins,
            "errors": [] if result.success else [result.reason],
        }
    except Exception as exc:  # noqa: BLE001 - validation report boundary.
        return {"success": False, "time_parameterization": None, "quantization": {}, "errors": [str(exc)]}


def _validate_leg_hash(leg: LegV40) -> dict[str, Any]:
    expected = str(leg.hashes.get("validity_hash", ""))
    actual = _leg_validity_hash(leg)
    return {"success": bool(expected) and expected == actual, "expected": expected, "actual": actual}


def _refreshed_leg_hashes(leg: LegV40) -> dict[str, Any]:
    payload = _leg_validity_payload(leg)
    hashes = dict(leg.hashes)
    hashes["validity_hash"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    hashes["self_hash32"] = f"0x{canonical_json_crc32_hex(payload).upper()}"
    hashes["planner_algorithm_version"] = PLANNER_ALGORITHM_VERSION
    return hashes


def _leg_validity_hash(leg: LegV40) -> str:
    return hashlib.sha256(canonical_json_bytes(_leg_validity_payload(leg))).hexdigest()


def _leg_validity_payload(leg: LegV40) -> dict[str, Any]:
    metrics = dict(leg.analysis.get("max_metrics", {}))
    return {
        "planner_algorithm_version": str(leg.hashes.get("planner_algorithm_version", PLANNER_ALGORITHM_VERSION)),
        "key": dict(leg.key),
        "control_points": list(leg.control_points),
        "yaw_profile": dict(leg.yaw_profile),
        "nodes": list(leg.nodes),
        "analysis_semantic": {
            "planned_time_ms": leg.analysis.get("planned_time_ms", 0),
            "total_length_mm": leg.analysis.get("total_length_mm", 0),
            "max_metrics": metrics,
            "min_clearance_mm": leg.analysis.get("min_clearance_mm"),
        },
    }
