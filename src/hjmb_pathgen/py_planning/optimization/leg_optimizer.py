"""Phase 6 deterministic directed leg optimizer."""

from __future__ import annotations

import hashlib
import math
import random
import time
from dataclasses import dataclass, replace
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_bytes, canonical_json_crc32_hex
from hjmb_pathgen.py_planning.geometry.bezier import BezierPath
from hjmb_pathgen.py_planning.geometry.initial_guess import InitialGuess, build_initial_guesses
from hjmb_pathgen.py_planning.geometry.sampling import geometry_samples_from_bezier
from hjmb_pathgen.py_planning.geometry.topology_gates import validate_ordered_topology_gates
from hjmb_pathgen.py_domain.collision import CollisionStatus
from hjmb_pathgen.py_domain.enums import LegState, YawPolicy
from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_domain.leg_optimization import (
    CandidateEvaluation,
    LegFailureCategory,
    LegOptimizationRequest,
    LegOptimizationResult,
    LegOptimizationProfileName,
)
from hjmb_pathgen.py_domain.planner_diagnostics import PlannerDiagnostic, PlannerStage
from hjmb_pathgen.py_planning.optimization.objective import candidate_objective_key
from hjmb_pathgen.py_planning.optimization.optimization_profiles import OptimizationProfile, optimization_profile_from_project
from hjmb_pathgen.py_planning.optimization.optimizer_backend import perturb_waypoints
from hjmb_pathgen.py_planning.dynamics.time_parameterization import (
    TimeParameterizationLimits,
    TimeParameterizationRequest,
    TimeParameterizationResult,
    time_parameterize,
)
from hjmb_pathgen.py_planning.optimization.yaw_windows import YawWindowProfile
from hjmb_pathgen.py_services.path_validation_service import validate_spatial_path_collision

PLANNER_ALGORITHM_VERSION = "PHASE6_LEG_OPTIMIZER_V5_TIME_DOMINANT_GATE_PROJECTION"


@dataclass(frozen=True)
class _EvaluatedCandidate:
    evaluation: CandidateEvaluation
    guess: InitialGuess
    path: BezierPath | None = None
    yaw_profile: YawWindowProfile | None = None
    time_result: TimeParameterizationResult | None = None
    local_nodes: tuple[dict[str, Any], ...] = ()


def optimize_leg(request: LegOptimizationRequest) -> LegOptimizationResult:
    start_time = time.perf_counter()
    diagnostics: list[PlannerDiagnostic] = []
    evaluations: list[CandidateEvaluation] = []
    try:
        _validate_request(request)
        profile = optimization_profile_from_project(request.profile_name, request.project.planner_profiles, override_time_budget_ms=request.time_budget_ms)
        _emit(request, diagnostics, PlannerStage.INITIALIZING, f"optimizer profile {profile.name.value}")
        rng = random.Random(request.seed)
        guesses = _ordered_guesses_with_seed(build_initial_guesses(request), profile, rng)
        if not guesses:
            return _failed_result(start_time, LegFailureCategory.INVALID_REQUEST, "no initial guesses", evaluations, diagnostics)

        best = _evaluate_guess_batch(request, profile, guesses, evaluations, diagnostics, start_time)
        if best is None:
            if _cancelled(request):
                return _cancelled_result(start_time, evaluations, diagnostics, best=None)
            if _deadline_expired(profile, start_time):
                return _timeout_result(start_time, evaluations, diagnostics, best=None)
            return _failed_result(start_time, LegFailureCategory.NO_VALID_CANDIDATE, "no valid candidate", evaluations, diagnostics)

        current_best = best
        for pass_index in range(1, profile.coordinate_passes + 1):
            if _cancelled(request):
                return _cancelled_result(start_time, evaluations, diagnostics, best=current_best, request=request, profile=profile)
            if _deadline_expired(profile, start_time):
                return _timeout_result(start_time, evaluations, diagnostics, best=current_best, request=request, profile=profile)
            step = profile.coordinate_step_mm / pass_index
            variants = _seeded_variants(current_best.guess, profile, rng, pass_index=pass_index, step_mm=step)
            if not variants:
                continue
            _emit(request, diagnostics, PlannerStage.REFINEMENT, f"coordinate refinement pass {pass_index}", data={"variant_count": len(variants)})
            refined = _evaluate_guess_batch(request, profile, variants, evaluations, diagnostics, start_time)
            if refined is not None and candidate_objective_key(refined.evaluation) < candidate_objective_key(current_best.evaluation):
                current_best = refined

        return _success_result(request, profile, current_best, start_time, evaluations, diagnostics, reason="OK")
    except ValueError as exc:
        return _failed_result(start_time, LegFailureCategory.INVALID_REQUEST, str(exc), evaluations, diagnostics)


def _evaluate_guess_batch(
    request: LegOptimizationRequest,
    profile: OptimizationProfile,
    guesses: tuple[InitialGuess, ...],
    evaluations: list[CandidateEvaluation],
    diagnostics: list[PlannerDiagnostic],
    start_time: float,
) -> _EvaluatedCandidate | None:
    best: _EvaluatedCandidate | None = None
    for guess in guesses:
        if _cancelled(request) or _deadline_expired(profile, start_time):
            break
        _emit(request, diagnostics, PlannerStage.INITIAL_GUESS, f"evaluating {guess.guess_id}", candidate_id=guess.guess_id)
        candidate = _evaluate_guess(request, profile, guess, start_time=start_time)
        evaluations.append(candidate.evaluation)
        if candidate.evaluation.success:
            if best is None or candidate_objective_key(candidate.evaluation) < candidate_objective_key(best.evaluation):
                best = candidate
    return best


def _evaluate_guess(
    request: LegOptimizationRequest,
    profile: OptimizationProfile,
    guess: InitialGuess,
    *,
    start_time: float | None = None,
) -> _EvaluatedCandidate:
    try:
        path = BezierPath.from_waypoints(guess.waypoints, tension=guess.tension)
        if _path_self_intersects(path):
            return _candidate_failure(guess, LegFailureCategory.INVALID_REQUEST, "Bezier path self-intersects")
    except ValueError as exc:
        return _candidate_failure(guess, LegFailureCategory.INVALID_REQUEST, str(exc))

    best: _EvaluatedCandidate | None = None
    for yaw_index, yaw_profile in enumerate(_yaw_candidates(request, profile)):
        # Always evaluate the first yaw candidate once the XY candidate has
        # been admitted by _evaluate_guess_batch.  If cancellation arrives
        # immediately afterwards, return that valid best-so-far result rather
        # than discarding completed work.  The GUI still provides hard process
        # termination for truly immediate user cancellation.
        if yaw_index > 0 and _cancelled(request):
            break
        if yaw_index > 0 and start_time is not None and _deadline_expired(profile, start_time):
            break
        candidate = _evaluate_xy_yaw(request, profile, guess, path, yaw_profile, yaw_index=yaw_index)
        if candidate.evaluation.success and (best is None or candidate_objective_key(candidate.evaluation) < candidate_objective_key(best.evaluation)):
            best = candidate
        elif best is None:
            best = candidate
    if best is None:
        return _candidate_failure(guess, LegFailureCategory.INVALID_REQUEST, "no yaw candidates")
    return best


def _evaluate_xy_yaw(
    request: LegOptimizationRequest,
    profile: OptimizationProfile,
    guess: InitialGuess,
    path: BezierPath,
    yaw_profile: YawWindowProfile,
    *,
    yaw_index: int,
) -> _EvaluatedCandidate:
    try:
        samples = geometry_samples_from_bezier(
            path,
            yaw_profile,
            max_spacing_mm=profile.max_spacing_mm,
            oversample_per_segment=profile.oversample_per_segment,
            arrival_state_id=request.to_state_id,
        )
    except ValueError as exc:
        return _candidate_failure(_yaw_guess(guess, yaw_index), LegFailureCategory.INVALID_REQUEST, str(exc))

    topology = validate_ordered_topology_gates(samples, request.topology_gates)
    if not topology.success:
        return _candidate_failure(
            _yaw_guess(guess, yaw_index),
            LegFailureCategory.TOPOLOGY_FAILED,
            "; ".join(topology.errors),
            topology=topology.to_dict(),
        )

    collision = validate_spatial_path_collision(samples, request.project, strict=profile.strict_collision)
    if collision.status != CollisionStatus.PASSED:
        return _candidate_failure(
            _yaw_guess(guess, yaw_index),
            LegFailureCategory.COLLISION_FAILED,
            f"collision validation {collision.status.value}",
            topology=topology.to_dict(),
            collision=collision.to_dict(),
            min_clearance_mm=collision.min_clearance_mm,
        )

    limits = TimeParameterizationLimits.from_project(request.project, profile_name=profile.name.value)
    limits = replace(limits, max_spacing_mm=profile.max_spacing_mm)
    time_result = time_parameterize(TimeParameterizationRequest(samples=samples, limits=limits))
    if not time_result.success:
        return _candidate_failure(
            _yaw_guess(guess, yaw_index),
            LegFailureCategory.TIME_PARAMETERIZATION_FAILED,
            time_result.reason,
            topology=topology.to_dict(),
            collision=collision.to_dict(),
            time_parameterization=time_result.to_dict(),
            min_clearance_mm=collision.min_clearance_mm,
        )

    nodes = _local_nodes_from_time_samples(time_result.samples)
    quality_metrics = _curve_quality_metrics(samples)
    combined_metrics = dict(time_result.max_metrics)
    combined_metrics.update(quality_metrics)
    evaluation = CandidateEvaluation(
        candidate_id=_yaw_guess(guess, yaw_index).guess_id,
        source=guess.source,
        success=True,
        planned_time_ms=time_result.planned_time_ms,
        total_length_mm=time_result.samples[-1].s_mm,
        min_clearance_mm=collision.min_clearance_mm,
        topology=topology.to_dict(),
        collision=collision.to_dict(),
        time_parameterization=time_result.to_dict(),
        max_metrics=combined_metrics,
    )
    return _EvaluatedCandidate(
        evaluation=evaluation,
        guess=guess,
        path=path,
        yaw_profile=yaw_profile,
        time_result=time_result,
        local_nodes=nodes,
    )



def _curve_quality_metrics(samples: tuple[object, ...]) -> dict[str, float]:
    curvatures = [abs(float(getattr(sample, "curvature_1_per_mm", 0.0))) for sample in samples]
    if not curvatures:
        return {
            "max_abs_curvature_1_per_mm": 0.0,
            "min_curvature_radius_mm": float("inf"),
            "max_curvature_jump_1_per_mm": 0.0,
            "curvature_squared_integral_1_per_mm": 0.0,
        }
    maximum = max(curvatures)
    jumps = [abs(right - left) for left, right in zip(curvatures, curvatures[1:])]
    integral = 0.0
    for left, right, left_sample, right_sample in zip(
        curvatures, curvatures[1:], samples, samples[1:]
    ):
        ds = max(0.0, float(getattr(right_sample, "s_mm")) - float(getattr(left_sample, "s_mm")))
        integral += 0.5 * (left * left + right * right) * ds
    return {
        "max_abs_curvature_1_per_mm": maximum,
        "min_curvature_radius_mm": (1.0 / maximum) if maximum > 1.0e-12 else 1.0e12,
        "max_curvature_jump_1_per_mm": max(jumps, default=0.0),
        "curvature_squared_integral_1_per_mm": integral,
    }

def _yaw_candidates(request: LegOptimizationRequest, profile: OptimizationProfile) -> tuple[YawWindowProfile, ...]:
    return tuple(
        YawWindowProfile(
            start_yaw_ddeg=request.from_pose.yaw_ddeg,
            finish_yaw_ddeg=request.to_pose.yaw_ddeg,
            policy=request.yaw_policy,
            alpha=alpha,
            start_window_end_s_ratio=start_end,
            finish_window_start_s_ratio=finish_start,
        )
        for alpha in profile.yaw_alpha_values
        for start_end, finish_start in profile.yaw_window_pairs
    )


def _yaw_guess(guess: InitialGuess, yaw_index: int) -> InitialGuess:
    return InitialGuess(guess_id=f"{guess.guess_id}_yaw{yaw_index}", source=guess.source, waypoints=guess.waypoints, tension=guess.tension)


def _build_leg(request: LegOptimizationRequest, profile: OptimizationProfile, candidate: _EvaluatedCandidate, *, elapsed_ms: int) -> LegV40:
    if candidate.path is None or candidate.yaw_profile is None or candidate.time_result is None:
        raise ValueError("selected candidate is incomplete")
    leg_key = _leg_key(request)
    leg_id = leg_id_from_key(leg_key)
    total_length = candidate.evaluation.total_length_mm
    yaw_profile_dict = candidate.yaw_profile.to_dict(total_length_mm=total_length)
    control_points = tuple(
        dict(point, representation="PIECEWISE_CUBIC_BEZIER", order=index)
        for index, point in enumerate(candidate.path.control_points_dicts())
    )
    analysis = _analysis_dict(request, profile, candidate, elapsed_ms=elapsed_ms)
    validity_payload = {
        "planner_algorithm_version": PLANNER_ALGORITHM_VERSION,
        "key": leg_key,
        "control_points": control_points,
        "yaw_profile": yaw_profile_dict,
        "nodes": candidate.local_nodes,
        "analysis_semantic": {
            "planned_time_ms": analysis["planned_time_ms"],
            "total_length_mm": analysis["total_length_mm"],
            "max_metrics": analysis["max_metrics"],
            "min_clearance_mm": analysis["min_clearance_mm"],
        },
    }
    validity_hash = hashlib.sha256(canonical_json_bytes(validity_payload)).hexdigest()
    state = LegState.PREVIEW_VALID if request.profile_name == LegOptimizationProfileName.QUICK_PREVIEW else LegState.VALID
    return LegV40(
        leg_id=leg_id,
        key=leg_key,
        state=state,
        source="PHASE6_OPTIMIZER",
        topology_profile=request.topology_profile,
        control_points=control_points,
        yaw_profile=yaw_profile_dict,
        nodes=candidate.local_nodes,
        analysis=analysis,
        hashes={
            "validity_hash": validity_hash,
            "self_hash32": f"0x{canonical_json_crc32_hex(validity_payload).upper()}",
            "dependency_hashes": request.dependency_payload,
            "planner_algorithm_version": PLANNER_ALGORITHM_VERSION,
        },
        review={
            "approved": False,
            "locked": False,
            "notes": "",
            "state": state.value,
            "stale_reason": "",
        },
    )


def _analysis_dict(request: LegOptimizationRequest, profile: OptimizationProfile, candidate: _EvaluatedCandidate, *, elapsed_ms: int) -> dict[str, Any]:
    time_result = candidate.time_result
    if time_result is None:
        raise ValueError("candidate has no timing")
    max_metrics = dict(candidate.evaluation.max_metrics or time_result.max_metrics)
    return {
        "planned_time_ms": candidate.evaluation.planned_time_ms,
        "total_length_mm": candidate.evaluation.total_length_mm,
        "max_speed_mmps": max_metrics.get("max_speed_mmps", 0.0),
        "max_accel_mmps2": max_metrics.get("max_total_accel_mmps2", 0.0),
        "max_lateral_accel_mmps2": max_metrics.get("max_lateral_accel_mmps2", 0.0),
        "max_wz_ddegps": max_metrics.get("max_wz_ddegps", 0.0),
        "max_beta_ddegps2": max_metrics.get("max_beta_ddegps2", 0.0),
        "max_wheel_rpm": max_metrics.get("max_wheel_rpm", 0.0),
        "max_metrics": max_metrics,
        "min_clearance_mm": candidate.evaluation.min_clearance_mm,
        "optimizer_seed": request.seed,
        "optimizer_elapsed_ms": elapsed_ms,
        "optimizer_profile": profile.name.value,
        "planner_algorithm_version": PLANNER_ALGORITHM_VERSION,
        "selected_candidate_id": candidate.evaluation.candidate_id,
        "selected_candidate_source": candidate.evaluation.source,
        "eval_counts": {
            "timed_samples": len(time_result.samples),
            "dense_nodes": len(candidate.local_nodes),
            "subdivision_count": time_result.subdivision_count,
            "repair_iterations": time_result.repair_iterations,
            "iteration_count": time_result.iteration_count,
        },
        "validation": {
            "topology": candidate.evaluation.topology,
            "collision_status": (candidate.evaluation.collision or {}).get("status"),
            "time_parameterization": candidate.evaluation.time_parameterization,
        },
        "warnings": list(time_result.warnings),
    }


def _leg_key(request: LegOptimizationRequest) -> dict[str, Any]:
    return {
        "version": "HJMB_PHASE6_DIRECTED_LEG_KEY_V1",
        "planner_algorithm_version": PLANNER_ALGORITHM_VERSION,
        "from_state_id": request.from_state_id,
        "to_state_id": request.to_state_id,
        "from_pose": request.from_pose.to_dict(),
        "to_pose": request.to_pose.to_dict(),
        "route_family": request.route_family,
        "topology_profile": request.topology_profile,
        "topology_gates": [gate.to_dict() for gate in request.topology_gates],
        "footprint_state": dict(request.footprint_state or {}),
        "load_state": dict(request.load_state or {}),
        "mechanism_state": dict(request.mechanism_state or {}),
        "unload_state": dict(request.unload_state or {}),
        "yaw_policy": request.yaw_policy.value if isinstance(request.yaw_policy, YawPolicy) else str(request.yaw_policy),
        "dependency_hashes": request.dependency_payload,
    }


def leg_key_from_request(request: LegOptimizationRequest) -> dict[str, Any]:
    return _leg_key(request)


def leg_id_from_key(leg_key: dict[str, Any]) -> str:
    return f"LEG_{hashlib.sha256(canonical_json_bytes(leg_key)).hexdigest()[:12].upper()}"


def _local_nodes_from_time_samples(samples: tuple[object, ...]) -> tuple[dict[str, Any], ...]:
    nodes: list[dict[str, Any]] = []
    for sample in samples:
        node = {
            "local_s_mm": round(float(getattr(sample, "s_mm"))),
            "x_mm": round(float(getattr(sample, "x_mm"))),
            "y_mm": round(float(getattr(sample, "y_mm"))),
            "yaw_ddeg": round(float(getattr(sample, "yaw_ddeg"))),
            "speed_mmps": round(float(getattr(sample, "speed_mmps"))),
            "vx_mmps": round(float(getattr(sample, "vx_mmps"))),
            "vy_mmps": round(float(getattr(sample, "vy_mmps"))),
            "wz_ddegps": round(float(getattr(sample, "wz_ddegps"))),
            "flags": int(getattr(sample, "flags")),
        }
        arrival_state_id = str(getattr(sample, "arrival_state_id", ""))
        if arrival_state_id:
            node["arrival_state_id"] = arrival_state_id

        # Millimetre quantisation can map two neighbouring floating-point
        # samples to the same XY coordinate while local_s/yaw still advance.
        # Keeping both creates a zero-length tangent when a saved Leg is
        # strictly revalidated.  Merge only consecutive quantisation duplicates;
        # this does not alter the continuous path or its endpoint semantics.
        if nodes and node["x_mm"] == nodes[-1]["x_mm"] and node["y_mm"] == nodes[-1]["y_mm"]:
            previous = nodes[-1]
            merged = dict(node)
            merged["flags"] = int(previous.get("flags", 0)) | int(node.get("flags", 0))
            if "arrival_state_id" not in merged and "arrival_state_id" in previous:
                merged["arrival_state_id"] = previous["arrival_state_id"]
            if int(previous.get("local_s_mm", 0)) == 0:
                merged["local_s_mm"] = 0
                merged["yaw_ddeg"] = previous.get("yaw_ddeg", merged["yaw_ddeg"])
                merged["vx_mmps"] = 0
                merged["vy_mmps"] = 0
                merged["wz_ddegps"] = 0
            nodes[-1] = merged
        else:
            nodes.append(node)
    if nodes:
        nodes[0]["local_s_mm"] = 0
        nodes[0]["vx_mmps"] = 0
        nodes[0]["vy_mmps"] = 0
        nodes[0]["wz_ddegps"] = 0
        nodes[-1]["vx_mmps"] = 0
        nodes[-1]["vy_mmps"] = 0
        nodes[-1]["wz_ddegps"] = 0
    return tuple(nodes)


def _candidate_failure(
    guess: InitialGuess,
    category: LegFailureCategory,
    reason: str,
    *,
    topology: dict[str, Any] | None = None,
    collision: dict[str, Any] | None = None,
    time_parameterization: dict[str, Any] | None = None,
    min_clearance_mm: float | None = None,
) -> _EvaluatedCandidate:
    return _EvaluatedCandidate(
        evaluation=CandidateEvaluation(
            candidate_id=guess.guess_id,
            source=guess.source,
            success=False,
            min_clearance_mm=min_clearance_mm,
            failure_category=category,
            failure_reason=reason,
            topology=topology,
            collision=collision,
            time_parameterization=time_parameterization,
        ),
        guess=guess,
    )


def _validate_request(request: LegOptimizationRequest) -> None:
    if not request.from_state_id or not request.to_state_id:
        raise ValueError("from_state_id and to_state_id are required")
    if request.from_state_id == request.to_state_id:
        raise ValueError("directed leg endpoints must differ")
    request.from_pose.to_dict()
    request.to_pose.to_dict()
    if request.route_family == "":
        raise ValueError("route_family is required")


def _ordered_guesses_with_seed(guesses: tuple[InitialGuess, ...], profile: OptimizationProfile, rng: random.Random) -> tuple[InitialGuess, ...]:
    # Initial guesses are deliberately ordered by reliability: manual/warm
    # starts, obstacle-aware detours, the straight line, then gate variants.
    # Shuffling that order previously meant a valid detour could be dropped by
    # max_initial_guesses, leaving FULL_AUTO to test only colliding straight
    # paths. Randomness is used only inside explicitly seeded variants.
    expanded: list[InitialGuess] = []
    for guess in guesses:
        expanded.append(guess)
        if len(guess.waypoints) == 2 and profile.random_variant_count:
            expanded.extend(_random_midpoint_guesses(guess, profile, rng))
    return tuple(expanded[: profile.max_initial_guesses])


def _random_midpoint_guesses(guess: InitialGuess, profile: OptimizationProfile, rng: random.Random) -> tuple[InitialGuess, ...]:
    start, finish = guess.waypoints[0], guess.waypoints[-1]
    dx = finish.x_mm - start.x_mm
    dy = finish.y_mm - start.y_mm
    norm = math.hypot(dx, dy)
    if norm <= 1.0e-9:
        return ()
    normal = (-dy / norm, dx / norm)
    result: list[InitialGuess] = []
    for index in range(profile.random_variant_count):
        along = 0.35 + 0.30 * rng.random()
        offset = (rng.random() * 2.0 - 1.0) * max(profile.coordinate_step_mm, 20.0)
        midpoint = guess.waypoints[0].__class__(
            start.x_mm + dx * along + normal[0] * offset,
            start.y_mm + dy * along + normal[1] * offset,
        )
        result.append(
            InitialGuess(
                f"{guess.guess_id}_seed{index}",
                "SEEDED_MIDPOINT",
                (start, midpoint, finish),
                tension=guess.tension,
            )
        )
    return tuple(result)


def _seeded_variants(guess: InitialGuess, profile: OptimizationProfile, rng: random.Random, *, pass_index: int, step_mm: float) -> tuple[InitialGuess, ...]:
    # The projected topology-gate seed already uses the closest legal crossing
    # points to the direct line.  Moving those gate points normal to the path
    # mostly makes the S wider and can multiply strict collision runtime.  Keep
    # the centre-gate fallback in the initial batch, but do not perturb the
    # selected shortest-gate geometry.
    if guess.guess_id.startswith("official_s_gate_shortest_seed"):
        return ()
    variants = list(perturb_waypoints(guess, pass_index=pass_index, step_mm=step_mm))
    if len(guess.waypoints) == 2:
        variants.extend(_random_midpoint_guesses(guess, profile, rng))
    rng.shuffle(variants)
    return tuple(variants)


def _path_self_intersects(path: BezierPath) -> bool:
    try:
        samples = path.sample_arclength(max_spacing_mm=20.0, oversample_per_segment=32)
    except ValueError:
        return True
    points = [(sample.x_mm, sample.y_mm) for sample in samples]
    for left_index in range(len(points) - 1):
        for right_index in range(left_index + 2, len(points) - 1):
            if left_index == 0 and right_index == len(points) - 2:
                continue
            if _segments_intersect(points[left_index], points[left_index + 1], points[right_index], points[right_index + 1]):
                return True
    return False


def _segments_intersect(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    def orient(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    return o1 * o2 < -1.0e-9 and o3 * o4 < -1.0e-9


def _success_result(
    request: LegOptimizationRequest,
    profile: OptimizationProfile,
    best: _EvaluatedCandidate,
    start_time: float,
    evaluations: list[CandidateEvaluation],
    diagnostics: list[PlannerDiagnostic],
    *,
    reason: str,
    result_state: LegState | None = None,
) -> LegOptimizationResult:
    leg = _build_leg(request, profile, best, elapsed_ms=_elapsed_ms(start_time))
    _emit(request, diagnostics, PlannerStage.COMPLETE, f"selected {best.evaluation.candidate_id}")
    return LegOptimizationResult(
        success=True,
        state=result_state or leg.state,
        leg=leg,
        reason=reason,
        evaluations=tuple(evaluations),
        diagnostics=tuple(diagnostics),
        elapsed_ms=_elapsed_ms(start_time),
    )


def _emit(
    request: LegOptimizationRequest,
    diagnostics: list[PlannerDiagnostic],
    stage: PlannerStage,
    message: str,
    *,
    candidate_id: str = "",
    data: dict[str, Any] | None = None,
) -> None:
    diagnostic = PlannerDiagnostic(stage=stage, message=message, candidate_id=candidate_id, data=data)
    diagnostics.append(diagnostic)
    if request.progress_callback is not None:
        request.progress_callback(diagnostic.to_dict())


def _cancelled(request: LegOptimizationRequest) -> bool:
    return bool(request.cancel_check and request.cancel_check())


def _deadline_expired(profile: OptimizationProfile, start_time: float) -> bool:
    return _elapsed_ms(start_time) > profile.time_budget_ms


def _elapsed_ms(start_time: float) -> int:
    return round((time.perf_counter() - start_time) * 1000.0)


def _failed_result(
    start_time: float,
    category: LegFailureCategory,
    reason: str,
    evaluations: list[CandidateEvaluation],
    diagnostics: list[PlannerDiagnostic],
) -> LegOptimizationResult:
    diagnostics.append(PlannerDiagnostic(stage=PlannerStage.FAILED, message=reason, data={"category": category.value}))
    return LegOptimizationResult(
        success=False,
        state=LegState.FAILED,
        leg=None,
        reason=reason,
        evaluations=tuple(evaluations),
        diagnostics=tuple(diagnostics),
        elapsed_ms=_elapsed_ms(start_time),
    )


def _cancelled_result(
    start_time: float,
    evaluations: list[CandidateEvaluation],
    diagnostics: list[PlannerDiagnostic],
    *,
    best: _EvaluatedCandidate | None,
    request: LegOptimizationRequest | None = None,
    profile: OptimizationProfile | None = None,
) -> LegOptimizationResult:
    diagnostics.append(PlannerDiagnostic(stage=PlannerStage.CANCELLED, message="optimization cancelled"))
    if best is not None and request is not None and profile is not None:
        return _success_result(
            request,
            profile,
            best,
            start_time,
            evaluations,
            diagnostics,
            reason="CANCELLED_WITH_BEST",
            result_state=LegState.CANCELLED_WITH_BEST,
        )
    return LegOptimizationResult(
        success=False,
        state=LegState.CANCELLED,
        leg=None,
        reason="CANCELLED",
        evaluations=tuple(evaluations),
        diagnostics=tuple(diagnostics),
        elapsed_ms=_elapsed_ms(start_time),
    )


def _timeout_result(
    start_time: float,
    evaluations: list[CandidateEvaluation],
    diagnostics: list[PlannerDiagnostic],
    *,
    best: _EvaluatedCandidate | None,
    request: LegOptimizationRequest | None = None,
    profile: OptimizationProfile | None = None,
) -> LegOptimizationResult:
    diagnostics.append(PlannerDiagnostic(stage=PlannerStage.CANCELLED, message="optimization timeout"))
    if best is not None and request is not None and profile is not None:
        return _success_result(
            request,
            profile,
            best,
            start_time,
            evaluations,
            diagnostics,
            reason="TIMEOUT_WITH_BEST",
            result_state=LegState.TIMEOUT_WITH_BEST,
        )
    return LegOptimizationResult(
        success=False,
        state=LegState.TIMEOUT,
        leg=None,
        reason="TIMEOUT",
        evaluations=tuple(evaluations),
        diagnostics=tuple(diagnostics),
        elapsed_ms=_elapsed_ms(start_time),
    )
