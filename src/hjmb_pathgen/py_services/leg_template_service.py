"""Pure synchronization, exact expansion, and validation for leg templates."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable

from hjmb_pathgen.py_domain.competition_task_config import CompetitionTaskConfigV40, LOGICAL_DROP_STATIONS
from hjmb_pathgen.py_domain.enums import YawPolicy
from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName, LegOptimizationRequest, Pose2D
from hjmb_pathgen.py_domain.leg_template import (
    LegTemplateFailureV40,
    LegTemplateInstanceState,
    LegTemplateInstancesV40,
    LegTemplateInstanceV40,
    LegTemplateRouteFamily,
    LegTemplatesV40,
    LegTemplateState,
    LegTemplateValidationEntryV40,
    LegTemplateValidationReportV40,
    LegTemplateV40,
)
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.topology import topology_gates_from_profile
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_bytes
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_leg_template_instances,
    load_leg_template_validation_report,
    load_leg_templates,
    load_project,
    save_leg_template_instances,
    save_leg_templates,
    save_leg_template_validation_report,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from hjmb_pathgen.py_planning.geometry.automatic_topology import (
    NO_GATE_PROFILE_ID,
    topology_profile_object,
)
from hjmb_pathgen.py_planning.optimization.leg_optimizer import PLANNER_ALGORITHM_VERSION, optimize_leg
from hjmb_pathgen.py_services.competition_task_config_service import load_competition_task_config
from hjmb_pathgen.py_services.leg_optimization_service import validate_leg


@dataclass(frozen=True)
class LegTemplateExpansionResult:
    instances: tuple[LegTemplateInstanceV40, ...]
    missing_profiles: tuple[str, ...]
    errors: tuple[str, ...] = ()


@dataclass(frozen=True)
class LegTemplateValidationResult:
    template: LegTemplateV40
    instances: tuple[LegTemplateInstanceV40, ...]
    report: LegTemplateValidationEntryV40


def leg_template_id(route_family: str, from_site: str, to_site: str) -> str:
    """Return the coordinate-independent stable identifier for a logical slot."""

    return f"LT_{route_family}_{from_site}_TO_{to_site}"


def leg_template_instance_id(
    template_id: str,
    from_state_key: str,
    to_state_key: str,
) -> str:
    payload = {"template_id": template_id, "from_state_key": from_state_key, "to_state_key": to_state_key}
    return f"LTI_{hashlib.sha256(canonical_json_bytes(payload)).hexdigest()[:16].upper()}"


def compute_leg_template_dependency_hashes(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
) -> dict[str, str]:
    payloads = {
        "project_pose_hash": {"sites": project.sites, "unload_pose_profiles": project.unload_pose_profiles},
        "collision_hash": {"field_objects": project.field_objects, "footprint": project.vehicle.get("footprint", {})},
        "topology_hash": project.topology_profiles,
        "planning_hash": {"dynamics": project.dynamics, "vehicle": project.vehicle, "planner_profiles": project.planner_profiles},
        "task_config_hash": task_config.to_dict(),
        "planner_algorithm_hash": PLANNER_ALGORITHM_VERSION,
    }
    return {key: hashlib.sha256(canonical_json_bytes(value)).hexdigest() for key, value in payloads.items()}


def expected_leg_template_slots(task_config: CompetitionTaskConfigV40) -> tuple[tuple[str, str, str], ...]:
    slots: list[tuple[str, str, str]] = []
    for route_name in (LegTemplateRouteFamily.PICK_1_TO_3.value, LegTemplateRouteFamily.PICK_3_TO_1.value):
        route = task_config.route_families[route_name]
        picks = tuple(str(item) for item in route["pickup_arrival_state_order"])
        drops = tuple(str(item) for item in route["drop_station_order"])
        chain = ("P_START",) + picks
        slots.extend((route_name, left, right) for left, right in zip(chain, chain[1:]))
        # The last pickup can only enter the first or middle unload station
        # of the selected route.  Entering the far-end station directly is not
        # a legal task topology: if only that station is needed, the opposite
        # route is selected; if an earlier station is also needed, unloading
        # must happen there before reaching the far end.
        slots.extend((route_name, picks[-1], drop) for drop in drops[:-1])
        slots.extend((route_name, drops[left], drops[right]) for left in range(len(drops)) for right in range(left + 1, len(drops)))
    return tuple(slots)


def sync_leg_templates(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    existing: LegTemplatesV40 | None = None,
) -> LegTemplatesV40:
    dependencies = compute_leg_template_dependency_hashes(project, task_config)
    previous = {item.template_id: item for item in (existing.templates if existing else ())}
    active_ids: set[str] = set()
    synchronized: list[LegTemplateV40] = []
    for route, from_site, to_site in expected_leg_template_slots(task_config):
        template_id = leg_template_id(route, from_site, to_site)
        active_ids.add(template_id)
        old = previous.get(template_id)
        if old is None:
            enabled = False
            waypoints = ()
            state = LegTemplateState.DISABLED
            last_validated_hash = ""
            audit_messages: tuple[str, ...] = ()
        else:
            enabled = old.enabled
            waypoints = old.waypoints
            dependencies_changed = old.dependency_hashes != dependencies
            state = LegTemplateState.STALE if dependencies_changed and enabled else old.state
            if not enabled:
                state = LegTemplateState.DISABLED
            last_validated_hash = "" if dependencies_changed else old.last_validated_hash
            audit_messages = old.audit_messages
        template_hash = _template_source_hash(template_id, enabled, route, from_site, to_site, waypoints)
        if old is not None and old.template_hash != template_hash and enabled:
            state = LegTemplateState.STALE
            last_validated_hash = ""
        synchronized.append(
            LegTemplateV40(
                template_id=template_id, enabled=enabled, route_family=LegTemplateRouteFamily(route),
                from_site=from_site, to_site=to_site, waypoints=waypoints, state=state,
                template_hash=template_hash, dependency_hashes=dependencies,
                last_validated_hash=last_validated_hash, orphaned=False, audit_messages=audit_messages,
            )
        )
    for old in previous.values():
        if old.template_id in active_ids:
            continue
        message = "slot is no longer legal under the current competition task configuration"
        synchronized.append(replace(old, enabled=False, state=LegTemplateState.DISABLED, orphaned=True,
                                    last_validated_hash="", audit_messages=old.audit_messages + (message,)))
    synchronized.sort(key=lambda item: item.template_id)
    return LegTemplatesV40(project.project_id, dependencies, tuple(synchronized))


def sync_leg_templates_for_layout(layout: ProjectLayout) -> LegTemplatesV40:
    project = load_project(layout.project_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    old = load_leg_templates(layout.leg_templates_json) if layout.leg_templates_json.exists() else None
    synchronized = sync_leg_templates(project, task_config, old)
    save_leg_templates(layout.leg_templates_json, synchronized)
    old_by_id = {item.template_id: item for item in old.templates} if old else {}
    current_by_id = {item.template_id: item for item in synchronized.templates}
    stale_template_ids = {
        template_id
        for template_id, current in current_by_id.items()
        if template_id in old_by_id
        and (
            old_by_id[template_id].template_hash != current.template_hash
            or old_by_id[template_id].dependency_hashes != current.dependency_hashes
            or current.orphaned
        )
    }
    if layout.leg_template_instances_json.exists():
        previous_instances = load_leg_template_instances(layout.leg_template_instances_json)
        dependencies_changed = previous_instances.dependency_hashes != synchronized.dependency_hashes
        if dependencies_changed or stale_template_ids:
            stale = tuple(
                _stale_instance(item)
                if dependencies_changed or item.template_id in stale_template_ids
                else item
                for item in previous_instances.instances
            )
            save_leg_template_instances(
                layout.leg_template_instances_json,
                LegTemplateInstancesV40(project.project_id, synchronized.dependency_hashes, stale),
            )
    else:
        save_leg_template_instances(
            layout.leg_template_instances_json,
            LegTemplateInstancesV40(project.project_id, synchronized.dependency_hashes, ()),
        )
    if layout.leg_template_validation_report_json.exists():
        previous_report = load_leg_template_validation_report(layout.leg_template_validation_report_json)
        dependencies_changed = previous_report.dependency_hashes != synchronized.dependency_hashes
        if dependencies_changed or stale_template_ids:
            reports = tuple(
                _stale_report_entry(entry)
                if dependencies_changed or entry.template_id in stale_template_ids
                else entry
                for entry in previous_report.template_reports
            )
            save_leg_template_validation_report(
                layout.leg_template_validation_report_json,
                LegTemplateValidationReportV40(project.project_id, synchronized.dependency_hashes, reports),
            )
    else:
        save_leg_template_validation_report(
            layout.leg_template_validation_report_json,
            LegTemplateValidationReportV40(project.project_id, synchronized.dependency_hashes, ()),
        )
    return synchronized


def expand_leg_template_instances(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    template: LegTemplateV40,
) -> LegTemplateExpansionResult:
    legal_ids = {leg_template_id(*slot) for slot in expected_leg_template_slots(task_config)}
    if template.orphaned or template.template_id not in legal_ids:
        return LegTemplateExpansionResult((), (), ("template is not a legal directed slot",))
    from_options, from_missing = _pose_options(project, task_config, template.from_site)
    to_options, to_missing = _pose_options(project, task_config, template.to_site)
    instances: list[LegTemplateInstanceV40] = []
    dependencies = compute_leg_template_dependency_hashes(project, task_config)
    for from_key, from_pose, from_profile in from_options:
        for to_key, to_pose, to_profile in to_options:
            instance_id = leg_template_instance_id(template.template_id, from_key, to_key)
            hashes = dict(dependencies)
            hashes["template_hash"] = template.template_hash
            hashes["instance_input_hash"] = _validation_input_hash(template, dependencies, from_key, to_key, from_pose, to_pose)
            instances.append(
                LegTemplateInstanceV40(
                    instance_id, template.template_id, from_key, to_key, from_pose, to_pose,
                    from_profile, to_profile, LegTemplateInstanceState.STALE, 0, None, {}, hashes, (), None,
                )
            )
    instances.sort(key=lambda item: item.instance_id)
    return LegTemplateExpansionResult(tuple(instances), tuple(sorted(set(from_missing + to_missing))))


def validate_leg_template(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    template: LegTemplateV40,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    cancel_check: Any | None = None,
    progress_callback: Any | None = None,
) -> LegTemplateValidationResult:
    expansion = expand_leg_template_instances(project, task_config, template)
    validated: list[LegTemplateInstanceV40] = []
    for index, instance in enumerate(expansion.instances):
        if cancel_check is not None and cancel_check():
            raise RuntimeError("CANCELLED")
        if progress_callback is not None:
            progress_callback({
                "template_id": template.template_id,
                "instance_id": instance.instance_id,
                "instance_index": index + 1,
                "instance_count": len(expansion.instances),
                "percent": round(100 * index / max(len(expansion.instances), 1)),
            })
        validated.append(_validate_instance(project, task_config, template, instance, profile_name, cancel_check))
    passed = sum(item.state == LegTemplateInstanceState.PASSED for item in validated)
    failed = len(validated) - passed
    status = aggregate_leg_template_state(item.state for item in validated)
    validation_hash = _template_validation_hash(template, compute_leg_template_dependency_hashes(project, task_config))
    updated = replace(template, state=status, last_validated_hash=validation_hash)
    reports = tuple(
        {
            "instance_id": item.instance_id,
            "status": item.state.value,
            "failure_reasons": [failure.message for failure in item.failures],
            "metrics": dict(item.analysis_metrics),
        }
        for item in validated
    )
    entry = LegTemplateValidationEntryV40(
        template_id=template.template_id, status=status,
        instance_counts={"total": len(validated), "passed": passed, "failed": failed},
        missing_profiles=expansion.missing_profiles, errors=expansion.errors, instance_reports=reports,
    )
    return LegTemplateValidationResult(updated, tuple(validated), entry)


def aggregate_leg_template_state(states: Iterable[LegTemplateInstanceState]) -> LegTemplateState:
    values = tuple(states)
    passed = sum(item == LegTemplateInstanceState.PASSED for item in values)
    if values and passed == len(values):
        return LegTemplateState.PASSED
    if passed:
        return LegTemplateState.PARTIAL
    return LegTemplateState.FAILED


def leg_template_instance_is_current_and_passed(
    instance: LegTemplateInstanceV40,
    template: LegTemplateV40,
    dependency_hashes: dict[str, str],
) -> bool:
    """Eligibility predicate for later rounds; this round does not consume it."""

    expected_input = _validation_input_hash(
        template, dependency_hashes, instance.from_state_key, instance.to_state_key,
        instance.from_pose, instance.to_pose,
    )
    return (
        instance.state == LegTemplateInstanceState.PASSED
        and instance.compiled_leg is not None
        and instance.template_id == template.template_id
        and template.last_validated_hash == _template_validation_hash(template, dependency_hashes)
        and instance.hashes.get("template_hash") == template.template_hash
        and instance.hashes.get("instance_input_hash") == expected_input
        and all(instance.hashes.get(key) == value for key, value in dependency_hashes.items())
    )


def validate_all_enabled_templates(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    templates: LegTemplatesV40,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    cancel_check: Any | None = None,
    progress_callback: Any | None = None,
) -> tuple[LegTemplatesV40, LegTemplateInstancesV40, LegTemplateValidationReportV40]:
    updated_by_id = {item.template_id: item for item in templates.templates}
    all_instances: list[LegTemplateInstanceV40] = []
    reports: list[LegTemplateValidationEntryV40] = []
    enabled = [item for item in templates.templates if item.enabled and not item.orphaned]
    for index, template in enumerate(enabled):
        if cancel_check is not None and cancel_check():
            raise RuntimeError("CANCELLED")
        if progress_callback is not None:
            progress_callback({
                "template_id": template.template_id,
                "template_index": index + 1,
                "template_count": len(enabled),
                "percent": round(100 * index / max(len(enabled), 1)),
            })
        result = validate_leg_template(
            project, task_config, template, profile_name=profile_name,
            cancel_check=cancel_check,
        )
        updated_by_id[template.template_id] = result.template
        all_instances.extend(result.instances)
        reports.append(result.report)
    dependencies = compute_leg_template_dependency_hashes(project, task_config)
    updated = LegTemplatesV40(project.project_id, dependencies, tuple(sorted(updated_by_id.values(), key=lambda item: item.template_id)))
    instances = LegTemplateInstancesV40(project.project_id, dependencies, tuple(sorted(all_instances, key=lambda item: item.instance_id)))
    report = LegTemplateValidationReportV40(project.project_id, dependencies, tuple(sorted(reports, key=lambda item: item.template_id)))
    return updated, instances, report


def validate_leg_template_for_layout(
    layout: ProjectLayout,
    template_id: str,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    expected_template_hash: str | None = None,
    expected_dependency_hashes: dict[str, str] | None = None,
    cancel_check: Any | None = None,
    progress_callback: Any | None = None,
) -> LegTemplateValidationResult:
    """Validate one slot and update all three documents with atomic file writes."""

    project = load_project(layout.project_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    templates = sync_leg_templates_for_layout(layout)
    try:
        template = next(item for item in templates.templates if item.template_id == template_id)
    except StopIteration as exc:
        raise KeyError(f"leg template not found: {template_id}") from exc
    _require_expected_inputs(template, expected_template_hash, expected_dependency_hashes)
    result = validate_leg_template(
        project, task_config, template, profile_name=profile_name,
        cancel_check=cancel_check, progress_callback=progress_callback,
    )

    _assert_validation_inputs_current(
        layout, {template_id: template.template_hash}, templates.dependency_hashes,
    )

    template_items = tuple(
        result.template if item.template_id == template_id else item
        for item in templates.templates
    )
    updated_templates = replace(templates, templates=template_items)

    existing_instances = load_leg_template_instances(layout.leg_template_instances_json)
    instance_items = tuple(item for item in existing_instances.instances if item.template_id != template_id) + result.instances
    updated_instances = LegTemplateInstancesV40(
        project.project_id, updated_templates.dependency_hashes,
        tuple(sorted(instance_items, key=lambda item: item.instance_id)),
    )

    existing_report = load_leg_template_validation_report(layout.leg_template_validation_report_json)
    report_items = tuple(item for item in existing_report.template_reports if item.template_id != template_id) + (result.report,)
    updated_report = LegTemplateValidationReportV40(
        project.project_id, updated_templates.dependency_hashes,
        tuple(sorted(report_items, key=lambda item: item.template_id)),
    )

    save_leg_templates(layout.leg_templates_json, updated_templates)
    save_leg_template_instances(layout.leg_template_instances_json, updated_instances)
    save_leg_template_validation_report(layout.leg_template_validation_report_json, updated_report)
    return result


def validate_all_enabled_templates_for_layout(
    layout: ProjectLayout,
    *,
    profile_name: LegOptimizationProfileName = LegOptimizationProfileName.STANDARD,
    expected_template_hashes: dict[str, str] | None = None,
    expected_dependency_hashes: dict[str, str] | None = None,
    cancel_check: Any | None = None,
    progress_callback: Any | None = None,
) -> tuple[LegTemplatesV40, LegTemplateInstancesV40, LegTemplateValidationReportV40]:
    project = load_project(layout.project_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    templates = sync_leg_templates(project, task_config, load_leg_templates(layout.leg_templates_json) if layout.leg_templates_json.exists() else None)
    if expected_dependency_hashes is not None and templates.dependency_hashes != expected_dependency_hashes:
        raise RuntimeError("leg template validation superseded: project dependencies changed")
    if expected_template_hashes is not None:
        actual = {item.template_id: item.template_hash for item in templates.templates if item.enabled and not item.orphaned}
        if actual != expected_template_hashes:
            raise RuntimeError("leg template validation superseded: enabled templates changed")
    updated, instances, report = validate_all_enabled_templates(
        project, task_config, templates, profile_name=profile_name,
        cancel_check=cancel_check, progress_callback=progress_callback,
    )
    _assert_validation_inputs_current(
        layout,
        {item.template_id: item.template_hash for item in templates.templates if item.enabled and not item.orphaned},
        templates.dependency_hashes,
    )
    save_leg_templates(layout.leg_templates_json, updated)
    save_leg_template_instances(layout.leg_template_instances_json, instances)
    save_leg_template_validation_report(layout.leg_template_validation_report_json, report)
    return updated, instances, report


def leg_template_topology_gates(project: ProjectV40, template: LegTemplateV40) -> tuple[dict[str, Any], ...]:
    """Expose the formal gate selection for GUI display without duplicating rules."""

    _profile_id, gates = _topology(project, template)
    return tuple(gate.to_dict() for gate in gates)


def export_leg_template_document(layout: ProjectLayout, document: str, target: str | Path) -> Path:
    sources = {
        "templates": layout.leg_templates_json,
        "instances": layout.leg_template_instances_json,
        "report": layout.leg_template_validation_report_json,
    }
    if document not in sources:
        raise ValueError(f"unknown leg template document: {document}")
    source = sources[document]
    if not source.exists():
        raise FileNotFoundError(f"leg template {document} document does not exist: {source}")
    # Parse before copying so an invalid internal file is never exported.
    if document == "templates":
        load_leg_templates(source)
    elif document == "instances":
        load_leg_template_instances(source)
    else:
        load_leg_template_validation_report(source)
    data = source.read_bytes()
    destination = Path(target)
    atomic_write_bytes(
        destination, data,
        validator=lambda path: _require_identical_bytes(path, data),
    )
    return destination


def export_all_leg_template_documents(layout: ProjectLayout, target_dir: str | Path) -> tuple[Path, Path, Path]:
    directory = Path(target_dir)
    return (
        export_leg_template_document(layout, "templates", directory / "leg_templates.json"),
        export_leg_template_document(layout, "instances", directory / "leg_template_instances.json"),
        export_leg_template_document(layout, "report", directory / "leg_template_validation_report.json"),
    )


def _validate_instance(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    template: LegTemplateV40,
    instance: LegTemplateInstanceV40,
    profile_name: LegOptimizationProfileName,
    cancel_check: Any | None,
) -> LegTemplateInstanceV40:
    topology_profile, gates = _topology(project, template)
    route = task_config.route_families[template.route_family.value]
    yaw_policy = YawPolicy.SHORTEST
    if template.from_site in LOGICAL_DROP_STATIONS and template.to_site in LOGICAL_DROP_STATIONS:
        yaw_policy = YawPolicy(str(route["yaw_direction"]))
    request = LegOptimizationRequest(
        project=project, from_state_id=instance.from_state_key, to_state_id=instance.to_state_key,
        from_pose=Pose2D.from_dict(instance.from_pose, field_name="from_pose"),
        to_pose=Pose2D.from_dict(instance.to_pose, field_name="to_pose"),
        route_family=template.route_family.value, topology_profile=topology_profile, topology_gates=gates,
        dependency_hashes=instance.hashes, profile_name=profile_name, seed=0,
        initial_control_points=tuple(item.to_dict() for item in template.waypoints), yaw_policy=yaw_policy,
        cancel_check=cancel_check,
    )
    result = optimize_leg(request)
    if not result.success or result.leg is None:
        failures = tuple(
            LegTemplateFailureV40(
                code=evaluation.failure_category.value if evaluation.failure_category else "PLANNING_FAILED",
                message=evaluation.failure_reason or result.reason,
                details={"candidate_id": evaluation.candidate_id, "source": evaluation.source},
            )
            for evaluation in result.evaluations if not evaluation.success
        ) or (LegTemplateFailureV40("PLANNING_FAILED", result.reason, {}),)
        return replace(instance, state=LegTemplateInstanceState.FAILED, failures=failures)
    leg = _normalize_compiled_leg(result.leg, template, instance)
    strict = validate_leg(project, leg)
    if not strict["valid"]:
        failures = (LegTemplateFailureV40("STRICT_VALIDATION_FAILED", "compiled leg failed strict validation", strict),)
        return replace(instance, state=LegTemplateInstanceState.FAILED, failures=failures)
    metrics = dict(leg.analysis.get("max_metrics", {}))
    metrics.update({
        "total_length_mm": leg.analysis.get("total_length_mm", 0.0),
        "max_abs_curvature_1_per_mm": _max_abs_discrete_curvature(leg.nodes),
    })
    return replace(
        instance, state=LegTemplateInstanceState.PASSED,
        planned_time_ms=int(leg.analysis.get("planned_time_ms", 0)),
        min_clearance_mm=leg.analysis.get("min_clearance_mm"), analysis_metrics=metrics,
        failures=(), compiled_leg=leg,
    )


def _topology(project: ProjectV40, template: LegTemplateV40) -> tuple[str, tuple[Any, ...]]:
    is_transfer = template.from_site.startswith("P_PICK_") and template.to_site in LOGICAL_DROP_STATIONS
    if not is_transfer:
        return NO_GATE_PROFILE_ID, ()
    route_profile = project.topology_profiles.get(template.route_family.value, {})
    profile_id = str(route_profile.get("transfer_profile_id", f"{template.route_family.value}_TRANSFER"))
    profile = topology_profile_object(project, profile_id, route_family=template.route_family.value)
    return profile_id, topology_gates_from_profile(profile)


def _pose_options(
    project: ProjectV40,
    task_config: CompetitionTaskConfigV40,
    site: str,
) -> tuple[list[tuple[str, dict[str, float], str | None]], list[str]]:
    base = project.sites[site]
    if not bool(base.get("configured")):
        return [], [f"{site}:site_not_configured"]
    if site not in LOGICAL_DROP_STATIONS:
        pose = {key: float(base[key]) for key in ("x_mm", "y_mm", "yaw_ddeg")}
        return [(site, pose, None)], []
    options: list[tuple[str, dict[str, float], str | None]] = []
    missing: list[str] = []
    for profile_id, catalog in sorted(task_config.unload_pose_catalog.items()):
        if str(catalog["station_site"]) != site:
            continue
        profile = project.unload_pose_profiles[profile_id]
        if not bool(profile.get("configured")):
            missing.append(profile_id)
            continue
        pose = {
            "x_mm": float(base["x_mm"]) + float(profile["dx_mm"]),
            "y_mm": float(base["y_mm"]) + float(profile["dy_mm"]),
            "yaw_ddeg": float(profile["yaw_ddeg"]),
        }
        options.append((f"{site}@{profile_id}", pose, profile_id))
    return options, missing


def _template_source_hash(template_id: str, enabled: bool, route: str, from_site: str, to_site: str, waypoints: Iterable[Any]) -> str:
    payload = {"template_id": template_id, "enabled": enabled, "route_family": route, "from_site": from_site,
               "to_site": to_site, "waypoints": [item.to_dict() for item in waypoints]}
    return hashlib.sha256(canonical_json_bytes(payload)).hexdigest()


def _template_validation_hash(template: LegTemplateV40, dependencies: dict[str, str]) -> str:
    return hashlib.sha256(canonical_json_bytes({"template_hash": template.template_hash, "dependencies": dependencies})).hexdigest()


def _validation_input_hash(template: LegTemplateV40, dependencies: dict[str, str], from_key: str, to_key: str,
                           from_pose: dict[str, float], to_pose: dict[str, float]) -> str:
    return hashlib.sha256(canonical_json_bytes({"template_hash": template.template_hash, "dependencies": dependencies,
                                                "from_state_key": from_key, "to_state_key": to_key,
                                                "from_pose": from_pose, "to_pose": to_pose})).hexdigest()


def _normalize_compiled_leg(
    leg: LegV40,
    template: LegTemplateV40,
    instance: LegTemplateInstanceV40,
) -> LegV40:
    analysis = dict(leg.analysis)
    analysis["optimizer_elapsed_ms"] = 0
    analysis["template_id"] = template.template_id
    analysis["template_instance_id"] = instance.instance_id
    hashes = dict(leg.hashes)
    hashes.update({
        "template_id": template.template_id,
        "template_hash": template.template_hash,
        "template_instance_id": instance.instance_id,
    })
    review = dict(leg.review)
    review["template_source"] = True
    return replace(
        leg,
        source="MANUAL_TEMPLATE",
        analysis=analysis,
        hashes=hashes,
        review=review,
    )


def _require_expected_inputs(
    template: LegTemplateV40,
    expected_template_hash: str | None,
    expected_dependency_hashes: dict[str, str] | None,
) -> None:
    if expected_template_hash is not None and template.template_hash != expected_template_hash:
        raise RuntimeError("leg template validation superseded: template changed before start")
    if expected_dependency_hashes is not None and template.dependency_hashes != expected_dependency_hashes:
        raise RuntimeError("leg template validation superseded: project dependencies changed before start")


def _assert_validation_inputs_current(
    layout: ProjectLayout,
    expected_template_hashes: dict[str, str],
    expected_dependency_hashes: dict[str, str],
) -> None:
    project = load_project(layout.project_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    current = load_leg_templates(layout.leg_templates_json) if layout.leg_templates_json.exists() else None
    synchronized = sync_leg_templates(project, task_config, current)
    if synchronized.dependency_hashes != expected_dependency_hashes:
        raise RuntimeError("leg template validation superseded: project dependencies changed during validation")
    current_hashes = {item.template_id: item.template_hash for item in synchronized.templates}
    if any(current_hashes.get(template_id) != value for template_id, value in expected_template_hashes.items()):
        raise RuntimeError("leg template validation superseded: template changed during validation")


def _require_identical_bytes(path: Path, expected: bytes) -> None:
    if path.read_bytes() != expected:
        raise ValueError(f"leg template export write-back mismatch: {path}")


def _stale_instance(instance: LegTemplateInstanceV40) -> LegTemplateInstanceV40:
    return replace(
        instance, state=LegTemplateInstanceState.STALE, planned_time_ms=0,
        min_clearance_mm=None, analysis_metrics={}, failures=(), compiled_leg=None,
    )


def _stale_report_entry(entry: LegTemplateValidationEntryV40) -> LegTemplateValidationEntryV40:
    reports = tuple(
        {
            **report,
            "status": LegTemplateInstanceState.STALE.value,
            "failure_reasons": list(report.get("failure_reasons", ())) + ["template inputs changed; revalidation required"],
        }
        for report in entry.instance_reports
    )
    return replace(
        entry, status=LegTemplateState.STALE,
        instance_counts={"total": len(reports), "passed": 0, "failed": 0, "stale": len(reports)},
        instance_reports=reports,
    )


def _max_abs_discrete_curvature(nodes: tuple[dict[str, Any], ...]) -> float:
    maximum = 0.0
    for first, middle, last in zip(nodes, nodes[1:], nodes[2:]):
        ax, ay = float(middle["x_mm"]) - float(first["x_mm"]), float(middle["y_mm"]) - float(first["y_mm"])
        bx, by = float(last["x_mm"]) - float(middle["x_mm"]), float(last["y_mm"]) - float(middle["y_mm"])
        a, b, chord = math.hypot(ax, ay), math.hypot(bx, by), math.hypot(ax + bx, ay + by)
        if a > 0 and b > 0 and chord > 0:
            maximum = max(maximum, abs(2.0 * (ax * by - ay * bx) / (a * b * chord)))
    return maximum
