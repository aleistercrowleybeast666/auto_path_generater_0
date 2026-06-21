"""Select validated automatic or operator-authored legs for FULL_AUTO cases."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
from typing import Any

from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_leg_template_instances,
    load_leg_templates,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.competition_task_config_service import load_competition_task_config
from hjmb_pathgen.py_services.leg_template_service import (
    compute_leg_template_dependency_hashes,
    leg_template_instance_is_current_and_passed,
)


FULL_AUTO_LEG_SOURCE_POLICY_KEY = "full_auto_leg_source_policy"


class FullAutoLegSourcePolicy(StrEnum):
    AUTO_ONLY = "AUTO_ONLY"
    MANUAL_ONLY = "MANUAL_ONLY"
    BEST_AVAILABLE = "BEST_AVAILABLE"


@dataclass(frozen=True)
class ManualTemplateLeg:
    leg: LegV40
    template_id: str
    instance_id: str

    @property
    def planned_time_ms(self) -> int:
        return int(self.leg.analysis.get("planned_time_ms", 0))


@dataclass(frozen=True)
class EffectiveLegSelection:
    leg: LegV40 | None
    selected_source: str
    selection_reason: str
    automatic_time_ms: int | None = None
    manual_time_ms: int | None = None
    template_id: str = ""
    template_instance_id: str = ""

    def to_ref_metadata(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "selected_source": self.selected_source,
            "selection_reason": self.selection_reason,
        }
        if self.automatic_time_ms is not None:
            result["automatic_time_ms"] = self.automatic_time_ms
        if self.manual_time_ms is not None:
            result["manual_time_ms"] = self.manual_time_ms
        if self.template_id:
            result["template_id"] = self.template_id
        if self.template_instance_id:
            result["template_instance_id"] = self.template_instance_id
        return result


def full_auto_leg_source_policy(project: ProjectV40) -> FullAutoLegSourcePolicy:
    raw = project.planner_profiles.get("default", {}).get(
        FULL_AUTO_LEG_SOURCE_POLICY_KEY,
        FullAutoLegSourcePolicy.BEST_AVAILABLE.value,
    )
    try:
        return FullAutoLegSourcePolicy(str(raw))
    except ValueError:
        return FullAutoLegSourcePolicy.BEST_AVAILABLE


def load_current_manual_template_legs(
    layout: ProjectLayout,
    project: ProjectV40,
) -> dict[str, ManualTemplateLeg]:
    """Return current PASSED template legs indexed by formal directed leg_id.

    This function is read-only.  It never synchronizes, validates, or writes the
    template documents while a FULL_AUTO job is running.
    """

    if not layout.leg_templates_json.exists() or not layout.leg_template_instances_json.exists():
        return {}
    templates = load_leg_templates(layout.leg_templates_json)
    instances = load_leg_template_instances(layout.leg_template_instances_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    dependencies = compute_leg_template_dependency_hashes(project, task_config)
    if templates.dependency_hashes != dependencies or instances.dependency_hashes != dependencies:
        return {}
    template_by_id = {item.template_id: item for item in templates.templates}
    result: dict[str, ManualTemplateLeg] = {}
    for instance in instances.instances:
        template = template_by_id.get(instance.template_id)
        if template is None or not template.enabled or template.orphaned:
            continue
        if not leg_template_instance_is_current_and_passed(instance, template, dependencies):
            continue
        if instance.compiled_leg is None:
            continue
        candidate = ManualTemplateLeg(
            leg=instance.compiled_leg,
            template_id=template.template_id,
            instance_id=instance.instance_id,
        )
        old = result.get(candidate.leg.leg_id)
        if old is None or candidate.planned_time_ms < old.planned_time_ms:
            result[candidate.leg.leg_id] = candidate
    return result



def effective_library_for_case_refs(
    layout: ProjectLayout,
    project: ProjectV40,
    base_library: LegLibraryV40,
    case: CaseManifestV40,
) -> LegLibraryV40:
    """Rebuild the exact library selected by a previously generated FULL_AUTO case.

    Automatic and manual-template legs deliberately share the same formal
    ``leg_id``.  The persistent automatic library therefore remains untouched;
    manual selections are overlaid only while compiling/exporting the case that
    explicitly references them.  Current template hashes are rechecked so stale
    operator geometry can never be exported silently.
    """

    manual_refs = [
        dict(ref) for ref in case.leg_refs
        if str(ref.get("selected_source", "")).upper() == "MANUAL_TEMPLATE"
    ]
    if not manual_refs:
        return base_library
    if not layout.leg_templates_json.exists() or not layout.leg_template_instances_json.exists():
        raise ValueError("case references manual Leg templates, but template documents are missing")

    templates = load_leg_templates(layout.leg_templates_json)
    instances = load_leg_template_instances(layout.leg_template_instances_json)
    task_config = load_competition_task_config(layout.competition_task_config_json)
    dependencies = compute_leg_template_dependency_hashes(project, task_config)
    if templates.dependency_hashes != dependencies or instances.dependency_hashes != dependencies:
        raise ValueError("manual Leg template dependencies are stale; revalidate templates before export")

    template_by_id = {item.template_id: item for item in templates.templates}
    instance_by_id = {item.instance_id: item for item in instances.instances}
    by_id = {item.leg_id: item for item in base_library.legs}
    for ref in manual_refs:
        leg_id = str(ref.get("leg_id", ""))
        instance_id = str(ref.get("template_instance_id", ""))
        template_id = str(ref.get("template_id", ""))
        instance = instance_by_id.get(instance_id)
        template = template_by_id.get(template_id)
        if instance is None or template is None or instance.template_id != template.template_id:
            raise ValueError(f"manual Leg template reference is missing: {template_id}/{instance_id}")
        if not template.enabled or template.orphaned:
            raise ValueError(f"manual Leg template is disabled or orphaned: {template_id}")
        if not leg_template_instance_is_current_and_passed(instance, template, dependencies):
            raise ValueError(f"manual Leg template instance is stale or failed: {instance_id}")
        if instance.compiled_leg is None or instance.compiled_leg.leg_id != leg_id:
            raise ValueError(f"manual Leg template instance does not match case leg_id: {instance_id}")
        expected = str(ref.get("expected_leg_hash32", ""))
        actual = str(instance.compiled_leg.hashes.get("self_hash32", ""))
        if expected and expected != actual:
            raise ValueError(f"manual Leg template hash changed for {instance_id}; regenerate the case")
        by_id[leg_id] = instance.compiled_leg
    return replace(base_library, legs=tuple(sorted(by_id.values(), key=lambda item: item.leg_id)))

def choose_effective_leg(
    policy: FullAutoLegSourcePolicy,
    *,
    automatic_leg: LegV40 | None,
    automatic_reusable: bool,
    manual_leg: ManualTemplateLeg | None,
    manual_reusable: bool,
) -> EffectiveLegSelection:
    auto_time = (
        int(automatic_leg.analysis.get("planned_time_ms", 0))
        if automatic_leg is not None and automatic_reusable
        else None
    )
    manual_time = manual_leg.planned_time_ms if manual_leg is not None and manual_reusable else None

    if policy == FullAutoLegSourcePolicy.AUTO_ONLY:
        return EffectiveLegSelection(
            leg=automatic_leg if automatic_reusable else None,
            selected_source="AUTOMATIC" if automatic_reusable else "MISSING",
            selection_reason="AUTO_ONLY" if automatic_reusable else "AUTOMATIC_UNAVAILABLE",
            automatic_time_ms=auto_time,
            manual_time_ms=manual_time,
        )
    if policy == FullAutoLegSourcePolicy.MANUAL_ONLY:
        return _manual_selection(
            manual_leg if manual_reusable else None,
            auto_time=auto_time,
            manual_time=manual_time,
            reason="MANUAL_ONLY" if manual_reusable else "MANUAL_TEMPLATE_UNAVAILABLE",
        )

    if manual_reusable and manual_leg is not None and (
        not automatic_reusable or automatic_leg is None or manual_time <= auto_time
    ):
        return _manual_selection(
            manual_leg,
            auto_time=auto_time,
            manual_time=manual_time,
            reason="ONLY_VALID_SOURCE" if not automatic_reusable else "FASTER_OR_EQUAL_THAN_AUTOMATIC",
        )
    if automatic_reusable and automatic_leg is not None:
        return EffectiveLegSelection(
            leg=automatic_leg,
            selected_source="AUTOMATIC",
            selection_reason="ONLY_VALID_SOURCE" if not manual_reusable else "FASTER_THAN_MANUAL_TEMPLATE",
            automatic_time_ms=auto_time,
            manual_time_ms=manual_time,
        )
    return EffectiveLegSelection(
        leg=None,
        selected_source="MISSING",
        selection_reason="NO_VALID_AUTOMATIC_OR_MANUAL_LEG",
        automatic_time_ms=auto_time,
        manual_time_ms=manual_time,
    )


def _manual_selection(
    manual: ManualTemplateLeg | None,
    *,
    auto_time: int | None,
    manual_time: int | None,
    reason: str,
) -> EffectiveLegSelection:
    return EffectiveLegSelection(
        leg=manual.leg if manual is not None else None,
        selected_source="MANUAL_TEMPLATE" if manual is not None else "MISSING",
        selection_reason=reason,
        automatic_time_ms=auto_time,
        manual_time_ms=manual_time,
        template_id=manual.template_id if manual is not None else "",
        template_instance_id=manual.instance_id if manual is not None else "",
    )


__all__ = [
    "FULL_AUTO_LEG_SOURCE_POLICY_KEY",
    "EffectiveLegSelection",
    "FullAutoLegSourcePolicy",
    "ManualTemplateLeg",
    "choose_effective_leg",
    "effective_library_for_case_refs",
    "full_auto_leg_source_policy",
    "load_current_manual_template_legs",
]
