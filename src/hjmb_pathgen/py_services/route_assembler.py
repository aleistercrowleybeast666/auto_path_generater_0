"""Phase 6 wrappers around task transition extraction."""

from __future__ import annotations

from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.task_plan import TransitionRequirement
from hjmb_pathgen.py_services.task_compiler import transition_requirements_for_case


def transition_requirements_from_case(case: CaseManifestV40, project: ProjectV40) -> tuple[TransitionRequirement, ...]:
    return transition_requirements_for_case(case, project)
