"""Shared V4 GUI state loaded from a project directory."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from hjmb_pathgen.codec.json_codec import load_case, load_leg_library, load_project, load_route_case_table
from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.models.leg import LegLibraryV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.models.protocol import DROP_SITE_KEYS, PICKUP_SITE_KEYS, REQUIRED_SITE_KEYS
from hjmb_pathgen.models.route_case import CaseManifestV40, RouteCaseTableV40
from hjmb_pathgen.services.project_service import ProjectLayout


ROTATABLE_SITE_KEYS = tuple(PICKUP_SITE_KEYS)
SITE_KEYS = tuple(REQUIRED_SITE_KEYS)


@dataclass
class ManualPointDraft:
    point_type: str
    x_mm: int
    y_mm: int
    yaw_ddeg: int | None = None
    exact_pass: bool = True

    def has_yaw(self) -> bool:
        return self.point_type in {"START", "ARRIVAL"}


@dataclass
class LoadedProjectState:
    layout: ProjectLayout
    project: ProjectV40
    route_table: RouteCaseTableV40 | None
    leg_library: LegLibraryV40 | None
    task_cases: dict[int, CaseManifestV40] = field(default_factory=dict)
    manual_cases: dict[int, CaseManifestV40] = field(default_factory=dict)
    final_bins: dict[int, Path] = field(default_factory=dict)
    reports: list[Path] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @classmethod
    def load(cls, root: str | Path) -> "LoadedProjectState":
        layout = ProjectLayout.open(root, create_dirs=False)
        project = load_project(layout.project_json)
        warnings: list[str] = []
        route_table = None
        leg_library = None
        if layout.route_case_table_json.exists():
            try:
                route_table = load_route_case_table(layout.route_case_table_json)
            except Exception as exc:  # noqa: BLE001 - GUI should preserve partial diagnostics.
                warnings.append(f"route_case_table.json 加载失败: {exc}")
        else:
            warnings.append("缺少 route_case_table.json")
        if layout.leg_library_json.exists():
            try:
                leg_library = load_leg_library(layout.leg_library_json)
            except Exception as exc:  # noqa: BLE001
                warnings.append(f"leg_library.json 加载失败: {exc}")
        else:
            warnings.append("缺少 leg_library.json")
        task_cases = _load_cases(layout.task_compiled_cases_dir, warnings, PathSource.TASK_COMPILED)
        manual_cases = _load_cases(layout.manual_free_cases_dir, warnings, PathSource.MANUAL_FREE)
        # Legacy flat cases are visible as task candidates only when mode dirs are empty.
        if not task_cases and layout.cases_dir.exists():
            task_cases.update(_load_cases(layout.cases_dir, warnings, PathSource.TASK_COMPILED, flat_only=True))
        final_bins = _scan_bins(layout.final_bin_dir)
        reports = sorted(path for path in layout.reports_dir.glob("**/*") if path.is_file()) if layout.reports_dir.exists() else []
        return cls(
            layout=layout,
            project=project,
            route_table=route_table,
            leg_library=leg_library,
            task_cases=task_cases,
            manual_cases=manual_cases,
            final_bins=final_bins,
            reports=reports,
            warnings=warnings,
        )

    def current_case(self, traj_id: int | None = None, source: PathSource = PathSource.TASK_COMPILED) -> CaseManifestV40 | None:
        cases = self.task_cases if source == PathSource.TASK_COMPILED else self.manual_cases
        if not cases:
            return None
        if traj_id is not None and traj_id in cases:
            return cases[traj_id]
        return cases[sorted(cases)[0]]


def site_kind(site_key: str) -> str:
    if site_key in PICKUP_SITE_KEYS:
        return "pickup_pose"
    if site_key in DROP_SITE_KEYS:
        return "drop_site"
    return "unknown"


def site_has_yaw(site_key: str) -> bool:
    return site_key in ROTATABLE_SITE_KEYS


def site_label(site_key: str) -> str:
    return site_key.replace("P_", "").replace("F_", "")


def _load_cases(
    directory: Path,
    warnings: list[str],
    source: PathSource,
    *,
    flat_only: bool = False,
) -> dict[int, CaseManifestV40]:
    result: dict[int, CaseManifestV40] = {}
    if not directory.exists():
        return result
    for path in sorted(directory.glob("P*.json")):
        if flat_only and path.parent.name in {"task_compiled", "manual_free"}:
            continue
        try:
            case = load_case(path)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"{path.name} 加载失败: {exc}")
            continue
        if case.path_source == source:
            result[case.traj_id] = case
    return result


def _scan_bins(directory: Path) -> dict[int, Path]:
    result: dict[int, Path] = {}
    if not directory.exists():
        return result
    for path in sorted(directory.glob("P*.BIN")):
        try:
            result[int(path.stem[1:])] = path
        except ValueError:
            continue
    return result


def project_summary(project: ProjectV40) -> dict[str, Any]:
    return {
        "project_id": project.project_id,
        "site_count": len(project.sites),
        "cylinders": len(project.field_objects.get("cylinders", [])),
        "pickup_boxes": len(project.field_objects.get("pickup_boxes", [])),
        "drop_boxes": len(project.field_objects.get("drop_boxes", [])),
    }
