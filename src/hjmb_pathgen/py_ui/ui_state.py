"""Shared V4 GUI state loaded from a project directory."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project, load_route_case_table
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import PICKUP_SITE_KEYS, REQUIRED_SITE_KEYS
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, RouteCaseTableV40
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


ROTATABLE_SITE_KEYS = tuple(PICKUP_SITE_KEYS)
SITE_KEYS = tuple(REQUIRED_SITE_KEYS)


@dataclass
class ManualPointDraft:
    point_type: str
    x_mm: int
    y_mm: int
    yaw_ddeg: int | None = None
    exact_pass: bool = True
    point_id: str | None = None

    def has_yaw(self) -> bool:
        return self.point_type in {"START", "ARRIVAL", "TASK_ANCHOR"}


@dataclass
class LoadedProjectState:
    layout: ProjectLayout
    project: ProjectV40
    route_table: RouteCaseTableV40 | None
    leg_library: LegLibraryV40 | None
    full_auto_cases: dict[int, CaseManifestV40] = field(default_factory=dict)
    semi_auto_cases: dict[int, CaseManifestV40] = field(default_factory=dict)
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
        full_auto_cases = _load_cases(layout.full_auto_cases_dir, warnings, GenerationMode.FULL_AUTO)
        semi_auto_cases = _load_cases(layout.semi_auto_cases_dir, warnings, GenerationMode.SEMI_AUTO)
        manual_cases = _load_cases(layout.manual_cases_dir, warnings, GenerationMode.MANUAL)
        final_bins = _scan_bins(layout.final_bin_dir)
        reports = sorted(path for path in layout.reports_dir.glob("**/*") if path.is_file()) if layout.reports_dir.exists() else []
        return cls(
            layout=layout,
            project=project,
            route_table=route_table,
            leg_library=leg_library,
            full_auto_cases=full_auto_cases,
            semi_auto_cases=semi_auto_cases,
            manual_cases=manual_cases,
            final_bins=final_bins,
            reports=reports,
            warnings=warnings,
        )

    def current_case(
        self,
        traj_id: int | None = None,
        mode: GenerationMode = GenerationMode.FULL_AUTO,
    ) -> CaseManifestV40 | None:
        cases = {
            GenerationMode.MANUAL: self.manual_cases,
            GenerationMode.SEMI_AUTO: self.semi_auto_cases,
            GenerationMode.FULL_AUTO: self.full_auto_cases,
        }[mode]
        if not cases:
            return None
        if traj_id is not None:
            return cases.get(traj_id)
        return cases[sorted(cases)[0]]


def site_kind(site_key: str) -> str:
    if site_key in PICKUP_SITE_KEYS:
        return "pickup_pose"
    return "unknown"


def site_has_yaw(site_key: str) -> bool:
    return site_key in ROTATABLE_SITE_KEYS


def site_label(site_key: str) -> str:
    return site_key.replace("P_", "").replace("F_", "")


def _load_cases(
    directory: Path,
    warnings: list[str],
    mode: GenerationMode,
) -> dict[int, CaseManifestV40]:
    result: dict[int, CaseManifestV40] = {}
    if not directory.exists():
        return result
    for path in sorted(directory.glob("P*.json")):
        try:
            case = load_case(path)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"{path.name} 加载失败: {exc}")
            continue
        if case.generation_mode == mode:
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
