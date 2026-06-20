"""V4.0 project directory layout services."""

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_case,
    load_leg_library,
    load_project,
    load_route_case_table,
    save_leg_library,
    save_project,
)
from hjmb_pathgen.py_domain.enums import LegState, GenerationMode
from hjmb_pathgen.py_domain.errors import ProjectLayoutError
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.protocol import (
    DIR_BIN,
    DIR_CACHE,
    DIR_CASES,
    DIR_DRAFTS,
    DIR_FINAL,
    DIR_MANUAL,
    DIR_PORTABLE,
    DIR_PRESETS,
    DIR_REPORTS,
    DIR_SEMI_AUTO,
    DIR_FULL_AUTO,
)

from hjmb_pathgen.py_io.layout.path_naming import bin_name, case_json_name, portable_name, validate_traj_id

PROJECT_JSON = "project.json"
ROUTE_CASE_TABLE_JSON = "route_case_table.json"
LEG_LIBRARY_JSON = "leg_library.json"
TRAJ_ID_CSV = "traj_id.csv"  # legacy import only
TASK_CONFIG_DIR = "task_config"
COMPETITION_TASK_CONFIG_JSON = "competition_task_config.json"
OPTIMIZATION_LOG_DIR = "optimization_log"
REUSABLE_LEG_STATES = {LegState.VALID, LegState.APPROVED, LegState.LOCKED}


class ProjectStatus(StrEnum):
    INITIALIZED = "INITIALIZED"
    INCOMPLETE_MAPPING = "INCOMPLETE_MAPPING"
    INCOMPLETE_LIBRARY = "INCOMPLETE_LIBRARY"
    READY_FOR_SINGLE_CASE = "READY_FOR_SINGLE_CASE"
    READY_FOR_BATCH = "READY_FOR_BATCH"
    INVALID = "INVALID"


@dataclass(frozen=True)
class ProjectStatusReport:
    status: ProjectStatus
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ProjectLayout:
    root: Path

    @classmethod
    def create(cls, root: str | Path, project: ProjectV40) -> "ProjectLayout":
        layout = cls.open(root, create_dirs=True)
        layout.ensure_directories()
        save_project(layout.project_json, project)
        empty_library = LegLibraryV40(
            planner_version="4.0.0",
            project_hash=canonical_json_crc32_hex(project.to_dict()),
            legs=(),
        )
        save_leg_library(layout.leg_library_json, empty_library)
        return layout

    @classmethod
    def open(cls, root: str | Path, *, create_dirs: bool = False) -> "ProjectLayout":
        path = Path(root)
        if create_dirs:
            path.mkdir(parents=True, exist_ok=True)
        if not path.exists() or not path.is_dir():
            raise ProjectLayoutError(f"project root is not a directory: {path}")
        return cls(root=path.resolve(strict=False))

    @property
    def project_json(self) -> Path:
        return self.resolve_project_path(PROJECT_JSON)

    @property
    def route_case_table_json(self) -> Path:
        return self.resolve_project_path(ROUTE_CASE_TABLE_JSON)

    @property
    def leg_library_json(self) -> Path:
        return self.resolve_project_path(LEG_LIBRARY_JSON)

    @property
    def traj_id_csv(self) -> Path:
        return self.resolve_project_path(TRAJ_ID_CSV)

    @property
    def task_config_dir(self) -> Path:
        return self.resolve_project_path(TASK_CONFIG_DIR)

    @property
    def competition_task_config_json(self) -> Path:
        return self.resolve_project_path(Path(TASK_CONFIG_DIR) / COMPETITION_TASK_CONFIG_JSON)

    @property
    def cases_dir(self) -> Path:
        return self.resolve_project_path(DIR_CASES)

    @property
    def full_auto_cases_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_CASES) / DIR_FULL_AUTO)

    @property
    def semi_auto_cases_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_CASES) / DIR_SEMI_AUTO)

    @property
    def manual_cases_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_CASES) / DIR_MANUAL)

    @property
    def bin_dir(self) -> Path:
        return self.resolve_project_path(DIR_BIN)

    @property
    def full_auto_bin_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / DIR_FULL_AUTO)

    @property
    def semi_auto_bin_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / DIR_SEMI_AUTO)

    @property
    def manual_bin_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / DIR_MANUAL)

    @property
    def final_bin_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / DIR_FINAL)

    @property
    def portable_dir(self) -> Path:
        return self.resolve_project_path(DIR_PORTABLE)

    @property
    def full_auto_portable_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_PORTABLE) / DIR_FULL_AUTO)

    @property
    def semi_auto_portable_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_PORTABLE) / DIR_SEMI_AUTO)

    @property
    def manual_portable_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_PORTABLE) / DIR_MANUAL)

    @property
    def reports_dir(self) -> Path:
        return self.resolve_project_path(DIR_REPORTS)

    @property
    def presets_dir(self) -> Path:
        return self.resolve_project_path(DIR_PRESETS)

    @property
    def drafts_dir(self) -> Path:
        return self.resolve_project_path(DIR_DRAFTS)

    @property
    def optimization_log_dir(self) -> Path:
        return self.resolve_project_path(Path(DIR_REPORTS) / OPTIMIZATION_LOG_DIR)

    @property
    def cache_dir(self) -> Path:
        return self.resolve_project_path(DIR_CACHE)

    def legacy_flat_case_json_path(self, traj_id: int) -> Path:
        return self.resolve_project_path(Path(DIR_CASES) / case_json_name(traj_id))

    def case_json_path_for_mode(self, traj_id: int, generation_mode: GenerationMode | str) -> Path:
        return self.resolve_project_path(Path(DIR_CASES) / _generation_mode_dir(generation_mode) / case_json_name(traj_id))

    def legacy_flat_bin_path(self, traj_id: int) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / bin_name(traj_id))

    def bin_path_for_mode(self, traj_id: int, generation_mode: GenerationMode | str) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / _generation_mode_dir(generation_mode) / bin_name(traj_id))

    def final_bin_path(self, traj_id: int) -> Path:
        return self.resolve_project_path(Path(DIR_BIN) / DIR_FINAL / bin_name(traj_id))

    def legacy_flat_portable_path(self, traj_id: int) -> Path:
        return self.resolve_project_path(Path(DIR_PORTABLE) / portable_name(traj_id))

    def portable_path_for_mode(self, traj_id: int, generation_mode: GenerationMode | str) -> Path:
        return self.resolve_project_path(Path(DIR_PORTABLE) / _generation_mode_dir(generation_mode) / portable_name(traj_id))

    def site_preset_path(self, preset_name: str) -> Path:
        safe = _safe_preset_name(preset_name)
        return self.resolve_project_path(Path(DIR_PRESETS) / f"{safe}.site_poses.json")

    def ensure_directories(self) -> None:
        for path in (
            self.cases_dir,
            self.manual_cases_dir,
            self.semi_auto_cases_dir,
            self.full_auto_cases_dir,
            self.bin_dir,
            self.manual_bin_dir,
            self.semi_auto_bin_dir,
            self.full_auto_bin_dir,
            self.final_bin_dir,
            self.portable_dir,
            self.manual_portable_dir,
            self.semi_auto_portable_dir,
            self.full_auto_portable_dir,
            self.reports_dir,
            self.optimization_log_dir,
            self.cache_dir,
            self.presets_dir,
            self.drafts_dir,
            self.task_config_dir,
        ):
            path.mkdir(parents=True, exist_ok=True)

    def resolve_project_path(self, relative_path: str | Path) -> Path:
        relative = Path(relative_path)
        if relative.is_absolute():
            raise ProjectLayoutError(f"absolute project-relative path is not allowed: {relative}")
        candidate = (self.root / relative).resolve(strict=False)
        if not _path_is_inside(candidate, self.root):
            raise ProjectLayoutError(f"project path escapes root: {relative}")
        return candidate

    def status(self) -> ProjectStatusReport:
        reasons: list[str] = []
        if not self.project_json.exists():
            return ProjectStatusReport(ProjectStatus.INVALID, ("missing project.json",))
        try:
            load_project(self.project_json)
        except Exception as exc:  # noqa: BLE001 - status report should include any loader reason.
            return ProjectStatusReport(ProjectStatus.INVALID, (f"invalid project.json: {exc}",))

        for path in (
            self.cases_dir,
            self.bin_dir,
            self.portable_dir,
            self.reports_dir,
            self.cache_dir,
            self.presets_dir,
            self.drafts_dir,
            self.task_config_dir,
        ):
            if not path.exists() or not path.is_dir():
                reasons.append(f"missing directory: {path.relative_to(self.root)}")
        if reasons:
            return ProjectStatusReport(ProjectStatus.INVALID, tuple(reasons))

        if not self.route_case_table_json.exists():
            reasons.append("route_case_table.json is not present; Phase 2 does not fake 360 mappings")
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_MAPPING, tuple(reasons))
        try:
            table = load_route_case_table(self.route_case_table_json)
        except Exception as exc:  # noqa: BLE001
            return ProjectStatusReport(ProjectStatus.INVALID, (f"invalid route_case_table.json: {exc}",))

        if not self.leg_library_json.exists():
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, ("missing leg_library.json",))
        try:
            library = load_leg_library(self.leg_library_json)
        except Exception as exc:  # noqa: BLE001
            return ProjectStatusReport(ProjectStatus.INVALID, (f"invalid leg_library.json: {exc}",))
        valid_leg_ids = {leg.leg_id for leg in library.legs if leg.state in REUSABLE_LEG_STATES}
        if not valid_leg_ids:
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, ("no VALID legs in leg_library.json",))

        ready_cases = []
        case_paths = (
            list(self.manual_cases_dir.glob("P*.json"))
            + list(self.semi_auto_cases_dir.glob("P*.json"))
            + list(self.full_auto_cases_dir.glob("P*.json"))
        )
        for path in sorted(case_paths):
            try:
                case = load_case(path)
            except Exception:
                continue
            refs = [str(ref.get("leg_id", "")) for ref in case.leg_refs]
            if refs and all(ref in valid_leg_ids for ref in refs):
                ready_cases.append(case.traj_id)

        if len(table.cases) == 360 and len(ready_cases) == 360:
            return ProjectStatusReport(ProjectStatus.READY_FOR_BATCH, ("all 360 referenced cases have VALID legs",))
        if ready_cases:
            return ProjectStatusReport(ProjectStatus.READY_FOR_SINGLE_CASE, (f"ready traj_id count: {len(ready_cases)}",))
        return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, ("no case has complete VALID leg dependencies",))

    def status_for_case(
        self,
        traj_id: int,
        generation_mode: GenerationMode = GenerationMode.FULL_AUTO,
    ) -> ProjectStatusReport:
        traj_id = validate_traj_id(traj_id)
        case_path = self.case_json_path_for_mode(traj_id, generation_mode)
        if not case_path.exists():
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, (f"missing case JSON: {case_path.name}",))
        if not self.leg_library_json.exists():
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, ("missing leg_library.json",))
        try:
            case = load_case(case_path)
            library = load_leg_library(self.leg_library_json)
        except Exception as exc:  # noqa: BLE001
            return ProjectStatusReport(ProjectStatus.INVALID, (str(exc),))
        valid_leg_ids = {leg.leg_id for leg in library.legs if leg.state in REUSABLE_LEG_STATES}
        missing = [str(ref.get("leg_id", "")) for ref in case.leg_refs if str(ref.get("leg_id", "")) not in valid_leg_ids]
        if missing:
            return ProjectStatusReport(ProjectStatus.INCOMPLETE_LIBRARY, (f"missing or non-VALID legs: {missing}",))
        return ProjectStatusReport(ProjectStatus.READY_FOR_SINGLE_CASE, (f"P{traj_id:04d} dependencies are present",))


def _path_is_inside(path: Path, root: Path) -> bool:
    root_text = os.path.normcase(str(root.resolve(strict=False)))
    path_text = os.path.normcase(str(path.resolve(strict=False)))
    try:
        return os.path.commonpath([root_text, path_text]) == root_text
    except ValueError:
        return False


def _safe_preset_name(preset_name: str) -> str:
    text = preset_name.strip()
    if not text:
        raise ProjectLayoutError("site preset name must not be empty")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")
    if any(ch not in allowed for ch in text):
        raise ProjectLayoutError(f"site preset name contains unsupported characters: {preset_name!r}")
    if text in {".", ".."} or text.endswith(".site_poses"):
        raise ProjectLayoutError(f"site preset name is not valid: {preset_name!r}")
    return text


def _generation_mode_dir(generation_mode: GenerationMode | str) -> str:
    mode = generation_mode if isinstance(generation_mode, GenerationMode) else GenerationMode(str(generation_mode))
    if mode == GenerationMode.MANUAL:
        return DIR_MANUAL
    if mode == GenerationMode.SEMI_AUTO:
        return DIR_SEMI_AUTO
    if mode == GenerationMode.FULL_AUTO:
        return DIR_FULL_AUTO
    raise ProjectLayoutError(f"unsupported generation_mode: {generation_mode!r}")
