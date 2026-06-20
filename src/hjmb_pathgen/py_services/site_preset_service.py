"""Site pose preset import/export services."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.json_codec import load_project, load_site_pose_preset, save_project, save_site_pose_preset
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.site_preset import SitePosePresetV40
from hjmb_pathgen.py_services.project_config_service import compute_project_functional_hashes
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout


@dataclass(frozen=True)
class SitePresetDiff:
    site_key: str
    before: dict[str, Any]
    after: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {"site_key": self.site_key, "before": dict(self.before), "after": dict(self.after)}


@dataclass(frozen=True)
class SitePresetPreview:
    preset: SitePosePresetV40
    diffs: tuple[SitePresetDiff, ...]
    current_site_config_hash: str
    preset_site_config_hash: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "preset_name": self.preset.preset_name,
            "diff_count": len(self.diffs),
            "diffs": [diff.to_dict() for diff in self.diffs],
            "current_site_config_hash": self.current_site_config_hash,
            "preset_site_config_hash": self.preset_site_config_hash,
        }


@dataclass(frozen=True)
class SitePresetApplyResult:
    project: ProjectV40
    preview: SitePresetPreview
    new_functional_hashes: dict[str, str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "applied": True,
            "preview": self.preview.to_dict(),
            "new_functional_hashes": dict(self.new_functional_hashes),
        }


def export_site_pose_preset(layout: ProjectLayout, preset_name: str, *, notes: str = "") -> Path:
    layout.ensure_directories()
    project = load_project(layout.project_json)
    preset = SitePosePresetV40(
        preset_name=preset_name,
        site_pose_provider=dict(project.site_pose_provider),
        sites=dict(project.sites),
        hashes={"site_config_hash": _site_hash(project.sites, project.site_pose_provider)},
        notes=notes,
    )
    path = layout.site_preset_path(preset_name)
    save_site_pose_preset(path, preset)
    return path


def import_site_pose_preset_preview(layout: ProjectLayout, preset_path: str | Path) -> SitePresetPreview:
    project = load_project(layout.project_json)
    preset = load_site_pose_preset(preset_path)
    diffs = tuple(
        SitePresetDiff(site_key=key, before=dict(project.sites[key]), after=dict(preset.sites[key]))
        for key in project.sites
        if project.sites[key] != preset.sites[key]
    )
    return SitePresetPreview(
        preset=preset,
        diffs=diffs,
        current_site_config_hash=_site_hash(project.sites, project.site_pose_provider),
        preset_site_config_hash=_site_hash(preset.sites, preset.site_pose_provider),
    )


def apply_site_pose_preset(layout: ProjectLayout, preset_path: str | Path) -> SitePresetApplyResult:
    project = load_project(layout.project_json)
    preview = import_site_pose_preset_preview(layout, preset_path)
    updated = ProjectV40(
        project_id=project.project_id,
        protocol_version=project.protocol_version,
        nominal_field=dict(project.nominal_field),
        coordinate_system=dict(project.coordinate_system),
        site_pose_provider=dict(preview.preset.site_pose_provider),
        sites=dict(preview.preset.sites),
        field_objects=dict(project.field_objects),
        vehicle=dict(project.vehicle),
        dynamics=dict(project.dynamics),
        unload_profiles=dict(project.unload_profiles),
        topology_profiles=dict(project.topology_profiles),
        action_profiles=dict(project.action_profiles),
        planner_profiles=dict(project.planner_profiles),
        start_check=dict(project.start_check),
        arrival_check=dict(project.arrival_check),
        finish_policy=dict(project.finish_policy),
        output=dict(project.output),
        traj_table=dict(project.traj_table),
    )
    save_project(layout.project_json, updated)
    return SitePresetApplyResult(
        project=updated,
        preview=preview,
        new_functional_hashes=compute_project_functional_hashes(updated),
    )


def _site_hash(sites: dict[str, Any], provider: dict[str, Any]) -> str:
    return canonical_json_crc32_hex({"site_pose_provider": provider, "sites": sites})
