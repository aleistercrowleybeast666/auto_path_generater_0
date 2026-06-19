"""V4.0 site pose preset model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.codec.legacy_rejection import reject_deleted_fields, reject_legacy_format

from .errors import expect_equal, reject_unknown_fields, require_fields
from .project import validate_project_sites
from .protocol import SITE_POSE_PRESET_FORMAT

SITE_PRESET_FIELDS = {
    "format",
    "preset_name",
    "protocol_version",
    "site_pose_provider",
    "sites",
    "hashes",
    "notes",
}


@dataclass(frozen=True)
class SitePosePresetV40:
    preset_name: str
    site_pose_provider: dict[str, Any]
    sites: dict[str, Any]
    hashes: dict[str, Any]
    notes: str = ""
    protocol_version: int = 40
    format: str = SITE_POSE_PRESET_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SitePosePresetV40":
        reject_deleted_fields(data, "SitePosePresetV40")
        reject_legacy_format(data.get("format"), "SitePosePresetV40")
        reject_unknown_fields(data, SITE_PRESET_FIELDS, "SitePosePresetV40")
        require_fields(data, SITE_PRESET_FIELDS - {"notes"}, "SitePosePresetV40")
        expect_equal(data["format"], SITE_POSE_PRESET_FORMAT, "SitePosePresetV40", "format")
        expect_equal(data["protocol_version"], 40, "SitePosePresetV40", "protocol_version")
        provider = dict(data["site_pose_provider"])
        expect_equal(provider.get("type"), "MANUAL", "SitePosePresetV40", "site_pose_provider.type")
        return cls(
            preset_name=str(data["preset_name"]),
            protocol_version=int(data["protocol_version"]),
            site_pose_provider=provider,
            sites=validate_project_sites(data["sites"], "SitePosePresetV40", "sites"),
            hashes=dict(data["hashes"]),
            notes=str(data.get("notes", "")),
        )

    def to_dict(self) -> dict[str, Any]:
        data = {
            "format": self.format,
            "preset_name": self.preset_name,
            "protocol_version": self.protocol_version,
            "site_pose_provider": self.site_pose_provider,
            "sites": self.sites,
            "hashes": self.hashes,
        }
        if self.notes:
            data["notes"] = self.notes
        return data
