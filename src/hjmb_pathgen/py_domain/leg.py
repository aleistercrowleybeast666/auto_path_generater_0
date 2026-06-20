"""V4.0 reusable directed leg library model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.py_io.codecs.legacy_rejection import reject_deleted_fields, reject_legacy_format

from .enums import LegState
from .errors import expect_equal, reject_unknown_fields, require_fields
from .protocol import LEG_LIBRARY_FORMAT

LEG_FIELDS = {
    "leg_id",
    "key",
    "state",
    "source",
    "topology_profile",
    "control_points",
    "yaw_profile",
    "nodes",
    "analysis",
    "hashes",
    "review",
}
LIBRARY_FIELDS = {"format", "planner_version", "project_hash", "legs"}


@dataclass(frozen=True)
class LegV40:
    leg_id: str
    key: dict[str, Any]
    state: LegState
    source: str
    topology_profile: str
    control_points: tuple[dict[str, Any], ...]
    yaw_profile: dict[str, Any]
    nodes: tuple[dict[str, Any], ...]
    analysis: dict[str, Any]
    hashes: dict[str, Any]
    review: dict[str, Any]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegV40":
        reject_unknown_fields(data, LEG_FIELDS, "LegV40")
        require_fields(data, LEG_FIELDS, "LegV40")
        return cls(
            leg_id=str(data["leg_id"]),
            key=dict(data["key"]),
            state=LegState(str(data["state"])),
            source=str(data["source"]),
            topology_profile=str(data["topology_profile"]),
            control_points=tuple(dict(item) for item in data["control_points"]),
            yaw_profile=dict(data["yaw_profile"]),
            nodes=tuple(dict(item) for item in data["nodes"]),
            analysis=dict(data["analysis"]),
            hashes=dict(data["hashes"]),
            review=dict(data["review"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "leg_id": self.leg_id,
            "key": self.key,
            "state": self.state.value,
            "source": self.source,
            "topology_profile": self.topology_profile,
            "control_points": list(self.control_points),
            "yaw_profile": self.yaw_profile,
            "nodes": list(self.nodes),
            "analysis": self.analysis,
            "hashes": self.hashes,
            "review": self.review,
        }


@dataclass(frozen=True)
class LegLibraryV40:
    planner_version: str
    project_hash: str
    legs: tuple[LegV40, ...]
    format: str = LEG_LIBRARY_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegLibraryV40":
        reject_deleted_fields(data, "LegLibraryV40")
        reject_legacy_format(data.get("format"), "LegLibraryV40")
        reject_unknown_fields(data, LIBRARY_FIELDS, "LegLibraryV40")
        require_fields(data, LIBRARY_FIELDS, "LegLibraryV40")
        expect_equal(data["format"], LEG_LIBRARY_FORMAT, "LegLibraryV40", "format")
        return cls(
            planner_version=str(data["planner_version"]),
            project_hash=str(data["project_hash"]),
            legs=tuple(LegV40.from_dict(item) for item in data["legs"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": self.format,
            "planner_version": self.planner_version,
            "project_hash": self.project_hash,
            "legs": [leg.to_dict() for leg in self.legs],
        }
