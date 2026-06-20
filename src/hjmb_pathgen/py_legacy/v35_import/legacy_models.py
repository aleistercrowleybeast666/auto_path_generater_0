"""Minimal read-only V3.5 models used only by migration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LegacyProjectV35:
    traj_id: int
    points: tuple[dict[str, Any], ...]
    actions: tuple[dict[str, Any], ...]
    fixed_sites: tuple[dict[str, Any], ...]
    route_meta: dict[str, Any]
    raw: dict[str, Any]


@dataclass(frozen=True)
class LegacyFixedSitesV35:
    fixed_sites: tuple[dict[str, Any], ...]
    raw: dict[str, Any]
