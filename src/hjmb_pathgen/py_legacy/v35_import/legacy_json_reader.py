"""Strict JSON readers for the two supported V3.5 migration inputs."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .legacy_models import LegacyFixedSitesV35, LegacyProjectV35


V35_PROJECT_FORMAT = "HJMB_PATH_EDITOR_JSON_V35"


def load_v35_project(
    path: str | Path,
    *,
    fixed_sites_path: str | Path | None = None,
) -> LegacyProjectV35:
    path = Path(path)
    raw = _read_object(path)
    if raw.get("format") != V35_PROJECT_FORMAT:
        raise ValueError(f"not a {V35_PROJECT_FORMAT} file: {path}")
    fixed_sites = tuple(_object_list(raw.get("fixed_sites", []), "fixed_sites"))
    external_path = Path(fixed_sites_path) if fixed_sites_path is not None else path.with_name("fixed_sites_v35.json")
    if not fixed_sites and external_path.exists():
        fixed_sites = load_v35_fixed_sites(external_path).fixed_sites
    return LegacyProjectV35(
        traj_id=int(raw.get("traj_id", 0)),
        points=tuple(_object_list(raw.get("points", []), "points")),
        actions=tuple(_object_list(raw.get("actions", []), "actions")),
        fixed_sites=fixed_sites,
        route_meta=dict(raw.get("route_meta", {})),
        raw=raw,
    )


def load_v35_fixed_sites(path: str | Path) -> LegacyFixedSitesV35:
    value = _read_json(path)
    raw = {"fixed_sites": value} if isinstance(value, list) else value
    if not isinstance(raw, dict):
        raise ValueError(f"legacy fixed-sites input must be an object or array: {path}")
    return LegacyFixedSitesV35(
        fixed_sites=tuple(_object_list(raw.get("fixed_sites", []), "fixed_sites")),
        raw=raw,
    )


def _read_object(path: str | Path) -> dict[str, Any]:
    value = _read_json(path)
    if not isinstance(value, dict):
        raise ValueError(f"legacy input must be a JSON object: {path}")
    return dict(value)


def _read_json(path: str | Path) -> Any:
    data = Path(path).read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    return json.loads(data.decode("utf-8"))


def _object_list(value: object, field_name: str) -> list[dict[str, Any]]:
    if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
        raise ValueError(f"legacy field {field_name} must be an array of objects")
    return [dict(item) for item in value]
