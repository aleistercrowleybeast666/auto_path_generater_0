"""Explicit V3.x and deleted-field rejection for V4.0 loaders."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.protocol import (
    CASE_FORMAT,
    LEG_LIBRARY_FORMAT,
    PROJECT_FORMAT,
    REMOVED_JSON_FIELDS,
    ROUTE_CASE_TABLE_FORMAT,
)

V40_FORMATS = {
    PROJECT_FORMAT,
    ROUTE_CASE_TABLE_FORMAT,
    LEG_LIBRARY_FORMAT,
    CASE_FORMAT,
}


def reject_legacy_format(format_name: object, object_type: str, field_path: str = "format") -> None:
    if isinstance(format_name, str) and ("_V3" in format_name or format_name.endswith("_V35")):
        raise V40ValidationError(
            object_type,
            field_path,
            "legacy V3.x format is rejected",
            actual=format_name,
            expected=sorted(V40_FORMATS),
        )


def reject_deleted_fields(value: Any, object_type: str, field_path: str = "$") -> None:
    if isinstance(value, Mapping):
        for key, nested in value.items():
            key_path = f"{field_path}.{key}" if field_path != "$" else str(key)
            if str(key) in REMOVED_JSON_FIELDS and not _is_allowed_v40_field(key_path):
                raise V40ValidationError(
                    object_type,
                    key_path,
                    "deleted V3.x field is rejected",
                    actual=key,
                    expected="V4.0 schema",
                )
            reject_deleted_fields(nested, object_type, key_path)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for index, nested in enumerate(value):
            reject_deleted_fields(nested, object_type, f"{field_path}[{index}]")


def _is_allowed_v40_field(field_path: str) -> bool:
    """Allow V4 fields whose names collide with removed V3 action fields."""

    if field_path.endswith(".flags") and ".nodes[" in field_path:
        return True
    if field_path.endswith(".gate_id") and (
        "topology_profiles" in field_path
        or ".topology_gates[" in field_path
        or ".topology.crossings[" in field_path
        or ".gates[" in field_path
    ):
        # ``gate_id`` was a removed runtime Gate field in V3.x, but V4 uses
        # the same spelling for offline virtual topology-gate definitions and
        # their validation diagnostics.
        return True
    return False
