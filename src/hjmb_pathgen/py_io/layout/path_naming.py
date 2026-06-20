"""Strict V4.0 Pxxxx filename helpers."""

from __future__ import annotations

import re
from pathlib import Path

from hjmb_pathgen.py_domain.errors import FilenameMismatchError, V40ValidationError, expect_int_range
from hjmb_pathgen.py_domain.protocol import MAX_TRAJ_ID, MIN_TRAJ_ID

_CASE_JSON_RE = re.compile(r"^P([0-9]{4})\.json$")
_BIN_RE = re.compile(r"^P([0-9]{4})\.BIN$")
_PORTABLE_RE = re.compile(r"^P([0-9]{4})\.portable\.json$")


def validate_traj_id(traj_id: int) -> int:
    return expect_int_range(traj_id, MIN_TRAJ_ID, MAX_TRAJ_ID, "Pxxxx filename", "traj_id")


def case_json_name(traj_id: int) -> str:
    traj_id = validate_traj_id(traj_id)
    return f"P{traj_id:04d}.json"


def bin_name(traj_id: int) -> str:
    traj_id = validate_traj_id(traj_id)
    return f"P{traj_id:04d}.BIN"


def portable_name(traj_id: int) -> str:
    traj_id = validate_traj_id(traj_id)
    return f"P{traj_id:04d}.portable.json"


def parse_case_json_name(path: str | Path) -> int:
    return _parse_name(path, _CASE_JSON_RE, "case JSON filename")


def parse_bin_name(path: str | Path) -> int:
    return _parse_name(path, _BIN_RE, "BIN filename")


def parse_portable_name(path: str | Path) -> int:
    return _parse_name(path, _PORTABLE_RE, "portable case filename")


def ensure_case_filename_matches(path: str | Path, traj_id: int) -> None:
    _ensure_matches(path, traj_id, parse_case_json_name(path), "case JSON filename")


def ensure_bin_filename_matches(path: str | Path, traj_id: int) -> None:
    _ensure_matches(path, traj_id, parse_bin_name(path), "BIN filename")


def ensure_portable_filename_matches(path: str | Path, traj_id: int) -> None:
    _ensure_matches(path, traj_id, parse_portable_name(path), "portable case filename")


def _parse_name(path: str | Path, pattern: re.Pattern[str], object_type: str) -> int:
    name = Path(path).name
    match = pattern.fullmatch(name)
    if not match:
        raise FilenameMismatchError(
            object_type,
            "name",
            "filename must be canonical Pxxxx form",
            actual=name,
            expected=pattern.pattern,
        )
    traj_id = int(match.group(1))
    try:
        return validate_traj_id(traj_id)
    except V40ValidationError as exc:
        raise FilenameMismatchError(
            object_type,
            "traj_id",
            "filename traj_id is out of competition range",
            actual=traj_id,
            expected=f"{MIN_TRAJ_ID}..{MAX_TRAJ_ID}",
        ) from exc


def _ensure_matches(path: str | Path, declared_traj_id: int, filename_traj_id: int, object_type: str) -> None:
    declared_traj_id = validate_traj_id(declared_traj_id)
    if filename_traj_id != declared_traj_id:
        raise FilenameMismatchError(
            object_type,
            "traj_id",
            "filename traj_id does not match document traj_id",
            actual=filename_traj_id,
            expected=declared_traj_id,
        )
