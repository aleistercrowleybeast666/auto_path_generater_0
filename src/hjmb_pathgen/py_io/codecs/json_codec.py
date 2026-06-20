"""Strict V4.0 JSON codecs with stable UTF-8 output."""

from __future__ import annotations

import json
from json import JSONDecodeError
from pathlib import Path
from typing import Callable, TypeVar

from hjmb_pathgen.py_domain.errors import JsonFormatError, JsonValidationError, V40ValidationError, WriteBackValidationError
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, PortableCaseV40, RouteCaseTableV40
from hjmb_pathgen.py_domain.site_preset import SitePosePresetV40
from hjmb_pathgen.py_io.layout.path_naming import ensure_case_filename_matches, ensure_portable_filename_matches

T = TypeVar("T")

_UTF8_BOM = b"\xef\xbb\xbf"


def load_project(path: str | Path) -> ProjectV40:
    return parse_project_bytes(Path(path).read_bytes(), source=str(path))


def load_route_case_table(path: str | Path) -> RouteCaseTableV40:
    return parse_route_case_table_bytes(Path(path).read_bytes(), source=str(path))


def load_leg_library(path: str | Path) -> LegLibraryV40:
    return parse_leg_library_bytes(Path(path).read_bytes(), source=str(path))


def load_case(path: str | Path, *, enforce_filename: bool = True) -> CaseManifestV40:
    model = parse_case_bytes(Path(path).read_bytes(), source=str(path))
    if enforce_filename:
        ensure_case_filename_matches(path, model.traj_id)
    return model


def load_portable_case(path: str | Path, *, enforce_filename: bool = True) -> PortableCaseV40:
    model = parse_portable_case_bytes(Path(path).read_bytes(), source=str(path))
    if enforce_filename:
        ensure_portable_filename_matches(path, model.traj_id)
    return model


def load_site_pose_preset(path: str | Path) -> SitePosePresetV40:
    return parse_site_pose_preset_bytes(Path(path).read_bytes(), source=str(path))


def parse_project_bytes(data: bytes, *, source: str = "<bytes>") -> ProjectV40:
    return _parse_model(data, ProjectV40.from_dict, "ProjectV40", source)


def parse_route_case_table_bytes(data: bytes, *, source: str = "<bytes>") -> RouteCaseTableV40:
    return _parse_model(data, RouteCaseTableV40.from_dict, "RouteCaseTableV40", source)


def parse_leg_library_bytes(data: bytes, *, source: str = "<bytes>") -> LegLibraryV40:
    return _parse_model(data, LegLibraryV40.from_dict, "LegLibraryV40", source)


def parse_case_bytes(data: bytes, *, source: str = "<bytes>") -> CaseManifestV40:
    return _parse_model(data, CaseManifestV40.from_dict, "CaseManifestV40", source)


def parse_portable_case_bytes(data: bytes, *, source: str = "<bytes>") -> PortableCaseV40:
    return _parse_model(data, PortableCaseV40.from_dict, "PortableCaseV40", source)


def parse_site_pose_preset_bytes(data: bytes, *, source: str = "<bytes>") -> SitePosePresetV40:
    return _parse_model(data, SitePosePresetV40.from_dict, "SitePosePresetV40", source)


def dump_project_bytes(model: ProjectV40) -> bytes:
    return _dump_model_bytes(model)


def dump_route_case_table_bytes(model: RouteCaseTableV40) -> bytes:
    return _dump_model_bytes(model)


def dump_leg_library_bytes(model: LegLibraryV40) -> bytes:
    return _dump_model_bytes(model)


def dump_case_bytes(model: CaseManifestV40) -> bytes:
    return _dump_model_bytes(model)


def dump_portable_case_bytes(model: PortableCaseV40) -> bytes:
    return _dump_model_bytes(model)


def dump_site_pose_preset_bytes(model: SitePosePresetV40) -> bytes:
    return _dump_model_bytes(model)


def save_project(path: str | Path, model: ProjectV40) -> None:
    _save_json(path, model, dump_project_bytes, lambda data: parse_project_bytes(data, source=str(path)))


def save_route_case_table(path: str | Path, model: RouteCaseTableV40) -> None:
    _save_json(path, model, dump_route_case_table_bytes, lambda data: parse_route_case_table_bytes(data, source=str(path)))


def save_leg_library(path: str | Path, model: LegLibraryV40) -> None:
    _save_json(path, model, dump_leg_library_bytes, lambda data: parse_leg_library_bytes(data, source=str(path)))


def save_case(path: str | Path, model: CaseManifestV40) -> None:
    ensure_case_filename_matches(path, model.traj_id)
    _save_json(path, model, dump_case_bytes, lambda data: _parse_case_for_final_path(data, path))


def save_portable_case(path: str | Path, model: PortableCaseV40) -> None:
    ensure_portable_filename_matches(path, model.traj_id)
    _save_json(path, model, dump_portable_case_bytes, lambda data: _parse_portable_for_final_path(data, path))


def save_site_pose_preset(path: str | Path, model: SitePosePresetV40) -> None:
    _save_json(path, model, dump_site_pose_preset_bytes, lambda data: parse_site_pose_preset_bytes(data, source=str(path)))


def _parse_case_for_final_path(data: bytes, path: str | Path) -> CaseManifestV40:
    parsed = parse_case_bytes(data, source=str(path))
    ensure_case_filename_matches(path, parsed.traj_id)
    return parsed


def _parse_portable_for_final_path(data: bytes, path: str | Path) -> PortableCaseV40:
    parsed = parse_portable_case_bytes(data, source=str(path))
    ensure_portable_filename_matches(path, parsed.traj_id)
    return parsed


def _parse_model(data: bytes, factory: Callable[[dict], T], object_type: str, source: str) -> T:
    decoded = _decode_json_bytes(data, object_type, source)
    if not isinstance(decoded, dict):
        raise JsonFormatError(object_type, "$", "top-level JSON value must be an object", actual=type(decoded).__name__)
    try:
        return factory(decoded)
    except V40ValidationError as exc:
        if isinstance(exc, (JsonFormatError, JsonValidationError)):
            raise
        raise JsonValidationError(object_type, "$", f"validation failed for {source}: {exc}") from exc
    except (TypeError, ValueError) as exc:
        raise JsonValidationError(object_type, "$", f"validation failed for {source}: {exc}") from exc


def _decode_json_bytes(data: bytes, object_type: str, source: str) -> object:
    if data.startswith(_UTF8_BOM):
        raise JsonFormatError(object_type, "$", f"UTF-8 BOM is not allowed in {source}", actual="BOM", expected="UTF-8 without BOM")
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise JsonFormatError(object_type, "$", f"file is not valid UTF-8: {source}", actual=str(exc), expected="UTF-8") from exc
    try:
        return json.loads(text)
    except JSONDecodeError as exc:
        raise JsonFormatError(object_type, "$", f"invalid JSON text in {source}", actual=str(exc)) from exc


def _dump_model_bytes(model: object) -> bytes:
    try:
        text = json.dumps(model.to_dict(), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise JsonValidationError(type(model).__name__, "$", "model cannot be serialized as stable JSON", actual=str(exc)) from exc
    return (text + "\n").encode("utf-8")


def _save_json(path: str | Path, model: T, dump_func: Callable[[T], bytes], parse_func: Callable[[bytes], T]) -> None:
    from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes

    path = Path(path)
    data = dump_func(model)

    def validator(temp_path: Path) -> None:
        parsed = parse_func(temp_path.read_bytes())
        if parsed != model:
            raise WriteBackValidationError(f"JSON write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
