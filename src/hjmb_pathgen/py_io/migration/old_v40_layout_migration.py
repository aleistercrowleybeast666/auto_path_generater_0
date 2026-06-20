"""Explicit migration from deprecated flat and two-mode V4 directories."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.errors import CompileError
from hjmb_pathgen.py_io.codecs.json_codec import parse_case_bytes
from hjmb_pathgen.py_io.layout.path_naming import case_json_name
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes


OLD_MODE_MAP = {
    "manual_free": GenerationMode.MANUAL,
    "task_compiled": GenerationMode.FULL_AUTO,
}
OLD_CASE_MODE_MAP = {
    "MANUAL_FREE": GenerationMode.MANUAL,
    "TASK_COMPILED": GenerationMode.FULL_AUTO,
}


@dataclass(frozen=True)
class LayoutMigrationItem:
    source: str
    target: str | None
    status: str
    reason: str = ""

    def to_dict(self) -> dict[str, str | None]:
        return {
            "source": self.source,
            "target": self.target,
            "status": self.status,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class LayoutMigrationReport:
    dry_run: bool
    items: tuple[LayoutMigrationItem, ...]

    @property
    def conflict_count(self) -> int:
        return sum(item.status == "CONFLICT" for item in self.items)

    @property
    def unresolved_count(self) -> int:
        return sum(item.status == "UNRESOLVED" for item in self.items)

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": "HJMB_V40_LAYOUT_MIGRATION_REPORT",
            "dry_run": self.dry_run,
            "conflict_count": self.conflict_count,
            "unresolved_count": self.unresolved_count,
            "items": [item.to_dict() for item in self.items],
        }


def migrate_old_v40_layout(
    layout: ProjectLayout,
    *,
    dry_run: bool = True,
    write_report: bool = True,
) -> LayoutMigrationReport:
    """Migrate known legacy paths without overwriting or guessing missing modes."""

    layout.ensure_directories()
    items: list[LayoutMigrationItem] = []
    case_modes: dict[int, GenerationMode] = {}

    for old_name, mode in OLD_MODE_MAP.items():
        old_case_dir = layout.cases_dir / old_name
        for source in sorted(old_case_dir.glob("P*.json")) if old_case_dir.exists() else ():
            item, resolved_mode = _migrate_case_file(layout, source, mode, dry_run=dry_run)
            items.append(item)
            if resolved_mode is not None:
                case_modes[_traj_id(source)] = resolved_mode

    for source in sorted(layout.cases_dir.glob("P*.json")):
        item, mode = _migrate_case_file(layout, source, None, dry_run=dry_run)
        items.append(item)
        if mode is not None:
            case_modes[_traj_id(source)] = mode

    for base_name, extension in (("bin", ".BIN"), ("portable", ".portable.json")):
        base = layout.bin_dir if base_name == "bin" else layout.portable_dir
        for old_name, mode in OLD_MODE_MAP.items():
            old_dir = base / old_name
            for source in sorted(old_dir.glob(f"P*{extension}")) if old_dir.exists() else ():
                items.append(_migrate_payload_file(layout, base_name, source, mode, dry_run=dry_run))
        for source in sorted(base.glob(f"P*{extension}")):
            mode = case_modes.get(_traj_id(source))
            if mode is None:
                items.append(
                    LayoutMigrationItem(
                        source=str(source),
                        target=None,
                        status="UNRESOLVED",
                        reason="flat output has no explicit generation_mode authority",
                    )
                )
            else:
                items.append(_migrate_payload_file(layout, base_name, source, mode, dry_run=dry_run))

    report = LayoutMigrationReport(dry_run=dry_run, items=tuple(items))
    if write_report:
        report_path = layout.reports_dir / "layout_migration_report.json"
        _write_json(report_path, report.to_dict())
    return report


def _migrate_case_file(
    layout: ProjectLayout,
    source: Path,
    directory_mode: GenerationMode | None,
    *,
    dry_run: bool,
) -> tuple[LayoutMigrationItem, GenerationMode | None]:
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
        mode = _case_mode(raw, directory_mode)
        payload = _normalized_case_payload(raw, mode)
        data = (json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
        parsed = parse_case_bytes(data, source=str(source))
        if parsed.generation_mode != mode:
            raise CompileError("normalized case mode mismatch")
    except Exception as exc:  # noqa: BLE001 - migration must report per-file failure.
        return (
            LayoutMigrationItem(str(source), None, "UNRESOLVED", str(exc)),
            None,
        )
    target = layout.case_json_path_for_mode(parsed.traj_id, mode)
    return _move_validated(source, target, data, dry_run=dry_run), mode


def _case_mode(raw: dict[str, Any], directory_mode: GenerationMode | None) -> GenerationMode:
    explicit = raw.get("generation_mode")
    if explicit is not None:
        mode = GenerationMode(str(explicit))
    else:
        legacy = str(raw.get("path_source", ""))
        if legacy not in OLD_CASE_MODE_MAP:
            raise CompileError("Case has no recognized generation_mode; refusing to guess")
        mode = OLD_CASE_MODE_MAP[legacy]
    if directory_mode is not None and mode != directory_mode:
        raise CompileError(
            f"directory mode {directory_mode.value} conflicts with Case mode {mode.value}"
        )
    return mode


def _normalized_case_payload(raw: dict[str, Any], mode: GenerationMode) -> dict[str, Any]:
    payload = dict(raw)
    payload.pop("path_source", None)
    payload["generation_mode"] = mode.value
    selected = dict(payload.get("selected_plan", {}))
    if selected.get("route_family") == "MANUAL_FREE":
        selected["route_family"] = "MANUAL"
    payload["selected_plan"] = selected
    if mode == GenerationMode.MANUAL:
        payload.setdefault("logical_points", [])
    elif "logical_points" not in payload:
        raise CompileError(
            f"legacy {mode.value} Case has no eight logical_points; regenerate explicitly"
        )
    return payload


def _migrate_payload_file(
    layout: ProjectLayout,
    kind: str,
    source: Path,
    mode: GenerationMode,
    *,
    dry_run: bool,
) -> LayoutMigrationItem:
    traj_id = _traj_id(source)
    target = (
        layout.bin_path_for_mode(traj_id, mode)
        if kind == "bin"
        else layout.portable_path_for_mode(traj_id, mode)
    )
    return _move_validated(source, target, source.read_bytes(), dry_run=dry_run)


def _move_validated(source: Path, target: Path, data: bytes, *, dry_run: bool) -> LayoutMigrationItem:
    if target.exists() and target.resolve(strict=False) != source.resolve(strict=False):
        return LayoutMigrationItem(str(source), str(target), "CONFLICT", "target already exists")
    if dry_run:
        return LayoutMigrationItem(str(source), str(target), "PLANNED")

    def validator(temp_path: Path) -> None:
        if temp_path.read_bytes() != data:
            raise CompileError(f"migration write-back mismatch: {target}")

    atomic_write_bytes(target, data, validator=validator)
    source.unlink()
    return LayoutMigrationItem(str(source), str(target), "MIGRATED")


def _traj_id(path: Path) -> int:
    stem = path.name.split(".", 1)[0]
    if len(stem) != 5 or not stem.startswith("P") or not stem[1:].isdigit():
        raise CompileError(f"invalid case filename: {path.name}")
    return int(stem[1:])


def _write_json(path: Path, value: dict[str, Any]) -> None:
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")

    def validator(temp_path: Path) -> None:
        if json.loads(temp_path.read_text(encoding="utf-8")) != value:
            raise CompileError(f"migration report write-back mismatch: {path}")

    atomic_write_bytes(path, data, validator=validator)
