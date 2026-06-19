"""V4.0 route case table and case manifest models."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.codec.legacy_rejection import reject_deleted_fields, reject_legacy_format

from .enums import PathSource, RouteFamily, StorageMode
from .errors import V40ValidationError, expect_equal, expect_int_range, reject_unknown_fields, require_fields
from .manual_path import validate_manual_path_dict
from .protocol import (
    CASE_FORMAT,
    MAX_BEAN_CODE,
    MAX_DROP_CODE,
    MAX_TRAJ_ID,
    MIN_BEAN_CODE,
    MIN_DROP_CODE,
    MIN_TRAJ_ID,
    ROUTE_CASE_TABLE_FORMAT,
)

ROUTE_ROW_FIELDS = {
    "traj_id",
    "file_name",
    "bean_code",
    "drop_code",
    "pick_assignment",
    "label_positions",
    "source_row_number",
    "source_row_hash",
    "raw_fields",
}
ROUTE_ROW_REQUIRED_FIELDS = ROUTE_ROW_FIELDS - {"source_row_number", "raw_fields"}
ROUTE_TABLE_FIELDS = {"format", "source_csv", "source_csv_sha256", "case_count", "cases"}
CASE_FIELDS = {
    "format",
    "storage_mode",
    "path_source",
    "traj_id",
    "bean_code",
    "drop_code",
    "source_mapping",
    "selected_plan",
    "manual_path",
    "arrival_states",
    "leg_refs",
    "actions",
    "finish",
    "estimates",
    "hashes",
    "review",
    "embedded_legs",
}


@dataclass(frozen=True)
class RouteCaseRowV40:
    traj_id: int
    file_name: str
    bean_code: int
    drop_code: int
    pick_assignment: dict[str, Any]
    label_positions: dict[str, Any]
    source_row_hash: str
    source_row_number: int | None = None
    raw_fields: dict[str, Any] | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RouteCaseRowV40":
        reject_unknown_fields(data, ROUTE_ROW_FIELDS, "RouteCaseRowV40")
        require_fields(data, ROUTE_ROW_REQUIRED_FIELDS, "RouteCaseRowV40")
        traj_id = expect_int_range(int(data["traj_id"]), MIN_TRAJ_ID, MAX_TRAJ_ID, "RouteCaseRowV40", "traj_id")
        expected_file = f"P{traj_id:04d}.BIN"
        if data["file_name"] != expected_file:
            raise V40ValidationError("RouteCaseRowV40", "file_name", "must match traj_id", actual=data["file_name"], expected=expected_file)
        return cls(
            traj_id=traj_id,
            file_name=str(data["file_name"]),
            bean_code=expect_int_range(int(data["bean_code"]), MIN_BEAN_CODE, MAX_BEAN_CODE, "RouteCaseRowV40", "bean_code"),
            drop_code=expect_int_range(int(data["drop_code"]), MIN_DROP_CODE, MAX_DROP_CODE, "RouteCaseRowV40", "drop_code"),
            pick_assignment=dict(data["pick_assignment"]),
            label_positions=dict(data["label_positions"]),
            source_row_hash=str(data["source_row_hash"]),
            source_row_number=int(data["source_row_number"]) if "source_row_number" in data else None,
            raw_fields=dict(data["raw_fields"]) if "raw_fields" in data else None,
        )

    def to_dict(self) -> dict[str, Any]:
        data = {
            "traj_id": self.traj_id,
            "file_name": self.file_name,
            "bean_code": self.bean_code,
            "drop_code": self.drop_code,
            "pick_assignment": self.pick_assignment,
            "label_positions": self.label_positions,
            "source_row_hash": self.source_row_hash,
        }
        if self.source_row_number is not None:
            data["source_row_number"] = self.source_row_number
        if self.raw_fields is not None:
            data["raw_fields"] = self.raw_fields
        return data


@dataclass(frozen=True)
class RouteCaseTableV40:
    source_csv: str
    source_csv_sha256: str
    cases: tuple[RouteCaseRowV40, ...]
    format: str = ROUTE_CASE_TABLE_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RouteCaseTableV40":
        reject_deleted_fields(data, "RouteCaseTableV40")
        reject_legacy_format(data.get("format"), "RouteCaseTableV40")
        reject_unknown_fields(data, ROUTE_TABLE_FIELDS, "RouteCaseTableV40")
        require_fields(data, ROUTE_TABLE_FIELDS, "RouteCaseTableV40")
        expect_equal(data["format"], ROUTE_CASE_TABLE_FORMAT, "RouteCaseTableV40", "format")
        cases = tuple(RouteCaseRowV40.from_dict(item) for item in data["cases"])
        expect_equal(data["case_count"], len(cases), "RouteCaseTableV40", "case_count")
        return cls(source_csv=str(data["source_csv"]), source_csv_sha256=str(data["source_csv_sha256"]), cases=cases)

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": self.format,
            "source_csv": self.source_csv,
            "source_csv_sha256": self.source_csv_sha256,
            "case_count": len(self.cases),
            "cases": [case.to_dict() for case in self.cases],
        }


@dataclass(frozen=True)
class CaseManifestV40:
    storage_mode: StorageMode
    path_source: PathSource
    traj_id: int
    bean_code: int
    drop_code: int
    source_mapping: dict[str, Any]
    selected_plan: dict[str, Any]
    arrival_states: tuple[dict[str, Any], ...]
    leg_refs: tuple[dict[str, Any], ...]
    actions: dict[str, Any]
    finish: dict[str, Any]
    estimates: dict[str, Any]
    hashes: dict[str, Any]
    review: dict[str, Any]
    manual_path: dict[str, Any] | None = None
    embedded_legs: tuple[dict[str, Any], ...] = ()
    format: str = CASE_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CaseManifestV40":
        reject_deleted_fields(data, "CaseManifestV40")
        reject_legacy_format(data.get("format"), "CaseManifestV40")
        reject_unknown_fields(data, CASE_FIELDS, "CaseManifestV40")
        require_fields(data, CASE_FIELDS - {"embedded_legs"}, "CaseManifestV40")
        expect_equal(data["format"], CASE_FORMAT, "CaseManifestV40", "format")
        storage_mode = StorageMode(str(data["storage_mode"]))
        path_source = PathSource(str(data["path_source"]))
        traj_id = expect_int_range(int(data["traj_id"]), MIN_TRAJ_ID, MAX_TRAJ_ID, "CaseManifestV40", "traj_id")
        if not re.fullmatch(r"P\d{4}", f"P{traj_id:04d}"):
            raise V40ValidationError("CaseManifestV40", "traj_id", "invalid case id", actual=traj_id)
        manual_path = data.get("manual_path")
        if path_source == PathSource.TASK_COMPILED:
            if manual_path is not None:
                raise V40ValidationError("CaseManifestV40", "manual_path", "TASK_COMPILED cases must set manual_path to null", actual=manual_path)
        else:
            manual_path = validate_manual_path_dict(manual_path)
        return cls(
            storage_mode=storage_mode,
            path_source=path_source,
            traj_id=traj_id,
            bean_code=expect_int_range(int(data["bean_code"]), MIN_BEAN_CODE, MAX_BEAN_CODE, "CaseManifestV40", "bean_code"),
            drop_code=expect_int_range(int(data["drop_code"]), MIN_DROP_CODE, MAX_DROP_CODE, "CaseManifestV40", "drop_code"),
            source_mapping=dict(data["source_mapping"]),
            selected_plan=dict(data["selected_plan"]),
            arrival_states=tuple(dict(item) for item in data["arrival_states"]),
            leg_refs=tuple(dict(item) for item in data["leg_refs"]),
            actions=dict(data["actions"]),
            finish=dict(data["finish"]),
            estimates=dict(data["estimates"]),
            hashes=dict(data["hashes"]),
            review=dict(data["review"]),
            manual_path=manual_path,
            embedded_legs=tuple(dict(item) for item in data.get("embedded_legs", ())),
        )._validated_case_source()

    def to_dict(self) -> dict[str, Any]:
        data = {
            "format": self.format,
            "storage_mode": self.storage_mode.value,
            "path_source": self.path_source.value,
            "traj_id": self.traj_id,
            "bean_code": self.bean_code,
            "drop_code": self.drop_code,
            "source_mapping": self.source_mapping,
            "selected_plan": self.selected_plan,
            "manual_path": self.manual_path,
            "arrival_states": list(self.arrival_states),
            "leg_refs": list(self.leg_refs),
            "actions": self.actions,
            "finish": self.finish,
            "estimates": self.estimates,
            "hashes": self.hashes,
            "review": self.review,
        }
        if self.embedded_legs:
            data["embedded_legs"] = list(self.embedded_legs)
        return data

    def _validated_case_source(self) -> "CaseManifestV40":
        if self.path_source == PathSource.TASK_COMPILED:
            return self
        route_family = str(self.selected_plan.get("route_family", ""))
        if route_family not in {RouteFamily.MANUAL_FREE.name, "MANUAL_FREE", "ROUTE_FAMILY_MANUAL_FREE"}:
            raise V40ValidationError("CaseManifestV40", "selected_plan.route_family", "MANUAL_FREE cases require route_family MANUAL_FREE", actual=route_family)
        if self.leg_refs:
            raise V40ValidationError("CaseManifestV40", "leg_refs", "MANUAL_FREE cases must not reference leg_library", actual=self.leg_refs)
        if not self.review.get("detached_from_library", False):
            raise V40ValidationError("CaseManifestV40", "review.detached_from_library", "MANUAL_FREE cases must be detached from library", actual=self.review)
        if not str(self.review.get("override_reason", "")).strip():
            raise V40ValidationError("CaseManifestV40", "review.override_reason", "MANUAL_FREE cases require an override_reason")
        if self.manual_path is None:
            raise V40ValidationError("CaseManifestV40", "manual_path", "MANUAL_FREE cases require manual_path")
        return self


class PortableCaseV40(CaseManifestV40):
    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PortableCaseV40":
        case = CaseManifestV40.from_dict(data)
        expect_equal(case.storage_mode, StorageMode.EMBEDDED, "PortableCaseV40", "storage_mode")
        return cls(**case.__dict__)
