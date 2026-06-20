"""V4.0 route case table and case manifest models."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.py_io.codecs.legacy_rejection import reject_deleted_fields, reject_legacy_format

from .enums import GenerationMode, RouteFamily, StorageMode
from .errors import V40ValidationError, expect_equal, expect_int_range, reject_unknown_fields, require_fields
from .manual_path import validate_manual_path_dict
from .semi_path import validate_semi_path_dict
from .protocol import (
    CASE_FORMAT,
    LOGICAL_TASK_POINT_KEYS,
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
    "generation_mode",
    "traj_id",
    "bean_code",
    "drop_code",
    "source_mapping",
    "selected_plan",
    "manual_path",
    "semi_path",
    "logical_points",
    "auxiliary_points",
    "derived_from",
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
    generation_mode: GenerationMode
    traj_id: int
    bean_code: int
    drop_code: int
    source_mapping: dict[str, Any]
    selected_plan: dict[str, Any]
    logical_points: tuple[dict[str, Any], ...]
    arrival_states: tuple[dict[str, Any], ...]
    leg_refs: tuple[dict[str, Any], ...]
    actions: dict[str, Any]
    finish: dict[str, Any]
    estimates: dict[str, Any]
    hashes: dict[str, Any]
    review: dict[str, Any]
    manual_path: dict[str, Any] | None = None
    semi_path: dict[str, Any] | None = None
    auxiliary_points: tuple[dict[str, Any], ...] = ()
    derived_from: dict[str, Any] | None = None
    embedded_legs: tuple[dict[str, Any], ...] = ()
    format: str = CASE_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CaseManifestV40":
        reject_deleted_fields(data, "CaseManifestV40")
        reject_legacy_format(data.get("format"), "CaseManifestV40")
        reject_unknown_fields(data, CASE_FIELDS, "CaseManifestV40")
        require_fields(
            data,
            CASE_FIELDS - {"embedded_legs", "auxiliary_points", "derived_from", "semi_path"},
            "CaseManifestV40",
        )
        expect_equal(data["format"], CASE_FORMAT, "CaseManifestV40", "format")
        storage_mode = StorageMode(str(data["storage_mode"]))
        generation_mode = GenerationMode(str(data["generation_mode"]))
        traj_id = expect_int_range(int(data["traj_id"]), MIN_TRAJ_ID, MAX_TRAJ_ID, "CaseManifestV40", "traj_id")
        if not re.fullmatch(r"P\d{4}", f"P{traj_id:04d}"):
            raise V40ValidationError("CaseManifestV40", "traj_id", "invalid case id", actual=traj_id)
        manual_path = data.get("manual_path")
        semi_path = data.get("semi_path")
        if generation_mode == GenerationMode.SEMI_AUTO and semi_path is None:
            # Explicit in-place migration for pre-ordered-path V4 drafts.  The old
            # snapshot stored eight anchors plus an unordered auxiliary list.  New
            # files always write one ordered semi_path, so project.json remains the
            # only authority for fixed poses.
            semi_path = _legacy_semi_path_from_case_data(data)
        if generation_mode == GenerationMode.MANUAL:
            manual_path = validate_manual_path_dict(manual_path)
            if semi_path is not None:
                raise V40ValidationError("CaseManifestV40", "semi_path", "MANUAL cases must set semi_path to null", actual=semi_path)
        elif generation_mode == GenerationMode.SEMI_AUTO:
            if manual_path is not None:
                raise V40ValidationError("CaseManifestV40", "manual_path", "SEMI_AUTO cases must set manual_path to null", actual=manual_path)
            if semi_path is not None:
                semi_path = validate_semi_path_dict(semi_path, require_complete=True)
        else:
            if manual_path is not None or semi_path is not None:
                raise V40ValidationError(
                    "CaseManifestV40",
                    "manual_path/semi_path",
                    "FULL_AUTO cases must not contain a user path",
                    actual={"manual_path": manual_path, "semi_path": semi_path},
                )
        logical_points = tuple(dict(item) for item in data["logical_points"])
        return cls(
            storage_mode=storage_mode,
            generation_mode=generation_mode,
            traj_id=traj_id,
            bean_code=expect_int_range(int(data["bean_code"]), MIN_BEAN_CODE, MAX_BEAN_CODE, "CaseManifestV40", "bean_code"),
            drop_code=expect_int_range(int(data["drop_code"]), MIN_DROP_CODE, MAX_DROP_CODE, "CaseManifestV40", "drop_code"),
            source_mapping=dict(data["source_mapping"]),
            selected_plan=dict(data["selected_plan"]),
            logical_points=logical_points,
            arrival_states=tuple(dict(item) for item in data["arrival_states"]),
            leg_refs=tuple(dict(item) for item in data["leg_refs"]),
            actions=dict(data["actions"]),
            finish=dict(data["finish"]),
            estimates=dict(data["estimates"]),
            hashes=dict(data["hashes"]),
            review=dict(data["review"]),
            manual_path=manual_path,
            semi_path=semi_path,
            auxiliary_points=(
                ()
                if generation_mode == GenerationMode.SEMI_AUTO and data.get("semi_path") is None
                else tuple(dict(item) for item in data.get("auxiliary_points", ()))
            ),
            derived_from=dict(data["derived_from"]) if data.get("derived_from") is not None else None,
            embedded_legs=tuple(dict(item) for item in data.get("embedded_legs", ())),
        )._validated_generation_mode()

    def to_dict(self) -> dict[str, Any]:
        data = {
            "format": self.format,
            "storage_mode": self.storage_mode.value,
            "generation_mode": self.generation_mode.value,
            "traj_id": self.traj_id,
            "bean_code": self.bean_code,
            "drop_code": self.drop_code,
            "source_mapping": self.source_mapping,
            "selected_plan": self.selected_plan,
            "manual_path": self.manual_path,
            "semi_path": self.semi_path,
            "logical_points": list(self.logical_points),
            "auxiliary_points": list(self.auxiliary_points),
            "derived_from": self.derived_from,
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

    def _validated_generation_mode(self) -> "CaseManifestV40":
        if self.generation_mode == GenerationMode.FULL_AUTO:
            _validate_logical_points(self.logical_points)
            if self.auxiliary_points:
                raise V40ValidationError(
                    "CaseManifestV40",
                    "auxiliary_points",
                    "FULL_AUTO cases are derived and must not contain user auxiliary points",
                    actual=len(self.auxiliary_points),
                    expected=0,
                )
            return self
        if self.generation_mode == GenerationMode.SEMI_AUTO:
            if self.logical_points:
                # Legacy snapshots remain readable, but new writers leave this empty
                # so project.json is the only authority for the eight fixed poses.
                _validate_logical_points(self.logical_points)
            if self.semi_path is None:
                raise V40ValidationError("CaseManifestV40", "semi_path", "SEMI_AUTO cases require an ordered semi_path")
            if self.leg_refs:
                raise V40ValidationError("CaseManifestV40", "leg_refs", "user-drawn SEMI_AUTO paths do not reference leg_library", actual=self.leg_refs)
            if self.auxiliary_points:
                raise V40ValidationError("CaseManifestV40", "auxiliary_points", "SEMI_AUTO waypoints belong in semi_path execution order", actual=self.auxiliary_points)
            return self
        route_family = str(self.selected_plan.get("route_family", ""))
        if route_family not in {RouteFamily.MANUAL.name, "MANUAL", "ROUTE_FAMILY_MANUAL"}:
            raise V40ValidationError("CaseManifestV40", "selected_plan.route_family", "MANUAL cases require route_family MANUAL", actual=route_family)
        if self.leg_refs:
            raise V40ValidationError("CaseManifestV40", "leg_refs", "MANUAL cases must not reference leg_library", actual=self.leg_refs)
        if not self.review.get("detached_from_library", False):
            raise V40ValidationError("CaseManifestV40", "review.detached_from_library", "MANUAL cases must be detached from library", actual=self.review)
        if not str(self.review.get("override_reason", "")).strip():
            raise V40ValidationError("CaseManifestV40", "review.override_reason", "MANUAL cases require an override_reason")
        if self.manual_path is None:
            raise V40ValidationError("CaseManifestV40", "manual_path", "MANUAL cases require manual_path")
        if self.logical_points:
            raise V40ValidationError(
                "CaseManifestV40",
                "logical_points",
                "MANUAL cases use manual_path and must not define the eight task anchors",
                actual=len(self.logical_points),
                expected=0,
            )
        if self.auxiliary_points:
            raise V40ValidationError(
                "CaseManifestV40",
                "auxiliary_points",
                "MANUAL cases express all geometry in manual_path",
                actual=len(self.auxiliary_points),
                expected=0,
            )
        return self


def _legacy_semi_path_from_case_data(data: dict[str, Any]) -> dict[str, Any]:
    selected = data.get("selected_plan") if isinstance(data.get("selected_plan"), dict) else {}
    route_raw = str(selected.get("route_family", ""))
    pickup_order = tuple(str(value) for value in selected.get("pickup_arrival_state_order", ()))
    if route_raw in {"PICK_1_TO_3", "ROUTE_FAMILY_PICK_1_TO_3", "1"} or pickup_order[:3] == ("P_PICK_1", "P_PICK_2L", "P_PICK_3"):
        sequence = ("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_3", "P_DROP_3", "P_DROP_2", "P_DROP_1")
    elif route_raw in {"PICK_3_TO_1", "ROUTE_FAMILY_PICK_3_TO_1", "2"} or pickup_order[:3] == ("P_PICK_3", "P_PICK_2R", "P_PICK_1"):
        sequence = ("P_START", "P_PICK_3", "P_PICK_2R", "P_PICK_1", "P_DROP_1", "P_DROP_2", "P_DROP_3")
    else:
        raise V40ValidationError(
            "CaseManifestV40",
            "semi_path",
            "legacy SEMI_AUTO case cannot be migrated because its route family is not one of the two legal routes",
            actual={"route_family": route_raw, "pickup_arrival_state_order": list(pickup_order)},
        )

    points: list[dict[str, Any]] = []
    for index, site_key in enumerate(sequence):
        points.append(
            {
                "type": "START" if index == 0 else "ARRIVAL",
                "site_key": site_key,
                "state_id": site_key,
            }
        )
    auxiliary = data.get("auxiliary_points", ())
    if isinstance(auxiliary, (list, tuple)) and auxiliary:
        migrated_waypoints: list[dict[str, Any]] = []
        for item in auxiliary:
            if not isinstance(item, dict) or "x_mm" not in item or "y_mm" not in item:
                continue
            migrated_waypoints.append(
                {
                    "type": "WAYPOINT",
                    "x_mm": int(item["x_mm"]),
                    "y_mm": int(item["y_mm"]),
                    "exact_pass": str(item.get("policy", "LOCKED_PASS")) == "LOCKED_PASS",
                }
            )
        # The legacy format did not record which leg owned an auxiliary point.
        # Keep them in their previous display-compatible location immediately
        # before the final fixed arrival instead of silently discarding them.
        points[-1:-1] = migrated_waypoints
    return {"points": points, "notes": "migrated from legacy unordered SEMI_AUTO anchors"}


def _validate_logical_points(points: tuple[dict[str, Any], ...]) -> None:
    point_ids = tuple(str(item.get("point_id", "")) for item in points)
    if len(points) != len(LOGICAL_TASK_POINT_KEYS) or set(point_ids) != set(LOGICAL_TASK_POINT_KEYS):
        raise V40ValidationError(
            "CaseManifestV40",
            "logical_points",
            "SEMI_AUTO and FULL_AUTO cases require exactly the eight logical task anchors",
            actual=list(point_ids),
            expected=list(LOGICAL_TASK_POINT_KEYS),
        )
    for index, item in enumerate(points):
        pose = item.get("pose")
        if not isinstance(pose, dict) or not {"x_mm", "y_mm", "yaw_ddeg"} <= set(pose):
            raise V40ValidationError(
                "CaseManifestV40",
                f"logical_points[{index}].pose",
                "must contain x_mm, y_mm, and yaw_ddeg",
                actual=pose,
            )


def _validate_auxiliary_points(points: tuple[dict[str, Any], ...]) -> None:
    allowed = {"LOCKED_PASS", "INITIAL_GUESS", "OPTIMIZABLE"}
    for index, item in enumerate(points):
        if set(item) != {"x_mm", "y_mm", "policy"}:
            raise V40ValidationError(
                "CaseManifestV40",
                f"auxiliary_points[{index}]",
                "must contain exactly x_mm, y_mm, and policy",
                actual=sorted(item),
                expected=["policy", "x_mm", "y_mm"],
            )
        if str(item["policy"]) not in allowed:
            raise V40ValidationError(
                "CaseManifestV40",
                f"auxiliary_points[{index}].policy",
                "unsupported semi-auto auxiliary point policy",
                actual=item["policy"],
                expected=sorted(allowed),
            )
        for coordinate in ("x_mm", "y_mm"):
            value = item[coordinate]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise V40ValidationError(
                    "CaseManifestV40",
                    f"auxiliary_points[{index}].{coordinate}",
                    "must be numeric",
                    actual=value,
                )


class PortableCaseV40(CaseManifestV40):
    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PortableCaseV40":
        case = CaseManifestV40.from_dict(data)
        expect_equal(case.storage_mode, StorageMode.EMBEDDED, "PortableCaseV40", "storage_mode")
        return cls(**case.__dict__)
