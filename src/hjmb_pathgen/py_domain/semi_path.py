"""SEMI_AUTO path schema.

Fixed START/ARRIVAL rows reference one of the eight project sites.  Free
WAYPOINT rows keep their coordinates in execution order.  Therefore a semi
case never copies the eight fixed poses and never loses which leg a waypoint
belongs to.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .enums import ManualPathPointType, RouteFamily
from .errors import V40ValidationError, reject_unknown_fields, require_fields
from .protocol import LOGICAL_TASK_POINT_KEYS

SEMI_PATH_FIELDS = {"points", "notes"}
SEMI_POINT_FIELDS = {
    "type",
    "site_key",
    "x_mm",
    "y_mm",
    "exact_pass",
    "max_speed_mmps",
    "corner_trim_mm",
    "state_id",
}

ROUTE_A_SITE_SEQUENCE = (
    "P_START",
    "P_PICK_1",
    "P_PICK_2L",
    "P_PICK_3",
    "P_DROP_3",
    "P_DROP_2",
    "P_DROP_1",
)
ROUTE_B_SITE_SEQUENCE = (
    "P_START",
    "P_PICK_3",
    "P_PICK_2R",
    "P_PICK_1",
    "P_DROP_1",
    "P_DROP_2",
    "P_DROP_3",
)


@dataclass(frozen=True)
class SemiPathPointV40:
    point_type: ManualPathPointType
    site_key: str | None = None
    x_mm: int | None = None
    y_mm: int | None = None
    exact_pass: bool = False
    max_speed_mmps: int | None = None
    corner_trim_mm: float = 200.0
    state_id: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, field_path: str) -> "SemiPathPointV40":
        if not isinstance(data, dict):
            raise V40ValidationError("SemiPathPointV40", field_path, "must be an object", actual=type(data).__name__)
        reject_unknown_fields(data, SEMI_POINT_FIELDS, "SemiPathPointV40", field_path)
        require_fields(data, {"type"}, "SemiPathPointV40", field_path)
        point_type = ManualPathPointType(str(data["type"]))
        site_key: str | None = None
        x_mm: int | None = None
        y_mm: int | None = None
        if point_type in (ManualPathPointType.START, ManualPathPointType.ARRIVAL):
            require_fields(data, {"site_key"}, "SemiPathPointV40", field_path)
            site_key = str(data["site_key"])
            if site_key not in LOGICAL_TASK_POINT_KEYS:
                raise V40ValidationError(
                    "SemiPathPointV40",
                    f"{field_path}.site_key",
                    "fixed path rows must reference one of the eight project sites",
                    actual=site_key,
                    expected=list(LOGICAL_TASK_POINT_KEYS),
                )
            if "x_mm" in data or "y_mm" in data:
                raise V40ValidationError(
                    "SemiPathPointV40",
                    field_path,
                    "fixed rows reference project.json and must not duplicate x/y",
                    actual=sorted(data),
                )
        else:
            require_fields(data, {"x_mm", "y_mm"}, "SemiPathPointV40", field_path)
            if "site_key" in data and data["site_key"] not in (None, ""):
                raise V40ValidationError("SemiPathPointV40", f"{field_path}.site_key", "WAYPOINT must be free", actual=data["site_key"])
            x_mm = _int(data["x_mm"], f"{field_path}.x_mm")
            y_mm = _int(data["y_mm"], f"{field_path}.y_mm")
        max_speed: int | None = None
        if data.get("max_speed_mmps") is not None:
            max_speed = _int(data["max_speed_mmps"], f"{field_path}.max_speed_mmps")
            if max_speed <= 0:
                raise V40ValidationError("SemiPathPointV40", f"{field_path}.max_speed_mmps", "must be positive", actual=max_speed)
        default_trim = 200.0 if point_type == ManualPathPointType.WAYPOINT else 0.0
        trim = data.get("corner_trim_mm", default_trim)
        if not isinstance(trim, (int, float)) or isinstance(trim, bool) or float(trim) < 0.0:
            raise V40ValidationError("SemiPathPointV40", f"{field_path}.corner_trim_mm", "must be a non-negative number", actual=trim)
        if point_type != ManualPathPointType.WAYPOINT:
            trim = 0.0
        return cls(
            point_type=point_type,
            site_key=site_key,
            x_mm=x_mm,
            y_mm=y_mm,
            exact_pass=bool(data.get("exact_pass", point_type != ManualPathPointType.WAYPOINT)),
            max_speed_mmps=max_speed,
            corner_trim_mm=float(trim),
            state_id=str(data.get("state_id", "")),
        )

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {"type": self.point_type.value}
        if self.site_key is not None:
            data["site_key"] = self.site_key
        else:
            data["x_mm"] = self.x_mm
            data["y_mm"] = self.y_mm
            if self.exact_pass:
                data["exact_pass"] = True
            if self.max_speed_mmps is not None:
                data["max_speed_mmps"] = self.max_speed_mmps
            if self.corner_trim_mm != 200.0:
                data["corner_trim_mm"] = self.corner_trim_mm
        if self.state_id:
            data["state_id"] = self.state_id
        return data


@dataclass(frozen=True)
class SemiPathV40:
    points: tuple[SemiPathPointV40, ...]
    notes: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, require_complete: bool = True) -> "SemiPathV40":
        if not isinstance(data, dict):
            raise V40ValidationError("SemiPathV40", "$", "semi_path must be an object", actual=type(data).__name__)
        reject_unknown_fields(data, SEMI_PATH_FIELDS, "SemiPathV40")
        require_fields(data, {"points"}, "SemiPathV40")
        if not isinstance(data["points"], list):
            raise V40ValidationError("SemiPathV40", "points", "must be an array", actual=type(data["points"]).__name__)
        points = tuple(SemiPathPointV40.from_dict(item, field_path=f"points[{index}]") for index, item in enumerate(data["points"]))
        validate_semi_path_points(points, require_complete=require_complete)
        return cls(points=points, notes=str(data.get("notes", "")))

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {"points": [point.to_dict() for point in self.points]}
        if self.notes:
            data["notes"] = self.notes
        return data

    @property
    def route_family(self) -> RouteFamily:
        return route_family_from_site_sequence(tuple(point.site_key for point in self.points if point.site_key))


def validate_semi_path_dict(data: object, *, require_complete: bool = True) -> dict[str, Any]:
    return SemiPathV40.from_dict(data, require_complete=require_complete).to_dict()


def validate_semi_path_points(points: tuple[SemiPathPointV40, ...], *, require_complete: bool = True) -> None:
    if not points:
        raise V40ValidationError("SemiPathV40", "points", "at least START is required")
    if points[0].point_type != ManualPathPointType.START or points[0].site_key != "P_START":
        raise V40ValidationError("SemiPathV40", "points[0]", "first row must be START at P_START")
    if sum(point.point_type == ManualPathPointType.START for point in points) != 1:
        raise V40ValidationError("SemiPathV40", "points", "exactly one START is required")
    if require_complete:
        if len(points) < 2 or points[-1].point_type != ManualPathPointType.ARRIVAL:
            raise V40ValidationError("SemiPathV40", "points", "complete path must end with ARRIVAL")
        route_family_from_site_sequence(tuple(point.site_key for point in points if point.site_key))


def route_family_from_site_sequence(site_sequence: tuple[str, ...]) -> RouteFamily:
    if site_sequence == ROUTE_A_SITE_SEQUENCE:
        return RouteFamily.PICK_1_TO_3
    if site_sequence == ROUTE_B_SITE_SEQUENCE:
        return RouteFamily.PICK_3_TO_1
    raise V40ValidationError(
        "SemiPathV40",
        "points",
        "fixed-site order must be one of the two legal competition routes",
        actual=list(site_sequence),
        expected=[list(ROUTE_A_SITE_SEQUENCE), list(ROUTE_B_SITE_SEQUENCE)],
    )


def _int(value: object, field_path: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise V40ValidationError("SemiPathPointV40", field_path, "must be an integer", actual=value)
    return value
