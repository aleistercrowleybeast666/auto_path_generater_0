"""Manual free-path JSON helpers for Phase 4."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.codec.legacy_rejection import reject_deleted_fields

from .enums import ManualPathPointType
from .errors import V40ValidationError, reject_unknown_fields, require_fields

MANUAL_PATH_FIELDS = {"points", "notes"}
MANUAL_POINT_FIELDS = {
    "type",
    "x_mm",
    "y_mm",
    "yaw_ddeg",
    "vx_mmps",
    "vy_mmps",
    "wz_ddegps",
    "exact_pass",
    "max_speed_mmps",
    "hints",
}


@dataclass(frozen=True)
class ManualPathPointV40:
    point_type: ManualPathPointType
    x_mm: int
    y_mm: int
    yaw_ddeg: int | None = None
    exact_pass: bool = False
    max_speed_mmps: int | None = None
    hints: dict[str, Any] | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, field_path: str) -> "ManualPathPointV40":
        reject_deleted_fields(data, "ManualPathPointV40")
        reject_unknown_fields(data, MANUAL_POINT_FIELDS, "ManualPathPointV40", field_path)
        require_fields(data, {"type", "x_mm", "y_mm"}, "ManualPathPointV40", field_path)
        point_type = ManualPathPointType(str(data["type"]))
        x_mm = _expect_int(data["x_mm"], f"{field_path}.x_mm")
        y_mm = _expect_int(data["y_mm"], f"{field_path}.y_mm")
        yaw_ddeg = data.get("yaw_ddeg")
        if point_type in (ManualPathPointType.START, ManualPathPointType.ARRIVAL):
            if yaw_ddeg is None:
                raise V40ValidationError("ManualPathPointV40", f"{field_path}.yaw_ddeg", "START/ARRIVAL require yaw_ddeg")
            yaw_ddeg = _expect_int(yaw_ddeg, f"{field_path}.yaw_ddeg")
        elif yaw_ddeg is not None:
            raise V40ValidationError("ManualPathPointV40", f"{field_path}.yaw_ddeg", "WAYPOINT yaw must be null or omitted", actual=yaw_ddeg)
        for key in ("vx_mmps", "vy_mmps", "wz_ddegps"):
            if key in data and _expect_int(data[key], f"{field_path}.{key}") != 0:
                raise V40ValidationError("ManualPathPointV40", f"{field_path}.{key}", "manual boundary velocities must be zero", actual=data[key])
        exact_pass = bool(data.get("exact_pass", point_type != ManualPathPointType.WAYPOINT))
        max_speed = None
        if "max_speed_mmps" in data and data["max_speed_mmps"] is not None:
            max_speed = _expect_int(data["max_speed_mmps"], f"{field_path}.max_speed_mmps")
            if max_speed <= 0:
                raise V40ValidationError("ManualPathPointV40", f"{field_path}.max_speed_mmps", "must be positive", actual=max_speed)
        hints = None
        if "hints" in data and data["hints"] is not None:
            if not isinstance(data["hints"], dict):
                raise V40ValidationError("ManualPathPointV40", f"{field_path}.hints", "must be an object", actual=type(data["hints"]).__name__)
            hints = dict(data["hints"])
        return cls(
            point_type=point_type,
            x_mm=x_mm,
            y_mm=y_mm,
            yaw_ddeg=yaw_ddeg,
            exact_pass=exact_pass,
            max_speed_mmps=max_speed,
            hints=hints,
        )

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "type": self.point_type.value,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
        }
        if self.yaw_ddeg is not None:
            data["yaw_ddeg"] = self.yaw_ddeg
        if self.exact_pass:
            data["exact_pass"] = True
        if self.max_speed_mmps is not None:
            data["max_speed_mmps"] = self.max_speed_mmps
        if self.hints:
            data["hints"] = dict(self.hints)
        return data


@dataclass(frozen=True)
class ManualPathV40:
    points: tuple[ManualPathPointV40, ...]
    notes: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ManualPathV40":
        reject_deleted_fields(data, "ManualPathV40")
        reject_unknown_fields(data, MANUAL_PATH_FIELDS, "ManualPathV40")
        require_fields(data, {"points"}, "ManualPathV40")
        raw_points = data["points"]
        if not isinstance(raw_points, list):
            raise V40ValidationError("ManualPathV40", "points", "must be an array", actual=type(raw_points).__name__)
        points = tuple(
            ManualPathPointV40.from_dict(item, field_path=f"points[{index}]")
            for index, item in enumerate(raw_points)
        )
        validate_manual_path_points(points)
        return cls(points=points, notes=str(data.get("notes", "")))

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {"points": [point.to_dict() for point in self.points]}
        if self.notes:
            data["notes"] = self.notes
        return data


def validate_manual_path_dict(data: object) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise V40ValidationError("ManualPathV40", "$", "manual_path must be an object", actual=type(data).__name__)
    return ManualPathV40.from_dict(data).to_dict()


def validate_manual_path_points(points: tuple[ManualPathPointV40, ...]) -> None:
    if len(points) < 2:
        raise V40ValidationError("ManualPathV40", "points", "at least START and ARRIVAL are required")
    if points[0].point_type != ManualPathPointType.START:
        raise V40ValidationError("ManualPathV40", "points[0].type", "first point must be START", actual=points[0].point_type.value)
    start_count = sum(1 for point in points if point.point_type == ManualPathPointType.START)
    if start_count != 1:
        raise V40ValidationError("ManualPathV40", "points", "exactly one START is required", actual=start_count)
    arrival_count = sum(1 for point in points if point.point_type == ManualPathPointType.ARRIVAL)
    if arrival_count < 1:
        raise V40ValidationError("ManualPathV40", "points", "at least one ARRIVAL is required")
    if points[-1].point_type != ManualPathPointType.ARRIVAL:
        raise V40ValidationError("ManualPathV40", f"points[{len(points) - 1}].type", "last point must be ARRIVAL", actual=points[-1].point_type.value)


def _expect_int(value: object, field_path: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise V40ValidationError("ManualPathPointV40", field_path, "must be an integer", actual=value)
    return value
