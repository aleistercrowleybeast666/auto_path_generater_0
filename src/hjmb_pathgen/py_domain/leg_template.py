"""Strict V4.0 models for operator-authored directed-leg templates."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from .errors import V40ValidationError, expect_equal, reject_unknown_fields, require_fields
from .leg import LegV40

LEG_TEMPLATES_FORMAT = "HJMB_LEG_TEMPLATES_JSON_V40"
LEG_TEMPLATE_INSTANCES_FORMAT = "HJMB_LEG_TEMPLATE_INSTANCES_JSON_V40"
LEG_TEMPLATE_VALIDATION_REPORT_FORMAT = "HJMB_LEG_TEMPLATE_VALIDATION_REPORT_JSON_V40"
LEG_TEMPLATE_CONFIG_VERSION = 1


class LegTemplateRouteFamily(StrEnum):
    PICK_1_TO_3 = "PICK_1_TO_3"
    PICK_3_TO_1 = "PICK_3_TO_1"


class LegTemplateState(StrEnum):
    DISABLED = "DISABLED"
    DRAFT = "DRAFT"
    STALE = "STALE"
    PASSED = "PASSED"
    PARTIAL = "PARTIAL"
    FAILED = "FAILED"


class LegTemplateInstanceState(StrEnum):
    PASSED = "PASSED"
    FAILED = "FAILED"
    STALE = "STALE"


@dataclass(frozen=True)
class LegTemplateWaypointV40:
    x_mm: float
    y_mm: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateWaypointV40":
        reject_unknown_fields(data, {"x_mm", "y_mm"}, "LegTemplateWaypointV40")
        require_fields(data, {"x_mm", "y_mm"}, "LegTemplateWaypointV40")
        return cls(_finite_number(data["x_mm"], "x_mm"), _finite_number(data["y_mm"], "y_mm"))

    def to_dict(self) -> dict[str, float]:
        return {"x_mm": self.x_mm, "y_mm": self.y_mm}


_TEMPLATE_FIELDS = {
    "template_id", "enabled", "route_family", "from_site", "to_site", "waypoints",
    "state", "template_hash", "dependency_hashes", "last_validated_hash",
    "orphaned", "audit_messages",
}


@dataclass(frozen=True)
class LegTemplateV40:
    template_id: str
    enabled: bool
    route_family: LegTemplateRouteFamily
    from_site: str
    to_site: str
    waypoints: tuple[LegTemplateWaypointV40, ...]
    state: LegTemplateState
    template_hash: str
    dependency_hashes: dict[str, str]
    last_validated_hash: str = ""
    orphaned: bool = False
    audit_messages: tuple[str, ...] = ()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateV40":
        reject_unknown_fields(data, _TEMPLATE_FIELDS, "LegTemplateV40")
        require_fields(data, _TEMPLATE_FIELDS, "LegTemplateV40")
        if not isinstance(data["enabled"], bool) or not isinstance(data["orphaned"], bool):
            raise V40ValidationError("LegTemplateV40", "enabled/orphaned", "must be booleans")
        return cls(
            template_id=_nonempty(data["template_id"], "template_id"),
            enabled=data["enabled"],
            route_family=LegTemplateRouteFamily(str(data["route_family"])),
            from_site=_nonempty(data["from_site"], "from_site"),
            to_site=_nonempty(data["to_site"], "to_site"),
            waypoints=tuple(LegTemplateWaypointV40.from_dict(_dict(item, "waypoints")) for item in _list(data["waypoints"], "waypoints")),
            state=LegTemplateState(str(data["state"])),
            template_hash=str(data["template_hash"]),
            dependency_hashes=_string_dict(data["dependency_hashes"], "dependency_hashes"),
            last_validated_hash=str(data["last_validated_hash"]),
            orphaned=data["orphaned"],
            audit_messages=tuple(str(item) for item in _list(data["audit_messages"], "audit_messages")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "template_id": self.template_id, "enabled": self.enabled,
            "route_family": self.route_family.value, "from_site": self.from_site, "to_site": self.to_site,
            "waypoints": [item.to_dict() for item in self.waypoints], "state": self.state.value,
            "template_hash": self.template_hash, "dependency_hashes": dict(self.dependency_hashes),
            "last_validated_hash": self.last_validated_hash, "orphaned": self.orphaned,
            "audit_messages": list(self.audit_messages),
        }


@dataclass(frozen=True)
class LegTemplatesV40:
    project_id: str
    dependency_hashes: dict[str, str]
    templates: tuple[LegTemplateV40, ...]
    config_version: int = LEG_TEMPLATE_CONFIG_VERSION
    format: str = LEG_TEMPLATES_FORMAT

    @classmethod
    def empty(cls, project_id: str = "") -> "LegTemplatesV40":
        return cls(project_id=project_id, dependency_hashes={}, templates=())

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplatesV40":
        fields = {"format", "config_version", "project_id", "dependency_hashes", "templates"}
        reject_unknown_fields(data, fields, "LegTemplatesV40")
        require_fields(data, fields, "LegTemplatesV40")
        expect_equal(data["format"], LEG_TEMPLATES_FORMAT, "LegTemplatesV40", "format")
        expect_equal(data["config_version"], LEG_TEMPLATE_CONFIG_VERSION, "LegTemplatesV40", "config_version")
        templates = tuple(LegTemplateV40.from_dict(_dict(item, "templates")) for item in _list(data["templates"], "templates"))
        _unique((item.template_id for item in templates), "LegTemplatesV40", "templates.template_id")
        return cls(str(data["project_id"]), _string_dict(data["dependency_hashes"], "dependency_hashes"), templates)

    def to_dict(self) -> dict[str, Any]:
        return {"format": self.format, "config_version": self.config_version, "project_id": self.project_id,
                "dependency_hashes": dict(self.dependency_hashes), "templates": [item.to_dict() for item in self.templates]}


@dataclass(frozen=True)
class LegTemplateFailureV40:
    code: str
    message: str
    details: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateFailureV40":
        fields = {"code", "message", "details"}
        reject_unknown_fields(data, fields, "LegTemplateFailureV40")
        require_fields(data, fields, "LegTemplateFailureV40")
        return cls(_nonempty(data["code"], "code"), str(data["message"]), _dict(data["details"], "details"))

    def to_dict(self) -> dict[str, Any]:
        return {"code": self.code, "message": self.message, "details": self.details}


_INSTANCE_FIELDS = {
    "instance_id", "template_id", "from_state_key", "to_state_key", "from_pose", "to_pose",
    "from_unload_pose_profile_id", "to_unload_pose_profile_id", "state", "planned_time_ms",
    "min_clearance_mm", "analysis_metrics", "hashes", "failures", "compiled_leg",
}


@dataclass(frozen=True)
class LegTemplateInstanceV40:
    instance_id: str
    template_id: str
    from_state_key: str
    to_state_key: str
    from_pose: dict[str, float]
    to_pose: dict[str, float]
    from_unload_pose_profile_id: str | None
    to_unload_pose_profile_id: str | None
    state: LegTemplateInstanceState
    planned_time_ms: int
    min_clearance_mm: float | None
    analysis_metrics: dict[str, Any]
    hashes: dict[str, str]
    failures: tuple[LegTemplateFailureV40, ...]
    compiled_leg: LegV40 | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateInstanceV40":
        reject_unknown_fields(data, _INSTANCE_FIELDS, "LegTemplateInstanceV40")
        require_fields(data, _INSTANCE_FIELDS, "LegTemplateInstanceV40")
        raw_leg = data["compiled_leg"]
        return cls(
            instance_id=_nonempty(data["instance_id"], "instance_id"), template_id=_nonempty(data["template_id"], "template_id"),
            from_state_key=_nonempty(data["from_state_key"], "from_state_key"), to_state_key=_nonempty(data["to_state_key"], "to_state_key"),
            from_pose=_pose(data["from_pose"], "from_pose"), to_pose=_pose(data["to_pose"], "to_pose"),
            from_unload_pose_profile_id=_optional_string(data["from_unload_pose_profile_id"], "from_unload_pose_profile_id"),
            to_unload_pose_profile_id=_optional_string(data["to_unload_pose_profile_id"], "to_unload_pose_profile_id"),
            state=LegTemplateInstanceState(str(data["state"])), planned_time_ms=int(data["planned_time_ms"]),
            min_clearance_mm=None if data["min_clearance_mm"] is None else _finite_number(data["min_clearance_mm"], "min_clearance_mm"),
            analysis_metrics=_dict(data["analysis_metrics"], "analysis_metrics"), hashes=_string_dict(data["hashes"], "hashes"),
            failures=tuple(LegTemplateFailureV40.from_dict(_dict(item, "failures")) for item in _list(data["failures"], "failures")),
            compiled_leg=None if raw_leg is None else LegV40.from_dict(_dict(raw_leg, "compiled_leg")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "instance_id": self.instance_id, "template_id": self.template_id,
            "from_state_key": self.from_state_key, "to_state_key": self.to_state_key,
            "from_pose": self.from_pose, "to_pose": self.to_pose,
            "from_unload_pose_profile_id": self.from_unload_pose_profile_id,
            "to_unload_pose_profile_id": self.to_unload_pose_profile_id, "state": self.state.value,
            "planned_time_ms": self.planned_time_ms, "min_clearance_mm": self.min_clearance_mm,
            "analysis_metrics": self.analysis_metrics, "hashes": dict(self.hashes),
            "failures": [item.to_dict() for item in self.failures],
            "compiled_leg": self.compiled_leg.to_dict() if self.compiled_leg else None,
        }


@dataclass(frozen=True)
class LegTemplateInstancesV40:
    project_id: str
    dependency_hashes: dict[str, str]
    instances: tuple[LegTemplateInstanceV40, ...]
    config_version: int = LEG_TEMPLATE_CONFIG_VERSION
    format: str = LEG_TEMPLATE_INSTANCES_FORMAT

    @classmethod
    def empty(cls, project_id: str = "") -> "LegTemplateInstancesV40":
        return cls(project_id, {}, ())

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateInstancesV40":
        fields = {"format", "config_version", "project_id", "dependency_hashes", "instances"}
        reject_unknown_fields(data, fields, "LegTemplateInstancesV40")
        require_fields(data, fields, "LegTemplateInstancesV40")
        expect_equal(data["format"], LEG_TEMPLATE_INSTANCES_FORMAT, "LegTemplateInstancesV40", "format")
        expect_equal(data["config_version"], LEG_TEMPLATE_CONFIG_VERSION, "LegTemplateInstancesV40", "config_version")
        items = tuple(LegTemplateInstanceV40.from_dict(_dict(item, "instances")) for item in _list(data["instances"], "instances"))
        _unique((item.instance_id for item in items), "LegTemplateInstancesV40", "instances.instance_id")
        return cls(str(data["project_id"]), _string_dict(data["dependency_hashes"], "dependency_hashes"), items)

    def to_dict(self) -> dict[str, Any]:
        return {"format": self.format, "config_version": self.config_version, "project_id": self.project_id,
                "dependency_hashes": dict(self.dependency_hashes), "instances": [item.to_dict() for item in self.instances]}


@dataclass(frozen=True)
class LegTemplateValidationEntryV40:
    template_id: str
    status: LegTemplateState
    instance_counts: dict[str, int]
    missing_profiles: tuple[str, ...]
    errors: tuple[str, ...]
    instance_reports: tuple[dict[str, Any], ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateValidationEntryV40":
        fields = {"template_id", "status", "instance_counts", "missing_profiles", "errors", "instance_reports"}
        reject_unknown_fields(data, fields, "LegTemplateValidationEntryV40")
        require_fields(data, fields, "LegTemplateValidationEntryV40")
        counts = _dict(data["instance_counts"], "instance_counts")
        if any(not isinstance(value, int) or isinstance(value, bool) or value < 0 for value in counts.values()):
            raise V40ValidationError("LegTemplateValidationEntryV40", "instance_counts", "values must be nonnegative integers")
        reports = tuple(_instance_report(item) for item in _list(data["instance_reports"], "instance_reports"))
        return cls(str(data["template_id"]), LegTemplateState(str(data["status"])), {str(k): int(v) for k, v in counts.items()},
                   tuple(str(x) for x in _list(data["missing_profiles"], "missing_profiles")),
                   tuple(str(x) for x in _list(data["errors"], "errors")), reports)

    def to_dict(self) -> dict[str, Any]:
        return {"template_id": self.template_id, "status": self.status.value, "instance_counts": self.instance_counts,
                "missing_profiles": list(self.missing_profiles), "errors": list(self.errors),
                "instance_reports": list(self.instance_reports)}


@dataclass(frozen=True)
class LegTemplateValidationReportV40:
    project_id: str
    dependency_hashes: dict[str, str]
    template_reports: tuple[LegTemplateValidationEntryV40, ...]
    config_version: int = LEG_TEMPLATE_CONFIG_VERSION
    format: str = LEG_TEMPLATE_VALIDATION_REPORT_FORMAT

    @classmethod
    def empty(cls, project_id: str = "") -> "LegTemplateValidationReportV40":
        return cls(project_id, {}, ())

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "LegTemplateValidationReportV40":
        fields = {"format", "config_version", "project_id", "dependency_hashes", "template_reports"}
        reject_unknown_fields(data, fields, "LegTemplateValidationReportV40")
        require_fields(data, fields, "LegTemplateValidationReportV40")
        expect_equal(data["format"], LEG_TEMPLATE_VALIDATION_REPORT_FORMAT, "LegTemplateValidationReportV40", "format")
        expect_equal(data["config_version"], LEG_TEMPLATE_CONFIG_VERSION, "LegTemplateValidationReportV40", "config_version")
        reports = tuple(LegTemplateValidationEntryV40.from_dict(_dict(item, "template_reports")) for item in _list(data["template_reports"], "template_reports"))
        _unique((item.template_id for item in reports), "LegTemplateValidationReportV40", "template_reports.template_id")
        return cls(str(data["project_id"]), _string_dict(data["dependency_hashes"], "dependency_hashes"), reports)

    def to_dict(self) -> dict[str, Any]:
        return {"format": self.format, "config_version": self.config_version, "project_id": self.project_id,
                "dependency_hashes": dict(self.dependency_hashes), "template_reports": [item.to_dict() for item in self.template_reports]}


def _instance_report(value: object) -> dict[str, Any]:
    data = _dict(value, "instance_report")
    fields = {"instance_id", "status", "failure_reasons", "metrics"}
    reject_unknown_fields(data, fields, "LegTemplateInstanceReportV40")
    require_fields(data, fields, "LegTemplateInstanceReportV40")
    LegTemplateInstanceState(str(data["status"]))
    _list(data["failure_reasons"], "failure_reasons")
    _dict(data["metrics"], "metrics")
    return {"instance_id": str(data["instance_id"]), "status": str(data["status"]),
            "failure_reasons": [str(x) for x in data["failure_reasons"]], "metrics": dict(data["metrics"])}


def _dict(value: object, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict): raise V40ValidationError("LegTemplateV40", field_name, "must be an object")
    return dict(value)

def _list(value: object, field_name: str) -> list[Any]:
    if not isinstance(value, list): raise V40ValidationError("LegTemplateV40", field_name, "must be an array")
    return value

def _finite_number(value: object, field_name: str) -> float:
    import math
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise V40ValidationError("LegTemplateV40", field_name, "must be a finite number")
    return float(value)

def _nonempty(value: object, field_name: str) -> str:
    text = str(value)
    if not text: raise V40ValidationError("LegTemplateV40", field_name, "must not be empty")
    return text

def _string_dict(value: object, field_name: str) -> dict[str, str]:
    data = _dict(value, field_name)
    if any(not isinstance(k, str) or not isinstance(v, str) for k, v in data.items()):
        raise V40ValidationError("LegTemplateV40", field_name, "keys and values must be strings")
    return dict(data)

def _optional_string(value: object, field_name: str) -> str | None:
    if value is None: return None
    if not isinstance(value, str) or not value: raise V40ValidationError("LegTemplateV40", field_name, "must be null or nonempty string")
    return value

def _pose(value: object, field_name: str) -> dict[str, float]:
    data = _dict(value, field_name)
    fields = {"x_mm", "y_mm", "yaw_ddeg"}
    reject_unknown_fields(data, fields, "LegTemplatePoseV40")
    require_fields(data, fields, "LegTemplatePoseV40")
    return {key: _finite_number(data[key], f"{field_name}.{key}") for key in ("x_mm", "y_mm", "yaw_ddeg")}

def _unique(values: object, object_type: str, field_name: str) -> None:
    items = list(values)
    if len(items) != len(set(items)): raise V40ValidationError(object_type, field_name, "values must be unique")
