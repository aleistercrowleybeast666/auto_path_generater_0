# -*- coding: utf-8 -*-
"""HJMB V3.5 project, trajectory, and protocol data models."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field as dc_field, replace
from typing import Dict, List, Optional, Tuple

from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG

PROJECT_FORMAT = "HJMB_PATH_EDITOR_JSON_V35"

POINT_TYPE_START = "START"
POINT_TYPE_WAYPOINT = "WAYPOINT"
POINT_TYPE_ARRIVAL = "ARRIVAL"
POINT_TYPES = (POINT_TYPE_START, POINT_TYPE_WAYPOINT, POINT_TYPE_ARRIVAL)

PATH_MODE_FREE = "FREE"
PATH_MODE_FIXED_8 = "FIXED_8"
PATH_MODES = (PATH_MODE_FREE, PATH_MODE_FIXED_8)

YAW_ROTATION_SHORTEST = "SHORTEST"
YAW_ROTATION_CW_ONLY = "CW_ONLY"
YAW_ROTATION_CCW_ONLY = "CCW_ONLY"
YAW_ROTATION_POLICIES = (
    YAW_ROTATION_SHORTEST,
    YAW_ROTATION_CW_ONLY,
    YAW_ROTATION_CCW_ONLY,
)

SITE_ID_FREE = 0xFF

ACTION_MODE_STOP_AND_WAIT = "STOP_AND_WAIT"
ACTION_MODE_ASYNC = "ASYNC"
ACTION_MODE_KINEMATIC = "KINEMATIC"
ACTION_MODE_NAMES = (
    ACTION_MODE_STOP_AND_WAIT,
    ACTION_MODE_ASYNC,
    ACTION_MODE_KINEMATIC,
)
ACTION_MODE_CODES = {
    ACTION_MODE_STOP_AND_WAIT: 0,
    ACTION_MODE_ASYNC: 1,
    ACTION_MODE_KINEMATIC: 2,
}
ACTION_MODE_NAMES_BY_CODE = {code: name for name, code in ACTION_MODE_CODES.items()}

PATH_ACT_PREP_PICK_1 = 0x11
PATH_ACT_PREP_PICK_2L = 0x12
PATH_ACT_PREP_PICK_2R = 0x13
PATH_ACT_PREP_PICK_3 = 0x14
PATH_ACT_PICK = 0x20
PATH_ACT_DROP_1 = 0x31
PATH_ACT_DROP_2 = 0x32
PATH_ACT_DROP_3 = 0x33
PATH_ACT_DROP_12 = 0x34
PATH_ACT_DROP_23 = 0x35
PATH_ACT_STORE = 0x40
PATH_ACT_PREP_STORE_1 = 0x41
PATH_ACT_PREP_STORE_2 = 0x42
PATH_ACT_PREP_STORE_3 = 0x43

ACTIONS: Dict[int, str] = {
    PATH_ACT_PREP_PICK_1: "PREP_PICK_1",
    PATH_ACT_PREP_PICK_2L: "PREP_PICK_2L",
    PATH_ACT_PREP_PICK_2R: "PREP_PICK_2R",
    PATH_ACT_PREP_PICK_3: "PREP_PICK_3",
    PATH_ACT_PICK: "PICK",
    PATH_ACT_DROP_1: "DROP_1",
    PATH_ACT_DROP_2: "DROP_2",
    PATH_ACT_DROP_3: "DROP_3",
    PATH_ACT_DROP_12: "DROP_12",
    PATH_ACT_DROP_23: "DROP_23",
    PATH_ACT_STORE: "STORE",
    PATH_ACT_PREP_STORE_1: "PREP_STORE_1",
    PATH_ACT_PREP_STORE_2: "PREP_STORE_2",
    PATH_ACT_PREP_STORE_3: "PREP_STORE_3",
}
ACTION_CODES = {name: code for code, name in ACTIONS.items()}
PREP_STORE_ACTION_SLOTS = {
    PATH_ACT_PREP_STORE_1: 1,
    PATH_ACT_PREP_STORE_2: 2,
    PATH_ACT_PREP_STORE_3: 3,
}
DROP_ACTIONS = (
    PATH_ACT_DROP_1,
    PATH_ACT_DROP_2,
    PATH_ACT_DROP_3,
    PATH_ACT_DROP_12,
    PATH_ACT_DROP_23,
)

TRAJ_FLAG_START = 0x01
TRAJ_FLAG_ARRIVAL = 0x02
TRAJ_FLAG_WAYPOINT = 0x04
TRAJ_FLAG_END = 0x08
VALID_TRAJ_FLAGS_MASK = (
    TRAJ_FLAG_START | TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_WAYPOINT | TRAJ_FLAG_END
)

MAX_EDIT_POINTS = 100
MAX_NODES = 2500
MAX_ACTIONS = 32
MAX_ARRIVALS = 32
MAX_TRAJ_ID = 359

FIXED_SITE_KEYS = (
    "P_START",
    "P_PICK_1",
    "P_PICK_2L",
    "P_PICK_2R",
    "P_PICK_3",
    "P_DROP_1",
    "P_DROP_2",
    "P_DROP_3",
)


def fixed_site_key_allows_yaw_override(site_key: str) -> bool:
    """All eight fixed sites may store 0xFFFF to mean yaw is unconstrained."""
    return site_key in FIXED_SITE_KEYS

REMOVED_TOP_LEVEL_FIELDS = {
    "cut_in",
    "preview_initial_pose",
}
REMOVED_POINT_FIELDS = {
    "stop_required",
    "gate_id",
    "marker_id",
    "scan",
    "is_end",
    "yaw_mode",
}
REMOVED_ACTION_FIELDS = {
    "unlock_gate_id",
    "flags",
    "arm_s_mm",
    "disarm_s_mm",
    "min_wait_ms",
    "trigger",
    "trigger_point_id",
    "trigger_offset_mm",
    "trigger_s_mm",
    "expire_s_mm",
    "window_start",
    "window_end",
    "window_start_point_id",
    "window_start_offset_mm",
    "window_end_point_id",
    "window_end_offset_mm",
    "check_start_s_mm",
    "departure_action_seq",
    "completion_barrier",
    "required_action_seq",
}


def parse_int(value, field_name: str = "value") -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    try:
        return int(str(value).strip(), 0)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是十进制或 0x 前缀整数，当前为 {value!r}") from exc


def parse_optional_int(value, field_name: str = "value") -> Optional[int]:
    if value is None or value == "":
        return None
    return parse_int(value, field_name)


def parse_float(value, field_name: str = "value") -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    try:
        return float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是数值，当前为 {value!r}") from exc


def hex8(value: int) -> str:
    return f"0x{value & 0xFF:02X}"


def _reject_fields(data: dict, removed: set[str], scope: str) -> None:
    for field_name in sorted(removed & set(data)):
        raise ValueError(f"V3.5 已删除 {scope}.{field_name}")


@dataclass
class FieldConfig:
    width_mm: int = 4000
    height_mm: int = 2000

    @classmethod
    def from_dict(cls, data: dict) -> "FieldConfig":
        return cls(
            width_mm=parse_int(data.get("width_mm", 4000), "field.width_mm"),
            height_mm=parse_int(data.get("height_mm", 2000), "field.height_mm"),
        )


@dataclass
class PlannerConfig:
    max_speed_mmps: int = 2000
    nominal_spacing_mm: int = 25
    max_spacing_mm: int = 50
    linear_accel_mmps2: int = 1200
    lateral_accel_mmps2: int = 1200
    max_wz_radps: float = 4.0
    angular_accel_moving_radps2: float = 2.0
    angular_accel_rotate_radps2: float = 5.0
    yaw_rotation_policy: str = YAW_ROTATION_SHORTEST
    max_ref_lead_mm: int = 50
    max_iterations: int = 40
    speed_convergence_mmps: float = 0.5

    @classmethod
    def from_dict(cls, data: dict) -> "PlannerConfig":
        policy = str(
            data.get("yaw_rotation_policy", YAW_ROTATION_SHORTEST)
        ).upper()
        if policy not in YAW_ROTATION_POLICIES:
            raise ValueError(f"planner.yaw_rotation_policy={policy!r} 非法")
        return cls(
            max_speed_mmps=parse_int(data.get("max_speed_mmps", 2000), "planner.max_speed_mmps"),
            nominal_spacing_mm=parse_int(
                data.get("nominal_spacing_mm", 25), "planner.nominal_spacing_mm"
            ),
            max_spacing_mm=parse_int(data.get("max_spacing_mm", 50), "planner.max_spacing_mm"),
            linear_accel_mmps2=parse_int(
                data.get("linear_accel_mmps2", 1200), "planner.linear_accel_mmps2"
            ),
            lateral_accel_mmps2=parse_int(
                data.get("lateral_accel_mmps2", 1200), "planner.lateral_accel_mmps2"
            ),
            max_wz_radps=parse_float(data.get("max_wz_radps", 4.0), "planner.max_wz_radps"),
            angular_accel_moving_radps2=parse_float(
                data.get("angular_accel_moving_radps2", 2.0),
                "planner.angular_accel_moving_radps2",
            ),
            angular_accel_rotate_radps2=parse_float(
                data.get("angular_accel_rotate_radps2", 5.0),
                "planner.angular_accel_rotate_radps2",
            ),
            yaw_rotation_policy=policy,
            max_ref_lead_mm=parse_int(
                data.get("max_ref_lead_mm", 50), "planner.max_ref_lead_mm"
            ),
            max_iterations=parse_int(data.get("max_iterations", 40), "planner.max_iterations"),
            speed_convergence_mmps=parse_float(
                data.get("speed_convergence_mmps", 0.5),
                "planner.speed_convergence_mmps",
            ),
        )


@dataclass
class StartCheckConfig:
    position_tolerance_mm: int = 30
    yaw_tolerance_ddeg: int = 50
    stable_time_ms: int = 100

    @classmethod
    def from_dict(cls, data: dict) -> "StartCheckConfig":
        return cls(
            position_tolerance_mm=parse_int(
                data.get("position_tolerance_mm", 30),
                "start_check.position_tolerance_mm",
            ),
            yaw_tolerance_ddeg=parse_int(
                data.get("yaw_tolerance_ddeg", 50),
                "start_check.yaw_tolerance_ddeg",
            ),
            stable_time_ms=parse_int(
                data.get("stable_time_ms", 100),
                "start_check.stable_time_ms",
            ),
        )


@dataclass
class ArrivalCheckConfig:
    position_tolerance_mm: int = 20
    yaw_tolerance_ddeg: int = 30
    speed_tolerance_mmps: int = 60
    wz_tolerance_ddegps: int = 50
    stable_time_ms: int = 100

    @classmethod
    def from_dict(cls, data: dict) -> "ArrivalCheckConfig":
        return cls(
            position_tolerance_mm=parse_int(
                data.get("position_tolerance_mm", 20),
                "arrival_check.position_tolerance_mm",
            ),
            yaw_tolerance_ddeg=parse_int(
                data.get("yaw_tolerance_ddeg", 30),
                "arrival_check.yaw_tolerance_ddeg",
            ),
            speed_tolerance_mmps=parse_int(
                data.get("speed_tolerance_mmps", 60),
                "arrival_check.speed_tolerance_mmps",
            ),
            wz_tolerance_ddegps=parse_int(
                data.get("wz_tolerance_ddegps", 50),
                "arrival_check.wz_tolerance_ddegps",
            ),
            stable_time_ms=parse_int(
                data.get("stable_time_ms", 100),
                "arrival_check.stable_time_ms",
            ),
        )


@dataclass
class FixedSite:
    site_id: int
    site_key: str
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_ddeg: int = 0

    @classmethod
    def from_dict(cls, data: dict) -> "FixedSite":
        return cls(
            site_id=parse_int(data.get("site_id", 0), "fixed_sites.site_id"),
            site_key=str(data.get("site_key", "")),
            x_mm=parse_float(data.get("x_mm", 0), "fixed_sites.x_mm"),
            y_mm=parse_float(data.get("y_mm", 0), "fixed_sites.y_mm"),
            yaw_ddeg=parse_int(data.get("yaw_ddeg", 0), "fixed_sites.yaw_ddeg"),
        )


def default_fixed_sites() -> List[FixedSite]:
    return [FixedSite(site_id=index, site_key=key) for index, key in enumerate(FIXED_SITE_KEYS)]


def validate_fixed_sites(sites: List[FixedSite]) -> List[str]:
    errors: List[str] = []
    if len(sites) != len(FIXED_SITE_KEYS):
        errors.append(f"fixed_sites 必须恰好 8 行，当前为 {len(sites)}")
        return errors
    seen: set[int] = set()
    for site in sites:
        if site.site_id in seen:
            errors.append(f"fixed_sites site_id={site.site_id} 重复")
        seen.add(site.site_id)
        if not 0 <= site.site_id < len(FIXED_SITE_KEYS):
            errors.append(f"fixed_sites site_id={site.site_id} 必须为 0~7")
            continue
        expected_key = FIXED_SITE_KEYS[site.site_id]
        if site.site_key != expected_key:
            errors.append(
                f"fixed_sites[{site.site_id}].site_key={site.site_key!r}，应为 {expected_key!r}"
            )
        if not (-32768 <= site.x_mm <= 32767 and -32768 <= site.y_mm <= 32767):
            errors.append(f"fixed_sites[{site.site_id}] 坐标超出 int16_t 范围")
        if site.yaw_ddeg != YAW_UNSPECIFIED_DDEG and not -32768 <= site.yaw_ddeg <= 32767:
            errors.append(f"fixed_sites[{site.site_id}].yaw_ddeg 超出 int16_t 范围")
        if (
            site.yaw_ddeg == YAW_UNSPECIFIED_DDEG
            and not fixed_site_key_allows_yaw_override(expected_key)
        ):
            errors.append(
                f"fixed_sites[{site.site_id}].yaw_ddeg=0xFFFF 表示该固定点不约束到点方向"
            )
    if seen != set(range(len(FIXED_SITE_KEYS))):
        errors.append(f"fixed_sites site_id 必须恰好覆盖 0~7，当前为 {sorted(seen)}")
    return errors


@dataclass
class VehicleProfile:
    wheel_radius_mm: float = 76.0
    rotation_radius_mm: float = 260.0
    wheel_plan_limit_rpm: int = 420
    wheel_hard_limit_rpm: int = 450
    mecanum_convention: str = "X_FL_FR_RL_RR"
    geometry_note: str = "示例初值，导出前请按实车核对"

    @classmethod
    def from_dict(cls, data: dict) -> "VehicleProfile":
        return cls(
            wheel_radius_mm=parse_float(
                data.get("wheel_radius_mm", 76.0), "vehicle_profile.wheel_radius_mm"
            ),
            rotation_radius_mm=parse_float(
                data.get("rotation_radius_mm", 260.0),
                "vehicle_profile.rotation_radius_mm",
            ),
            wheel_plan_limit_rpm=parse_int(
                data.get("wheel_plan_limit_rpm", 420),
                "vehicle_profile.wheel_plan_limit_rpm",
            ),
            wheel_hard_limit_rpm=parse_int(
                data.get("wheel_hard_limit_rpm", 450),
                "vehicle_profile.wheel_hard_limit_rpm",
            ),
            mecanum_convention=str(data.get("mecanum_convention", "X_FL_FR_RL_RR")),
            geometry_note=str(data.get("geometry_note", "示例初值，导出前请按实车核对")),
        )


@dataclass
class StartRegion:
    enabled: bool = False
    polygon_mm: List[List[float]] = dc_field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "StartRegion":
        polygon = []
        for index, pair in enumerate(data.get("polygon_mm", [])):
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                raise ValueError(f"start_region.polygon_mm[{index}] 必须为 [x, y]")
            polygon.append(
                [
                    parse_float(pair[0], f"start_region.polygon_mm[{index}].x"),
                    parse_float(pair[1], f"start_region.polygon_mm[{index}].y"),
                ]
            )
        return cls(enabled=bool(data.get("enabled", False)), polygon_mm=polygon)


@dataclass
class OverlayConfig:
    selected_analysis_mode: str = "normal"
    scale_mode: str = "planner"
    speed_threshold_mmps: int = 1500
    accel_threshold_mmps2: int = 800
    beta_threshold_radps2: float = 1.5

    @classmethod
    def from_dict(cls, data: dict) -> "OverlayConfig":
        return cls(
            selected_analysis_mode=str(data.get("selected_analysis_mode", "normal")),
            scale_mode=str(data.get("scale_mode", "planner")),
            speed_threshold_mmps=parse_int(
                data.get("speed_threshold_mmps", 1500),
                "overlay.speed_threshold_mmps",
            ),
            accel_threshold_mmps2=parse_int(
                data.get("accel_threshold_mmps2", 800),
                "overlay.accel_threshold_mmps2",
            ),
            beta_threshold_radps2=parse_float(
                data.get("beta_threshold_radps2", 1.5),
                "overlay.beta_threshold_radps2",
            ),
        )


@dataclass
class ControllerPreview:
    kp_x: float = 1.0
    kp_y: float = 1.0
    kp_yaw: float = 1.0

    @classmethod
    def from_dict(cls, data: dict) -> "ControllerPreview":
        return cls(
            kp_x=parse_float(data.get("kp_x", 1.0), "controller_preview.kp_x"),
            kp_y=parse_float(data.get("kp_y", 1.0), "controller_preview.kp_y"),
            kp_yaw=parse_float(data.get("kp_yaw", 1.0), "controller_preview.kp_yaw"),
        )


def _default_action_durations() -> Dict[str, int]:
    return {
        "PREP_PICK_1": 500,
        "PREP_PICK_2L": 500,
        "PREP_PICK_2R": 500,
        "PREP_PICK_3": 500,
        "PICK": 1200,
        "DROP_1": 700,
        "DROP_2": 700,
        "DROP_3": 700,
        "DROP_12": 700,
        "DROP_23": 700,
        "PREP_STORE_1": 400,
        "PREP_STORE_2": 400,
        "PREP_STORE_3": 400,
        "STORE": 1500,
    }


@dataclass
class MechanismProfile:
    action_duration_ms: Dict[str, int] = dc_field(default_factory=_default_action_durations)
    drop_safety_margin_ms: int = 300

    @classmethod
    def from_dict(cls, data: dict) -> "MechanismProfile":
        durations = _default_action_durations()
        for name, value in data.get("action_duration_ms", {}).items():
            target_name = str(name).upper()
            if target_name in durations:
                durations[target_name] = parse_int(
                    value, f"mechanism_profile.action_duration_ms.{name}"
                )
        return cls(
            action_duration_ms=durations,
            drop_safety_margin_ms=parse_int(
                data.get("drop_safety_margin_ms", 300),
                "mechanism_profile.drop_safety_margin_ms",
            ),
        )


@dataclass
class EditPoint:
    point_id: int = 0
    type: str = POINT_TYPE_WAYPOINT
    site_id: int = SITE_ID_FREE
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_ddeg: int = YAW_UNSPECIFIED_DDEG
    max_speed_mmps: int = 0
    corner_trim_mm: float = 200.0
    exact_pass: bool = False

    @classmethod
    def from_dict(cls, data: dict) -> "EditPoint":
        _reject_fields(data, REMOVED_POINT_FIELDS, "point")
        point_type = str(data.get("type", POINT_TYPE_WAYPOINT)).upper()
        default_yaw = YAW_UNSPECIFIED_DDEG if point_type == POINT_TYPE_WAYPOINT else 0
        default_site = SITE_ID_FREE
        default_corner_trim = 0 if point_type == POINT_TYPE_START else 200
        default_exact_pass = point_type == POINT_TYPE_START
        yaw_ddeg = parse_int(data.get("yaw_ddeg", default_yaw), "point.yaw_ddeg")
        if yaw_ddeg == 0xFF:
            yaw_ddeg = YAW_UNSPECIFIED_DDEG
        return cls(
            point_id=parse_int(data.get("point_id", 0), "point.point_id"),
            type=point_type,
            site_id=parse_int(data.get("site_id", default_site), "point.site_id"),
            x_mm=parse_float(data.get("x_mm", 0), "point.x_mm"),
            y_mm=parse_float(data.get("y_mm", 0), "point.y_mm"),
            yaw_ddeg=yaw_ddeg,
            max_speed_mmps=parse_int(data.get("max_speed_mmps", 0), "point.max_speed_mmps"),
            corner_trim_mm=parse_float(
                data.get("corner_trim_mm", default_corner_trim), "point.corner_trim_mm"
            ),
            exact_pass=bool(data.get("exact_pass", default_exact_pass)),
        )


@dataclass
class MechanicalAction:
    action_seq: int = 0
    action: int = PATH_ACT_PREP_PICK_1
    mode: str = ACTION_MODE_STOP_AND_WAIT
    timeout_ms: int = 3000
    post_wait_ms: int = 0
    arrival_point_id: Optional[int] = None
    accel_limit_mmps2: int = 0
    beta_limit_ddegps2: int = 0
    wz_limit_ddegps: int = 0
    speed_limit_mmps: int = 0
    stable_time_ms: int = 0

    @classmethod
    def from_dict(cls, data: dict) -> "MechanicalAction":
        _reject_fields(data, REMOVED_ACTION_FIELDS, "action")
        action_value = data.get("action", PATH_ACT_PREP_PICK_1)
        if isinstance(action_value, str) and action_value.upper() in ACTION_CODES:
            action_value = ACTION_CODES[action_value.upper()]
        mode = str(data.get("mode", ACTION_MODE_STOP_AND_WAIT)).upper()
        if mode not in ACTION_MODE_NAMES:
            raise ValueError(f"action.mode={mode!r} 非法")

        limits = data.get("limits", {})
        if mode != ACTION_MODE_KINEMATIC and "limits" in data:
            raise ValueError(f"action {mode} 不允许配置 limits")
        return cls(
            action_seq=parse_int(data.get("action_seq", 0), "action.action_seq"),
            action=parse_int(action_value, "action.action"),
            mode=mode,
            timeout_ms=parse_int(data.get("timeout_ms", 3000), "action.timeout_ms"),
            post_wait_ms=parse_int(data.get("post_wait_ms", 0), "action.post_wait_ms"),
            arrival_point_id=parse_optional_int(
                data.get("arrival_point_id"), "action.arrival_point_id"
            ),
            accel_limit_mmps2=parse_int(
                data.get(
                    "accel_limit_mmps2",
                    limits.get("accel_limit_mmps2", 0),
                ),
                "action.limits.accel_limit_mmps2",
            ),
            beta_limit_ddegps2=parse_int(
                data.get("beta_limit_ddegps2", limits.get("beta_limit_ddegps2", 0)),
                "action.limits.beta_limit_ddegps2",
            ),
            wz_limit_ddegps=parse_int(
                data.get("wz_limit_ddegps", limits.get("wz_limit_ddegps", 0)),
                "action.limits.wz_limit_ddegps",
            ),
            speed_limit_mmps=parse_int(
                data.get("speed_limit_mmps", limits.get("speed_limit_mmps", 0)),
                "action.limits.speed_limit_mmps",
            ),
            stable_time_ms=parse_int(
                data.get("stable_time_ms", limits.get("stable_time_ms", 0)),
                "action.limits.stable_time_ms",
            ),
        )

    def to_config_dict(self) -> dict:
        result = {
            "action_seq": self.action_seq,
            "action": ACTIONS.get(self.action, self.action),
            "mode": self.mode,
            "timeout_ms": self.timeout_ms,
            "post_wait_ms": self.post_wait_ms,
        }
        if self.mode == ACTION_MODE_STOP_AND_WAIT:
            result["arrival_point_id"] = self.arrival_point_id
        elif self.mode == ACTION_MODE_KINEMATIC:
            result["limits"] = {
                "accel_limit_mmps2": self.accel_limit_mmps2,
                "beta_limit_ddegps2": self.beta_limit_ddegps2,
                "wz_limit_ddegps": self.wz_limit_ddegps,
                "speed_limit_mmps": self.speed_limit_mmps,
                "stable_time_ms": self.stable_time_ms,
            }
        return result


@dataclass
class ResolvedMechanicalAction:
    action_seq: int
    action: int
    mode: str
    arrival_id: int
    timeout_ms: int
    post_wait_ms: int
    check_start_s_mm: int
    accel_limit_mmps2: int
    beta_limit_ddegps2: int
    wz_limit_ddegps: int
    speed_limit_mmps: int
    stable_time_ms: int
    execution_hint: str = ""
    fallback_arrival_id: Optional[int] = None


@dataclass
class ArrivalDepartureLock:
    arrival_id: int
    departure_action_seq: int
    bound_action_seqs: List[int]


@dataclass
class GeometrySample:
    s_mm: float
    x_mm: float
    y_mm: float
    tangent_x: float = 1.0
    tangent_y: float = 0.0
    normal_x: float = 0.0
    normal_y: float = 1.0
    curvature_kappa_per_m: float = 0.0
    source_segment: int = 0
    source_point: Optional[int] = None


@dataclass
class GeometryResult:
    samples: List[GeometrySample]
    point_s_mm: Dict[int, float]


@dataclass
class TrajectoryNode:
    s_mm: float
    x_mm: float
    y_mm: float
    yaw_rad: float
    vx_mmps: float
    vy_mmps: float
    wz_radps: float
    arrival_id: int = 0xFF
    flags: int = 0
    speed_mmps: float = 0.0
    a_t_mmps2: float = 0.0
    a_n_mmps2: float = 0.0
    a_total_mmps2: float = 0.0
    beta_radps2: float = 0.0
    curvature_kappa_per_m: float = 0.0
    q_rad_per_mm: float = 0.0
    q_prime_rad_per_mm2: float = 0.0
    max_wheel_rpm: float = 0.0
    constraint_source: str = "global speed"
    source_point: Optional[int] = None


@dataclass
class PlanSummary:
    total_length_mm: float = 0.0
    formal_time_ms: int = 0
    mechanical_wait_time_ms: int = 0
    estimated_total_time_ms: int = 0
    max_speed_mmps: float = 0.0
    max_a_total_mmps2: float = 0.0
    max_a_n_mmps2: float = 0.0
    max_wz_radps: float = 0.0
    max_beta_radps2: float = 0.0
    max_wheel_rpm: float = 0.0
    max_wheel_rpm_s_mm: float = 0.0
    wheel_limited_length_mm: float = 0.0
    high_accel_length_mm: float = 0.0


@dataclass
class PlanResult:
    nodes: List[TrajectoryNode]
    actions: List[ResolvedMechanicalAction]
    summary: PlanSummary
    warnings: List[str] = dc_field(default_factory=list)
    departure_locks: List[ArrivalDepartureLock] = dc_field(default_factory=list)


@dataclass
class PathProject:
    traj_id: int = 0
    path_mode: str = PATH_MODE_FREE
    field: FieldConfig = dc_field(default_factory=FieldConfig)
    planner: PlannerConfig = dc_field(default_factory=PlannerConfig)
    start_check: StartCheckConfig = dc_field(default_factory=StartCheckConfig)
    arrival_check: ArrivalCheckConfig = dc_field(default_factory=ArrivalCheckConfig)
    fixed_sites: List[FixedSite] = dc_field(default_factory=default_fixed_sites)
    route_meta: dict = dc_field(default_factory=dict)
    start_region: StartRegion = dc_field(default_factory=StartRegion)
    overlay: OverlayConfig = dc_field(default_factory=OverlayConfig)
    controller_preview: ControllerPreview = dc_field(default_factory=ControllerPreview)
    vehicle_profile: VehicleProfile = dc_field(default_factory=VehicleProfile)
    mechanism_profile: MechanismProfile = dc_field(default_factory=MechanismProfile)
    collision_check_enabled: bool = False
    reachability_check_enabled: bool = False
    points: List[EditPoint] = dc_field(default_factory=list)
    actions: List[MechanicalAction] = dc_field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "PathProject":
        _reject_fields(data, REMOVED_TOP_LEVEL_FIELDS, "project")
        project_format = data.get("format")
        if project_format != PROJECT_FORMAT:
            raise ValueError(
                "工程格式不兼容，V3.5 不接受 V3.4 或更早 JSON；"
                f"当前 format={project_format!r}"
            )
        path_mode = str(data.get("path_mode", PATH_MODE_FREE)).upper()
        if path_mode not in PATH_MODES:
            raise ValueError(f"path_mode={path_mode!r} 非法")
        if "fixed_sites" in data:
            fixed_sites = [FixedSite.from_dict(item) for item in data["fixed_sites"]]
        else:
            fixed_sites = default_fixed_sites()
        project = cls(
            traj_id=parse_int(data.get("traj_id", 0), "traj_id"),
            path_mode=path_mode,
            field=FieldConfig.from_dict(data.get("field", {})),
            planner=PlannerConfig.from_dict(data.get("planner", {})),
            start_check=StartCheckConfig.from_dict(data.get("start_check", {})),
            arrival_check=ArrivalCheckConfig.from_dict(data.get("arrival_check", {})),
            fixed_sites=fixed_sites,
            route_meta=dict(data.get("route_meta", {})),
            start_region=StartRegion.from_dict(data.get("start_region", {})),
            overlay=OverlayConfig.from_dict(data.get("overlay", {})),
            controller_preview=ControllerPreview.from_dict(
                data.get("controller_preview", {})
            ),
            vehicle_profile=VehicleProfile.from_dict(data.get("vehicle_profile", {})),
            mechanism_profile=MechanismProfile.from_dict(
                data.get("mechanism_profile", {})
            ),
            collision_check_enabled=bool(data.get("collision_check_enabled", False)),
            reachability_check_enabled=bool(
                data.get("reachability_check_enabled", False)
            ),
            points=[EditPoint.from_dict(item) for item in data.get("points", [])],
            actions=[MechanicalAction.from_dict(item) for item in data.get("actions", [])],
        )
        return project

    def _point_to_config_dict(self, point: EditPoint) -> dict:
        base = {"point_id": point.point_id, "type": point.type}
        if point.type == POINT_TYPE_WAYPOINT:
            base.update(
                {
                    "site_id": SITE_ID_FREE,
                    "x_mm": point.x_mm,
                    "y_mm": point.y_mm,
                    "yaw_ddeg": YAW_UNSPECIFIED_DDEG,
                    "max_speed_mmps": point.max_speed_mmps,
                    "corner_trim_mm": point.corner_trim_mm,
                    "exact_pass": point.exact_pass,
                }
            )
            return base
        if self.path_mode == PATH_MODE_FIXED_8:
            base["site_id"] = point.site_id
            if point.site_id == SITE_ID_FREE:
                base.update(
                    {
                        "x_mm": point.x_mm,
                        "y_mm": point.y_mm,
                        "yaw_ddeg": point.yaw_ddeg,
                    }
                )
                return base
            # Fixed x/y/yaw live only in project.json/fixed_sites.  Even when
            # yaw is 0xFFFF, do not copy an override into every path point.
            return base
        base.update(
            {
                "site_id": SITE_ID_FREE,
                "x_mm": point.x_mm,
                "y_mm": point.y_mm,
                "yaw_ddeg": point.yaw_ddeg,
            }
        )
        return base

    def to_config_dict(self) -> dict:
        """Serialize only editable configuration; planned nodes are never persisted."""
        return {
            "format": PROJECT_FORMAT,
            "traj_id": self.traj_id,
            "path_mode": self.path_mode,
            "field": asdict(self.field),
            "planner": asdict(self.planner),
            "start_check": asdict(self.start_check),
            "arrival_check": asdict(self.arrival_check),
            "fixed_sites": [asdict(site) for site in self.fixed_sites],
            "route_meta": self.route_meta,
            "start_region": asdict(self.start_region),
            "overlay": asdict(self.overlay),
            "controller_preview": asdict(self.controller_preview),
            "vehicle_profile": asdict(self.vehicle_profile),
            "mechanism_profile": asdict(self.mechanism_profile),
            "collision_check_enabled": self.collision_check_enabled,
            "reachability_check_enabled": self.reachability_check_enabled,
            "points": [self._point_to_config_dict(point) for point in self.points],
            "actions": [action.to_config_dict() for action in self.actions],
        }

    def to_dict(self) -> dict:
        return self.to_config_dict()


@dataclass
class TrajectoryHeaderV35:
    traj_id: int
    flags: int
    field_width_mm: int
    field_height_mm: int
    nominal_spacing_mm: int
    node_count: int
    action_count: int
    arrival_count: int
    file_crc32: int
    node_offset: int
    action_offset: int
    total_length_mm: int
    planned_motion_time_ms: int
    start_pos_tolerance_mm: int
    start_yaw_tolerance_ddeg: int
    start_stable_time_ms: int
    arrival_pos_tolerance_mm: int
    arrival_yaw_tolerance_ddeg: int
    arrival_speed_tolerance_mmps: int
    arrival_wz_tolerance_ddegps: int
    arrival_stable_time_ms: int


@dataclass
class ParsedTrajectoryV35:
    header: TrajectoryHeaderV35
    nodes: List[TrajectoryNode]
    actions: List[ResolvedMechanicalAction]


def _fixed_site_by_id(project: PathProject) -> Dict[int, FixedSite]:
    errors = validate_fixed_sites(project.fixed_sites)
    if errors:
        raise ValueError("\n".join(errors))
    return {site.site_id: site for site in project.fixed_sites}


def resolve_edit_points(project: PathProject) -> List[EditPoint]:
    """Return resolved point copies for planning without mutating project.points."""
    points: List[EditPoint] = []
    fixed_sites = _fixed_site_by_id(project) if project.path_mode == PATH_MODE_FIXED_8 else {}
    used_fixed_arrivals: set[int] = set()

    for row, point in enumerate(project.points):
        resolved = replace(point)
        resolved.type = resolved.type.upper()
        if resolved.type not in POINT_TYPES:
            raise ValueError(f"point[{row}] type={resolved.type!r} 非法")
        if row == 0 and resolved.type != POINT_TYPE_START:
            raise ValueError("point[0] 必须为 START")
        if row != 0 and resolved.type == POINT_TYPE_START:
            raise ValueError("START 只能出现在 point[0]")
        if resolved.type == POINT_TYPE_WAYPOINT:
            if resolved.site_id != SITE_ID_FREE:
                raise ValueError(f"point_id={resolved.point_id} WAYPOINT site_id 必须为 255")
            resolved.yaw_ddeg = YAW_UNSPECIFIED_DDEG
            points.append(resolved)
            continue

        if project.path_mode == PATH_MODE_FREE:
            if resolved.site_id != SITE_ID_FREE:
                raise ValueError(
                    f"FREE point_id={resolved.point_id} 的 START/ARRIVAL site_id 必须为 255"
                )
            points.append(resolved)
            continue

        if resolved.type == POINT_TYPE_START and resolved.site_id != 0:
            raise ValueError(f"FIXED_8 START point_id={resolved.point_id} 必须引用 site_id=0")
        if resolved.type == POINT_TYPE_ARRIVAL:
            if resolved.site_id == SITE_ID_FREE:
                raise ValueError(
                    f"FIXED_8 ARRIVAL point_id={resolved.point_id} 未选择固定点 site_id"
                )
            if resolved.site_id == 0:
                raise ValueError(f"FIXED_8 ARRIVAL point_id={resolved.point_id} 不能引用 START site")
            if resolved.site_id in used_fixed_arrivals:
                raise ValueError(f"FIXED_8 ARRIVAL site_id={resolved.site_id} 在当前路径中重复")
            used_fixed_arrivals.add(resolved.site_id)
        site = fixed_sites.get(resolved.site_id)
        if site is None:
            raise ValueError(
                f"FIXED_8 point_id={resolved.point_id} 引用了不存在的 site_id={resolved.site_id}"
            )
        resolved.x_mm = site.x_mm
        resolved.y_mm = site.y_mm
        # 0xFFFF is a complete fixed-site value meaning unconstrained yaw, not
        # a request for a per-point override.
        resolved.yaw_ddeg = site.yaw_ddeg
        points.append(resolved)
    return points


def make_default_project() -> PathProject:
    """Create a new project with default parameters and no path content."""
    return PathProject()
