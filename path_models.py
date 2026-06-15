# -*- coding: utf-8 -*-
"""HJMB V3.3 project, trajectory, and protocol-neutral data models."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field as dc_field
from typing import Dict, List, Optional, Tuple

PROJECT_FORMAT = "HJMB_PATH_EDITOR_JSON_V33"

POINT_TYPE_CUT_IN = "CUT_IN"
POINT_TYPE_WAYPOINT = "WAYPOINT"
POINT_TYPE_ARRIVAL = "ARRIVAL"
POINT_TYPES = (POINT_TYPE_CUT_IN, POINT_TYPE_WAYPOINT, POINT_TYPE_ARRIVAL)

YAW_UNSPECIFIED_DDEG = 0xFF

ACTION_FLAG_LOCKED = 0x01
ACTION_FLAG_HOLD_PATH = 0x02
ACTION_FLAG_REQUIRED_AT_END = 0x04
VALID_ACTION_FLAGS_MASK = (
    ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH | ACTION_FLAG_REQUIRED_AT_END
)

ACTION_GATE_ACCEL = 0xFE
ACTION_GATE_UNCONDITIONAL = 0xFF

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

TRAJ_FLAG_GATE = 0x01
TRAJ_FLAG_STOP = 0x02
TRAJ_FLAG_SCAN = 0x04
TRAJ_FLAG_SLOW_ZONE = 0x08
TRAJ_FLAG_CUT_IN = 0x10
TRAJ_FLAG_ARRIVAL = 0x20
TRAJ_FLAG_WAYPOINT = 0x40
TRAJ_FLAG_END = 0x80

MAX_EDIT_POINTS = 100
MAX_NODES = 2500
MAX_ACTIONS = 32
MAX_GATES = 32
MAX_TRAJ_ID = 359


def parse_int(value, field_name: str = "value") -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    try:
        return int(str(value).strip(), 0)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是十进制或 0x 前缀整数，当前为 {value!r}") from exc


def parse_float(value, field_name: str = "value") -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    try:
        return float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是数值，当前为 {value!r}") from exc


def hex8(value: int) -> str:
    return f"0x{value & 0xFF:02X}"


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
    max_ref_lead_mm: int = 50
    max_iterations: int = 40
    speed_convergence_mmps: float = 0.5

    @classmethod
    def from_dict(cls, data: dict) -> "PlannerConfig":
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
class CutInConfig:
    capture_radius_mm: int = 100
    target_speed_mmps: int = 800
    approach_max_speed_mmps: int = 1000
    straight_length_mm: int = 500
    yaw_tolerance_ddeg: int = 100
    tangent_tolerance_ddeg: int = 150
    align_yaw: bool = True
    allow_first_segment_capture: bool = True

    @classmethod
    def from_dict(cls, data: dict) -> "CutInConfig":
        return cls(
            capture_radius_mm=parse_int(
                data.get("capture_radius_mm", 100), "cut_in.capture_radius_mm"
            ),
            target_speed_mmps=parse_int(
                data.get("target_speed_mmps", 800), "cut_in.target_speed_mmps"
            ),
            approach_max_speed_mmps=parse_int(
                data.get("approach_max_speed_mmps", 1000),
                "cut_in.approach_max_speed_mmps",
            ),
            straight_length_mm=parse_int(
                data.get("straight_length_mm", 500), "cut_in.straight_length_mm"
            ),
            yaw_tolerance_ddeg=parse_int(
                data.get("yaw_tolerance_ddeg", 100), "cut_in.yaw_tolerance_ddeg"
            ),
            tangent_tolerance_ddeg=parse_int(
                data.get("tangent_tolerance_ddeg", 150),
                "cut_in.tangent_tolerance_ddeg",
            ),
            align_yaw=bool(data.get("align_yaw", True)),
            allow_first_segment_capture=bool(
                data.get("allow_first_segment_capture", True)
            ),
        )


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
            mecanum_convention=str(
                data.get("mecanum_convention", "X_FL_FR_RL_RR")
            ),
            geometry_note=str(
                data.get("geometry_note", "示例初值，导出前请按实车核对")
            ),
        )


@dataclass
class PreviewInitialPose:
    enabled: bool = False
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_ddeg: int = 0
    initial_speed_mmps: float = 0.0

    @classmethod
    def from_dict(cls, data: dict) -> "PreviewInitialPose":
        return cls(
            enabled=bool(data.get("enabled", False)),
            x_mm=parse_float(data.get("x_mm", 0), "preview_initial_pose.x_mm"),
            y_mm=parse_float(data.get("y_mm", 0), "preview_initial_pose.y_mm"),
            yaw_ddeg=parse_int(
                data.get("yaw_ddeg", 0), "preview_initial_pose.yaw_ddeg"
            ),
            initial_speed_mmps=parse_float(
                data.get("initial_speed_mmps", 0),
                "preview_initial_pose.initial_speed_mmps",
            ),
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
    show_cut_in_capture: bool = True
    show_cut_in_preview: bool = True
    speed_threshold_mmps: int = 1500
    accel_threshold_mmps2: int = 800
    beta_threshold_radps2: float = 1.5

    @classmethod
    def from_dict(cls, data: dict) -> "OverlayConfig":
        return cls(
            selected_analysis_mode=str(data.get("selected_analysis_mode", "normal")),
            scale_mode=str(data.get("scale_mode", "planner")),
            show_cut_in_capture=bool(data.get("show_cut_in_capture", True)),
            show_cut_in_preview=bool(data.get("show_cut_in_preview", True)),
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
        legacy_names = {
            "STORE_1": "DROP_1",
            "STORE_2": "DROP_2",
            "STORE_3": "DROP_3",
            "DUMP": "STORE",
        }
        for name, value in data.get("action_duration_ms", {}).items():
            target_name = legacy_names.get(str(name), str(name))
            if target_name in durations:
                durations[target_name] = parse_int(
                    value, f"mechanism_profile.action_duration_ms.{name}"
                )
        return cls(
            action_duration_ms=durations,
            drop_safety_margin_ms=parse_int(
                data.get(
                    "drop_safety_margin_ms",
                    data.get("dump_safety_margin_ms", 300),
                ),
                "mechanism_profile.drop_safety_margin_ms",
            ),
        )


@dataclass
class EditPoint:
    point_id: int = 0
    type: str = POINT_TYPE_WAYPOINT
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_ddeg: int = YAW_UNSPECIFIED_DDEG
    max_speed_mmps: int = 0
    corner_trim_mm: float = 200.0
    exact_pass: bool = False
    stop_required: bool = False
    gate_id: int = 0xFF
    marker_id: int = 0xFF
    scan: bool = False
    is_end: bool = False

    @classmethod
    def from_dict(cls, data: dict) -> "EditPoint":
        if "yaw_mode" in data:
            raise ValueError("V3.3 已删除 point.yaw_mode")
        point_type = str(data.get("type", POINT_TYPE_WAYPOINT)).upper()
        default_yaw = (
            YAW_UNSPECIFIED_DDEG
            if point_type == POINT_TYPE_WAYPOINT
            else 0
        )
        return cls(
            point_id=parse_int(data.get("point_id", 0), "point.point_id"),
            type=point_type,
            x_mm=parse_float(data.get("x_mm", 0), "point.x_mm"),
            y_mm=parse_float(data.get("y_mm", 0), "point.y_mm"),
            yaw_ddeg=parse_int(
                data.get("yaw_ddeg", default_yaw), "point.yaw_ddeg"
            ),
            max_speed_mmps=parse_int(
                data.get("max_speed_mmps", 0), "point.max_speed_mmps"
            ),
            corner_trim_mm=parse_float(
                data.get("corner_trim_mm", 200), "point.corner_trim_mm"
            ),
            exact_pass=bool(data.get("exact_pass", False)),
            stop_required=bool(data.get("stop_required", False)),
            gate_id=parse_int(data.get("gate_id", 0xFF), "point.gate_id"),
            marker_id=parse_int(data.get("marker_id", 0xFF), "point.marker_id"),
            scan=bool(data.get("scan", False)),
            is_end=bool(data.get("is_end", False)),
        )


@dataclass
class MechanicalAction:
    action_seq: int = 0
    action: int = PATH_ACT_PREP_PICK_1
    unlock_gate_id: int = ACTION_GATE_UNCONDITIONAL
    flags: int = 0
    timeout_ms: int = 0
    arm_s_mm: int = 0
    disarm_s_mm: int = 0xFFFF
    accel_limit_mmps2: int = 0
    beta_limit_ddegps2: int = 0
    speed_limit_mmps: int = 0
    stable_time_ms: int = 0

    @classmethod
    def from_dict(cls, data: dict) -> "MechanicalAction":
        action_value = data.get("action", PATH_ACT_PREP_PICK_1)
        if isinstance(action_value, str) and action_value.upper() in ACTION_CODES:
            action_value = ACTION_CODES[action_value.upper()]
        return cls(
            action_seq=parse_int(data.get("action_seq", 0), "action.action_seq"),
            action=parse_int(action_value, "action.action"),
            unlock_gate_id=parse_int(
                data.get("unlock_gate_id", ACTION_GATE_UNCONDITIONAL),
                "action.unlock_gate_id",
            ),
            flags=parse_int(data.get("flags", 0), "action.flags"),
            timeout_ms=parse_int(data.get("timeout_ms", 0), "action.timeout_ms"),
            arm_s_mm=parse_int(data.get("arm_s_mm", 0), "action.arm_s_mm"),
            disarm_s_mm=parse_int(
                data.get("disarm_s_mm", 0xFFFF), "action.disarm_s_mm"
            ),
            accel_limit_mmps2=parse_int(
                data.get("accel_limit_mmps2", 0), "action.accel_limit_mmps2"
            ),
            beta_limit_ddegps2=parse_int(
                data.get("beta_limit_ddegps2", 0), "action.beta_limit_ddegps2"
            ),
            speed_limit_mmps=parse_int(
                data.get("speed_limit_mmps", 0), "action.speed_limit_mmps"
            ),
            stable_time_ms=parse_int(
                data.get("stable_time_ms", 0), "action.stable_time_ms"
            ),
        )


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
    gate_id: int = 0xFF
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
class CutInPreviewResult:
    enabled: bool = False
    reachable: bool = False
    distance_mm: float = 0.0
    time_ms: int = 0
    peak_speed_mmps: float = 0.0
    warning: str = ""


@dataclass
class PlanSummary:
    total_length_mm: float = 0.0
    formal_time_ms: int = 0
    cut_in_preview_time_ms: int = 0
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
    actions: List[MechanicalAction]
    summary: PlanSummary
    cut_in_preview: CutInPreviewResult
    warnings: List[str] = dc_field(default_factory=list)


@dataclass
class PathProject:
    traj_id: int = 0
    field: FieldConfig = dc_field(default_factory=FieldConfig)
    planner: PlannerConfig = dc_field(default_factory=PlannerConfig)
    cut_in: CutInConfig = dc_field(default_factory=CutInConfig)
    start_region: StartRegion = dc_field(default_factory=StartRegion)
    preview_initial_pose: PreviewInitialPose = dc_field(default_factory=PreviewInitialPose)
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
        project_format = data.get("format")
        if project_format != PROJECT_FORMAT:
            raise ValueError(
                "工程格式不兼容，请使用旧版编辑器转换或按 V3.3 重新绘制；"
                f"当前 format={project_format!r}"
            )
        return cls(
            traj_id=parse_int(data.get("traj_id", 0), "traj_id"),
            field=FieldConfig.from_dict(data.get("field", {})),
            planner=PlannerConfig.from_dict(data.get("planner", {})),
            cut_in=CutInConfig.from_dict(data.get("cut_in", {})),
            start_region=StartRegion.from_dict(data.get("start_region", {})),
            preview_initial_pose=PreviewInitialPose.from_dict(
                data.get("preview_initial_pose", {})
            ),
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
            actions=[
                MechanicalAction.from_dict(item) for item in data.get("actions", [])
            ],
        )

    def to_config_dict(self) -> dict:
        """Serialize only editable configuration; planned nodes are never persisted."""
        return {
            "format": PROJECT_FORMAT,
            "traj_id": self.traj_id,
            "field": asdict(self.field),
            "planner": asdict(self.planner),
            "cut_in": asdict(self.cut_in),
            "start_region": asdict(self.start_region),
            "preview_initial_pose": asdict(self.preview_initial_pose),
            "overlay": asdict(self.overlay),
            "controller_preview": asdict(self.controller_preview),
            "vehicle_profile": asdict(self.vehicle_profile),
            "mechanism_profile": asdict(self.mechanism_profile),
            "collision_check_enabled": self.collision_check_enabled,
            "reachability_check_enabled": self.reachability_check_enabled,
            "points": [asdict(point) for point in self.points],
            "actions": [asdict(action) for action in self.actions],
        }

    def to_dict(self) -> dict:
        return self.to_config_dict()


@dataclass
class TrajectoryHeaderV33:
    traj_id: int
    flags: int
    field_width_mm: int
    field_height_mm: int
    nominal_spacing_mm: int
    node_count: int
    action_count: int
    gate_count: int
    file_crc32: int
    node_offset: int
    action_offset: int
    total_length_mm: int
    planned_time_ms: int
    cut_in_capture_radius_mm: int
    cut_in_speed_mmps: int
    approach_max_speed_mmps: int
    cut_in_straight_length_mm: int
    cut_in_yaw_tolerance_ddeg: int
    cut_in_tangent_tolerance_ddeg: int
    approach_flags: int


@dataclass
class ParsedTrajectoryV33:
    header: TrajectoryHeaderV33
    nodes: List[TrajectoryNode]
    actions: List[MechanicalAction]


def make_default_project() -> PathProject:
    """Create a new project with default parameters and no path content."""
    return PathProject()
