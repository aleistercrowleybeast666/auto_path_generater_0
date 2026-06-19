"""V4.0 protocol constants."""

from __future__ import annotations

from .enums import HeaderFlag

MAGIC = b"HJMB"
BIN_VERSION = 40
JSON_VERSION = "V40"

HEADER_SIZE = 104
NODE_SIZE = 16
SEGMENT_SIZE = 24
ACTION_SIZE = 22

NOMINAL_FIELD_LENGTH_MM = 4000
NOMINAL_FIELD_WIDTH_MM = 2000

MIN_TRAJ_ID = 0
MAX_TRAJ_ID = 359
MIN_BEAN_CODE = 0
MAX_BEAN_CODE = 5
MIN_DROP_CODE = 0
MAX_DROP_CODE = 59
MIN_NODE_COUNT = 2
MAX_NODE_COUNT = 2500
MIN_SEGMENT_COUNT = 1
MAX_SEGMENT_COUNT = 64
MIN_ACTION_COUNT = 0
MAX_ACTION_COUNT = 64
MIN_ARRIVAL_COUNT = 1
MAX_ARRIVAL_COUNT = 32
MAX_TOTAL_LENGTH_MM = 65535

PROJECT_FORMAT = "HJMB_PATH_PROJECT_JSON_V40"
ROUTE_CASE_TABLE_FORMAT = "HJMB_ROUTE_CASE_TABLE_JSON_V40"
LEG_LIBRARY_FORMAT = "HJMB_LEG_LIBRARY_JSON_V40"
CASE_FORMAT = "HJMB_ROUTE_CASE_JSON_V40"
SITE_POSE_PRESET_FORMAT = "HJMB_SITE_POSE_PRESET_JSON_V40"

DIR_CASES = "cases"
DIR_BIN = "bin"
DIR_PORTABLE = "portable"
DIR_TASK_COMPILED = "task_compiled"
DIR_MANUAL_FREE = "manual_free"
DIR_FINAL = "final"
DIR_REPORTS = "reports"
DIR_CACHE = "cache"
DIR_PRESETS = "presets"
DIR_DRAFTS = "drafts"

CASE_JSON_PATTERN = "P{traj_id:04d}.json"
CASE_BIN_PATTERN = "P{traj_id:04d}.BIN"
PORTABLE_CASE_PATTERN = "P{traj_id:04d}.portable.json"

REQUIRED_HEADER_FLAGS = (
    HeaderFlag.SPATIAL_TRAJECTORY
    | HeaderFlag.WORLD_VELOCITY
    | HeaderFlag.FIXED_DIRECT_START
    | HeaderFlag.ARRIVAL_ALWAYS_STOP
    | HeaderFlag.ACTION_STATUS_REQUIRED
    | HeaderFlag.SEGMENT_TABLE_PRESENT
    | HeaderFlag.FINISH_POLICY_PRESENT
    | HeaderFlag.CASE_METADATA_PRESENT
    | HeaderFlag.RUNTIME_FIELD_SCALE
)
VALID_HEADER_FLAGS = REQUIRED_HEADER_FLAGS | HeaderFlag.MANUAL_OVERRIDE

REQUIRED_SITE_KEYS = (
    "P_START",
    "P_PICK_1",
    "P_PICK_2L",
    "P_PICK_2R",
    "P_PICK_3",
    "F_DROP_4",
    "F_DROP_5",
    "F_DROP_6",
    "F_DROP_7",
    "F_DROP_8",
)
PICKUP_SITE_KEYS = REQUIRED_SITE_KEYS[:5]
DROP_SITE_KEYS = REQUIRED_SITE_KEYS[5:]

REQUIRED_UNLOAD_PROFILE_KEYS = (
    "BIN_1",
    "BIN_2",
    "BIN_3",
    "BIN_12",
    "BIN_23",
)

FUNCTIONAL_HASH_KEYS = (
    "site_config_hash",
    "vehicle_config_hash",
    "collision_config_hash",
    "obstacle_geometry_hash",
    "dynamics_config_hash",
    "planner_config_hash",
    "action_profile_hash",
    "topology_config_hash",
)

REMOVED_JSON_FIELDS = frozenset(
    {
        "cut_in",
        "stop_required",
        "gate_id",
        "unlock_gate_id",
        "flags",
        "trigger",
        "trigger_s_mm",
        "expire_s_mm",
        "window_start",
        "window_end",
        "arm_s_mm",
        "disarm_s_mm",
        "min_wait_ms",
    }
)
