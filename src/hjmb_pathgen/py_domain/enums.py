"""V4.0 protocol enum values."""

from __future__ import annotations

from enum import IntEnum, IntFlag, StrEnum


class RouteFamily(IntEnum):
    MANUAL = 0x00
    PICK_1_TO_3 = 0x01
    PICK_3_TO_1 = 0x02


class FinishMode(IntEnum):
    AT_FINAL_DROP = 0x00


class HeaderFlag(IntFlag):
    SPATIAL_TRAJECTORY = 0x0001
    WORLD_VELOCITY = 0x0002
    FIXED_DIRECT_START = 0x0004
    ARRIVAL_ALWAYS_STOP = 0x0008
    ACTION_STATUS_REQUIRED = 0x0010
    SEGMENT_TABLE_PRESENT = 0x0020
    FINISH_POLICY_PRESENT = 0x0040
    CASE_METADATA_PRESENT = 0x0080
    RUNTIME_FIELD_SCALE = 0x0100
    MANUAL_OVERRIDE = 0x0200


class NodeFlag(IntFlag):
    START = 0x01
    ARRIVAL = 0x02
    EXACT_PASS = 0x04
    FINISH_ARM = 0x08
    # Reserved in formal V4.0 output; retained to reject legacy/development BINs.
    SAFE_END = 0x10


class SegmentFlag(IntFlag):
    NORMAL = 0x01
    # Reserved in formal V4.0 output; retained to reject legacy/development BINs.
    FINISH_CLEAR = 0x02
    LIBRARY_REUSED = 0x04
    MANUAL_OVERRIDE = 0x08
    OPTIMIZED = 0x10


class ActionMode(IntEnum):
    STOP_AND_WAIT = 0x00
    ASYNC = 0x01
    KINEMATIC = 0x02


class ActionCode(IntEnum):
    NONE = 0x00
    PREP_PICK_1 = 0x11
    PREP_PICK_2L = 0x12
    PREP_PICK_2R = 0x13
    PREP_PICK_3 = 0x14
    PICK = 0x20
    DROP_1 = 0x31
    DROP_2 = 0x32
    DROP_3 = 0x33
    DROP_12 = 0x34
    DROP_23 = 0x35
    STORE = 0x40
    PREP_STORE_1 = 0x41
    PREP_STORE_2 = 0x42
    PREP_STORE_3 = 0x43


class UnloadMask(StrEnum):
    BIN_1 = "BIN_1"
    BIN_2 = "BIN_2"
    BIN_3 = "BIN_3"
    BIN_12 = "BIN_12"
    BIN_23 = "BIN_23"


class YawPolicy(StrEnum):
    CW_ONLY = "CW_ONLY"
    CCW_ONLY = "CCW_ONLY"
    SHORTEST = "SHORTEST"


class StorageMode(StrEnum):
    REFERENCED = "REFERENCED"
    EMBEDDED = "EMBEDDED"


class GenerationMode(StrEnum):
    MANUAL = "MANUAL"
    SEMI_AUTO = "SEMI_AUTO"
    FULL_AUTO = "FULL_AUTO"


class ManualPathPointType(StrEnum):
    START = "START"
    WAYPOINT = "WAYPOINT"
    ARRIVAL = "ARRIVAL"


class LegState(StrEnum):
    MISSING = "MISSING"
    PREVIEW_VALID = "PREVIEW_VALID"
    VALID = "VALID"
    STALE = "STALE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    CANCELLED_WITH_BEST = "CANCELLED_WITH_BEST"
    TIMEOUT = "TIMEOUT"
    TIMEOUT_WITH_BEST = "TIMEOUT_WITH_BEST"
    LOCKED = "LOCKED"
    APPROVED = "APPROVED"
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
