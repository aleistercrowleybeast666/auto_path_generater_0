"""V4.0 compiled trajectory models."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from .enums import ActionCode, ActionMode, FinishMode, HeaderFlag, NodeFlag, RouteFamily, SegmentFlag
from .errors import V40ValidationError, expect_int_range
from .protocol import (
    ACTION_SIZE,
    BIN_VERSION,
    HEADER_SIZE,
    MAGIC,
    MAX_ACTION_COUNT,
    MAX_ARRIVAL_COUNT,
    MAX_BEAN_CODE,
    MAX_DROP_CODE,
    MAX_NODE_COUNT,
    MAX_SEGMENT_COUNT,
    MAX_TOTAL_LENGTH_MM,
    MAX_TRAJ_ID,
    MIN_ACTION_COUNT,
    MIN_ARRIVAL_COUNT,
    MIN_BEAN_CODE,
    MIN_DROP_CODE,
    MIN_NODE_COUNT,
    MIN_SEGMENT_COUNT,
    MIN_TRAJ_ID,
    NODE_SIZE,
    NOMINAL_FIELD_LENGTH_MM,
    NOMINAL_FIELD_WIDTH_MM,
    REQUIRED_HEADER_FLAGS,
    SEGMENT_SIZE,
    VALID_HEADER_FLAGS,
)


def _u8(value: int, path: str) -> int:
    return expect_int_range(value, 0, 0xFF, "V40 BIN", path)


def _i8(value: int, path: str) -> int:
    return expect_int_range(value, -0x80, 0x7F, "V40 BIN", path)


def _u16(value: int, path: str) -> int:
    return expect_int_range(value, 0, 0xFFFF, "V40 BIN", path)


def _i16(value: int, path: str) -> int:
    return expect_int_range(value, -0x8000, 0x7FFF, "V40 BIN", path)


def _u32(value: int, path: str) -> int:
    return expect_int_range(value, 0, 0xFFFFFFFF, "V40 BIN", path)


@dataclass(frozen=True)
class HeaderV40:
    magic: bytes = MAGIC
    version: int = BIN_VERSION
    header_size: int = HEADER_SIZE
    node_size: int = NODE_SIZE
    segment_size: int = SEGMENT_SIZE
    action_size: int = ACTION_SIZE
    route_family: int = int(RouteFamily.MANUAL)
    finish_mode: int = int(FinishMode.AT_FINAL_DROP)
    reserved_u8: int = 0
    traj_id: int = 0
    flags: int = int(REQUIRED_HEADER_FLAGS)
    bean_code: int = 0
    drop_code: int = 0
    field_width_mm: int = NOMINAL_FIELD_LENGTH_MM
    field_height_mm: int = NOMINAL_FIELD_WIDTH_MM
    nominal_spacing_mm: int = 25
    node_count: int = 0
    segment_count: int = 0
    action_count: int = 0
    arrival_count: int = 0
    reserved0: int = 0
    file_crc32: int = 0
    node_offset: int = HEADER_SIZE
    segment_offset: int = HEADER_SIZE
    action_offset: int = HEADER_SIZE
    total_length_mm: int = 0
    planned_motion_time_ms: int = 0
    planned_total_estimate_ms: int = 0
    source_case_hash32: int = 0
    source_project_hash32: int = 0
    start_pos_tolerance_mm: int = 20
    start_yaw_tolerance_ddeg: int = 50
    start_stable_time_ms: int = 100
    arrival_pos_tolerance_mm: int = 20
    arrival_yaw_tolerance_ddeg: int = 50
    arrival_speed_tolerance_mmps: int = 10
    arrival_wz_tolerance_ddegps: int = 10
    arrival_stable_time_ms: int = 100
    finish_axis: int = 0
    finish_direction: int = 0
    finish_line_mm: int = 0
    finish_envelope_margin_mm: int = 0
    finish_stable_time_ms: int = 0
    finish_brake_accel_mmps2: int = 0
    finish_max_runout_mm: int = 0
    finish_hard_timeout_ms: int = 0
    finish_signal_flags: int = 0
    reserved1: int = 0

    def with_layout_counts(
        self,
        node_count: int,
        segment_count: int,
        action_count: int,
        arrival_count: int,
        *,
        total_length_mm: int,
    ) -> "HeaderV40":
        segment_offset = HEADER_SIZE + node_count * NODE_SIZE
        action_offset = segment_offset + segment_count * SEGMENT_SIZE
        return replace(
            self,
            node_count=node_count,
            segment_count=segment_count,
            action_count=action_count,
            arrival_count=arrival_count,
            node_offset=HEADER_SIZE,
            segment_offset=segment_offset,
            action_offset=action_offset,
            total_length_mm=total_length_mm,
        )

    @property
    def file_size(self) -> int:
        return (
            HEADER_SIZE
            + self.node_count * NODE_SIZE
            + self.segment_count * SEGMENT_SIZE
            + self.action_count * ACTION_SIZE
        )

    def to_tuple(self, *, zero_crc: bool = False) -> tuple:
        self.validate()
        return (
            self.magic,
            self.version,
            self.header_size,
            self.node_size,
            self.segment_size,
            self.action_size,
            self.route_family,
            self.finish_mode,
            self.reserved_u8,
            self.traj_id,
            self.flags,
            self.bean_code,
            self.drop_code,
            self.field_width_mm,
            self.field_height_mm,
            self.nominal_spacing_mm,
            self.node_count,
            self.segment_count,
            self.action_count,
            self.arrival_count,
            self.reserved0,
            0 if zero_crc else self.file_crc32,
            self.node_offset,
            self.segment_offset,
            self.action_offset,
            self.total_length_mm,
            self.planned_motion_time_ms,
            self.planned_total_estimate_ms,
            self.source_case_hash32,
            self.source_project_hash32,
            self.start_pos_tolerance_mm,
            self.start_yaw_tolerance_ddeg,
            self.start_stable_time_ms,
            self.arrival_pos_tolerance_mm,
            self.arrival_yaw_tolerance_ddeg,
            self.arrival_speed_tolerance_mmps,
            self.arrival_wz_tolerance_ddegps,
            self.arrival_stable_time_ms,
            self.finish_axis,
            self.finish_direction,
            self.finish_line_mm,
            self.finish_envelope_margin_mm,
            self.finish_stable_time_ms,
            self.finish_brake_accel_mmps2,
            self.finish_max_runout_mm,
            self.finish_hard_timeout_ms,
            self.finish_signal_flags,
            self.reserved1,
        )

    @classmethod
    def from_tuple(cls, values: tuple) -> "HeaderV40":
        header = cls(*values)
        header.validate()
        return header

    def validate(self) -> None:
        if self.magic != MAGIC:
            raise V40ValidationError("V40 Header", "magic", "wrong magic", actual=self.magic, expected=MAGIC)
        if self.version != BIN_VERSION:
            raise V40ValidationError(
                "V40 Header",
                "version",
                "unsupported BIN version",
                actual=self.version,
                expected=BIN_VERSION,
            )
        for field_name, expected in (
            ("header_size", HEADER_SIZE),
            ("node_size", NODE_SIZE),
            ("segment_size", SEGMENT_SIZE),
            ("action_size", ACTION_SIZE),
            ("field_width_mm", NOMINAL_FIELD_LENGTH_MM),
            ("field_height_mm", NOMINAL_FIELD_WIDTH_MM),
        ):
            actual = getattr(self, field_name)
            if actual != expected:
                raise V40ValidationError("V40 Header", field_name, "unexpected value", actual=actual, expected=expected)
        if self.reserved_u8 != 0 or self.reserved0 != 0 or self.reserved1 != 0:
            raise V40ValidationError(
                "V40 Header",
                "reserved",
                "reserved fields must be zero",
                actual=(self.reserved_u8, self.reserved0, self.reserved1),
                expected=(0, 0, 0),
            )
        _u8(self.route_family, "route_family")
        if self.route_family not in [int(item) for item in RouteFamily]:
            raise V40ValidationError("V40 Header", "route_family", "unknown route family", actual=self.route_family)
        _u8(self.finish_mode, "finish_mode")
        if self.finish_mode != int(FinishMode.AT_FINAL_DROP):
            raise V40ValidationError("V40 Header", "finish_mode", "formal V4.0 supports only AT_FINAL_DROP", actual=self.finish_mode)
        expect_int_range(self.traj_id, MIN_TRAJ_ID, MAX_TRAJ_ID, "V40 Header", "traj_id")
        expect_int_range(self.bean_code, MIN_BEAN_CODE, MAX_BEAN_CODE, "V40 Header", "bean_code")
        expect_int_range(self.drop_code, MIN_DROP_CODE, MAX_DROP_CODE, "V40 Header", "drop_code")
        expect_int_range(self.node_count, MIN_NODE_COUNT, MAX_NODE_COUNT, "V40 Header", "node_count")
        expect_int_range(self.segment_count, MIN_SEGMENT_COUNT, MAX_SEGMENT_COUNT, "V40 Header", "segment_count")
        expect_int_range(self.action_count, MIN_ACTION_COUNT, MAX_ACTION_COUNT, "V40 Header", "action_count")
        expect_int_range(self.arrival_count, MIN_ARRIVAL_COUNT, MAX_ARRIVAL_COUNT, "V40 Header", "arrival_count")
        expect_int_range(self.total_length_mm, 0, MAX_TOTAL_LENGTH_MM, "V40 Header", "total_length_mm")
        if self.flags & ~int(VALID_HEADER_FLAGS):
            raise V40ValidationError("V40 Header", "flags", "unknown flag bits", actual=self.flags)
        if (self.flags & int(REQUIRED_HEADER_FLAGS)) != int(REQUIRED_HEADER_FLAGS):
            raise V40ValidationError(
                "V40 Header",
                "flags",
                "required flags are missing",
                actual=self.flags,
                expected=int(REQUIRED_HEADER_FLAGS),
            )
        for name in (
            "file_crc32",
            "node_offset",
            "segment_offset",
            "action_offset",
            "planned_motion_time_ms",
            "planned_total_estimate_ms",
            "source_case_hash32",
            "source_project_hash32",
        ):
            _u32(getattr(self, name), name)
        for name in (
            "field_width_mm",
            "field_height_mm",
            "nominal_spacing_mm",
            "start_pos_tolerance_mm",
            "start_yaw_tolerance_ddeg",
            "start_stable_time_ms",
            "arrival_pos_tolerance_mm",
            "arrival_yaw_tolerance_ddeg",
            "arrival_speed_tolerance_mmps",
            "arrival_wz_tolerance_ddegps",
            "arrival_stable_time_ms",
            "finish_envelope_margin_mm",
            "finish_stable_time_ms",
            "finish_brake_accel_mmps2",
            "finish_max_runout_mm",
            "finish_hard_timeout_ms",
            "finish_signal_flags",
        ):
            _u16(getattr(self, name), name)
        _u8(self.finish_axis, "finish_axis")
        _i8(self.finish_direction, "finish_direction")
        _i16(self.finish_line_mm, "finish_line_mm")


@dataclass(frozen=True)
class NodeV40:
    s_mm: int
    x_mm: int
    y_mm: int
    yaw_ddeg: int
    vx_mmps: int
    vy_mmps: int
    wz_ddegps: int
    arrival_id: int = 0xFF
    flags: int = 0

    def to_tuple(self) -> tuple:
        self.validate()
        return (
            self.s_mm,
            self.x_mm,
            self.y_mm,
            self.yaw_ddeg,
            self.vx_mmps,
            self.vy_mmps,
            self.wz_ddegps,
            self.arrival_id,
            self.flags,
        )

    @classmethod
    def from_tuple(cls, values: tuple) -> "NodeV40":
        node = cls(*values)
        node.validate()
        return node

    def validate(self) -> None:
        _u16(self.s_mm, "node.s_mm")
        for name in ("x_mm", "y_mm", "yaw_ddeg", "vx_mmps", "vy_mmps", "wz_ddegps"):
            _i16(getattr(self, name), f"node.{name}")
        _u8(self.arrival_id, "node.arrival_id")
        _u8(self.flags, "node.flags")
        if self.flags & ~sum(int(flag) for flag in NodeFlag):
            raise V40ValidationError("V40 Node", "flags", "reserved flag bits are set", actual=self.flags)


@dataclass(frozen=True)
class SegmentV40:
    segment_id: int
    start_node_index: int
    end_node_index: int
    start_s_mm: int
    end_s_mm: int
    start_arrival_id: int = 0xFF
    end_arrival_id: int = 0xFF
    flags: int = 0
    reserved0: int = 0
    planned_time_ms: int = 0
    source_leg_hash32: int = 0
    reserved1: int = 0

    def to_tuple(self) -> tuple:
        self.validate()
        return (
            self.segment_id,
            self.start_node_index,
            self.end_node_index,
            self.start_s_mm,
            self.end_s_mm,
            self.start_arrival_id,
            self.end_arrival_id,
            self.flags,
            self.reserved0,
            self.planned_time_ms,
            self.source_leg_hash32,
            self.reserved1,
        )

    @classmethod
    def from_tuple(cls, values: tuple) -> "SegmentV40":
        segment = cls(*values)
        segment.validate()
        return segment

    def validate(self) -> None:
        for name in ("segment_id", "start_node_index", "end_node_index", "start_s_mm", "end_s_mm", "reserved1"):
            _u16(getattr(self, name), f"segment.{name}")
        for name in ("start_arrival_id", "end_arrival_id", "flags", "reserved0"):
            _u8(getattr(self, name), f"segment.{name}")
        if self.reserved0 != 0 or self.reserved1 != 0:
            raise V40ValidationError(
                "V40 Segment",
                "reserved",
                "reserved fields must be zero",
                actual=(self.reserved0, self.reserved1),
            )
        if self.flags & ~sum(int(flag) for flag in SegmentFlag):
            raise V40ValidationError("V40 Segment", "flags", "reserved flag bits are set", actual=self.flags)
        _u32(self.planned_time_ms, "segment.planned_time_ms")
        _u32(self.source_leg_hash32, "segment.source_leg_hash32")


@dataclass(frozen=True)
class ActionV40:
    action_seq: int
    action: int
    mode: int
    arrival_id: int = 0xFF
    timeout_ms: int = 0
    post_wait_ms: int = 0
    check_start_s_mm: int = 0xFFFF
    accel_limit_mmps2: int = 0
    beta_limit_ddegps2: int = 0
    wz_limit_ddegps: int = 0
    speed_limit_mmps: int = 0
    stable_time_ms: int = 0
    reserved: int = 0

    def to_tuple(self) -> tuple:
        self.validate()
        return (
            self.action_seq,
            self.action,
            self.mode,
            self.arrival_id,
            self.timeout_ms,
            self.post_wait_ms,
            self.check_start_s_mm,
            self.accel_limit_mmps2,
            self.beta_limit_ddegps2,
            self.wz_limit_ddegps,
            self.speed_limit_mmps,
            self.stable_time_ms,
            self.reserved,
        )

    @classmethod
    def from_tuple(cls, values: tuple) -> "ActionV40":
        action = cls(*values)
        action.validate()
        return action

    def validate(self) -> None:
        for name in ("action_seq", "action", "mode", "arrival_id"):
            _u8(getattr(self, name), f"action.{name}")
        if self.action not in [int(item) for item in ActionCode]:
            raise V40ValidationError("V40 Action", "action", "unknown action code", actual=self.action)
        if self.mode not in [int(item) for item in ActionMode]:
            raise V40ValidationError("V40 Action", "mode", "unknown action mode", actual=self.mode)
        for name in (
            "timeout_ms",
            "post_wait_ms",
            "check_start_s_mm",
            "accel_limit_mmps2",
            "beta_limit_ddegps2",
            "wz_limit_ddegps",
            "speed_limit_mmps",
            "stable_time_ms",
            "reserved",
        ):
            _u16(getattr(self, name), f"action.{name}")
        if self.reserved != 0:
            raise V40ValidationError("V40 Action", "reserved", "reserved field must be zero", actual=self.reserved)


@dataclass(frozen=True)
class CompiledTrajectoryV40:
    header: HeaderV40
    nodes: tuple[NodeV40, ...]
    segments: tuple[SegmentV40, ...]
    actions: tuple[ActionV40, ...] = ()

    def normalized(self) -> "CompiledTrajectoryV40":
        arrival_count = sum(1 for node in self.nodes if node.flags & int(NodeFlag.ARRIVAL))
        total_length_mm = self.nodes[-1].s_mm if self.nodes else 0
        header = self.header.with_layout_counts(
            len(self.nodes),
            len(self.segments),
            len(self.actions),
            arrival_count,
            total_length_mm=total_length_mm,
        )
        return replace(self, header=header)

    def validate(self) -> None:
        normalized = self.normalized()
        normalized.header.validate()
        if self.header != normalized.header and self.header.file_crc32 == 0:
            pass
        for index, node in enumerate(self.nodes):
            node.validate()
            if index == 0 and not (node.flags & int(NodeFlag.START)):
                raise V40ValidationError("V40 Trajectory", "nodes[0].flags", "first node must be START")
        if not self.nodes or not (self.nodes[-1].flags & int(NodeFlag.FINISH_ARM)):
            raise V40ValidationError("V40 Trajectory", "nodes[-1].flags", "last node must be FINISH_ARM")
        for segment in self.segments:
            segment.validate()
            if segment.start_node_index > segment.end_node_index:
                raise V40ValidationError(
                    "V40 Segment",
                    "start_node_index",
                    "start must be <= end",
                    actual=(segment.start_node_index, segment.end_node_index),
                )
        expected_action_seq = 0
        for action in self.actions:
            action.validate()
            if action.action_seq != expected_action_seq:
                raise V40ValidationError(
                    "V40 Action",
                    "action_seq",
                    "actions must be contiguous",
                    actual=action.action_seq,
                    expected=expected_action_seq,
                )
            expected_action_seq += 1


def tupled(items: Iterable) -> tuple:
    return tuple(items)
