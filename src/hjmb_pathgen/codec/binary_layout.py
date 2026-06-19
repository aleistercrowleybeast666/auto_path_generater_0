"""Packed V4.0 binary layout and minimal encode/decode."""

from __future__ import annotations

import struct
from dataclasses import replace

from hjmb_pathgen.models.compiled import (
    ActionV40,
    CompiledTrajectoryV40,
    HeaderV40,
    NodeV40,
    SegmentV40,
)
from hjmb_pathgen.models.errors import BinaryCrcError, BinaryFormatError, BinaryLayoutError
from hjmb_pathgen.models.protocol import ACTION_SIZE, HEADER_SIZE, NODE_SIZE, SEGMENT_SIZE

from .crc32 import crc32_ieee
from .validators import validate_compiled_trajectory

HEADER_FMT = "<4sBBBBBBBBHHBBHHHHHBBHIIIIIIIIIHHHHHHHHBbhHHHHHHI"
NODE_FMT = "<HhhhhhhBB"
SEGMENT_FMT = "<HHHHHBBBBIIH"
ACTION_FMT = "<BBBBHHHHHHHHH"

HEADER_FIELD_NAMES = (
    "magic",
    "version",
    "header_size",
    "node_size",
    "segment_size",
    "action_size",
    "route_family",
    "finish_mode",
    "reserved_u8",
    "traj_id",
    "flags",
    "bean_code",
    "drop_code",
    "field_width_mm",
    "field_height_mm",
    "nominal_spacing_mm",
    "node_count",
    "segment_count",
    "action_count",
    "arrival_count",
    "reserved0",
    "file_crc32",
    "node_offset",
    "segment_offset",
    "action_offset",
    "total_length_mm",
    "planned_motion_time_ms",
    "planned_total_estimate_ms",
    "source_case_hash32",
    "source_project_hash32",
    "start_pos_tolerance_mm",
    "start_yaw_tolerance_ddeg",
    "start_stable_time_ms",
    "arrival_pos_tolerance_mm",
    "arrival_yaw_tolerance_ddeg",
    "arrival_speed_tolerance_mmps",
    "arrival_wz_tolerance_ddegps",
    "arrival_stable_time_ms",
    "finish_axis",
    "finish_direction",
    "finish_line_mm",
    "finish_envelope_margin_mm",
    "finish_stable_time_ms",
    "finish_brake_accel_mmps2",
    "finish_max_runout_mm",
    "finish_hard_timeout_ms",
    "finish_signal_flags",
    "reserved1",
)

CRC32_OFFSET = struct.calcsize("<4sBBBBBBBBHHBBHHHHHBBH")

assert struct.calcsize(HEADER_FMT) == HEADER_SIZE
assert struct.calcsize(NODE_FMT) == NODE_SIZE
assert struct.calcsize(SEGMENT_FMT) == SEGMENT_SIZE
assert struct.calcsize(ACTION_FMT) == ACTION_SIZE


def _pack_header(header: HeaderV40, *, zero_crc: bool) -> bytes:
    return struct.pack(HEADER_FMT, *header.to_tuple(zero_crc=zero_crc))


def _pack_items(fmt: str, items: tuple) -> bytes:
    payload = bytearray()
    for item in items:
        payload.extend(struct.pack(fmt, *item.to_tuple()))
    return bytes(payload)


def encode_compiled_trajectory(trajectory: CompiledTrajectoryV40) -> bytes:
    trajectory = trajectory.normalized()
    validate_compiled_trajectory(trajectory)
    header = replace(trajectory.header, file_crc32=0)
    payload = bytearray()
    payload.extend(_pack_header(header, zero_crc=True))
    payload.extend(_pack_items(NODE_FMT, trajectory.nodes))
    payload.extend(_pack_items(SEGMENT_FMT, trajectory.segments))
    payload.extend(_pack_items(ACTION_FMT, trajectory.actions))
    crc = crc32_ieee(bytes(payload))
    payload[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
    return bytes(payload)


def _check_file_crc(data: bytes, actual_crc: int) -> None:
    mutable = bytearray(data)
    mutable[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
    expected = crc32_ieee(bytes(mutable))
    if actual_crc != expected:
        raise BinaryCrcError(
            "V40 BIN",
            "file_crc32",
            "CRC mismatch",
            actual=f"0x{actual_crc:08X}",
            expected=f"0x{expected:08X}",
        )


def decode_compiled_trajectory(data: bytes) -> CompiledTrajectoryV40:
    if len(data) < HEADER_SIZE:
        raise BinaryFormatError("V40 BIN", "file_size", "file too short", actual=len(data), expected=HEADER_SIZE)
    header_values = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    header = HeaderV40.from_tuple(header_values)
    expected_size = header.file_size
    if len(data) != expected_size:
        raise BinaryLayoutError(
            "V40 BIN",
            "file_size",
            "wrong file size",
            actual=len(data),
            expected=expected_size,
        )
    if header.node_offset != HEADER_SIZE:
        raise BinaryLayoutError(
            "V40 Header",
            "node_offset",
            "wrong node offset",
            actual=header.node_offset,
            expected=HEADER_SIZE,
        )
    expected_segment_offset = header.node_offset + header.node_count * NODE_SIZE
    expected_action_offset = expected_segment_offset + header.segment_count * SEGMENT_SIZE
    if header.segment_offset != expected_segment_offset:
        raise BinaryLayoutError(
            "V40 Header",
            "segment_offset",
            "wrong segment offset",
            actual=header.segment_offset,
            expected=expected_segment_offset,
        )
    if header.action_offset != expected_action_offset:
        raise BinaryLayoutError(
            "V40 Header",
            "action_offset",
            "wrong action offset",
            actual=header.action_offset,
            expected=expected_action_offset,
        )
    _check_file_crc(data, header.file_crc32)

    nodes = []
    for index in range(header.node_count):
        offset = header.node_offset + index * NODE_SIZE
        nodes.append(NodeV40.from_tuple(struct.unpack(NODE_FMT, data[offset : offset + NODE_SIZE])))
    segments = []
    for index in range(header.segment_count):
        offset = header.segment_offset + index * SEGMENT_SIZE
        segments.append(SegmentV40.from_tuple(struct.unpack(SEGMENT_FMT, data[offset : offset + SEGMENT_SIZE])))
    actions = []
    for index in range(header.action_count):
        offset = header.action_offset + index * ACTION_SIZE
        actions.append(ActionV40.from_tuple(struct.unpack(ACTION_FMT, data[offset : offset + ACTION_SIZE])))

    trajectory = CompiledTrajectoryV40(header=header, nodes=tuple(nodes), segments=tuple(segments), actions=tuple(actions))
    validate_compiled_trajectory(trajectory)
    return trajectory
