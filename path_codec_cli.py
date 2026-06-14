# -*- coding: utf-8 -*-
"""HJMB V2.5 path BIN codec, V2 migration helpers, and command-line tool."""
from __future__ import annotations

import argparse
import json
import re
import struct
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

MAGIC = b"HJMB"
VERSION = 25
VERSION_V2 = 2
HEADER_SIZE = 32
POINT_SIZE = 12
ACTION_SIZE = 8
MAX_POINTS = 30
MAX_ACTIONS = 32
MAX_GATES = 30
MAX_TRAJ_ID = 359
HEADER_FMT = "<4sBBBBHHIBBBB3I"
HEADER_FMT_V2 = "<4sBBBBHHI4I"
POINT_FMT = "<hhhBBBBBB"
POINT_FMT_V2 = POINT_FMT
ACTION_FMT = "<BBBBHH"
CRC32_OFFSET = 12
PROJECT_FORMAT = "HJMB_PATH_EDITOR_JSON_V25"
PROJECT_FORMAT_V2 = "HJMB_PATH_EDITOR_JSON_V2"

PATH_POINT_PASS = 0x00
PATH_POINT_ARRIVE_SCAN = 0x01
POINT_TYPES: Dict[int, str] = {
    PATH_POINT_PASS: "0x00 途径点 PASS",
    PATH_POINT_ARRIVE_SCAN: "0x01 到达扫码 ARRIVE_SCAN",
}

ACTIONS: Dict[int, str] = {
    0x11: "0x11 预备取豆1 PREP_PICK_1",
    0x12: "0x12 预备取豆2L PREP_PICK_2L",
    0x13: "0x13 预备取豆2R PREP_PICK_2R",
    0x14: "0x14 预备取豆3 PREP_PICK_3",
    0x20: "0x20 取豆 PICK",
    0x31: "0x31 暂存1 STORE_1",
    0x32: "0x32 暂存2 STORE_2",
    0x33: "0x33 暂存3 STORE_3",
    0x41: "0x41 倒1号箱 DROP_BOX1",
    0x42: "0x42 倒2号箱 DROP_BOX2",
    0x43: "0x43 同时倒1/2号箱 DROP_BOX1_2",
    0x44: "0x44 倒3号箱 DROP_BOX3",
    0x46: "0x46 同时倒2/3号箱 DROP_BOX2_3",
}
DROP_ACTIONS = {0x41, 0x42, 0x43, 0x44, 0x46}

PATH_FLAG_WAIT_ACTION_V2 = 0x01
PATH_FLAG_SKIP_SCAN = 0x02
PATH_FLAG_SLOW_ZONE = 0x04
PATH_FLAG_END = 0x80
VALID_PATH_FLAGS_MASK = PATH_FLAG_SKIP_SCAN | PATH_FLAG_SLOW_ZONE | PATH_FLAG_END
VALID_PATH_FLAGS_MASK_V2 = VALID_PATH_FLAGS_MASK | PATH_FLAG_WAIT_ACTION_V2

ACTION_FLAG_LOCKED = 0x01
ACTION_FLAG_HOLD_PATH = 0x02
ACTION_FLAG_REQUIRED_AT_END = 0x04
VALID_ACTION_FLAGS_MASK = ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH | ACTION_FLAG_REQUIRED_AT_END
ACTION_FLAGS: Dict[int, str] = {
    ACTION_FLAG_LOCKED: "LOCKED",
    ACTION_FLAG_HOLD_PATH: "HOLD_PATH",
    ACTION_FLAG_REQUIRED_AT_END: "REQUIRED_AT_END",
}


@dataclass
class PathPoint:
    x_mm: int = 0
    y_mm: int = 0
    yaw_ddeg: int = 0
    point_id: int = 0
    type: int = PATH_POINT_PASS
    gate_id: int = 0xFF
    marker_id: int = 0xFF
    flags: int = 0

    @staticmethod
    def from_dict(data: dict) -> "PathPoint":
        return PathPoint(
            x_mm=parse_int(data.get("x_mm", 0), "x_mm"),
            y_mm=parse_int(data.get("y_mm", 0), "y_mm"),
            yaw_ddeg=parse_int(data.get("yaw_ddeg", 0), "yaw_ddeg"),
            point_id=parse_int(data.get("point_id", 0), "point_id"),
            type=parse_int(data.get("type", PATH_POINT_PASS), "type"),
            gate_id=parse_int(data.get("gate_id", 0xFF), "gate_id"),
            marker_id=parse_int(data.get("marker_id", 0xFF), "marker_id"),
            flags=parse_int(data.get("flags", 0), "flags"),
        )


@dataclass
class MechanicalAction:
    action_seq: int = 0
    action: int = 0x11
    unlock_gate_id: int = 0xFF
    flags: int = 0
    timeout_ms: int = 0

    @staticmethod
    def from_dict(data: dict) -> "MechanicalAction":
        return MechanicalAction(
            action_seq=parse_int(data.get("action_seq", 0), "action_seq"),
            action=parse_int(data.get("action", 0x11), "action"),
            unlock_gate_id=parse_int(data.get("unlock_gate_id", 0xFF), "unlock_gate_id"),
            flags=parse_int(data.get("flags", 0), "flags"),
            timeout_ms=parse_int(data.get("timeout_ms", 0), "timeout_ms"),
        )


@dataclass
class LegacyPathPoint:
    x_mm: int
    y_mm: int
    yaw_ddeg: int
    point_id: int
    type: int
    action: int
    marker_id: int
    flags: int

    @staticmethod
    def from_dict(data: dict) -> "LegacyPathPoint":
        return LegacyPathPoint(
            x_mm=parse_int(data.get("x_mm", 0), "x_mm"),
            y_mm=parse_int(data.get("y_mm", 0), "y_mm"),
            yaw_ddeg=parse_int(data.get("yaw_ddeg", 0), "yaw_ddeg"),
            point_id=parse_int(data.get("point_id", 0), "point_id"),
            type=parse_int(data.get("type", PATH_POINT_PASS), "type"),
            action=parse_int(data.get("action", 0), "action"),
            marker_id=parse_int(data.get("marker_id", 0xFF), "marker_id"),
            flags=parse_int(data.get("flags", 0), "flags"),
        )


@dataclass
class ProjectLoadResult:
    traj_id: int
    points: List[PathPoint]
    actions: List[MechanicalAction]
    migrated_from_v2: bool = False
    migration_summary: str = ""


def parse_int(value, field_name: str = "value") -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    try:
        return int(str(value).strip(), 0)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须是十进制或 0x 前缀整数，当前为 {value!r}") from exc


def hex8(value: int) -> str:
    return f"0x{value & 0xFF:02X}"


def flags_to_text(flags: int, names: Dict[int, str]) -> str:
    labels = [name for bit, name in names.items() if flags & bit]
    return hex8(flags) + ((" " + "|".join(labels)) if labels else "")


def bin_path_traj_id(path: Path) -> int:
    match = re.fullmatch(r"P(\d{4})\.BIN", path.name, re.IGNORECASE)
    if match is None:
        raise ValueError("BIN 文件名必须为 P0000.BIN ~ P0359.BIN")
    traj_id = int(match.group(1))
    if not (0 <= traj_id <= MAX_TRAJ_ID):
        raise ValueError("BIN 文件名编号必须在 P0000.BIN ~ P0359.BIN 范围内")
    return traj_id


def gate_count_from_points(points: List[PathPoint]) -> int:
    return sum(point.gate_id != 0xFF for point in points)


def validate_project(
    traj_id: int,
    points: List[PathPoint],
    actions: List[MechanicalAction],
) -> List[str]:
    errors: List[str] = []
    if not (0 <= traj_id <= MAX_TRAJ_ID):
        errors.append(f"traj_id 必须在 0~{MAX_TRAJ_ID} 范围内，当前为 {traj_id}")
    if not (1 <= len(points) <= MAX_POINTS):
        errors.append(f"路径点数量必须为 1~{MAX_POINTS}，当前为 {len(points)}")
    if not (0 <= len(actions) <= MAX_ACTIONS):
        errors.append(f"机械动作数量必须为 0~{MAX_ACTIONS}，当前为 {len(actions)}")

    gate_ids: List[int] = []
    gate_types: Dict[int, int] = {}
    for row, point in enumerate(points):
        if point.point_id != row:
            errors.append(f"路径点第 {row} 行 point_id={point.point_id}，应为 {row}")
        if not (-32768 <= point.x_mm <= 32767):
            errors.append(f"路径点第 {row} 行 x_mm={point.x_mm}，超出 int16_t 范围")
        if not (-32768 <= point.y_mm <= 32767):
            errors.append(f"路径点第 {row} 行 y_mm={point.y_mm}，超出 int16_t 范围")
        if not (0 <= point.yaw_ddeg <= 3599):
            errors.append(f"路径点第 {row} 行 yaw_ddeg={point.yaw_ddeg}，应为 0~3599")
        if point.type not in POINT_TYPES:
            errors.append(f"路径点第 {row} 行 type={hex8(point.type)} 非法")
        if point.gate_id != 0xFF:
            if not (0 <= point.gate_id < MAX_GATES):
                errors.append(
                    f"路径点第 {row} 行 gate_id={point.gate_id}，必须为 0xFF 或 0~{MAX_GATES - 1}"
                )
            elif point.gate_id in gate_types:
                errors.append(f"路径点第 {row} 行 gate_id={point.gate_id} 与前面的 Gate 重复")
            else:
                gate_ids.append(point.gate_id)
                gate_types[point.gate_id] = point.type
        if not (0 <= point.marker_id <= 0xFF):
            errors.append(f"路径点第 {row} 行 marker_id={point.marker_id}，应为 0~255")
        if not (0 <= point.flags <= 0xFF):
            errors.append(f"路径点第 {row} 行 flags={point.flags}，应为 0~255")
        elif point.flags & ~VALID_PATH_FLAGS_MASK:
            errors.append(
                f"路径点第 {row} 行 flags 含未定义位 {hex8(point.flags & ~VALID_PATH_FLAGS_MASK)}；bit0 在 V2.5 必须为 0"
            )
        if point.flags & PATH_FLAG_END and row != len(points) - 1:
            errors.append(f"路径点第 {row} 行 END 只能设置在最后一个路径点")

    expected_gate_ids = list(range(len(gate_ids)))
    if gate_ids != expected_gate_ids:
        errors.append(
            f"Gate 必须按路径顺序连续为 {expected_gate_ids}，当前为 {gate_ids}"
        )
    if len(gate_ids) > MAX_GATES:
        errors.append(f"Gate 数量必须为 0~{MAX_GATES}，当前为 {len(gate_ids)}")
    if points and not (points[-1].flags & PATH_FLAG_END):
        errors.append("最后一个路径点必须设置 END flag")

    previous_locked_gate = -1
    for row, action_item in enumerate(actions):
        if action_item.action_seq != row:
            errors.append(
                f"机械动作第 {row} 行 action_seq={action_item.action_seq}，应为 {row}"
            )
        if action_item.action not in ACTIONS:
            if action_item.action == 0:
                errors.append(f"机械动作第 {row} 行 action=0x00；动作数组不能保存 NONE")
            else:
                errors.append(f"机械动作第 {row} 行 action={hex8(action_item.action)} 非法")
        if not (0 <= action_item.unlock_gate_id <= 0xFF):
            errors.append(
                f"机械动作第 {row} 行 unlock_gate_id={action_item.unlock_gate_id}，应为 0~255"
            )
        if not (0 <= action_item.flags <= 0xFF):
            errors.append(f"机械动作第 {row} 行 flags={action_item.flags}，应为 0~255")
        elif action_item.flags & ~VALID_ACTION_FLAGS_MASK:
            errors.append(
                f"机械动作第 {row} 行 flags 含未定义位 {hex8(action_item.flags & ~VALID_ACTION_FLAGS_MASK)}"
            )
        if not (0 <= action_item.timeout_ms <= 0xFFFF):
            errors.append(
                f"机械动作第 {row} 行 timeout_ms={action_item.timeout_ms}，应为 0~65535"
            )

        locked = bool(action_item.flags & ACTION_FLAG_LOCKED)
        hold_path = bool(action_item.flags & ACTION_FLAG_HOLD_PATH)
        required_at_end = bool(action_item.flags & ACTION_FLAG_REQUIRED_AT_END)
        gate_exists = action_item.unlock_gate_id in gate_types

        if not locked and action_item.unlock_gate_id != 0xFF:
            errors.append(
                f"机械动作第 {row} 行 LOCKED=0 时 unlock_gate_id 必须为 0xFF，当前为 {action_item.unlock_gate_id}"
            )
        if locked and not gate_exists:
            errors.append(
                f"机械动作第 {row} 行 LOCKED=1，但 unlock_gate_id={action_item.unlock_gate_id} 未引用现有 Gate"
            )
        if hold_path and not locked:
            errors.append(f"机械动作第 {row} 行 HOLD_PATH 必须同时设置 LOCKED")
        if required_at_end and not gate_exists:
            errors.append(
                f"机械动作第 {row} 行 REQUIRED_AT_END 必须引用现有 Gate，当前 unlock_gate_id={hex8(action_item.unlock_gate_id)}"
            )

        is_key_action = action_item.action == 0x20 or action_item.action in DROP_ACTIONS
        if is_key_action:
            required_flags = ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH
            if action_item.flags & required_flags != required_flags:
                errors.append(
                    f"机械动作第 {row} 行 {ACTIONS.get(action_item.action, hex8(action_item.action))} 必须设置 LOCKED|HOLD_PATH"
                )
            if not gate_exists:
                errors.append(
                    f"机械动作第 {row} 行关键动作必须引用现有 ARRIVE_SCAN Gate"
                )
            elif gate_types[action_item.unlock_gate_id] != PATH_POINT_ARRIVE_SCAN:
                errors.append(
                    f"机械动作第 {row} 行引用 Gate {action_item.unlock_gate_id}，但该 Gate 位于 PASS 点；PICK/DROP 必须引用 ARRIVE_SCAN Gate"
                )

        if locked and gate_exists:
            if action_item.unlock_gate_id < previous_locked_gate:
                errors.append(
                    f"机械动作第 {row} 行 unlock_gate_id={action_item.unlock_gate_id} 小于前一个锁定动作 Gate {previous_locked_gate}，会造成 FIFO 死锁风险"
                )
            previous_locked_gate = action_item.unlock_gate_id

    return errors


def _validate_legacy_project(traj_id: int, points: List[LegacyPathPoint]) -> List[str]:
    errors: List[str] = []
    if not (0 <= traj_id <= MAX_TRAJ_ID):
        errors.append(f"V2 traj_id 必须在 0~{MAX_TRAJ_ID}，当前为 {traj_id}")
    if not (1 <= len(points) <= MAX_POINTS):
        errors.append(f"V2 路径点数量必须为 1~{MAX_POINTS}，当前为 {len(points)}")
    for row, point in enumerate(points):
        if point.point_id != row:
            errors.append(f"V2 路径点第 {row} 行 point_id={point.point_id}，应为 {row}")
        if not (-32768 <= point.x_mm <= 32767):
            errors.append(f"V2 路径点第 {row} 行 x_mm 超出 int16_t 范围")
        if not (-32768 <= point.y_mm <= 32767):
            errors.append(f"V2 路径点第 {row} 行 y_mm 超出 int16_t 范围")
        if not (0 <= point.yaw_ddeg <= 3599):
            errors.append(f"V2 路径点第 {row} 行 yaw_ddeg 应为 0~3599")
        if point.type not in POINT_TYPES:
            errors.append(f"V2 路径点第 {row} 行 type={hex8(point.type)} 非法")
        if point.action != 0 and point.action not in ACTIONS:
            errors.append(f"V2 路径点第 {row} 行 action={hex8(point.action)} 非法")
        if not (0 <= point.marker_id <= 0xFF):
            errors.append(f"V2 路径点第 {row} 行 marker_id 应为 0~255")
        if point.flags & ~VALID_PATH_FLAGS_MASK_V2:
            errors.append(
                f"V2 路径点第 {row} 行 flags 含未定义位 {hex8(point.flags & ~VALID_PATH_FLAGS_MASK_V2)}"
            )
        if point.flags & PATH_FLAG_END and row != len(points) - 1:
            errors.append(f"V2 路径点第 {row} 行 END 只能设置在最后一个路径点")
    if points and not (points[-1].flags & PATH_FLAG_END):
        errors.append("V2 最后一个路径点必须设置 END flag")
    return errors


def convert_v2_project(
    traj_id: int,
    legacy_points: List[LegacyPathPoint],
) -> ProjectLoadResult:
    legacy_errors = _validate_legacy_project(traj_id, legacy_points)
    if legacy_errors:
        raise ValueError("V2 工程字段校验失败：\n" + "\n".join(legacy_errors))

    points = [
        PathPoint(
            x_mm=point.x_mm,
            y_mm=point.y_mm,
            yaw_ddeg=point.yaw_ddeg,
            point_id=point.point_id,
            type=point.type,
            gate_id=0xFF,
            marker_id=point.marker_id,
            flags=point.flags & VALID_PATH_FLAGS_MASK,
        )
        for point in legacy_points
    ]
    actions: List[MechanicalAction] = []
    gate_count = 0
    locked_count = 0
    wait_count = 0

    for row, legacy_point in enumerate(legacy_points):
        if legacy_point.action == 0:
            continue

        is_key_action = legacy_point.action == 0x20 or legacy_point.action in DROP_ACTIONS
        had_wait = bool(legacy_point.flags & PATH_FLAG_WAIT_ACTION_V2)
        needs_gate = is_key_action or had_wait
        action_flags = 0
        unlock_gate_id = 0xFF

        if needs_gate:
            unlock_gate_id = gate_count
            points[row].gate_id = gate_count
            gate_count += 1
            action_flags |= ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH
            locked_count += 1
            if had_wait and not is_key_action:
                wait_count += 1

        if legacy_point.action in DROP_ACTIONS and row == len(legacy_points) - 1:
            action_flags |= ACTION_FLAG_REQUIRED_AT_END

        actions.append(
            MechanicalAction(
                action_seq=len(actions),
                action=legacy_point.action,
                unlock_gate_id=unlock_gate_id,
                flags=action_flags,
                timeout_ms=0,
            )
        )

    summary = (
        f"已将 V2.0 工程显式转换为 V2.5：保留 {len(points)} 个路径点，"
        f"提取 {len(actions)} 个机械动作，创建 {gate_count} 个 Gate；"
        f"其中 {locked_count} 个动作设置 LOCKED|HOLD_PATH，"
        f"{wait_count} 个来自原 WAIT_ACTION。请校验后另存为 V2.5 文件。"
    )
    return ProjectLoadResult(
        traj_id=traj_id,
        points=points,
        actions=actions,
        migrated_from_v2=True,
        migration_summary=summary,
    )


def load_project_dict(data: dict) -> ProjectLoadResult:
    project_format = data.get("format")
    if project_format == "HJMB_PATH_EDITOR_JSON_V1":
        raise ValueError("V1 action 语义与 V2/V2.5 不兼容，不能自动转换")
    if project_format == PROJECT_FORMAT:
        missing_keys = [key for key in ("points", "actions") if key not in data]
        if missing_keys:
            raise ValueError(f"V2.5 JSON 顶层缺少必需字段：{', '.join(missing_keys)}")
        return ProjectLoadResult(
            traj_id=parse_int(data.get("traj_id", 0), "traj_id"),
            points=[PathPoint.from_dict(item) for item in data.get("points", [])],
            actions=[MechanicalAction.from_dict(item) for item in data.get("actions", [])],
        )
    if project_format in (None, PROJECT_FORMAT_V2):
        if "points" not in data:
            raise ValueError("V2 JSON 顶层缺少必需字段：points")
        legacy_points = [LegacyPathPoint.from_dict(item) for item in data.get("points", [])]
        return convert_v2_project(parse_int(data.get("traj_id", 0), "traj_id"), legacy_points)
    raise ValueError(f"不支持的 JSON 工程格式：{project_format}")


def project_to_dict(
    traj_id: int,
    points: List[PathPoint],
    actions: List[MechanicalAction],
    field: Optional[dict] = None,
) -> dict:
    result = {
        "format": PROJECT_FORMAT,
        "traj_id": traj_id,
        "points": [asdict(point) for point in points],
        "actions": [asdict(action_item) for action_item in actions],
    }
    if field is not None:
        result["field"] = field
    return result


class PathCodec:
    """Strict V2.5 codec plus explicit V2 import helpers."""

    validate_project = staticmethod(validate_project)

    @staticmethod
    def build_bin(
        traj_id: int,
        points: List[PathPoint],
        actions: List[MechanicalAction],
    ) -> bytes:
        errors = validate_project(traj_id, points, actions)
        if errors:
            raise ValueError("\n".join(errors))

        gate_count = gate_count_from_points(points)
        header = struct.pack(
            HEADER_FMT,
            MAGIC,
            VERSION,
            HEADER_SIZE,
            POINT_SIZE,
            len(points),
            traj_id,
            0,
            0,
            ACTION_SIZE,
            len(actions),
            gate_count,
            0,
            0,
            0,
            0,
        )
        payload = bytearray()
        for point in points:
            payload += struct.pack(
                POINT_FMT,
                point.x_mm,
                point.y_mm,
                point.yaw_ddeg,
                point.point_id,
                point.type,
                point.gate_id,
                point.marker_id,
                point.flags,
                0,
            )
        for action_item in actions:
            payload += struct.pack(
                ACTION_FMT,
                action_item.action_seq,
                action_item.action,
                action_item.unlock_gate_id,
                action_item.flags,
                action_item.timeout_ms,
                0,
            )

        data = bytearray(header + payload)
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        crc = zlib.crc32(data) & 0xFFFFFFFF
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
        return bytes(data)

    @staticmethod
    def parse_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> Tuple[int, List[PathPoint], List[MechanicalAction]]:
        if len(data) < HEADER_SIZE:
            raise ValueError(f"文件太短：{len(data)} 字节，无法读取 32 字节 V2.5 Header")

        header = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        (
            magic,
            version,
            header_size,
            point_size,
            point_count,
            traj_id,
            file_flags,
            file_crc32,
            action_size,
            action_count,
            header_gate_count,
            reserved0,
            reserved1,
            reserved2,
            reserved3,
        ) = header
        if magic != MAGIC:
            raise ValueError(f"Header.magic={magic!r}，期望 {MAGIC!r}")
        if version != VERSION:
            raise ValueError(f"Header.version={version}，期望 V2.5 的 {VERSION}")
        if header_size != HEADER_SIZE:
            raise ValueError(f"Header.header_size={header_size}，期望 {HEADER_SIZE}")
        if point_size != POINT_SIZE:
            raise ValueError(f"Header.point_size={point_size}，期望 {POINT_SIZE}")
        if action_size != ACTION_SIZE:
            raise ValueError(f"Header.action_size={action_size}，期望 {ACTION_SIZE}")
        if not (1 <= point_count <= MAX_POINTS):
            raise ValueError(f"Header.point_count={point_count}，应为 1~{MAX_POINTS}")
        if not (0 <= action_count <= MAX_ACTIONS):
            raise ValueError(f"Header.action_count={action_count}，应为 0~{MAX_ACTIONS}")
        if not (0 <= header_gate_count <= MAX_GATES):
            raise ValueError(f"Header.gate_count={header_gate_count}，应为 0~{MAX_GATES}")
        if not (0 <= traj_id <= MAX_TRAJ_ID):
            raise ValueError(f"Header.traj_id={traj_id}，应为 0~{MAX_TRAJ_ID}")
        if expected_traj_id is not None and traj_id != expected_traj_id:
            raise ValueError(f"Header.traj_id={traj_id}，文件名要求 {expected_traj_id}")
        if file_flags != 0:
            raise ValueError(f"Header.flags=0x{file_flags:04X}，必须为 0")
        if reserved0 != 0:
            raise ValueError(f"Header.reserved0={reserved0}，必须为 0")
        if any((reserved1, reserved2, reserved3)):
            raise ValueError("Header.reserved[3] 必须全部为 0")

        expected_size = HEADER_SIZE + point_count * POINT_SIZE + action_count * ACTION_SIZE
        if len(data) != expected_size:
            raise ValueError(f"文件大小={len(data)}，按 Header 应为 {expected_size}")

        crc_data = bytearray(data)
        crc_data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        calculated_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
        if calculated_crc != file_crc32:
            raise ValueError(
                f"CRC32 校验失败：文件为 0x{file_crc32:08X}，计算为 0x{calculated_crc:08X}"
            )

        points: List[PathPoint] = []
        offset = HEADER_SIZE
        for row in range(point_count):
            values = struct.unpack(POINT_FMT, data[offset : offset + POINT_SIZE])
            x_mm, y_mm, yaw_ddeg, point_id, point_type, gate_id, marker_id, flags, reserved = values
            if reserved != 0:
                raise ValueError(f"路径点第 {row} 行 reserved={reserved}，必须为 0")
            points.append(
                PathPoint(
                    x_mm=x_mm,
                    y_mm=y_mm,
                    yaw_ddeg=yaw_ddeg,
                    point_id=point_id,
                    type=point_type,
                    gate_id=gate_id,
                    marker_id=marker_id,
                    flags=flags,
                )
            )
            offset += POINT_SIZE

        actions: List[MechanicalAction] = []
        for row in range(action_count):
            values = struct.unpack(ACTION_FMT, data[offset : offset + ACTION_SIZE])
            action_seq, action, unlock_gate_id, flags, timeout_ms, reserved = values
            if reserved != 0:
                raise ValueError(f"机械动作第 {row} 行 reserved={reserved}，必须为 0")
            actions.append(
                MechanicalAction(
                    action_seq=action_seq,
                    action=action,
                    unlock_gate_id=unlock_gate_id,
                    flags=flags,
                    timeout_ms=timeout_ms,
                )
            )
            offset += ACTION_SIZE

        errors = validate_project(traj_id, points, actions)
        actual_gate_count = gate_count_from_points(points)
        if actual_gate_count != header_gate_count:
            errors.insert(
                0,
                f"Header.gate_count={header_gate_count}，但路径点实际推导为 {actual_gate_count}",
            )
        if errors:
            raise ValueError("V2.5 字段校验失败：\n" + "\n".join(errors))
        return traj_id, points, actions

    @staticmethod
    def build_v2_bin(traj_id: int, points: List[LegacyPathPoint]) -> bytes:
        errors = _validate_legacy_project(traj_id, points)
        if errors:
            raise ValueError("\n".join(errors))
        header = struct.pack(
            HEADER_FMT_V2,
            MAGIC,
            VERSION_V2,
            HEADER_SIZE,
            POINT_SIZE,
            len(points),
            traj_id,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        payload = bytearray()
        for point in points:
            payload += struct.pack(
                POINT_FMT_V2,
                point.x_mm,
                point.y_mm,
                point.yaw_ddeg,
                point.point_id,
                point.type,
                point.action,
                point.marker_id,
                point.flags,
                0,
            )
        data = bytearray(header + payload)
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        crc = zlib.crc32(data) & 0xFFFFFFFF
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
        return bytes(data)

    @staticmethod
    def parse_v2_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> Tuple[int, List[LegacyPathPoint]]:
        if len(data) < HEADER_SIZE:
            raise ValueError(f"V2 文件太短：{len(data)} 字节")
        (
            magic,
            version,
            header_size,
            point_size,
            point_count,
            traj_id,
            file_flags,
            file_crc32,
            *reserved,
        ) = struct.unpack(HEADER_FMT_V2, data[:HEADER_SIZE])
        if magic != MAGIC:
            raise ValueError(f"V2 Header.magic={magic!r}，期望 {MAGIC!r}")
        if version != VERSION_V2:
            raise ValueError(f"V2 Header.version={version}，期望 {VERSION_V2}")
        if header_size != HEADER_SIZE:
            raise ValueError(f"V2 Header.header_size={header_size}，期望 {HEADER_SIZE}")
        if point_size != POINT_SIZE:
            raise ValueError(f"V2 Header.point_size={point_size}，期望 {POINT_SIZE}")
        if not (1 <= point_count <= MAX_POINTS):
            raise ValueError(f"V2 Header.point_count={point_count}，应为 1~{MAX_POINTS}")
        if not (0 <= traj_id <= MAX_TRAJ_ID):
            raise ValueError(f"V2 Header.traj_id={traj_id}，应为 0~{MAX_TRAJ_ID}")
        if expected_traj_id is not None and traj_id != expected_traj_id:
            raise ValueError(f"V2 Header.traj_id={traj_id}，文件名要求 {expected_traj_id}")
        if file_flags != 0:
            raise ValueError(f"V2 Header.flags=0x{file_flags:04X}，必须为 0")
        if any(reserved):
            raise ValueError("V2 Header.reserved[4] 必须全部为 0")

        expected_size = HEADER_SIZE + point_count * POINT_SIZE
        if len(data) != expected_size:
            raise ValueError(f"V2 文件大小={len(data)}，按 Header 应为 {expected_size}")
        crc_data = bytearray(data)
        crc_data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        calculated_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
        if calculated_crc != file_crc32:
            raise ValueError(
                f"V2 CRC32 校验失败：文件为 0x{file_crc32:08X}，计算为 0x{calculated_crc:08X}"
            )

        points: List[LegacyPathPoint] = []
        offset = HEADER_SIZE
        for row in range(point_count):
            values = struct.unpack(POINT_FMT_V2, data[offset : offset + POINT_SIZE])
            x_mm, y_mm, yaw_ddeg, point_id, point_type, action, marker_id, flags, point_reserved = values
            if point_reserved != 0:
                raise ValueError(f"V2 路径点第 {row} 行 reserved={point_reserved}，必须为 0")
            points.append(
                LegacyPathPoint(
                    x_mm=x_mm,
                    y_mm=y_mm,
                    yaw_ddeg=yaw_ddeg,
                    point_id=point_id,
                    type=point_type,
                    action=action,
                    marker_id=marker_id,
                    flags=flags,
                )
            )
            offset += POINT_SIZE

        errors = _validate_legacy_project(traj_id, points)
        if errors:
            raise ValueError("V2 字段校验失败：\n" + "\n".join(errors))
        return traj_id, points

    @staticmethod
    def load_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> ProjectLoadResult:
        if len(data) < 5:
            raise ValueError("文件太短，无法识别 HJMB 版本")
        version = data[4]
        if version == VERSION:
            traj_id, points, actions = PathCodec.parse_bin(data, expected_traj_id)
            return ProjectLoadResult(traj_id, points, actions)
        if version == VERSION_V2:
            traj_id, legacy_points = PathCodec.parse_v2_bin(data, expected_traj_id)
            return convert_v2_project(traj_id, legacy_points)
        raise ValueError(f"不支持的 HJMB BIN version={version}；仅支持 V2.5(25) 和显式迁移 V2.0(2)")


def json_to_bin(json_path: Path, bin_path: Path) -> ProjectLoadResult:
    project = load_project_dict(json.loads(json_path.read_text(encoding="utf-8")))
    if bin_path_traj_id(bin_path) != project.traj_id:
        raise ValueError(
            f"BIN 文件名编号必须与 traj_id 一致，应为 P{project.traj_id:04d}.BIN"
        )
    data = PathCodec.build_bin(project.traj_id, project.points, project.actions)
    bin_path.write_bytes(data)
    PathCodec.parse_bin(bin_path.read_bytes(), expected_traj_id=project.traj_id)
    return project


def main() -> None:
    parser = argparse.ArgumentParser(description="HJMB V2.5 路径 BIN 生成/校验工具")
    subparsers = parser.add_subparsers(dest="cmd", required=True)
    build_parser = subparsers.add_parser("build", help="从 V2.5/V2 JSON 生成 V2.5 BIN")
    build_parser.add_argument("json")
    build_parser.add_argument("bin")
    check_parser = subparsers.add_parser("check", help="校验 V2.5 BIN，V2 BIN 会显式迁移后报告")
    check_parser.add_argument("bin")
    args = parser.parse_args()

    if args.cmd == "build":
        project = json_to_bin(Path(args.json), Path(args.bin))
        if project.migrated_from_v2:
            print(project.migration_summary)
        print(
            f"OK: traj_id={project.traj_id}, point_count={len(project.points)}, "
            f"action_count={len(project.actions)}, gate_count={gate_count_from_points(project.points)}"
        )
        return

    bin_path = Path(args.bin)
    project = PathCodec.load_bin(
        bin_path.read_bytes(),
        expected_traj_id=bin_path_traj_id(bin_path),
    )
    if project.migrated_from_v2:
        print(project.migration_summary)
    print(f"traj_id={project.traj_id}")
    print(f"point_count={len(project.points)}")
    print(f"action_count={len(project.actions)}")
    print(f"gate_count={gate_count_from_points(project.points)}")


if __name__ == "__main__":
    main()
