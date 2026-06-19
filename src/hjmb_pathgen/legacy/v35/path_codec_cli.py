# -*- coding: utf-8 -*-
"""HJMB V3.5 JSON/BIN codec and command-line planner."""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import zlib
from pathlib import Path
from typing import List, Optional

from .path_models import (
    ACTION_MODE_CODES,
    ACTION_MODE_NAMES_BY_CODE,
    ACTION_MODE_STOP_AND_WAIT,
    ACTIONS,
    MAX_ACTIONS,
    MAX_ARRIVALS,
    MAX_NODES,
    MAX_TRAJ_ID,
    PATH_MODE_FIXED_8,
    ParsedTrajectoryV35,
    PathProject,
    PlanResult,
    ResolvedMechanicalAction,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_END,
    TRAJ_FLAG_START,
    TRAJ_FLAG_WAYPOINT,
    TrajectoryHeaderV35,
    TrajectoryNode,
    VALID_TRAJ_FLAGS_MASK,
)
from .trajectory_planner import plan_project, validate_resolved_actions

MAGIC = b"HJMB"
VERSION = 35
HEADER_SIZE = 64
NODE_SIZE = 16
ACTION_SIZE = 22
HEADER_FMT = "<4sBBBBHHHHHHBBHIIIIIIHHHHHHHH"
NODE_FMT = "<HhhhhhhBB"
ACTION_FMT = "<BBBBHHHHHHHHH"
CRC32_OFFSET = 24

FILE_FLAG_SPATIAL_TRAJECTORY = 0x0001
FILE_FLAG_WORLD_VELOCITY = 0x0002
FILE_FLAG_FIXED_DIRECT_START = 0x0004
FILE_FLAG_ARRIVAL_ALWAYS_STOP = 0x0008
FILE_FLAG_ACTION_STATUS_REQUIRED = 0x0010
FILE_FLAG_FIXED_8_SOURCE = 0x0020
FILE_FLAG_AUTO_ACTION_START = 0x0040
REQUIRED_FILE_FLAGS = (
    FILE_FLAG_SPATIAL_TRAJECTORY
    | FILE_FLAG_WORLD_VELOCITY
    | FILE_FLAG_FIXED_DIRECT_START
    | FILE_FLAG_ARRIVAL_ALWAYS_STOP
    | FILE_FLAG_ACTION_STATUS_REQUIRED
    | FILE_FLAG_AUTO_ACTION_START
)
VALID_FILE_FLAGS = REQUIRED_FILE_FLAGS | FILE_FLAG_FIXED_8_SOURCE

assert struct.calcsize(HEADER_FMT) == HEADER_SIZE
assert struct.calcsize(NODE_FMT) == NODE_SIZE
assert struct.calcsize(ACTION_FMT) == ACTION_SIZE


def bin_path_traj_id(path: Path) -> int:
    match = re.fullmatch(r"P(\d{4})\.BIN", path.name, re.IGNORECASE)
    if match is None:
        raise ValueError("BIN 文件名必须为 P0000.BIN ~ P0359.BIN")
    traj_id = int(match.group(1))
    if not 0 <= traj_id <= MAX_TRAJ_ID:
        raise ValueError("BIN 文件名编号必须在 P0000.BIN ~ P0359.BIN 范围内")
    return traj_id


def load_project_dict(data: dict) -> PathProject:
    return PathProject.from_dict(data)


def load_project_json(path: Path) -> PathProject:
    return load_project_dict(json.loads(path.read_text(encoding="utf-8")))


def project_to_dict(project: PathProject) -> dict:
    return project.to_config_dict()


def save_project_json(project: PathProject, path: Path) -> None:
    path.write_text(
        json.dumps(project.to_config_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def arrival_count_from_nodes(nodes: List[TrajectoryNode]) -> int:
    return sum(bool(node.flags & TRAJ_FLAG_ARRIVAL) for node in nodes)


def _check_int_range(value: int, lower: int, upper: int, field_name: str) -> int:
    if not lower <= value <= upper:
        raise ValueError(f"{field_name}={value} 超出 {lower}~{upper}")
    return value


def _quantize_node(node: TrajectoryNode, index: int) -> tuple:
    s_mm = _check_int_range(int(round(node.s_mm)), 0, 0xFFFF, f"node[{index}].s_mm")
    x_mm = _check_int_range(int(round(node.x_mm)), -32768, 32767, f"node[{index}].x_mm")
    y_mm = _check_int_range(int(round(node.y_mm)), -32768, 32767, f"node[{index}].y_mm")
    yaw_ddeg = _check_int_range(
        int(round(math.degrees(node.yaw_rad) * 10.0)),
        -32768,
        32767,
        f"node[{index}].yaw_ddeg",
    )
    vx_mmps = _check_int_range(
        int(round(node.vx_mmps)), -32768, 32767, f"node[{index}].vx_mmps"
    )
    vy_mmps = _check_int_range(
        int(round(node.vy_mmps)), -32768, 32767, f"node[{index}].vy_mmps"
    )
    wz_ddegps = _check_int_range(
        int(round(math.degrees(node.wz_radps) * 10.0)),
        -32768,
        32767,
        f"node[{index}].wz_ddegps",
    )
    _check_int_range(node.arrival_id, 0, 0xFF, f"node[{index}].arrival_id")
    _check_int_range(node.flags, 0, 0xFF, f"node[{index}].flags")
    return (
        s_mm,
        x_mm,
        y_mm,
        yaw_ddeg,
        vx_mmps,
        vy_mmps,
        wz_ddegps,
        node.arrival_id,
        node.flags,
    )


def _pack_action(action: ResolvedMechanicalAction, index: int) -> bytes:
    values = (
        action.action_seq,
        action.action,
        ACTION_MODE_CODES[action.mode],
        action.arrival_id,
        action.timeout_ms,
        action.post_wait_ms,
        action.check_start_s_mm,
        action.accel_limit_mmps2,
        action.beta_limit_ddegps2,
        action.wz_limit_ddegps,
        action.speed_limit_mmps,
        action.stable_time_ms,
        0,
    )
    for field_index, value in enumerate(values[:4]):
        _check_int_range(value, 0, 0xFF, f"action[{index}].byte[{field_index}]")
    for field_index, value in enumerate(values[4:-1], start=4):
        _check_int_range(value, 0, 0xFFFF, f"action[{index}].u16[{field_index}]")
    return struct.pack(ACTION_FMT, *values)


def _integrate_quantized_time(nodes: List[TrajectoryNode]) -> int:
    time_ms = 0.0
    for previous, current in zip(nodes[:-1], nodes[1:]):
        ds = current.s_mm - previous.s_mm
        denominator = previous.speed_mmps + current.speed_mmps
        if ds > 0 and denominator <= 1e-6:
            raise ValueError(
                f"s={previous.s_mm:.0f}~{current.s_mm:.0f} mm 两端速度均为 0"
            )
        if ds > 0:
            time_ms += 2000.0 * ds / denominator
    return int(round(time_ms))


class PathCodec:
    """Strict V3.5 codec. Earlier HJMB formats are deliberately rejected."""

    @staticmethod
    def build_bin(project: PathProject, plan: Optional[PlanResult] = None) -> bytes:
        if plan is None:
            plan = plan_project(project)
        nodes = plan.nodes
        actions = plan.actions
        if not 2 <= len(nodes) <= MAX_NODES:
            raise ValueError(f"node_count 必须为 2~{MAX_NODES}")
        if len(actions) > MAX_ACTIONS:
            raise ValueError(f"action_count 不能超过 {MAX_ACTIONS}")
        arrival_count = arrival_count_from_nodes(nodes)
        if not 1 <= arrival_count <= MAX_ARRIVALS:
            raise ValueError(f"arrival_count 必须为 1~{MAX_ARRIVALS}")

        file_flags = REQUIRED_FILE_FLAGS
        if project.path_mode == PATH_MODE_FIXED_8:
            file_flags |= FILE_FLAG_FIXED_8_SOURCE

        node_offset = HEADER_SIZE
        action_offset = HEADER_SIZE + len(nodes) * NODE_SIZE
        total_length_mm = _check_int_range(
            int(round(plan.summary.total_length_mm)),
            0,
            0xFFFF,
            "Header.total_length_mm",
        )
        header = struct.pack(
            HEADER_FMT,
            MAGIC,
            VERSION,
            HEADER_SIZE,
            NODE_SIZE,
            ACTION_SIZE,
            project.traj_id,
            file_flags,
            project.field.width_mm,
            project.field.height_mm,
            project.planner.nominal_spacing_mm,
            len(nodes),
            len(actions),
            arrival_count,
            0,
            0,
            node_offset,
            action_offset,
            total_length_mm,
            plan.summary.formal_time_ms,
            0,
            project.start_check.position_tolerance_mm,
            project.start_check.yaw_tolerance_ddeg,
            project.start_check.stable_time_ms,
            project.arrival_check.position_tolerance_mm,
            project.arrival_check.yaw_tolerance_ddeg,
            project.arrival_check.speed_tolerance_mmps,
            project.arrival_check.wz_tolerance_ddegps,
            project.arrival_check.stable_time_ms,
        )
        payload = bytearray()
        previous_s = -1
        for index, node in enumerate(nodes):
            values = _quantize_node(node, index)
            if index > 0 and values[0] <= previous_s:
                raise ValueError(
                    f"量化后 node[{index}].s_mm={values[0]} 未严格递增；请减少节点密度"
                )
            previous_s = values[0]
            payload.extend(struct.pack(NODE_FMT, *values))
        for index, action in enumerate(actions):
            payload.extend(_pack_action(action, index))

        data = bytearray(header + payload)
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        crc = zlib.crc32(data) & 0xFFFFFFFF
        data[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
        return bytes(data)

    @staticmethod
    def parse_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> ParsedTrajectoryV35:
        if len(data) < HEADER_SIZE:
            raise ValueError(
                f"文件太短：{len(data)} 字节，无法读取 {HEADER_SIZE} 字节 V3.5 Header"
            )
        values = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        (
            magic,
            version,
            header_size,
            node_size,
            action_size,
            traj_id,
            file_flags,
            field_width_mm,
            field_height_mm,
            nominal_spacing_mm,
            node_count,
            action_count,
            arrival_count,
            reserved0,
            file_crc32,
            node_offset,
            action_offset,
            total_length_mm,
            planned_motion_time_ms,
            reserved1,
            start_pos_tolerance_mm,
            start_yaw_tolerance_ddeg,
            start_stable_time_ms,
            arrival_pos_tolerance_mm,
            arrival_yaw_tolerance_ddeg,
            arrival_speed_tolerance_mmps,
            arrival_wz_tolerance_ddegps,
            arrival_stable_time_ms,
        ) = values
        if magic != MAGIC:
            raise ValueError(f"Header.magic={magic!r}，期望 {MAGIC!r}")
        if version != VERSION:
            raise ValueError(
                f"不兼容的 HJMB BIN version={version}；仅支持 V3.5({VERSION})"
            )
        if header_size != HEADER_SIZE:
            raise ValueError(f"Header.header_size={header_size}，期望 {HEADER_SIZE}")
        if node_size != NODE_SIZE:
            raise ValueError(f"Header.node_size={node_size}，期望 {NODE_SIZE}")
        if action_size != ACTION_SIZE:
            raise ValueError(f"Header.action_size={action_size}，期望 {ACTION_SIZE}")
        if not 0 <= traj_id <= MAX_TRAJ_ID:
            raise ValueError(f"Header.traj_id={traj_id}，应为 0~{MAX_TRAJ_ID}")
        if expected_traj_id is not None and traj_id != expected_traj_id:
            raise ValueError(
                f"Header.traj_id={traj_id}，但文件名要求 traj_id={expected_traj_id}"
            )
        if not 2 <= node_count <= MAX_NODES:
            raise ValueError(f"Header.node_count={node_count}，应为 2~{MAX_NODES}")
        if not 0 <= action_count <= MAX_ACTIONS:
            raise ValueError(f"Header.action_count={action_count}，应为 0~{MAX_ACTIONS}")
        if not 1 <= arrival_count <= MAX_ARRIVALS:
            raise ValueError(f"Header.arrival_count={arrival_count}，应为 1~{MAX_ARRIVALS}")
        if field_width_mm != 4000 or field_height_mm != 2000:
            raise ValueError("Header.field_width_mm/field_height_mm 必须为 4000/2000")
        if not 1 <= nominal_spacing_mm <= 50:
            raise ValueError("Header.nominal_spacing_mm 必须为 1~50")
        if reserved0 != 0 or reserved1 != 0:
            raise ValueError("Header.reserved0/reserved1 必须全部为 0")
        if file_flags & REQUIRED_FILE_FLAGS != REQUIRED_FILE_FLAGS:
            raise ValueError(
                f"Header.flags=0x{file_flags:04X} 缺少 V3.5 必需标志"
            )
        if file_flags & ~VALID_FILE_FLAGS:
            raise ValueError(f"Header.flags 含未定义位 0x{file_flags & ~VALID_FILE_FLAGS:04X}")
        if node_offset != HEADER_SIZE:
            raise ValueError(f"Header.node_offset={node_offset}，期望 {HEADER_SIZE}")
        expected_action_offset = HEADER_SIZE + node_count * NODE_SIZE
        if action_offset != expected_action_offset:
            raise ValueError(
                f"Header.action_offset={action_offset}，期望 {expected_action_offset}"
            )
        expected_size = action_offset + action_count * ACTION_SIZE
        if len(data) != expected_size:
            raise ValueError(f"文件大小={len(data)}，按 Header 应为 {expected_size}")

        crc_data = bytearray(data)
        crc_data[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        calculated_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
        if calculated_crc != file_crc32:
            raise ValueError(
                f"CRC32 校验失败：文件为 0x{file_crc32:08X}，"
                f"计算为 0x{calculated_crc:08X}"
            )

        nodes: List[TrajectoryNode] = []
        offset = node_offset
        previous_s = -1
        start_count = 0
        end_count = 0
        parsed_arrival_ids: List[int] = []
        for index in range(node_count):
            (
                s_mm,
                x_mm,
                y_mm,
                yaw_ddeg,
                vx_mmps,
                vy_mmps,
                wz_ddegps,
                arrival_id,
                flags,
            ) = struct.unpack(NODE_FMT, data[offset : offset + NODE_SIZE])
            if index == 0 and s_mm != 0:
                raise ValueError("首节点 s_mm 必须为 0")
            if index > 0 and s_mm <= previous_s:
                raise ValueError(f"node[{index}].s_mm={s_mm} 未严格递增")
            previous_s = s_mm
            if flags & ~VALID_TRAJ_FLAGS_MASK:
                raise ValueError(f"node[{index}].flags 含 V3.5 保留位")
            if flags & TRAJ_FLAG_START:
                start_count += 1
                if index != 0:
                    raise ValueError("START 只能出现在首节点")
                if flags & (TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_WAYPOINT | TRAJ_FLAG_END):
                    raise ValueError("START 节点不能同时设置 ARRIVAL/WAYPOINT/END")
                if arrival_id != 0xFF:
                    raise ValueError("START 节点 arrival_id 必须为 0xFF")
                if any((vx_mmps, vy_mmps, wz_ddegps)):
                    raise ValueError("START 节点 vx/vy/wz 必须为 0")
            if flags & TRAJ_FLAG_ARRIVAL:
                parsed_arrival_ids.append(arrival_id)
                if arrival_id == 0xFF:
                    raise ValueError(f"ARRIVAL node[{index}] arrival_id 不能为 0xFF")
                if any((vx_mmps, vy_mmps, wz_ddegps)):
                    raise ValueError(f"ARRIVAL node[{index}] vx/vy/wz 必须为 0")
            elif arrival_id != 0xFF:
                raise ValueError(f"非 ARRIVAL node[{index}] arrival_id 必须为 0xFF")
            if flags & TRAJ_FLAG_END:
                end_count += 1
                if index != node_count - 1 or not flags & TRAJ_FLAG_ARRIVAL:
                    raise ValueError("END 只能出现在最后一个 ARRIVAL")
            speed = math.hypot(vx_mmps, vy_mmps)
            nodes.append(
                TrajectoryNode(
                    s_mm=float(s_mm),
                    x_mm=float(x_mm),
                    y_mm=float(y_mm),
                    yaw_rad=math.radians(yaw_ddeg / 10.0),
                    vx_mmps=float(vx_mmps),
                    vy_mmps=float(vy_mmps),
                    wz_radps=math.radians(wz_ddegps / 10.0),
                    arrival_id=arrival_id,
                    flags=flags,
                    speed_mmps=speed,
                )
            )
            offset += NODE_SIZE

        if start_count != 1:
            raise ValueError("START 必须全文件唯一")
        if not nodes[0].flags & TRAJ_FLAG_START:
            raise ValueError("首节点必须设置 START")
        if nodes[-1].flags & (TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END) != (
            TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END
        ):
            raise ValueError("末节点必须设置 ARRIVAL|END")
        if end_count != 1:
            raise ValueError("END 必须全文件唯一")
        if parsed_arrival_ids != list(range(arrival_count)):
            raise ValueError(
                f"arrival_id 必须按路径连续为 {list(range(arrival_count))}，"
                f"当前为 {parsed_arrival_ids}"
            )
        if nodes[-1].s_mm != total_length_mm:
            raise ValueError(
                f"末节点 s_mm={nodes[-1].s_mm:.0f} 与 total_length_mm={total_length_mm} 不一致"
            )
        integrated_time_ms = _integrate_quantized_time(nodes)
        if abs(integrated_time_ms - planned_motion_time_ms) > 25:
            raise ValueError(
                f"Header.planned_motion_time_ms={planned_motion_time_ms} 与量化节点积分 "
                f"{integrated_time_ms} ms 不一致"
            )

        actions: List[ResolvedMechanicalAction] = []
        for index in range(action_count):
            action_values = struct.unpack(
                ACTION_FMT, data[offset : offset + ACTION_SIZE]
            )
            (
                action_seq,
                action,
                mode_code,
                arrival_id,
                timeout_ms,
                post_wait_ms,
                check_start_s_mm,
                accel_limit_mmps2,
                beta_limit_ddegps2,
                wz_limit_ddegps,
                speed_limit_mmps,
                stable_time_ms,
                reserved,
            ) = action_values
            if reserved != 0:
                raise ValueError(f"action[{index}].reserved={reserved}，必须为 0")
            mode = ACTION_MODE_NAMES_BY_CODE.get(mode_code)
            if mode is None:
                raise ValueError(f"action[{index}].mode={mode_code} 非法")
            actions.append(
                ResolvedMechanicalAction(
                    action_seq=action_seq,
                    action=action,
                    mode=mode,
                    arrival_id=arrival_id,
                    timeout_ms=timeout_ms,
                    post_wait_ms=post_wait_ms,
                    check_start_s_mm=check_start_s_mm,
                    accel_limit_mmps2=accel_limit_mmps2,
                    beta_limit_ddegps2=beta_limit_ddegps2,
                    wz_limit_ddegps=wz_limit_ddegps,
                    speed_limit_mmps=speed_limit_mmps,
                    stable_time_ms=stable_time_ms,
                )
            )
            offset += ACTION_SIZE

        action_errors = validate_resolved_actions(actions, nodes[-1].s_mm, arrival_count)
        if action_errors:
            raise ValueError("V3.5 机械动作校验失败：\n" + "\n".join(action_errors))

        header = TrajectoryHeaderV35(
            traj_id=traj_id,
            flags=file_flags,
            field_width_mm=field_width_mm,
            field_height_mm=field_height_mm,
            nominal_spacing_mm=nominal_spacing_mm,
            node_count=node_count,
            action_count=action_count,
            arrival_count=arrival_count,
            file_crc32=file_crc32,
            node_offset=node_offset,
            action_offset=action_offset,
            total_length_mm=total_length_mm,
            planned_motion_time_ms=planned_motion_time_ms,
            start_pos_tolerance_mm=start_pos_tolerance_mm,
            start_yaw_tolerance_ddeg=start_yaw_tolerance_ddeg,
            start_stable_time_ms=start_stable_time_ms,
            arrival_pos_tolerance_mm=arrival_pos_tolerance_mm,
            arrival_yaw_tolerance_ddeg=arrival_yaw_tolerance_ddeg,
            arrival_speed_tolerance_mmps=arrival_speed_tolerance_mmps,
            arrival_wz_tolerance_ddegps=arrival_wz_tolerance_ddegps,
            arrival_stable_time_ms=arrival_stable_time_ms,
        )
        return ParsedTrajectoryV35(header=header, nodes=nodes, actions=actions)

    @staticmethod
    def load_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> ParsedTrajectoryV35:
        if len(data) < 5:
            raise ValueError("文件太短，无法识别 HJMB 版本")
        if data[4] != VERSION:
            raise ValueError(
                f"不兼容的 HJMB BIN version={data[4]}，V3.5 不兼容旧 BIN"
            )
        return PathCodec.parse_bin(data, expected_traj_id)


def json_to_bin(
    json_path: Path,
    bin_path: Path,
) -> tuple[PathProject, PlanResult]:
    project = load_project_json(json_path)
    expected_traj_id = bin_path_traj_id(bin_path)
    if expected_traj_id != project.traj_id:
        raise ValueError(
            f"BIN 文件名编号必须与 traj_id 一致，应为 P{project.traj_id:04d}.BIN"
        )
    plan = plan_project(project)
    data = PathCodec.build_bin(project, plan)
    bin_path.write_bytes(data)
    PathCodec.parse_bin(bin_path.read_bytes(), expected_traj_id=project.traj_id)
    return project, plan


def _print_summary(project: PathProject, plan: PlanResult) -> None:
    summary = plan.summary
    print(f"traj_id={project.traj_id}")
    print(f"path_mode={project.path_mode}")
    print(f"edit_point_count={len(project.points)}")
    print(f"node_count={len(plan.nodes)}")
    print(f"arrival_count={arrival_count_from_nodes(plan.nodes)}")
    print(f"action_count={len(plan.actions)}")
    modes = ",".join(action.mode for action in plan.actions) or "-"
    print(f"action_modes={modes}")
    auto_starts = [
        f"{action.action_seq}:{action.check_start_s_mm}:{action.execution_hint}"
        for action in plan.actions
        if action.check_start_s_mm != 0xFFFF
    ]
    print(f"auto_check_start_s_mm={','.join(auto_starts) if auto_starts else '-'}")
    locks = [
        f"{lock.arrival_id}->{lock.departure_action_seq}"
        for lock in plan.departure_locks
    ]
    print(f"departure_locks={','.join(locks) if locks else '-'}")
    print(f"start_check={project.start_check.position_tolerance_mm},"
          f"{project.start_check.yaw_tolerance_ddeg},"
          f"{project.start_check.stable_time_ms}")
    print(f"arrival_check={project.arrival_check.position_tolerance_mm},"
          f"{project.arrival_check.yaw_tolerance_ddeg},"
          f"{project.arrival_check.speed_tolerance_mmps},"
          f"{project.arrival_check.wz_tolerance_ddegps},"
          f"{project.arrival_check.stable_time_ms}")
    print(f"total_length_mm={summary.total_length_mm:.1f}")
    print(f"planned_motion_time_ms={summary.formal_time_ms}")
    print(f"mechanical_wait_time_ms_estimate={summary.mechanical_wait_time_ms}")
    print(f"estimated_total_time_ms={summary.estimated_total_time_ms}")
    print(f"max_speed_mmps={summary.max_speed_mmps:.1f}")
    print(f"max_a_total_mmps2={summary.max_a_total_mmps2:.1f}")
    print(f"max_a_n_mmps2={summary.max_a_n_mmps2:.1f}")
    print(f"max_wz_radps={summary.max_wz_radps:.4f}")
    print(f"max_beta_radps2={summary.max_beta_radps2:.4f}")
    print(f"max_wheel_rpm={summary.max_wheel_rpm:.2f}")
    print(f"max_wheel_rpm_s_mm={summary.max_wheel_rpm_s_mm:.1f}")
    for warning in plan.warnings:
        print(f"warning={warning}")


def main() -> None:
    parser = argparse.ArgumentParser(description="HJMB V3.5 空间轨迹生成/校验工具")
    subparsers = parser.add_subparsers(dest="cmd", required=True)
    build_parser = subparsers.add_parser("build", help="V3.5 JSON 规划并生成 BIN")
    build_parser.add_argument("json")
    build_parser.add_argument("bin")
    check_parser = subparsers.add_parser("check", help="严格校验 V3.5 BIN")
    check_parser.add_argument("bin")
    summary_parser = subparsers.add_parser("summary", help="规划并输出统计摘要")
    summary_parser.add_argument("json")
    plan_parser = subparsers.add_parser("plan", help="执行完整规划和验证，不写 BIN")
    plan_parser.add_argument("json")
    args = parser.parse_args()

    if args.cmd == "build":
        project, plan = json_to_bin(Path(args.json), Path(args.bin))
        _print_summary(project, plan)
        print(f"OK: wrote {args.bin}")
        return
    if args.cmd == "check":
        bin_path = Path(args.bin)
        parsed = PathCodec.parse_bin(
            bin_path.read_bytes(),
            expected_traj_id=bin_path_traj_id(bin_path),
        )
        header = parsed.header
        print(f"traj_id={header.traj_id}")
        print(f"version={VERSION}")
        print(f"node_count={header.node_count}")
        print(f"arrival_count={header.arrival_count}")
        print(f"action_count={header.action_count}")
        print(f"planned_motion_time_ms={header.planned_motion_time_ms}")
        print(f"CRC32=0x{header.file_crc32:08X}")
        print("OK: V3.5 BIN validation passed")
        return
    project = load_project_json(Path(args.json))
    plan = plan_project(project)
    _print_summary(project, plan)
    if args.cmd == "plan":
        data = PathCodec.build_bin(project, plan)
        PathCodec.parse_bin(data, expected_traj_id=project.traj_id)
        print("OK: V3.5 plan validation passed")


if __name__ == "__main__":
    main()
