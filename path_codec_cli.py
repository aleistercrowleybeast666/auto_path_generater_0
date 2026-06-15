# -*- coding: utf-8 -*-
"""HJMB V3.3 JSON/BIN codec and command-line planner."""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import zlib
from pathlib import Path
from typing import List, Optional

from path_models import (
    ACTION_GATE_ACCEL,
    ACTIONS,
    MAX_ACTIONS,
    MAX_GATES,
    MAX_NODES,
    MAX_TRAJ_ID,
    MechanicalAction,
    ParsedTrajectoryV33,
    PathProject,
    PlanResult,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_CUT_IN,
    TRAJ_FLAG_END,
    TRAJ_FLAG_GATE,
    TRAJ_FLAG_STOP,
    TrajectoryHeaderV33,
    TrajectoryNode,
)
from trajectory_planner import plan_project, validate_actions

MAGIC = b"HJMB"
VERSION = 33
HEADER_SIZE = 64
NODE_SIZE = 16
ACTION_SIZE = 20
HEADER_FMT = "<4sBBBBHHHHHHBBHIIIIIIHHHHHHHH"
NODE_FMT = "<HhhhhhhBB"
ACTION_FMT = "<BBBBHHHHHHHH"
CRC32_OFFSET = 24

FILE_FLAG_SPATIAL_TRAJECTORY = 0x0001
FILE_FLAG_WORLD_VELOCITY = 0x0002
FILE_FLAG_ACCEL_GATE_USED = 0x0004
FILE_FLAG_LIDAR_CUT_IN = 0x0008
FILE_FLAG_ARRIVAL_YAW_ANCHORS = 0x0010
REQUIRED_FILE_FLAGS = (
    FILE_FLAG_SPATIAL_TRAJECTORY
    | FILE_FLAG_WORLD_VELOCITY
    | FILE_FLAG_LIDAR_CUT_IN
    | FILE_FLAG_ARRIVAL_YAW_ANCHORS
)
VALID_FILE_FLAGS = REQUIRED_FILE_FLAGS | FILE_FLAG_ACCEL_GATE_USED

APPROACH_FLAG_LIDAR_REQUIRED = 0x0001
APPROACH_FLAG_NO_STOP_AT_CUT_IN = 0x0002
APPROACH_FLAG_ALIGN_YAW_TO_CUT_IN = 0x0004
APPROACH_FLAG_ALLOW_FIRST_SEGMENT_CAPTURE = 0x0008
REQUIRED_APPROACH_FLAGS = (
    APPROACH_FLAG_LIDAR_REQUIRED | APPROACH_FLAG_NO_STOP_AT_CUT_IN
)
VALID_APPROACH_FLAGS = (
    REQUIRED_APPROACH_FLAGS
    | APPROACH_FLAG_ALIGN_YAW_TO_CUT_IN
    | APPROACH_FLAG_ALLOW_FIRST_SEGMENT_CAPTURE
)

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


def gate_count_from_nodes(nodes: List[TrajectoryNode]) -> int:
    return len(
        {
            node.gate_id
            for node in nodes
            if node.flags & TRAJ_FLAG_GATE and node.gate_id != 0xFF
        }
    )


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
    _check_int_range(node.gate_id, 0, 0xFF, f"node[{index}].gate_id")
    _check_int_range(node.flags, 0, 0xFF, f"node[{index}].flags")
    return (
        s_mm,
        x_mm,
        y_mm,
        yaw_ddeg,
        vx_mmps,
        vy_mmps,
        wz_ddegps,
        node.gate_id,
        node.flags,
    )


def _pack_action(action: MechanicalAction, index: int) -> bytes:
    values = (
        action.action_seq,
        action.action,
        action.unlock_gate_id,
        action.flags,
        action.timeout_ms,
        action.arm_s_mm,
        action.disarm_s_mm,
        action.accel_limit_mmps2,
        action.beta_limit_ddegps2,
        action.speed_limit_mmps,
        action.stable_time_ms,
        0,
    )
    for field_index, value in enumerate(values[:4]):
        _check_int_range(value, 0, 0xFF, f"action[{index}].byte[{field_index}]")
    for field_index, value in enumerate(values[4:-1], start=4):
        _check_int_range(value, 0, 0xFFFF, f"action[{index}].u16[{field_index}]")
    return struct.pack(ACTION_FMT, *values)


class PathCodec:
    """Strict V3.3 codec. Earlier HJMB formats are deliberately rejected."""

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
        gate_count = gate_count_from_nodes(nodes)
        if gate_count > MAX_GATES:
            raise ValueError(f"gate_count 不能超过 {MAX_GATES}")

        file_flags = REQUIRED_FILE_FLAGS
        if any(action.unlock_gate_id == ACTION_GATE_ACCEL for action in actions):
            file_flags |= FILE_FLAG_ACCEL_GATE_USED
        approach_flags = REQUIRED_APPROACH_FLAGS
        if project.cut_in.align_yaw:
            approach_flags |= APPROACH_FLAG_ALIGN_YAW_TO_CUT_IN
        if project.cut_in.allow_first_segment_capture:
            approach_flags |= APPROACH_FLAG_ALLOW_FIRST_SEGMENT_CAPTURE

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
            gate_count,
            0,
            0,
            node_offset,
            action_offset,
            total_length_mm,
            plan.summary.formal_time_ms,
            0,
            project.cut_in.capture_radius_mm,
            project.cut_in.target_speed_mmps,
            project.cut_in.approach_max_speed_mmps,
            project.cut_in.straight_length_mm,
            project.cut_in.yaw_tolerance_ddeg,
            project.cut_in.tangent_tolerance_ddeg,
            approach_flags,
            0,
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
    ) -> ParsedTrajectoryV33:
        if len(data) < HEADER_SIZE:
            raise ValueError(
                f"文件太短：{len(data)} 字节，无法读取 {HEADER_SIZE} 字节 V3.3 Header"
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
            gate_count,
            reserved0,
            file_crc32,
            node_offset,
            action_offset,
            total_length_mm,
            planned_time_ms,
            reserved1,
            capture_radius_mm,
            cut_in_speed_mmps,
            approach_max_speed_mmps,
            straight_length_mm,
            yaw_tolerance_ddeg,
            tangent_tolerance_ddeg,
            approach_flags,
            reserved2,
        ) = values
        if magic != MAGIC:
            raise ValueError(f"Header.magic={magic!r}，期望 {MAGIC!r}")
        if version != VERSION:
            raise ValueError(
                f"不兼容的 HJMB BIN version={version}；仅支持 V3.3({VERSION})"
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
        if not 0 <= gate_count <= MAX_GATES:
            raise ValueError(f"Header.gate_count={gate_count}，应为 0~{MAX_GATES}")
        if field_width_mm != 4000 or field_height_mm != 2000:
            raise ValueError("Header.field_width_mm/field_height_mm 必须为 4000/2000")
        if not 1 <= nominal_spacing_mm <= 50:
            raise ValueError("Header.nominal_spacing_mm 必须为 1~50")
        if reserved0 != 0 or reserved1 != 0 or reserved2 != 0:
            raise ValueError("Header.reserved0/reserved1/reserved2 必须全部为 0")
        if file_flags & REQUIRED_FILE_FLAGS != REQUIRED_FILE_FLAGS:
            raise ValueError(
                f"Header.flags=0x{file_flags:04X} 缺少 V3.3 必需标志"
            )
        if file_flags & ~VALID_FILE_FLAGS:
            raise ValueError(f"Header.flags 含未定义位 0x{file_flags & ~VALID_FILE_FLAGS:04X}")
        if approach_flags & REQUIRED_APPROACH_FLAGS != REQUIRED_APPROACH_FLAGS:
            raise ValueError("Header.approach_flags 缺少 LIDAR_REQUIRED/NO_STOP")
        if approach_flags & ~VALID_APPROACH_FLAGS:
            raise ValueError("Header.approach_flags 含未定义位")
        if capture_radius_mm == 0 or cut_in_speed_mmps == 0:
            raise ValueError("Header 切入捕获半径和切入速度必须大于 0")
        if approach_max_speed_mmps < cut_in_speed_mmps:
            raise ValueError("Header.approach_max_speed_mmps 小于 cut_in_speed_mmps")
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
        cut_in_count = 0
        end_count = 0
        gate_ids: List[int] = []
        for index in range(node_count):
            (
                s_mm,
                x_mm,
                y_mm,
                yaw_ddeg,
                vx_mmps,
                vy_mmps,
                wz_ddegps,
                gate_id,
                flags,
            ) = struct.unpack(NODE_FMT, data[offset : offset + NODE_SIZE])
            if index == 0 and s_mm != 0:
                raise ValueError("首节点 s_mm 必须为 0")
            if index > 0 and s_mm <= previous_s:
                raise ValueError(f"node[{index}].s_mm={s_mm} 未严格递增")
            previous_s = s_mm
            if gate_id == ACTION_GATE_ACCEL:
                raise ValueError(f"node[{index}].gate_id 禁止使用 0xFE")
            if flags & TRAJ_FLAG_GATE:
                if gate_id == 0xFF:
                    raise ValueError(f"node[{index}] 设置 GATE 但 gate_id=0xFF")
                gate_ids.append(gate_id)
            elif gate_id != 0xFF:
                raise ValueError(f"node[{index}] 未设置 GATE 但 gate_id={gate_id}")
            if flags & TRAJ_FLAG_STOP and any((vx_mmps, vy_mmps, wz_ddegps)):
                raise ValueError(f"STOP node[{index}] 的 vx/vy/wz 必须为 0")
            if flags & (TRAJ_FLAG_CUT_IN | TRAJ_FLAG_ARRIVAL) and wz_ddegps != 0:
                raise ValueError(
                    f"V3.3 yaw 锚点 node[{index}] 的 wz_ddegps 必须为 0"
                )
            cut_in_count += bool(flags & TRAJ_FLAG_CUT_IN)
            end_count += bool(flags & TRAJ_FLAG_END)
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
                    gate_id=gate_id,
                    flags=flags,
                    speed_mmps=speed,
                )
            )
            offset += NODE_SIZE

        if not nodes[0].flags & TRAJ_FLAG_CUT_IN:
            raise ValueError("首节点必须设置 CUT_IN")
        if nodes[0].flags & (TRAJ_FLAG_STOP | TRAJ_FLAG_GATE):
            raise ValueError("首节点 CUT_IN 禁止 STOP/GATE")
        if abs(nodes[0].speed_mmps - cut_in_speed_mmps) > 2.0:
            raise ValueError(
                f"首节点速度 {nodes[0].speed_mmps:.1f} 与 Header.cut_in_speed "
                f"{cut_in_speed_mmps} 不一致"
            )
        required_last = TRAJ_FLAG_ARRIVAL | TRAJ_FLAG_END | TRAJ_FLAG_STOP
        if nodes[-1].flags & required_last != required_last:
            raise ValueError("末节点必须设置 ARRIVAL|END|STOP")
        if cut_in_count != 1 or end_count != 1:
            raise ValueError("CUT_IN 和 END 必须各出现一次")
        if nodes[-1].s_mm != total_length_mm:
            raise ValueError(
                f"末节点 s_mm={nodes[-1].s_mm:.0f} 与 total_length_mm={total_length_mm} 不一致"
            )
        integrated_time_ms = 0.0
        for previous, current in zip(nodes[:-1], nodes[1:]):
            ds = current.s_mm - previous.s_mm
            denominator = previous.speed_mmps + current.speed_mmps
            if ds > 0 and denominator <= 1e-6:
                raise ValueError(
                    f"s={previous.s_mm:.0f}~{current.s_mm:.0f} mm 两端速度均为 0"
                )
            integrated_time_ms += 2000.0 * ds / denominator
        if abs(round(integrated_time_ms) - planned_time_ms) > 25:
            raise ValueError(
                f"Header.planned_time_ms={planned_time_ms} 与量化节点积分 "
                f"{round(integrated_time_ms)} ms 不一致"
            )
        unique_gate_ids = []
        for gate_id in gate_ids:
            if gate_id not in unique_gate_ids:
                unique_gate_ids.append(gate_id)
        if unique_gate_ids != list(range(gate_count)):
            raise ValueError(
                f"编号 Gate 必须按路径连续为 {list(range(gate_count))}，"
                f"当前为 {unique_gate_ids}"
            )

        actions: List[MechanicalAction] = []
        for index in range(action_count):
            action_values = struct.unpack(
                ACTION_FMT, data[offset : offset + ACTION_SIZE]
            )
            (
                action_seq,
                action,
                unlock_gate_id,
                flags,
                timeout_ms,
                arm_s_mm,
                disarm_s_mm,
                accel_limit_mmps2,
                beta_limit_ddegps2,
                speed_limit_mmps,
                stable_time_ms,
                reserved,
            ) = action_values
            if reserved != 0:
                raise ValueError(f"action[{index}].reserved={reserved}，必须为 0")
            actions.append(
                MechanicalAction(
                    action_seq=action_seq,
                    action=action,
                    unlock_gate_id=unlock_gate_id,
                    flags=flags,
                    timeout_ms=timeout_ms,
                    arm_s_mm=arm_s_mm,
                    disarm_s_mm=disarm_s_mm,
                    accel_limit_mmps2=accel_limit_mmps2,
                    beta_limit_ddegps2=beta_limit_ddegps2,
                    speed_limit_mmps=speed_limit_mmps,
                    stable_time_ms=stable_time_ms,
                )
            )
            offset += ACTION_SIZE

        dummy_project = PathProject(actions=actions)
        action_errors = validate_actions(dummy_project, nodes)
        if action_errors:
            raise ValueError("V3.3 动作/Gate 校验失败：\n" + "\n".join(action_errors))
        accel_flag_expected = any(
            action.unlock_gate_id == ACTION_GATE_ACCEL for action in actions
        )
        if accel_flag_expected != bool(file_flags & FILE_FLAG_ACCEL_GATE_USED):
            raise ValueError("FILE_FLAG_ACCEL_GATE_USED 与动作中的 0xFE Gate 不一致")

        header = TrajectoryHeaderV33(
            traj_id=traj_id,
            flags=file_flags,
            field_width_mm=field_width_mm,
            field_height_mm=field_height_mm,
            nominal_spacing_mm=nominal_spacing_mm,
            node_count=node_count,
            action_count=action_count,
            gate_count=gate_count,
            file_crc32=file_crc32,
            node_offset=node_offset,
            action_offset=action_offset,
            total_length_mm=total_length_mm,
            planned_time_ms=planned_time_ms,
            cut_in_capture_radius_mm=capture_radius_mm,
            cut_in_speed_mmps=cut_in_speed_mmps,
            approach_max_speed_mmps=approach_max_speed_mmps,
            cut_in_straight_length_mm=straight_length_mm,
            cut_in_yaw_tolerance_ddeg=yaw_tolerance_ddeg,
            cut_in_tangent_tolerance_ddeg=tangent_tolerance_ddeg,
            approach_flags=approach_flags,
        )
        return ParsedTrajectoryV33(header=header, nodes=nodes, actions=actions)

    @staticmethod
    def load_bin(
        data: bytes,
        expected_traj_id: Optional[int] = None,
    ) -> ParsedTrajectoryV33:
        if len(data) < 5:
            raise ValueError("文件太短，无法识别 HJMB 版本")
        if data[4] != VERSION:
            raise ValueError(
                f"不兼容的 HJMB BIN version={data[4]}，请使用旧版编辑器转换或重新绘制"
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
    print(f"edit_point_count={len(project.points)}")
    print(f"node_count={len(plan.nodes)}")
    print(f"action_count={len(plan.actions)}")
    print(f"gate_count={gate_count_from_nodes(plan.nodes)}")
    print(f"total_length_mm={summary.total_length_mm:.1f}")
    print(f"formal_time_ms={summary.formal_time_ms}")
    print(f"cut_in_preview_time_ms={summary.cut_in_preview_time_ms}")
    print(f"mechanical_wait_time_ms={summary.mechanical_wait_time_ms}")
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
    parser = argparse.ArgumentParser(description="HJMB V3.3 空间轨迹生成/校验工具")
    subparsers = parser.add_subparsers(dest="cmd", required=True)
    build_parser = subparsers.add_parser("build", help="V3.3 JSON 规划并生成 BIN")
    build_parser.add_argument("json")
    build_parser.add_argument("bin")
    check_parser = subparsers.add_parser("check", help="严格校验 V3.3 BIN")
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
        print(f"action_count={header.action_count}")
        print(f"gate_count={header.gate_count}")
        print(f"total_length_mm={header.total_length_mm}")
        print(f"planned_time_ms={header.planned_time_ms}")
        print(f"crc32=0x{header.file_crc32:08X}")
        print(f"max_feed_speed_mmps={max(node.speed_mmps for node in parsed.nodes):.1f}")
        for index, node in enumerate(parsed.nodes):
            if node.flags:
                print(
                    f"node[{index}]: s={node.s_mm:.0f}, flags=0x{node.flags:02X}, "
                    f"gate=0x{node.gate_id:02X}, v={node.speed_mmps:.1f}"
                )
        for action in parsed.actions:
            print(
                f"action[{action.action_seq}]: {ACTIONS.get(action.action, hex(action.action))}, "
                f"gate=0x{action.unlock_gate_id:02X}, flags=0x{action.flags:02X}, "
                f"arm={action.arm_s_mm}, disarm={action.disarm_s_mm}"
            )
        print("OK: V3.3 BIN validation passed")
        return

    project = load_project_json(Path(args.json))
    plan = plan_project(project)
    _print_summary(project, plan)
    if args.cmd == "plan":
        print("OK: planning and validation passed")


if __name__ == "__main__":
    main()
