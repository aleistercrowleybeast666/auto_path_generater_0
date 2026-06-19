"""Synthetic V4.0 case-to-trajectory assembly for Phase 2."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Any

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32
from hjmb_pathgen.codec.validators import validate_compiled_trajectory
from hjmb_pathgen.models.compiled import ActionV40, CompiledTrajectoryV40, HeaderV40, NodeV40, SegmentV40
from hjmb_pathgen.models.enums import (
    ActionCode,
    ActionMode,
    FinishMode,
    HeaderFlag,
    LegState,
    NodeFlag,
    RouteFamily,
    SegmentFlag,
    StorageMode,
)
from hjmb_pathgen.models.errors import CompileError, MissingDependencyError, StaleDependencyError
from hjmb_pathgen.models.leg import LegLibraryV40, LegV40
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.models.route_case import CaseManifestV40
from hjmb_pathgen.models.protocol import REQUIRED_HEADER_FLAGS

REUSABLE_LEG_STATES = {LegState.VALID, LegState.APPROVED, LegState.LOCKED}


@dataclass(frozen=True)
class CaseCompileRequest:
    case: CaseManifestV40
    leg_library: LegLibraryV40 | None = None
    project: ProjectV40 | None = None


@dataclass(frozen=True)
class _NodeMeta:
    node: NodeV40
    meta: dict[str, Any]


def compile_case_to_trajectory(request: CaseCompileRequest) -> CompiledTrajectoryV40:
    case = request.case
    legs_by_id = _available_legs(case, request.leg_library)
    referenced_legs = [_resolve_leg_ref(ref, legs_by_id) for ref in case.leg_refs]
    if not referenced_legs:
        raise CompileError(f"P{case.traj_id:04d} has no leg_refs; Phase 2 requires precompiled dense legs")

    assembled_nodes, segment_specs = _assemble_nodes_and_segments(case, referenced_legs)
    nodes, arrival_state_to_id = _renumber_arrivals_and_finish(assembled_nodes, case)
    segments = _make_segments(nodes, segment_specs)
    actions = _compile_actions(case, arrival_state_to_id, nodes[-1].s_mm)
    header = _make_header(case, request.project, segments)
    trajectory = CompiledTrajectoryV40(header=header, nodes=nodes, segments=segments, actions=actions).normalized()
    validate_compiled_trajectory(trajectory)
    return trajectory


def _available_legs(case: CaseManifestV40, library: LegLibraryV40 | None) -> dict[str, LegV40]:
    ref_ids = {str(ref.get("leg_id", "")) for ref in case.leg_refs}
    if case.storage_mode == StorageMode.EMBEDDED:
        if not case.embedded_legs:
            raise MissingDependencyError(f"P{case.traj_id:04d} is EMBEDDED but has no embedded_legs")
        legs = {LegV40.from_dict(item).leg_id: LegV40.from_dict(item) for item in case.embedded_legs}
        extra = sorted(set(legs) - ref_ids)
        if extra:
            raise CompileError(f"P{case.traj_id:04d} embedded_legs contains unreferenced legs: {extra}")
        return legs
    if case.storage_mode == StorageMode.REFERENCED:
        if library is None:
            raise MissingDependencyError(f"P{case.traj_id:04d} requires leg_library.json")
        return {leg.leg_id: leg for leg in library.legs}
    raise CompileError(f"unsupported storage_mode for P{case.traj_id:04d}: {case.storage_mode}")


def _resolve_leg_ref(ref: dict[str, Any], legs_by_id: dict[str, LegV40]) -> tuple[dict[str, Any], LegV40]:
    leg_id = str(ref.get("leg_id", ""))
    if not leg_id:
        raise CompileError("leg_ref.leg_id is required")
    leg = legs_by_id.get(leg_id)
    if leg is None:
        raise MissingDependencyError(f"missing referenced leg: {leg_id}")
    if leg.state not in REUSABLE_LEG_STATES:
        raise StaleDependencyError(f"leg {leg_id} is not reusable: {leg.state.value}")
    expected = _ref_expected_hash(ref)
    actual = _leg_hash32(leg)
    if expected is not None and expected != actual:
        raise StaleDependencyError(f"leg {leg_id} hash mismatch: actual=0x{actual:08X}, expected=0x{expected:08X}")
    return ref, leg


def _assemble_nodes_and_segments(
    case: CaseManifestV40,
    referenced_legs: list[tuple[dict[str, Any], LegV40]],
) -> tuple[list[_NodeMeta], list[dict[str, Any]]]:
    assembled: list[_NodeMeta] = []
    segment_specs: list[dict[str, Any]] = []
    global_s = 0

    for leg_index, (_ref, leg) in enumerate(referenced_legs):
        leg_nodes = [_node_from_leg_node(node) for node in leg.nodes]
        if len(leg_nodes) < 2:
            raise CompileError(f"leg {leg.leg_id} must contain at least two dense nodes")
        if leg_nodes[0].node.s_mm != 0:
            raise CompileError(f"leg {leg.leg_id} first local_s_mm must be 0")

        start_index = len(assembled)
        previous_local_s = leg_nodes[0].node.s_mm
        for node_index, node_meta in enumerate(leg_nodes):
            if leg_index > 0 and node_index == 0:
                _check_shared_boundary(assembled[-1].node, node_meta.node, leg.leg_id)
                start_index = len(assembled) - 1
                previous_local_s = node_meta.node.s_mm
                continue
            if leg_index == 0 and node_index == 0:
                node_global_s = 0
            else:
                delta = node_meta.node.s_mm - previous_local_s
                if delta < 0:
                    raise CompileError(f"leg {leg.leg_id} local_s_mm must be monotonic")
                global_s += delta
                node_global_s = global_s
            previous_local_s = node_meta.node.s_mm
            flags = node_meta.node.flags
            if len(assembled) == 0:
                flags |= int(NodeFlag.START | NodeFlag.EXACT_PASS)
            else:
                flags &= ~int(NodeFlag.START)
            flags &= ~int(NodeFlag.FINISH_ARM | NodeFlag.SAFE_END)
            assembled.append(_NodeMeta(node=replace(node_meta.node, s_mm=node_global_s, flags=flags), meta=node_meta.meta))

        end_index = len(assembled) - 1
        segment_specs.append(
            {
                "leg_id": leg.leg_id,
                "start_index": start_index,
                "end_index": end_index,
                "planned_time_ms": int(leg.analysis.get("planned_time_ms", 0)),
                "source_leg_hash32": _leg_hash32(leg),
            }
        )

    if not assembled:
        raise CompileError(f"P{case.traj_id:04d} produced no nodes")
    return assembled, segment_specs


def _check_shared_boundary(previous: NodeV40, current: NodeV40, leg_id: str) -> None:
    previous_pose = (previous.x_mm, previous.y_mm, previous.yaw_ddeg, previous.vx_mmps, previous.vy_mmps, previous.wz_ddegps)
    current_pose = (current.x_mm, current.y_mm, current.yaw_ddeg, current.vx_mmps, current.vy_mmps, current.wz_ddegps)
    if previous_pose != current_pose:
        raise CompileError(f"leg {leg_id} does not share the previous boundary node")


def _renumber_arrivals_and_finish(
    assembled_nodes: list[_NodeMeta],
    case: CaseManifestV40,
) -> tuple[tuple[NodeV40, ...], dict[str, int]]:
    arrival_state_to_id: dict[str, int] = {}
    case_arrival_states = [str(item.get("state_id", "")) for item in case.arrival_states]
    result: list[NodeV40] = []
    arrival_id = 0
    for index, node_meta in enumerate(assembled_nodes):
        flags = node_meta.node.flags
        if flags & int(NodeFlag.ARRIVAL):
            flags |= int(NodeFlag.EXACT_PASS)
            state_id = str(node_meta.meta.get("arrival_state_id") or node_meta.meta.get("state_id") or "")
            if not state_id and arrival_id < len(case_arrival_states):
                state_id = case_arrival_states[arrival_id]
            if state_id:
                arrival_state_to_id[state_id] = arrival_id
            result.append(replace(node_meta.node, flags=flags, arrival_id=arrival_id))
            arrival_id += 1
        else:
            result.append(replace(node_meta.node, arrival_id=0xFF))

    last = result[-1]
    if not (last.flags & int(NodeFlag.ARRIVAL)):
        raise CompileError("final formal node must be the last drop ARRIVAL")
    last_flags = (last.flags & ~int(NodeFlag.START | NodeFlag.SAFE_END)) | int(NodeFlag.FINISH_ARM | NodeFlag.EXACT_PASS)
    result[-1] = replace(last, flags=last_flags)
    return tuple(result), arrival_state_to_id


def _make_segments(nodes: tuple[NodeV40, ...], segment_specs: list[dict[str, Any]]) -> tuple[SegmentV40, ...]:
    segments: list[SegmentV40] = []
    for segment_id, spec in enumerate(segment_specs):
        start_index = int(spec["start_index"])
        end_index = int(spec["end_index"])
        start_node = nodes[start_index]
        end_node = nodes[end_index]
        segments.append(
            SegmentV40(
                segment_id=segment_id,
                start_node_index=start_index,
                end_node_index=end_index,
                start_s_mm=start_node.s_mm,
                end_s_mm=end_node.s_mm,
                start_arrival_id=start_node.arrival_id,
                end_arrival_id=end_node.arrival_id,
                flags=int(SegmentFlag.NORMAL | SegmentFlag.LIBRARY_REUSED),
                planned_time_ms=int(spec["planned_time_ms"]),
                source_leg_hash32=int(spec["source_leg_hash32"]),
            )
        )
    return tuple(segments)


def _compile_actions(case: CaseManifestV40, arrival_state_to_id: dict[str, int], total_length_mm: int) -> tuple[ActionV40, ...]:
    compiled_actions = list(case.actions.get("compiled", ()))
    actions: list[ActionV40] = []
    for index, item in enumerate(compiled_actions):
        mode = _enum_int(ActionMode, item.get("mode", ActionMode.STOP_AND_WAIT))
        arrival_id = item.get("arrival_id", 0xFF)
        arrival_state_id = item.get("arrival_state_id")
        if arrival_state_id is not None:
            state_key = str(arrival_state_id)
            if state_key not in arrival_state_to_id:
                raise CompileError(f"action {index} references unknown arrival_state_id: {state_key}")
            arrival_id = arrival_state_to_id[state_key]
        if mode == int(ActionMode.STOP_AND_WAIT) and arrival_id == 0xFF:
            raise CompileError(f"action {index} STOP_AND_WAIT requires arrival_id or arrival_state_id")
        if mode in (int(ActionMode.ASYNC), int(ActionMode.KINEMATIC)):
            arrival_id = 0xFF
        check_start_s_mm = int(item.get("check_start_s_mm", 0xFFFF if mode != int(ActionMode.KINEMATIC) else 0))
        if mode == int(ActionMode.KINEMATIC) and check_start_s_mm > total_length_mm:
            raise CompileError(f"action {index} KINEMATIC check_start_s_mm exceeds total length")
        actions.append(
            ActionV40(
                action_seq=index,
                action=_enum_int(ActionCode, item.get("action", ActionCode.NONE)),
                mode=mode,
                arrival_id=int(arrival_id),
                timeout_ms=int(item.get("timeout_ms", 1000)),
                post_wait_ms=int(item.get("post_wait_ms", 0)),
                check_start_s_mm=check_start_s_mm,
                accel_limit_mmps2=int(item.get("accel_limit_mmps2", 0)),
                beta_limit_ddegps2=int(item.get("beta_limit_ddegps2", 0)),
                wz_limit_ddegps=int(item.get("wz_limit_ddegps", 0)),
                speed_limit_mmps=int(item.get("speed_limit_mmps", 0)),
                stable_time_ms=int(item.get("stable_time_ms", 0)),
            )
        )
    return tuple(actions)


def _make_header(case: CaseManifestV40, project: ProjectV40 | None, segments: tuple[SegmentV40, ...]) -> HeaderV40:
    route_family = _enum_int(RouteFamily, case.selected_plan.get("route_family", RouteFamily.MANUAL_FREE))
    finish_source = dict(project.finish_policy if project is not None else {})
    finish_source.update(case.finish)
    flags = int(REQUIRED_HEADER_FLAGS)
    if case.review.get("detached_from_library") or case.review.get("manual_override"):
        flags |= int(HeaderFlag.MANUAL_OVERRIDE)
    project_hash = canonical_json_crc32(project.to_dict()) if project is not None else 0
    planned_motion_time_ms = sum(segment.planned_time_ms for segment in segments)
    planned_total_estimate_ms = int(case.estimates.get("planned_total_estimate_ms", 0)) or planned_motion_time_ms
    header_kwargs: dict[str, Any] = {
        "traj_id": case.traj_id,
        "bean_code": case.bean_code,
        "drop_code": case.drop_code,
        "route_family": route_family,
        "finish_mode": _finish_mode_value(finish_source.get("mode", FinishMode.AT_FINAL_DROP)),
        "flags": flags,
        "planned_motion_time_ms": planned_motion_time_ms,
        "planned_total_estimate_ms": planned_total_estimate_ms,
        "source_case_hash32": canonical_json_crc32(_case_compile_hash_dict(case)),
        "source_project_hash32": project_hash,
        "finish_axis": 0,
        "finish_direction": 0,
        "finish_line_mm": 0,
        "finish_envelope_margin_mm": 0,
        "finish_stable_time_ms": 0,
        "finish_brake_accel_mmps2": 0,
        "finish_max_runout_mm": 0,
        "finish_hard_timeout_ms": 0,
        "finish_signal_flags": int(finish_source.get("signal_flags", finish_source.get("finish_signal_flags", 0))),
    }
    if project is not None:
        header_kwargs.update(_project_check_fields(project))
    return HeaderV40(**header_kwargs)


def _project_check_fields(project: ProjectV40) -> dict[str, int]:
    start = project.start_check
    arrival = project.arrival_check
    return {
        "start_pos_tolerance_mm": int(start.get("position_tolerance_mm", 20)),
        "start_yaw_tolerance_ddeg": int(start.get("yaw_tolerance_ddeg", 50)),
        "start_stable_time_ms": int(start.get("stable_time_ms", 100)),
        "arrival_pos_tolerance_mm": int(arrival.get("position_tolerance_mm", 20)),
        "arrival_yaw_tolerance_ddeg": int(arrival.get("yaw_tolerance_ddeg", 50)),
        "arrival_speed_tolerance_mmps": int(arrival.get("speed_tolerance_mmps", 10)),
        "arrival_wz_tolerance_ddegps": int(arrival.get("wz_tolerance_ddegps", 10)),
        "arrival_stable_time_ms": int(arrival.get("stable_time_ms", 100)),
    }


def _case_compile_hash_dict(case: CaseManifestV40) -> dict[str, Any]:
    data = case.to_dict()
    data.pop("storage_mode", None)
    data.pop("embedded_legs", None)
    return data


def _node_from_leg_node(data: dict[str, Any]) -> _NodeMeta:
    local_s = int(data.get("local_s_mm", data.get("s_mm", 0)))
    flags = _node_flags(data.get("flags", 0))
    node = NodeV40(
        s_mm=local_s,
        x_mm=int(data["x_mm"]),
        y_mm=int(data["y_mm"]),
        yaw_ddeg=int(data.get("yaw_ddeg", 0)),
        vx_mmps=int(data.get("vx_mmps", 0)),
        vy_mmps=int(data.get("vy_mmps", 0)),
        wz_ddegps=int(data.get("wz_ddegps", 0)),
        arrival_id=0xFF,
        flags=flags,
    )
    return _NodeMeta(node=node, meta=dict(data))


def _node_flags(value: object) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, str):
        return int(NodeFlag[value])
    flags = 0
    for item in value or ():
        if isinstance(item, int):
            flags |= int(item)
        else:
            flags |= int(NodeFlag[str(item)])
    return flags


def _ref_expected_hash(ref: dict[str, Any]) -> int | None:
    for key in ("expected_leg_hash32", "expected_hash32", "source_leg_hash32", "leg_hash32"):
        if key in ref:
            return _hash_value(ref[key])
    return None


def _leg_hash32(leg: LegV40) -> int:
    for key in ("self_hash32", "hash32", "leg_hash32"):
        if key in leg.hashes:
            return _hash_value(leg.hashes[key])
    data = leg.to_dict()
    data["hashes"] = {}
    return canonical_json_crc32(data)


def _hash_value(value: object) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        return value & 0xFFFFFFFF
    if isinstance(value, str):
        text = value.strip()
        if text.lower().startswith("0x"):
            return int(text, 16) & 0xFFFFFFFF
        return int(text, 16 if any(ch in "abcdefABCDEF" for ch in text) else 10) & 0xFFFFFFFF
    raise CompileError(f"unsupported hash value: {value!r}")


def _enum_int(enum_type: type[IntEnum], value: object) -> int:
    if isinstance(value, enum_type):
        return int(value)
    if isinstance(value, int) and not isinstance(value, bool):
        return int(enum_type(value))
    if isinstance(value, str):
        name = value
        for prefix in ("ROUTE_FAMILY_", "FINISH_MODE_", "ACTION_MODE_", "PATH_ACT_"):
            if name.startswith(prefix):
                name = name[len(prefix) :]
        return int(enum_type[name])
    raise CompileError(f"cannot convert enum value {value!r} for {enum_type.__name__}")


def _finish_mode_value(value: object) -> int:
    if isinstance(value, str) and value in {"AT_SAFE_END", "HALF_PLANE_THEN_SAFE_BRAKE"}:
        raise CompileError(f"legacy finish mode is not allowed in formal V4.0 output: {value}")
    mode = _enum_int(FinishMode, value)
    if mode != int(FinishMode.AT_FINAL_DROP):
        raise CompileError(f"formal V4.0 supports only AT_FINAL_DROP finish mode: {value!r}")
    return mode
