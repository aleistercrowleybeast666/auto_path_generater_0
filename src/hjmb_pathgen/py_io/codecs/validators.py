"""Protocol-level V4.0 trajectory validators."""

from __future__ import annotations

from hjmb_pathgen.py_domain.compiled import CompiledTrajectoryV40
from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode, FinishMode, HeaderFlag, NodeFlag, RouteFamily, SegmentFlag
from hjmb_pathgen.py_domain.errors import BinaryLayoutError


def _fail(field_path: str, message: str, *, actual: object | None = None, expected: object | None = None) -> None:
    raise BinaryLayoutError("V40 Trajectory", field_path, message, actual=actual, expected=expected)


def _is_zero_motion(node: object) -> bool:
    return node.vx_mmps == 0 and node.vy_mmps == 0 and node.wz_ddegps == 0


def _limits_tuple(action: object) -> tuple[int, int, int, int, int]:
    return (
        action.accel_limit_mmps2,
        action.beta_limit_ddegps2,
        action.wz_limit_ddegps,
        action.speed_limit_mmps,
        action.stable_time_ms,
    )


def validate_compiled_trajectory(trajectory: CompiledTrajectoryV40) -> None:
    """Validate protocol semantics that are stricter than field packing.

    This intentionally does not perform Phase 4+ dynamic, wheel-rpm, topology,
    or collision validation. It covers the Phase 2 structure and action rules.
    """

    trajectory.validate()
    header = trajectory.header
    expected = header.with_layout_counts(
        len(trajectory.nodes),
        len(trajectory.segments),
        len(trajectory.actions),
        sum(1 for node in trajectory.nodes if node.flags & int(NodeFlag.ARRIVAL)),
        total_length_mm=trajectory.nodes[-1].s_mm if trajectory.nodes else 0,
    )
    for field_name in (
        "node_count",
        "segment_count",
        "action_count",
        "arrival_count",
        "node_offset",
        "segment_offset",
        "action_offset",
        "total_length_mm",
    ):
        actual = getattr(header, field_name)
        expected_value = getattr(expected, field_name)
        if actual != expected_value:
            _fail(f"header.{field_name}", "derived header field mismatch", actual=actual, expected=expected_value)
    if header.finish_mode != int(FinishMode.AT_FINAL_DROP):
        _fail("header.finish_mode", "formal V4.0 supports only AT_FINAL_DROP", actual=header.finish_mode, expected=int(FinishMode.AT_FINAL_DROP))
    legacy_finish_fields = (
        header.finish_axis,
        header.finish_direction,
        header.finish_line_mm,
        header.finish_envelope_margin_mm,
        header.finish_stable_time_ms,
        header.finish_brake_accel_mmps2,
        header.finish_max_runout_mm,
        header.finish_hard_timeout_ms,
    )
    if any(legacy_finish_fields):
        _fail("header.finish_legacy_fields", "legacy half-plane/safe-end finish fields must be zero", actual=legacy_finish_fields, expected="all zero")
    if header.finish_signal_flags & ~0x0007:
        _fail("header.finish_signal_flags", "unknown finish signal bits", actual=header.finish_signal_flags, expected="0x0000..0x0007")

    _validate_nodes(trajectory)
    _validate_segments(trajectory)
    _validate_actions(trajectory)


def _validate_nodes(trajectory: CompiledTrajectoryV40) -> None:
    nodes = trajectory.nodes
    if not nodes:
        _fail("nodes", "node table must not be empty")
    start_indexes = [index for index, node in enumerate(nodes) if node.flags & int(NodeFlag.START)]
    if start_indexes != [0]:
        _fail("nodes.START", "START must appear exactly once at node 0", actual=start_indexes, expected=[0])
    start = nodes[0]
    if start.s_mm != 0:
        _fail("nodes[0].s_mm", "START s must be zero", actual=start.s_mm, expected=0)
    if start.arrival_id != 0xFF:
        _fail("nodes[0].arrival_id", "START arrival_id must be 0xFF", actual=start.arrival_id, expected=0xFF)
    if not _is_zero_motion(start):
        _fail("nodes[0]", "START velocity must be zero")

    arrival_ids: list[int] = []
    finish_arm_indexes: list[int] = []
    safe_end_indexes: list[int] = []
    last_s = -1
    for index, node in enumerate(nodes):
        if node.s_mm < last_s:
            _fail(f"nodes[{index}].s_mm", "s_mm must be monotonic nondecreasing", actual=node.s_mm, expected=f">={last_s}")
        last_s = node.s_mm
        if node.flags & int(NodeFlag.ARRIVAL):
            if not (node.flags & int(NodeFlag.EXACT_PASS)):
                _fail(f"nodes[{index}].flags", "ARRIVAL must also set EXACT_PASS", actual=node.flags)
            if not _is_zero_motion(node):
                _fail(f"nodes[{index}]", "ARRIVAL velocity must be zero")
            arrival_ids.append(node.arrival_id)
        elif node.arrival_id != 0xFF:
            _fail(
                f"nodes[{index}].arrival_id",
                "non-ARRIVAL node arrival_id must be 0xFF",
                actual=node.arrival_id,
                expected=0xFF,
            )
        if node.flags & int(NodeFlag.FINISH_ARM):
            finish_arm_indexes.append(index)
        if node.flags & int(NodeFlag.SAFE_END):
            safe_end_indexes.append(index)

    expected_ids = list(range(len(arrival_ids)))
    if arrival_ids != expected_ids:
        _fail("nodes.arrival_id", "ARRIVAL ids must be contiguous in path order", actual=arrival_ids, expected=expected_ids)
    if len(finish_arm_indexes) != 1:
        _fail("nodes.FINISH_ARM", "FINISH_ARM must appear exactly once", actual=finish_arm_indexes, expected="one index")
    if safe_end_indexes:
        _fail("nodes.SAFE_END", "SAFE_END is reserved and must be zero in formal V4.0 output", actual=safe_end_indexes, expected=[])
    if finish_arm_indexes != [len(nodes) - 1]:
        _fail("nodes.FINISH_ARM", "FINISH_ARM must be the final node", actual=finish_arm_indexes, expected=[len(nodes) - 1])
    finish_node = nodes[-1]
    if not (finish_node.flags & int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS)) == int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS):
        _fail("nodes[-1].flags", "final FINISH_ARM node must also be ARRIVAL and EXACT_PASS", actual=finish_node.flags)
    if not _is_zero_motion(finish_node):
        _fail("nodes[-1]", "final drop ARRIVAL velocity must be zero")


def _validate_segments(trajectory: CompiledTrajectoryV40) -> None:
    header = trajectory.header
    nodes = trajectory.nodes
    planned_sum = 0
    finish_clear_indexes: list[int] = []
    previous_end = None
    previous_end_s = None

    for index, segment in enumerate(trajectory.segments):
        if segment.segment_id != index:
            _fail(f"segments[{index}].segment_id", "segment ids must be contiguous", actual=segment.segment_id, expected=index)
        if segment.flags == 0:
            _fail(f"segments[{index}].flags", "segment flags must not be empty")
        if segment.start_node_index >= len(nodes) or segment.end_node_index >= len(nodes):
            _fail(
                f"segments[{index}].node_index",
                "segment node index out of range",
                actual=(segment.start_node_index, segment.end_node_index),
                expected=f"0..{len(nodes) - 1}",
            )
        if segment.start_node_index > segment.end_node_index:
            _fail(
                f"segments[{index}].node_index",
                "start_node_index must be <= end_node_index",
                actual=(segment.start_node_index, segment.end_node_index),
            )
        if index == 0 and segment.start_node_index != 0:
            _fail("segments[0].start_node_index", "first segment must start at node 0", actual=segment.start_node_index, expected=0)
        if previous_end is not None and segment.start_node_index != previous_end:
            _fail(
                f"segments[{index}].start_node_index",
                "adjacent segments must share one boundary node",
                actual=segment.start_node_index,
                expected=previous_end,
            )
        start_node = nodes[segment.start_node_index]
        end_node = nodes[segment.end_node_index]
        if segment.start_s_mm != start_node.s_mm:
            _fail(f"segments[{index}].start_s_mm", "start_s must match start node", actual=segment.start_s_mm, expected=start_node.s_mm)
        if segment.end_s_mm != end_node.s_mm:
            _fail(f"segments[{index}].end_s_mm", "end_s must match end node", actual=segment.end_s_mm, expected=end_node.s_mm)
        if previous_end_s is not None and segment.start_s_mm != previous_end_s:
            _fail(
                f"segments[{index}].start_s_mm",
                "adjacent segments must share boundary s",
                actual=segment.start_s_mm,
                expected=previous_end_s,
            )
        if segment.start_arrival_id != start_node.arrival_id:
            _fail(
                f"segments[{index}].start_arrival_id",
                "start_arrival_id must match start node",
                actual=segment.start_arrival_id,
                expected=start_node.arrival_id,
            )
        if segment.end_arrival_id != end_node.arrival_id:
            _fail(
                f"segments[{index}].end_arrival_id",
                "end_arrival_id must match end node",
                actual=segment.end_arrival_id,
                expected=end_node.arrival_id,
            )
        if segment.flags & int(SegmentFlag.FINISH_CLEAR):
            finish_clear_indexes.append(index)
        planned_sum += segment.planned_time_ms
        previous_end = segment.end_node_index
        previous_end_s = segment.end_s_mm

    if previous_end != len(nodes) - 1:
        _fail("segments[-1].end_node_index", "last segment must end at final FINISH_ARM", actual=previous_end, expected=len(nodes) - 1)
    if finish_clear_indexes:
        _fail("segments.FINISH_CLEAR", "FINISH_CLEAR is reserved and must be zero in formal V4.0 output", actual=finish_clear_indexes, expected=[])
    if abs(planned_sum - header.planned_motion_time_ms) > 1:
        _fail(
            "header.planned_motion_time_ms",
            "segment planned_time_ms sum must match header",
            actual=planned_sum,
            expected=header.planned_motion_time_ms,
        )


def _validate_actions(trajectory: CompiledTrajectoryV40) -> None:
    header = trajectory.header
    last_stop_arrival_id = -1
    finish_arrival_id = trajectory.nodes[-1].arrival_id
    drop_actions = {int(ActionCode.DROP_1), int(ActionCode.DROP_2), int(ActionCode.DROP_3), int(ActionCode.DROP_12), int(ActionCode.DROP_23)}
    for index, action in enumerate(trajectory.actions):
        if action.action_seq != index:
            _fail(f"actions[{index}].action_seq", "action_seq must be contiguous", actual=action.action_seq, expected=index)
        if action.timeout_ms <= 0:
            _fail(f"actions[{index}].timeout_ms", "timeout_ms must be positive", actual=action.timeout_ms, expected=">0")

        limits = _limits_tuple(action)
        if action.mode == int(ActionMode.STOP_AND_WAIT):
            if not 0 <= action.arrival_id < header.arrival_count:
                _fail(
                    f"actions[{index}].arrival_id",
                    "STOP_AND_WAIT arrival_id out of range",
                    actual=action.arrival_id,
                    expected=f"0..{header.arrival_count - 1}",
                )
            if action.arrival_id < last_stop_arrival_id:
                _fail(
                    f"actions[{index}].arrival_id",
                    "STOP_AND_WAIT arrival_id must be nondecreasing by action_seq",
                    actual=action.arrival_id,
                    expected=f">={last_stop_arrival_id}",
                )
            last_stop_arrival_id = action.arrival_id
            if action.check_start_s_mm != 0xFFFF:
                _fail(f"actions[{index}].check_start_s_mm", "STOP_AND_WAIT has no check_start", actual=action.check_start_s_mm, expected=0xFFFF)
            if limits != (0, 0, 0, 0, 0):
                _fail(f"actions[{index}]", "STOP_AND_WAIT must not carry kinematic limits", actual=limits, expected=(0, 0, 0, 0, 0))
        elif action.mode == int(ActionMode.ASYNC):
            if action.arrival_id != 0xFF:
                _fail(f"actions[{index}].arrival_id", "ASYNC has no arrival trigger", actual=action.arrival_id, expected=0xFF)
            if action.check_start_s_mm != 0xFFFF:
                _fail(f"actions[{index}].check_start_s_mm", "ASYNC has no check_start", actual=action.check_start_s_mm, expected=0xFFFF)
            if limits != (0, 0, 0, 0, 0):
                _fail(f"actions[{index}]", "ASYNC must not carry kinematic limits", actual=limits, expected=(0, 0, 0, 0, 0))
        elif action.mode == int(ActionMode.KINEMATIC):
            if action.arrival_id != 0xFF:
                _fail(f"actions[{index}].arrival_id", "KINEMATIC has no user arrival window", actual=action.arrival_id, expected=0xFF)
            if action.check_start_s_mm > header.total_length_mm:
                _fail(
                    f"actions[{index}].check_start_s_mm",
                    "KINEMATIC check_start out of range",
                    actual=action.check_start_s_mm,
                    expected=f"0..{header.total_length_mm}",
                )
            if action.stable_time_ms <= 0:
                _fail(f"actions[{index}].stable_time_ms", "KINEMATIC stable_time_ms must be positive", actual=action.stable_time_ms, expected=">0")
            if not any(limits[:4]):
                _fail(f"actions[{index}]", "KINEMATIC needs at least one motion limit", actual=limits[:4], expected="any nonzero limit")
    # Working MANUAL/SEMI_AUTO trajectories may deliberately omit mechanical
    # actions while the chassis path is being tested.  MANUAL_OVERRIDE marks
    # those non-final work products.  Formal export performs the stricter final
    # DROP check in mode_output_service._require_final_drop().
    if header.route_family == int(RouteFamily.MANUAL) or (header.flags & int(HeaderFlag.MANUAL_OVERRIDE)):
        return
    if not trajectory.actions:
        _fail("actions", "formal FULL_AUTO output requires a final DROP STOP_AND_WAIT action")
    final = trajectory.actions[-1]
    if final.action not in drop_actions or final.mode != int(ActionMode.STOP_AND_WAIT) or final.arrival_id != finish_arrival_id:
        _fail(
            "actions[-1]",
            "final action must be DROP_* STOP_AND_WAIT bound to final FINISH_ARM ARRIVAL",
            actual=(final.action, final.mode, final.arrival_id),
            expected=(sorted(drop_actions), int(ActionMode.STOP_AND_WAIT), finish_arrival_id),
        )
