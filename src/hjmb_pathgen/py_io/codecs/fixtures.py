"""Deterministic V4.0 fixtures used by tests and documentation."""

from __future__ import annotations

from pathlib import Path

from hjmb_pathgen.py_domain.compiled import ActionV40, CompiledTrajectoryV40, HeaderV40, NodeV40, SegmentV40
from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode, NodeFlag, RouteFamily, SegmentFlag

from .binary_layout import encode_compiled_trajectory


def minimal_compiled_trajectory() -> CompiledTrajectoryV40:
    nodes = (
        NodeV40(
            s_mm=0,
            x_mm=0,
            y_mm=0,
            yaw_ddeg=0,
            vx_mmps=0,
            vy_mmps=0,
            wz_ddegps=0,
            arrival_id=0xFF,
            flags=int(NodeFlag.START | NodeFlag.EXACT_PASS),
        ),
        NodeV40(
            s_mm=100,
            x_mm=100,
            y_mm=0,
            yaw_ddeg=0,
            vx_mmps=0,
            vy_mmps=0,
            wz_ddegps=0,
            arrival_id=0,
            flags=int(NodeFlag.ARRIVAL | NodeFlag.EXACT_PASS | NodeFlag.FINISH_ARM),
        ),
    )
    segments = (
        SegmentV40(
            segment_id=0,
            start_node_index=0,
            end_node_index=1,
            start_s_mm=0,
            end_s_mm=100,
            start_arrival_id=0xFF,
            end_arrival_id=0,
            flags=int(SegmentFlag.NORMAL),
            planned_time_ms=1000,
        ),
    )
    actions = (
        ActionV40(
            action_seq=0,
            action=int(ActionCode.PICK),
            mode=int(ActionMode.STOP_AND_WAIT),
            arrival_id=0,
            timeout_ms=2000,
            check_start_s_mm=0xFFFF,
        ),
    )
    header = HeaderV40(
        route_family=int(RouteFamily.MANUAL),
        traj_id=0,
        bean_code=0,
        drop_code=0,
        planned_motion_time_ms=1000,
        planned_total_estimate_ms=3000,
        finish_signal_flags=0,
    )
    return CompiledTrajectoryV40(header=header, nodes=nodes, segments=segments, actions=actions).normalized()


def minimal_bin_bytes() -> bytes:
    return encode_compiled_trajectory(minimal_compiled_trajectory())


def write_minimal_bin(path: Path) -> None:
    path.write_bytes(minimal_bin_bytes())


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Write deterministic V4.0 minimal.bin")
    parser.add_argument("path")
    args = parser.parse_args()
    write_minimal_bin(Path(args.path))
