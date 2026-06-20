from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.py_domain.enums import RouteFamily, UnloadMask, YawPolicy
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_io.codecs.legacy_rejection import reject_deleted_fields
from hjmb_pathgen.py_domain.task_plan import CandidatePlan, UnloadStep
from hjmb_pathgen.py_planning.geometry.automatic_topology import (
    NO_GATE_PROFILE_ID,
    default_transfer_gates,
    topology_profile_for_transition,
)
from hjmb_pathgen.py_services.execution_time_estimator import estimate_fifo_execution
from hjmb_pathgen.py_services.task_compiler import (
    automatic_candidate_subset,
    build_case_draft,
    compile_task_candidates,
    preferred_route_family_for_candidate,
    unload_stop_ranks,
)

from phase3_helpers import phase3_project, phase3_project_dict
from unit.test_phase3_task_compiler import route_row


def _candidate(route_family: RouteFamily, ranks: tuple[int, ...]) -> CandidatePlan:
    steps = tuple(
        UnloadStep(
            step_index=index,
            unload_mask=UnloadMask.BIN_1,
            target_ranks=(rank,),
            bean_types=("YELLOW",),
            physical_sites=(f"F_DROP_{rank + 3}",),
            vehicle_bins=("BIN_1",),
            anchor_site=f"F_DROP_{rank + 3}",
            yaw_ddeg=0,
        )
        for index, rank in enumerate(ranks, start=1)
    )
    return CandidatePlan(
        candidate_id=f"{route_family.name}_{'_'.join(map(str, ranks))}",
        semantic_hash="test",
        traj_id=0,
        route_family=route_family,
        pickup_position_order=(),
        pickup_arrival_state_order=(),
        drop_target_rank_order=ranks,
        vehicle_bin_assignment={},
        unload_sequence=steps,
        yaw_direction=YawPolicy.SHORTEST,
        yaw_sequence_ddeg=(),
        source_actions=(),
        estimated_mechanism_time_ms=0,
    )


def test_deterministic_left_right_route_rule() -> None:
    for ranks, expected in (
        ((1, 2), RouteFamily.PICK_3_TO_1),
        ((2, 3), RouteFamily.PICK_1_TO_3),
        ((1, 2, 3), RouteFamily.PICK_1_TO_3),
        ((1, 3), RouteFamily.PICK_1_TO_3),
    ):
        pair = tuple(_candidate(route, ranks) for route in (RouteFamily.PICK_1_TO_3, RouteFamily.PICK_3_TO_1))
        selected = automatic_candidate_subset(pair)
        assert len(selected) == 1
        assert selected[0].route_family == expected
        assert preferred_route_family_for_candidate(selected[0]) == expected
        assert unload_stop_ranks(selected[0]) == ranks


def test_only_pickup_to_first_drop_uses_ordered_s_gates() -> None:
    project = phase3_project()
    candidates = compile_task_candidates(route_row(), project).candidates
    case = build_case_draft(route_row(), project, preferred_candidate_id=candidates[0].candidate_id)
    requirements = case.transition_requirements
    gated = [item for item in requirements if item.topology_profile != NO_GATE_PROFILE_ID]
    assert len(gated) == 1
    assert gated[0].from_state_id.startswith("P_PICK_")
    assert gated[0].to_state_id == "DROP_STEP_1"
    assert all(
        item.topology_profile == NO_GATE_PROFILE_ID
        for item in requirements
        if item is not gated[0]
    )


def test_generated_left_and_right_gate_lanes_are_opposite() -> None:
    data = phase3_project_dict()
    data["field_objects"]["cylinders"] = [
        {
            "obstacle_id": "CYLINDER_PICKUP",
            "center_x_mm": 1000,
            "center_y_mm": 0,
            "radius_mm": 51,
            "configured": True,
            "enabled": True,
        },
        {
            "obstacle_id": "CYLINDER_DROP",
            "center_x_mm": -1000,
            "center_y_mm": 0,
            "radius_mm": 51,
            "configured": True,
            "enabled": True,
        },
    ]
    project = ProjectV40.from_dict(data)
    left = default_transfer_gates(project, "PICK_1_TO_3")
    right = default_transfer_gates(project, "PICK_3_TO_1")
    assert len(left) == len(right) == 2
    assert left[0]["b"]["y_mm"] < 0 < left[1]["a"]["y_mm"]
    assert right[0]["a"]["y_mm"] > 0 > right[1]["b"]["y_mm"]
    assert left[0]["a"]["x_mm"] > left[1]["a"]["x_mm"]
    assert topology_profile_for_transition(project, "PICK_1_TO_3", "P_PICK_3", "DROP_STEP_1") == "S_LEFT_TRANSFER"
    assert topology_profile_for_transition(project, "PICK_1_TO_3", "P_PICK_1", "P_PICK_2L") == NO_GATE_PROFILE_ID


def test_fifo_estimator_overlaps_async_and_carries_remainder_to_stop() -> None:
    project = phase3_project()
    actions = (
        {"action": "PREP_PICK_1", "mode": "ASYNC", "estimated_time_ms": 5000},
        {
            "action": "PICK",
            "mode": "STOP_AND_WAIT",
            "arrival_state_id": "A",
            "estimated_time_ms": 1000,
        },
        {"action": "PREP_STORE_1", "mode": "ASYNC", "estimated_time_ms": 5000},
        {
            "action": "STORE",
            "mode": "STOP_AND_WAIT",
            "arrival_state_id": "B",
            "estimated_time_ms": 1000,
        },
    )
    estimate = estimate_fifo_execution(
        project,
        actions,
        motion_time_ms=10000,
        arrival_release_ms={"A": 3000, "B": 7000},
    )
    assert estimate.mechanism_busy_time_ms == 12000
    assert estimate.added_wait_time_ms == 5000
    assert estimate.total_time_ms == 15000
    # The second async action is still active when B is reached at 10 s,
    # therefore the B stop action starts at 11 s and the remaining motion shifts.
    assert estimate.action_timeline[-1]["release_ms"] == 10000
    assert estimate.action_timeline[-1]["start_ms"] == 11000
    assert estimate.action_timeline[-1]["fifo_wait_ms"] == 1000


def test_v40_virtual_gate_id_is_allowed_but_runtime_v3_gate_is_rejected() -> None:
    reject_deleted_fields(
        {"topology_profiles": {"PICK_1_TO_3": {"gates": [{"gate_id": "S1"}]}}},
        "project",
    )
    try:
        reject_deleted_fields({"actions": [{"gate_id": 1}]}, "case")
    except V40ValidationError:
        pass
    else:
        raise AssertionError("legacy runtime gate_id must still be rejected")
