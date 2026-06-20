from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_services.competition_task_config_service import (
    build_route_case_table_from_task_config,
    default_competition_task_config,
    save_competition_task_config,
)
from hjmb_pathgen.py_services.semi_auto_path_service import resolve_semi_path
from hjmb_pathgen.py_services.task_compiler import (
    automatic_candidate_subset,
    build_case_draft,
    compile_task_candidates,
)

from phase3_helpers import phase3_project


def test_task_json_generates_complete_english_only_mapping() -> None:
    config = default_competition_task_config()
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "task_config" / "competition_task_config.json"
        path.parent.mkdir(parents=True)
        save_competition_task_config(path, config)
        table = build_route_case_table_from_task_config(path)

    assert len(table.cases) == 360
    assert table.source_csv == "task_config/competition_task_config.json"
    first = table.cases[0]
    assert first.traj_id == 0
    assert first.pick_assignment == {
        "PICK_1": "YELLOW",
        "PICK_2": "GREEN",
        "PICK_3": "WHITE",
    }
    assert {key: first.label_positions[key] for key in ("1", "2", "3")} == {
        "1": "F_DROP_4",
        "2": "F_DROP_5",
        "3": "F_DROP_6",
    }


def test_traj_zero_compiles_unique_bin_assignment_and_two_unload_stops() -> None:
    project = phase3_project()
    config = default_competition_task_config()
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "task_config" / "competition_task_config.json"
        path.parent.mkdir(parents=True)
        save_competition_task_config(path, config)
        row = build_route_case_table_from_task_config(path).cases[0]

    candidate_set = compile_task_candidates(row, project, config)
    assert len(candidate_set.candidates) == 2
    for candidate in candidate_set.candidates:
        assert candidate.vehicle_bin_assignment == {
            "YELLOW": "BIN_1",
            "GREEN": "BIN_2",
            "WHITE": "BIN_3",
        }
        assert candidate.stop_count == 2
        assert {step.unload_pose_profile_id for step in candidate.unload_sequence} == {
            "DROP_F45_BIN_12",
            "DROP_F6_BIN_3",
        }

    selected = automatic_candidate_subset(candidate_set.candidates, config)
    assert len(selected) == 2
    assert {item.route_family.name for item in selected} == {
        "PICK_1_TO_3",
        "PICK_3_TO_1",
    }
    for item in selected:
        assert {step.unload_mask.value for step in item.unload_sequence} == {
            "BIN_12",
            "BIN_3",
        }

    built = build_case_draft(row, project, task_config=config)
    assert [item["action"] for item in built.case.actions["source"][-2:]] == [
        "DROP_12",
        "DROP_3",
    ]
    assert [item["type"] for item in built.case.arrival_states] == [
        "PICK",
        "PICK",
        "PICK",
        "DROP",
        "DROP",
    ]
    assert len(built.transition_requirements) == 5
    assert all(
        int(item["pose"]["yaw_ddeg"]) != 0xFFFF
        for item in built.case.arrival_states
        if item["type"] == "DROP"
    )


def test_all_360_rows_have_legal_candidates_with_configured_pose_catalog() -> None:
    project = phase3_project()
    config = default_competition_task_config()
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "task_config" / "competition_task_config.json"
        path.parent.mkdir(parents=True)
        save_competition_task_config(path, config)
        table = build_route_case_table_from_task_config(path)

    for row in table.cases:
        candidates = compile_task_candidates(row, project, config).candidates
        assert candidates, f"P{row.traj_id:04d} has no legal candidate"
        for candidate in candidates:
            for bean, vehicle_bin in candidate.vehicle_bin_assignment.items():
                target = next(
                    item
                    for item in row.label_positions.items()
                    if item[0] == {"YELLOW": "1", "GREEN": "2", "WHITE": "3"}[bean]
                )[1]
                assert vehicle_bin in config.bin_reachability[target]
            for step in candidate.unload_sequence:
                if len(step.physical_sites) > 1:
                    assert set(step.physical_sites) in (
                        {"F_DROP_4", "F_DROP_5"},
                        {"F_DROP_7", "F_DROP_8"},
                    )


def test_semi_auto_drop_profile_resolves_fixed_0xffff_to_real_yaw() -> None:
    project = phase3_project()
    data = project.to_dict()
    data["planner_profiles"].setdefault("default", {})["use_unload_pose_profiles"] = True
    for site in ("P_DROP_1", "P_DROP_2", "P_DROP_3"):
        data["sites"][site]["yaw_ddeg"] = 0xFFFF
    project = project.__class__.from_dict(data)

    # Reuse a valid manifest shell, but supply a user-authored semi path.
    config = default_competition_task_config()
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "task_config" / "competition_task_config.json"
        path.parent.mkdir(parents=True)
        save_competition_task_config(path, config)
        row = build_route_case_table_from_task_config(path).cases[0]
    full_case = build_case_draft(row, project, task_config=config).case
    semi_path = {
        "points": [
            {"type": "START", "site_key": "P_START"},
            {"type": "ARRIVAL", "site_key": "P_PICK_1"},
            {"type": "ARRIVAL", "site_key": "P_PICK_2L"},
            {"type": "ARRIVAL", "site_key": "P_PICK_3"},
            {
                "type": "ARRIVAL",
                "site_key": "P_DROP_3",
                "state_id": "DROP_A",
                "unload_pose_profile_id": "DROP_F45_BIN_12",
            },
            {
                "type": "ARRIVAL",
                "site_key": "P_DROP_2",
                "state_id": "DROP_B",
                "unload_pose_profile_id": "DROP_F6_BIN_3",
            },
            {
                "type": "ARRIVAL",
                "site_key": "P_DROP_1",
                "state_id": "DROP_C",
                "unload_pose_profile_id": "DROP_F78_BIN_23",
            },
        ]
    }
    semi_case = replace(
        full_case,
        generation_mode=GenerationMode.SEMI_AUTO,
        semi_path=semi_path,
        leg_refs=(),
    )
    resolved = resolve_semi_path(semi_case, project)
    drop_rows = [item for item in resolved["points"] if item.get("hints", {}).get("unload_pose_profile_id")]
    assert len(drop_rows) == 3
    assert all(int(item["yaw_ddeg"]) != 0xFFFF for item in drop_rows)
    assert [item["yaw_ddeg"] for item in drop_rows] == [450, -900, -450]
