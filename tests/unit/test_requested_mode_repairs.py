from __future__ import annotations

import inspect
import json
import os
import unittest
import tempfile
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication, QComboBox

from hjmb_pathgen.py_domain.manual_path import ManualPathV40
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.protocol import YAW_UNSPECIFIED_DDEG
from hjmb_pathgen.py_domain.semi_path import ROUTE_A_SITE_SEQUENCE, ROUTE_B_SITE_SEQUENCE, route_family_from_site_sequence
from hjmb_pathgen.py_io.codecs.json_codec import load_project, save_project
from hjmb_pathgen.py_services.manual_path_service import build_manual_spatial_path
from hjmb_pathgen.py_services.mode_output_service import write_semi_auto_outputs
from hjmb_pathgen.py_services.semi_auto_path_service import plan_semi_auto_case, resolve_semi_path
from hjmb_pathgen.py_services.task_compiler import build_case_draft, compile_task_candidates
from hjmb_pathgen.py_ui.v35_base.path_models import (
    EditPoint,
    PATH_MODE_FIXED_8,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    PathProject,
    resolve_edit_points,
)
from hjmb_pathgen.py_ui.v35_base.editor import MainWindow as V35BaseMainWindow
from hjmb_pathgen.py_ui.v35_exact_main_window import V35ExactV4MainWindow
from tests.phase3_helpers import phase3_project_dict
from tests.unit.test_phase3_task_compiler import route_row


FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def _qt_app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_fixed_site_yaw_combo_popup_is_not_destroyed_when_opened() -> None:
    app = _qt_app()
    window = V35BaseMainWindow()
    try:
        window.plan_timer.stop()
        combo = window.fixed_site_table.cellWidget(0, 4)
        assert isinstance(combo, QComboBox)
        combo.showPopup()
        app.processEvents()
        assert window.fixed_site_table.cellWidget(0, 4) is combo
        combo.hidePopup()
    finally:
        window.close()


def test_new_semi_auto_view_starts_with_no_path_points() -> None:
    _qt_app()
    window = V35ExactV4MainWindow()
    try:
        window._generation_mode = GenerationMode.SEMI_AUTO
        window._prepare_empty_mode_view()
        assert window.project.points == []
    finally:
        window.close()


def test_legacy_project_yaw_0xff_migrates_to_0xffff() -> None:
    data = json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))
    data["sites"]["P_PICK_2L"]["yaw_ddeg"] = 0xFF
    project = ProjectV40.from_dict(data)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "project.json"
        save_project(path, project)
        loaded = load_project(path)
        raw = json.loads(path.read_text(encoding="utf-8"))
    assert loaded.sites["P_PICK_2L"]["yaw_ddeg"] == YAW_UNSPECIFIED_DDEG
    assert raw["sites"]["P_PICK_2L"]["yaw_ddeg"] == YAW_UNSPECIFIED_DDEG



def test_semi_auto_exposes_no_selectable_optimization_profile() -> None:
    assert "profile_name" not in inspect.signature(plan_semi_auto_case).parameters
    assert "profile_name" not in inspect.signature(write_semi_auto_outputs).parameters

def test_fixed_site_0xffff_is_not_duplicated_as_a_per_point_override() -> None:
    project = PathProject(path_mode=PATH_MODE_FIXED_8)
    project.fixed_sites[1].yaw_ddeg = YAW_UNSPECIFIED_DDEG
    project.points = [
        EditPoint(point_id=0, type=POINT_TYPE_START, site_id=0, yaw_ddeg=1234),
        EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL, site_id=1, yaw_ddeg=-900),
    ]
    raw_points = project.to_config_dict()["points"]
    assert raw_points == [
        {"point_id": 0, "type": "START", "site_id": 0},
        {"point_id": 1, "type": "ARRIVAL", "site_id": 1},
    ]
    resolved = resolve_edit_points(project)
    assert resolved[1].yaw_ddeg == YAW_UNSPECIFIED_DDEG


def test_manual_path_restores_local_bezier_and_keeps_unconstrained_yaw() -> None:
    path = ManualPathV40.from_dict(
        {
            "points": [
                {"type": "START", "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
                {"type": "WAYPOINT", "x_mm": 500, "y_mm": 0, "corner_trim_mm": 150},
                {"type": "ARRIVAL", "x_mm": 500, "y_mm": 500, "yaw_ddeg": YAW_UNSPECIFIED_DDEG},
            ]
        }
    )
    samples = build_manual_spatial_path(path)
    # The corner vertex is replaced by a local quadratic Bezier, not a hard turn.
    assert not any(abs(item.x_mm - 500.0) < 1.0e-6 and abs(item.y_mm) < 1.0e-6 for item in samples)
    assert any(350.0 < item.x_mm < 500.0 and 0.0 < item.y_mm < 150.0 for item in samples)
    # 0xFFFF is unconstrained: planning keeps the previous yaw instead of treating it as 25.5 degrees.
    assert abs(samples[-1].yaw_ddeg) < 1.0e-9


def test_manual_yaw_is_concentrated_in_two_low_speed_windows() -> None:
    path = ManualPathV40.from_dict(
        {
            "points": [
                {"type": "START", "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
                {"type": "ARRIVAL", "x_mm": 1000, "y_mm": 0, "yaw_ddeg": 900},
            ]
        }
    )
    samples = build_manual_spatial_path(path)
    middle = [item for item in samples if 0.4 * samples[-1].s_mm <= item.s_mm <= 0.6 * samples[-1].s_mm]
    assert middle
    assert max(abs(item.yaw_ddeg_per_mm) for item in middle) < 1.0e-9
    assert any(abs(item.yaw_ddeg_per_mm) > 1.0e-6 for item in samples[: len(samples) // 3])
    assert any(abs(item.yaw_ddeg_per_mm) > 1.0e-6 for item in samples[-len(samples) // 3 :])


def test_semi_auto_fixed_order_is_exactly_route_a_or_b() -> None:
    assert route_family_from_site_sequence(ROUTE_A_SITE_SEQUENCE).name == "PICK_1_TO_3"
    assert route_family_from_site_sequence(ROUTE_B_SITE_SEQUENCE).name == "PICK_3_TO_1"
    with unittest.TestCase().assertRaisesRegex(Exception, "two legal competition routes"):
        route_family_from_site_sequence(
            (
                "P_START",
                "P_PICK_1",
                "P_PICK_2L",
                "P_PICK_2R",
                "P_PICK_3",
                "P_DROP_3",
                "P_DROP_2",
                "P_DROP_1",
            )
        )



def test_semi_auto_removes_only_a_redundant_waypoint_at_a_fixed_stop() -> None:
    project = ProjectV40.from_dict(phase3_project_dict())
    case_data = {
        "format": "HJMB_ROUTE_CASE_JSON_V40",
        "storage_mode": "REFERENCED",
        "generation_mode": "SEMI_AUTO",
        "traj_id": 0,
        "bean_code": 0,
        "drop_code": 0,
        "source_mapping": {},
        "selected_plan": {
            "route_family": "PICK_1_TO_3",
            "vehicle_bin_assignment": {},
            "drop_targets": [],
            "unload_sequence": [],
            "yaw_direction": "SHORTEST",
            "locked_by_user": True,
        },
        "manual_path": None,
        "semi_path": {
            "points": [
                {"type": "START", "site_key": "P_START"},
                {
                    "type": "WAYPOINT",
                    "x_mm": project.sites["P_PICK_1"]["x_mm"],
                    "y_mm": project.sites["P_PICK_1"]["y_mm"],
                },
                {"type": "ARRIVAL", "site_key": "P_PICK_1"},
                {"type": "ARRIVAL", "site_key": "P_PICK_2L"},
                {"type": "ARRIVAL", "site_key": "P_PICK_3"},
                {"type": "ARRIVAL", "site_key": "P_DROP_3"},
                {"type": "ARRIVAL", "site_key": "P_DROP_2"},
                {"type": "ARRIVAL", "site_key": "P_DROP_1"},
            ]
        },
        "logical_points": [],
        "auxiliary_points": [],
        "arrival_states": [],
        "leg_refs": [],
        "actions": {"source": [], "compiled": []},
        "finish": {"mode": "AT_FINAL_DROP"},
        "estimates": {},
        "hashes": {},
        "review": {
            "state": "STALE",
            "detached_from_library": True,
            "manual_override": True,
            "approved": False,
            "override_reason": "test",
        },
    }
    case = CaseManifestV40.from_dict(case_data)
    resolved = resolve_semi_path(case, project)
    assert len(resolved["points"]) == 7
    assert resolved["points"][1]["type"] == "ARRIVAL"
    assert resolved["points"][1]["hints"]["site_key"] == "P_PICK_1"

def test_full_auto_candidates_never_mix_pick_2l_and_2r_and_use_only_two_orders() -> None:
    project = ProjectV40.from_dict(phase3_project_dict())
    candidates = compile_task_candidates(route_row(), project).candidates
    orders = {candidate.pickup_arrival_state_order for candidate in candidates}
    assert orders == {
        ("P_PICK_1", "P_PICK_2L", "P_PICK_3"),
        ("P_PICK_3", "P_PICK_2R", "P_PICK_1"),
    }
    for candidate in candidates:
        assert not ({"P_PICK_2L", "P_PICK_2R"} <= set(candidate.pickup_arrival_state_order))
        if candidate.route_family.name == "PICK_1_TO_3":
            assert candidate.drop_target_rank_order == (3, 2, 1)
        else:
            assert candidate.drop_target_rank_order == (1, 2, 3)


def test_full_auto_resolves_0xffff_only_in_derived_case_not_project() -> None:
    data = phase3_project_dict()
    data["sites"]["P_PICK_2L"]["yaw_ddeg"] = 0xFF
    project = ProjectV40.from_dict(data)
    candidate = next(
        item
        for item in compile_task_candidates(route_row(), project).candidates
        if item.route_family.name == "PICK_1_TO_3"
    )
    draft = build_case_draft(route_row(), project, preferred_candidate_id=candidate.candidate_id)
    pick_2l = next(item for item in draft.case.arrival_states if item["state_id"] == "P_PICK_2L")
    assert project.sites["P_PICK_2L"]["yaw_ddeg"] == YAW_UNSPECIFIED_DDEG
    assert pick_2l["pose"]["yaw_ddeg"] != YAW_UNSPECIFIED_DDEG
    assert pick_2l["pose"]["yaw_source"] == "UNCONSTRAINED_0XFFFF"
    assert all(req.from_pose["yaw_ddeg"] != YAW_UNSPECIFIED_DDEG and req.to_pose["yaw_ddeg"] != YAW_UNSPECIFIED_DDEG for req in draft.transition_requirements)
