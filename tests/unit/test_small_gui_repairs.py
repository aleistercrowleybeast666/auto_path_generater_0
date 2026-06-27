from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication, QComboBox, QPlainTextEdit, QScrollArea

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_services.fixed_site_collision_service import (
    FixedSiteCollisionEntry,
    FixedSiteCollisionReport,
    FixedSiteCollisionResult,
)
from hjmb_pathgen.py_ui.v35_base.editor import (
    DROP_STATIONS,
    PICKUP_STATIONS,
    MainWindow as V35BaseMainWindow,
)
from hjmb_pathgen.py_ui.v35_base.path_models import (
    EditPoint,
    MechanicalAction,
    PATH_MODE_FIXED_8,
    PATH_MODE_FREE,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    SITE_ID_FREE,
    YAW_UNSPECIFIED_DDEG,
)
from hjmb_pathgen.py_ui.v35_exact_main_window import V35ExactV4MainWindow
from hjmb_pathgen.py_ui.ui_state import LoadedProjectState

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def minimal_project() -> ProjectV40:
    data = json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))
    return ProjectV40.from_dict(data)


class SmallGuiRepairsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_fixed_site_yaw_combo_popup_is_not_destroyed_when_opened(self) -> None:
        window = V35BaseMainWindow()
        try:
            window.plan_timer.stop()
            combo = window.fixed_site_table.cellWidget(0, 4)
            self.assertIsInstance(combo, QComboBox)
            assert isinstance(combo, QComboBox)
            combo.showPopup()
            self.app.processEvents()
            self.assertIs(window.fixed_site_table.cellWidget(0, 4), combo)
            combo.hidePopup()
        finally:
            window.close()

    def test_new_semi_auto_view_starts_with_no_path_points(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._generation_mode = GenerationMode.SEMI_AUTO
            window._load_current_mode_case()
            self.assertEqual(window.project.points, [])
        finally:
            window.close()

    def test_double_click_defaults_to_rounded_waypoint_in_manual_and_semi_auto(self) -> None:
        modes = (
            (GenerationMode.MANUAL, PATH_MODE_FREE),
            (GenerationMode.SEMI_AUTO, PATH_MODE_FIXED_8),
        )
        for generation_mode, path_mode in modes:
            with self.subTest(generation_mode=generation_mode):
                window = V35ExactV4MainWindow()
                try:
                    window._generation_mode = generation_mode
                    window.project.path_mode = path_mode
                    window.project.points = []
                    window.add_point_from_canvas(10, 20)
                    window.add_point_from_canvas(30, 40)

                    point = window.project.points[1]
                    self.assertEqual(point.type, POINT_TYPE_WAYPOINT)
                    self.assertEqual(point.yaw_ddeg, YAW_UNSPECIFIED_DDEG)
                    self.assertFalse(point.exact_pass)
                    self.assertEqual(point.corner_trim_mm, 200)
                finally:
                    window.close()

    def test_edit_removes_curve_but_keeps_editable_points(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            points = window.project.points
            window.plan_result = object()
            with patch.object(window, "refresh_field") as refresh_field:
                window.schedule_plan()

            self.assertIsNone(window.plan_result)
            self.assertIs(window.project.points, points)
            refresh_field.assert_called_once()
        finally:
            window.close()

    def test_traj_id_is_both_editable_and_selectable(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            combo = window.traj_id_combo
            self.assertTrue(combo.isEditable())

            combo.setEditText("127")
            window._commit_traj_id_selection()  # noqa: SLF001
            self.assertEqual(window.project.traj_id, 127)
            self.assertEqual(combo.currentText(), "P0127")

            combo.setCurrentIndex(combo.findData(359))
            window._traj_id_combo_activated(combo.currentIndex())  # noqa: SLF001
            self.assertEqual(window.project.traj_id, 359)
            self.assertEqual(combo.currentText(), "P0359")
        finally:
            window.close()

    def test_missing_selected_traj_id_does_not_fall_back_to_zero(self) -> None:
        case_zero = object()
        state = LoadedProjectState(
            layout=None,  # type: ignore[arg-type]
            project=None,  # type: ignore[arg-type]
            route_table=None,
            leg_library=None,
            manual_cases={0: case_zero},  # type: ignore[dict-item]
        )

        self.assertIsNone(state.current_case(127, GenerationMode.MANUAL))
        self.assertIs(state.current_case(None, GenerationMode.MANUAL), case_zero)

        window = V35ExactV4MainWindow()
        try:
            window._v4_state = state  # noqa: SLF001
            window.traj_id_combo.setEditText("127")
            window._commit_traj_id_selection()  # noqa: SLF001
            self.assertEqual(window.project.traj_id, 127)
            self.assertEqual(window.traj_id_combo.currentText(), "P0127")
        finally:
            window.close()

    def test_fixed_site_page_has_whole_page_scroll_bars(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            scroll = window.fixed_page_scroll
            self.assertIsInstance(scroll, QScrollArea)
            self.assertEqual(scroll.horizontalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
            self.assertEqual(scroll.verticalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
            self.assertGreaterEqual(scroll.widget().minimumWidth(), 1000)
        finally:
            window.close()

    def test_base_field_station_constants_match_current_layout(self) -> None:
        self.assertEqual(
            tuple((station_id, x_mm, y_mm) for station_id, x_mm, y_mm, _w, _h in PICKUP_STATIONS),
            ((1, 1855, 500), (2, 1600, 0), (3, 1855, -500)),
        )
        self.assertEqual(
            tuple((station_id, x_mm, y_mm) for station_id, x_mm, y_mm, _w, _h in DROP_STATIONS),
            (
                (4, -1640, 875),
                (5, -1875, 400),
                (6, -1875, 0),
                (7, -1875, -400),
                (8, -1640, -875),
            ),
        )

    def test_fixed_site_collision_controls_exist_with_initial_status(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            self.assertEqual(window.fixed_site_collision_button.text(), "检测固定点碰撞")
            self.assertEqual(window.fixed_site_collision_status_label.text(), "固定点碰撞：未检测")
        finally:
            window.close()

    def test_fixed_site_collision_click_uses_unsaved_sites_without_save_or_worker(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._v4_state = LoadedProjectState(  # noqa: SLF001
                layout=None,  # type: ignore[arg-type]
                project=minimal_project(),
                route_table=None,
                leg_library=None,
            )
            window.project.fixed_sites[0].x_mm = 1234
            window.project.fixed_sites[0].y_mm = 567
            window.project.fixed_sites[0].yaw_ddeg = 890
            captured: dict[str, ProjectV40] = {}

            def fake_check(project: ProjectV40) -> FixedSiteCollisionReport:
                captured["project"] = project
                return FixedSiteCollisionReport(FixedSiteCollisionResult.PASSED, entries=())

            with (
                patch("hjmb_pathgen.py_ui.v35_exact_main_window.check_fixed_site_collisions", side_effect=fake_check),
                patch.object(window, "_show_fixed_site_collision_dialog"),
                patch.object(window, "_save_project_config") as save_project_config,
                patch.object(window, "schedule_plan") as schedule_plan,
                patch.object(window, "_start_worker") as start_worker,
            ):
                window.fixed_site_collision_button.click()

            self.assertEqual(captured["project"].sites["P_START"]["x_mm"], 1234)
            self.assertEqual(captured["project"].sites["P_START"]["y_mm"], 567)
            self.assertEqual(captured["project"].sites["P_START"]["yaw_ddeg"], 890)
            save_project_config.assert_not_called()
            schedule_plan.assert_not_called()
            start_worker.assert_not_called()
        finally:
            window.close()

    def test_fixed_site_collision_status_label_distinguishes_results(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._v4_state = LoadedProjectState(  # noqa: SLF001
                layout=None,  # type: ignore[arg-type]
                project=minimal_project(),
                route_table=None,
                leg_library=None,
            )
            cases = (
                (FixedSiteCollisionResult.PASSED, "通过"),
                (FixedSiteCollisionResult.FAILED, "发现碰撞"),
                (FixedSiteCollisionResult.INCOMPLETE, "检测不完整"),
            )
            for result, expected_text in cases:
                with self.subTest(result=result):
                    report = FixedSiteCollisionReport(result, entries=())
                    with (
                        patch("hjmb_pathgen.py_ui.v35_exact_main_window.check_fixed_site_collisions", return_value=report),
                        patch.object(window, "_show_fixed_site_collision_dialog"),
                    ):
                        window.fixed_site_collision_button.click()
                    self.assertIn(expected_text, window.fixed_site_collision_status_label.text())
        finally:
            window.close()

    def test_fixed_site_collision_dialog_uses_scrollable_full_detail_text(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            entries = tuple(
                FixedSiteCollisionEntry(
                    site_key=f"P_DROP_{index}",
                    profile_id=f"DROP_PROFILE_{index}",
                    x_mm=float(index),
                    y_mm=float(index + 100),
                    yaw_ddeg=float(index * 10),
                    checked=True,
                    passed=True,
                    min_signed_clearance_mm=float(index + 200),
                    closest_obstacle_id=f"OBSTACLE_{index}",
                )
                for index in range(30)
            )
            report = FixedSiteCollisionReport(FixedSiteCollisionResult.PASSED, entries=entries)

            dialog = window._build_fixed_site_collision_dialog(report)  # noqa: SLF001
            try:
                detail = dialog.findChild(QPlainTextEdit, "fixed_site_collision_detail_text")
                self.assertIsNotNone(detail)
                assert detail is not None
                self.assertTrue(detail.isReadOnly())
                self.assertEqual(detail.verticalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
                self.assertIn("DROP_PROFILE_0", detail.toPlainText())
                self.assertIn("DROP_PROFILE_29", detail.toPlainText())
                self.assertGreaterEqual(len(detail.toPlainText().splitlines()), 60)
            finally:
                dialog.close()
        finally:
            window.close()

    def test_semi_arrival_defaults_to_0xff_unselected(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._generation_mode = GenerationMode.SEMI_AUTO  # noqa: SLF001
            window.project.path_mode = PATH_MODE_FIXED_8
            window.project.points = [
                EditPoint(point_id=0, type=POINT_TYPE_START, site_id=0),
                EditPoint(point_id=1, type=POINT_TYPE_WAYPOINT, site_id=SITE_ID_FREE),
            ]
            window.on_point_type_changed(1, POINT_TYPE_ARRIVAL)
            self.assertEqual(window.project.points[1].site_id, SITE_ID_FREE)
            combo = window.point_table.cellWidget(1, 2)
            self.assertIsInstance(combo, QComboBox)
            assert isinstance(combo, QComboBox)
            self.assertEqual(combo.currentData(), SITE_ID_FREE)
            self.assertIn("0xFF", combo.currentText())
        finally:
            window.close()

    def test_point_delete_keeps_action_bound_to_same_logical_point(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window.project.points = [
                EditPoint(point_id=0, type=POINT_TYPE_START),
                EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL),
                EditPoint(point_id=2, type=POINT_TYPE_ARRIVAL),
                EditPoint(point_id=3, type=POINT_TYPE_ARRIVAL),
            ]
            target = window.project.points[3]
            window.project.actions = [MechanicalAction(action_seq=0, arrival_point_id=3)]
            window.refresh_all()
            window.point_table.selectRow(1)
            window.delete_point()
            self.assertIs(window.project.points[2], target)
            self.assertEqual(window.project.actions[0].arrival_point_id, 2)
        finally:
            window.close()

    def test_semi_unload_selector_uses_short_readable_labels(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._generation_mode = GenerationMode.SEMI_AUTO  # noqa: SLF001
            window.project.path_mode = PATH_MODE_FIXED_8
            window.project.points = [
                EditPoint(point_id=0, type=POINT_TYPE_START, site_id=0),
                EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL, site_id=7),
            ]
            window.use_unload_pose_profiles_check.setChecked(True)
            window.refresh_point_table(1)
            combo = window.point_table.cellWidget(1, 5)
            self.assertIsInstance(combo, QComboBox)
            assert isinstance(combo, QComboBox)
            labels = [combo.itemText(index) for index in range(combo.count())]
            self.assertTrue(any("储箱" in label and "号箱" in label for label in labels))
            self.assertFalse(any("yaw=" in label for label in labels))
            self.assertGreaterEqual(combo.minimumWidth(), 250)
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
