from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication, QComboBox

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_ui.v35_base.editor import MainWindow as V35BaseMainWindow
from hjmb_pathgen.py_ui.v35_base.path_models import (
    PATH_MODE_FIXED_8,
    PATH_MODE_FREE,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)
from hjmb_pathgen.py_ui.v35_exact_main_window import V35ExactV4MainWindow


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


if __name__ == "__main__":
    unittest.main()
