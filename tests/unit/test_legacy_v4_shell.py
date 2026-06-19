from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsRectItem

from hjmb_pathgen.legacy.v35.path_models import (
    EditPoint,
    PATH_MODE_FREE,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
)
from hjmb_pathgen.ui.legacy_shell import LegacyV4MainWindow


class LegacyV4ShellTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_launcher_shell_keeps_legacy_editor_surface(self):
        window = LegacyV4MainWindow()
        self.addCleanup(window.close)

        self.assertTrue(hasattr(window, "point_table"))
        self.assertTrue(hasattr(window, "fixed_site_table"))
        self.assertTrue(hasattr(window, "action_table"))
        self.assertIn("V4 项目/手动输出", [window.right_tabs.tabText(i) for i in range(window.right_tabs.count())])
        self.assertIn("V4 生成/360", [window.right_tabs.tabText(i) for i in range(window.right_tabs.count())])

        rect_count = sum(isinstance(item, QGraphicsRectItem) for item in window.field.scene_obj.items())
        ellipse_count = sum(isinstance(item, QGraphicsEllipseItem) for item in window.field.scene_obj.items())
        self.assertGreaterEqual(rect_count, 11)
        self.assertGreaterEqual(ellipse_count, 2)

    def test_legacy_points_convert_to_v40_manual_free_case(self):
        window = LegacyV4MainWindow()
        self.addCleanup(window.close)
        window.project.points = [
            EditPoint(point_id=0, type=POINT_TYPE_START, x_mm=0, y_mm=0, yaw_ddeg=0),
            EditPoint(
                point_id=1,
                type=POINT_TYPE_WAYPOINT,
                x_mm=400,
                y_mm=100,
                yaw_ddeg=YAW_UNSPECIFIED_DDEG,
                max_speed_mmps=500,
            ),
            EditPoint(point_id=2, type=POINT_TYPE_ARRIVAL, x_mm=800, y_mm=0, yaw_ddeg=900),
        ]
        window.v4_traj_spin.setValue(7)

        case = window._v4_case_from_legacy_points()  # noqa: SLF001 - GUI shell regression test.

        self.assertEqual(case.traj_id, 7)
        self.assertEqual(case.path_source.value, "MANUAL_FREE")
        assert case.manual_path is not None
        self.assertEqual([point["type"] for point in case.manual_path["points"]], ["START", "WAYPOINT", "ARRIVAL"])
        self.assertEqual(case.manual_path["points"][1]["max_speed_mmps"], 500)
        self.assertEqual(case.manual_path["points"][2]["yaw_ddeg"], 900)

    def test_fixed_site_tab_forces_display_and_requires_selection_to_move(self):
        window = LegacyV4MainWindow()
        self.addCleanup(window.close)
        window.project.path_mode = PATH_MODE_FREE
        window.project.fixed_sites[1].x_mm = 100
        window.project.fixed_sites[1].y_mm = 200
        window.project.fixed_sites[1].yaw_ddeg = 900
        window.project.fixed_sites[5].x_mm = -100
        window.project.fixed_sites[5].y_mm = -200
        window.project.fixed_sites[5].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        window.refresh_fixed_site_table()

        window.right_tabs.setCurrentWidget(window.fixed_site_tab)
        window.fixed_site_table.selectRow(1)
        window.refresh_field()
        self.assertIn(1, window.field.fixed_site_yaw_handle_items)
        self.assertEqual(window.fixed_site_table.item(5, 4).text(), "×")

        before = (window.project.fixed_sites[1].x_mm, window.project.fixed_sites[1].y_mm)
        window.fixed_site_table.clearSelection()
        window.move_selected_fixed_site_to(333, 444)
        self.assertEqual((window.project.fixed_sites[1].x_mm, window.project.fixed_sites[1].y_mm), before)

        window.fixed_site_table.selectRow(1)
        window.move_selected_fixed_site_to(333, 444)
        self.assertEqual((window.project.fixed_sites[1].x_mm, window.project.fixed_sites[1].y_mm), (333, 444))

        window.fixed_site_table.selectRow(5)
        window.refresh_field()
        self.assertNotIn(5, window.field.fixed_site_yaw_handle_items)


if __name__ == "__main__":
    unittest.main()
