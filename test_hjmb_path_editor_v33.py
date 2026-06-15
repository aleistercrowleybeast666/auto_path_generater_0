# -*- coding: utf-8 -*-
import os
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import hjmb_path_editor as editor
from path_models import MechanicalAction
from v33_test_utils import make_curve_project


class HjmbPathEditorV33Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = editor.MainWindow()
        self.window.plan_timer.stop()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()

    def load_drag_fixture(self):
        self.window.project = make_curve_project()
        self.window.refresh_all()
        self.window.plan_now()

    def test_startup_project_is_empty(self):
        self.assertEqual(self.window.project.points, [])
        self.assertEqual(self.window.project.actions, [])
        self.assertIsNone(self.window.plan_result)
        self.assertEqual(self.window.plan_error, "")
        self.assertIn("等待添加路径点", self.window.status_label.text())

    def test_selecting_point_does_not_rebuild_scene(self):
        self.load_drag_fixture()
        point_item = self.window.field.point_items[1]
        self.window.point_table.selectRow(1)
        self.app.processEvents()
        self.assertIs(self.window.field.point_items[1], point_item)
        self.assertTrue(point_item.isSelected())

    def test_waypoint_move_updates_marker_and_control_lines_in_place(self):
        self.load_drag_fixture()
        field = self.window.field
        point_item = field.point_items[1]
        point_item.setPos(field.world_to_scene(-650, 40))
        self.app.processEvents()

        point = self.window.project.points[1]
        self.assertEqual((point.x_mm, point.y_mm), (-650, 40))
        self.assertNotIn(1, field.yaw_handle_items)
        self.assertNotIn(1, field.yaw_line_items)
        self.assertEqual(
            field.point_labels[1].pos(),
            point_item.pos() + editor.QPointF(8, -23),
        )

    def test_arrival_yaw_handle_updates_line_without_scene_rebuild(self):
        self.load_drag_fixture()
        field = self.window.field
        handle = field.yaw_handle_items[3]
        line = field.yaw_line_items[3]
        handle.setPos(
            field.world_to_scene(
                self.window.project.points[3].x_mm,
                self.window.project.points[3].y_mm + editor.YAW_ARROW_LENGTH_MM,
            )
        )
        self.app.processEvents()

        self.assertAlmostEqual(self.window.project.points[3].yaw_ddeg, 900, delta=1)
        self.assertAlmostEqual(line.line().p2().x(), handle.pos().x())
        self.assertAlmostEqual(line.line().p2().y(), handle.pos().y())

    def test_waypoint_yaw_is_read_only_ff(self):
        self.load_drag_fixture()
        self.assertEqual(self.window.point_table.item(1, 4).text(), "0xFF")
        self.assertFalse(
            bool(self.window.point_table.item(1, 4).flags() & editor.Qt.ItemIsEditable)
        )
        self.assertNotIn(1, self.window.field.yaw_handle_items)

    def test_clear_project_requires_confirmation_and_preserves_parameters(self):
        self.load_drag_fixture()
        self.window.project.actions.append(MechanicalAction())
        self.window.project.planner.max_speed_mmps = 1234
        self.window.current_json_path = Path("loaded.json")

        with patch.object(
            editor.QMessageBox, "question", return_value=editor.QMessageBox.No
        ):
            self.window.clear_project()
        self.assertTrue(self.window.project.points)
        self.assertTrue(self.window.project.actions)

        with patch.object(
            editor.QMessageBox, "question", return_value=editor.QMessageBox.Yes
        ):
            self.window.clear_project()
        self.assertEqual(self.window.project.points, [])
        self.assertEqual(self.window.project.actions, [])
        self.assertEqual(self.window.project.planner.max_speed_mmps, 1234)
        self.assertIsNone(self.window.current_json_path)
        self.assertIsNone(self.window.plan_result)
        self.assertEqual(self.window.point_table.rowCount(), 0)
        self.assertEqual(self.window.action_table.rowCount(), 0)
        self.assertEqual(self.window.field.point_items, {})


if __name__ == "__main__":
    unittest.main()
