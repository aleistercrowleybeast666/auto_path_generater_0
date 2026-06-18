# -*- coding: utf-8 -*-
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import hjmb_path_editor as editor
from path_models import (
    PATH_MODE_FIXED_8,
    PATH_MODE_FREE,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    SITE_ID_FREE,
    YAW_UNSPECIFIED_DDEG,
    resolve_edit_points,
)
from v35_test_utils import make_straight_project


class HJMBPathEditorV35Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = editor.MainWindow()
        self.window.project = make_straight_project()
        self.window.refresh_all()

    def tearDown(self):
        self.window.close()

    def _combo_values(self, table, row: int, column: int):
        combo = table.cellWidget(row, column)
        return [combo.itemData(index) for index in range(combo.count())]

    def test_gui_point_columns_no_legacy_stop_gate_flags(self):
        columns = [
            self.window.point_table.horizontalHeaderItem(index).text()
            for index in range(self.window.point_table.columnCount())
        ]
        self.assertEqual(columns, list(editor.POINT_TABLE_COLUMNS))
        self.assertNotIn("stop", columns)
        self.assertNotIn("gate", columns)

    def test_gui_action_columns_use_v35_auto_start(self):
        columns = [
            self.window.action_table.horizontalHeaderItem(index).text()
            for index in range(self.window.action_table.columnCount())
        ]
        self.assertEqual(columns, list(editor.ACTION_TABLE_COLUMNS))
        self.assertIn("post_wait_ms", columns)
        self.assertIn("auto_check_start_s", columns)
        self.assertIn("auto_execution_hint", columns)
        self.assertNotIn("arrival/trigger", columns)
        self.assertNotIn("min_wait", columns)
        self.assertNotIn("window_start", columns)
        self.assertNotIn("window_end", columns)

    def test_fixed_site_table_has_exactly_8_rows(self):
        self.assertEqual(self.window.fixed_site_table.rowCount(), 8)

    def test_fixed_8_start_arrival_coordinates_are_read_only(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.points[0].type = POINT_TYPE_START
        self.window.project.points[0].site_id = 0
        self.window.project.points[1].type = POINT_TYPE_ARRIVAL
        self.window.project.points[1].site_id = 1
        self.window.refresh_all()
        self.assertFalse(
            bool(self.window.point_table.item(0, 3).flags() & editor.Qt.ItemIsEditable)
        )
        self.assertFalse(
            bool(self.window.point_table.item(1, 5).flags() & editor.Qt.ItemIsEditable)
        )

    def test_free_start_arrival_coordinates_are_editable(self):
        self.window.project.path_mode = PATH_MODE_FREE
        self.window.refresh_all()
        self.assertTrue(
            bool(self.window.point_table.item(0, 3).flags() & editor.Qt.ItemIsEditable)
        )
        self.assertTrue(
            bool(self.window.point_table.item(1, 5).flags() & editor.Qt.ItemIsEditable)
        )

    def test_switch_to_fixed_8_keeps_arrival_position(self):
        self.window.project.points[1].x_mm = 1234
        self.window.project.points[1].y_mm = 321
        self.window.project.points[1].yaw_ddeg = 450
        self.window.path_mode_combo.setCurrentIndex(
            self.window.path_mode_combo.findData(PATH_MODE_FIXED_8)
        )
        resolved = resolve_edit_points(self.window.project)
        self.assertEqual((resolved[1].x_mm, resolved[1].y_mm, resolved[1].yaw_ddeg), (1234, 321, 450))
        self.assertNotEqual((self.window.project.fixed_sites[1].x_mm, self.window.project.fixed_sites[1].y_mm), (0, 0))

    def test_fixed_site_selection_moves_point_to_site_without_overwriting_site(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.fixed_sites[1].x_mm = 800
        self.window.project.fixed_sites[1].y_mm = -120
        self.window.project.fixed_sites[1].yaw_ddeg = 300
        self.window.project.points[1].site_id = SITE_ID_FREE
        self.window.project.points[1].x_mm = 1234
        self.window.project.points[1].y_mm = 321
        self.window.refresh_all(selected_point=1)

        combo = self.window.point_table.cellWidget(1, 2)
        combo.setCurrentIndex(combo.findData(1))

        self.assertEqual(self.window.project.points[1].site_id, 1)
        self.assertEqual((self.window.project.fixed_sites[1].x_mm, self.window.project.fixed_sites[1].y_mm), (800, -120))
        self.assertEqual((self.window.project.points[1].x_mm, self.window.project.points[1].y_mm), (800, -120))
        resolved = resolve_edit_points(self.window.project)
        self.assertEqual((resolved[1].x_mm, resolved[1].y_mm, resolved[1].yaw_ddeg), (800, -120, 300))

    def test_arrival_site_combo_has_unset_default(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.points[1].site_id = SITE_ID_FREE
        self.window.refresh_all(selected_point=1)
        self.assertEqual(self._combo_values(self.window.point_table, 1, 2)[0], SITE_ID_FREE)

    def test_fixed_sites_are_drawn_on_field(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.refresh_all()
        tooltips = [item.toolTip() for item in self.window.field.scene_obj.items()]
        self.assertTrue(any("0 P_START" in tooltip for tooltip in tooltips))
        self.assertTrue(any("1 P_PICK_1" in tooltip for tooltip in tooltips))

    def test_fixed_site_with_yaw_uses_arrow_handle_when_selected(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.fixed_sites[1].yaw_ddeg = 300
        self.window.refresh_all()
        self.window.right_tabs.setCurrentWidget(self.window.fixed_site_tab)
        self.window.fixed_site_table.selectRow(1)
        self.window.refresh_field()

        self.assertIn(1, self.window.field.fixed_site_yaw_line_items)
        self.assertIn(1, self.window.field.fixed_site_yaw_handle_items)

    def test_fixed_site_yaw_ff_keeps_cross_without_arrow_handle(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.fixed_sites[5].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        self.window.refresh_all()
        self.window.right_tabs.setCurrentWidget(self.window.fixed_site_tab)
        self.window.fixed_site_table.selectRow(5)
        self.window.refresh_field()

        self.assertNotIn(5, self.window.field.fixed_site_yaw_line_items)
        self.assertNotIn(5, self.window.field.fixed_site_yaw_handle_items)

    def test_drag_fixed_site_yaw_handle_updates_site_and_references(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.fixed_sites[1].x_mm = 0
        self.window.project.fixed_sites[1].y_mm = 0
        self.window.project.fixed_sites[1].yaw_ddeg = 0
        self.window.project.points[1].site_id = 1
        self.window.refresh_all()
        self.window.right_tabs.setCurrentWidget(self.window.fixed_site_tab)
        self.window.fixed_site_table.selectRow(1)
        self.window.refresh_field()

        handle = self.window.field.fixed_site_yaw_handle_items[1]
        handle.setPos(self.window.field.world_to_scene(0, editor.YAW_ARROW_LENGTH_MM))

        self.assertEqual(self.window.project.fixed_sites[1].yaw_ddeg, 900)
        self.assertEqual(self.window.project.points[1].yaw_ddeg, 900)
        self.assertEqual(self.window.fixed_site_table.item(1, 4).text(), "900")

    def test_fixed_site_tab_double_click_route_moves_selected_site(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.refresh_all()
        self.window.right_tabs.setCurrentWidget(self.window.fixed_site_tab)
        self.window.fixed_site_table.selectRow(1)
        point_count = len(self.window.project.points)

        self.window.add_point_from_canvas(333, -222)

        self.assertEqual(len(self.window.project.points), point_count)
        self.assertEqual((self.window.project.fixed_sites[1].x_mm, self.window.project.fixed_sites[1].y_mm), (333, -222))

    def test_drop_yaw_unspecified_unlocks_arrival_yaw_override(self):
        self.window.project.path_mode = PATH_MODE_FIXED_8
        self.window.project.fixed_sites[5].x_mm = 1200
        self.window.project.fixed_sites[5].y_mm = 400
        self.window.project.fixed_sites[5].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        self.window.project.points[1].site_id = 5
        self.window.project.points[1].yaw_ddeg = 450
        self.window.refresh_all(selected_point=1)

        self.assertFalse(
            bool(self.window.point_table.item(1, 3).flags() & editor.Qt.ItemIsEditable)
        )
        self.assertTrue(
            bool(self.window.point_table.item(1, 5).flags() & editor.Qt.ItemIsEditable)
        )
        resolved = resolve_edit_points(self.window.project)
        self.assertEqual((resolved[1].x_mm, resolved[1].y_mm, resolved[1].yaw_ddeg), (1200, 400, 450))
        exported = self.window.project.to_config_dict()
        self.assertEqual(exported["points"][1]["yaw_ddeg"], 450)

    def test_start_type_is_locked_to_first_row(self):
        first_values = self._combo_values(self.window.point_table, 0, 1)
        second_values = self._combo_values(self.window.point_table, 1, 1)
        self.assertEqual(first_values, [POINT_TYPE_START])
        self.assertNotIn(POINT_TYPE_START, second_values)
        self.assertIn(POINT_TYPE_WAYPOINT, second_values)
        self.assertIn(POINT_TYPE_ARRIVAL, second_values)

    def test_insert_before_start_inserts_after_start(self):
        self.window.point_table.selectRow(0)
        self.window.insert_point()
        self.assertEqual(len(self.window.project.points), 3)
        self.assertEqual(self.window.project.points[0].type, POINT_TYPE_START)
        self.assertEqual(self.window.project.points[0].point_id, 0)
        self.assertEqual(self.window.project.points[1].type, POINT_TYPE_WAYPOINT)

    def test_start_cannot_be_deleted_or_moved(self):
        original_types = [point.type for point in self.window.project.points]
        self.window.point_table.selectRow(0)
        self.window.delete_point()
        self.assertEqual([point.type for point in self.window.project.points], original_types)

        self.window.move_point(1)
        self.assertEqual([point.type for point in self.window.project.points], original_types)

        self.window.point_table.selectRow(1)
        self.window.move_point(-1)
        self.assertEqual([point.type for point in self.window.project.points], original_types)

    def test_site_header_explains_free_mode_gray_state(self):
        tooltip = self.window.point_table.horizontalHeaderItem(2).toolTip()
        self.assertIn("FREE", tooltip)
        self.assertIn("灰显", tooltip)

    def test_auto_action_columns_are_read_only(self):
        self.window.add_action()
        self.window.refresh_all(selected_action=0)
        self.assertFalse(
            bool(self.window.action_table.item(0, 11).flags() & editor.Qt.ItemIsEditable)
        )
        self.assertFalse(
            bool(self.window.action_table.item(0, 12).flags() & editor.Qt.ItemIsEditable)
        )


if __name__ == "__main__":
    unittest.main()
