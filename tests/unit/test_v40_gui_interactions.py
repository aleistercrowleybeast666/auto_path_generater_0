from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication

from hjmb_pathgen.services.example_project_service import create_synthetic_example_project
from hjmb_pathgen.ui.field_view import FIELD_SCALE
from hjmb_pathgen.ui.graphics_items import DragCommit, YawCommit
from hjmb_pathgen.ui.main_window import V4MainWindow


class V40GuiInteractionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.source_traj = Path(__file__).resolve().parents[2] / "traj_id.csv"

    def _window_with_project(self):
        tmp = tempfile.TemporaryDirectory()
        root = Path(tmp.name) / "project"
        create_synthetic_example_project(root, source_traj_csv=self.source_traj, generate_outputs=False)
        window = V4MainWindow(root)
        self.addCleanup(window.close)
        self.addCleanup(tmp.cleanup)
        return window

    def test_open_project_populates_real_field_models(self):
        window = self._window_with_project()
        self.assertEqual(window.tabs.count(), 9)
        self.assertFalse(window.cancel_button.isEnabled())
        self.assertTrue(window.log_dock.isHidden())
        self.assertEqual(window.project_sites_tab.model.rowCount(), 10)
        self.assertEqual(window.route_leg_tab.model.rowCount(), 66)
        self.assertEqual(window.task_cases_tab.model.rowCount(), 360)
        view = window.project_sites_tab.field_view
        self.assertAlmostEqual(view.world_to_scene(-2000, 1000).x(), 0.0)
        self.assertAlmostEqual(view.world_to_scene(-2000, 1000).y(), 0.0)
        self.assertAlmostEqual(view.world_to_scene(2000, -1000).x(), 4000 * FIELD_SCALE)
        self.assertAlmostEqual(view.world_to_scene(2000, -1000).y(), 2000 * FIELD_SCALE)
        self.assertEqual(view.scene_to_world_int(view.world_to_scene(123, -456)), (123, -456))
        scene_rect = view.scene_obj.sceneRect()
        self.assertGreater(scene_rect.width(), 1000)
        self.assertLess(scene_rect.width(), 1120)
        self.assertGreater(scene_rect.width(), scene_rect.height() * 1.75)
        dump = window.scene_dumps()["project_sites"]
        self.assertEqual(dump["field_boundary_count"], 1)
        self.assertGreaterEqual(dump["grid_line_count"], 20)
        self.assertEqual(dump["cylinder_count"], 2)
        self.assertEqual(dump["pickup_box_count"], 3)
        self.assertEqual(dump["drop_box_count"], 5)
        self.assertEqual(dump["site_count"], 10)
        self.assertEqual(dump["site_yaw_handle_count"], 5)

    def test_fixed_site_double_click_drag_yaw_and_no_worker(self):
        window = self._window_with_project()
        tab = window.project_sites_tab
        tab.select_site("P_START")
        tab.set_site_from_world("P_START", 123, 456)
        site = window._state.project.sites["P_START"]  # noqa: SLF001 - interaction regression test.
        self.assertEqual((site["x_mm"], site["y_mm"], site["configured"]), (123, 456, True))
        self.assertIsNone(window._worker)  # noqa: SLF001
        self.assertEqual(window.dirty_status.text(), "dirty / STALE")

        tab._site_position_preview("P_START", 150, 470)  # noqa: SLF001
        tab._site_position_committed(DragCommit("P_START", 123, 456, 150, 470))  # noqa: SLF001
        self.assertEqual((site["x_mm"], site["y_mm"]), (150, 470))

        tab._site_yaw_preview("P_START", 900)  # noqa: SLF001
        tab._site_yaw_committed(YawCommit("P_START", 0, 900))  # noqa: SLF001
        self.assertEqual(site["yaw_ddeg"], 900)
        self.assertNotIn("F_DROP_4", tab.field_view.site_yaw_items)

        tab.undo_stack.undo()
        self.assertEqual(site["yaw_ddeg"], 0)
        tab.undo_stack.redo()
        self.assertEqual(site["yaw_ddeg"], 900)

    def test_manual_free_sparse_path_interaction(self):
        window = self._window_with_project()
        tab = window.manual_free_tab
        tab.add_point("START", 0, 0)
        tab.add_point("WAYPOINT", 100, 100)
        tab.add_point("ARRIVAL", 200, 100)
        self.assertEqual(len(tab.points), 3)
        dump = window.scene_dumps()["manual_free"]
        self.assertEqual(dump["manual_point_count"], 3)
        self.assertEqual(dump["manual_yaw_handle_count"], 2)
        self.assertGreaterEqual(dump["sparse_path_count"], 2)

        tab._position_preview(1, 120, 130)  # noqa: SLF001
        tab._position_committed(DragCommit(1, 100, 100, 120, 130))  # noqa: SLF001
        self.assertEqual((tab.points[1].x_mm, tab.points[1].y_mm), (120, 130))
        tab._yaw_preview(2, -450)  # noqa: SLF001
        tab._yaw_committed(YawCommit(2, 0, -450))  # noqa: SLF001
        self.assertEqual(tab.points[2].yaw_ddeg, -450)
        self.assertIsNone(window._worker)  # noqa: SLF001

    def test_route_leg_visualization_scene_dump(self):
        window = self._window_with_project()
        dump = window.scene_dumps()["route_leg"]
        self.assertGreaterEqual(dump["dense_path_count"], 1)
        self.assertGreaterEqual(dump["speed_overlay_count"], 1)
        self.assertGreaterEqual(dump["collision_footprint_count"], 3)
        selected = window.route_leg_tab.selected_leg()
        self.assertIsNotNone(selected)
        if selected is not None:
            window.leg_library_tab.openLegRequested.emit(selected.leg_id)
            self.assertEqual(window.tabs.currentWidget(), window.route_leg_tab)


if __name__ == "__main__":
    unittest.main()
