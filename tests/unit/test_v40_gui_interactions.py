from __future__ import annotations

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication, QPushButton

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_io.codecs.json_codec import load_case
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.case_draft_service import generate_case_draft
from hjmb_pathgen.py_services.example_project_service import create_synthetic_example_project
from hjmb_pathgen.py_services.mode_case_service import convert_full_auto_to_semi_auto
from hjmb_pathgen.py_ui.field_view import FIELD_SCALE
from hjmb_pathgen.py_ui.graphics_items import DragCommit, YawCommit
from hjmb_pathgen.py_ui.main_window import V4MainWindow


class V40GuiInteractionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls._template_tmp = tempfile.TemporaryDirectory()
        cls.template_root = Path(cls._template_tmp.name) / "template"
        source_traj = Path(__file__).resolve().parents[2] / "traj_id.csv"
        create_synthetic_example_project(
            cls.template_root,
            source_traj_csv=source_traj,
            generate_outputs=False,
        )
        generate_case_draft(ProjectLayout.open(cls.template_root), 0)

    @classmethod
    def tearDownClass(cls):
        cls._template_tmp.cleanup()

    def _window_with_project(self):
        tmp = tempfile.TemporaryDirectory()
        root = Path(tmp.name) / "project"
        shutil.copytree(self.template_root, root)
        window = V4MainWindow(root)
        self.addCleanup(window.close)
        self.addCleanup(tmp.cleanup)
        return window

    def test_open_project_populates_two_page_v4_field(self):
        window = self._window_with_project()
        self.assertEqual(window.tabs.count(), 2)
        self.assertFalse(window.cancel_button.isEnabled())
        self.assertTrue(window.log_dock.isHidden())
        self.assertEqual(len(window.context.state.project.sites), 8)
        self.assertGreater(window.optimization_batch_page.leg_table.rowCount(), 0)

        view = window.path_editor_page.field_view
        self.assertAlmostEqual(view.world_to_scene(-2000, 1000).x(), 0.0)
        self.assertAlmostEqual(view.world_to_scene(-2000, 1000).y(), 0.0)
        self.assertAlmostEqual(view.world_to_scene(2000, -1000).x(), 4000 * FIELD_SCALE)
        self.assertAlmostEqual(view.world_to_scene(2000, -1000).y(), 2000 * FIELD_SCALE)
        self.assertEqual(view.scene_to_world_int(view.world_to_scene(123, -456)), (123, -456))
        dump = window.scene_dumps()["path_editor"]
        self.assertEqual(dump["field_boundary_count"], 1)
        self.assertEqual(dump["cylinder_count"], 2)
        self.assertEqual(dump["pickup_box_count"], 3)
        self.assertEqual(dump["drop_box_count"], 5)

        button_texts = {button.text() for button in window.findChildren(QPushButton)}
        for expected in (
            "新建V4项目",
            "打开V4项目",
            "保存项目配置",
            "保存当前Case JSON",
            "生成/更新当前路径",
            "验证当前路径",
            "导出当前BIN",
            "设为最终版本",
        ):
            self.assertIn(expected, button_texts)
        self.assertFalse(any("V3.5" in text for text in button_texts))

    def test_manual_sparse_path_drag_yaw_marks_stale_without_worker(self):
        window = self._window_with_project()
        page = window.path_editor_page
        page.add_point("START", 0, 0)
        page.add_point("WAYPOINT", 100, 100)
        page.add_point("ARRIVAL", 200, 100)
        self.assertEqual(len(page.points), 3)
        dump = window.scene_dumps()["path_editor"]
        self.assertEqual(dump["manual_point_count"], 3)
        self.assertEqual(dump["manual_yaw_handle_count"], 2)

        page._position_preview(1, 120, 130)  # noqa: SLF001
        page._position_committed(DragCommit(1, 100, 100, 120, 130))  # noqa: SLF001
        self.assertEqual((page.points[1].x_mm, page.points[1].y_mm), (120, 130))
        page._yaw_preview(2, -450)  # noqa: SLF001
        page._yaw_committed(YawCommit(2, 0, -450))  # noqa: SLF001
        self.assertEqual(page.points[2].yaw_ddeg, -450)
        self.assertIsNone(window._worker)  # noqa: SLF001
        self.assertEqual(window.dirty_status.text(), "dirty / STALE")

    def test_full_auto_is_read_only_and_converts_to_semi_auto_copy(self):
        window = self._window_with_project()
        window._set_mode_combo(GenerationMode.FULL_AUTO)  # noqa: SLF001
        window._sync_mode_and_traj()  # noqa: SLF001
        page = window.path_editor_page
        self.assertFalse(page.field_view.editable)
        self.assertEqual(len(page.points), 8)
        full_path = window.context.state.layout.case_json_path_for_mode(0, GenerationMode.FULL_AUTO)
        full_before = full_path.read_bytes()

        converted = convert_full_auto_to_semi_auto(window.context.state.layout, 0)
        self.assertEqual(converted.generation_mode, GenerationMode.SEMI_AUTO)
        self.assertEqual(len(converted.logical_points), 8)
        self.assertEqual(converted.derived_from["generation_mode"], "FULL_AUTO")
        self.assertEqual(full_path.read_bytes(), full_before)
        semi_path = window.context.state.layout.case_json_path_for_mode(0, GenerationMode.SEMI_AUTO)
        self.assertTrue(semi_path.exists())
        self.assertEqual(load_case(semi_path).review["state"], "STALE")

        window.load_project_path(window.context.state.layout.root)
        window._set_mode_combo(GenerationMode.SEMI_AUTO)  # noqa: SLF001
        window._sync_mode_and_traj()  # noqa: SLF001
        self.assertTrue(window.path_editor_page.field_view.editable)
        self.assertEqual(len(window.path_editor_page.points), 8)


if __name__ == "__main__":
    unittest.main()
