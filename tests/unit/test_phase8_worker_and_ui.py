from __future__ import annotations

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication, QTabWidget

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_services.mode_output_service import export_final_bin
from hjmb_pathgen.py_workers.worker_process import start_worker_job
from hjmb_pathgen.py_ui.main_window import V4MainWindow

from tests.unit.test_phase7_generation import collect_unique_legs, populate_library_for_collection, write_one_row_project


class Phase8WorkerAndUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_v4_ui_has_two_core_pages_and_three_modes_without_running_worker(self):
        window = V4MainWindow()
        try:
            tabs = window.findChild(QTabWidget)
            self.assertIsNotNone(tabs)
            assert tabs is not None
            self.assertEqual(tabs.count(), 2)
            self.assertEqual(
                [tabs.tabText(index) for index in range(tabs.count())],
                ["路径编辑", "最优路段与批量生成"],
            )
            self.assertEqual(
                [window.mode_combo.itemData(index) for index in range(window.mode_combo.count())],
                ["MANUAL", "SEMI_AUTO", "FULL_AUTO"],
            )
            self.assertIsNone(window._worker)  # noqa: SLF001 - GUI regression assertion.
        finally:
            window.close()

    def test_final_export_blocks_unapproved_generated_case(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())
            from hjmb_pathgen.py_services.phase7_generation_service import generate_one

            generate_one(layout, 0)
            with self.assertRaisesRegex(Exception, "approved"):
                export_final_bin(layout, 0, generation_mode=GenerationMode.FULL_AUTO)

    def test_worker_process_reports_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            handle = start_worker_job(layout.root, "validate-all", {})
            deadline = time.time() + 20.0
            messages = []
            while time.time() < deadline and handle.is_alive():
                messages.extend(handle.poll())
                time.sleep(0.05)
            handle.join(5.0)
            messages.extend(handle.poll())
            kinds = {message.kind for message in messages}
            self.assertTrue({"result", "error"} & kinds)


if __name__ == "__main__":
    unittest.main()
