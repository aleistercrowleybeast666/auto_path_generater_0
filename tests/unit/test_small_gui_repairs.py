from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication, QComboBox

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_ui.v35_base.editor import MainWindow as V35BaseMainWindow
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


if __name__ == "__main__":
    unittest.main()
