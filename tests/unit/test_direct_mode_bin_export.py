from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtWidgets import QApplication

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_ui.v35_exact_main_window import V35ExactV4MainWindow


class DirectModeBinExportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_export_overwrites_authoritative_mode_path_without_dialog(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            with tempfile.TemporaryDirectory() as directory:
                layout = ProjectLayout.open(directory, create_dirs=True)
                layout.ensure_directories()
                window._v4_state = SimpleNamespace(layout=layout)  # noqa: SLF001
                window._v4_dirty = False  # noqa: SLF001
                for mode, expected_parent in (
                    (GenerationMode.MANUAL, "manual"),
                    (GenerationMode.SEMI_AUTO, "semi_auto"),
                    (GenerationMode.FULL_AUTO, "full_auto"),
                ):
                    with self.subTest(mode=mode):
                        target = layout.bin_path_for_mode(42, mode)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_bytes(b"existing")
                        compiled = object()
                        window._generation_mode = mode  # noqa: SLF001
                        with (
                            patch.object(window, "_ensure_v4_workspace", return_value=True),
                            patch.object(window, "_current_traj_id", return_value=42),
                            patch.object(window, "update_status"),
                            patch("hjmb_pathgen.py_ui.v35_exact_main_window.load_bin", return_value=compiled),
                            patch("hjmb_pathgen.py_ui.v35_exact_main_window.save_bin") as save,
                            patch("hjmb_pathgen.py_ui.v35_exact_main_window.QFileDialog.getSaveFileName", side_effect=AssertionError("must not show dialog")),
                        ):
                            window.export_bin()
                        self.assertEqual(target.parent.name, expected_parent)
                        save.assert_called_once_with(target, compiled)
        finally:
            window._v4_state = None  # noqa: SLF001
            window.close()


if __name__ == "__main__":
    unittest.main()
