from __future__ import annotations

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))


class ImportSmokeTest(unittest.TestCase):
    def test_v40_and_legacy_imports(self):
        import hjmb_path_editor
        import hjmb_pathgen
        from hjmb_pathgen.codec.binary_layout import HEADER_FMT
        from hjmb_pathgen.models.project import ProjectV40

        self.assertTrue(HEADER_FMT.startswith("<4s"))
        self.assertIsNotNone(ProjectV40)
        self.assertTrue(hasattr(hjmb_path_editor, "MainWindow"))
        self.assertEqual(hjmb_pathgen.__version__, "4.0.0")


if __name__ == "__main__":
    unittest.main()
