from __future__ import annotations

import unittest
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))


class ImportSmokeTest(unittest.TestCase):
    def test_v40_entry_and_py_packages_import_without_legacy_runtime(self):
        code = """
import json, sys
import hjmb_path_editor, hjmb_pathgen
import hjmb_pathgen.py_app, hjmb_pathgen.py_domain, hjmb_pathgen.py_io
import hjmb_pathgen.py_planning, hjmb_pathgen.py_services, hjmb_pathgen.py_ui
import hjmb_pathgen.py_utils, hjmb_pathgen.py_workers
from hjmb_pathgen.py_io.codecs.binary_layout import HEADER_FMT
from hjmb_pathgen.py_domain.project import ProjectV40
print(json.dumps({
    "header": HEADER_FMT,
    "project": ProjectV40.__name__,
    "entry": callable(hjmb_path_editor.main),
    "legacy": [name for name in sys.modules if name.startswith("hjmb_pathgen.py_legacy")],
    "version": hjmb_pathgen.__version__,
}))
"""
        completed = subprocess.run(
            [sys.executable, "-c", code],
            cwd=Path(__file__).resolve().parents[2],
            check=True,
            capture_output=True,
            text=True,
        )
        result = json.loads(completed.stdout)
        self.assertTrue(result["header"].startswith("<4s"))
        self.assertEqual(result["project"], "ProjectV40")
        self.assertTrue(result["entry"])
        self.assertEqual(result["legacy"], [])
        self.assertEqual(result["version"], "4.0.0")


if __name__ == "__main__":
    unittest.main()
