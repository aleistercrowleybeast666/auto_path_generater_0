from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.codec.json_codec import load_project
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.services.project_service import ProjectLayout
from hjmb_pathgen.services.site_preset_service import apply_site_pose_preset, export_site_pose_preset, import_site_pose_preset_preview

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


class Phase4SitePresetTest(unittest.TestCase):
    def test_export_preview_and_apply_site_pose_preset(self):
        project = ProjectV40.from_dict(json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8")))
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            preset_path = export_site_pose_preset(layout, "measured_a", notes="fixture")
            self.assertTrue(preset_path.exists())
            preview = import_site_pose_preset_preview(layout, preset_path)
            self.assertEqual(preview.diffs, ())

            data = json.loads(preset_path.read_text(encoding="utf-8"))
            data["sites"]["P_START"]["x_mm"] = 42
            preset_path.write_text(json.dumps(data, ensure_ascii=False, sort_keys=True, indent=2) + "\n", encoding="utf-8")
            preview = import_site_pose_preset_preview(layout, preset_path)
            self.assertEqual(len(preview.diffs), 1)
            self.assertEqual(preview.diffs[0].site_key, "P_START")

            result = apply_site_pose_preset(layout, preset_path)
            self.assertTrue(result.to_dict()["applied"])
            updated = load_project(layout.project_json)
            self.assertEqual(updated.sites["P_START"]["x_mm"], 42)


if __name__ == "__main__":
    unittest.main()
