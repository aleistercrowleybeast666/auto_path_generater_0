from __future__ import annotations

import json
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, PortableCaseV40, RouteCaseTableV40

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures"


def load_json(path: str) -> dict:
    return json.loads((FIXTURE_ROOT / path).read_text(encoding="utf-8"))


class V40ModelsTest(unittest.TestCase):
    def test_minimal_project_round_trip(self):
        model = ProjectV40.from_dict(load_json("v40/minimal_project.json"))
        self.assertEqual(ProjectV40.from_dict(model.to_dict()), model)

    def test_minimal_route_case_table_round_trip(self):
        model = RouteCaseTableV40.from_dict(load_json("v40/minimal_route_case_table.json"))
        self.assertEqual(RouteCaseTableV40.from_dict(model.to_dict()), model)

    def test_minimal_leg_library_round_trip(self):
        model = LegLibraryV40.from_dict(load_json("v40/minimal_leg_library.json"))
        self.assertEqual(LegLibraryV40.from_dict(model.to_dict()), model)

    def test_minimal_case_round_trip(self):
        model = CaseManifestV40.from_dict(load_json("v40/minimal_case.json"))
        self.assertEqual(CaseManifestV40.from_dict(model.to_dict()), model)

    def test_minimal_portable_case_round_trip(self):
        model = PortableCaseV40.from_dict(load_json("v40/minimal_portable_case.json"))
        self.assertEqual(PortableCaseV40.from_dict(model.to_dict()), model)

    def test_unknown_deleted_and_legacy_fields_rejected(self):
        data = load_json("v40/minimal_project.json")
        data["unexpected"] = True
        with self.assertRaisesRegex(V40ValidationError, "unknown"):
            ProjectV40.from_dict(data)

        data = load_json("v40/minimal_case.json")
        data["actions"]["source"] = [{"trigger_s_mm": 10}]
        with self.assertRaisesRegex(V40ValidationError, "trigger_s_mm"):
            CaseManifestV40.from_dict(data)

        with self.assertRaisesRegex(V40ValidationError, "legacy V3"):
            ProjectV40.from_dict(load_json("legacy/v35_minimal.json"))

    def test_integer_range_overflow_rejected(self):
        data = load_json("v40/minimal_route_case_table.json")
        data["cases"][0]["traj_id"] = 360
        data["cases"][0]["file_name"] = "P0360.BIN"
        with self.assertRaisesRegex(V40ValidationError, "traj_id"):
            RouteCaseTableV40.from_dict(data)


if __name__ == "__main__":
    unittest.main()
