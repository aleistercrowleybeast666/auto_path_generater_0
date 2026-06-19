from __future__ import annotations

import tempfile
import unittest
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.codec.json_codec import load_case, save_case
from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.services.example_project_service import create_synthetic_example_project
from hjmb_pathgen.services.mode_output_service import export_final_bin
from hjmb_pathgen.services.phase9_delivery_service import (
    final_drop_audit_from_bin,
    generate_golden_manifest,
    output_layout_report,
    release_manifest,
    protocol_conformance_report,
)
from hjmb_pathgen.services.project_service import ProjectLayout

from tests.unit.test_phase7_generation import (
    collect_unique_legs,
    manual_case_dict,
    populate_library_for_collection,
    write_manual_free_outputs,
    write_one_row_project,
)
from hjmb_pathgen.services.phase7_generation_service import generate_one


class Phase9DeliveryTest(unittest.TestCase):
    def test_protocol_conformance_report_matches_v40_constants(self):
        report = protocol_conformance_report("HJMB_path_file_protocol_v4.0.txt")
        self.assertTrue(report["passed"], report)
        checks = {item["name"]: item for item in report["checks"]}
        self.assertEqual(checks["struct_sizes"]["details"]["actual"], {"HeaderV40": 104, "NodeV40": 16, "SegmentV40": 24, "ActionV40": 22})

    def test_golden_manifest_is_reproducible_for_generated_case(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())
            generate_one(layout, 0)

            first = generate_golden_manifest(layout)
            second = generate_golden_manifest(layout)
            self.assertEqual(first["manifest_sha256"], second["manifest_sha256"])
            self.assertEqual(first["case_count"], 1)
            self.assertTrue(first["entries"][0]["roundtrip_byte_identical"])

    def test_final_export_and_final_drop_audit_after_explicit_approval(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            collection = collect_unique_legs(layout)
            populate_library_for_collection(layout, collection.to_dict())
            generate_one(layout, 0)
            task_path = layout.case_json_path_for_source(0, PathSource.TASK_COMPILED)
            case = load_case(task_path)
            review = dict(case.review)
            review["approved"] = True
            save_case(task_path, replace(case, review=review))

            output = export_final_bin(layout, 0, path_source=PathSource.TASK_COMPILED)
            self.assertEqual(output.bin_path, layout.final_bin_path(0))
            audit = final_drop_audit_from_bin(output.bin_path)
            self.assertTrue(audit["passed"], audit)

    def test_output_layout_reports_path_source_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            layout = write_one_row_project(Path(tmp))
            wrong = layout.case_json_path_for_source(0, PathSource.TASK_COMPILED)
            wrong.parent.mkdir(parents=True, exist_ok=True)
            save_case(wrong, load_case_dict(manual_case_dict()))

            report = output_layout_report(layout)
            self.assertFalse(report["passed"])
            self.assertEqual(report["path_source_mismatch_count"], 1)

    def test_synthetic_example_project_has_360_route_table_and_unique_legs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "example"
            report = create_synthetic_example_project(root, source_traj_csv=Path("traj_id.csv"))
            self.assertTrue(report["synthetic"])
            self.assertEqual(report["route_case_count"], 360)
            self.assertGreater(report["unique_leg_count"], 0)
            layout = ProjectLayout.open(root)
            self.assertTrue(layout.project_json.exists())
            self.assertTrue(layout.route_case_table_json.exists())
            self.assertTrue(layout.leg_library_json.exists())

    def test_release_manifest_excludes_runtime_and_build_junk(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "src").mkdir()
            (root / "src" / "app.py").write_text("print('ok')\n", encoding="utf-8")
            (root / "__pycache__").mkdir()
            (root / "__pycache__" / "x.pyc").write_bytes(b"cache")
            (root / "dist").mkdir()
            (root / "dist" / "app.exe").write_bytes(b"bin")

            report = release_manifest(root)
            paths = {item["path"] for item in report["files"]}
            self.assertEqual(paths, {"src/app.py"})


def load_case_dict(data: dict):
    from hjmb_pathgen.models.route_case import CaseManifestV40

    return CaseManifestV40.from_dict(data)


if __name__ == "__main__":
    unittest.main()
