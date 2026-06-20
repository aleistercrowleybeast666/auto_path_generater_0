from __future__ import annotations

import json
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.codecs.csv_codec import EXPECTED_TRAJ_HEADERS, load_traj_id_csv
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_project
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_app.cli_main import main as cli_main
from hjmb_pathgen.py_services.case_draft_service import generate_all_case_drafts, generate_case_draft
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table

from phase3_helpers import phase3_project, root_traj_csv_path


class Phase3OfficialTrajIdIntegrationTest(unittest.TestCase):
    def test_real_traj_id_csv_generates_360_case_drafts_without_bin(self):
        csv_path = root_traj_csv_path()
        if not csv_path.exists():
            self.skipTest("repository root traj_id.csv is not present")

        csv_table = load_traj_id_csv(csv_path)
        self.assertEqual(len(csv_table.rows), 360)
        self.assertEqual(tuple(csv_table.rows[0].raw_fields), EXPECTED_TRAJ_HEADERS)

        with tempfile.TemporaryDirectory() as tmp:
            layout = _phase3_layout(Path(tmp) / "project", csv_path)
            table_result = write_route_case_table(layout)
            self.assertEqual(len(table_result.route_case_table.cases), 360)
            self.assertEqual(_run_cli("validate-traj-table", "--project", str(layout.root))["status"], "OK")
            candidate_cli = _run_cli("list-candidates", "--project", str(layout.root), "--traj-id", "0")
            self.assertEqual(candidate_cli["status"], "OK")
            self.assertGreaterEqual(candidate_cli["candidate_count"], 2)

            single = generate_case_draft(layout, 0)
            single_hash = canonical_json_crc32_hex(single.case.to_dict())

            batch = generate_all_case_drafts(layout)
            self.assertEqual(len(batch.results), 360)
            self.assertEqual(batch.failures, ())
            self.assertGreater(batch.unique_transition_requirement_count, 0)
            self.assertEqual(list(layout.bin_dir.glob("P*.BIN")), [])

            loaded_cases = [
                load_case(layout.case_json_path_for_mode(traj_id, GenerationMode.FULL_AUTO))
                for traj_id in range(360)
            ]
            self.assertEqual([case.traj_id for case in loaded_cases], list(range(360)))
            self.assertEqual(
                canonical_json_crc32_hex(
                    load_case(layout.case_json_path_for_mode(0, GenerationMode.FULL_AUTO)).to_dict()
                ),
                single_hash,
            )
            self.assertTrue(all(case.storage_mode.value == "REFERENCED" for case in loaded_cases))
            self.assertTrue(all(not case.leg_refs for case in loaded_cases))
            self.assertTrue(all(case.actions["compiled"] == [] for case in loaded_cases))

            first_hashes = {result.traj_id: result.case_hash for result in batch.results}
            second = generate_all_case_drafts(layout)
            second_hashes = {result.traj_id: result.case_hash for result in second.results}
            self.assertEqual(first_hashes, second_hashes)
            self.assertEqual(batch.unique_transition_requirement_count, second.unique_transition_requirement_count)

            summary = json.loads(batch.report_json_path.read_text(encoding="utf-8"))
            self.assertFalse(summary["generated_bin"])
            self.assertEqual(summary["case_draft_count"], 360)
            self.assertEqual(summary["source_csv_sha256"], csv_table.source_csv_sha256)


def _phase3_layout(root: Path, csv_path: Path) -> ProjectLayout:
    layout = ProjectLayout.open(root, create_dirs=True)
    layout.ensure_directories()
    save_project(layout.project_json, phase3_project())
    shutil.copyfile(csv_path, layout.traj_id_csv)
    return layout


def _run_cli(*args: str) -> dict:
    stream = StringIO()
    with redirect_stdout(stream):
        exit_code = cli_main(list(args))
    result = json.loads(stream.getvalue())
    result["exit_code"] = exit_code
    return result


if __name__ == "__main__":
    unittest.main()
