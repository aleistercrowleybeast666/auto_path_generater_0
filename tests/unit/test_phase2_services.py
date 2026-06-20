from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_io.codecs.bin_codec import load_bin
from hjmb_pathgen.py_io.codecs.json_codec import save_case, save_leg_library
from hjmb_pathgen.py_domain.errors import AtomicWriteError, ProjectLayoutError, WriteBackValidationError
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, PortableCaseV40
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes
from hjmb_pathgen.py_services.batch_service import write_batch_outputs
from hjmb_pathgen.py_services.case_compiler import CaseCompileRequest
from hjmb_pathgen.py_services.output_service import CaseOutputOptions, write_case_outputs
from hjmb_pathgen.py_services.portable_service import export_portable_case
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout, ProjectStatus

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def fixture_dict(name: str) -> dict:
    return json.loads((FIXTURE_ROOT / name).read_text(encoding="utf-8"))


def synthetic_models() -> tuple[ProjectV40, LegLibraryV40, CaseManifestV40, PortableCaseV40]:
    project = ProjectV40.from_dict(fixture_dict("minimal_project.json"))
    library = LegLibraryV40.from_dict(fixture_dict("synthetic_leg_library.json"))
    case = CaseManifestV40.from_dict(fixture_dict("synthetic_case.json"))
    portable = PortableCaseV40.from_dict(fixture_dict("synthetic_portable_case.json"))
    return project, library, case, portable


class Phase2ServiceTest(unittest.TestCase):
    def test_project_layout_create_status_and_path_safety(self):
        project, library, case, _portable = synthetic_models()
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            self.assertTrue(layout.project_json.exists())
            self.assertTrue(layout.leg_library_json.exists())
            self.assertTrue(layout.cases_dir.is_dir())
            self.assertEqual(layout.status().status, ProjectStatus.INCOMPLETE_MAPPING)
            with self.assertRaises(ProjectLayoutError):
                layout.resolve_project_path(Path("..") / "escape.json")

            save_leg_library(layout.leg_library_json, library)
            save_case(layout.case_json_path_for_mode(case.traj_id, GenerationMode.FULL_AUTO), case)
            self.assertEqual(
                layout.status_for_case(case.traj_id, GenerationMode.FULL_AUTO).status,
                ProjectStatus.READY_FOR_SINGLE_CASE,
            )

    def test_atomic_writer_keeps_old_final_on_failures_and_cleans_temp(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "value.bin"
            atomic_write_bytes(target, b"old", validator=lambda path: None)
            self.assertEqual(target.read_bytes(), b"old")

            with self.assertRaises(WriteBackValidationError):
                atomic_write_bytes(target, b"new", validator=lambda path: (_ for _ in ()).throw(ValueError("bad temp")))
            self.assertEqual(target.read_bytes(), b"old")

            def failing_replace(_src: str, _dst: str) -> None:
                raise OSError("replace failed")

            with self.assertRaises(AtomicWriteError):
                atomic_write_bytes(target, b"newer", validator=lambda path: None, replace_func=failing_replace)
            self.assertEqual(target.read_bytes(), b"old")
            self.assertEqual(list(Path(tmp).glob(".*.tmp")), [])

    def test_portable_export_and_fixture_regenerate_identical_bin(self):
        _project, library, case, portable_fixture = synthetic_models()
        portable = export_portable_case(case, library)
        self.assertEqual(portable.traj_id, case.traj_id)
        self.assertEqual(len(portable.embedded_legs), 2)
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", synthetic_models()[0])
            save_leg_library(layout.leg_library_json, library)
            ref = write_case_outputs(
                layout,
                CaseCompileRequest(case=case, leg_library=library),
                CaseOutputOptions(write_case_json=False, write_bin=True, write_portable=False, write_report=False, dry_run=True),
            )
            embedded = write_case_outputs(
                layout,
                CaseCompileRequest(case=portable_fixture),
                CaseOutputOptions(write_case_json=False, write_bin=True, write_portable=False, write_report=False, dry_run=True),
            )
            self.assertEqual(ref.bin_bytes, embedded.bin_bytes)

    def test_single_and_partial_batch_outputs_are_byte_identical(self):
        project, library, case, _portable = synthetic_models()
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            save_leg_library(layout.leg_library_json, library)
            request = CaseCompileRequest(case=case, leg_library=library, project=project)
            single = write_case_outputs(
                layout,
                request,
                CaseOutputOptions(write_case_json=True, write_bin=True, write_portable=True, write_report=True),
            )
            single_bytes = load_bin(single.bin_path).header.file_crc32.to_bytes(4, "little") + single.bin_path.read_bytes()

            batch = write_batch_outputs(
                layout,
                [request],
                CaseOutputOptions(write_case_json=True, write_bin=True, write_portable=False, write_report=False),
            )
            self.assertEqual(len(batch.results), 1)
            self.assertEqual(batch.failures, ())
            batch_path = layout.bin_path_for_mode(case.traj_id, GenerationMode.FULL_AUTO)
            batch_bytes = load_bin(batch_path).header.file_crc32.to_bytes(4, "little") + batch_path.read_bytes()
            self.assertEqual(single_bytes, batch_bytes)
            self.assertTrue(batch.validation_report_path.exists())
            self.assertTrue(batch.batch_summary_path.exists())

    def test_empty_batch_skeleton_does_not_create_fake_cases(self):
        project, _library, _case, _portable = synthetic_models()
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            result = write_batch_outputs(layout, [])
            self.assertEqual(result.results, ())
            self.assertEqual(result.failures, ())
            self.assertEqual(list(layout.cases_dir.glob("*.json")), [])
            report = json.loads(result.validation_report_path.read_text(encoding="utf-8"))
            self.assertFalse(report["generated_360"])
            self.assertTrue(report["development_batch"])


if __name__ == "__main__":
    unittest.main()
