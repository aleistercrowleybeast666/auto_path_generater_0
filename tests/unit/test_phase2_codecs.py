from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_io.codecs.bin_codec import decode_trajectory, encode_trajectory, load_bin, save_bin
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_case,
    load_leg_library,
    load_portable_case,
    load_project,
    load_route_case_table,
    save_case,
    save_leg_library,
    save_portable_case,
    save_project,
    save_route_case_table,
)
from hjmb_pathgen.py_domain.errors import BinaryLayoutError, FilenameMismatchError, JsonFormatError
from hjmb_pathgen.py_domain.leg import LegLibraryV40
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40, PortableCaseV40, RouteCaseTableV40
from hjmb_pathgen.py_services.case_compiler import CaseCompileRequest, compile_case_to_trajectory
from hjmb_pathgen.py_io.layout.path_naming import bin_name, case_json_name, parse_bin_name, portable_name

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def fixture_dict(name: str) -> dict:
    return json.loads((FIXTURE_ROOT / name).read_text(encoding="utf-8"))


def synthetic_models() -> tuple[ProjectV40, LegLibraryV40, CaseManifestV40, PortableCaseV40]:
    project = ProjectV40.from_dict(fixture_dict("minimal_project.json"))
    library = LegLibraryV40.from_dict(fixture_dict("synthetic_leg_library.json"))
    case = CaseManifestV40.from_dict(fixture_dict("synthetic_case.json"))
    portable = PortableCaseV40.from_dict(fixture_dict("synthetic_portable_case.json"))
    return project, library, case, portable


class Phase2CodecTest(unittest.TestCase):
    def test_strict_path_naming(self):
        self.assertEqual(case_json_name(7), "P0007.json")
        self.assertEqual(bin_name(7), "P0007.BIN")
        self.assertEqual(portable_name(7), "P0007.portable.json")
        self.assertEqual(parse_bin_name("P0007.BIN"), 7)
        for name in ("P1.BIN", "P00001.BIN", "p0007.bin", "P0360.BIN"):
            with self.subTest(name=name):
                with self.assertRaises(FilenameMismatchError):
                    parse_bin_name(name)

    def test_five_json_codecs_save_load_round_trip(self):
        project, library, case, portable = synthetic_models()
        table = RouteCaseTableV40.from_dict(fixture_dict("minimal_route_case_table.json"))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            save_project(root / "project.json", project)
            save_route_case_table(root / "route_case_table.json", table)
            save_leg_library(root / "leg_library.json", library)
            save_case(root / "P0007.json", case)
            save_portable_case(root / "P0007.portable.json", portable)

            self.assertEqual(load_project(root / "project.json"), project)
            self.assertEqual(load_route_case_table(root / "route_case_table.json"), table)
            self.assertEqual(load_leg_library(root / "leg_library.json"), library)
            self.assertEqual(load_case(root / "P0007.json"), case)
            self.assertEqual(load_portable_case(root / "P0007.portable.json"), portable)

    def test_json_bom_and_filename_mismatch_rejected(self):
        _project, _library, case, _portable = synthetic_models()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            save_case(root / "P0007.json", case)
            bom_path = root / "P0008.json"
            bom_path.write_bytes(b"\xef\xbb\xbf" + (root / "P0007.json").read_bytes())
            with self.assertRaises(JsonFormatError):
                load_case(bom_path)
            mismatch = root / "P0008.json"
            mismatch.write_bytes((root / "P0007.json").read_bytes())
            with self.assertRaises(FilenameMismatchError):
                load_case(mismatch)

    def test_bin_codec_filename_and_action_validation(self):
        _project, library, case, portable = synthetic_models()
        referenced = compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library))
        portable_trajectory = compile_case_to_trajectory(CaseCompileRequest(case=portable))
        referenced_bytes = encode_trajectory(referenced)
        self.assertEqual(referenced_bytes, encode_trajectory(portable_trajectory))

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "P0007.BIN"
            save_bin(path, referenced)
            self.assertEqual(encode_trajectory(load_bin(path)), referenced_bytes)
            with self.assertRaises(FilenameMismatchError):
                decode_trajectory(path.read_bytes(), expected_filename="P0008.BIN")

        bad_action = replace(referenced.actions[1], stable_time_ms=0)
        bad = replace(referenced, actions=(referenced.actions[0], bad_action)).normalized()
        with self.assertRaisesRegex(BinaryLayoutError, "stable_time"):
            encode_trajectory(bad)


    def test_codec_rejects_xy_chord_longer_than_s_increment(self):
        _project, library, case, _portable = synthetic_models()
        trajectory = compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library))
        bad_node = replace(trajectory.nodes[1], x_mm=trajectory.nodes[0].x_mm + 50, y_mm=trajectory.nodes[0].y_mm, s_mm=10)
        bad_nodes = (trajectory.nodes[0], bad_node, *trajectory.nodes[2:])
        bad = replace(trajectory, nodes=bad_nodes).normalized()
        with self.assertRaisesRegex(BinaryLayoutError, "XY chord exceeds s increment"):
            encode_trajectory(bad)

    def test_compiler_outputs_expected_segment_structure(self):
        _project, library, case, _portable = synthetic_models()
        trajectory = compile_case_to_trajectory(CaseCompileRequest(case=case, leg_library=library))
        self.assertEqual(trajectory.header.traj_id, 7)
        self.assertEqual(trajectory.header.node_count, 4)
        self.assertEqual(trajectory.header.segment_count, 2)
        self.assertEqual(trajectory.header.action_count, 3)
        self.assertEqual([node.s_mm for node in trajectory.nodes], [0, 100, 220, 300])
        self.assertEqual([node.arrival_id for node in trajectory.nodes], [0xFF, 0, 0xFF, 1])
        self.assertEqual([(seg.start_node_index, seg.end_node_index) for seg in trajectory.segments], [(0, 1), (1, 3)])


if __name__ == "__main__":
    unittest.main()
