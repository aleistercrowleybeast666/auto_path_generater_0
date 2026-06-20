from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hjmb_pathgen.py_io.codecs.csv_codec import parse_traj_id_csv_bytes, reconstruct_semantic_row
from hjmb_pathgen.py_domain.errors import CsvFormatError, CsvValidationError

from phase3_helpers import make_valid_traj_csv_bytes


class Phase3CsvCodecTest(unittest.TestCase):
    def test_utf8_bom_and_non_bom_parse_to_reversible_table(self):
        plain = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(), source_csv="traj_id.csv")
        bom = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(bom=True), source_csv="traj_id.csv")
        self.assertEqual(len(plain.rows), 360)
        self.assertEqual(len(bom.rows), 360)
        self.assertNotEqual(plain.source_csv_sha256, bom.source_csv_sha256)

        table = plain.to_route_case_table()
        self.assertEqual(table.case_count if hasattr(table, "case_count") else len(table.cases), 360)
        self.assertEqual(table.cases[0].source_row_number, 2)
        self.assertEqual(reconstruct_semantic_row(table.cases[0])["文件名"], "P0000.BIN")
        self.assertEqual(reconstruct_semantic_row(table.cases[0])["①号位豆子"], "黄豆")

    def test_source_row_hash_ignores_line_endings(self):
        lf = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(lineterminator="\n"))
        crlf = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(lineterminator="\r\n"))
        self.assertNotEqual(lf.source_csv_sha256, crlf.source_csv_sha256)
        self.assertEqual(lf.rows[123].source_row_hash, crlf.rows[123].source_row_hash)

    def test_non_utf8_rejected(self):
        with self.assertRaises(CsvFormatError):
            parse_traj_id_csv_bytes("traj_id,文件名\n".encode("gbk"))

    def test_header_must_be_exact(self):
        with self.assertRaises(CsvFormatError) as missing:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_header=lambda header: header.pop()))
        self.assertIn("missing", str(missing.exception))

        def duplicate(header: list[str]) -> None:
            header[-1] = header[-2]

        with self.assertRaises(CsvFormatError) as duplicated:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_header=duplicate))
        self.assertIn("duplicate", str(duplicated.exception))

        def extra(header: list[str]) -> None:
            header[-1] = "unknown"

        with self.assertRaises(CsvFormatError) as unknown:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_header=extra))
        self.assertIn("extra", str(unknown.exception))

    def test_middle_blank_row_rejected(self):
        def mutate(rows: list[list[str]]) -> None:
            rows.insert(10, [])

        with self.assertRaises(CsvValidationError) as ctx:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_row=mutate))
        self.assertIn("blank row", str(ctx.exception))

    def test_duplicate_missing_and_formula_errors_are_reported(self):
        def mutate(rows: list[list[str]]) -> None:
            rows[2][0] = rows[1][0]
            rows[3][0] = "359"

        with self.assertRaises(CsvValidationError) as ctx:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_row=mutate))
        text = str(ctx.exception)
        self.assertIn("duplicate traj_id", text)
        self.assertIn("formula mismatch", text)

    def test_invalid_business_values_include_row_and_column(self):
        def mutate(rows: list[list[str]]) -> None:
            rows[1][4] = "红豆"
            rows[2][7] = "九号位"

        with self.assertRaises(CsvValidationError) as ctx:
            parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_row=mutate))
        text = str(ctx.exception)
        self.assertIn("row 2.①号位豆子", text)
        self.assertIn("row 3.数字1在几号位", text)

    def test_empty_box_swap_does_not_change_effective_signature(self):
        original = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes())

        def mutate(rows: list[list[str]]) -> None:
            for bean_code in range(6):
                row_index = 1 + bean_code * 60
                rows[row_index][10], rows[row_index][11] = rows[row_index][11], rows[row_index][10]

        swapped = parse_traj_id_csv_bytes(make_valid_traj_csv_bytes(mutate_row=mutate))
        self.assertEqual(original.rows[0].target_position_signature(), swapped.rows[0].target_position_signature())
        self.assertNotEqual(original.rows[0].raw_fields["数字4在几号位"], swapped.rows[0].raw_fields["数字4在几号位"])


if __name__ == "__main__":
    unittest.main()
