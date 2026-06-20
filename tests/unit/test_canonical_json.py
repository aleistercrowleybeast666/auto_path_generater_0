from __future__ import annotations

import json
import math
import unittest

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_bytes, canonical_json_crc32


class CanonicalJsonTest(unittest.TestCase):
    def test_key_order_and_whitespace_do_not_change_hash(self):
        left = json.loads('{"b": 2, "a": 1}')
        right = json.loads('{\n  "a": 1,\n  "b": 2\n}')
        self.assertEqual(canonical_json_bytes(left), b'{"a":1,"b":2}')
        self.assertEqual(canonical_json_crc32(left), canonical_json_crc32(right))

    def test_array_order_and_value_changes_affect_hash(self):
        self.assertNotEqual(canonical_json_crc32({"a": [1, 2]}), canonical_json_crc32({"a": [2, 1]}))
        self.assertNotEqual(canonical_json_crc32({"a": 1}), canonical_json_crc32({"a": 1.0}))
        self.assertNotEqual(canonical_json_crc32({"a": 1}), canonical_json_crc32({"a": 2}))

    def test_rejects_non_finite_numbers(self):
        with self.assertRaises(ValueError):
            canonical_json_bytes({"bad": math.nan})
        with self.assertRaises(ValueError):
            canonical_json_bytes({"bad": math.inf})

    def test_unicode_is_stable_utf8(self):
        self.assertEqual(canonical_json_bytes({"name": "搬运"}), '{"name":"搬运"}'.encode("utf-8"))


if __name__ == "__main__":
    unittest.main()
