from __future__ import annotations

import unittest

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.codec.crc32 import crc32_ieee


class Crc32Test(unittest.TestCase):
    def test_known_vectors(self):
        self.assertEqual(crc32_ieee(b""), 0x00000000)
        self.assertEqual(crc32_ieee(b"123456789"), 0xCBF43926)
        self.assertEqual(crc32_ieee(b"123456789"), crc32_ieee(b"123456789"))


if __name__ == "__main__":
    unittest.main()
