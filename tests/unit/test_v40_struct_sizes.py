from __future__ import annotations

import struct
import unittest

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.codec.binary_layout import ACTION_FMT, HEADER_FMT, NODE_FMT, SEGMENT_FMT
from hjmb_pathgen.models.protocol import ACTION_SIZE, HEADER_SIZE, NODE_SIZE, SEGMENT_SIZE


class V40StructSizeTest(unittest.TestCase):
    def test_struct_sizes(self):
        self.assertEqual(struct.calcsize(HEADER_FMT), 104)
        self.assertEqual(struct.calcsize(NODE_FMT), 16)
        self.assertEqual(struct.calcsize(SEGMENT_FMT), 24)
        self.assertEqual(struct.calcsize(ACTION_FMT), 22)
        self.assertEqual((HEADER_SIZE, NODE_SIZE, SEGMENT_SIZE, ACTION_SIZE), (104, 16, 24, 22))


if __name__ == "__main__":
    unittest.main()
