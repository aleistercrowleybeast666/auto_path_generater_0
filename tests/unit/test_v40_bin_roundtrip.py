from __future__ import annotations

import struct
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_io.codecs.binary_layout import (
    ACTION_FMT,
    CRC32_OFFSET,
    HEADER_FIELD_NAMES,
    HEADER_FMT,
    SEGMENT_FMT,
    decode_compiled_trajectory,
    encode_compiled_trajectory,
)
from hjmb_pathgen.py_io.codecs.crc32 import crc32_ieee
from hjmb_pathgen.py_io.codecs.fixtures import minimal_compiled_trajectory, minimal_bin_bytes
from hjmb_pathgen.py_domain.errors import V40ValidationError
from hjmb_pathgen.py_domain.protocol import ACTION_SIZE, HEADER_SIZE, SEGMENT_SIZE

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def rewrite_crc(data: bytes) -> bytes:
    mutable = bytearray(data)
    mutable[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
    crc = crc32_ieee(bytes(mutable))
    mutable[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
    return bytes(mutable)


def repack_header(data: bytes, field_name: str, value: int | bytes, *, recalc_crc: bool = True) -> bytes:
    values = list(struct.unpack(HEADER_FMT, data[:HEADER_SIZE]))
    values[HEADER_FIELD_NAMES.index(field_name)] = value
    packed = bytearray(struct.pack(HEADER_FMT, *values) + data[HEADER_SIZE:])
    return rewrite_crc(bytes(packed)) if recalc_crc else bytes(packed)


class V40BinRoundTripTest(unittest.TestCase):
    def test_minimal_fixture_matches_deterministic_encoder(self):
        fixture = (FIXTURE_ROOT / "minimal.bin").read_bytes()
        self.assertEqual(fixture, minimal_bin_bytes())

    def test_decode_encode_round_trip_is_byte_identical(self):
        fixture = (FIXTURE_ROOT / "minimal.bin").read_bytes()
        decoded = decode_compiled_trajectory(fixture)
        self.assertEqual(encode_compiled_trajectory(decoded), fixture)
        self.assertEqual(decode_compiled_trajectory(encode_compiled_trajectory(minimal_compiled_trajectory())), decoded)

    def test_header_magic_version_size_offset_crc_errors(self):
        fixture = (FIXTURE_ROOT / "minimal.bin").read_bytes()
        with self.assertRaisesRegex(V40ValidationError, "magic"):
            decode_compiled_trajectory(repack_header(fixture, "magic", b"BAD!", recalc_crc=True))
        with self.assertRaisesRegex(V40ValidationError, "version"):
            decode_compiled_trajectory(repack_header(fixture, "version", 35, recalc_crc=True))
        with self.assertRaisesRegex(V40ValidationError, "header_size"):
            decode_compiled_trajectory(repack_header(fixture, "header_size", 103, recalc_crc=True))
        with self.assertRaisesRegex(V40ValidationError, "segment_offset"):
            decode_compiled_trajectory(repack_header(fixture, "segment_offset", 105, recalc_crc=True))
        with self.assertRaisesRegex(V40ValidationError, "file_size"):
            decode_compiled_trajectory(fixture + b"\x00")
        damaged = bytearray(fixture)
        damaged[-1] ^= 0x01
        with self.assertRaisesRegex(V40ValidationError, "CRC"):
            decode_compiled_trajectory(bytes(damaged))

    def test_reserved_and_unknown_flags_rejected(self):
        fixture = (FIXTURE_ROOT / "minimal.bin").read_bytes()
        with self.assertRaisesRegex(V40ValidationError, "reserved"):
            decode_compiled_trajectory(repack_header(fixture, "reserved0", 1, recalc_crc=True))
        with self.assertRaisesRegex(V40ValidationError, "flags"):
            decode_compiled_trajectory(repack_header(fixture, "flags", 0x8000, recalc_crc=True))

        segment_offset = HEADER_SIZE + 2 * 16
        segment = list(struct.unpack(SEGMENT_FMT, fixture[segment_offset : segment_offset + SEGMENT_SIZE]))
        segment[7] = 0x80
        patched = bytearray(fixture)
        patched[segment_offset : segment_offset + SEGMENT_SIZE] = struct.pack(SEGMENT_FMT, *segment)
        with self.assertRaisesRegex(V40ValidationError, "flags"):
            decode_compiled_trajectory(rewrite_crc(bytes(patched)))

        action_offset = segment_offset + SEGMENT_SIZE
        action = list(struct.unpack(ACTION_FMT, fixture[action_offset : action_offset + ACTION_SIZE]))
        action[-1] = 1
        patched = bytearray(fixture)
        patched[action_offset : action_offset + ACTION_SIZE] = struct.pack(ACTION_FMT, *action)
        with self.assertRaisesRegex(V40ValidationError, "reserved"):
            decode_compiled_trajectory(rewrite_crc(bytes(patched)))


if __name__ == "__main__":
    unittest.main()
