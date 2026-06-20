"""CRC-32/IEEE helpers."""

from __future__ import annotations

import zlib


def crc32_ieee(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF
