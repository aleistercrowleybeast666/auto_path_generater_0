"""Canonical JSON serialization and hashing for V4.0 traceability."""

from __future__ import annotations

import json
from typing import Any

from .crc32 import crc32_ieee


def canonical_json_bytes(value: Any) -> bytes:
    text = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return text.encode("utf-8")


def canonical_json_crc32(value: Any) -> int:
    return crc32_ieee(canonical_json_bytes(value))


def canonical_json_crc32_hex(value: Any) -> str:
    return f"{canonical_json_crc32(value):08x}"
