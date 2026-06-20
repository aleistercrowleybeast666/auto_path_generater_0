"""Atomic same-directory file writes with write-back validation."""

from __future__ import annotations

import os
import uuid
from pathlib import Path
from typing import Callable

from hjmb_pathgen.py_domain.errors import AtomicWriteError, WriteBackValidationError

ReplaceFunc = Callable[[str, str], None]
ValidatorFunc = Callable[[Path], None]
AfterWriteFunc = Callable[[Path], None]


def atomic_write_bytes(
    path: str | Path,
    data: bytes,
    *,
    validator: ValidatorFunc | None = None,
    replace_func: ReplaceFunc | None = None,
    after_write: AfterWriteFunc | None = None,
) -> None:
    """Write bytes atomically after validating a flushed temp file."""

    target = Path(path)
    parent = target.parent
    parent.mkdir(parents=True, exist_ok=True)
    replace_func = replace_func or os.replace
    temp_path = parent / f".{target.name}.{uuid.uuid4().hex}.tmp"

    try:
        with temp_path.open("xb") as file:
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
        if after_write is not None:
            after_write(temp_path)
        if validator is not None:
            try:
                validator(temp_path)
            except WriteBackValidationError:
                raise
            except Exception as exc:  # noqa: BLE001 - converted into a phase-specific error.
                raise WriteBackValidationError(f"write-back validation failed for {target}: {exc}") from exc
        replace_func(str(temp_path), str(target))
        _fsync_directory_best_effort(parent)
    except WriteBackValidationError:
        _cleanup_temp(temp_path)
        raise
    except Exception as exc:  # noqa: BLE001 - keep final output untouched and report one clear error type.
        _cleanup_temp(temp_path)
        raise AtomicWriteError(f"atomic write failed for {target}: {exc}") from exc


def _cleanup_temp(temp_path: Path) -> None:
    try:
        if temp_path.exists():
            temp_path.unlink()
    except OSError:
        pass


def _fsync_directory_best_effort(path: Path) -> None:
    if os.name == "nt":
        return
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
