"""Public V4.0 BIN codec APIs."""

from __future__ import annotations

from pathlib import Path

from hjmb_pathgen.py_domain.compiled import CompiledTrajectoryV40
from hjmb_pathgen.py_domain.errors import BinaryLayoutError, WriteBackValidationError
from hjmb_pathgen.py_io.layout.path_naming import ensure_bin_filename_matches

from .binary_layout import decode_compiled_trajectory, encode_compiled_trajectory


def encode_trajectory(trajectory: CompiledTrajectoryV40, *, validate_roundtrip: bool = True) -> bytes:
    data = encode_compiled_trajectory(trajectory)
    if validate_roundtrip:
        decoded = decode_compiled_trajectory(data)
        reencoded = encode_compiled_trajectory(decoded)
        if reencoded != data:
            raise BinaryLayoutError("V40 BIN", "round_trip", "encode/decode/re-encode is not byte-identical")
    return data


def decode_trajectory(data: bytes, *, expected_filename: str | Path | None = None) -> CompiledTrajectoryV40:
    trajectory = decode_compiled_trajectory(data)
    if expected_filename is not None:
        ensure_bin_filename_matches(expected_filename, trajectory.header.traj_id)
    return trajectory


def load_bin(path: str | Path) -> CompiledTrajectoryV40:
    path = Path(path)
    return decode_trajectory(path.read_bytes(), expected_filename=path)


def save_bin(path: str | Path, trajectory: CompiledTrajectoryV40) -> None:
    from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes

    path = Path(path)
    data = encode_trajectory(trajectory)
    ensure_bin_filename_matches(path, trajectory.normalized().header.traj_id)

    def validator(temp_path: Path) -> None:
        decoded = decode_trajectory(temp_path.read_bytes(), expected_filename=path)
        if encode_trajectory(decoded) != data:
            raise WriteBackValidationError(f"BIN write-back mismatch for {path}")

    atomic_write_bytes(path, data, validator=validator)
