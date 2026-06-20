"""Validation errors for V4.0 data contracts."""

from __future__ import annotations


class V40ValidationError(ValueError):
    """Raised when V4.0 input violates the explicit data contract."""

    def __init__(
        self,
        object_type: str,
        field_path: str,
        message: str,
        *,
        actual: object | None = None,
        expected: object | None = None,
    ) -> None:
        details = f"{object_type}.{field_path}: {message}"
        if actual is not None:
            details += f"; actual={actual!r}"
        if expected is not None:
            details += f"; expected={expected!r}"
        details += "; V4.0 does not support silent migration"
        super().__init__(details)
        self.object_type = object_type
        self.field_path = field_path
        self.actual = actual
        self.expected = expected


class JsonFormatError(V40ValidationError):
    """Raised when a V4.0 JSON file is not valid UTF-8 JSON text."""


class JsonValidationError(V40ValidationError):
    """Raised when a decoded V4.0 JSON document violates the schema."""


class CsvFormatError(V40ValidationError):
    """Raised when traj_id.csv bytes or table shape are malformed."""


class CsvValidationError(V40ValidationError):
    """Raised when traj_id.csv business semantics are invalid."""


class BinaryFormatError(V40ValidationError):
    """Raised when V4.0 BIN bytes are malformed before semantic validation."""


class BinaryCrcError(BinaryFormatError):
    """Raised when the V4.0 BIN CRC does not match the payload."""


class BinaryLayoutError(BinaryFormatError):
    """Raised when V4.0 BIN layout or semantic structure is invalid."""


class FilenameMismatchError(V40ValidationError):
    """Raised when a Pxxxx filename and internal traj_id disagree."""


class ProjectLayoutError(RuntimeError):
    """Raised when project directory paths are invalid or unsafe."""


class MissingDependencyError(RuntimeError):
    """Raised when a case references a required dependency that is absent."""


class StaleDependencyError(RuntimeError):
    """Raised when a case references a dependency that is not reusable."""


class AtomicWriteError(RuntimeError):
    """Raised when an atomic write cannot complete without replacing final output."""


class WriteBackValidationError(AtomicWriteError):
    """Raised when write-then-read validation fails before final replacement."""


class CompileError(RuntimeError):
    """Raised when a V4.0 case cannot be assembled into a compiled trajectory."""


def reject_unknown_fields(
    data: dict,
    allowed_fields: set[str],
    object_type: str,
    field_path: str = "$",
) -> None:
    unknown = sorted(set(data) - allowed_fields)
    if unknown:
        raise V40ValidationError(
            object_type,
            field_path,
            "unknown fields are not allowed",
            actual=unknown,
            expected=sorted(allowed_fields),
        )


def require_fields(
    data: dict,
    required_fields: set[str],
    object_type: str,
    field_path: str = "$",
) -> None:
    missing = sorted(required_fields - set(data))
    if missing:
        raise V40ValidationError(
            object_type,
            field_path,
            "required fields are missing",
            actual=missing,
            expected=sorted(required_fields),
        )


def expect_equal(
    actual: object,
    expected: object,
    object_type: str,
    field_path: str,
) -> None:
    if actual != expected:
        raise V40ValidationError(
            object_type,
            field_path,
            "unexpected value",
            actual=actual,
            expected=expected,
        )


def expect_int_range(
    value: int,
    lower: int,
    upper: int,
    object_type: str,
    field_path: str,
) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not lower <= value <= upper:
        raise V40ValidationError(
            object_type,
            field_path,
            "integer out of range",
            actual=value,
            expected=f"{lower}..{upper}",
        )
    return value


def strict_model_fields(
    data: dict,
    required_fields: set[str],
    optional_fields: set[str],
    object_type: str,
    field_path: str = "$",
) -> None:
    allowed = required_fields | optional_fields
    reject_unknown_fields(data, allowed, object_type, field_path)
    require_fields(data, required_fields, object_type, field_path)
