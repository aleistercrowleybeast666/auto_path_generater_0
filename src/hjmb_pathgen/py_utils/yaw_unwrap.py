"""Continuous yaw unwrapping helpers for Phase 3 task candidates."""

from __future__ import annotations

from hjmb_pathgen.py_domain.enums import YawPolicy
from hjmb_pathgen.py_domain.errors import CompileError

YAW_PERIOD_DDEG = 3600


def unwrap_yaw_sequence(nominal_yaws_ddeg: list[int] | tuple[int, ...], direction: YawPolicy | str) -> tuple[int, ...]:
    if not nominal_yaws_ddeg:
        return ()
    policy = direction if isinstance(direction, YawPolicy) else YawPolicy(str(direction))
    values = [int(value) for value in nominal_yaws_ddeg]
    result = [values[0]]
    for nominal in values[1:]:
        if policy == YawPolicy.CW_ONLY:
            unwrapped = _unwrap_cw(result[-1], nominal)
        elif policy == YawPolicy.CCW_ONLY:
            unwrapped = _unwrap_ccw(result[-1], nominal)
        elif policy == YawPolicy.SHORTEST:
            unwrapped = _unwrap_shortest(result[-1], nominal)
        else:
            raise CompileError(f"unsupported yaw policy: {policy}")
        _validate_int16(unwrapped)
        result.append(unwrapped)
    for value in result:
        _validate_int16(value)
    return tuple(result)


def _unwrap_cw(previous: int, nominal: int) -> int:
    value = nominal
    while value > previous:
        value -= YAW_PERIOD_DDEG
    return value


def _unwrap_ccw(previous: int, nominal: int) -> int:
    value = nominal
    while value < previous:
        value += YAW_PERIOD_DDEG
    return value


def _unwrap_shortest(previous: int, nominal: int) -> int:
    candidates = [nominal + YAW_PERIOD_DDEG * offset for offset in range(-10, 11)]
    return min(candidates, key=lambda value: (abs(value - previous), value))


def _validate_int16(value: int) -> None:
    if not -0x8000 <= value <= 0x7FFF:
        raise CompileError(f"yaw_ddeg out of int16 range after unwrap: {value}")
