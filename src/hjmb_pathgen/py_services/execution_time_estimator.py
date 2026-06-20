"""FIFO-aware chassis/mechanism execution-time estimation.

The estimate follows the V4 action semantics: actions are strictly FIFO;
ASYNC actions may overlap chassis motion, while STOP_AND_WAIT actions cannot
start before their ARRIVAL and delay departure until DONE plus post-wait.  If
an earlier ASYNC action is still active at a later stop, its remaining time is
therefore carried into that stop automatically.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping

from hjmb_pathgen.py_domain.enums import ActionCode, ActionMode
from hjmb_pathgen.py_domain.project import ProjectV40


@dataclass(frozen=True)
class ExecutionTimeEstimate:
    motion_time_ms: int
    mechanism_busy_time_ms: int
    added_wait_time_ms: int
    total_time_ms: int
    action_timeline: tuple[dict[str, Any], ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "motion_time_ms": self.motion_time_ms,
            "mechanism_busy_time_ms": self.mechanism_busy_time_ms,
            "added_wait_time_ms": self.added_wait_time_ms,
            "total_time_ms": self.total_time_ms,
            "action_timeline": list(self.action_timeline),
        }


def estimate_fifo_execution(
    project: ProjectV40,
    source_actions: Iterable[Mapping[str, Any]],
    *,
    motion_time_ms: int,
    arrival_release_ms: Mapping[object, int] | None = None,
    compiled_actions: Iterable[object] | None = None,
    time_at_s_mm: Callable[[int], int] | None = None,
) -> ExecutionTimeEstimate:
    source = [dict(item) for item in source_actions]
    compiled = list(compiled_actions or ())
    releases = dict(arrival_release_ms or {})
    fifo_available = 0
    route_delay = 0
    busy_sum = 0
    timeline: list[dict[str, Any]] = []

    for index, item in enumerate(source):
        compiled_item = compiled[index] if index < len(compiled) else None
        action_name = _action_name(item, compiled_item)
        mode = _mode(item, compiled_item, project, action_name)
        duration = _duration_ms(item, project, action_name)
        busy_sum += duration

        release = 0
        release_kind = "IMMEDIATE"
        if mode == ActionMode.STOP_AND_WAIT:
            key = _arrival_key(item, compiled_item)
            release = int(releases.get(key, 0))
            # The chassis reaches later arrivals later when previous stop work
            # has already extended the schedule.
            release += route_delay
            release_kind = f"ARRIVAL:{key}"
        elif mode == ActionMode.KINEMATIC:
            check_s = _compiled_int(compiled_item, "check_start_s_mm", item.get("check_start_s_mm", 0xFFFF))
            if check_s != 0xFFFF and time_at_s_mm is not None:
                release = int(time_at_s_mm(check_s)) + route_delay
                release_kind = f"S:{check_s}"

        start = max(fifo_available, release)
        finish = start + duration
        fifo_available = finish
        if mode == ActionMode.STOP_AND_WAIT:
            base_release = max(0, release - route_delay)
            route_delay = max(route_delay, finish - base_release)

        timeline.append(
            {
                "action_seq": index,
                "action": action_name,
                "mode": mode.name,
                "release_ms": release,
                "release_kind": release_kind,
                "start_ms": start,
                "finish_ms": finish,
                "duration_ms": duration,
                "fifo_wait_ms": max(0, start - release),
            }
        )

    total = max(int(motion_time_ms) + route_delay, fifo_available)
    return ExecutionTimeEstimate(
        motion_time_ms=int(motion_time_ms),
        mechanism_busy_time_ms=busy_sum,
        added_wait_time_ms=max(0, total - int(motion_time_ms)),
        total_time_ms=total,
        action_timeline=tuple(timeline),
    )


def arrival_release_from_segments(segments: Iterable[object]) -> dict[object, int]:
    result: dict[object, int] = {}
    elapsed = 0
    for segment in segments:
        elapsed += int(getattr(segment, "planned_time_ms", 0))
        arrival_id = int(getattr(segment, "end_arrival_id", 0xFF))
        if arrival_id != 0xFF:
            result[arrival_id] = elapsed
    return result


def time_at_s_from_segments(segments: Iterable[object]) -> Callable[[int], int]:
    ordered = tuple(segments)

    def lookup(s_mm: int) -> int:
        elapsed = 0
        for segment in ordered:
            start_s = int(getattr(segment, "start_s_mm", 0))
            end_s = int(getattr(segment, "end_s_mm", start_s))
            duration = int(getattr(segment, "planned_time_ms", 0))
            if s_mm <= end_s:
                if end_s <= start_s:
                    return elapsed
                ratio = min(1.0, max(0.0, (s_mm - start_s) / (end_s - start_s)))
                return elapsed + round(duration * ratio)
            elapsed += duration
        return elapsed

    return lookup


def _duration_ms(item: dict[str, Any], project: ProjectV40, action_name: str) -> int:
    profile = project.action_profiles.get(action_name, {})
    profile = dict(profile) if isinstance(profile, dict) else {}
    if "estimated_time_ms" in item:
        core = int(item.get("estimated_time_ms", 0))
    else:
        core = int(profile.get("estimated_time_ms", 0))
    post_wait = int(item.get("post_wait_ms", profile.get("post_wait_ms", 0)))
    return max(0, core) + max(0, post_wait)


def _mode(item: dict[str, Any], compiled_item: object | None, project: ProjectV40, action_name: str) -> ActionMode:
    raw = item.get("mode")
    if raw is None and compiled_item is not None:
        raw = getattr(compiled_item, "mode", None)
    if raw is None:
        profile = project.action_profiles.get(action_name, {})
        if isinstance(profile, dict):
            raw = profile.get("mode")
    if isinstance(raw, ActionMode):
        return raw
    if isinstance(raw, str):
        return ActionMode[raw.removeprefix("ACTION_MODE_")]
    return ActionMode(int(raw or 0))


def _arrival_key(item: dict[str, Any], compiled_item: object | None) -> object:
    if compiled_item is not None:
        arrival_id = int(getattr(compiled_item, "arrival_id", 0xFF))
        if arrival_id != 0xFF:
            return arrival_id
    for key in ("arrival_state_id", "arrival_point_index", "arrival_point_id", "arrival_id"):
        if item.get(key) is not None:
            return item[key]
    return 0


def _action_name(item: dict[str, Any], compiled_item: object | None) -> str:
    raw = item.get("profile_key", item.get("action", "NONE"))
    if isinstance(raw, str):
        return raw.removeprefix("PATH_ACT_")
    if compiled_item is not None:
        raw = getattr(compiled_item, "action", raw)
    try:
        return ActionCode(int(raw)).name
    except (ValueError, TypeError):
        return str(raw)


def _compiled_int(compiled_item: object | None, field: str, fallback: object) -> int:
    if compiled_item is None:
        return int(fallback)
    return int(getattr(compiled_item, field, fallback))
