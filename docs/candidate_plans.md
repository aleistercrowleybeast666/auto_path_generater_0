# Phase 3 Candidate Plans

Each Case row produces candidates for both route families. Candidate IDs are
stable and based on canonical task semantics, never on Python `hash()`, list
index, time, or generation order.

```text
C_<ROUTE_FAMILY>_<8 hex chars>
```

The semantic hash includes route family, pickup state sequence, drop targets,
vehicle-bin assignment, unload sequence, unwrapped yaw sequence, source actions,
and relevant action/unload profile hashes.

## Single And Dual Unload

The compiler always attempts the full single-bin baseline. It may add one dual
unload step only when the two target physical sites are adjacent.

Allowed masks:

```text
BIN_1
BIN_2
BIN_3
BIN_12
BIN_23
```

Forbidden masks are never generated:

```text
BIN_13
BIN_123
any non-adjacent pair
overlapping dual unloads in one candidate
```

`BIN_12` anchors to the physical site associated with `BIN_1`; `BIN_23`
anchors to the physical site associated with `BIN_2`. The arrival yaw comes
from `project.unload_profiles`. If a dual profile is missing or marked
`"calibrated": false`, the dual candidate is skipped with an explicit reason.

## Yaw

The compiler stores continuous unwrapped yaw in ddeg:

```text
CW_ONLY  -> monotonically nonincreasing
CCW_ONLY -> monotonically nondecreasing
SHORTEST -> closest equivalent angle
```

Equal consecutive nominal yaw values do not force an extra turn.

## Locking

The plan lock service supports:

```text
list_candidates(layout, traj_id)
select_candidate(layout, traj_id, candidate_id, lock=False)
lock_candidate(layout, traj_id, candidate_id)
unlock_candidate(layout, traj_id)
```

A locked candidate is preserved on regeneration only if the same candidate ID
still exists and the semantic hash is unchanged. Invalid locks are reported as
conflicts instead of silently switching to a different plan.
