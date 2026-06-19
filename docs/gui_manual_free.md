# GUI Manual Free

The `Manual Free` tab has an independent V4 field canvas and sparse-point table.

Supported point types:

- `START`: first point, has `x/y/yaw`
- `WAYPOINT`: has `x/y`, no fixed yaw
- `ARRIVAL`: has `x/y/yaw` and is a stop point

Use the toolbar to select an add mode, then double-click the field. Dragging
points or yaw handles updates only the in-memory manual path draft and marks the
GUI dirty/STALE. It does not overwrite `TASK_COMPILED` outputs and does not
start planning.

Working output remains mode-scoped:

- `cases/manual_free/Pxxxx.json`
- `bin/manual_free/Pxxxx.BIN`
- final export still goes through `bin/final/Pxxxx.BIN`
