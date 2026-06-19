# Phase 9 GUI Addendum

After Phase 9 packaging, a source-level GUI regression was fixed: the V4 window
now has real graphics-backed editors instead of placeholder tabs.

Validation focus:

- `V4FieldView` exists and always draws field boundary/grid.
- Opening a project loads `project.json`, `route_case_table.json`,
  `leg_library.json`, task/manual cases, reports, and final BIN inventory.
- `Project/Sites` loads all ten fixed sites and draws exactly five yaw handles
  for `P_START`/pickup poses.
- `Manual Free` supports START/WAYPOINT/ARRIVAL sparse editing.
- `Route/Leg` draws dense leg paths, speed overlay, and collision footprint.
- Editing marks dirty/STALE and does not launch a worker.
- Cancel is disabled while idle.

Packaging boundary:

This addendum is a source update only. PyInstaller was not rerun, no new EXE was
generated, and no new release ZIP was produced in this GUI fix round.
