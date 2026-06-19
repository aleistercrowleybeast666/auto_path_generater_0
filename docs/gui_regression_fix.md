# GUI Regression Fix

This addendum fixes the V4 GUI regression where the Phase 8 window exposed nine
workflow tabs but most tabs were placeholders.

Implemented source-level fixes:

- model-backed `Project/Sites` table and V4 field canvas
- editable fixed-site markers and yaw handles
- independent `Manual Free` canvas and sparse-point table
- real `Route/Leg` visualization from `leg_library.json`
- real `Leg Library`, `Task/360`, `Actions`, `Vehicle/Collision`,
  `Planning`, and `Reports/Final` tabs
- docked log area and disabled idle Cancel button
- structured GUI scene dump tests

The fix ports only V3-era graphics interaction concepts. It does not port V3.5
project models, JSON/BIN formats, CUT_IN behavior, old action fields, fixed-8
business logic, or the old auto-planning timer.

This round did not rerun PyInstaller and did not create a new executable or
release archive.
