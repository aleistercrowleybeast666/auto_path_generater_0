# Phase 5 Acceptance Notes

Phase 5 keeps V4.0 as the active protocol. It does not create V4.1 and does
not change BIN version 40 or the packed sizes:

```text
Header  104 bytes
Node     16 bytes
Segment  24 bytes
Action   22 bytes
```

Implemented in this phase:

- strict collision footprint and obstacle configuration in `project.json`
- `LARGE_CIRCLE`, `SMALL_CIRCLE`, and `PICKUP_CLIPPED_DISK`
- cylinder, drop-box, pickup-box, and field-boundary signed-clearance checks
- continuous segment subdivision using `dp + R_large * abs(dyaw)`
- minimum clearance and first-collision diagnostics
- MANUAL_FREE and existing dense-geometry validation services
- collision report atomic writes
- formal export guard for non-`PASSED` collision status
- CLI wrappers for collision config, case, leg, and report inspection

Out of scope and not implemented:

- Phase 6 geometry optimization
- automatic S-route gate optimization
- automatic yaw search
- 360 final BIN batch assembly
- Phase 8 UI rewrite
- STM32 firmware

Focused validation:

```powershell
python -m unittest tests.unit.test_phase5_collision_geometry tests.unit.test_phase5_path_validation -v
```
