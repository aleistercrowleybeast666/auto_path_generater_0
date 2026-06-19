# Phase 6 Acceptance Notes

Phase 6 keeps BIN version 40 and does not change packed struct sizes:

```text
Header  104 bytes
Node     16 bytes
Segment  24 bytes
Action   22 bytes
```

Implemented in this phase:

- directed leg optimization request/result models
- ordered topology gate validation
- piecewise cubic Bezier XY representation
- arc-length sampling with tangent and curvature
- TWO_LOW_SPEED_WINDOWS yaw generation
- deterministic initial guesses and coordinate refinement
- candidate evaluation through Phase 5 collision and Phase 4 retiming
- `QUICK_PREVIEW`, `STANDARD`, and `FINAL` profiles
- atomic leg library save, approve, lock, unlock, stale checks
- CLI wrappers for transition listing, leg optimization, validation, retiming,
  approval, locking, and inspection

Out of scope and not implemented:

- 360 final unique-leg collection
- filling every Case manifest with final leg refs
- formal 360 BIN batch output
- Phase 8 UI workflow rewrite
- STM32 firmware

Focused validation:

```powershell
python -m unittest -v tests.unit.test_phase6_geometry_yaw_topology tests.unit.test_phase6_leg_optimizer tests.unit.test_phase6_leg_library
```

