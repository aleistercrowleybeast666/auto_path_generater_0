# Phase 8 Acceptance

Implemented in this phase:

- mode-scoped TASK_COMPILED and MANUAL_FREE output directories
- explicit `bin/final/` export path
- generated task cases are not auto-approved
- locked candidate preservation and `LOCK_CONFLICT`
- active reusable-leg audit during generation
- generated KINEMATIC `check_start_s_mm`
- upgraded `validate_one/all` report content
- explicit leg clear service and CLI
- process worker foundation
- V4 PySide6 workflow window with Phase 8 tabs

Validation commands used:

```bash
python -m unittest tests.unit.test_phase7_generation
python -m unittest tests.unit.test_phase8_worker_and_ui
python -m unittest tests.unit.test_phase2_services tests.unit.test_v40_struct_sizes tests.unit.test_v40_bin_roundtrip
```
