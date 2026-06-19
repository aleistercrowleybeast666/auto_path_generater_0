# Phase 4 Acceptance Notes

Phase 4 keeps V4.0 as the active protocol and keeps BIN version 40 unchanged.

Implemented in this phase:

- strict ten-site `project.json` configuration with explicit `configured`
- strict unload profile configuration for `BIN_1`, `BIN_2`, `BIN_3`,
  `BIN_12`, and `BIN_23`
- site pose preset JSON `HJMB_SITE_POSE_PRESET_JSON_V40`
- project functional hashes and STALE helper
- Case `path_source` with `TASK_COMPILED` and `MANUAL_FREE`
- manual free path validation and retiming
- V4 `z=v^2` time parameterization for finite geometry

Still out of scope:

- Phase 5 continuous collision checking
- Phase 6 automatic geometry optimization
- final 360 competition BIN generation
- firmware runtime implementation

Focused validation command:

```powershell
python -m unittest tests.unit.test_v40_models tests.unit.test_phase3_task_compiler tests.unit.test_phase4_project_config tests.unit.test_phase4_site_presets tests.unit.test_phase4_manual_path tests.unit.test_phase4_time_parameterization
```

Acceptance run on Python 3.14.0 / PySide6 6.10.1:

```powershell
python -m compileall src tests
# OK

python -m unittest tests.unit.test_phase4_project_config tests.unit.test_phase4_site_presets tests.unit.test_phase4_manual_path tests.unit.test_phase4_time_parameterization -v
# Ran 13 tests in 0.129s
# OK

python -m unittest discover -s tests -v
# Ran 58 tests in 16.332s
# OK

python -m unittest -v
# Ran 104 tests in 16.547s
# OK
```

GUI smoke:

```powershell
python -c "import os; os.environ['QT_QPA_PLATFORM']='offscreen'; from PySide6.QtWidgets import QApplication; import hjmb_path_editor as h; app=QApplication([]); win=h.MainWindow(); print('window ok')"
# window ok
```
