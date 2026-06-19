# Phase 2 Acceptance Notes

Phase 2 establishes the V4.0 data pipeline:

```text
typed model -> JSON/BIN codec -> atomic write -> immediate read-back validation
```

It intentionally does not implement Phase 3 `traj_id.csv` parsing, 360-case task
compilation, route-family decision logic, path optimization, collision checking,
or UI workflow changes.

## Implemented Modules

```text
src/hjmb_pathgen/codec/json_codec.py
src/hjmb_pathgen/codec/bin_codec.py
src/hjmb_pathgen/codec/validators.py
src/hjmb_pathgen/services/path_naming.py
src/hjmb_pathgen/services/project_service.py
src/hjmb_pathgen/services/atomic_writer.py
src/hjmb_pathgen/services/case_compiler.py
src/hjmb_pathgen/services/portable_service.py
src/hjmb_pathgen/services/output_service.py
src/hjmb_pathgen/services/batch_service.py
```

## Synthetic Fixture

Phase 2 tests use:

```text
tests/fixtures/v40/minimal_project.json
tests/fixtures/v40/synthetic_leg_library.json
tests/fixtures/v40/synthetic_case.json
tests/fixtures/v40/synthetic_portable_case.json
```

The fixture has two directed dense legs, two ARRIVAL nodes, two segments,
one STOP_AND_WAIT action, and one KINEMATIC action. It is a protocol fixture,
not an optimized competition path.

## Verification Commands

```powershell
python -m unittest tests.unit.test_phase2_codecs tests.unit.test_phase2_services -v
python -m unittest discover -s tests -v
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

## Latest Local Results

Environment:

```text
C:\Users\chdxm\AppData\Local\Programs\Python\Python314\python.exe
Python 3.14.0, MSC v.1944 64 bit AMD64
PySide6 6.10.1
Windows-11-10.0.26200-SP0
```

Commands:

```powershell
python -m compileall -q src tests
```

Result: passed.

```powershell
python -m unittest tests.unit.test_phase2_codecs tests.unit.test_phase2_services -v
```

Result: `Ran 10 tests`, `OK`.

```powershell
python -m unittest discover -s tests -v
```

Result: `Ran 28 tests`, `OK`.

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Result: `Ran 74 tests`, `OK`.

Smoke checks:

```powershell
python -c "import hjmb_path_editor; print('import ok')"
$env:QT_QPA_PLATFORM='offscreen'
python -c "from PySide6.QtWidgets import QApplication; import hjmb_path_editor as h; app=QApplication([]); win=h.MainWindow(); print('window ok')"
python -c "import sys; sys.path.insert(0, 'src'); from hjmb_pathgen.codec.bin_codec import encode_trajectory; from hjmb_pathgen.services.project_service import ProjectStatus; print('phase2 import ok', ProjectStatus.INCOMPLETE_MAPPING.value)"
```

Result: all passed.

Phase 2 acceptance conclusion: passed for the implemented scope. Phase 3 has not
started.
