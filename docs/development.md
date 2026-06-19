# Development Baseline

This document records the Phase 0 development commands for the current baseline.
The current application entry point remains the V3.5 GUI. Phase 2 adds a
separate V4.0 data pipeline under `src/hjmb_pathgen/models`,
`src/hjmb_pathgen/codec`, and `src/hjmb_pathgen/services`.

## Supported Interpreter

Primary development and acceptance environment:

```powershell
python --version
```

Expected primary interpreter:

```text
Python 3.14.x, official Windows x64 CPython
```

Python 3.13 compatibility is allowed. MSYS2/MinGW Python is not required for this
project baseline.

## Create Environment

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -e .
```

The project depends on PySide6. For Python 3.14, use a PySide6 release whose
package metadata includes Python 3.14 support. The project metadata requires
`PySide6>=6.10.1,<7`.

## Start GUI

```powershell
python hjmb_path_editor.py
```

For an import smoke check without showing the GUI:

```powershell
python -c "import hjmb_path_editor; print('import ok')"
```

For a package smoke check from the repository without installation:

```powershell
python -c "import sys; sys.path.insert(0, 'src'); import hjmb_pathgen; print(hjmb_pathgen.__version__)"
```

For a headless Qt smoke check:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -c "from PySide6.QtWidgets import QApplication; import hjmb_path_editor as h; app=QApplication([]); win=h.MainWindow(); print('window ok')"
```

## CLI

```powershell
python path_codec_cli.py plan example_path.json
python path_codec_cli.py summary example_path.json
python path_codec_cli.py build example_path.json P0000.BIN
python path_codec_cli.py check P0000.BIN
```

`P0000.BIN` is a generated output and is ignored by Git.

## Tests

Run the current unittest suite:

```powershell
python -m unittest tests.unit.test_phase2_codecs tests.unit.test_phase2_services -v
```

```powershell
python -m unittest discover -s tests -v
```

Run the full regression suite, including legacy V3.5 tests:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Compile source files:

```powershell
python -m py_compile hjmb_path_editor.py path_codec_cli.py path_models.py path_geometry.py trajectory_planner.py trajectory_graphics.py batch_models.py batch_generator.py v35_test_utils.py
```
