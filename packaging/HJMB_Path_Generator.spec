# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

ROOT = Path(SPECPATH).parent

datas = [
    (str(ROOT / "HJMB_path_file_protocol_v4.0.txt"), "protocol"),
    (str(ROOT / "docs"), "docs"),
]
examples_dir = ROOT / "examples"
if examples_dir.exists():
    datas.append((str(examples_dir), "examples"))

a = Analysis(
    [str(ROOT / "hjmb_path_editor.py")],
    pathex=[str(ROOT), str(ROOT / "src")],
    binaries=[],
    datas=datas,
    hiddenimports=[
        "hjmb_pathgen.py_ui.main_window",
        "hjmb_pathgen.py_app.cli_main",
        "PySide6.QtCore",
        "PySide6.QtGui",
        "PySide6.QtWidgets",
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tests", "tkinter"],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="HJMB_Path_Generator",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="HJMB_Path_Generator_V4.0",
)
