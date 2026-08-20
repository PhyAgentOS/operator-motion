# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller one-file bundles for the two Motion Dora executables."""

from pathlib import Path

import pinocchio
from PyInstaller.building.api import EXE, PYZ
from PyInstaller.building.build_main import Analysis
from PyInstaller.utils.hooks import collect_all


SPEC_DIR = Path(SPEC).resolve().parent
ROOT = SPEC_DIR.parent

datas = []
binaries = []
hiddenimports = []

# These packages contain dynamic imports, generated message modules, native
# extensions, or package data that static import analysis cannot reliably see.
for package in (
    "dora",
    "forge_tool",
    "forge_msgs",
    "forge_kinematics",
    "pinocchio",
    "cmeel",
    "numpy",
    "yaml",
):
    package_datas, package_binaries, package_hiddenimports = collect_all(package)
    datas += package_datas
    binaries += package_binaries
    hiddenimports += package_hiddenimports

# Pinocchio is installed by cmeel below cmeel.prefix. Include that prefix's
# shared libraries explicitly; PyInstaller will retain the SONAME aliases that
# its dependency analysis needs in the one-file extraction directory.
cmeel_lib = Path(pinocchio.__file__).resolve().parents[3]
binaries += [
    (str(library), ".")
    for library in cmeel_lib.glob("*.so*")
    if library.is_file()
]

# Keep the TOCs deterministic and avoid embedding a file more than once when
# package collectors overlap.
datas = list(dict.fromkeys(datas))
binaries = list(dict.fromkeys(binaries))
hiddenimports = list(dict.fromkeys(hiddenimports))


def analysis(entry: str) -> Analysis:
    return Analysis(
        [str(SPEC_DIR / entry)],
        pathex=[
            str(ROOT / "packages" / "motion_server" / "src"),
            str(ROOT / "src"),
        ],
        binaries=binaries,
        datas=datas,
        hiddenimports=hiddenimports,
        hookspath=[],
        hooksconfig={},
        runtime_hooks=[],
        excludes=[],
        noarchive=False,
        optimize=0,
    )


def executable(bundle: Analysis, name: str) -> EXE:
    pyz = PYZ(bundle.pure)
    return EXE(
        pyz,
        bundle.scripts,
        bundle.binaries,
        bundle.datas,
        [],
        name=name,
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=False,
        upx_exclude=[],
        runtime_tmpdir=None,
        console=True,
        disable_windowed_traceback=False,
        argv_emulation=False,
        target_arch=None,
        codesign_identity=None,
        entitlements_file=None,
    )


motion_server_analysis = analysis("pyinstaller_entry_motion_server.py")
motion_server = executable(motion_server_analysis, "motion_server")

motion_action_policy_analysis = analysis("pyinstaller_entry_action_policy.py")
motion_action_policy = executable(motion_action_policy_analysis, "motion_action_policy")
