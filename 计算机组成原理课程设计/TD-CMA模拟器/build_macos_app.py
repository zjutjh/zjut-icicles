from __future__ import annotations

import os
from pathlib import Path
import shutil


ROOT = Path(__file__).resolve().parent
DIST_APP = ROOT / "dist" / "TD-CMA-Simulator.app"
RESOURCE_APP = DIST_APP / "Contents" / "Resources" / "app"
MACOS_DIR = DIST_APP / "Contents" / "MacOS"


INFO_PLIST = """<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key>
  <string>tdcma-launcher</string>
  <key>CFBundleIdentifier</key>
  <string>local.tdcma.simulator</string>
  <key>CFBundleName</key>
  <string>TD-CMA Simulator</string>
  <key>CFBundleDisplayName</key>
  <string>TD-CMA Simulator</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
  <key>CFBundleShortVersionString</key>
  <string>1.0</string>
  <key>CFBundleVersion</key>
  <string>1</string>
  <key>LSMinimumSystemVersion</key>
  <string>10.15</string>
</dict>
</plist>
"""


LAUNCHER = """#!/bin/zsh
set -e

APP_ROOT="$(cd "$(dirname "$0")/../Resources/app" && pwd)"
cd "$APP_ROOT"

exec /usr/bin/python3 web_app.py --auto-port --open
"""


def copy_path(src: Path, dst: Path) -> None:
    if dst.exists():
        if dst.is_dir():
            shutil.rmtree(dst)
        else:
            dst.unlink()
    if src.is_dir():
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store"))
    else:
        shutil.copy2(src, dst)


def main() -> None:
    if DIST_APP.exists():
        shutil.rmtree(DIST_APP)
    MACOS_DIR.mkdir(parents=True, exist_ok=True)
    RESOURCE_APP.mkdir(parents=True, exist_ok=True)

    (DIST_APP / "Contents" / "Info.plist").write_text(INFO_PLIST, encoding="utf-8")
    launcher = MACOS_DIR / "tdcma-launcher"
    launcher.write_text(LAUNCHER, encoding="utf-8")
    launcher.chmod(0o755)

    for name in ["web_app.py", "tdcma_core.py", "README.md"]:
        copy_path(ROOT / name, RESOURCE_APP / name)
    for name in ["web_static", "examples"]:
        copy_path(ROOT / name, RESOURCE_APP / name)

    print(f"created {DIST_APP}")


if __name__ == "__main__":
    main()

