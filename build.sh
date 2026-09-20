#!/usr/bin/env bash
# Regenerate everything from params.json (needs FreeCAD 1.x: freecadcmd on PATH)
set -e
cd "$(dirname "$0")"
freecadcmd -c "import runpy; runpy.run_path('build.py')" 2>&1 | grep -v "%)" | grep -v "^\s*$" | grep -v "^Recompute\|^MbD\|^Time = "
python3 tools_bundle.py
