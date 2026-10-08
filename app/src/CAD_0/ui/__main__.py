"""Allow running the CAD_0 desktop app with `python -m CAD_0.ui`.

Usage:
    # Build first:
    cmake -B build ... -DCAD_0_BUILD_PYTHON=ON
    cmake --build build

    # Run from the project root:
    PYTHONPATH=build/python python -m CAD_0.ui
"""

import sys
import os

# Auto-detect build/python directory relative to this file.
# This file is at: <project_root>/build/python/CAD_0/ui/__main__.py
# or:              <project_root>/app/src/CAD_0/ui/__main__.py
_build_python = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "build", "python")
_build_python = os.path.abspath(_build_python)
if os.path.isdir(_build_python) and _build_python not in sys.path:
    sys.path.insert(0, _build_python)

# Also check CWD
_cwd_build = os.path.join(os.getcwd(), "build", "python")
if os.path.isdir(_cwd_build) and _cwd_build not in sys.path:
    sys.path.insert(0, _cwd_build)

from .main_window import main

if __name__ == "__main__":
    main()
