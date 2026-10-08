"""CAD_0.ui — PySide6 desktop UI for CAD_0.

Run with:
    PYTHONPATH=build/python python -m CAD_0.ui
"""

import sys
import os

# Auto-detect build/python directory and add to sys.path.
# This file is at: <build_dir>/python/CAD_0/ui/__main__.py
# The _CAD_0.so is at: <build_dir>/python/CAD_0/_CAD_0.so
# So the parent of CAD_0/ (i.e. build/python/) must be on sys.path.
_this_dir = os.path.dirname(os.path.abspath(__file__))      # .../CAD_0/ui/
_pkg_dir = os.path.dirname(_this_dir)                         # .../CAD_0/
_build_python = os.path.dirname(_pkg_dir)                      # .../python/
_project_build = os.path.join(os.path.dirname(os.path.dirname(_build_python)), "build", "python")

for p in [_build_python, _project_build, os.path.join(os.getcwd(), "build", "python")]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from .main_window import main

if __name__ == "__main__":
    main()
