"""Allow running the CAD_0 desktop app with `python -m CAD_0.ui`.

Usage:
    cd CAD_0
    cmake -B build ... -DCAD_0_BUILD_PYTHON=ON
    cmake --build build
    PYTHONPATH=build/python python -m CAD_0.ui
"""

from .main_window import main

if __name__ == "__main__":
    main()
