# app/CMakeLists.txt — PySide6 desktop application (Phase C.6-C.8)
#
# Installs the Python package under app/src/CAD_0/ into the
# CAD_0 Python package directory so that `import CAD_0.ui` works.
#
# Phase C status: written but not exercised in CI. Requires PySide6
# (pip install CAD_0[ui]). The C++ kernel is fully tested; only this
# UI layer needs a display.

# Copy the Python sources next to the compiled extension.
# The bindings/python/CAD_0/ directory becomes the canonical location
# for the Python package; this app/ tree just adds the `ui` submodule.

install(DIRECTORY src/CAD_0/ui
    DESTINATION CAD_0
    FILES_MATCHING
        PATTERN "*.py"
)

# Phase H will add a briefcase/pyinstaller bundle target here.
