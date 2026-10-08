"""CAD_0 package — Python UI layer + kernel access.

This package hosts the PySide6 desktop UI. The actual CAD kernel lives
in the compiled `_CAD_0` extension (see bindings/python/).

When running from the source tree, set PYTHONPATH to include the
build/python directory so the compiled extension is found:

    PYTHONPATH=build/python python -m CAD_0.ui

Or run directly from the build/python directory where both the compiled
extension and this package are installed.
"""

from __future__ import annotations

__version__ = "0.1.0"

# Try to import the compiled extension. If it fails, the UI will still
# open but with limited functionality (no kernel access).
try:
    from . import _CAD_0  # noqa: F401
    from ._CAD_0 import math, geometry, sdf, scene, camera, commands  # noqa: F401
    from ._CAD_0.geometry import (  # noqa: F401
        Shape,
        ShapeId,
        FeatureId,
        DatumCS,
        Transform,
        Metadata,
        Representation,
        Units,
    )
    _HAS_KERNEL = True
except ImportError:
    _HAS_KERNEL = False
