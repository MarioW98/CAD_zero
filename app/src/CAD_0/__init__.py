"""CAD_0 — dual-representation CAD kernel.

This __init__.py is used in the build/python/ directory where both the
compiled _CAD_0 extension and the Python UI live together.
"""

from __future__ import annotations

__version__ = "0.1.0"

# Import the compiled extension and re-export its submodules.
# This works when _CAD_0.so/.pyd is in the same directory as this file.
try:
    from . import _CAD_0  # noqa: F401
    from ._CAD_0 import math, geometry, sdf, scene, camera, commands  # noqa: F401
    from ._CAD_0.geometry import (  # noqa: F401
        Shape, ShapeId, FeatureId, DatumCS, Transform,
        Metadata, Representation, Units,
    )
    # Python-side io wrapper (auto-detect export from file extension).
    # Bug fix: this import was missing, so `CAD_0.io` (documented in the
    # README) raised AttributeError after a plain `import CAD_0`.
    from . import io  # noqa: F401
    _HAS_KERNEL = True
except ImportError:
    _HAS_KERNEL = False
