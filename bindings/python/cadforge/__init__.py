"""cadforge — dual-representation CAD kernel (native SDF + B-Rep).

Public API entry point. The compiled extension is imported lazily so that
`import cadforge` works even on a pure-Python checkout (e.g. to inspect
types or read docstrings).
"""

from __future__ import annotations

__version__ = "0.1.0"

# Lazy import: the compiled extension lives in `_cadforge`. We re-export
# the public symbols so that the canonical access path is `from cadforge
# import sdf` (not `from cadforge._cadforge import sdf`).
from . import _cadforge  # noqa: F401  (compiled extension)

from ._cadforge import math, geometry, sdf, io  # noqa: F401
from ._cadforge.geometry import (
    Shape,
    ShapeId,
    FeatureId,
    DatumCS,
    Transform,
    Metadata,
    Representation,
    Units,
)

__all__ = [
    "__version__",
    "math",
    "geometry",
    "sdf",
    "io",
    "Shape",
    "ShapeId",
    "FeatureId",
    "DatumCS",
    "Transform",
    "Metadata",
    "Representation",
    "Units",
]
