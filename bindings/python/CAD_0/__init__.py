"""CAD_0 - dual-representation CAD kernel (native SDF + B-Rep).

Public API entry point. The compiled extension is imported lazily so that
`import CAD_0` works even on a pure-Python checkout (e.g. to inspect
types or read docstrings).
"""

from __future__ import annotations

__version__ = "0.1.0"

# Lazy import: the compiled extension lives in `_CAD_0`. We re-export
# the public symbols so that the canonical access path is `from CAD_0
# import sdf` (not `from CAD_0._CAD_0 import sdf`).
from . import _CAD_0  # noqa: F401  (compiled extension)

from ._CAD_0 import math, geometry, sdf, io  # noqa: F401
from ._CAD_0.geometry import (
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
