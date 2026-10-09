"""CAD_0 - dual-representation CAD kernel (native SDF + B-Rep).

Unified package: kernel (C++ extension) + Python convenience wrappers
+ PySide6 desktop UI.

Run the desktop app:
    cmake -B build -DCAD_0_BUILD_PYTHON=ON -DPython3_EXECUTABLE=$(which python3) -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build build
    PYTHONPATH=build/python python -m CAD_0.ui
"""

from __future__ import annotations

__version__ = "0.1.0"

# Import the compiled C++ extension
from . import _CAD_0  # noqa: F401

# Re-export kernel submodules
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

# Python-side io module (wraps C++ io with auto-detect export)
from . import io  # noqa: F401

# Python-side viewer (matplotlib 3D + 2D slices)
try:
    from . import viewer  # noqa: F401
except ImportError:
    pass  # matplotlib not installed

# UI module (PySide6 desktop app)
try:
    from . import ui  # noqa: F401
except ImportError:
    pass  # PySide6 not installed

__all__ = [
    "__version__",
    "math",
    "geometry",
    "sdf",
    "io",
    "viewer",
    "scene",
    "camera",
    "commands",
    "ui",
    "Shape",
    "ShapeId",
    "FeatureId",
    "DatumCS",
    "Transform",
    "Metadata",
    "Representation",
    "Units",
]
