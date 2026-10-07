"""Native SDF kernel — Python convenience wrappers.

The bindings expose the C++ classes under `cadforge.sdf`. We do not wrap
them further; the goal is to keep the Python API as close as possible to
the C++ API so that examples and tutorials translate 1:1.
"""

from ._cadforge import sdf as _sdf

# Convenience re-exports
from ._cadforge.sdf import (
    SDFBody,
    EvalResult,
    CpuEvalBackend,
    TriangleMesh,
    MarchingCubesOptions,
    sphere,
    box,
    cylinder,
    torus,
    cone,
    capsule,
    plane,
    union,
    intersect,
    subtract,
    smooth_union,
    translate,
    rotate,
    scale,
    twist,
    evaluate,
    marching_cubes,
    weld_vertices,
)

__all__ = [
    "SDFBody",
    "EvalResult",
    "CpuEvalBackend",
    "TriangleMesh",
    "MarchingCubesOptions",
    "sphere",
    "box",
    "cylinder",
    "torus",
    "cone",
    "capsule",
    "plane",
    "union",
    "intersect",
    "subtract",
    "smooth_union",
    "translate",
    "rotate",
    "scale",
    "twist",
    "evaluate",
    "marching_cubes",
    "weld_vertices",
]
