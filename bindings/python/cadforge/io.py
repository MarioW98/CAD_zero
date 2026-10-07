"""IO module — re-exports from the compiled extension.

The compiled extension exposes:
    cadforge.io.export_stl_binary(path, mesh, name="cadforge")
    cadforge.io.export_stl_ascii(path, mesh, name="cadforge")
    cadforge.io.export_obj(path, mesh, name="cadforge")
    cadforge.io.export_3mf(path, mesh, name="cadforge", application="cadforge")

For Pythonic convenience, we also expose a single high-level `export()`
function that auto-detects the format from the file extension.
"""

from __future__ import annotations

from pathlib import Path

from ._cadforge import io as _io
from ._cadforge.sdf import TriangleMesh


def export(path: str | Path, mesh: TriangleMesh, *, name: str = "cadforge") -> bool:
    """Export a triangle mesh to a file, auto-detected from the extension.

    Supported extensions:
        .stl  -> binary STL
        .obj  -> Wavefront OBJ
        .3mf  -> 3MF (XML in ZIP)

    For ASCII STL, call `cadforge.io.export_stl_ascii` explicitly.
    """
    p = Path(path)
    suffix = p.suffix.lower()
    if suffix == ".stl":
        return _io.export_stl_binary(str(p), mesh, name)
    if suffix == ".obj":
        return _io.export_obj(str(p), mesh, name)
    if suffix == ".3mf":
        return _io.export_3mf(str(p), mesh, name, "cadforge")
    raise ValueError(
        f"Unknown file extension '{suffix}'. Use .stl, .obj, or .3mf."
    )


# Re-export the low-level functions too.
export_stl_binary = _io.export_stl_binary
export_stl_ascii = _io.export_stl_ascii
export_obj = _io.export_obj
export_3mf = _io.export_3mf


__all__ = [
    "TriangleMesh",
    "export",
    "export_stl_binary",
    "export_stl_ascii",
    "export_obj",
    "export_3mf",
]
