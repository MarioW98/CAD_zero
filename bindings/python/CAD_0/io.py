"""IO module — re-exports from the compiled extension.

The compiled extension exposes:
    CAD_0.io.export_stl_binary(path, mesh, name="CAD_0")
    CAD_0.io.export_stl_ascii(path, mesh, name="CAD_0")
    CAD_0.io.export_obj(path, mesh, name="CAD_0")
    CAD_0.io.export_3mf(path, mesh, name="CAD_0", application="CAD_0")
    CAD_0.io.save_native(path, scene, camera=None)   — native .CAD_0 format
    CAD_0.io.load_native(path, scene)                — returns the camera dict

For Pythonic convenience, we also expose a single high-level `export()`
function that auto-detects the format from the file extension, plus
`save()` / `load()` wrappers around the native project format
(`.CAD_0`, ADR-0007: JSON + MAJOR.MINOR.PATCH versioning).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

from ._CAD_0 import io as _io
from ._CAD_0.sdf import TriangleMesh

# Version of the native project format this build reads/writes.
NATIVE_VERSION: str = _io.NATIVE_VERSION

# Camera keys accepted/returned by save()/load() (mirrors the desktop
# app's orbit camera; fov is in degrees).
CAMERA_KEYS = ("target", "distance", "yaw", "pitch", "fov_deg")


def export(path: str | Path, mesh: TriangleMesh, *, name: str = "CAD_0") -> bool:
    """Export a triangle mesh to a file, auto-detected from the extension.

    Supported extensions:
        .stl  -> binary STL
        .obj  -> Wavefront OBJ
        .3mf  -> 3MF (XML in ZIP)

    For ASCII STL, call `CAD_0.io.export_stl_ascii` explicitly.
    """
    p = Path(path)
    suffix = p.suffix.lower()
    if suffix == ".stl":
        return _io.export_stl_binary(str(p), mesh, name)
    if suffix == ".obj":
        return _io.export_obj(str(p), mesh, name)
    if suffix == ".3mf":
        return _io.export_3mf(str(p), mesh, name, "CAD_0")
    raise ValueError(
        f"Unknown file extension '{suffix}'. Use .stl, .obj, or .3mf (projects: save())."
    )


def save(path: str | Path, scene: Any, camera: Optional[dict] = None) -> None:
    """Save a project to the native .CAD_0 format (ADR-0007).

    The write is atomic: the file is serialized to ``<path>.tmp`` and
    renamed over ``path`` only on success, so a crash mid-save never
    destroys the previous file. Raises RuntimeError with an explicit
    message on failure.

    Args:
        path: destination file (conventionally with a ``.CAD_0`` extension).
        scene: a CAD_0.scene.Scene.
        camera: optional dict with keys ``target`` (3-sequence), ``distance``,
            ``yaw``, ``pitch``, ``fov_deg`` — the viewport camera state.
    """
    _io.save_native(str(Path(path)), scene, camera)


def load(path: str | Path, scene: Any) -> dict:
    """Load a native .CAD_0 project into ``scene`` (transactional).

    The document is fully parsed and validated BEFORE the scene is
    touched: a corrupted or incompatible file raises RuntimeError and
    leaves the scene unchanged. On success the scene contains the saved
    nodes with their original ids, and the saved camera is returned as a
    dict (see CAMERA_KEYS) — or default values if the file had none.
    """
    return _io.load_native(str(Path(path)), scene)


# Re-export the low-level functions too.
export_stl_binary = _io.export_stl_binary
export_stl_ascii = _io.export_stl_ascii
export_obj = _io.export_obj
export_3mf = _io.export_3mf
save_native = _io.save_native
load_native = _io.load_native


__all__ = [
    "TriangleMesh",
    "NATIVE_VERSION",
    "CAMERA_KEYS",
    "export",
    "export_stl_binary",
    "export_stl_ascii",
    "export_obj",
    "export_3mf",
    "save",
    "load",
    "save_native",
    "load_native",
]
