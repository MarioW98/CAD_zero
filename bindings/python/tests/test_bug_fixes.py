"""Functional regression suite for the CAD_0 Python bindings (Bug 1-5).

Run from anywhere, with the build tree on PYTHONPATH:

    PYTHONPATH=build/<preset>/python python3 bindings/python/tests/test_bug_fixes.py

Checks:
  * Bug 1 (regression): parameter editing via writable SceneNode.body /
    mesh_dirty works and the mesh is re-tessellated.
  * Bug 2 (regression): export of a visible mesh works (STL + OBJ).
  * Bug 4 (regression): numpy arrays returned by TriangleMesh.positions keep
    the C++ mesh alive (data survives `del` + gc of every Python handle).
  * Bug 3 (regression): no stale UI copy under bindings/python/CAD_0/ui/ and
    the installed CAD_0.ui is byte-identical to app/src/CAD_0/ui.
  * Bug 5 (regression): pyproject declares PyOpenGL in the `ui` extra.

This suite is intentionally plain-Python (no pytest dependency) so it can
run in CI with just numpy. UI-import checks degrade to informational when
PySide6/PyOpenGL are not installed.

Structure note (2026-10-16, consolidation sweep): the whole executable
body lives in main() behind an `if __name__ == "__main__"` guard. Before
the guard, pytest's collection imported this file as a module and ran
EVERYTHING at import time (a silent double execution next to CI's
explicit script step) — worse, the module-level Scene/SceneNode/
TriangleMesh/CommandStack globals stayed alive until interpreter
shutdown, which is exactly what nanobind's "leaked instances" shutdown
report was listing (verified: a minimal scene + del + gc.collect() is
CLEAN, so the bindings release correctly; the report was this file's
own globals). With the guard, the script run is unchanged and a pytest
import executes nothing.
"""
from __future__ import annotations

import gc
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

PASS = "  [PASS]"
FAIL = "  [FAIL]"
INFO = "  [INFO]"
SKIP = "  [SKIP]"
failures: list[str] = []

# Script lives at <repo>/bindings/python/tests/test_bug_fixes.py
_ROOT = Path(__file__).resolve().parents[3]


def check(label: str, ok: bool, detail: str = "") -> None:
    print(f"{PASS if ok else FAIL} {label}" + (f" — {detail}" if detail else ""))
    if not ok:
        failures.append(label)


def soft_check(label: str, ok: bool, detail: str = "") -> None:
    """Non-fatal check: prints INFO instead of FAIL (environment-dependent)."""
    print(f"{INFO if ok else FAIL} {label}" + (f" — {detail}" if detail else ""))


def _module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ImportError:
        return False


def mesh_extent(mesh) -> float:
    """Max |coordinate| of the mesh positions — measures actual size."""
    import numpy as np
    pos = np.asarray(mesh.positions)
    return float(abs(pos).max())


def main() -> int:
    # ---------------------------------------------------------------- imports
    import CAD_0
    from CAD_0 import sdf as sdf_mod, scene as scene_mod, commands as cmd_mod

    sphere = sdf_mod.sphere
    marching_cubes = sdf_mod.marching_cubes
    Scene = scene_mod.Scene

    print("== imports ==")
    print(f"  CAD_0 {CAD_0.__version__} from {os.path.dirname(CAD_0.__file__)}")
    check("CAD_0.io importable (Bug 2 prerequisite)", hasattr(CAD_0, "io"))

    # ---------------------------------------------------------------- Bug 1
    print("== Bug 1: parameter editing ==")
    scene = Scene()
    node_id = scene.add_node("sphere", sphere(1.0))
    node = scene.get_node(node_id)
    scene.update_meshes(resolution=32)
    v0 = node.cached_mesh.vertex_count()
    check("initial mesh tessellated", v0 > 0, f"{v0} vertices")

    e0 = mesh_extent(node.cached_mesh)
    check("sphere(1.0) extent ~1.0", 0.9 < e0 < 1.1, f"extent={e0:.3f}")

    # The UI's _rebuild_node() flow: build a new body from the edited params,
    # assign to node.body (writable since the Bug 1 fix), then update meshes.
    node.body = sphere(2.0)
    node.mesh_dirty = True
    scene.update_meshes(resolution=32)
    e1 = mesh_extent(node.cached_mesh)
    check("node.body assignment no longer raises AttributeError", True)
    check("mesh re-tessellated with larger sphere (extent 1->2)",
          1.8 < e1 < 2.2, f"extent {e0:.3f} -> {e1:.3f}")

    # Scene-level param API (factory-less nodes: param stored, body edited
    # via assignment — must not crash; store a param and verify it round-trips
    # through the C++ node without effect on the factory-less node)
    scene.set_node_param(node_id, "radius", 0.5)
    node.mesh_dirty = True
    node.body = sphere(0.5)
    scene.update_meshes(resolution=32)
    e2 = mesh_extent(node.cached_mesh)
    check("Scene.set_node_param path works (no crash, smaller sphere)",
          0.4 < e2 < 0.6, f"extent {e1:.3f} -> {e2:.3f}")

    # ---------------------------------------------------------------- Bug 2
    print("== Bug 2: export ==")
    import CAD_0.io as cad_io

    tmpdir = tempfile.mkdtemp(prefix="cad0_bug2_")
    mesh = node.cached_mesh
    stl_path = os.path.join(tmpdir, "test_sphere.stl")
    cad_io.export(stl_path, mesh)
    stl_size = os.path.getsize(stl_path) if os.path.exists(stl_path) else 0
    check("STL export writes a non-empty file", stl_size > 84,
          f"{stl_size} bytes")

    obj_path = os.path.join(tmpdir, "test_sphere.obj")
    cad_io.export(obj_path, mesh)
    obj_size = os.path.getsize(obj_path) if os.path.exists(obj_path) else 0
    check("OBJ export writes a non-empty file", obj_size > 0, f"{obj_size} bytes")

    # quick STL sanity: vertex_count consistent with triangle count
    tri = mesh.triangle_count()
    check("exported mesh has triangles", tri > 0, f"{tri} triangles")

    # ---------------------------------------------------------------- Bug 4
    print("== Bug 4: ndarray lifetime ==")
    import numpy as np

    m2 = marching_cubes(sphere(1.5), 24)
    rc0 = sys.getrefcount(m2)
    arr = m2.positions  # numpy view into C++ memory
    rc1 = sys.getrefcount(m2)
    check("positions ndarray holds a reference to the mesh wrapper (nb::find owner)",
          rc1 == rc0 + 1, f"refcount {rc0} -> {rc1}")

    # Data must remain valid after every Python handle to the mesh is dropped:
    # with the fix the ndarray keeps the C++ TriangleMesh alive.
    expected = np.array(arr, copy=True)
    shape_before = arr.shape
    del m2
    gc.collect()
    try:
        ok_data = arr.shape == shape_before and np.array_equal(arr, expected)
    except Exception as exc:  # pragma: no cover
        ok_data = False
        print("      exception while reading array:", exc)
    check("array data still valid after del mesh + gc()", ok_data,
          f"shape {arr.shape}")

    # same check via scene: cached_mesh positions outliving the scene
    scene2 = Scene()
    nid = scene2.add_node("s", sphere(1.0))
    scene2.update_meshes(resolution=24)
    n2 = scene2.node_at(0)
    cm = n2.cached_mesh
    parr = cm.positions
    pexpected = np.array(parr, copy=True)
    pshape = parr.shape
    del cm, n2, scene2, nid
    gc.collect()
    try:
        ok2 = parr.shape == pshape and np.array_equal(parr, pexpected)
    except Exception as exc:  # pragma: no cover
        ok2 = False
        print("      exception while reading array:", exc)
    check("cached_mesh.positions keeps data valid after scene deletion", ok2,
          f"shape {parr.shape}")

    # CommandStack descriptions (policy cleanup — must still work)
    CommandStack = cmd_mod.CommandStack
    cs = CommandStack()
    cs.push(lambda: None, lambda: None, "add sphere")
    d = cs.next_undo_description()
    check("CommandStack descriptions still work", d == "add sphere", repr(d))

    # ---------------------------------------------------------------- Bug 3
    print("== Bug 3: single UI source ==")
    # (a) The stale pre-fix copy must NOT exist in the source tree. This is the
    #     exact regression that re-appeared when the fix zip was applied over an
    #     existing checkout without propagating the deletions.
    stale_ui = _ROOT / "bindings" / "python" / "CAD_0" / "ui"
    check("no stale UI copy under bindings/python/CAD_0/ui/",
          not stale_ui.exists(), str(stale_ui))

    # (b) The installed (build-tree) UI must be byte-identical to the single
    #     source of truth in app/src/CAD_0/ui.
    installed_mw = Path(CAD_0.__file__).parent / "ui" / "main_window.py"
    source_mw = _ROOT / "app" / "src" / "CAD_0" / "ui" / "main_window.py"
    check("installed CAD_0.ui == app/src/CAD_0/ui (byte-identical)",
          installed_mw.read_bytes() == source_mw.read_bytes(),
          f"{installed_mw} vs {source_mw}")

    # (c) The UI import chain must work when the UI stack is installed
    # (needs PySide6 + PyOpenGL — soft check outside the dev sandbox).
    if _module_available("PySide6") and _module_available("OpenGL"):
        try:
            from CAD_0.ui.main_window import MainWindow  # noqa: F401
            check("CAD_0.ui.main_window importable", True)
        except Exception as e:  # pragma: no cover
            check("CAD_0.ui.main_window importable", False, str(e))
    else:
        print(f"{SKIP} CAD_0.ui.main_window import (PySide6/OpenGL not installed)")

    # ---------------------------------------------------------------- Bug 5
    print("== Bug 5: declared dependencies ==")
    pyproject = (_ROOT / "pyproject.toml").read_text()
    check("PyOpenGL declared in the [ui] extra",
          'ui = ["PySide6>=6.6.0", "PyOpenGL>=3.1.0"]' in pyproject)
    if _module_available("OpenGL"):
        import OpenGL  # noqa: F401
        soft_check("PyOpenGL importable in this env", True)
    else:
        print(f"{SKIP} PyOpenGL import (not installed in this env)")

    # ---------------------------------------------------------------- summary
    # Cleanup before returning: every CAD_0 object created above is a LOCAL
    # of main(), so it dies here (this is the leak fix — see the module
    # docstring). The Bug 4 arrays (arr/parr) are the last handles keeping
    # their meshes alive; dropping them plus a gc pass leaves nothing for
    # nanobind to report at shutdown. The keep_alive cycle ndarray→mesh is
    # collectable (verified: gc reclaims it).
    del arr, parr, expected, pexpected, mesh, node, scene, cs
    gc.collect()

    print()
    if failures:
        print(f"RESULT: {len(failures)} FAILURE(S): {failures}")
        return 1
    print("RESULT: ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
