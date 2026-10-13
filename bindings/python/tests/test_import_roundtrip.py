"""CAD_0 STL/OBJ import + mesh-node bindings — end-to-end Python tests.

FASE 2 of the import feature: the kernel loaders (io.load_stl /
io.load_obj), the mesh-kind scene node (scene.add_node(name, mesh)),
and the mixed-scene save/load round-trip through the Python API.

Pattern follows test_native_roundtrip.py.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import CAD_0
import CAD_0.io as cad_io
from CAD_0 import math as math_mod
from CAD_0 import sdf as sdf_mod
from CAD_0 import scene as scene_mod

import numpy as np
import pytest


def _camera() -> dict:
    return {
        "target": (0.0, 0.0, 0.0),
        "distance": 6.0,
        "yaw": 0.5,
        "pitch": 0.2,
        "fov_deg": 50.0,
    }


def _make_stl(tmp: Path, resolution: int = 24) -> Path:
    """A real imported file: MC sphere exported to binary STL, then loaded."""
    body = sdf_mod.sphere(1.5)
    mesh = sdf_mod.marching_cubes(body, resolution, True)
    path = tmp / "sphere.stl"
    assert cad_io.export_stl_binary(str(path), mesh, "sphere")
    return path


def _make_obj(tmp: Path) -> Path:
    body = sdf_mod.box(math_mod.Vec3f(1.0, 1.0, 1.0))
    mesh = sdf_mod.marching_cubes(body, 16, True)
    path = tmp / "box.obj"
    assert cad_io.export_obj(str(path), mesh, "box")
    return path


def _mesh_signature(mesh) -> tuple:
    """Importance-check signature: counts + bounds + position checksum."""
    positions = np.asarray(mesh.positions)
    indices = np.asarray(mesh.indices)
    return (
        positions.shape[0],
        indices.shape[0] // 3,
        float(positions.min()), float(positions.max()),
        float(np.abs(positions).sum()),
    )


# ---- Loaders exposed ----------------------------------------------------------


def test_loaders_are_exposed():
    for name in ("load_stl", "load_obj"):
        assert hasattr(cad_io, name)
        assert name in cad_io.__all__


def test_load_stl_binary_welds_back():
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        mesh = cad_io.load_stl(str(_make_stl(Path(d))))
        # The exported STL duplicates vertices per triangle; the loader
        # welds them: the vertex count matches a directly-tessellated mesh.
        direct = sdf_mod.marching_cubes(sdf_mod.sphere(1.5), 24, True)
        assert mesh.vertex_count() == direct.vertex_count()
        assert mesh.triangle_count() == direct.triangle_count()
        # STL facet normals are advisory -> the loaded mesh carries none
        # (the binding exposes empty normals as None).
        assert mesh.normals is None or mesh.normals.size == 0


def test_load_obj_shared_indices_noop_weld():
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        mesh = cad_io.load_obj(str(_make_obj(Path(d))))
        direct = sdf_mod.marching_cubes(
            sdf_mod.box(math_mod.Vec3f(1.0, 1.0, 1.0)), 16, True)
        assert mesh.vertex_count() == direct.vertex_count()
        assert mesh.triangle_count() == direct.triangle_count()
        # OBJ vn normals are loaded (per-position, first reference wins).
        assert mesh.normals is not None and mesh.normals.shape[0] > 0


def test_loader_errors_are_runtime_errors_with_path():
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        missing = str(Path(d) / "no_such_file.stl")
        with pytest.raises(RuntimeError, match="cannot open"):
            cad_io.load_stl(missing)
        with pytest.raises(RuntimeError, match="cannot open"):
            cad_io.load_obj(str(Path(d) / "no_such_file.obj"))

        garbage = Path(d) / "garbage.stl"
        garbage.write_bytes(b"not an stl at all")
        with pytest.raises(RuntimeError, match='must start with the "solid"'):
            cad_io.load_stl(str(garbage))


# ---- Mesh-kind scene nodes ----------------------------------------------------


def test_add_mesh_node_and_kind_dispatch():
    scene = scene_mod.Scene()
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        mesh = cad_io.load_stl(str(_make_stl(Path(d))))

        sdf_id = scene.add_node("sphere", sdf_mod.sphere(1.0))
        mesh_id = scene.add_node("imported", mesh)

        sdf_node = scene.get_node(sdf_id)
        mesh_node = scene.get_node(mesh_id)

        assert sdf_node.kind == "sdf"
        assert mesh_node.kind == "mesh"

        # Honest dispatch: the mesh node has no builder/params.
        assert mesh_node.builder == ""
        assert dict(mesh_node.params) == {}

        # cached_mesh dispatches to the payload for mesh nodes.
        assert mesh_node.cached_mesh.vertex_count() == mesh.vertex_count()
        assert mesh_node.mesh.vertex_count() == mesh.vertex_count()

        # Assigning an SDF body to a mesh node is refused (ADR-0002).
        with pytest.raises(RuntimeError, match="cannot assign an SDF body"):
            mesh_node.body = sdf_mod.sphere(1.0)

        # Parametric editing is a no-op, not a crash.
        scene.set_node_param(mesh_id, "radius", 5.0)
        assert dict(mesh_node.params) == {}


def test_update_meshes_never_touches_mesh_nodes():
    scene = scene_mod.Scene()
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        mesh = cad_io.load_stl(str(_make_stl(Path(d), resolution=12)))
        before = np.array(mesh.positions).copy()
        scene.add_node("imported", mesh)
        scene.add_node("sphere", sdf_mod.sphere(1.0))

        scene.update_meshes(48)

        node = scene.node_at(0)
        assert np.array_equal(np.asarray(node.mesh.positions), before)
        assert node.mesh_dirty is False
        assert scene.node_at(1).cached_mesh.vertex_count() > 0


# ---- Mixed-scene persistence through the Python API ---------------------------


def test_mixed_scene_roundtrip():
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        tmp = Path(d)
        scene = scene_mod.Scene()

        # An SDF node with editing metadata + an imported mesh node.
        scene.add_node("sphere", sdf_mod.sphere(1.25))
        imported = cad_io.load_stl(str(_make_stl(tmp, resolution=16)))
        scene.add_node("imported_sphere", imported)
        probe = math_mod.Vec3f(0.5, 0.5, 0.5)
        sdf_values_before = [
            scene.node_at(i).body.value(probe) for i in (0,)
        ]
        sig_before = _mesh_signature(scene.node_at(1).mesh)

        path = tmp / "mixed.CAD_0"
        cad_io.save(path, scene, _camera())

        loaded = scene_mod.Scene()
        cam = cad_io.load(path, loaded)
        assert loaded.size() == 2

        # Kinds survive.
        assert loaded.node_at(0).kind == "sdf"
        assert loaded.node_at(1).kind == "mesh"

        # The mesh is identical: counts + bounds + checksum.
        sig_after = _mesh_signature(loaded.node_at(1).mesh)
        assert sig_after == sig_before

        # SDF evaluation is bit-identical.
        assert loaded.node_at(0).body.value(probe) == sdf_values_before[0]

        # Camera round-trips.
        assert cam["distance"] == 6.0
        assert cam["fov_deg"] == 50.0

        # Re-save is byte-identical (stable blob ids from node ids).
        # (Checked BEFORE the add_node below — that bumps next_id.)
        path2 = tmp / "mixed2.CAD_0"
        cad_io.save(path2, loaded, cam)
        assert path.read_bytes() == path2.read_bytes()

        # Ids keep growing monotonically.
        assert loaded.add_node("x", sdf_mod.sphere(0.5)) >= 2


def test_transactional_load_on_corrupt_blob():
    with tempfile.TemporaryDirectory(prefix="cad0_import_") as d:
        tmp = Path(d)
        scene = scene_mod.Scene()
        scene.add_node("sphere", sdf_mod.sphere(1.0))
        mesh = cad_io.load_obj(str(_make_obj(tmp)))
        scene.add_node("box", mesh)
        path = tmp / "mixed.CAD_0"
        cad_io.save(path, scene, _camera())

        # Corrupt the blob section: the container length check fires
        # (explicit error), and the target scene is untouched.
        data = path.read_bytes()
        path.write_bytes(data[:-8])
        target = scene_mod.Scene()
        target.add_node("keepme", sdf_mod.sphere(0.5))
        with pytest.raises(RuntimeError, match="truncated or corrupt"):
            cad_io.load(path, target)
        assert target.size() == 1


def test_formatted_output_survives_numpy_first_import():
    """Regression — gcc-11 CI toolchain (diagnosed 2026-10-14).

    gcc <= 11 emits the std::locale facet ``id`` statics as
    vague-linkage GNU_UNIQUE symbols in every DSO compiled with them.
    When the first libstdc++ in the process arrives through numpy's
    RTLD_LOCAL dependency chain (numpy.libs -> openblas/libgfortran ->
    libstdc++.so.6), the unique-symbol unification with the kernel's
    facet-id copies never happens: ``use_facet`` then throws
    std::bad_cast, libstdc++'s formatted-output wrappers swallow it as
    a plain failbit, and EVERY formatted stream write (``<<`` int /
    float) from gcc-11-compiled code fails silently — export_obj
    returned False and wrote truncated files (this is exactly what
    test_load_obj_shared_indices_noop_weld /
    test_transactional_load_on_corrupt_blob used to do on the gcc-11
    build before the fix).

    CAD_0.__init__ now promotes libstdc++.so.6 into the GLOBAL dlopen
    scope at package import (see _promote_libstdcpp_global in
    app/src/CAD_0/__init__.py). The in-process order (CAD_0 imported
    by an earlier test module, numpy at the top of THIS module) is
    already covered by the rest of this suite; this test covers the
    OTHER hostile order — numpy imported BEFORE CAD_0 — which can only
    be built in a fresh subprocess. Loader-scope only: no evaluation
    values change (ADR-0014 untouched), gcc >= 12 TUs never had the
    bug (the guard is a harmless no-op there).
    """
    import os
    import subprocess
    import sys
    import textwrap

    py_dir = os.path.dirname(os.path.dirname(CAD_0.__file__))
    code = textwrap.dedent(
        """
        import numpy  # FIRST: the hostile import order
        import CAD_0
        from CAD_0 import math as m, sdf as s
        import CAD_0.io as io
        import sys, tempfile, os
        body = s.box(m.Vec3f(1.0, 1.0, 1.0))
        mesh = s.marching_cubes(body, 16, True)
        assert mesh.triangle_count() > 0
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "box.obj")
            if not io.export_obj(p, mesh, "box"):
                raise SystemExit("export_obj FAILED with numpy-first import")
            with open(p) as f:
                head = f.read()
            if "# triangles:" not in head or "\\nv " not in head:
                raise SystemExit("truncated OBJ output")
        print("OK")
        """
    )
    env = dict(os.environ, PYTHONPATH=py_dir)
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True,
        env=env, timeout=120)
    assert result.returncode == 0, (
        f"numpy-first subprocess failed:\n{result.stdout}\n{result.stderr}")
    assert "OK" in result.stdout
