"""Graphics sprint — display normals from exact SDF gradients (contract).

The display pipeline's shading contract, pinned with tight margins on
the ADR-0027 perforated-box fixture (box extent 0.5 minus cylinder
r=0.2 h=2.0, tessellated at the viewport's resolution 48):

  1. CURVED SURFACE — every vertex on the hole wall carries the local
     radial as its display normal (dot > 0.99, oriented toward the hole
     axis: that is the outward direction of the perforated solid).
  2. FLAT SURFACE — every vertex on a flat box face carries the face
     axis (dot > 0.99).
  3. CREST (the point of the sprint) — a vertex on the hole wall at a
     known small distance from the box/hole rim keeps ITS region's
     normal (the radial), never an average with the top-face normal.
  4. FALLBACK — a vertex where the field gradient is ~0 (degenerate
     point) keeps the mesh normal; the buffer never contains NaN.
  5. BUFFER SANITY — every row is unit length, no NaN/inf, and the
     buffer is a deterministic pure function of (body, positions).
  6. EXPORT ISOLATION — the display pass never mutates the mesh: the
     raw buffers and the STL/OBJ/3MF exports are byte-identical before
     and after computing display normals (the stronger pre/post-commit
     byte proof runs outside the suite; this pins the in-process half).
  7. WIRING — the offscreen MainWindow viewport uploads the
     field-sampled buffer (bit-exact match against a per-node
     reconstruction through the real app flow).

Requires the compiled extension; the wiring test also needs PySide6
under QT_QPA_PLATFORM=offscreen (the established driver pattern).
"""

from __future__ import annotations

import numpy as np
import pytest

import CAD_0
from CAD_0 import math as cmath
from CAD_0 import scene as cscene

# The one pinned alignment threshold for the whole contract.
#
# Margin rationale: the display normal is the ANALYTIC gradient at the
# vertex and the reference direction (local radial / face axis) is
# evaluated AT THE SAME vertex, so the exact answer is dot = 1 - O(1e-7)
# (float32 rounding plus the defensive renormalization inside the CSG
# operators). 0.99 leaves two orders of magnitude of slack for that
# noise while still failing any real cross-crease contamination: a
# 50/50 blend of the hole-wall radial and the top-face normal lands at
# dot ~= 0.71, and even a 20%/80% blend only reaches ~= 0.97. No Approx,
# no per-test tuning — one threshold, one comment.
DOT_EPS = 0.99

# MC vertices sit on the surface up to the linear-interpolation chord
# error, ~cell^2 / (8*r) ~= 3e-4 for the hole wall at res 48. The
# selection band below is an order of magnitude wider (robust) yet far
# tighter than one grid cell (~0.022), so it cannot catch neighboring
# regions.
ON_SURFACE_TOL = 2e-3


def build_perforated_box():
    """The ADR-0027 fixture body: box extent 0.5^3 minus cylinder
    r=0.2 h=2.0 (Y axis) — the same geometry the app flow produces
    (registry defaults + tool radius 0.2)."""
    box = CAD_0.sdf.box(cmath.Vec3f(0.5, 0.5, 0.5))
    cyl = CAD_0.sdf.cylinder(0.2, 2.0)
    return CAD_0.sdf.subtract(box, cyl)


def _perforated_box_fixture(resolution: int = 48):
    """Scene with the ADR-0027 perforated box, tessellated like the
    viewport does (Scene.update_meshes — the display pipeline's own
    tessellation entry point)."""
    scene = cscene.Scene()
    scene.add_node("Box", build_perforated_box())
    scene.update_meshes(resolution)
    node = scene.node_at(0)
    return scene, node


def _display_buffer(node):
    """The display-pipeline buffer for a node — the exact call the
    viewport's _display_normals helper makes (viewport_widget.py)."""
    mesh = node.cached_mesh
    return CAD_0.sdf.display_normals(node.body, mesh.positions, mesh.normals)


# ---- 1. Curved surface: the hole wall shades as a true cylinder --------


def test_hole_wall_display_normal_is_local_radial():
    _, node = _perforated_box_fixture()
    mesh = node.cached_mesh
    pos = np.asarray(mesh.positions)
    dn = _display_buffer(node)

    rho = np.hypot(pos[:, 0], pos[:, 2])
    wall = (
        (np.abs(rho - 0.2) < ON_SURFACE_TOL)
        & (np.abs(pos[:, 1]) < 0.45)  # away from both rims
    )
    assert wall.sum() >= 100, (
        f"selection is broken: only {wall.sum()} wall vertices found")

    # Outward direction of the perforated solid on the hole wall points
    # TOWARD the axis: -radial. Evaluated at each vertex itself.
    inv_rho = 1.0 / rho[wall]
    expected = np.stack(
        [-pos[wall, 0] * inv_rho,
         np.zeros(int(wall.sum()), dtype=np.float64),
         -pos[wall, 2] * inv_rho], axis=1)
    dots = np.einsum("ij,ij->i", dn[wall], expected)
    assert dots.min() > DOT_EPS, (
        f"worst wall-vertex alignment dot = {dots.min():.4f}")


# ---- 2. Flat surface: box faces shade as their exact axes --------------


def test_flat_faces_display_normal_is_face_axis():
    _, node = _perforated_box_fixture()
    mesh = node.cached_mesh
    pos = np.asarray(mesh.positions)
    dn = _display_buffer(node)

    # +X face (the hole does not reach it: r = 0.2 < 0.5). Constrain
    # the face-interior coordinates (y, z) only — an x-radius bound
    # here would exclude the face itself.
    face_x = (
        (np.abs(pos[:, 0] - 0.5) < ON_SURFACE_TOL)
        & (np.abs(pos[:, 1]) < 0.45)
        & (np.abs(pos[:, 2]) < 0.45)
    )
    assert face_x.sum() >= 100
    dots = dn[face_x, 0]
    assert dots.min() > DOT_EPS, (
        f"worst +X face alignment dot = {dots.min():.4f}")

    # Top face (+Y), away from the hole rim.
    rho = np.hypot(pos[:, 0], pos[:, 2])
    face_y = (
        (np.abs(pos[:, 1] - 0.5) < ON_SURFACE_TOL)
        & (rho > 0.2 + 0.05)  # clear of the rim circle
    )
    assert face_y.sum() >= 100
    dots = dn[face_y, 1]
    assert dots.min() > DOT_EPS, (
        f"worst top-face alignment dot = {dots.min():.4f}")


# ---- 3. Crest: near the rim, the region's own normal — no average ------


def test_crease_vertex_keeps_its_region_normal():
    _, node = _perforated_box_fixture()
    mesh = node.cached_mesh
    pos = np.asarray(mesh.positions)
    dn = _display_buffer(node)

    # Grid cell size from the mesh's own span (the tessellation domain
    # is the padded body bounds at resolution 48).
    span = pos.max(axis=0) - pos.min(axis=0)
    cell = float(span.max()) / 48.0

    rho = np.hypot(pos[:, 0], pos[:, 2])
    # Wall vertices at a KNOWN small distance from the top rim: between
    # 1.5 and 3.5 cells below y = 0.5. Clearly inside the cylinder
    # region of the CSG min, but close enough that the crossing edge's
    # far grid corner sits in the top-face region — exactly where the
    # old corner-interpolated mesh normals smeared toward (0, 1, 0).
    dist_to_rim = 0.5 - np.abs(pos[:, 1])
    crest = (
        (np.abs(rho - 0.2) < ON_SURFACE_TOL)
        & (dist_to_rim > 1.5 * cell)
        & (dist_to_rim < 3.5 * cell)
        & (np.abs(pos[:, 1]) > 0)  # upper half, near the top rim
    )
    assert crest.sum() >= 10, (
        f"selection is broken: only {crest.sum()} near-crest vertices")

    radial_x = -pos[crest, 0] / rho[crest]
    radial_z = -pos[crest, 2] / rho[crest]
    dots = dn[crest, 0] * radial_x + dn[crest, 2] * radial_z
    assert dots.min() > DOT_EPS, (
        f"worst near-crest radial alignment dot = {dots.min():.4f} — "
        "the display normal is blending across the crease")

    # Explicit no-blend guard: the pure radial has ZERO y-component, so
    # any meaningful contamination from the top-face normal (0,1,0)
    # shows up as |n.y|. 0.1 is far above float noise (~1e-7) and far
    # below the ~0.45+ a 50/50 average would leave.
    assert np.abs(dn[crest, 1]).max() < 0.1, (
        f"max |n.y| near the crest = {np.abs(dn[crest, 1]).max():.4f} — "
        "the top-face normal is leaking into the hole-wall region")


# ---- 4. Fallback: degenerate gradient keeps the mesh normal ------------


def test_degenerate_gradient_falls_back_to_mesh_normal():
    sphere = CAD_0.sdf.sphere(1.0)
    # The sphere's center is THE canonical degenerate point: the field
    # gradient there is exactly (0,0,0) (SphereSDF normalizes p/r, and
    # Vec3f::normalized() returns the zero vector unchanged).
    pts = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
    fallback = np.array(
        [[0.0, 0.0, 1.0], [9.0, 9.0, 9.0], [0.0, 1.0, 0.0]], dtype=np.float32)

    out = CAD_0.sdf.display_normals(sphere, pts, fallback)
    assert np.array_equal(out[0], fallback[0]), (
        "degenerate vertex must keep the mesh normal")
    assert np.array_equal(out[2], fallback[2]), (
        "degenerate vertex must keep the mesh normal")
    # Surface vertex: the exact gradient, not the fallback.
    assert out[1, 0] > DOT_EPS
    assert not np.isnan(out).any()

    # Without a fallback the degenerate row is (0,0,0) — still never NaN.
    out = CAD_0.sdf.display_normals(sphere, pts[:1], None)
    assert np.array_equal(out[0], np.zeros(3, dtype=np.float32))
    assert not np.isnan(out).any()


# ---- 5. Buffer sanity: unit length, no NaN, deterministic --------------


def test_display_buffer_is_unit_length_no_nan_deterministic():
    _, node = _perforated_box_fixture()
    dn = _display_buffer(node)
    assert dn.shape == (len(node.cached_mesh.positions), 3)
    assert not np.isnan(dn).any()
    assert not np.isinf(dn).any()
    lengths = np.linalg.norm(dn.astype(np.float64), axis=1)
    assert np.abs(lengths - 1.0).max() < 1e-5, (
        f"worst |n| deviation = {np.abs(lengths - 1.0).max():.2e}")

    # Pure function of (body, positions): recompute, bit-identical.
    dn2 = _display_buffer(node)
    assert np.array_equal(dn, dn2)


# ---- 6. Export isolation: the display pass never touches the mesh ------


def test_display_pass_does_not_mutate_mesh_or_exports(tmp_path):
    import hashlib

    import CAD_0.io as cad_io

    _, node = _perforated_box_fixture()
    mesh = node.cached_mesh
    pos0 = np.array(mesh.positions)  # copies, compared after
    idx0 = np.array(mesh.indices)
    nrm0 = np.array(mesh.normals)

    def export_hashes(tag):
        out = {}
        for ext in ("stl", "obj", "3mf"):
            p = tmp_path / f"{tag}.{ext}"
            assert cad_io.export(str(p), mesh)
            out[ext] = hashlib.sha256(p.read_bytes()).hexdigest()
        return out

    before = export_hashes("before")
    dn = _display_buffer(node)  # the display pass
    assert dn is not None
    after = export_hashes("after")

    assert before == after, "exports changed across the display pass"
    assert np.array_equal(np.asarray(mesh.positions), pos0)
    assert np.array_equal(np.asarray(mesh.indices), idx0)
    assert np.array_equal(np.asarray(mesh.normals), nrm0)


# ---- 7. Wiring: the offscreen viewport uploads the field buffer --------


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_viewport_uploads_field_sampled_buffer(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # The established offscreen-driver pattern (test_ui_operations):
    # modal dialogs (close prompts, warnings) default to Discard — a
    # headless QMessageBox blocks forever otherwise.
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    monkeypatch.setattr(
        QMessageBox, "warning",
        staticmethod(lambda *a, **k: None))

    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    win = MainWindow()
    win._current_path = None
    try:
        assert win._has_kernel

        # The honest in-app flow (the ADR-0027 smoke pattern): default
        # box + cylinder, tool radius edited to 0.2, then subtract.
        win._create_registered("box")
        win._create_registered("cylinder")
        ids = []
        from PySide6.QtCore import Qt
        for i in range(win._feature_tree.topLevelItemCount()):
            it = win._feature_tree.topLevelItem(i)
            ids.append(it.data(0, Qt.UserRole))
        box_id, cyl_id = ids[-2:]
        win._node_params[cyl_id]["radius"] = 0.2
        win._rebuild_node(cyl_id)
        win._do_subtract(box_id, cyl_id, 0.0, 0.0, 0.0)

        vp = win._viewport
        assert vp._mesh_normals is not None

        # Reconstruct the expected buffer exactly as _refresh_meshes
        # builds it: visible, non-empty nodes in order, display normals
        # from the field for Sdf nodes.
        expected = []
        for i in range(win._scene.node_count()):
            node = win._scene.node_at(i)
            if node is None or not node.visible:
                continue
            mesh = node.cached_mesh
            pos = mesh.positions
            if pos is None or len(pos) == 0 or len(mesh.indices) == 0:
                continue
            expected.append(
                CAD_0.sdf.display_normals(node.body, pos, mesh.normals))
        expected = np.concatenate(expected)

        assert vp._mesh_normals.shape == expected.shape
        assert np.array_equal(vp._mesh_normals, expected), (
            "the viewport is not uploading the field-sampled buffer")
        assert not np.isnan(vp._mesh_normals).any()

        # ---- Optional item: View -> Tessellation Resolution -------------
        #
        # The menu exists with the four pinned options and 48 default;
        # changing it re-tessellates (round-trip 48 -> 32 -> 48), the
        # display buffer follows the new vertex count, and the 32 count
        # matches a fresh kernel-side tessellation at 32.
        tess_menu = None
        for top in win.menuBar().actions():
            if top.text() == "&View":
                for act in top.menu().actions():
                    if act.text() == "Tessellation &Resolution":
                        tess_menu = act.menu()
                        break
                break
        assert tess_menu is not None, "View menu has no Tessellation Resolution"
        texts = [a.text() for a in tess_menu.actions()]
        assert texts == ["&32", "&48", "&64", "&96"]
        checked = [a.text() for a in tess_menu.actions() if a.isChecked()]
        assert checked == ["&48"], f"default must be 48, got {checked}"

        v48 = len(vp._mesh_positions)
        win._set_tessellation_resolution(32)
        v32 = len(vp._mesh_positions)
        assert v32 != v48, "resolution change did not re-tessellate"
        assert vp._mesh_normals.shape == (v32, 3)

        # Cross-check against the kernel path: a fresh scene
        # tessellated at 32 must produce the same vertex count.
        scene32 = cscene.Scene()
        scene32.add_node("Box", build_perforated_box())
        scene32.update_meshes(32)
        assert v32 == len(np.asarray(scene32.node_at(0).cached_mesh.positions))

        # Round-trip back to 48 (the re-dirty pass must work in both
        # directions, and the default-resolution mesh must come back).
        win._set_tessellation_resolution(48)
        assert len(vp._mesh_positions) == v48

    finally:
        from PySide6.QtCore import QCoreApplication, QEvent
        win.close()
        win.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        app.processEvents()
        del win
        import gc
        gc.collect()
