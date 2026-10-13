"""CAD_0 viewport picking regression tests.

Feature (CODESPACES.md, Controls): "Click on shape | Select it". A short
click on a 3D shape must select it — in the feature tree, in the
property panel, and with a visible highlight in the 3D viewport.

The picking chain under test:

    _ViewportWindow.mouseReleaseEvent   (short click, <5px drag)
      -> wrapper ``clicked(ndc_x, ndc_y)`` signal
      -> MainWindow._on_viewport_click   (ray-AABB over node bounds)
      -> selection in the feature tree -> property panel + 3D highlight

Three groups of tests:

* HANDLER tests emit the wrapper's ``clicked`` signal directly (NDC
  coordinates) — fully headless (offscreen QPA), no GL context, no
  window manager. They cover the ray/AABB math, the wide-aspect
  geometry (a maximized window is never square), the selection
  FEEDBACK (tree + property panel + viewport highlight state), and
  picking of imported mesh-kind nodes (STL/OBJ).
* BRIDGE tests synthesize real QMouseEvents on a standalone
  _ViewportWindow (never shown, so no GL context is needed — works in
  any environment) and assert the pixel -> NDC mapping, the <5px
  short-click threshold and the left-button-only rule.
* The full GL embed path (real window container) is covered separately
  in test_ui_navigation.py::test_short_click_still_picks (needs GL).
"""

from __future__ import annotations

import pytest

_ROOT = __import__("pathlib").Path(__file__).resolve().parents[3]


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.fixture()
def main_window(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # Avoid modal dialogs blocking headless runs.
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    yield window
    window.close()
    window.deleteLater()
    app.processEvents()


def _scene_probe(w):
    """Diagnostic dump of every node's bounds (for failure messages)."""
    lines = []
    for i in range(w._scene.node_count()):
        n = w._scene.node_at(i)
        pick = w._node_pick_bounds(n)
        lines.append(
            f"node {i} id={n.id} kind={n.kind!r} visible={n.visible} "
            f"pick_bounds={pick}")
    cam = (w._viewport._cam_target, w._viewport._cam_distance,
           w._viewport._cam_yaw, w._viewport._cam_pitch,
           w._viewport._cam_aspect, w._viewport._cam_fov)
    return "camera=(target=%s dist=%s yaw=%s pitch=%s aspect=%s fov=%s)" % cam \
        + " | " + " ; ".join(lines)


# ---------------------------------------------------------------------------
# HANDLER tests (emit clicked(ndc) directly — no GL needed)
# ---------------------------------------------------------------------------

def test_center_click_selects_sphere(main_window):
    """The center of the screen looks exactly at the camera target.

    With the default camera (target origin, distance 5, yaw 0, pitch
    0.3) and a sphere at the origin, the NDC (0, 0) ray passes through
    the sphere's bounding box: the click MUST select the sphere.
    """
    w = main_window
    assert w._has_kernel, "kernel not available — set PYTHONPATH to build/python"

    w._create_registered("sphere")
    assert w._scene.size() == 1
    assert w._feature_tree.topLevelItemCount() == 1

    # Default camera: documented contract of ViewportWidget.__init__.
    assert w._viewport._cam_target == (0.0, 0.0, 0.0)
    assert w._viewport._cam_distance == 5.0
    assert w._viewport._cam_yaw == 0.0
    assert w._viewport._cam_pitch == 0.3

    # Center click: the ray from the camera through NDC (0, 0) looks
    # exactly at the target (the sphere's center).
    w._viewport.clicked.emit(0.0, 0.0)

    from PySide6.QtCore import Qt
    current = w._feature_tree.currentItem()
    assert current is not None, (
        "center click selected nothing\n" + _scene_probe(w))
    assert current.data(0, Qt.UserRole) == w._scene.node_at(0).id, (
        "wrong node selected\n" + _scene_probe(w))


def test_click_gives_3d_highlight_feedback(main_window):
    """FEEDBACK: a click must reach the viewport's highlight state.

    The original bug report ("click never selects") was caused by the
    selection having NO visible effect in the 3D view: the renderer
    drew every node in the same color. The fix routes the tree
    selection into ViewportWidget.set_selection, and _refresh_meshes
    records per-node triangle ranges so _draw_meshes can repaint the
    selected node in the highlight color. This test pins the plumbing.
    """
    w = main_window
    assert w._has_kernel

    w._create_registered("sphere")
    node_id = w._scene.node_at(0).id

    # The node's triangles have a recorded range (highlight target).
    assert node_id in w._viewport._node_ranges, (
        "no triangle range recorded for the sphere — "
        "3D highlight cannot draw")
    rng = w._viewport._node_ranges[node_id]
    assert rng[1] > rng[0], "empty triangle range for the sphere"

    assert w._viewport._selected_node_id is None
    w._viewport.clicked.emit(0.0, 0.0)

    assert w._viewport._selected_node_id == node_id, (
        "click did not set the viewport highlight state\n"
        + _scene_probe(w))

    # Deselecting from the tree clears the highlight too (the tree and
    # the 3D view can never disagree).
    w._feature_tree.clearSelection()
    assert w._viewport._selected_node_id is None


def test_offcenter_click_with_wide_aspect_hits_sphere(main_window):
    """A maximized window is NOT square: _cam_aspect is ~2.

    With aspect 2.0 the ray through NDC (0.05, 0.05) crosses the
    target plane at ~(0.21, 0.10) — well inside the sphere's
    [-1, 1]^3 box (the default sphere radius is 1.0). A click on a
    shape that sits slightly off the exact center of a WIDE window
    must still pick it: the picking ray and the renderer's frustum
    must agree on the aspect factor.
    """
    w = main_window
    assert w._has_kernel

    w._create_registered("sphere")
    w._viewport._cam_aspect = 2.0  # what resizeGL() sets for an 800x400 GL window

    w._viewport.clicked.emit(0.05, 0.05)

    from PySide6.QtCore import Qt
    current = w._feature_tree.currentItem()
    assert current is not None, (
        "off-center click on a wide window selected nothing\n"
        + _scene_probe(w))
    assert current.data(0, Qt.UserRole) == w._scene.node_at(0).id


def test_far_offcenter_click_with_wide_aspect_misses(main_window):
    """The same wide aspect: NDC (0.4, 0) is OFF the shape.

    The ray crosses the z=+1 face plane at x~1.3 (> 1) and the x=+1
    plane at z~1.9 (> 1): the [-1, 1]^3 box is missed entirely —
    nothing may be selected. Guards against a picking ray that would
    "swallow" every click (e.g. a wrong aspect or a degenerate box).
    """
    w = main_window
    assert w._has_kernel

    w._create_registered("sphere")
    w._viewport._cam_aspect = 2.0

    w._viewport.clicked.emit(0.4, 0.0)

    assert w._feature_tree.currentItem() is None, (
        "a click far off the shape must select nothing\n"
        + _scene_probe(w))


def test_empty_space_click_deselects_current_selection(main_window):
    """Clicking where there is NO object clears the current selection.

    A center click selects the sphere (tree row + 3D highlight +
    property panel); a click on empty space must then deselect it,
    exactly like Ctrl-clicking the selected tree row: no tree
    selection, no 3D highlight, "No selection" in the property panel.
    With the default camera (fov 45, aspect 1.0, pitch 0.3) the ray
    through NDC (0.9, 0.9) misses the [-1, 1]^3 sphere box entirely
    (its y-slab is only reachable at t < 0, behind the camera).
    """
    w = main_window
    assert w._has_kernel

    w._create_registered("sphere")
    node_id = w._scene.node_at(0).id
    w._viewport.clicked.emit(0.0, 0.0)
    # The click selected: all three feedback places agree.
    assert w._feature_tree.selectedItems() != []
    assert w._viewport._selected_node_id == node_id
    assert w._property_panel._current_node_name is not None

    # Click on empty space -> deselect everywhere.
    w._viewport.clicked.emit(0.9, 0.9)

    assert w._feature_tree.selectedItems() == [], (
        "click on empty space must clear the tree selection\n"
        + _scene_probe(w))
    assert w._selected_node_id is None, (
        "click on empty space must reset _selected_node_id")
    assert w._viewport._selected_node_id is None, (
        "click on empty space must clear the 3D highlight")
    assert w._property_panel._current_node_name is None, (
        "click on empty space must clear the property panel")
    assert w._property_panel._header.text() == "No selection"


def test_empty_space_click_without_selection_is_noop(main_window):
    """Clicking empty space with nothing selected changes nothing.

    No selection was ever made; the miss must be a silent no-op: no
    crash, no phantom selection, scene untouched.
    """
    w = main_window
    assert w._has_kernel

    w._create_registered("sphere")
    w._viewport.clicked.emit(0.9, 0.9)

    assert w._feature_tree.selectedItems() == []
    assert w._selected_node_id is None
    assert w._viewport._selected_node_id is None
    assert w._scene.size() == 1


def test_selected_node_id_initialized_in_constructor(main_window):
    """MainWindow._selected_node_id exists from construction.

    Regression guard: the attribute used to be assigned only in
    _rebuild_ui_from_scene()/_on_tree_selection_changed() and read
    elsewhere with a defensive getattr(). It now has one explicit
    initialization in __init__ (None = nothing selected), so plain
    attribute access is safe at any point of the widget's life.
    """
    w = main_window
    assert w._selected_node_id is None


def test_imported_mesh_node_is_clickable(main_window, tmp_path):
    """Mesh-kind nodes (imported STL/OBJ) are first-class pickables.

    The old handler skipped ``kind == "mesh"`` nodes (no SDF body);
    the fix computes their AABB from the mesh's own positions, so
    clicking imported geometry selects it — with the same highlight
    feedback as SDF shapes.
    """
    w = main_window
    assert w._has_kernel

    stl = _write_cube_stl(tmp_path)  # unit cube centered at the origin
    assert w._import_mesh(str(stl), "stl") is True
    assert w._scene.size() == 1
    node = w._scene.node_at(0)
    assert node.kind == "mesh"

    # Center click: the ray looks straight at the cube's center.
    w._viewport.clicked.emit(0.0, 0.0)

    from PySide6.QtCore import Qt
    current = w._feature_tree.currentItem()
    assert current is not None, (
        "click on an imported mesh selected nothing\n" + _scene_probe(w))
    assert current.data(0, Qt.UserRole) == node.id

    # Same highlight feedback as SDF nodes: the imported geometry is in
    # the render batch (cached_mesh is display_mesh in the bindings),
    # so its triangle range exists and the selection reaches the
    # viewport highlight state.
    assert w._viewport._selected_node_id == node.id
    assert node.id in w._viewport._node_ranges

    # And a click far off the small cube must not select it. (Assert on
    # selectedItems(): Qt's currentItem persists across clearSelection.)
    w._feature_tree.clearSelection()
    w._viewport.clicked.emit(0.9, 0.0)
    assert w._feature_tree.selectedItems() == [], (
        "click off the imported cube must select nothing\n"
        + _scene_probe(w))


def _write_cube_stl(tmp_path):
    """ASCII STL: a unit cube centered at the origin (12 triangles)."""
    h = 0.5
    faces = [
        ((0, 0, -1), [(-h, -h, -h), (h, -h, -h), (h, h, -h)],
                   [(-h, -h, -h), (h, h, -h), (-h, h, -h)]),
        ((0, 0, 1), [(-h, -h, h), (-h, h, h), (h, h, h)],
                   [(-h, -h, h), (h, h, h), (h, -h, h)]),
        ((0, -1, 0), [(-h, -h, -h), (-h, -h, h), (h, -h, h)],
                     [(-h, -h, -h), (h, -h, h), (h, -h, -h)]),
        ((0, 1, 0), [(-h, h, -h), (h, h, -h), (h, h, h)],
                    [(-h, h, -h), (h, h, h), (-h, h, h)]),
        ((-1, 0, 0), [(-h, -h, -h), (-h, h, -h), (-h, h, h)],
                     [(-h, -h, -h), (-h, h, h), (-h, -h, h)]),
        ((1, 0, 0), [(h, -h, -h), (h, -h, h), (h, h, h)],
                    [(h, -h, -h), (h, h, h), (h, h, -h)]),
    ]
    lines = ["solid cube"]
    for normal, t1, t2 in faces:
        for tri in (t1, t2):
            lines.append(f"facet normal {normal[0]} {normal[1]} {normal[2]}")
            lines.append("  outer loop")
            for v in tri:
                lines.append(f"    vertex {v[0]} {v[1]} {v[2]}")
            lines.append("  endloop")
            lines.append("endfacet")
    lines.append("endsolid cube")
    path = tmp_path / "cube.stl"
    path.write_text("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------------------
# BRIDGE tests (real QMouseEvents on a standalone GL window — no context
# is ever created because the window is never shown; runs anywhere)
# ---------------------------------------------------------------------------

@pytest.fixture()
def gl_bridge(monkeypatch):
    """A ViewportWidget + standalone _ViewportWindow (800x400, unshown)."""
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.viewport_widget import ViewportWidget, _ViewportWindow

    wrapper = ViewportWidget()
    gl = _ViewportWindow(wrapper)
    gl.resize(800, 400)  # non-square on purpose: aspect 2.0
    received = []
    wrapper.clicked.connect(lambda x, y: received.append((x, y)))
    yield wrapper, gl, received
    gl.deleteLater()
    wrapper.deleteLater()
    app.processEvents()


def _send_mouse(gl, kind, x, y, button, buttons):
    from PySide6.QtCore import QCoreApplication, QPointF
    from PySide6.QtGui import QMouseEvent
    ev = QMouseEvent(kind, QPointF(x, y), QPointF(x, y), button, buttons,
                     __import__("PySide6.QtCore", fromlist=["Qt"]).Qt
                     .KeyboardModifier.NoModifier)
    QCoreApplication.sendEvent(gl, ev)


def _click_gl(gl, received, x, y, drag=0):
    from PySide6.QtCore import QEvent, Qt
    received.clear()
    _send_mouse(gl, QEvent.Type.MouseButtonPress, x, y,
                Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton)
    if drag:
        _send_mouse(gl, QEvent.Type.MouseMove, x + drag, y + drag,
                    Qt.MouseButton.NoButton, Qt.MouseButton.LeftButton)
    _send_mouse(gl, QEvent.Type.MouseButtonRelease, x, y,
                Qt.MouseButton.LeftButton, Qt.MouseButton.NoButton)


def test_bridge_pixel_to_ndc_non_square(gl_bridge):
    """Pixel -> NDC mapping on a non-square (800x400) window.

    A maximized window is never square; the bridge divides by the GL
    window's own width/height, which must match the window the events
    arrive on. Verified at the exact center, a corner, and a quarter
    point.
    """
    _, gl, received = gl_bridge
    assert (gl.width(), gl.height()) == (800, 400)

    _click_gl(gl, received, 400, 200)
    assert received == [(0.0, 0.0)], f"center: {received}"

    _click_gl(gl, received, 0, 0)
    assert received == [(-1.0, 1.0)], f"top-left corner: {received}"

    _click_gl(gl, received, 600, 100)
    assert received and abs(received[0][0] - 0.5) < 1e-9 \
        and abs(received[0][1] - 0.5) < 1e-9, f"(600,100): {received}"


def test_bridge_short_click_vs_drag(gl_bridge):
    """<5px of total movement = click (picks); >=5px = drag (orbits)."""
    from PySide6.QtCore import Qt
    _, gl, received = gl_bridge

    # A 2px jitter is still a click.
    _click_gl(gl, received, 400, 200, drag=2)
    assert received == [(0.0, 0.0)], f"2px jitter should pick: {received}"

    # A 6px drag is an orbit, never a pick.
    _click_gl(gl, received, 400, 200, drag=6)
    assert received == [], "6px drag must NOT pick"


def test_bridge_right_button_never_picks(gl_bridge):
    """Right/middle clicks are navigation, not selection."""
    from PySide6.QtCore import QEvent, Qt
    _, gl, received = gl_bridge
    received.clear()

    _send_mouse(gl, QEvent.Type.MouseButtonPress, 400, 200,
                Qt.MouseButton.RightButton, Qt.MouseButton.RightButton)
    _send_mouse(gl, QEvent.Type.MouseButtonRelease, 400, 200,
                Qt.MouseButton.RightButton, Qt.MouseButton.NoButton)
    assert received == [], "right click must not pick"
