"""CAD_0 UI navigation smoke test — BUG 1 (invisible menubar) + BUG 2
(dead viewport navigation with GL active) regressions.

Two layers, same policy as test_ui_import.py:

* Source-level checks (no PySide6 required, run everywhere): the
  MainWindow forces the menu bar to be non-native, and the viewport
  navigation handlers live on _ViewportWindow (the embedded GL window
  that really receives the events), NOT on the wrapper QWidget.

* Full end-to-end (PySide6 required; skipped when absent): headless
  MainWindow under QT_QPA_PLATFORM=offscreen — the menu bar carries
  File/Create/Edit/View plus the key actions (Open, Save, Import STL,
  Import OBJ); synthetic mouse/wheel events sent to the GL window steer
  the camera at the FIRST movement (orbit yaw/pitch, pan target, zoom
  distance); a short click crosses the picking bridge (the wrapper's
  ``clicked`` signal) and selects the SDF node under the cursor; the
  no-GL fallback path stays usable without exceptions.

The navigation part needs a real GL context (offscreen QPA + Mesa
software GL). When GL is unavailable the test skips with an explicit
reason; CODESPACES.md documents how to run it with software GL.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

# The app source is at <root>/app/src/CAD_0/ui/ (the build copies it over
# the Python package; the canonical copy is the app one).
_ROOT = Path(__file__).resolve().parents[3]
_UI_DIR = _ROOT / "app" / "src" / "CAD_0" / "ui"
UI_MAIN_WINDOW = _UI_DIR / "main_window.py"
UI_VIEWPORT = _UI_DIR / "viewport_widget.py"

_NAV_HANDLERS = ("mousePressEvent", "mouseMoveEvent", "mouseReleaseEvent",
                 "wheelEvent")


def _methods_of_class(src: str, class_name: str) -> set[str]:
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {n.name for n in node.body if isinstance(n, (ast.FunctionDef,))}
    return set()


# ---- Source-level checks (always run) ------------------------------------------


def test_menubar_forced_in_window_source():
    """BUG 1: the menubar is forced non-native right after menu creation."""
    src = UI_MAIN_WINDOW.read_text()
    assert "setNativeMenuBar(False)" in src
    assert "menuBar().setVisible(True)" in src


def test_navigation_handlers_live_on_gl_window_source():
    """BUG 2: input handlers on _ViewportWindow, none left on the wrapper."""
    src = UI_VIEWPORT.read_text()
    wrapper = _methods_of_class(src, "ViewportWidget")
    window = _methods_of_class(src, "_ViewportWindow")
    # One strategy only: the wrapper no longer carries any handler.
    for handler in _NAV_HANDLERS:
        assert handler not in wrapper, (
            f"{handler} must not live on ViewportWidget (wrapper never "
            "receives mouse events when GL is active)")
        assert handler in window, (
            f"{handler} must live on _ViewportWindow (the embedded GL "
            "window that really receives the events)")
    # Focus handling: losing the window mid-drag must reset the drag.
    assert "focusOutEvent" in window
    # The picking bridge: the short click re-emits the wrapper signal.
    assert "self._parent.clicked.emit" in src


def test_picking_skips_mesh_nodes_source():
    """Clicking with imported meshes in the scene must not degenerate."""
    src = UI_MAIN_WINDOW.read_text()
    # Mesh nodes have no SDF body: their empty bounds would swallow every
    # click. The honest dispatch skips them in the picking loop.
    assert 'if node.kind == "mesh":' in src


# ---- Full end-to-end (PySide6) --------------------------------------------------


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

    # Avoid modal dialogs blocking headless runs (close prompts).
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


def _require_gl_window(w):
    """The GL event path only exists when a GL context was created."""
    gl = w._viewport._gl_window
    if gl is None:
        pytest.skip(
            "GL context unavailable in this environment — the navigation "
            "smoke test needs software GL (offscreen QPA + Mesa). See "
            "CODESPACES.md for the documented local run.")
    return gl


def _mouse(kind, local, button, buttons):
    from PySide6.QtCore import QEvent, QPointF, Qt
    from PySide6.QtGui import QMouseEvent
    return QMouseEvent(kind, QPointF(local[0], local[1]),
                       QPointF(local[0], local[1]), button, buttons,
                       Qt.KeyboardModifier.NoModifier)


def _drag_gl(gl, start, end, button):
    """Press at ``start``, move to ``end``, release — on the GL window."""
    from PySide6.QtCore import QCoreApplication, QEvent, Qt
    press = _mouse(QEvent.Type.MouseButtonPress, start, button, button)
    move = _mouse(QEvent.Type.MouseMove, end, Qt.MouseButton.NoButton, button)
    release = _mouse(QEvent.Type.MouseButtonRelease, end, button,
                     Qt.MouseButton.NoButton)
    QCoreApplication.sendEvent(gl, press)
    QCoreApplication.sendEvent(gl, move)
    QCoreApplication.sendEvent(gl, release)


def _wheel_gl(gl, notches=1):
    from PySide6.QtCore import QCoreApplication, QPoint, QPointF, Qt
    from PySide6.QtGui import QWheelEvent
    ev = QWheelEvent(QPointF(100, 100), QPointF(100, 100), QPoint(0, 0),
                     QPoint(0, 120 * notches), Qt.MouseButton.NoButton,
                     Qt.KeyboardModifier.NoModifier,
                     Qt.ScrollPhase.NoScrollPhase, False)
    QCoreApplication.sendEvent(gl, ev)


def test_menu_bar_visible_with_key_actions(main_window):
    """BUG 1: the bar exists in-window and carries the app's menus."""
    w = main_window
    mb = w.menuBar()

    # Forced in-window (non-native) and not hidden: it renders as soon as
    # the window shows. (The test never calls show() on purpose — an
    # exposed+destroyed GL window segfaults at interpreter exit, a
    # pre-existing shutdown issue unrelated to this fix. Construction
    # already proves the bar is in the window's layout.)
    assert mb.isHidden() is False
    assert mb.isNativeMenuBar() is False

    # Top-level menus: File, Create, Edit, View (+ Help).
    titles = {a.text().replace("&", "") for a in mb.actions()}
    assert {"File", "Create", "Edit", "View"} <= titles

    # Key File actions exist: Open, Save, Import STL, Import OBJ.
    file_menu = next(a.menu() for a in mb.actions()
                     if a.text().replace("&", "") == "File")
    file_texts = {a.text().replace("&", "") for a in file_menu.actions()}
    assert "Open..." in file_texts
    assert "Save" in file_texts
    assert "Import STL..." in file_texts
    assert "Import OBJ..." in file_texts


def test_orbit_responds_at_first_movement(main_window):
    """BUG 2: a left drag on the GL window moves yaw/pitch immediately."""
    w = main_window
    gl = _require_gl_window(w)

    yaw0, pitch0 = w._viewport._cam_yaw, w._viewport._cam_pitch
    # ONE movement only: the camera must respond at the first drag step
    # (no warm-up events, no click-through-the-wrapper path).
    from PySide6.QtCore import QCoreApplication, QEvent, Qt
    QCoreApplication.sendEvent(
        gl, _mouse(QEvent.Type.MouseButtonPress, (100, 100),
                   Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton))
    QCoreApplication.sendEvent(
        gl, _mouse(QEvent.Type.MouseMove, (160, 70),
                   Qt.MouseButton.NoButton, Qt.MouseButton.LeftButton))

    assert w._viewport._cam_yaw > yaw0
    assert w._viewport._cam_pitch > pitch0  # screen up -> pitch up

    QCoreApplication.sendEvent(
        gl, _mouse(QEvent.Type.MouseButtonRelease, (160, 70),
                   Qt.MouseButton.LeftButton, Qt.MouseButton.NoButton))


def test_pan_moves_target(main_window):
    """BUG 2: a middle drag on the GL window pans the target."""
    w = main_window
    gl = _require_gl_window(w)

    target0 = w._viewport._cam_target
    _drag_gl(gl, (100, 100), (160, 140),
             __import__("PySide6").QtCore.Qt.MouseButton.MiddleButton)

    target1 = w._viewport._cam_target
    assert target1[0] < target0[0]  # drag right -> target moves left
    assert target1[1] > target0[1]  # drag down -> target moves up


def test_wheel_zooms_distance(main_window):
    """BUG 2: the wheel on the GL window changes the camera distance."""
    w = main_window
    gl = _require_gl_window(w)

    dist0 = w._viewport._cam_distance
    _wheel_gl(gl, notches=+3)  # zoom in
    assert w._viewport._cam_distance < dist0

    dist1 = w._viewport._cam_distance
    _wheel_gl(gl, notches=-3)  # zoom out
    assert w._viewport._cam_distance > dist1


def test_short_click_still_picks(main_window):
    """BUG 2: the picking bridge survives the event re-routing.

    A short click (press+release, no drag) must cross the GL window ->
    wrapper ``clicked`` signal -> MainWindow selection path.
    """
    w = main_window
    gl = _require_gl_window(w)
    assert w._has_kernel

    # A parametric SDF node at the origin (the registry path the Create
    # menu uses) — squarely under the default camera.
    w._create_registered("sphere")
    assert w._scene.size() == 1
    assert w._feature_tree.topLevelItemCount() == 1

    # Click the viewport center (NDC 0,0): the sphere's bounding box is
    # hit by the center ray and the node gets selected in the tree. The
    # unexposed offscreen window keeps a default size, so the click point
    # is computed from the window's ACTUAL geometry.
    from PySide6.QtCore import Qt
    gl.resize(400, 300)
    center = (gl.width() // 2, gl.height() // 2)
    _drag_gl(gl, center, center, Qt.MouseButton.LeftButton)  # zero-drag click

    current = w._feature_tree.currentItem()
    assert current is not None
    assert current.data(0, Qt.UserRole) == w._scene.node_at(0).id


def test_fallback_path_usable_without_exceptions(main_window, monkeypatch):
    """No-GL fallback: the window builds and stays usable, no handlers."""
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from CAD_0.ui.viewport_widget import ViewportWidget
    from CAD_0.ui.main_window import MainWindow

    # Force the GL probe to fail: the fallback label path must engage.
    monkeypatch.setattr(
        ViewportWidget, "_check_gl_available",
        staticmethod(lambda fmt: False))
    app = QApplication.instance() or QApplication([])
    w = MainWindow()

    assert w._viewport._gl_window is None
    assert w._viewport._gl_fallback_label is not None
    assert w._viewport._gl_container is None
    # The app is still a normal Qt app: menus work, no stray handlers.
    titles = {a.text().replace("&", "") for a in w.menuBar().actions()}
    assert {"File", "Create", "Edit", "View"} <= titles

    # Synthetic input on the fallback widget raises nothing (the wrapper
    # no longer carries navigation handlers — GL only).
    from PySide6.QtCore import QCoreApplication, QEvent, Qt
    press = _mouse(QEvent.Type.MouseButtonPress, (50, 50),
                   Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton)
    QCoreApplication.sendEvent(w._viewport, press)  # must not raise
    w._status.showMessage("fallback ok")

    w.close()
    w.deleteLater()
    app.processEvents()
