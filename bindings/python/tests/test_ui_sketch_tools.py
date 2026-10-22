# bindings/python/tests/test_ui_sketch_tools.py
#
# ADR-0030 D6 — the wiring smoke (offscreen, the established driver):
# the TOOL SELECTOR and the EXTRUDE-from-menu reachability. The canvas
# (sketch_canvas.py) implements TOOLS with set_tool() — complete, and
# untouched by the wiring; what these tests pin is the way to REACH
# it, which did not exist (the diagnosis: zero set_tool call sites,
# no button, no palette, a status bar promising tools the UI never
# carried). Every test below goes through the wiring (the actions,
# the shortcuts, the menu slots) — none calls canvas.set_tool behind
# the widget's back, so each one FAILS before the wiring and passes
# after: the regression that nails the defect for good.
#
#   selector lifecycle    -> test_selector_appears_with_the_view_and_hides
#   per-view state        -> test_selector_sets_the_active_view_tool
#   pinned shortcuts      -> test_shortcut_changes_tool_and_checked_state
#   THE KEY TEST          -> test_circle_gesture_after_selector_wiring
#   the empty hint        -> test_empty_canvas_hint_pinned_channel
#   inline editor digits  -> test_inline_editor_digits_not_shortcuts
#   extrude from the menu -> test_extrude_from_menu_prism_or_visible_refusal
#   the whole sequence    -> test_the_hole_sequence_from_menus_and_canvas
from __future__ import annotations

import gc

import pytest


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.fixture()
def sketch_app(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # Modal boxes never block headless runs; the refusal and
    # instruction texts are captured for the assertions that need
    # them (the gestures smoke's pattern, extended to information).
    refusals = []
    informations = []
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    monkeypatch.setattr(
        QMessageBox, "critical",
        staticmethod(lambda *a, **k: refusals.append(a[2] if len(a) > 2
                                                    else str(a))))
    monkeypatch.setattr(
        QMessageBox, "information",
        staticmethod(lambda *a, **k: informations.append(a[2] if len(a) > 2
                                                        else str(a))))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    window._refusals = refusals
    window._informations = informations
    yield window
    from PySide6.QtCore import QCoreApplication, QEvent
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    del window
    gc.collect()


def _open_canvas(win):
    """Create → Sketch (the menu slot), the canvas pinned to the
    deterministic view: centre on the origin, 60 px per world unit."""
    before = win._command_stack.undo_count()
    win.on_create_sketch()
    assert win._command_stack.undo_count() == before + 1
    sid = max(win._sketch_views)
    canvas = win._sketch_views[sid]
    canvas.resize(800, 600)
    canvas.show()
    canvas._center = (0.0, 0.0)
    canvas._scale = 60.0
    return sid, canvas


def _tree_row(win, node_id):
    """The feature-tree row of a node id — a LOOKUP only, no
    selection side effect (the subtract needs a pure multi-select)."""
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            return it
    return None


def _select_tree_row(win, node_id):
    """The user's click on a feature-tree row (the selection surface
    the Feature commands consume — unit 2's pinned rule)."""
    it = _tree_row(win, node_id)
    if it is not None:
        win._feature_tree.setCurrentItem(it)
    return it


def _circle_gesture(canvas, c, rim):
    """The circle gesture (D5.1's pinned press-drag-release) with NO
    tool switch inside — the test's whole point is that the WIRING
    (action or shortcut) is what set the tool."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))


def _segment_gesture(canvas, p0, p1):
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*p0))
    QTest.mouseMove(canvas, QPoint(*p1))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*p1))


def _find_operation(win, op_type, **match):
    """The id of the operation node of the given type (the newest when
    several match), or None — the menu slots return nothing, the scene
    is the truth."""
    found = None
    for i in range(win._scene.node_count()):
        node = win._scene.node_at(i)
        op = getattr(node, "operation", None)
        if op is None or op.get("type") != op_type:
            continue
        if any(op.get(k) != v for k, v in match.items()):
            continue
        found = node.id
    return found


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_selector_appears_with_the_view_and_hides(sketch_app):
    """The selector is visible exactly while a canvas view is the
    current tab: hidden at startup (the declared choice — with nothing
    to act on there is no selector, and its shortcuts are inert
    too), visible with the sketch view, hidden again on the 3D view
    and after the view closes."""
    win = sketch_app
    # No sketch view: hidden (isVisibleTo ignores the unshown window —
    # what is pinned is the selector's own visibility state).
    assert not win._sketch_toolbar.isVisibleTo(win)

    sid, _canvas = _open_canvas(win)
    assert win._sketch_toolbar.isVisibleTo(win)

    # The 3D view tab: hidden again.
    win._tabs.setCurrentWidget(win._viewport)
    assert not win._sketch_toolbar.isVisibleTo(win)

    # Back to the sketch view: visible, and reflecting ITS tool.
    win._tabs.setCurrentWidget(win._sketch_views[sid])
    assert win._sketch_toolbar.isVisibleTo(win)

    # Closing the sketch view (the only one): the selector hides.
    win._on_tab_close_requested(win._tabs.indexOf(win._sketch_views[sid]))
    assert not win._sketch_toolbar.isVisibleTo(win)
    assert sid not in win._sketch_views


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_selector_sets_the_active_view_tool(sketch_app):
    """The four buttons call set_tool on the ACTIVE view's canvas —
    each view keeps its OWN tool (per-view state), the checked action
    is the readable active state, and switching tabs re-reflects the
    view that comes to the front."""
    win = sketch_app
    sid_a, canvas_a = _open_canvas(win)
    sid_b, canvas_b = _open_canvas(win)

    # Two views, both on the default tool.
    assert canvas_a.tool() == "select"
    assert canvas_b.tool() == "select"
    assert win._tool_actions["select"].isChecked()

    # B is the active view: the buttons act on B only.
    win._tool_actions["circle"].trigger()
    assert canvas_b.tool() == "circle"
    assert canvas_a.tool() == "select"        # per-view state
    assert win._tool_actions["circle"].isChecked()

    # Switch to A: the selector reflects A's own tool.
    win._tabs.setCurrentWidget(canvas_a)
    assert win._tool_actions["select"].isChecked()
    win._tool_actions["arc"].trigger()
    assert canvas_a.tool() == "arc"
    assert canvas_b.tool() == "circle"        # untouched

    # All four actions route (the round trip over the group).
    for tool in ("segment", "select", "circle", "arc"):
        win._tool_actions[tool].trigger()
        assert canvas_a.tool() == tool
        assert win._tool_actions[tool].isChecked()


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_shortcut_changes_tool_and_checked_state(sketch_app):
    """The pinned keyboard shortcuts (1/2/3/4) switch the ACTIVE
    view's tool, and the checked action follows — the same wiring the
    buttons use, reached by key. (Shortcuts need the window shown:
    the offscreen QPA delivers them to the active window only.)"""
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication

    win = sketch_app
    _sid, canvas = _open_canvas(win)
    win.show()
    QApplication.instance().processEvents()

    QTest.keyClick(canvas, Qt.Key_3)
    assert canvas.tool() == "circle"
    assert win._tool_actions["circle"].isChecked()

    QTest.keyClick(canvas, Qt.Key_1)
    assert canvas.tool() == "select"
    assert win._tool_actions["select"].isChecked()

    QTest.keyClick(canvas, Qt.Key_2)
    assert canvas.tool() == "segment"
    QTest.keyClick(canvas, Qt.Key_4)
    assert canvas.tool() == "arc"
    assert win._tool_actions["arc"].isChecked()


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_circle_gesture_after_selector_wiring(sketch_app):
    """THE KEY TEST — the switching-on: after the Circle ACTION (and,
    separately, after its shortcut), the simulated drawing gesture
    PRODUCES the entity with the expected parameters. Before the
    wiring the canvas sat on the default tool and the click did
    nothing: this is the regression that nails the defect for good.
    """
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication

    win = sketch_app
    sid, canvas = _open_canvas(win)
    undo_before = win._command_stack.undo_count()

    # The button path: click Circle on the selector, then draw.
    win._tool_actions["circle"].trigger()
    assert canvas.tool() == "circle"
    _circle_gesture(canvas, (400, 300), (520, 300))
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and len(chains[0]) == 1
    circ = chains[0][0]
    assert circ["kind"] == "circle"
    assert circ["center"] == (0.0, 0.0)
    assert circ["radius"] == pytest.approx(2.0, abs=1e-6)
    assert circ["hole"] is False            # OUTER, the declared default
    assert win._command_stack.undo_count() == undo_before + 1

    # The shortcut path: key 3, then the same gesture — a second
    # circle, its own chain (the gesture's values land 1:1 again).
    # The shown window lays the canvas out at its own size: the
    # gesture is aimed from the ACTUAL widget centre (which the
    # pinned transform maps to the world origin).
    win._tool_actions["select"].trigger()
    win.show()
    QApplication.instance().processEvents()
    QTest.keyClick(canvas, Qt.Key_3)
    assert canvas.tool() == "circle"
    cx, cy = canvas.width() // 2, canvas.height() // 2
    expected_center = canvas.screen_to_world(cx, cy)
    _circle_gesture(canvas, (cx, cy), (cx + 120, cy))
    chains = win._scene.sketch_chains(sid)
    circ2 = chains[1][0]
    assert circ2["kind"] == "circle"
    assert circ2["center"][0] == pytest.approx(expected_center[0], abs=1e-6)
    assert circ2["center"][1] == pytest.approx(expected_center[1], abs=1e-6)
    assert circ2["radius"] == pytest.approx(2.0, abs=1e-6)
    assert win._command_stack.undo_count() == undo_before + 2


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_empty_canvas_hint_pinned_channel(sketch_app):
    """The empty-canvas hint travels ONLY through the pinned status
    channel (the canvas status bar, D5.3/D5.5 — no new graphic
    element): shown while the view has no entities, dropped by the
    first gesture, back after an undo empties the view again — and a
    refusal standing in the channel is never clobbered."""
    win = sketch_app
    sid, canvas = _open_canvas(win)

    # An empty view: the hint is in the pinned channel.
    assert canvas.status_message == win.EMPTY_SKETCH_HINT

    # The first gesture clears the channel (the canvas's own rule)
    # and the entity keeps it clear: no hint on a non-empty view.
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))
    assert canvas.status_message == ""

    # An undo empties the view again: the hint returns.
    win.on_undo()
    assert win._scene.sketch_chains(sid) == [[]]
    assert canvas.status_message == win.EMPTY_SKETCH_HINT

    # A refusal keeps the channel: a degenerate circle (r ~ 0) on the
    # empty view refuses visibly, and the hint does NOT overwrite it.
    _circle_gesture(canvas, (400, 300), (401, 300))
    assert "circle refused" in canvas.status_message
    assert canvas.status_message != win.EMPTY_SKETCH_HINT


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_inline_editor_digits_not_shortcuts(sketch_app):
    """The digit shortcuts do not steal text input: typing into the
    canvas's inline editor (a focused QLineEdit) edits the value, the
    tool does not move — the wiring's scope guard, pinned here so it
    cannot silently regress."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication

    win = sketch_app
    sid, canvas = _open_canvas(win)
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))   # r = 2.0
    win.show()
    QApplication.instance().processEvents()

    # Select the circle, double-click its radius label: the inline
    # editor opens focused (D5.2's text affordance).
    canvas.select_entity(0, 0)
    labels = canvas.dimension_labels()
    r_label = next(t for t in labels if t.startswith("r ="))
    ls = canvas.world_to_screen(*labels[r_label][0])
    QTest.mouseDClick(canvas, Qt.LeftButton, Qt.NoModifier,
                      QPoint(int(ls[0]) + 4, int(ls[1]) - 4))
    assert canvas._inline_editor is not None
    editor = canvas._inline_editor
    editor.setText("")

    # The digit reaches the EDITOR, not the shortcut; Return commits
    # the edit through the one write route.
    QTest.keyClicks(editor, "3")
    assert editor.text() == "3"
    assert canvas.tool() == "circle"         # the tool did not move
    QTest.keyClick(editor, Qt.Key_Return)
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(3.0, abs=1e-6)
    assert canvas.tool() == "circle"


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_extrude_from_menu_prism_or_visible_refusal(sketch_app,
                                                    monkeypatch):
    """Feature → Extrude from the MENU: on a selected CLOSED sketch
    the prism lands (a body, the FEAT row, the recipe wired); on an
    OPEN sketch the kernel's refusal is visible and NO body is born;
    with no sketch selected the instruction is visible — never
    silence, never a guess."""
    from CAD_0 import math as _math

    win = sketch_app

    # No selection: the visible instruction, nothing created.
    win.on_extrude()
    assert any("Select a sketch node" in m for m in win._informations)
    assert win._scene.node_count() == 0

    # A closed sketch (one circle — a circle IS a closed chain):
    sid, canvas = _open_canvas(win)
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))
    assert win._scene.sketch_closed(sid)

    # The menu path with the dialog answered (distance 1.5, +Z).
    _select_tree_row(win, sid)
    assert win._selected_node_id == sid
    monkeypatch.setattr(win, "_extrude_dialog", lambda node: (1.5, 1))
    win.on_extrude()

    prism_id = _find_operation(win, "extrude", sketch=sid)
    assert prism_id is not None
    prism = win._scene.get_node(prism_id)
    assert prism.kind == "sdf"                # the prism IS a body
    assert not prism.operand_missing
    # The extrusion spans [0, 1.5] on +Z: the axis probe sits 0.75
    # from both slab faces (the established probe style).
    assert prism.body.value(_math.Vec3f(0.0, 0.0, 0.75)) == \
        pytest.approx(-0.75, abs=1e-6)
    row = _select_tree_row(win, prism_id)
    assert row.text(1) == "FEAT"

    # An OPEN sketch (one segment — a free endpoint): the kernel
    # refuses, the message is visible, NO body is born.
    count = win._scene.node_count()
    sid2, canvas2 = _open_canvas(win)
    win._tool_actions["segment"].trigger()
    _segment_gesture(canvas2, (100, 100), (340, 220))
    assert not win._scene.sketch_closed(sid2)
    _select_tree_row(win, sid2)
    win.on_extrude()                          # dialog still patched
    assert _find_operation(win, "extrude", sketch=sid2) is None
    assert win._scene.node_count() == count + 1   # only the sketch2 node
    refusal = win._refusals[-1]
    assert "segment[chain 0 #" in refusal
    assert "open" in refusal                  # the chain is open, said


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_hole_sequence_from_menus_and_canvas(sketch_app, monkeypatch):
    """The whole parametric hole, from menus and canvas only (the
    order's acceptance sequence): Create → Box (edited through the
    panel), Create → Sketch, the circle drawn by gesture after the
    selector, Feature → Extrude, Edit → Subtract, Debug → the genus-1
    report."""
    from PySide6.QtCore import Qt
    from CAD_0 import math as _math

    win = sketch_app

    # Create → Box (the menu command's slot), extents via the panel.
    win._create_registered("box")
    box_id = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.text(1) == "SDF":
            box_id = it.data(0, Qt.UserRole)
    assert box_id is not None
    _select_tree_row(win, box_id)
    for axis in ("extent_x", "extent_y", "extent_z"):
        win._on_parameter_changed(axis, 1.2)

    # Create → Sketch: the canvas opens, the tool comes from the
    # SELECTOR, the circle from the GESTURE (r = 0.35, OUTER).
    win.on_create_sketch()
    sid = max(win._sketch_views)
    canvas = win._sketch_views[sid]
    canvas.resize(800, 600)
    canvas.show()
    canvas._center = (0.0, 0.0)
    canvas._scale = 400.0 / 0.7               # 0.35 world = 200 px
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (600, 300))
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(0.35, abs=1e-6)
    assert win._scene.sketch_closed(sid)

    # Feature → Extrude (the menu path, dialog answered 3.0 / +Z).
    _select_tree_row(win, sid)
    monkeypatch.setattr(win, "_extrude_dialog", lambda node: (3.0, 1))
    win.on_extrude()
    prism_id = _find_operation(win, "extrude", sketch=sid)
    assert prism_id is not None
    assert _select_tree_row(win, prism_id).text(1) == "FEAT"

    # Edit → Subtract (the menu path): target the box, tool the
    # prism, the placement the API use case pins — two SDF rows
    # selected in the tree (unit 2's selection rule, ExtendedSelection
    # built row by row — no setCurrentItem, it clears the rest).
    win._feature_tree.clearSelection()
    _tree_row(win, box_id).setSelected(True)
    _tree_row(win, prism_id).setSelected(True)
    ids = win._selected_sdf_ids()
    assert ids == [box_id, prism_id]
    monkeypatch.setattr(
        win, "_subtract_dialog",
        lambda a, b: (box_id, prism_id, 0.3, 0.0, -1.5))
    win.on_subtract()
    hole_id = _find_operation(win, "subtract")
    assert hole_id is not None
    assert _select_tree_row(win, hole_id).text(1) == "CSG"

    # The hole is real, and Debug reports genus 1 (the same pure
    # string builder the Debug menu shows).
    body = win._scene.get_node(hole_id).body
    assert body.value(_math.Vec3f(0.3, 0.0, 0.0)) == pytest.approx(
        0.35, abs=1e-6)
    text = win._sdf_to_brep_report_text(win._scene.get_node(hole_id))
    assert "\u03c7 = V - E + F = 0" in text
    assert "Genus = 1" in text
