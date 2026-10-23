# bindings/python/tests/test_ui_property_commit.py
#
# The typed-parameter regression (D6, offscreen, the established
# driver pattern). The user's click-test crashed the window typing
# digits into the circle's radius field in the property panel — the
# faulthandler stack (Codespaces):
#
#     QLineEdit::text()                    <- freed object
#     QAbstractSpinBox::keyPressEvent()
#     QWidget::event -> processKeyEvent -> app.exec()
#
# Derivation: the editor factory built the numeric fields with the
# DEFAULT keyboardTracking (per-digit valueChanged), so every typed
# digit wrote the recipe, and the write's echo rebuilt the panel's
# rows from INSIDE the spinbox's own keyPressEvent —
# QFormLayout::removeRow deletes the old editors synchronously, and
# the stack returned into QAbstractSpinBox::keyPressEvent on a freed
# object. Secondary defect, same root: a half-typed "15" produced TWO
# writes and TWO undo entries (D5.2 wants one completed edit = one
# write = one undo entry). The string field had the per-keystroke
# twin (textChanged).
#
# Every test here drives the REAL surface (QTest.keyClicks on the
# actual editor, focus-out commits, the canvas's own edit route) and
# fails on the pre-fix code in the bug's own way — crash, double
# undo, or a destroyed editor — so the defect cannot be reborn:
#
#   THE KEY TEST   -> test_typed_radius_commits_once_without_crashing
#   in-place echo  -> test_recipe_echo_updates_editors_in_place
#   deferred rows  -> test_panel_rebuild_is_deferred_to_the_next_pass
#   string commit  -> test_string_field_commits_once_per_completed_edit
#   multi highlight-> test_viewport_highlights_every_selected_body
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

    # Modal boxes never block headless runs (the gestures smoke's
    # pattern): refusals and instructions are captured, not shown.
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
    """Create -> Sketch (the menu slot), the canvas pinned to the
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


def _circle_gesture(canvas, c, rim):
    """The circle gesture (D5.1's pinned press-drag-release)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))


def _panel_field(win, name):
    """The panel's editor widget for a parameter row, located through
    the FORM LAYOUT (label text -> field widget) — the lookup that
    also works on the pre-fix panel, so the very same test can
    express the bug before the fix and pin it after."""
    from PySide6.QtWidgets import QFormLayout
    form = win._property_panel._form
    for row in range(form.rowCount()):
        label_item = form.itemAt(row, QFormLayout.LabelRole)
        field_item = form.itemAt(row, QFormLayout.FieldRole)
        if label_item is None or field_item is None:
            continue
        label = label_item.widget()
        if label is not None and label.text() == name:
            return field_item.widget()
    return None


def _tree_ids(win):
    from PySide6.QtCore import Qt
    ids = []
    for i in range(win._feature_tree.topLevelItemCount()):
        ids.append(win._feature_tree.topLevelItem(i).data(0, Qt.UserRole))
    return ids


def _tree_row(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            return it
    return None


def _selected_circle(win):
    """The circle's recipe entity as the SCENE reads it — the value
    assertions go to the source of truth, never to a UI copy."""
    sid = max(win._sketch_views)
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and len(chains[0]) == 1
    return sid, chains[0][0]


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_typed_radius_commits_once_without_crashing(sketch_app):
    """THE KEY TEST — the user's exact gesture, digit by digit.

    Typing "15" into the radius field (inside the existing value, NO
    Enter) must (a) keep the window alive — the test completing is
    the no-crash assertion, the pre-fix code died inside the first
    keystroke with the panel's rows deleted under the spinbox's own
    keyPressEvent — and (b) commit exactly ONCE: nothing while the
    number is half-typed, then ONE write, ONE undo entry and the
    recipe's final value on the focus-out that completes the edit.
    """
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QDoubleSpinBox

    win = sketch_app
    sid, canvas = _open_canvas(win)
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))   # circle, r = 2.0
    canvas.select_entity(0, 0)   # the panel shows cx/cy/r/hole
    undo_before = win._command_stack.undo_count()

    field = _panel_field(win, "r")
    assert isinstance(field, QDoubleSpinBox)
    win.show()
    QApplication.instance().processEvents()
    field.setFocus()
    QApplication.instance().processEvents()
    assert field.hasFocus()

    # The digits — inside the existing value, NO Enter.
    field.selectAll()
    QTest.keyClicks(field, "15")

    # A half-typed number is not a modification: nothing written,
    # nothing undone, the recipe still carries the drawn radius.
    assert win._command_stack.undo_count() == undo_before
    _sid, circ = _selected_circle(win)
    assert circ["radius"] == pytest.approx(2.0, abs=1e-6)

    # Focus-out completes the edit: ONE write, ONE undo entry, the
    # final value in the recipe (asserted on the command stack AND on
    # the value read back from the scene).
    field.clearFocus()
    QApplication.instance().processEvents()
    assert win._command_stack.undo_count() == undo_before + 1
    assert "Set r = 15" in win._command_stack.next_undo_description()
    _sid, circ = _selected_circle(win)
    assert circ["radius"] == pytest.approx(15.0, abs=1e-6)

    # The undo restores the drawn radius — and the SAME editor, live,
    # refreshed in place by the echo (no row was ever rebuilt).
    win.on_undo()
    _sid, circ = _selected_circle(win)
    assert circ["radius"] == pytest.approx(2.0, abs=1e-6)
    assert _panel_field(win, "r") is field
    assert field.value() == pytest.approx(2.0, abs=1e-6)


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_recipe_echo_updates_editors_in_place(sketch_app):
    """A canvas-side edit of the selected entity (the route a
    released drag takes) refreshes the panel's editors IN PLACE: the
    widget objects survive the echo, and the echo writes nothing of
    its own (no second undo entry, no cascade)."""
    win = sketch_app
    sid, canvas = _open_canvas(win)
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))
    canvas.select_entity(0, 0)
    r_field = _panel_field(win, "r")
    cx_field = _panel_field(win, "cx")
    undo_before = win._command_stack.undo_count()

    canvas.edit_entity_params(0, 0, {"r": 7.5}, "gesture edit")

    # Same live editors — the rows were not rebuilt.
    assert _panel_field(win, "r") is r_field
    assert _panel_field(win, "cx") is cx_field
    # The value followed the recipe...
    assert r_field.value() == pytest.approx(7.5, abs=1e-6)
    _sid, circ = _selected_circle(win)
    assert circ["radius"] == pytest.approx(7.5, abs=1e-6)
    # ...and the echo added NO write of its own: the canvas edit is
    # the only command this produced.
    assert win._command_stack.undo_count() == undo_before + 1
    assert win._command_stack.next_undo_description() == "gesture edit"


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_panel_rebuild_is_deferred_to_the_next_pass(sketch_app):
    """When the panel is NOT showing the edited entity, the row
    rebuild waits for the next event-loop pass (QTimer.singleShot(0)):
    nothing is destroyed synchronously inside the write, and the
    deferred rebuild reads the CURRENT selection at fire time."""
    from PySide6.QtWidgets import QApplication

    win = sketch_app
    sid, canvas = _open_canvas(win)
    win._tool_actions["circle"].trigger()
    _circle_gesture(canvas, (400, 300), (520, 300))
    canvas.select_entity(0, 0)   # panel = circle[chain 0 #0]

    # The defense path this test engineers: whatever the panel is
    # showing when the echo lands, it is not the edited entity — the
    # rebuild must NOT happen synchronously inside the write.
    win._property_panel.set_parameters("stale surface", {"a": 1.0})
    canvas.edit_entity_params(0, 0, {"r": 4.5}, "edit")

    assert win._property_panel.current_label() == "stale surface", (
        "the rebuild must not run synchronously inside the write")

    # The next loop pass delivers it — from the CURRENT state.
    QApplication.instance().processEvents()
    assert win._property_panel.current_label() == "circle[chain 0 #0]"
    assert _panel_field(win, "r").value() == pytest.approx(4.5, abs=1e-6)


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_string_field_commits_once_per_completed_edit(sketch_app):
    """The string editor commits ONCE per completed edit: typing a
    string emits nothing until the edit is done (Return or
    focus-out), then exactly ONE parameter_changed with the final
    text — the per-keystroke twin of the numeric mine, closed by the
    same principle."""
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QLineEdit

    win = sketch_app
    panel = win._property_panel
    panel.set_parameters("string surface", {"label": "abc"})
    field = _panel_field(win, "label")
    assert isinstance(field, QLineEdit)

    emitted = []
    panel.parameter_changed.connect(
        lambda n, v: emitted.append((n, v)))

    win.show()
    QApplication.instance().processEvents()
    field.setFocus()
    QApplication.instance().processEvents()
    assert field.hasFocus()

    field.selectAll()
    QTest.keyClicks(field, "hello")   # five keystrokes, no Return
    assert emitted == [], (
        "a half-typed string is not a modification")

    field.clearFocus()
    assert emitted == [("label", "hello")], (
        "one completed edit = one commit, with the final text")


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_viewport_highlights_every_selected_body(sketch_app):
    """The extended selection reaches the WHOLE highlight: with two
    bodies selected (the subtract's target and tool), the viewport
    carries BOTH ids — the selection always worked, only the
    highlight used to ignore every row past the first."""
    from PySide6.QtWidgets import QApplication

    win = sketch_app
    win._create_registered("box")
    win._create_registered("cylinder")
    box_id, cyl_id = _tree_ids(win)[-2:]
    win.show()
    QApplication.instance().processEvents()

    row_box = _tree_row(win, box_id)
    row_cyl = _tree_row(win, cyl_id)

    # The user's Ctrl-click pair: a click on one row, then the
    # extended add on the other (setCurrentItem selects and clears;
    # setSelected adds without clearing — the established pattern).
    win._feature_tree.setCurrentItem(row_box)
    row_cyl.setSelected(True)

    # BOTH bodies are in the set the draw loop paints.
    assert win._viewport._selected_node_ids == {box_id, cyl_id}
    # The anchor surface the panel routes on is intact (items[0]).
    assert win._selected_node_id == box_id

    # A plain click collapses back to the single-selection shape
    # (the pinned picking contract, unchanged).
    win._feature_tree.setCurrentItem(row_cyl)
    assert win._viewport._selected_node_ids == {cyl_id}
    assert win._viewport._selected_node_id == cyl_id
    assert win._selected_node_id == cyl_id
