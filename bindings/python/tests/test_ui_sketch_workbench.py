# bindings/python/tests/test_ui_sketch_workbench.py
#
# G4 m3 — LA WORKBENCH (ADR-0040 D4): the UI smoke suite extended, the
# five surfaces each mapped to a test driven through the WIRING (the
# real MainWindow + the real SketchCanvas, injected QTest events and
# the menu actions a user's hand fires — nothing called behind the
# widget's back, the established D6 driver):
#
#   1. THE HONEST COUNTER   -> test_the_honest_counter_crossed_with_the_solver
#      the bill on the status line IS the solver's own count — the
#      test crosses the two voices (the UI never says a number the
#      graph does not confirm), at every cold site;
#   2. THE PIN              -> test_three_pins_on_the_macro_rectangle
#      the macro's rectangle, then three pins: 16-13=3 -> 16-16=0
#      FULL, the count returns, the solve green — the pins consume
#      the values the parameters had;
#   3. THE CLASSES ON SCREEN -> test_the_classes_on_screen_not_solvable_and_over_constrained
#      the 4/5/10 triangle built LIVE -> sketch_not_solvable with the
#      residual, the culprit and the diagnosis visible; the redundant
#      parallel+perpendicular pair (the m2's own case, now as a user
#      gesture) -> sketch_over_constrained with the culprit;
#   4. THE SUGGESTIONS      -> test_the_suggestions_stay_value_offers
#      unchanged: a gesture near a suggestion NEVER writes a
#      constraint — with or without the SHIFT acceptance, the
#      value-offer writes VALUES only (the axes family's promise);
#   5. THE DETERMINISM      -> test_the_same_gesture_twice_bit_identical
#      the same macro twice: the same bill, the same solve, the
#      chains value-identical — and the undo/redo round trip too;
#   6. THE DOWNSTREAM EDIT  -> test_the_golden_8_live_the_edit_through_the_solve
#      the solved triangle extruded, the dimension edited 5 -> 5.5
#      through the panel's own row: the solve runs, the recipe's
#      fields update, the DAG re-materializes the prism, the names
#      re-derive IDENTICALLY (the resolver's class 1, silent), ONE
#      undo entry, N rebuilds bit-identical — the golden 8 as a
#      gesture.
from __future__ import annotations

import gc
import math

import pytest


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.fixture()
def workbench_app(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

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


# --- The deterministic driver (the gestures smoke's own discipline) ------

def _open_canvas(win):
    """Create -> Sketch, the canvas pinned: centre on the origin,
    60 px per world unit (the round trips of INTEGER world points are
    then exact — the anchored chaining lands verbatim)."""
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


def _to_screen(canvas, wx, wy):
    return canvas.world_to_screen(wx, wy)


def _draw_segment(canvas, p0, p1, shift=False, tool="segment"):
    """An injected press-drag-release at WORLD points (D5.1's pinned
    gesture; the SHIFT at the release accepts the candidate)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    canvas.set_tool(tool)
    s0 = _to_screen(canvas, *p0)
    s1 = _to_screen(canvas, *p1)
    mods = Qt.ShiftModifier if shift else Qt.NoModifier
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier,
                     QPoint(int(round(s0[0])), int(round(s0[1]))))
    QTest.mouseMove(canvas, QPoint(int(round(s1[0])), int(round(s1[1]))))
    QTest.mouseRelease(canvas, Qt.LeftButton, mods,
                       QPoint(int(round(s1[0])), int(round(s1[1]))))


def _click_world(canvas, world, shift=False):
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    s = _to_screen(canvas, *world)
    mods = Qt.ShiftModifier if shift else Qt.NoModifier
    QTest.mouseClick(canvas, Qt.LeftButton, mods,
                     QPoint(int(round(s[0])), int(round(s[1]))))


def _select_entity(canvas, world):
    """A select-tool click on the entity nearest the world point."""
    canvas.set_tool("select")
    _click_world(canvas, world)


def _pin(win, canvas, field):
    """The pin through the WIRING: the Constraints -> Pin menu's own
    action for the selected entity's field (the menu a user opens)."""
    win._rebuild_pin_menu()
    actions = [a for a in win._pin_menu.actions() if a.text() == field]
    assert actions, ("the pin menu carries the field {} — the selected "
                     "entity's own".format(field))
    actions[0].trigger()


def _armed_two_clicks(win, canvas, kind, first, second):
    """The two-step constraint pick through the WIRING: the menu
    action arms it, the two clicks land the operands."""
    win._constraint_actions[kind].trigger()
    assert canvas._armed is not None
    _click_world(canvas, first)
    _click_world(canvas, second)
    assert canvas._armed is None  # the landing clears the arming


def _panel_field(win, name):
    """The panel's editor for a row, located through the form layout
    (the property-commit smoke's own lookup)."""
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


def _panel_rows(win):
    """(label, value-text) for every row the panel is showing — the
    editors and the readonly rows alike, the VALUE as the panel
    renders it."""
    from PySide6.QtWidgets import QFormLayout
    form = win._property_panel._form
    out = []
    for row in range(form.rowCount()):
        label_item = form.itemAt(row, QFormLayout.LabelRole)
        field_item = form.itemAt(row, QFormLayout.FieldRole)
        if label_item is None or field_item is None:
            continue
        label = label_item.widget()
        field = field_item.widget()
        if label is None or field is None:
            continue
        text = field.text() if hasattr(field, "text") else str(field)
        out.append((label.text(), text))
    return out


def _seg(win, sid, chain, index):
    return win._scene.sketch_chains(sid)[chain][index]


def _seg_len(e):
    a, b = e["a"], e["b"]
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _digest_names(body):
    import CAD_0
    brep = CAD_0.brep
    return [brep.face_query(body, i)["name"]
            for i in range(brep.face_count(body))]


# ===========================================================================
# 1. THE HONEST COUNTER — the UI's number crossed with the graph's own
# ===========================================================================

def test_the_honest_counter_crossed_with_the_solver(workbench_app):
    """LA BOLLETTA on the canvas's status line is the bill's OWN text
    (describe()), recomputed at the cold sites only — and the test
    crosses the two voices at every step: an INDEPENDENT model built
    from the scene's chains and the session's constraints must answer
    the same count. The UI never says a number the graph does not
    confirm."""
    from CAD_0.ui.sketch_workbench import SketchSolverSession

    win = workbench_app
    sid, canvas = _open_canvas(win)

    # The empty sketch: 12 free is a lie of nothing — nothing drawn,
    # the bill reports the void honestly.
    assert canvas.bill_text() == "0 - 0 = 0 free"

    # A drawn segment: the counter moves at the cold site (the write
    # route's _after_write is the only place it moves).
    _draw_segment(canvas, (0.0, 0.0), (4.0, 0.0))
    assert canvas.bill_text() == "4 - 0 = 4 free"
    # THE CROSS, voice one: an independent model, built from the
    # recipe and NO constraints (the graph's own count).
    session = win._sketch_sessions[sid]
    assert SketchSolverSession(win._scene, sid).bill_text() == \
        canvas.bill_text()

    # The undo of the creation is a cold site too: the counter
    # descends with the recipe, the same voice.
    win.on_undo()
    assert canvas.bill_text() == "0 - 0 = 0 free"

    # The honest ladder is the macro: 16 - 13 = 3, the rigid motion's
    # own count, REPORTED.
    win._tool_actions["rectangle"].trigger()
    assert canvas.tool() == "rectangle"
    _draw_segment(canvas, (1.0, 1.0), (5.0, 4.0), tool="rectangle")
    assert canvas.bill_text() == "16 - 13 = 3 free"
    # THE CROSS, voice two: the independent model with the session's
    # constraints re-admitted.
    fresh = SketchSolveModel_of(win, sid, session)
    assert fresh.bill().describe() == canvas.bill_text()

    # A pin (the cold site): 16 - 14 = 2.
    _select_entity(canvas, (2.5, 1.0))   # the bottom side of the macro
    assert canvas.selected_entity() == (0, 0)   # the macro reused the
    # emptied trailing chain (the _create_entity pattern)
    _pin(win, canvas, "x0")
    assert canvas.bill_text() == "16 - 14 = 2 free"

    # A parameter write (the cold site): the DoF do not move but the
    # counter refreshes from the graph — the same count, the cross
    # again after the write.
    undo_before = win._command_stack.undo_count()
    _panel_field(win, "x1").setValue(5.5)
    assert win._command_stack.undo_count() == undo_before + 1
    assert canvas.bill_text() == "16 - 14 = 2 free"
    fresh = SketchSolveModel_of(win, sid, session)
    assert fresh.bill().describe() == canvas.bill_text()

    # THE FULL state, named in the bill's own words:
    for field in ("y0", "y1"):
        _pin(win, canvas, field)
    assert canvas.bill_text() == "16 - 16 = 0 (FULL)"

    # The independent graph confirms the FULL count — the two voices
    # agree at every cold site of the whole climb.
    fresh = SketchSolveModel_of(win, sid, session)
    assert fresh.bill().describe() == "16 - 16 = 0 (FULL)"
    assert fresh.bill().full() is True


def SketchSolveModel_of(win, sid, session):
    """The independent voice: a model built from the SCENE's chains
    with the session's constraints re-admitted — the graph the
    counter's number must confirm."""
    from CAD_0.ui.sketch_workbench import native_chains
    import CAD_0
    model = CAD_0.scene.SketchSolveModel(
        native_chains(win._scene, sid))
    for c in session.constraints():
        record = model.add_constraint(c)
        assert record.admitted, record.message
    return model


# ===========================================================================
# 2. THE PIN — three pins on the macro's rectangle, 16-16=0, solve green
# ===========================================================================

def test_three_pins_on_the_macro_rectangle(workbench_app):
    """The macro's ONE act (4 entities + 13 constraints, ONE undo
    entry, the values visible), then three pins: the bill 16-13=3
    right after the gesture, 16-16=0 FULL after the pins — the count
    returns — and the solve green. The pins consume the values the
    parameters had at the moment of the gesture."""
    win = workbench_app
    sid, canvas = _open_canvas(win)
    undo_before = win._command_stack.undo_count()

    # IL MACRO RETTANGOLO: press at a corner, release at the opposite
    # one — 4 entities + the constraint set in ONE act (D1's own
    # table: 4 coincidences + 2 parallels + 1 perpendicular +
    # 2 lengths = 9 records CONSUMING 13 — the bill's subtrahend,
    # the ADR's own "13").
    win._tool_actions["rectangle"].trigger()
    _draw_segment(canvas, (1.0, 1.0), (5.0, 4.0), tool="rectangle")

    # ONE undo entry for the whole act.
    assert win._command_stack.undo_count() == undo_before + 1
    # 4 entities, one closed chain (the composition's own contour).
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and len(chains[0]) == 4
    assert win._scene.sketch_closed(sid)
    assert all(e["kind"] == "segment" for e in chains[0])
    # The constraint set, counted on the session's own list: 9
    # records (4 coincidences + 2 parallels + 1 perpendicular +
    # 2 lengths) CONSUMING 13 — the derivation's own numbers.
    session = win._sketch_sessions[sid]
    assert session.constraint_count() == 9
    kinds = [str(c.kind) for c in session.constraints()]
    assert kinds.count("coincidence") == 4
    assert kinds.count("parallel") == 2
    assert kinds.count("perpendicular") == 1
    assert kinds.count("length") == 2
    # THE BILL right after the gesture: 16 - 13 = 3 (the rigid
    # motion: translation 2 + rotation 1), reported, never hidden.
    assert canvas.bill_text() == "16 - 13 = 3 free"
    # The values visible (D5.1): the length labels render the macro's
    # own dimensions w=4, h=3.
    labels = canvas.dimension_labels()
    assert any(text.startswith("length = 4") for text in labels)
    assert any(text.startswith("length = 3") for text in labels)

    # TRE PIN: x0, y0 (the corner A) and y1 (B on the same horizontal)
    # — the pins consume the CURRENT values (the recipe's own floats).
    _select_entity(canvas, (2.5, 1.0))     # the bottom side
    assert canvas.selected_entity() == (0, 0)
    e = _seg(win, sid, 0, 0)
    pins = []
    for field in ("x0", "y0", "y1"):
        expected = {"x0": e["a"][0], "y0": e["a"][1],
                    "y1": e["b"][1]}[field]
        pins.append((field, expected))
        _pin(win, canvas, field)
    # each pin is ONE undo entry, the count climbs one at a time
    assert win._command_stack.undo_count() == undo_before + 4
    # the pins' values are the parameters' own at gesture time
    renders = session.describe_all()
    for field, expected in pins:
        needle = "the pin on s0.{} = {}".format(
            field, _fmt(expected))
        assert any(r.startswith(needle) for r in renders), \
            "the pin consumed the parameter's own value: " + needle

    # LA BOLLETTA TORNA: 16 - 16 = 0 FULL.
    assert canvas.bill_text() == "16 - 16 = 0 (FULL)"

    # THE SOLVE GREEN: the explicit act (the menu's own action), the
    # words in the register's vocabulary with the free count.
    win._solve_action.trigger()
    words = canvas.feedback_text()
    assert words == "solved — 16 - 16 = 0 (FULL)"
    # The geometry already holds every constraint (the macro wrote the
    # values at the gesture's own corners): nothing moved, no command.
    assert win._command_stack.undo_count() == undo_before + 4

    # The counter and the words are THE PANEL'S ROWS too (the registry
    # consumer — zero new panel code): reselect the entity, the
    # surface routes with the Solver row.
    _select_entity(canvas, (2.5, 1.0))
    rows = dict(_panel_rows(win))
    assert any(label.startswith("the pin on s0.") for label in rows)
    assert any(label.startswith("the length |s0|") for label in rows)
    # the words on the panel: the class in the register's vocabulary
    from PySide6.QtWidgets import QLabel
    solver_labels = [w for w in win._property_panel.findChildren(QLabel)
                     if w.text() == "Solver"]
    assert solver_labels  # the row exists
    # its VALUE is read from the trailing readonly row (the last row)
    form_values = [text for text, _w in _panel_rows(win)]
    assert "Solver" in form_values
    # the sketch node's own surface carries the bill and the words
    _click_tree_row(win, sid)
    node_rows = dict(_panel_rows(win))
    assert node_rows.get("DoF") == "16 - 16 = 0 (FULL)"
    assert node_rows.get("Solver") == "solved — 16 - 16 = 0 (FULL)"

    # THE UNDO LADDER: the pins pop (the count descends), the macro's
    # one entry removes entities AND constraints together.
    for _ in range(3):
        win.on_undo()
    assert canvas.bill_text() == "16 - 13 = 3 free"
    assert session.constraint_count() == 9
    win.on_undo()   # the macro
    assert session.constraint_count() == 0
    chains = win._scene.sketch_chains(sid)
    assert sum(len(c) for c in chains) == 0
    assert canvas.bill_text() == "0 - 0 = 0 free"


def _fmt(v):
    if v == int(v):
        return str(int(v))
    return "{:.6g}".format(v)


def _click_tree_row(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        item = win._feature_tree.topLevelItem(i)
        if item.data(0, Qt.UserRole) == node_id:
            win._feature_tree.setCurrentItem(item)
            return item
    return None


# ===========================================================================
# 3. THE CLASSES ON SCREEN — not_solvable with the diagnosis,
#    over_constrained with the culprit (the m2's pair, as a gesture)
# ===========================================================================

def test_the_classes_on_screen_not_solvable_and_over_constrained(workbench_app):
    """The 4/5/10 triangle built LIVE (the gestures, the armed picks,
    the pins, the dimension edit): the solve answers
    sketch_not_solvable with the residual, the named culprit and the
    diagnosis visible in the interface's words. The redundant
    parallel+perpendicular pair — the m2's own dependent-angular case,
    now as a USER GESTURE — refuses at the admission with the culprit:
    sketch_over_constrained, never a silence."""
    win = workbench_app
    sid, canvas = _open_canvas(win)

    # THE TRIANGLE, DAL VIVO: three segments drawn as one chain (the
    # integer anchors chain verbatim — the exact-formula round trips).
    _draw_segment(canvas, (0.0, 0.0), (4.0, 0.0))     # s0: A -> B
    _draw_segment(canvas, (4.0, 0.0), (0.0, 3.0))     # s1: B -> C
    _draw_segment(canvas, (0.0, 3.0), (0.0, 0.0))     # s2: C -> A
    assert win._scene.sketch_closed(sid)

    # The three coincidences (the chain's joints) — the armed pick.
    _armed_two_clicks(win, canvas, "coincidence",
                      (4.0, 0.0), (4.0, 0.0))   # s0.end ~ s1.start
    _armed_two_clicks(win, canvas, "coincidence",
                      (0.0, 3.0), (0.0, 3.0))   # s1.end ~ s2.start
    _armed_two_clicks(win, canvas, "coincidence",
                      (0.0, 0.0), (0.0, 0.0))   # s2.end ~ s0.start
    session = win._sketch_sessions[sid]
    assert session.constraint_count() == 3
    assert canvas.bill_text() == "12 - 6 = 6 free"

    # The three pins (x_A, y_A, y_B — at the drawn values, exactly 0).
    _select_entity(canvas, (2.0, 0.0))
    assert canvas.selected_entity() == (0, 0)
    for field in ("x0", "y0", "y1"):
        _pin(win, canvas, field)
    assert canvas.bill_text() == "12 - 9 = 3 free"

    # The three lengths at the current measures (4, 5, 3 exact — the
    # integer vertices' own hypotenuses).
    for pick, expect in (((2.0, 0.0), 4.0), ((2.0, 1.5), 5.0),
                         ((0.0, 1.5), 3.0)):
        _select_entity(canvas, pick)
        chain, index = canvas.selected_entity()
        assert _seg_len(_seg(win, sid, chain, index)) == \
            pytest.approx(expect, abs=1e-9)
        win._constraint_actions["length"].trigger()
    # 9 records (3 coincidences + 3 lengths + 3 pins) CONSUMING 12 —
    # the golden 1's own arithmetic (coincidences 6 + lengths 3 +
    # pins 3).
    assert session.constraint_count() == 9
    assert canvas.bill_text() == "12 - 12 = 0 (FULL)"

    # The solve is green on the 3-4-5 (the drawn geometry already
    # holds everything — the integer landing).
    win._solve_action.trigger()
    assert canvas.feedback_text() == "solved — 12 - 12 = 0 (FULL)"

    # THE EDIT TO 10 — the dimension's row on the panel, the registry
    # route: the re-value, the solve, the honest failure.
    _select_entity(canvas, (0.0, 1.5))              # s2 (|CA| = 3)
    row = _panel_field(win, "the length |s2| = 3")
    assert row is not None
    row.setValue(10.0)

    # THE CLASS, IN WORDS: sketch_not_solvable with the residual, the
    # named culprit and the diagnosis — the register's vocabulary.
    words = canvas.feedback_text()
    assert words.startswith("sketch_not_solvable — ")
    assert "the largest violation 0.166667" in words
    assert "the coincidence on (s0.end = s1.start), row 1" in words
    assert "|F| = 0.408248" in words
    assert "4+5 < 10 — no triangle with these sides exists" in words
    assert "the recipe untouched" in words
    # THE WORDS ON THE PANEL (the registry consumer): the entity's
    # surface carries the Solver row after the next routing.
    _select_entity(canvas, (0.0, 1.5))
    from PySide6.QtWidgets import QLabel
    texts = [w.text() for w in win._property_panel.findChildren(QLabel)]
    assert "Solver" in texts
    # The recipe untouched by the failed solve: the lengths stand.
    assert _seg_len(_seg(win, sid, 0, 0)) == pytest.approx(4.0, abs=1e-9)
    assert _seg_len(_seg(win, sid, 0, 1)) == pytest.approx(5.0, abs=1e-9)
    # The constraint's VALUE moved (the edit is authored data): 10.
    renders = session.describe_all()
    assert any(r == "the length |s2| = 10" for r in renders)

    # THE REDUNDANT PAIR, as a user gesture: the m2's own dependent
    # angular case (one relative angle, one DoF) on a fresh sketch.
    sid2, canvas2 = _open_canvas(win)
    _draw_segment(canvas2, (0.0, 0.0), (4.0, 0.3))
    _draw_segment(canvas2, (0.5, 0.2), (3.8, 0.9))
    assert canvas2.bill_text() == "8 - 0 = 8 free"
    # the parallel lands: 8 - 1 = 7
    _armed_two_clicks(win, canvas2, "parallel",
                      (2.0, 0.15), (2.0, 0.55))
    session2 = win._sketch_sessions[sid2]
    assert session2.constraint_count() == 1
    assert canvas2.bill_text() == "8 - 1 = 7 free"
    # the perpendicular on the SAME pair: REFUSED at the admission,
    # the culprit named — the class in the interface's words. (The
    # two drawn segments live in two chains: the render carries the
    # chain prefix — the multi-chain form, the naming grammar's own.)
    _armed_two_clicks(win, canvas2, "perpendicular",
                      (2.0, 0.15), (2.0, 0.55))
    words2 = canvas2.feedback_text()
    assert words2.startswith("sketch_over_constrained: ")
    assert "the perpendicular on (c0 s0 ⊥ c1 s0)" in words2
    assert "REDUNDANT" in words2
    assert "8 - 1 = 7 free" in words2
    assert "Remedy" in words2
    # The refused candidate NEVER entered the list (the
    # over-constrained never reaches the solver).
    assert session2.constraint_count() == 1
    # and the panel carries the refusal after the routing
    _select_entity(canvas2, (2.0, 0.15))
    from PySide6.QtWidgets import QLabel as _QL
    texts2 = [w.text() for w in
              win._property_panel.findChildren(_QL)]
    assert "Solver" in texts2


# ===========================================================================
# 4. THE SUGGESTIONS UNCHANGED — value-offers forever
# ===========================================================================

def test_the_suggestions_stay_value_offers(workbench_app):
    """A gesture NEAR a suggestion (the near-horizontal release,
    inside the 6 px H/V band) writes the pointer's raw values
    verbatim — and the accepted one (SHIFT) writes the offered VALUE.
    Neither EVER writes a constraint: the value-offers stay
    value-offers until the axes family lands (ADR-0040 D1's promise,
    pinned here). The test pins it on the graph: the constraint list
    stays EMPTY."""
    win = workbench_app
    sid, canvas = _open_canvas(win)

    # NEAR the horizontal suggestion, no SHIFT: the raw values land
    # (y0 != y1 — the offer was never taken, let alone constrained).
    # The release at world y = 0.05 is 3 px above the anchor — inside
    # the 6 px H/V band, and its screen position quantizes exactly.
    _draw_segment(canvas, (0.0, 0.0), (4.0, 0.05), shift=False)
    session = win._sketch_sessions[sid]
    assert session.constraint_count() == 0        # NOTHING in the graph
    e = _seg(win, sid, 0, 0)
    assert e["a"][1] == pytest.approx(0.0, abs=1e-9)
    assert e["b"][1] == pytest.approx(0.05, abs=1e-6)
    assert e["a"][1] != e["b"][1]
    assert canvas.bill_text() == "4 - 0 = 4 free"

    # The SECOND segment, WITH the SHIFT acceptance: the offered
    # VALUE is written (y exactly the anchor's) — a value, NOT a
    # relation. The graph still carries no constraint.
    _draw_segment(canvas, (0.0, 1.0), (4.0, 1.03), shift=True)
    assert session.constraint_count() == 0        # still NOTHING
    e2 = _seg(win, sid, 1, 0)
    # the accepted H offer wrote the anchor's y verbatim
    assert e2["a"][1] == pytest.approx(1.0, abs=1e-9)
    assert e2["b"][1] == pytest.approx(1.0, abs=1e-9)
    assert canvas.bill_text() == "8 - 0 = 8 free"

    # And the graph's own voice: an independent model confirms —
    # zero constraints, the raw DoF, the values as drawn.
    from CAD_0.ui.sketch_workbench import native_chains
    import CAD_0
    model = CAD_0.scene.SketchSolveModel(native_chains(win._scene, sid))
    assert model.bill().describe() == "8 - 0 = 8 free"
    r = model.solve()
    assert str(r.status).find("solved") >= 0
    assert r.bill.free() == 8


# ===========================================================================
# 5. THE DETERMINISM OF THE ACT — the same gesture twice, bit-identical
# ===========================================================================

def test_the_same_gesture_twice_bit_identical(workbench_app):
    """The same macro gesture on two sketches: the same bill, the
    same solve words, the chains VALUE-IDENTICAL (field for field —
    the m2's own comparator discipline: never raw bytes, the padding
    lesson). The undo/redo round trip re-executes from the restored
    state: bit-identical again."""
    from CAD_0.ui.sketch_canvas import _chains_value_identical

    win = workbench_app
    sid1, canvas1 = _open_canvas(win)
    win._tool_actions["rectangle"].trigger()
    _draw_segment(canvas1, (1.0, 1.0), (5.0, 4.0), tool="rectangle")
    bill1 = canvas1.bill_text()
    assert bill1 == "16 - 13 = 3 free"
    words1 = None
    win._solve_action.trigger()
    words1 = canvas1.feedback_text()
    assert words1 == "solved — 16 - 13 = 3 free"
    dump1 = win._scene.sketch_chains(sid1)

    # THE SAME GESTURE, again: a fresh sketch, the same world points.
    sid2, canvas2 = _open_canvas(win)
    win._tool_actions["rectangle"].trigger()
    _draw_segment(canvas2, (1.0, 1.0), (5.0, 4.0), tool="rectangle")
    assert canvas2.bill_text() == bill1
    win._solve_action.trigger()
    assert canvas2.feedback_text() == words1
    dump2 = win._scene.sketch_chains(sid2)
    assert _chains_value_identical(dump1, dump2)

    # THE UNDO/REDO ROUND TRIP: the macro's one entry undone (the
    # entities AND the constraints together), redone from the restored
    # state — bit-identical to the first execution.
    undo_before = win._command_stack.undo_count()
    win.on_undo()
    session2 = win._sketch_sessions[sid2]
    assert session2.constraint_count() == 0
    assert sum(len(c) for c in win._scene.sketch_chains(sid2)) == 0
    win.on_redo()
    assert session2.constraint_count() == 9
    assert canvas2.bill_text() == bill1
    dump2_redo = win._scene.sketch_chains(sid2)
    assert _chains_value_identical(dump1, dump2_redo)
    assert win._command_stack.undo_count() == undo_before


# ===========================================================================
# 6. THE DOWNSTREAM EDIT — the golden 8 live: the edit through the
#    solve, the regeneration, the names identical (class 1, silent)
# ===========================================================================

def test_the_golden_8_live_the_edit_through_the_solve(workbench_app):
    """IL REVISIONATORE PARAMETRICO DAL VIVO: the triangle solved
    (12-12=0), extruded (the prism 6/9/5 with the golden's own
    names), THEN the dimension |BC| 5 -> 5.5 through the panel's own
    constraint row: the solve runs (the golden's closed form), the
    recipe's fields update, the DAG pass re-materializes the prism,
    the names re-derive IDENTICALLY (the resolver's class 1, silent —
    the reference answers by name before and after), ONE undo entry,
    the round trip bit-identical."""
    from CAD_0.ui.sketch_canvas import _chains_value_identical

    win = workbench_app
    sid, canvas = _open_canvas(win)

    # The solved triangle (test 3's ladder, compressed): drawn,
    # coincided, pinned, lengthed — 12 - 12 = 0 FULL.
    _draw_segment(canvas, (0.0, 0.0), (4.0, 0.0))     # s0: A -> B
    _draw_segment(canvas, (4.0, 0.0), (0.0, 3.0))     # s1: B -> C
    _draw_segment(canvas, (0.0, 3.0), (0.0, 0.0))     # s2: C -> A
    _armed_two_clicks(win, canvas, "coincidence", (4.0, 0.0), (4.0, 0.0))
    _armed_two_clicks(win, canvas, "coincidence", (0.0, 3.0), (0.0, 3.0))
    _armed_two_clicks(win, canvas, "coincidence", (0.0, 0.0), (0.0, 0.0))
    _select_entity(canvas, (2.0, 0.0))
    for field in ("x0", "y0", "y1"):
        _pin(win, canvas, field)
    for pick in ((2.0, 0.0), (2.0, 1.5), (0.0, 1.5)):
        _select_entity(canvas, pick)
        win._constraint_actions["length"].trigger()
    session = win._sketch_sessions[sid]
    assert canvas.bill_text() == "12 - 12 = 0 (FULL)"

    # THE EXTRUDE (distance 2, +Z): the prism with the golden's own
    # names — the roots unmoved, the DAG's first materialization.
    prism_id = win._do_extrude(sid, 2.0, 1)
    assert prism_id is not None
    import CAD_0
    brep = CAD_0.brep
    node = win._scene.get_node(prism_id)
    body = node.exact_brep
    digest_before = _digest_names(body)
    assert digest_before == [
        "ext#{}:lat(0)".format(prism_id),
        "ext#{}:lat(1)".format(prism_id),
        "ext#{}:lat(2)".format(prism_id),
        "ext#{}:cap(lo)".format(prism_id),
        "ext#{}:cap(hi)".format(prism_id),
    ]

    # THE EDIT — the dimension's row on the panel (the registry
    # consumer route): |BC| 5 -> 5.5, the solve, the store, the DAG.
    _select_entity(canvas, (2.0, 1.5))              # s1: B -> C
    row = _panel_field(win, "the length |s1| = 5")
    assert row is not None
    undo_before = win._command_stack.undo_count()
    row.setValue(5.5)
    # ONE undo entry for the whole act (the re-value, the solve, the
    # store-back, the DAG pass).
    assert win._command_stack.undo_count() == undo_before + 1

    # The golden's closed form: the new C at x = (9+16-30.25)/8,
    # y = sqrt(9 - x^2) — the branch of the initial point (y > 0).
    xc = (3.0 * 3.0 + 4.0 * 4.0 - 5.5 * 5.5) / (2.0 * 4.0)
    yc = math.sqrt(9.0 - xc * xc)
    assert xc == pytest.approx(-0.65625, abs=1e-12)
    s1 = _seg(win, sid, 0, 1)
    assert s1["b"][0] == pytest.approx(xc, abs=1e-6)   # the store's floor
    assert s1["b"][1] == pytest.approx(yc, abs=1e-6)
    # the constraints hold: |AB| = 4, |BC| = 5.5, |CA| = 3 — at the
    # store's own resolution (the solved values pass the float32
    # quantizer; the bands hold with their declared margins)
    assert _seg_len(_seg(win, sid, 0, 0)) == pytest.approx(4.0, abs=1e-6)
    assert _seg_len(_seg(win, sid, 0, 1)) == pytest.approx(5.5, abs=1e-6)
    assert _seg_len(_seg(win, sid, 0, 2)) == pytest.approx(3.0, abs=1e-6)
    # the solved words
    assert canvas.feedback_text() == "solved — 12 - 12 = 0 (FULL)"

    # THE NAMES RE-DERIVED IDENTICALLY (the class-1 leg: the value
    # level edit moves no root, the reference answers by name) and
    # the regeneration WITHOUT reselection — the same digest the
    # golden 8 compares, byte for byte, and the by-index query still
    # answering the reference's own name.
    node_after = win._scene.get_node(prism_id)
    digest_after = _digest_names(node_after.exact_brep)
    assert digest_after == digest_before
    assert brep.face_query(node_after.exact_brep, 1)["name"] == \
        "ext#{}:lat(1)".format(prism_id)

    # THE UNDO: ONE entry restores the WHOLE pre-act recipe (the
    # solve had moved two entities — s1.b and s2.a — not just the
    # edited dimension's segment).
    win.on_undo()
    s1 = _seg(win, sid, 0, 1)
    assert s1["b"][0] == pytest.approx(0.0, abs=1e-9)
    assert s1["b"][1] == pytest.approx(3.0, abs=1e-9)
    assert _seg_len(_seg(win, sid, 0, 1)) == pytest.approx(5.0, abs=1e-9)
    # THE REDO — the whole act re-executed from the restored state:
    # deterministic, bit-identical (ADR-0014 on the solve).
    win.on_redo()
    s1 = _seg(win, sid, 0, 1)
    assert s1["b"][0] == pytest.approx(xc, abs=1e-6)
    assert s1["b"][1] == pytest.approx(yc, abs=1e-6)
    dump_redo = win._scene.sketch_chains(sid)

    # N REBUILDS BIT-IDENTICAL: the whole act replayed on a fresh
    # sketch reproduces the same chains, value for value.
    for _ in range(2):
        win.on_undo()
        win.on_redo()
    assert _chains_value_identical(win._scene.sketch_chains(sid),
                                   dump_redo)
    assert canvas.bill_text() == "12 - 12 = 0 (FULL)"


# ===========================================================================
# 7. THE PERSISTENCE (G4 m4, ADR-0040 D3.1) — the constraints ride the
#    PAYLOAD: the session's list lands in the NODE, the file carries it
#    as named fields (v0.5), a loaded session HYDRATES it back — the
#    m3 "closing the project drops them" declaration retired by the
#    very act that lands it.
# ===========================================================================

def test_the_constraints_ride_the_payload_and_survive_the_file(
        workbench_app, tmp_path):
    """THE WHOLE CHAIN, through the wiring: the macro's nine records
    are the NODE's payload (flushed inside the command), the file
    saves them as v0.5 named fields, a fresh document loads them back
    field for field, a fresh session hydrates — the same bill, the
    same renders — and the re-save is byte-identical."""
    import CAD_0

    win = workbench_app
    sid, canvas = _open_canvas(win)
    win._tool_actions["rectangle"].trigger()
    _draw_segment(canvas, (1.0, 1.0), (5.0, 4.0), tool="rectangle")
    assert canvas.bill_text() == "16 - 13 = 3 free"

    # THE PAYLOAD: the session's authored list IS the node's own data
    # (nine records — the macro's writes, flushed inside the command;
    # 4 coincidences + 2 parallels + 1 perpendicular + 2 lengths,
    # D1's own table).
    payload = win._scene.sketch_constraints(sid)
    assert len(payload) == 9
    kinds = [str(c.kind) for c in payload]
    assert kinds.count("coincidence") == 4
    assert kinds.count("parallel") == 2
    assert kinds.count("perpendicular") == 1
    assert kinds.count("length") == 2

    # THE FILE (v0.5): the constraints ride named fields beside the
    # chains — the bytes carry the vocabulary the kernel owns.
    path = tmp_path / "constraints_v0_5.CAD_0"
    CAD_0.io.save(path, win._scene)
    text = path.read_text(errors="replace")
    assert "\"version\": \"0.5.0\"" in text
    assert "\"constraints\"" in text
    assert "\"kind\": \"coincidence\"" in text
    assert "\"kind\": \"length\"" in text
    assert "\"end\": \"start\"" in text

    # A FRESH DOCUMENT: load → the payload back, field for field (the
    # load never solves — the recipe is the truth; the bill at rest is
    # the next cold act's own answer).
    scene2 = CAD_0.scene.Scene()
    cam = CAD_0.io.load(path, scene2)
    loaded = scene2.sketch_constraints(1)
    assert len(loaded) == 9
    assert [str(c.kind) for c in loaded] == kinds
    # The values back: the gesture's w = 4, h = 3.
    lengths = sorted(c.value for c in loaded if str(c.kind) == "length")
    assert lengths == pytest.approx([3.0, 4.0])
    # The operands back as canonical indices: the macro's first
    # coincidence binds s0.end to s1.start (chain 0, entities 0/1).
    first = loaded[0]
    assert first.a.entity.chain == 0 and first.a.entity.entity == 0
    assert first.a.end is True
    assert first.b.entity.chain == 0 and first.b.entity.entity == 1

    # A SESSION HYDRATES: the m3 "closing the project drops them"
    # declaration retired — the loaded list is the editing surface's
    # own state, and the bill reads exactly the same.
    from CAD_0.ui.sketch_workbench import SketchSolverSession
    session2 = SketchSolverSession(scene2, 1)
    assert session2.constraint_count() == 9
    assert session2.bill_text() == canvas.bill_text()
    assert session2.describe_all() == win._sketch_sessions[sid].describe_all()

    # THE RE-SAVE: byte-identical to the first save (the round-trip
    # loses nothing — the digest proof, at the Python door too).
    path2 = tmp_path / "constraints_v0_5_rt.CAD_0"
    CAD_0.io.save(path2, scene2, cam)
    assert path2.read_bytes() == path.read_bytes()
