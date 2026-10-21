# bindings/python/tests/test_ui_sketch_gestures.py
#
# ADR-0030 D6 — the UI smoke (offscreen, the established driver):
# the GESTURE SEMANTICS of the D5 binding specification, each clause
# mapped to a test. The real MainWindow + the real SketchCanvas are
# driven with injected mouse events (QTest) — the same handlers a
# user's hand triggers; nothing is called behind the widget's back.
#
#   D5.1 creation        -> test_gesture_creation_writes_named_params
#   D5.1 abort           -> test_abort_is_not_an_undo
#   D5.5 out-of-class    -> test_out_of_class_input_refused_visible
#   D5.4 suggestions     -> test_suggestion_refused_and_accepted
#   D5.2 handle drag     -> test_handle_drag_one_undo_dag_regen
#   D5.2/5.3 via-agnostic-> test_text_edit_same_route_bit_identical
#   D5.3 definition state-> test_open_chain_amber_extrude_refused_closed
#   D5.3/D3 crossing     -> test_crossed_sketch_refused_then_recovery
#   D5.2 text edit rule  -> test_length_readonly_radius_editable
#   D5.6 node-as-view    -> test_canvas_is_a_view_of_the_node
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
def sketch_app(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # Modal boxes never block headless runs; the refusal texts are
    # captured for the assertions that need them.
    shown = []
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    monkeypatch.setattr(
        QMessageBox, "critical",
        staticmethod(lambda *a, **k: shown.append(a[2] if len(a) > 2
                                                  else str(a))))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    window._refusals = shown
    yield window
    from PySide6.QtCore import QCoreApplication, QEvent
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    del window
    gc.collect()


def _seed_sketch(win, chains):
    """Add a sketch node seeded via the API (the F-1 pattern: the
    canvas proves the interaction model, the API seeds fixtures)."""
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QTreeWidgetItem
    sid = win._scene.add_sketch_node("S", chains)
    item = QTreeWidgetItem([win._scene.get_node(sid).name, "Sketch"])
    item.setData(0, Qt.UserRole, sid)
    win._feature_tree.addTopLevelItem(item)
    return sid


def _canvas_for(win, sid):
    canvas = win.open_sketch_view(sid)
    assert canvas is not None
    canvas.resize(800, 600)
    canvas.show()
    # Deterministic view: centre on the origin, 60 px per world unit
    # (pan/zoom are transient view state — the test pins them).
    canvas._center = (0.0, 0.0)
    canvas._scale = 60.0
    return canvas


def _open_canvas(win):
    before = win._command_stack.undo_count()
    win.on_create_sketch()
    assert win._command_stack.undo_count() == before + 1
    sid = max(win._sketch_views)
    return sid, _canvas_for(win, sid)


def _seg_chain(points):
    """A seeded chain of connected segments through the points."""
    import CAD_0
    scene = CAD_0.scene
    ents = [scene.sketch_segment(a, b, True)
            for a, b in zip(points, points[1:])]
    return scene.SketchChain(ents)


def _draw_segment(canvas, p0, p1, shift=False):
    """An injected press-drag-release (D5.1's pinned gesture)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    mods = Qt.ShiftModifier if shift else Qt.NoModifier
    canvas.set_tool("segment")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*p0))
    QTest.mouseMove(canvas, QPoint(*p1))
    QTest.mouseRelease(canvas, Qt.LeftButton, mods, QPoint(*p1))


def _draw_circle(canvas, c, rim):
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    canvas.set_tool("circle")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))


def _draw_arc(canvas, c, rim, end):
    """The two-phase gesture: press-drag-release fixes r and a0; the
    click fixes a1 (D5.1's pinned table)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    canvas.set_tool("arc")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))
    QTest.mouseMove(canvas, QPoint(*end))
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*end))


def _undo_count(win):
    return win._command_stack.undo_count()


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_gesture_creation_writes_named_params(sketch_app):
    """D5.1/D6: an injected press-drag-release draws a segment -> the
    entity exists WITH the expected parameters, 1:1, under the named
    fields; same for the circle, and for the arc's two-phase gesture."""
    win = sketch_app
    sid, canvas = _open_canvas(win)

    # Segment: press (100,100) -> release (340,220). World values at
    # 60 px/unit on the origin: a=(-5, 10/3), b=(-1, 4/3).
    _draw_segment(canvas, (100, 100), (340, 220))
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and len(chains[0]) == 1
    seg = chains[0][0]
    assert seg["kind"] == "segment"
    assert seg["a"][0] == pytest.approx(-5.0, abs=1e-6)
    assert seg["a"][1] == pytest.approx(10.0 / 3.0, abs=1e-6)
    assert seg["b"][0] == pytest.approx(-1.0, abs=1e-6)
    assert seg["b"][1] == pytest.approx(4.0 / 3.0, abs=1e-6)
    assert seg["interior_left"] is True

    # The completed gesture routed the entity to the panel (the named
    # parameters under the serialized names of the table — D5.1).
    assert win._selected_entity == (sid, 0, 0)
    assert win._property_panel._header.text().startswith(
        "segment[chain 0 #0]")

    # Circle: press at the centre, release on the rim -> cx, cy, r,
    # OUTER by the declared default (hole=False), its own chain.
    _draw_circle(canvas, (400, 300), (520, 300))
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 2
    circ = chains[1][0]
    assert circ["kind"] == "circle"
    assert circ["center"] == (0.0, 0.0)
    assert circ["radius"] == pytest.approx(2.0, abs=1e-6)
    assert circ["hole"] is False  # OUTER, declared and visible

    # Arc: centre (280,300) -> world (-2, 0); rim (400,300) -> world
    # (0, 0): r = 2 and a0 = 0; the click at (280,180) -> world
    # (-2, 2) fixes a1 = pi/2 -> sweep = pi/2.
    _draw_arc(canvas, (280, 300), (400, 300), (280, 180))
    chains = win._scene.sketch_chains(sid)
    arc = chains[2][0]
    assert arc["kind"] == "arc"
    assert arc["center"][0] == pytest.approx(-2.0, abs=1e-6)
    assert arc["center"][1] == pytest.approx(0.0, abs=1e-6)
    assert arc["radius"] == pytest.approx(2.0, abs=1e-6)
    assert arc["start_angle"] == pytest.approx(0.0, abs=1e-6)
    assert arc["sweep"] == pytest.approx(math.pi / 2.0, abs=1e-5)
    assert arc["hole"] is False

    # Three completed gestures + the sketch creation = 4 undo entries,
    # one per completed gesture (D5.6's granularity).
    assert _undo_count(win) == 4


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_abort_is_not_an_undo(sketch_app):
    """D5.1/D6: ESC mid-gesture -> no entity, no undo entry; ESC
    mid-drag -> parameters untouched. An abort is not an undo."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest

    win = sketch_app
    sid, canvas = _open_canvas(win)
    _draw_segment(canvas, (100, 100), (340, 220))
    before_chains = win._scene.sketch_chains(sid)
    before_undo = _undo_count(win)

    # ESC mid-gesture.
    canvas.set_tool("circle")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(400, 300))
    QTest.mouseMove(canvas, QPoint(500, 300))
    QTest.keyPress(canvas, Qt.Key_Escape)
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(500, 300))
    assert win._scene.sketch_chains(sid) == before_chains
    assert _undo_count(win) == before_undo

    # Right-click aborts too (D5.1: no context menu mid-gesture).
    canvas.set_tool("segment")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(100, 100))
    QTest.mouseMove(canvas, QPoint(300, 300))
    QTest.mouseClick(canvas, Qt.RightButton, Qt.NoModifier, QPoint(300, 300))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(300, 300))
    assert win._scene.sketch_chains(sid) == before_chains
    assert _undo_count(win) == before_undo

    # ESC mid-drag: select the segment, drag its endpoint, ESC.
    canvas.set_tool("select")
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(230, 170))
    assert canvas.selected_entity() == (0, 0)
    handles = {name: canvas.world_to_screen(*pos)
               for name, pos, _role in canvas.handles_for(
                   win._scene.sketch_chains(sid)[0][0])}
    end1 = QPoint(int(handles["endpoint1"][0]), int(handles["endpoint1"][1]))
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, end1)
    QTest.mouseMove(canvas, QPoint(600, 400))
    QTest.keyPress(canvas, Qt.Key_Escape)
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(600, 400))
    assert win._scene.sketch_chains(sid) == before_chains
    assert _undo_count(win) == before_undo


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_out_of_class_input_refused_visible(sketch_app):
    """D5.5/D6: out-of-class input during the gesture (degenerate
    release, r ~ 0, degenerate sweep) -> NO entity created, the
    rejection message visible in the pinned channel, the recipe
    byte-identical to before the gesture."""
    win = sketch_app
    sid, canvas = _open_canvas(win)
    before = win._scene.sketch_chains(sid)
    before_undo = _undo_count(win)

    # Degenerate release: press and release within 2 px.
    _draw_segment(canvas, (400, 300), (401, 300))
    assert win._scene.sketch_chains(sid) == before
    assert _undo_count(win) == before_undo
    assert "segment refused" in canvas.status_message
    assert "2 px" in canvas.status_message

    # r ~ 0: release on the centre.
    _draw_circle(canvas, (400, 300), (401, 300))
    assert win._scene.sketch_chains(sid) == before
    assert _undo_count(win) == before_undo
    assert "circle refused" in canvas.status_message

    # Degenerate sweep: the arc's click at the a0 endpoint.
    _draw_arc(canvas, (200, 300), (350, 300), (351, 300))
    assert win._scene.sketch_chains(sid) == before
    assert _undo_count(win) == before_undo
    assert "sweep" in canvas.status_message

    # The message persists until the next gesture begins (D5.3/D5.5).
    _draw_circle(canvas, (400, 300), (520, 300))
    assert canvas.status_message == ""
    assert len(win._scene.sketch_chains(sid)) == 1


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_suggestion_refused_and_accepted(sketch_app):
    """D5.4/D6: a candidate under threshold, release WITHOUT SHIFT ->
    the recipe holds the pointer's raw values byte-for-byte, no
    hidden write, no structural change; WITH SHIFT -> the candidate's
    values, explicit, and the accepted coincidence appends the entity
    to the connected chain (the structural effect)."""
    win = sketch_app
    _sid, _canvas = _open_canvas(win)   # a first sketch (untouched)
    sid = _seed_sketch(win, [_seg_chain([(0.0, 0.0), (2.0, 0.0)])])
    canvas = _canvas_for(win, sid)

    # A horizontal candidate is under threshold (|dy| = 3 px < 6):
    # press (100,300) -> world (-5, 0); release (300,297) -> world
    # (-5/3, 0.05). Without SHIFT the raw y is written verbatim — no
    # auto-flatten, no hidden state, and the entity opens its OWN
    # chain (no structural effect without acceptance).
    _draw_segment(canvas, (100, 300), (300, 297))
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 2
    seg = chains[1][0]
    assert seg["a"][0] == pytest.approx(-5.0, abs=1e-6)
    assert seg["a"][1] == pytest.approx(0.0, abs=1e-6)
    assert seg["b"][1] == pytest.approx(0.05, abs=1e-6)
    assert seg["b"][1] != pytest.approx(seg["a"][1], abs=1e-9)

    # With SHIFT: the candidate's value is written (y1 = y0).
    _draw_segment(canvas, (100, 350), (300, 347), shift=True)
    chains = win._scene.sketch_chains(sid)
    seg2 = chains[2][0]
    assert seg2["b"][1] == pytest.approx(seg2["a"][1], abs=1e-6)

    # The accepted endpoint coincidence: release within 8 px of the
    # seeded segment's start (0,0) = screen (400,300), WITH SHIFT ->
    # the new endpoint is exactly (0,0) and the entity joins chain 0.
    _draw_segment(canvas, (150, 450), (403, 303), shift=True)
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 3
    closing = chains[0][1]
    assert closing["b"] == (0.0, 0.0)  # the candidate's exact value

    # The same release WITHOUT SHIFT: raw values, a separate chain.
    _draw_segment(canvas, (150, 500), (403, 303))
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 4
    raw = chains[3][0]
    assert raw["b"] != (0.0, 0.0)    # the pointer's raw position
    assert raw["b"][0] == pytest.approx(0.05, abs=1e-6)
    assert raw["b"][1] == pytest.approx(-0.05, abs=1e-6)


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_handle_drag_one_undo_dag_regen(sketch_app):
    """D5.2/D6: endpoint drag released -> the parameter changed AND
    the dependent DAG re-materialized, as ONE undo entry; undo
    restores the parameter and the dependent follows."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    from CAD_0 import math as _math

    win = sketch_app
    sid, canvas = _open_canvas(win)
    _draw_circle(canvas, (400, 300), (520, 300))   # r = 2.0, chain 0
    ex = win._do_extrude(sid, 1.5, 1)
    assert ex is not None
    body = win._scene.get_node(ex).body
    # A point the PROFILE dominates before and after the edit.
    probe = _math.Vec3f(2.5, 0.0, 0.75)

    def field():
        return body.value(probe)

    before = field()   # 2.5 - 2.0 = 0.5

    # Select the circle and drag its radius point: (520,300) ->
    # (580,300): r 2.0 -> 3.0.
    canvas.set_tool("select")
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(520, 300))
    assert canvas.selected_entity() == (0, 0)
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(520, 300))
    QTest.mouseMove(canvas, QPoint(580, 300))
    undo_before = _undo_count(win)
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(580, 300))
    assert _undo_count(win) == undo_before + 1   # ONE undo entry

    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(3.0, abs=1e-6)
    # The DAG regenerated the dependent: the wall moved with r.
    assert field() == pytest.approx(before - 1.0, abs=1e-6)

    # Undo: the parameter (and the dependent) return.
    win.on_undo()
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(2.0, abs=1e-6)
    assert field() == pytest.approx(before, abs=1e-6)


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_text_edit_same_route_bit_identical(sketch_app):
    """D5.2/D5.3/D6: the same final state reached by text edit
    produces the BIT-IDENTICAL prism — the via-agnostic property
    (gesture and text write the SAME named parameter through the SAME
    route; ADR-0014 across interaction paths)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    from CAD_0 import math as _math

    win = sketch_app
    sid, canvas = _open_canvas(win)
    _draw_circle(canvas, (400, 300), (520, 300))   # r = 2.0, chain 0
    ex = win._do_extrude(sid, 1.5, 1)
    body = win._scene.get_node(ex).body
    probe = _math.Vec3f(2.5, 0.0, 0.75)

    def field():
        return body.value(probe)

    # Via GESTURE (drag): r -> 3.0.
    canvas.set_tool("select")
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(520, 300))
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(520, 300))
    QTest.mouseMove(canvas, QPoint(580, 300))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(580, 300))
    r_dragged = win._scene.sketch_chains(sid)[0][0]["radius"]
    v_drag = field()
    assert r_dragged == pytest.approx(3.0, abs=1e-6)
    win.on_undo()

    # Via TEXT (the panel route): the same r the drag wrote.
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(520, 300))
    assert canvas.selected_entity() == (0, 0)
    assert win._selected_entity == (sid, 0, 0)
    win._on_parameter_changed("r", float(r_dragged))
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == r_dragged
    # Same recipe state -> the same prism, bit-identical.
    assert field() == v_drag


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_open_chain_amber_extrude_refused_closed(sketch_app):
    """D5.3/D5.5/D6: an open chain renders under-defined (the amber
    class, derived from recipe connectivity); Extrude refused with the
    open entities named; after an accepted coincidence closes the
    chain -> Extrude succeeds."""
    win = sketch_app
    # Seeded three-side chain (open): (0,0)->(2,0)->(2,2)->(0.5,2) —
    # the closing edge is deliberately NOT axis-aligned with its
    # target endpoint, so the coincidence candidate is the nearest
    # one at the completing release (the pinned nearest rule).
    sid = _seed_sketch(
        win, [_seg_chain([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0),
                          (0.5, 2.0)])])
    canvas = _canvas_for(win, sid)

    # The definition state is derived from recipe connectivity, the
    # aspect a pure function of it (the pair the smoke asserts).
    assert not win._scene.sketch_closed(sid)
    aspects = canvas.entity_aspects()
    assert set(aspects.values()) == {"under_defined"}
    # The distance to closure is shown (D5's amendment).
    gaps = canvas.open_chain_gaps()
    assert len(gaps) == 1
    assert gaps[0][2] == pytest.approx(math.hypot(0.5, 2.0), abs=1e-6)

    # Extrude on the open chain: refused, the entities named, nothing
    # written (the kernel's own message — one opinion).
    count = win._scene.node_count()
    assert win._do_extrude(sid, 1.0, 1) is None
    assert win._scene.node_count() == count
    assert win._refusals
    refusal = win._refusals[-1]
    assert "must share their joint" in refusal
    assert "segment[chain 0 #" in refusal

    # The closing segment: press at (0.5,2) [the chain's end],
    # release within 8 px of (0,0) [the chain's start] WITH SHIFT ->
    # the accepted coincidence (the nearest candidate here) writes
    # (0,0) exactly and appends the entity to chain 0.
    _draw_segment(canvas, (430, 180), (402, 302), shift=True)
    assert win._scene.sketch_closed(sid)
    assert set(canvas.entity_aspects().values()) == {"outer_closed"}

    # Now the extrude succeeds.
    ex = win._do_extrude(sid, 1.0, 1)
    assert ex is not None
    assert not win._scene.get_node(ex).operand_missing


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_crossed_sketch_refused_then_recovery(sketch_app):
    """D5.3/D3/D6: a crossed sketch (a hole circle over an outer
    segment, the crossing mid-span) -> Extrude refused with BOTH
    entities and the crossing point named; after the hole is dragged
    clear -> Extrude succeeds (the same refusal/recovery shape)."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    import CAD_0

    win = sketch_app
    scene = CAD_0.scene
    # Closed outer rectangle + a hole circle crossing the bottom
    # segment MID-SPAN, strictly between the joint and the midpoint
    # (D3's lie #5b configuration): every certificate sample passes,
    # the pairwise instrument refuses — both entities and the crossing
    # point named.
    outer = _seg_chain([(0.0, 0.0), (4.0, 0.0), (4.0, 2.0),
                        (0.0, 2.0), (0.0, 0.0)])
    hole = scene.SketchChain([scene.sketch_circle((0.8, 0.2), 0.5, True)])
    sid = _seed_sketch(win, [outer, hole])
    canvas = _canvas_for(win, sid)

    assert win._scene.sketch_closed(sid)
    # The hole role renders with its own aspect (closed chain, hole).
    assert canvas.entity_aspects()[(1, 0)] == "hole_closed"

    # Extrude: the pairwise certificate refuses the mid-span crossing,
    # both entities and the crossing point named.
    assert win._do_extrude(sid, 1.0, 1) is None
    refusal = win._refusals[-1]
    assert "intersect only at declared shared joints" in refusal
    assert "circle[chain 1 #0]" in refusal
    assert "segment[chain 0 #0]" in refusal
    assert "crossing at" in refusal

    # Drag the hole clear: click its curve to select, then drag the
    # centre handle: (0.8,0.2) -> (2,1) — strictly inside, clear of
    # every segment (a tangent would refuse too — tangency included).
    canvas.set_tool("select")
    rim = canvas.world_to_screen(1.3, 0.2)
    QTest.mouseClick(canvas, Qt.LeftButton, Qt.NoModifier,
                     QPoint(int(rim[0]), int(rim[1])))
    assert canvas.selected_entity() == (1, 0)
    centre = canvas.world_to_screen(0.8, 0.2)
    clear = canvas.world_to_screen(2.0, 1.0)
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier,
                     QPoint(int(centre[0]), int(centre[1])))
    QTest.mouseMove(canvas, QPoint(int(clear[0]), int(clear[1])))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier,
                       QPoint(int(clear[0]), int(clear[1])))
    chains = win._scene.sketch_chains(sid)
    assert chains[1][0]["center"][0] == pytest.approx(2.0, abs=1e-6)
    assert chains[1][0]["center"][1] == pytest.approx(1.0, abs=1e-6)

    # The same command now succeeds — the recovery is an edit away.
    ex = win._do_extrude(sid, 1.0, 1)
    assert ex is not None


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_length_readonly_radius_editable(sketch_app):
    """D5.2/D6: the radius is text-editable (it writes r, a named
    recipe parameter with no hidden anchor choice); the displayed
    LENGTH is read-only (writing it needs the anchor choice) and says
    so visibly."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest

    win = sketch_app
    sid, canvas = _open_canvas(win)
    _draw_circle(canvas, (400, 300), (520, 300))   # chain 0: r = 2
    _draw_segment(canvas, (100, 100), (340, 220))  # chain 1

    # The dimension labels: the circle's r and the segment's length.
    labels = canvas.dimension_labels()
    assert any(t.startswith("r =") for t in labels)
    assert any(t.startswith("length =") for t in labels)

    # Double-click on the radius label: the inline editor opens, the
    # edit of r lands through the same route (one undo entry).
    canvas.select_entity(0, 0)
    r_label = next(t for t in labels if t.startswith("r ="))
    wpos = labels[r_label][0]
    ls = canvas.world_to_screen(*wpos)
    undo_before = _undo_count(win)
    QTest.mouseDClick(canvas, Qt.LeftButton, Qt.NoModifier,
                      QPoint(int(ls[0]) + 4, int(ls[1]) - 4))
    assert canvas._inline_editor is not None
    canvas._inline_editor.setText("2.5")
    canvas._inline_editor.returnPressed.emit()
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(2.5, abs=1e-6)
    assert _undo_count(win) == undo_before + 1

    # Double-click on the segment's length label: read-only, visibly.
    canvas.select_entity(1, 0)
    length_label = next(t for t in labels if t.startswith("length ="))
    wpos = canvas.dimension_labels()[length_label][0]
    ls = canvas.world_to_screen(*wpos)
    QTest.mouseDClick(canvas, Qt.LeftButton, Qt.NoModifier,
                      QPoint(int(ls[0]) + 4, int(ls[1]) - 4))
    assert canvas._inline_editor is None
    assert "read-only" in canvas.status_message

    # A refused text edit (r <= 0): the field refuses, the previous
    # value stands (D5.5).
    canvas.select_entity(0, 0)
    canvas.edit_entity_params(0, 0, {"r": -1.0}, "Edit radius")
    chains = win._scene.sketch_chains(sid)
    assert chains[0][0]["radius"] == pytest.approx(2.5, abs=1e-6)
    assert "radius" in canvas.status_message


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_canvas_is_a_view_of_the_node(sketch_app):
    """D5.6/D6: the canvas is a VIEW of the sketch node — one view per
    sketch; closing the tab commits nothing (every gesture already
    wrote); deleting the sketch closes the view and the extrude goes
    dangling; undo recovers it, id-stable, and the view can reopen."""
    from PySide6.QtCore import Qt

    win = sketch_app
    sid, canvas = _open_canvas(win)
    _draw_circle(canvas, (400, 300), (520, 300))
    ex = win._do_extrude(sid, 1.5, 1)

    # One view per sketch: opening again focuses the same widget.
    assert win.open_sketch_view(sid) is canvas
    assert len(win._sketch_views) == 1

    # Closing the view commits nothing: the recipe is unchanged.
    win._on_tab_close_requested(win._tabs.indexOf(canvas))
    assert sid not in win._sketch_views
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and chains[0][0]["kind"] == "circle"

    # Deleting the sketch: the extrude goes dangling (visible), the
    # last body kept; the row shows the flagged state.
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == sid:
            win._feature_tree.setCurrentItem(it)
            break
    win.on_delete_selected()
    assert win._scene.get_node(sid) is None
    node = win._scene.get_node(ex)
    assert node.operand_missing
    assert not node.operation_pending
    row_type = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == ex:
            row_type = it.text(1)
    assert row_type == "FEAT!"

    # Undo: the sketch returns with its original id and payload.
    win.on_undo()
    assert win._scene.get_node(sid) is not None
    assert win._scene.get_node(sid).id == sid
    chains = win._scene.sketch_chains(sid)
    assert len(chains) == 1 and chains[0][0]["kind"] == "circle"
    # The view can be reopened (the canvas is a view, not an owner).
    reopened = win.open_sketch_view(sid)
    assert reopened is not None and reopened is not canvas
