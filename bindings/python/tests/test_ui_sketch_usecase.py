# bindings/python/tests/test_ui_sketch_usecase.py
#
# ADR-0030 D6 — the draft's end-to-end flow stands, driven through
# the REAL app (offscreen): create sketch -> draw the circle WITH THE
# GESTURE -> Extrude -> Subtract from a box -> the hole is in the
# tree and the Debug report shows chi = 0, genus 1; a radius edit
# propagates (the report follows); delete sketch -> the dangling row;
# undo -> recovered, same ids; save/load round-trip with the same
# report; undo is gesture-granular (LIFO over creations and edits).
#
# examples/sketch_hole.py is the same recipe in code (D5.8's two
# faces of one recipe) — its assertions run here too.
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
def app_window(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    monkeypatch.setattr(
        QMessageBox, "critical",
        staticmethod(lambda *a, **k: None))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    yield window
    from PySide6.QtCore import QCoreApplication, QEvent
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    del window
    gc.collect()


def _draw_circle(canvas, c, rim):
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    canvas.set_tool("circle")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))


def _tree_row(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            return it
    return None


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_parametric_hole_end_to_end(app_window, tmp_path):
    """The final use case, executable by the user: box -> sketch
    circle drawn -> Extrude -> Subtract(box, prism) -> the hole; the
    Debug report shows the expected genus."""
    from PySide6.QtCore import Qt
    from CAD_0 import math as _math

    win = app_window
    assert win._has_kernel

    # The box (the target) — created from the registry defaults, then
    # EDITED through the param path BEFORE the subtract (the honest
    # in-app flow: the default extent 0.5 would leave the offset
    # prism tangent to the side walls; extent 1.2 gives the pierced
    # box of the API tests).
    win._create_registered("box")
    box_id = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.text(1) == "SDF":
            box_id = it.data(0, Qt.UserRole)
    assert box_id is not None
    row = _tree_row(win, box_id)
    win._feature_tree.setCurrentItem(row)
    for axis in ("extent_x", "extent_y", "extent_z"):
        win._on_parameter_changed(axis, 1.2)
    box_body = win._scene.get_node(box_id).body
    b = box_body.bounds()
    assert b.min.x == pytest.approx(-1.2, abs=1e-6)
    assert b.max.x == pytest.approx(1.2, abs=1e-6)

    # The sketch, its canvas, and THE GESTURE: press at the centre,
    # release on the rim -> r = 0.35 (world), OUTER by default.
    win.on_create_sketch()
    sketch_id = max(win._sketch_views)
    canvas = win._sketch_views[sketch_id]
    canvas.resize(800, 600)
    canvas.show()
    canvas._center = (0.0, 0.0)
    canvas._scale = 400.0 / 0.7   # 0.35 world = 200 px
    _draw_circle(canvas, (400, 300), (600, 300))
    chains = win._scene.sketch_chains(sketch_id)
    assert len(chains) == 1 and chains[0][0]["kind"] == "circle"
    assert chains[0][0]["radius"] == pytest.approx(0.35, abs=1e-6)
    assert chains[0][0]["hole"] is False
    assert win._scene.sketch_closed(sketch_id)

    # Feature -> Extrude (the core; the dialog is chrome): distance
    # 3.0, +Z. The prism row: "Extrude (Sketch#N)", Type FEAT.
    prism_id = win._do_extrude(sketch_id, 3.0, 1)
    assert prism_id is not None
    row = _tree_row(win, prism_id)
    assert row.text(0) == f"Extrude (Sketch#{sketch_id})"
    assert row.text(1) == "FEAT"
    prism = win._scene.get_node(prism_id)
    assert prism.kind == "sdf"  # the prism IS a body
    assert prism.operation["sketch"] == sketch_id

    # Edit -> Subtract(box, prism): the genealogy the canvas's use
    # case consumes (the tool is the EXTRUDE's FeatureId).
    hole_id = win._do_subtract(box_id, prism_id, 0.3, 0.0, -1.5)
    assert hole_id is not None
    hole_row = _tree_row(win, hole_id)
    assert hole_row.text(1) == "CSG"
    body = win._scene.get_node(hole_id).body

    # The hole is real: on the axis, inside the removed prism.
    assert body.value(_math.Vec3f(0.3, 0.0, 0.0)) == pytest.approx(
        0.35, abs=1e-6)

    # The Debug report: chi = 0, genus 1 (the same pure string builder
    # the Debug menu shows).
    text = win._sdf_to_brep_report_text(win._scene.get_node(hole_id))
    assert "\u03c7 = V - E + F = 0" in text
    assert "Genus = 1" in text

    # THE EDIT: the circle's radius (the panel's text route). The
    # report follows: the hole deepens to 0.5, still genus 1.
    canvas.select_entity(0, 0)
    assert win._selected_entity == (sketch_id, 0, 0)
    win._on_parameter_changed("r", 0.5)
    chains = win._scene.sketch_chains(sketch_id)
    assert chains[0][0]["radius"] == pytest.approx(0.5, abs=1e-6)
    body = win._scene.get_node(hole_id).body
    assert body.value(_math.Vec3f(0.3, 0.0, 0.0)) == pytest.approx(
        0.5, abs=1e-6)
    text = win._sdf_to_brep_report_text(win._scene.get_node(hole_id))
    assert "Genus = 1" in text

    # Delete the sketch: the extrude goes dangling (visible rows,
    # last body kept); undo recovers it with the SAME id.
    row = _tree_row(win, sketch_id)
    win._feature_tree.setCurrentItem(row)
    win.on_delete_selected()
    assert win._scene.get_node(sketch_id) is None
    assert win._scene.get_node(prism_id).operand_missing
    assert _tree_row(win, prism_id).text(1) == "FEAT!"
    # The subtract's operand (the extrude) still EXISTS with its kept
    # body — the display continues, the row stays healthy; the
    # problem is flagged where it lives (the extrude's row).
    assert _tree_row(win, hole_id).text(1) == "CSG"
    # The last good body is kept — display (and the report) continue.
    assert win._sdf_to_brep_report_text(
        win._scene.get_node(hole_id)) is not None

    win.on_undo()   # the sketch delete
    assert win._scene.get_node(sketch_id) is not None
    assert win._scene.get_node(sketch_id).id == sketch_id
    chains = win._scene.sketch_chains(sketch_id)
    assert chains[0][0]["radius"] == pytest.approx(0.5, abs=1e-6)
    assert _tree_row(win, prism_id).text(1) == "FEAT"
    assert _tree_row(win, hole_id).text(1) == "CSG"

    # The v0.4 save/load round-trip: the same recipe, the same
    # report (the recipe is the document).
    path = tmp_path / "the_hole.CAD_0"
    import CAD_0
    CAD_0.io.save(path, win._scene)
    fresh = CAD_0.scene.Scene()
    CAD_0.io.load(path, fresh)
    loaded_hole = fresh.get_node(hole_id)
    assert loaded_hole is not None
    loaded_text = win._sdf_to_brep_report_text(loaded_hole)
    assert "Genus = 1" in loaded_text
    assert loaded_hole.body.value(_math.Vec3f(0.3, 0.0, 0.0)) == \
        pytest.approx(0.5, abs=1e-6)

    # Undo is gesture-granular, LIFO over creations and edits: the
    # radius edit, the subtract, the extrude, the circle, the sketch,
    # the box edits, the box.
    win.on_undo()   # the radius edit
    chains = win._scene.sketch_chains(sketch_id)
    assert chains[0][0]["radius"] == pytest.approx(0.35, abs=1e-6)
    win.on_undo()   # the subtract
    assert win._scene.get_node(hole_id) is None
    win.on_undo()   # the extrude
    assert win._scene.get_node(prism_id) is None
    win.on_undo()   # the circle gesture
    # The entity is gone; the empty chain remains (a legal editing
    # state — removing the last entity leaves the chain).
    assert win._scene.sketch_chains(sketch_id) == [[]]
    win.on_undo()   # the sketch creation
    assert win._scene.get_node(sketch_id) is None
    for _axis in ("extent_x", "extent_y", "extent_z"):
        win.on_undo()   # the box's param edits, LIFO
    b = win._scene.get_node(box_id).body.bounds()
    assert b.min.x == pytest.approx(-0.5, abs=1e-6)
    assert b.max.x == pytest.approx(0.5, abs=1e-6)
    win.on_undo()   # the box
    assert win._scene.get_node(box_id) is None


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_sketch_hole_example_is_the_same_recipe():
    """D5.8: examples/sketch_hole.py runs clean — the CLI face of the
    same recipe (entity creation, extrude, subtract by FeatureId, the
    bit-identical radius propagation, the v0.4 round-trip, the
    genus-1 report)."""
    import subprocess
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    example = root / "examples" / "sketch_hole.py"
    assert example.exists()
    result = subprocess.run(
        [sys.executable, str(example)],
        capture_output=True, text=True, timeout=600,
        cwd=str(root))
    out = result.stdout
    assert result.returncode == 0, result.stderr[-2000:]
    assert "Genus = 1" in out
    assert "chi = V - E + F = 0" in out
    assert "0.35" in out and "0.5" in out
