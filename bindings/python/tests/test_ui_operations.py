"""ADR-0027 Phase F — UI operations smoke test (offscreen).

Drives the REAL MainWindow (offscreen QPA, the established driver
pattern) through the boolean feature's app-level contract:

  * create box + cylinder → subtract → the operation row exists
    ("CSG" type, operands named), the operands are hidden from the
    viewport, and the operation's body is the subtract;
  * undo removes the operation and un-hides the operands; redo
    restores it WITH THE SAME ID and the same operand references
    (never a rebind — ADR-0027 D2);
  * the Debug report on the operation node shows χ = 0 and genus 1
    (the first genus > 0 body, ADR-0027 D6);
  * deleting an operand leaves a VISIBLE dangling state (last-good
    body kept, "CSG!" row, actionable error message) and undoing the
    delete recovers it;
  * operand param edits propagate through the DAG (edit the cylinder
    radius → the operation's field changes accordingly);
  * the operation's placement offset is editable and re-materializes
    the body (world-anchored offset, review correction 2).

Requires the compiled extension and PySide6; runs under
QT_QPA_PLATFORM=offscreen with the EGL software-GL workaround.
"""

from __future__ import annotations

import gc

import pytest

from CAD_0 import math as _math


def _v(x, y, z):
    """Vec3f helper — SDFBody.value wants a real Vec3f."""
    return _math.Vec3f(x, y, z)


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

    # Avoid modal dialogs blocking headless runs (close prompts,
    # warnings). The subtract DIALOG is never used by these tests (they
    # drive the dialog-free core _do_subtract, the _import_mesh
    # pattern); question boxes default to Discard like the navigation
    # fixture.
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    yield window
    # Teardown: the documented offscreen pattern (see
    # test_ui_navigation.py) — flush Qt's deferred deletes so the C++
    # QWidget does not outlive interpreter shutdown holding the
    # Python-wrapped Scene and CommandStack.
    from PySide6.QtCore import QCoreApplication, QEvent
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    del window
    gc.collect()


def _tree_ids(win):
    from PySide6.QtCore import Qt
    ids = []
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        ids.append(it.data(0, Qt.UserRole))
    return ids


def _row_text_for(win, node_id, column):
    """The tree-row text of a node id (search by id, not by index —
    rows shift when operands are deleted)."""
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            return it.text(column)
    return None


def _select_row(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            win._feature_tree.setCurrentItem(it)
            return True
    return False


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_subtract_flow_end_to_end(main_window):
    win = main_window
    assert win._has_kernel

    # Create the operands (registry defaults). The cylinder's default
    # radius (0.5) would be TANGENT to the default box's faces (extent
    # 0.5 → [-0.5, 0.5]^3) — a P3 violation — so the tool is edited to
    # r = 0.2 through the param-edit path BEFORE the subtract (the
    # honest in-app flow).
    win._create_registered("box")
    win._create_registered("cylinder")
    box_id, cyl_id = _tree_ids(win)[-2:]

    win._node_params[cyl_id]["radius"] = 0.2
    win._rebuild_node(cyl_id)
    assert win._scene.get_node(cyl_id).body.describe().startswith(
        "cylinder(")

    # ---- Subtract (dialog-free core, the _import_mesh pattern).
    win._do_subtract(box_id, cyl_id, 0.0, 0.0, 0.0)
    ids = _tree_ids(win)
    assert len(ids) == 3
    op_id = ids[-1]

    op_node = win._scene.get_node(op_id)
    assert op_node is not None
    assert op_node.operation is not None
    assert op_node.operation["target"] == box_id
    assert op_node.operation["tool"] == cyl_id
    # The row names its operands; the Type column is CSG.
    from PySide6.QtCore import Qt
    row = win._feature_tree.topLevelItem(2)
    assert f"Box#{box_id}" in row.text(0)
    assert f"Cylinder#{cyl_id}" in row.text(0)
    assert row.text(1) == "CSG"
    assert row.data(0, Qt.UserRole) == op_id
    # The operands are hidden from the viewport (still in the tree).
    assert win._scene.get_node(box_id).visible is False
    assert win._scene.get_node(cyl_id).visible is False
    # The materialized body IS the subtract.
    assert op_node.body.describe().startswith("subtract(")
    # Inside the hole the field is positive (outside the result):
    # r = 0.2 → f(0,0,0) = 0.2 exactly.
    assert op_node.body.value(_v(0.0, 0.0, 0.0)) == pytest.approx(0.2)

    # ---- Undo: the operation is removed, the operands visible again.
    win.on_undo()
    assert _tree_ids(win) == [box_id, cyl_id]
    assert win._scene.get_node(op_id) is None
    assert win._scene.get_node(box_id).visible is True
    assert win._scene.get_node(cyl_id).visible is True

    # ---- Redo: the operation returns WITH THE SAME ID and the same
    # operand references — never a rebind (ADR-0027 D2).
    win.on_redo()
    ids_after = _tree_ids(win)
    assert ids_after == [box_id, cyl_id, op_id]
    op_node = win._scene.get_node(op_id)
    assert op_node is not None
    assert op_node.operation["target"] == box_id
    assert op_node.operation["tool"] == cyl_id
    assert op_node.body.describe().startswith("subtract(")
    assert win._scene.get_node(box_id).visible is False
    assert win._scene.get_node(cyl_id).visible is False

    # ---- The Debug report on the operation node: χ = 0, genus 1,
    # closed-orientable yes (ADR-0027 D6 — the report content, built
    # by the pure text helper so it is assertable offscreen). G1
    # unit 6: the report GREW — mass + provenance beside the
    # structural block, and the χ label carries the H term.
    win._selected_node_id = op_id
    text = win._body_report_text(win._scene.get_node(op_id))
    assert text is not None
    assert "faceted extraction (MC33)" in text
    chi_line = [ln for ln in text.splitlines()
                if "V - E + F - H =" in ln][0]
    assert chi_line.endswith("= 0")
    assert "Genus = 1" in text
    assert "Closed orientable (is_closed_orientable): yes" in text
    # The grown blocks: the mass (approximated — the faceted body's
    # own polyhedral numbers) and the provenance tallies.
    assert "APPROXIMATED (all_exact = False" in text
    assert "subshapes exact / " in text
    assert "min |f| on grid (P2 margin):" in text

    # ---- Operand param edit propagates through the DAG (the radius
    # feeds the operation's field: f(0,0,0) tracks the tool radius).
    win._node_params[cyl_id]["radius"] = 0.3
    win._rebuild_node(cyl_id)
    op_node = win._scene.get_node(op_id)
    assert op_node.body.value(_v(0.0, 0.0, 0.0)) == pytest.approx(0.3)
    # The offset stays world-anchored (editing the radius did NOT move
    # the hole — review correction 2: the hole is still centered).
    assert op_node.operation["offset_x"] == 0.0

    # ---- The placement offset is editable and re-materializes,
    # through the REAL param-edit flow (the property panel signal).
    # With the tool at r = 0.3 centered (offset 0): f(0,0,0) = 0.3
    # (hole center), f(0.1,0,0) = 0.2. After offset_x = 0.1 the hole
    # moves: the two readings SWAP (the world-anchored shift, review
    # correction 2 — editing the offset moves the hole).
    assert op_node.body.value(_v(0.1, 0.0, 0.0)) == pytest.approx(0.2)
    win._selected_node_id = op_id
    win._on_parameter_changed("offset_x", 0.1)
    op_node = win._scene.get_node(op_id)
    assert op_node.body.value(_v(0.0, 0.0, 0.0)) == pytest.approx(0.2)
    assert op_node.body.value(_v(0.1, 0.0, 0.0)) == pytest.approx(0.3)
    assert op_node.operation["offset_x"] == pytest.approx(0.1)
    # And the offset edit is undoable (the recipe rolls back).
    win.on_undo()
    op_node = win._scene.get_node(op_id)
    assert op_node.operation["offset_x"] == pytest.approx(0.0)
    assert op_node.body.value(_v(0.0, 0.0, 0.0)) == pytest.approx(0.3)

    # ---- Operand deletion → the VISIBLE dangling state (undo of the
    # delete → recovered; the LIFO interleave is exercised in the C++
    # suite — here the app-level surface).
    assert _select_row(win, cyl_id)
    win.on_delete_selected()
    assert win._scene.get_node(cyl_id) is None
    op_node = win._scene.get_node(op_id)
    assert op_node is not None
    assert op_node.operand_missing is True
    # The last-good body is KEPT for display.
    assert op_node.body.describe().startswith("subtract(")
    # The row flags the error; the panel message is actionable.
    assert _row_text_for(win, op_id, 1) == "CSG!"
    err = win._scene.operation_error(op_id)
    assert f"#{cyl_id}" in err
    assert "(tool)" in err
    assert "undo" in err
    # The Debug report still works on the dangling node (its body is
    # the kept last-good geometry).
    win._selected_node_id = op_id
    text = win._body_report_text(op_node)
    assert text is not None
    chi_line = [ln for ln in text.splitlines()
                if "V - E + F - H =" in ln][0]
    assert chi_line.endswith("= 0")

    # ---- Undo the delete: the operand returns with the SAME id and
    # the operation recovers.
    win.on_undo()
    assert win._scene.get_node(cyl_id) is not None
    assert win._scene.get_node(cyl_id).id == cyl_id
    op_node = win._scene.get_node(op_id)
    assert op_node.operand_missing is False
    assert op_node.operation["tool"] == cyl_id  # unchanged reference
    assert _row_text_for(win, op_id, 1) == "CSG"


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_create_undo_redo_same_ids(main_window):
    """ADR-0027 D2 prerequisite, app-level: create's redo keeps the id
    (the pre-Phase-F behavior re-allocated ids on redo)."""
    win = main_window
    win._create_registered("box")
    ids = _tree_ids(win)
    assert len(ids) == 1
    nid = ids[0]

    win.on_undo()
    assert _tree_ids(win) == []
    assert win._scene.get_node(nid) is None

    win.on_redo()
    ids_after = _tree_ids(win)
    assert ids_after == [nid]
    node = win._scene.get_node(nid)
    assert node is not None
    assert node.id == nid
    assert node.builder == "box"


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_subtract_needs_two_sdf_bodies(main_window, monkeypatch):
    """The entry point guards: not-two-bodies is a visible message, not
    a guess (the dialog is never reached)."""
    win = main_window
    win._create_registered("box")
    # One body only → warning (mocked via monkeypatch, the navigation
    # pattern) and no operation created.
    from PySide6.QtWidgets import QMessageBox
    calls = []
    monkeypatch.setattr(
        QMessageBox, "information",
        staticmethod(lambda *a, **k: calls.append(a)
                     or QMessageBox.StandardButton.Ok))
    win.on_subtract()
    assert calls  # the warning was shown
    assert win._scene.node_count() == 1  # nothing happened


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_debug_report_multi_shell_genus_n_d(main_window):
    """ADR-0027 D6 / acceptance correction 1, the Debug-report surface.

    The genus guard's OTHER branch: a multi-shell body (two disjoint
    spheres, chi = 4) must report genus "n/d" in the Debug report,
    coherently with SdfToBrepReport (bridge T15) and the binding dict
    (test_report_genus_rule_derived_and_n_d). The derived branch is
    already pinned above on the pierced box (chi = 0, Genus = 1).
    """
    import CAD_0
    from CAD_0 import sdf as sdf_mod

    vec = _math.Vec3f
    two = sdf_mod.union(
        sdf_mod.sphere(1.0),
        sdf_mod.translate(sdf_mod.sphere(1.0), vec(5.0, 0.0, 0.0)))
    nid = main_window._scene.add_node("TwoSpheres", two)

    text = main_window._body_report_text(
        main_window._scene.get_node(nid))
    assert text is not None
    assert "faceted extraction (MC33)" in text
    chi_line = [ln for ln in text.splitlines()
                if "V - E + F - H =" in ln][0]
    assert chi_line.endswith("= 4")   # chi = 4: two closed shells
    assert "Genus = n/d" in text       # NOT derived (chi > 2)
    assert "multi-shell" in text
    assert "Closed orientable (is_closed_orientable): yes" in text
