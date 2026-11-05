# bindings/python/tests/test_ui_fillet_workbench.py
#
# G2-u3 m5 — IL FILLET, IL MORSO 4: L'UI (ADR-0039 D6 rung 5): the UI
# smoke suite extended, the six surfaces each mapped to a test driven
# through the WIRING (the real MainWindow + the real FilletDialog, the
# dialog's own widgets fired the way a user's hand fires them — the
# established G4 m3 driver; the modal exec is presentation, the
# widgets are the surface):
#
#   1. THE SELECTION      -> test_the_selection_resolves_the_reference
#      both guide cases: the edge list carries the kernel's OWN names
#      (box#1:vertical2, the blind rim's pinned string), the clicked
#      row resolves the reference CHARACTER FOR CHARACTER — the user
#      sees the name the kernel uses, never a parse of the render;
#   2. THE APPLICATION    -> test_the_application_produces_the_golden_body
#      the fillet applied from the dialog at the guide radii: the
#      panel's counts/volume rows are the KERNEL's own answers (read
#      back through the report module — additivity: the golden corpus
#      unmoved, the numbers not re-declared), ONE undo entry;
#   3. THE REFUSALS       -> test_the_refusals_in_words_with_the_bound
#      the three typed bands refused IN THE DIALOG, the kernel's own
#      message quoted verbatim — the offending radius AND the bound
#      inside the text (fillet_radius_exceeds_face, fillet_exits_
#      annulus, fillet_eats_wall), the dialog staying open;
#   4. THE EDIT           -> test_the_radius_edit_class1_silent
#      the radius edited from the panel's own editor: ONE undo entry,
#      the counts FIXED for the in-band r, the volume the closed
#      form's own value, the reference row unchanged, the selection
#      NEVER taken back to the dialog;
#   5. THE DEATH          -> test_the_death_of_the_reference_class4
#      the sketch re-author (the tool's circle replaced by the square
#      chain, the quartet's own scenario) + the placement edit through
#      the panel: ONE rebuild, the class-4 payload SHOWN (what died,
#      why, the remedy), the last good body KEPT, never a crash;
#   6. THE DETERMINISM    -> test_the_same_sequence_twice_the_same_scene
#      the whole gesture sequence run twice: the same ids, the same
#      names, the same numbers — and the edit's undo/redo round trip
#      bit-identical (no hidden state, no random id in the UI).
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

    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    monkeypatch.setattr(
        QMessageBox, "critical",
        staticmethod(lambda *a, **k: None))
    monkeypatch.setattr(
        QMessageBox, "warning",
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


# --- The deterministic driver (the G4 m3 gestures' own discipline) ------

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
    """(label, value-text) for every row the panel is showing."""
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


def _row_value(win, name):
    for label, text in _panel_rows(win):
        if label == name:
            return text
    return None


def _select_row(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            win._feature_tree.setCurrentItem(it)
            return True
    return False


def _row_type(win, node_id):
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            return it.text(1)
    return None


def _open_fillet_dialog(win, selected=None):
    """The dialog built the way Feature → Fillet builds it (the same
    constructor, the same injected route) — the modal exec is
    presentation, the widgets are the surface under test."""
    from CAD_0.ui.fillet_dialog import FilletDialog
    return FilletDialog(win._scene,
                        win._selected_node_id if selected is None
                        else selected,
                        apply_route=win._apply_fillet, parent=win)


def _dialog_edge_names(dlg):
    from PySide6.QtCore import Qt
    return [dlg._edges.item(i).data(Qt.UserRole)
            for i in range(dlg._edges.count())]


def _click_edge(dlg, name):
    """The row CLICKED the way a hand clicks it (QTest at the row's
    own rect) — the selection is the list's own act."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    names = _dialog_edge_names(dlg)
    assert name in names, (name, names[:4])
    index = names.index(name)
    rect = dlg._edges.visualItemRect(dlg._edges.item(index))
    QTest.mouseClick(dlg._edges.viewport(), Qt.LeftButton, Qt.NoModifier,
                     rect.center())
    return dlg.selected_reference() == name


def _apply_dialog(dlg, radius):
    """The radius set and the Apply fired (the button's own slot)."""
    dlg._radius.setValue(radius)
    dlg._on_apply()


def _flush(app=None):
    """One event-loop pass — the deferred refreshes (the radius edit's
    panel echo) land on the next iteration by design."""
    from PySide6.QtWidgets import QApplication
    (app or QApplication.instance()).processEvents()


def _make_guide_box(win):
    """The guide geometry's box [-1,1]^3, built the honest in-app
    way: Create -> Box (the registry's defaults — extent 0.5), then
    the three extents edited to 1.0 through the panel's own editors
    (the test_ui_operations precedent: the param-edit path BEFORE the
    feature, never a scene-side shortcut). Returns the node id."""
    win._create_registered("box")
    box_id = win._scene.node_at(win._scene.node_count() - 1).id
    assert _select_row(win, box_id)
    for axis in ("extent_x", "extent_y", "extent_z"):
        editor = _panel_field(win, axis)
        assert editor is not None, axis
        editor.setValue(1.0)
    node = win._scene.get_node(box_id)
    assert node is not None and node.builder == "box"
    return box_id


def _blind_chain(win, offset_z=-0.5):
    """The m3 guide body ON THE WINDOW'S SCENE (the scene API setup,
    the m4 test's own recipe — the UI rides it): the sketch circle,
    the extrude, the box, the blind subtract. Returns the node ids
    (sketch, extrude, box, subtract) and the rim reference, taken from
    the operand's own query (never typed by hand)."""
    from CAD_0 import sdf as sdf_mod
    from CAD_0 import brep as brep_mod
    from CAD_0 import scene as scene_mod
    from CAD_0 import math as math_mod

    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    sk = win._scene.add_sketch_node("Blind_Sketch", chains, [])
    ex = win._scene.add_extrude_node("Blind_Extrude", sketch=sk,
                                     distance=2.0, direction=1)
    bx = win._scene.add_node("Blind_Box",
                             sdf_mod.box(math_mod.Vec3f(1.0, 1.0, 1.0)))
    node = win._scene.get_node(bx)
    node.builder = "box"
    node.set_params({"extent_x": 1.0, "extent_y": 1.0,
                     "extent_z": 1.0})
    sub = win._scene.add_operation_node("Blind_Subtract", target=bx,
                                        tool=ex, offset_z=offset_z)
    for nid in (sk, ex, bx):
        win._scene.set_visible(nid, False)
    win._rebuild_ui_from_scene()
    blind = win._scene.get_node(sub)
    assert blind.exact_brep is not None
    rim = None
    for i in range(brep_mod.edge_count(blind.exact_brep)):
        e = brep_mod.edge_query(blind.exact_brep, i)
        if "@hi" in e["name"] and "isect" in e["name"]:
            rim = e["name"]
            break
    # The grammar's own derivation, checked (the DAG ids are the node
    # ids — the morse-5 rule): never typed by hand.
    assert rim == "sub#{}:isect[box#{}:top \u00d7 ext#{}:lat(0)]@hi".format(
        sub, bx, ex)
    return sk, ex, bx, sub, rim


def _volume_at(radius):
    """The closed form's own value (the ADR's derivation — the
    classical form IS the test): (8 - 0.375*pi) - V_r at the given r,
    with a = 0.5."""
    a, r = 0.5, radius
    v_r = 2.0 * math.pi * (a * r * r * (1.0 - math.pi / 4.0) +
                           r ** 3 * (5.0 / 6.0 - math.pi / 4.0))
    return (8.0 - 0.375 * math.pi) - v_r


# ===========================================================================
# 1. THE SELECTION — the reference the kernel uses, character for
#    character, both guide cases
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_selection_resolves_the_reference(workbench_app):
    win = workbench_app
    assert win._has_kernel

    # ---- Guide case 1: LO SPIGOLO DEL BOX — the factory's own
    # vertical, reachable from the Create menu (the fitted primitive,
    # named under the node's DAG id by the bridge's feature-aware fit).
    box_id = _make_guide_box(win)
    _select_row(win, box_id)

    dlg = _open_fillet_dialog(win)
    names = _dialog_edge_names(dlg)
    expected = "box#{}:vertical2".format(box_id)
    # The list carries the kernel's own name — the row IS the
    # reference, character for character.
    assert expected in names
    # The reference the click resolves: the label shows it, the
    # dialog's own answer carries it — the same string the recipe
    # will store.
    assert _click_edge(dlg, expected)
    assert dlg._reference.text() == "Reference: {}".format(expected)
    assert dlg.selected_reference() == expected
    # The unnamed edges declared honestly, never hidden: the box's 12
    # edges are ALL named (the factory's ring) — the label stays empty.
    assert dlg._unnamed.text() == ""
    dlg.reject()

    # ---- Guide case 2: IL RIM CIECO — the u2 guide body's pierced
    # rim, the pinned string the contract wrote "so the fillet cannot
    # bend it" (ADR-0038 D3).
    sk, ex, bx, sub, rim = _blind_chain(win)
    _select_row(win, sub)
    dlg = _open_fillet_dialog(win)
    names = _dialog_edge_names(dlg)
    assert rim in names
    assert _click_edge(dlg, rim)
    assert dlg._reference.text() == "Reference: {}".format(rim)
    # The seam-wired pair taken WHOLE: one logical name in the list
    # (the morse-5 rule — the halves do not double the row).
    assert names.count(rim) == 1
    # The blind body's 17 edges are ALL named (the box's ring, the
    # two rim pairs, the seam) — the label stays empty. The honestly
    # UNNAMED declaration is exercised on the OTHER candidate: the
    # extrude's prism (the walk edges read honestly empty).
    assert dlg._unnamed.text() == ""
    for i in range(dlg._combo.count()):
        if dlg._combo.itemData(i) == ex:
            dlg._combo.setCurrentIndex(i)
            break
    assert "honestly unnamed" in dlg._unnamed.text()
    dlg.reject()


# ===========================================================================
# 2. THE APPLICATION — the golden's body from the dialog, the numbers
#    read from the kernel (additivity: every golden unmoved)
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_application_produces_the_golden_body(workbench_app):
    import CAD_0
    from CAD_0 import brep as brep_mod

    win = workbench_app

    # ---- The box class at r = 0.5: 10/15/7, chi 2, 7.5 + pi/8.
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, selected=box_id)
    expected = "box#{}:vertical2".format(box_id)
    assert _click_edge(dlg, expected)
    undo_before = win._command_stack.undo_count()
    _apply_dialog(dlg, 0.5)
    assert dlg.result() == dlg.DialogCode.Accepted
    assert win._command_stack.undo_count() == undo_before + 1

    fid = win._selected_node_id
    node = win._scene.get_node(fid)
    assert node.kind == "fillet"
    assert node.fillet_refusal == ""
    assert node.fillet_edge == expected
    assert node.fillet_radius == 0.5
    # THE KERNEL'S OWN ANSWERS, read back through the report module
    # (the single consumer — the UI never recomputes a number):
    data = CAD_0.report.body_report_data(node.fillet_brep)
    assert (data["V"], data["E"], data["F"], data["H"], data["chi"]) == \
        (10, 15, 7, 0, 2)
    assert data["all_exact"] is True
    assert data["volume"] == pytest.approx(7.5 + math.pi / 8.0, rel=1e-12)

    # The panel's rows carry the same numbers (the surface the user
    # reads) — and the born names beside the geometry.
    assert _row_value(win, "Counts (V/E/F/H)") == "10/15/7/0"
    assert _row_value(win, "\u03c7 (V\u2212E+F\u2212H)") == "2"
    volume_row = _row_value(win, "Volume")
    assert volume_row is not None and volume_row.endswith("(exact)")
    assert float(volume_row.split(" ")[0]) == pytest.approx(
        7.5 + math.pi / 8.0, rel=1e-9)
    born_face = _row_value(win, "Born face(s)")
    assert born_face == ("fil#{}:flank[box#{}:back \u00d7 "
                         "box#{}:right] [cylinder]").format(fid, box_id,
                                                            box_id)
    # The two tangent wires named (the level absent — the tangency
    # IS the level); the two quarter arcs honestly unnamed — they
    # read a DEFAULT shape_id (no structural match in the panel) and
    # live in the REPORT's edge table (the "-" rows, the name
    # column's own honesty — pinned below).
    born_edges = _row_value(win, "Born edge names")
    assert born_edges is not None
    assert ("fil#{}:isect[box#{}:back \u00d7 fil#{}:flank[box#{}:back "
            "\u00d7 box#{}:right]]").format(
                fid, box_id, fid, box_id, box_id) in born_edges
    assert ("fil#{}:isect[box#{}:right \u00d7 fil#{}:flank[box#{}:back "
            "\u00d7 box#{}:right]]").format(
                fid, box_id, fid, box_id, box_id) in born_edges
    # THE REPORT SAYS EVERYTHING (the m5 rung's clause): the Debug
    # report on the fillet node — the counts with the H term, the
    # flank's family in the face table, the NAME COLUMN with the
    # honestly unnamed "-" rows beside the named.
    report_text = win._body_report_text(node)
    assert report_text is not None
    assert "V = 10, E = 15, F = 7, H (inner loops) = 0" in report_text
    assert "fil#{}:flank[box#{}:back \u00d7 box#{}:right]".format(
        fid, box_id, box_id) in report_text
    # The flank's FAMILY in the face query (the table's own column).
    assert any("flank" in ln and "cylinder" in ln
               for ln in report_text.splitlines())
    edge_lines = [ln for ln in report_text.splitlines()
                  if ln.startswith("  ") and " | " in ln]
    assert any(" | - " in ln for ln in edge_lines)
    # The reference row: the string verbatim.
    assert _row_value(win, "Edge reference") == expected

    # ---- The rim class at r = 0.25: 14/20/9, H 1, the closed form.
    sk, ex, bx, sub, rim = _blind_chain(win)
    _select_row(win, sub)
    dlg = _open_fillet_dialog(win)
    assert _click_edge(dlg, rim)
    undo_before = win._command_stack.undo_count()
    _apply_dialog(dlg, 0.25)
    assert win._command_stack.undo_count() == undo_before + 1

    fid2 = win._selected_node_id
    node2 = win._scene.get_node(fid2)
    assert node2.fillet_refusal == ""
    assert node2.fillet_edge == rim
    data2 = CAD_0.report.body_report_data(node2.fillet_brep)
    assert (data2["V"], data2["E"], data2["F"], data2["H"],
            data2["chi"]) == (14, 20, 9, 1, 2)
    assert data2["all_exact"] is True
    assert data2["volume"] == pytest.approx(_volume_at(0.25), rel=1e-12)
    # The toroidal flank born, the family the query's own answer.
    born_face2 = _row_value(win, "Born face(s)")
    assert born_face2 == ("fil#{}:flank[box#{}:top \u00d7 "
                          "ext#{}:lat(0)] [torus]").format(fid2, bx, ex)
    # The tangent wires (the level absent) and the flank's own seam.
    born_edges2 = _row_value(win, "Born edge names")
    flank = "fil#{}:flank[box#{}:top \u00d7 ext#{}:lat(0)]".format(
        fid2, bx, ex)
    assert "fil#{}:isect[box#{}:top \u00d7 {}]".format(
        fid2, bx, flank) in born_edges2
    assert "fil#{}:isect[ext#{}:lat(0) \u00d7 {}]".format(
        fid2, ex, flank) in born_edges2
    assert "fil#{}:seam[{}]".format(fid2, flank) in born_edges2

    # THE ADDITIVITY: the operands' own goldens unmoved (the blind
    # body's counts read through the same report surface).
    blind = win._scene.get_node(sub)
    data_blind = CAD_0.report.body_report_data(blind.exact_brep)
    assert (data_blind["V"], data_blind["E"], data_blind["F"],
            data_blind["H"]) == (12, 17, 8, 1)
    assert data_blind["volume"] == pytest.approx(
        8.0 - 0.375 * math.pi, rel=1e-12)


# ===========================================================================
# 3. THE REFUSALS — in words, with the bound carried, in the dialog
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_refusals_in_words_with_the_bound(workbench_app):
    win = workbench_app

    # ---- fillet_radius_exceeds_face: the box class, r = 2.5 against
    # the reach 2 — the refusal names the member, the radius, the
    # bound and the epsilon (every number in the message).
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, selected=box_id)
    assert _click_edge(dlg, "box#{}:vertical2".format(box_id))
    nodes_before = win._scene.node_count()
    undo_before = win._command_stack.undo_count()
    _apply_dialog(dlg, 2.5)
    msg = dlg._status.text()
    # The dialog STAYS OPEN (the user adjusts the radius) and nothing
    # was written: no node, no undo entry.
    assert dlg.result() != dlg.DialogCode.Accepted
    assert win._scene.node_count() == nodes_before
    assert win._command_stack.undo_count() == undo_before
    assert "fillet_radius_exceeds_face" in msg
    assert "2.500000" in msg
    assert "2.000000" in msg
    dlg.reject()

    # ---- fillet_exits_annulus: the rim class, r = 0.7 against the
    # reach 0.5 (the pierced face's own bound).
    sk, ex, bx, sub, rim = _blind_chain(win)
    _select_row(win, sub)
    dlg = _open_fillet_dialog(win)
    assert _click_edge(dlg, rim)
    _apply_dialog(dlg, 0.7)
    msg = dlg._status.text()
    assert dlg.result() != dlg.DialogCode.Accepted
    assert "fillet_exits_annulus" in msg
    assert "0.700000" in msg
    assert "0.500000" in msg
    dlg.reject()

    # ---- fillet_eats_wall: the SHORT-wall blind body (the floor at
    # 0.75 — the wall's height 0.25), r = 0.3 meets only the wall's
    # band.
    sk2, ex2, bx2, sub2, rim2 = _blind_chain(win, offset_z=0.75)
    _select_row(win, sub2)
    dlg = _open_fillet_dialog(win)
    assert _click_edge(dlg, rim2)
    _apply_dialog(dlg, 0.3)
    msg = dlg._status.text()
    assert dlg.result() != dlg.DialogCode.Accepted
    assert "fillet_eats_wall" in msg
    assert "0.300000" in msg
    assert "0.250000" in msg
    dlg.reject()
    # No fillet node exists: every application refused.
    kinds = [win._scene.node_at(i).kind
             for i in range(win._scene.node_count())]
    assert "fillet" not in kinds


# ===========================================================================
# 4. THE EDIT — class 1 silent from the panel, ONE undo entry
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_radius_edit_class1_silent(workbench_app):
    import CAD_0

    win = workbench_app
    sk, ex, bx, sub, rim = _blind_chain(win)
    _select_row(win, sub)
    dlg = _open_fillet_dialog(win)
    assert _click_edge(dlg, rim)
    _apply_dialog(dlg, 0.25)
    fid = win._selected_node_id
    node = win._scene.get_node(fid)

    # The panel's own radius editor: the class-1 SILENT edit — the
    # SAME road the binding travels (set_fillet_radius, the find, ONE
    # scene mutation), never a UI-side rebuild.
    editor = _panel_field(win, "radius")
    assert editor is not None
    assert editor.value() == 0.25
    panel_label_before = win._property_panel.current_label()
    selected_before = win._selected_node_id
    counts_before = _row_value(win, "Counts (V/E/F/H)")
    undo_before = win._command_stack.undo_count()

    editor.setValue(0.125)
    _flush()

    # ONE undo entry — and the counts FIXED (the class-1 contract:
    # the same topology for every in-band r).
    assert win._command_stack.undo_count() == undo_before + 1
    assert win._selected_node_id == selected_before
    assert win._property_panel.current_label() == panel_label_before
    assert _row_value(win, "Counts (V/E/F/H)") == counts_before
    assert _row_value(win, "Counts (V/E/F/H)") == "14/20/9/1"
    # The reference row unchanged — the edit never re-picks.
    assert _row_value(win, "Edge reference") == rim
    # The volume row: the closed form's own value at the new r (the
    # report module's answer — never recomputed here beyond the
    # classical form, which IS the test). The deferred panel echo has
    # landed (one event-loop pass): the read-only rows re-routed from
    # the recipe — the volume row the NEW radius's own answer.
    volume_row = _row_value(win, "Volume")
    assert volume_row is not None
    assert float(volume_row.split(" ")[0]) == pytest.approx(
        _volume_at(0.125), rel=1e-9)
    # The recipe's own field moved (the scene's answer).
    node = win._scene.get_node(fid)
    assert node.fillet_radius == 0.125
    assert node.fillet_refusal == ""
    data = CAD_0.report.body_report_data(node.fillet_brep)
    assert (data["V"], data["E"], data["F"], data["H"]) == (14, 20, 9, 1)
    assert data["volume"] == pytest.approx(_volume_at(0.125), rel=1e-12)

    # The status bar spoke the re-run's answer (reference_resolved).
    # (The message is transient by design; the recipe and the panel
    # are the durable state — pinned above.)

    # UNDO restores the old radius (ONE entry back): the counts fixed,
    # the volume the closed form at 0.25 again.
    win.on_undo()
    node = win._scene.get_node(fid)
    assert node.fillet_radius == 0.25
    data = CAD_0.report.body_report_data(node.fillet_brep)
    assert data["volume"] == pytest.approx(_volume_at(0.25), rel=1e-12)
    # REDO forward again — bit-identical numbers.
    win.on_redo()
    node = win._scene.get_node(fid)
    assert node.fillet_radius == 0.125
    data = CAD_0.report.body_report_data(node.fillet_brep)
    assert data["volume"] == pytest.approx(_volume_at(0.125), rel=1e-12)


# ===========================================================================
# 5. THE DEATH — the class-4 payload shown, the body kept, no crash
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_death_of_the_reference_class4(workbench_app):
    import CAD_0
    from CAD_0 import scene as scene_mod

    win = workbench_app
    sk, ex, bx, sub, rim = _blind_chain(win)
    _select_row(win, sub)
    dlg = _open_fillet_dialog(win)
    assert _click_edge(dlg, rim)
    _apply_dialog(dlg, 0.25)
    fid = win._selected_node_id

    # THE RE-AUTHOR (the quartet's own scenario): the tool's circle
    # REPLACED by the square chain — through the scene's entity doors
    # (the canvas's own write family; v1 has no delete gesture, the
    # API route is the established test driver).
    win._scene.sketch_set_entity(sk, 0, 0, scene_mod.sketch_segment(
        (-0.5, -0.5), (0.5, -0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (0.5, -0.5), (0.5, 0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (0.5, 0.5), (-0.5, 0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (-0.5, 0.5), (-0.5, -0.5), True))

    # THE PLACEMENT EDIT from the panel (the user's own route): the
    # tool moved to span the box — the offset edit runs the DAG pass
    # and the operand re-materializes (the square THROUGH hole), the
    # rim dying with the authored circle. ONE rebuild, the captured
    # old tavola serving the class-4 answer.
    assert _select_row(win, sub)
    editor = _panel_field(win, "offset_z")
    assert editor is not None
    assert editor.value() == -0.5
    editor.setValue(-1.0)

    # The operand's new state: the square through hole (the quartet's
    # own counts class), the fillet's reference DEAD.
    sub_node = win._scene.get_node(sub)
    assert sub_node.hybrid_refusal == ""
    fillet_node = win._scene.get_node(fid)
    why = fillet_node.fillet_refusal
    # THE CLASS-4 PAYLOAD, shown: WHAT died (the row, named), WHY
    # (the authored entity was replaced), the remedy (reselect, or
    # revert) — the resolver's own words, never a mute "not found".
    assert why.startswith("reference_vanished")
    assert rim in why
    assert "the authored entity was replaced" in why
    assert "reselect" in why
    assert "revert" in why
    # The last good body KEPT (the visible state): the counts still
    # the last good re-run's — never a mute body.
    data = CAD_0.report.body_report_data(fillet_node.fillet_brep)
    assert (data["V"], data["E"], data["F"], data["H"]) == (14, 20, 9, 1)

    # The tree row shows the visible refusal state (FIL!).
    assert _row_type(win, fid) == "FIL!"
    # The panel (on selection) carries the payload and the kept
    # numbers — no crash, no empty panel.
    assert _select_row(win, fid)
    refusal_row = _row_value(win, "REFUSAL")
    assert refusal_row is not None
    assert refusal_row.startswith("reference_vanished")
    assert _row_value(win, "Counts (V/E/F/H)") == "14/20/9/1"
    assert _row_value(win, "Edge reference") == rim

    # The operand's own rows stay honest (the square body materialized).
    assert _select_row(win, sub)
    assert _row_type(win, sub) == "CSG"


# ===========================================================================
# 6. THE DETERMINISM — the same sequence twice, the same scene
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_same_sequence_twice_the_same_scene(workbench_app,
                                                monkeypatch):
    import CAD_0
    from CAD_0 import scene as scene_mod

    win = workbench_app

    def the_sequence():
        sk, ex, bx, sub, rim = _blind_chain(win)
        _select_row(win, sub)
        dlg = _open_fillet_dialog(win)
        assert _click_edge(dlg, rim)
        _apply_dialog(dlg, 0.25)
        fid = win._selected_node_id
        editor = _panel_field(win, "radius")
        editor.setValue(0.125)
        node = win._scene.get_node(fid)
        data = CAD_0.report.body_report_data(node.fillet_brep)
        return {
            "ids": [win._scene.node_at(i).id
                    for i in range(win._scene.node_count())],
            "kinds": [win._scene.node_at(i).kind
                      for i in range(win._scene.node_count())],
            "names": [win._scene.node_at(i).name
                      for i in range(win._scene.node_count())],
            "fillet_id": fid,
            "fillet_name": node.name,
            "reference": node.fillet_edge,
            "radius": node.fillet_radius,
            "counts": (data["V"], data["E"], data["F"], data["H"],
                       data["chi"]),
            "volume": data["volume"],
            "born_face": _row_value(win, "Born face(s)"),
            "tree_types": [_row_type(win, i)
                           for i in [sk, ex, bx, sub, fid]],
            "undo": win._command_stack.undo_count(),
        }

    # The sequence ONCE, on the window's scene.
    first = the_sequence()
    assert first["fillet_id"] == 5
    assert first["ids"] == [1, 2, 3, 4, 5]
    assert first["kinds"] == ["sketch", "sdf", "sdf", "sdf", "fillet"]
    assert first["tree_types"] == ["Sketch", "FEAT", "SDF", "CSG",
                                   "FIL"]
    assert first["undo"] == 2  # the application + the edit

    # The undo/redo round trip of the whole sequence: back to the
    # chain, forward again — the numbers bit-identical.
    win.on_undo()   # the radius edit
    win.on_undo()   # the application
    assert [win._scene.node_at(i).kind
            for i in range(win._scene.node_count())] == \
        ["sketch", "sdf", "sdf", "sdf"]
    win.on_redo()   # the application returns (from the recipe)
    win.on_redo()   # the radius edit returns
    node = win._scene.get_node(first["fillet_id"])
    data = CAD_0.report.body_report_data(node.fillet_brep)
    assert (data["V"], data["E"], data["F"], data["H"]) == \
        first["counts"][:4]
    assert data["volume"] == first["volume"]
    assert node.fillet_radius == first["radius"]
    assert node.fillet_edge == first["reference"]

    # The sequence AGAIN on a FRESH window: the same ids, the same
    # names, the same numbers — no hidden state, no random id (the
    # ids come from the scene's monotonic counter, the same sequence
    # answers the same scene).
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow
    win2 = MainWindow()
    win2._current_path = None
    try:
        # (the same gestures, the same order — the second window)
        sk, ex, bx, sub, rim = _blind_chain(win2)
        _select_row(win2, sub)
        dlg = _open_fillet_dialog(win2)
        assert _click_edge(dlg, rim)
        _apply_dialog(dlg, 0.25)
        fid2 = win2._selected_node_id
        editor = _panel_field(win2, "radius")
        editor.setValue(0.125)
        node2 = win2._scene.get_node(fid2)
        data2 = CAD_0.report.body_report_data(node2.fillet_brep)
        second = {
            "ids": [win2._scene.node_at(i).id
                    for i in range(win2._scene.node_count())],
            "kinds": [win2._scene.node_at(i).kind
                      for i in range(win2._scene.node_count())],
            "names": [win2._scene.node_at(i).name
                      for i in range(win2._scene.node_count())],
            "fillet_id": fid2,
            "fillet_name": node2.name,
            "reference": node2.fillet_edge,
            "radius": node2.fillet_radius,
            "counts": (data2["V"], data2["E"], data2["F"], data2["H"],
                       data2["chi"]),
            "volume": data2["volume"],
            "born_face": _row_value(win2, "Born face(s)"),
            "tree_types": [_row_type(win2, i)
                           for i in [sk, ex, bx, sub, fid2]],
            "undo": win2._command_stack.undo_count(),
        }
        assert second == first
    finally:
        win2.close()
        win2.deleteLater()
        gc.collect()
