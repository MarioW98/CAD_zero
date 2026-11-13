# bindings/python/tests/test_ui_fillet_chain_m6ter.py
#
# G2-u3 m6-correttiva-ter — IL RAGGIUNTO DEL FILLET E I DUE CORPI
# SOVRAPPOSTI: the third click-test's TWO new defects, each with its
# naive counterproof (RED on the un-cured tip, the house rule 3):
#
#   DIFETTO A — "sembra funzionare solo sulle facce 'vertical'": the
#   picker offered ALL 12 named edges as if equal; the class-1 wall
#   (the derivation tied to the vertical axis — the CylinderZ flank,
#   the m2 frame writer's own family) spoke ONLY at the Apply, as a
#   refusal the maintainer had to fire to see. The cure: the wall
#   DECLARED at the pick surface — every row carries the kernel's own
#   verdict (the admission dry run, the same path the Apply walks),
#   the ring edges say WHY they are outside (the family named), the
#   verticals say IN CLASS.
#
#   DIFETTO B — "come se avessi 2 cubi uno dentro l'altro": the
#   second fillet consumed the HIDDEN BOX (the discovery's first
#   match in document order) and generated a SIBLING body — both
#   visible, the two overlapping cubes. The cure: the parametric
#   chain's semantics — the second fillet consumes the CURRENT STATE
#   (the previous fillet's body) when the reference survives
#   (ADR-0036 D3: MATCH through the kept stamps); a reference that
#   died (VANISHED) refuses BY NAME with the killer and the cause;
#   the hiding pattern rides the chain (one body in the viewport,
#   always).
from __future__ import annotations

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


# --- the driver (the workbench suite's own gestures) ----------------------

def _panel_field(win, name):
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


def _open_fillet_dialog(win, selected=None):
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
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    names = _dialog_edge_names(dlg)
    assert name in names, (name, names[:4])
    index = names.index(name)
    rect = dlg._edges.visualItemRect(dlg._edges.item(index))
    QTest.mouseClick(dlg._edges.viewport(), Qt.LeftButton, Qt.NoModifier,
                     rect.center())
    return dlg.selected_reference() == name


def _apply_dialog(dlg, radius):
    dlg._radius.setValue(radius)
    dlg._on_apply()


def _make_guide_box(win):
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


def _visible_body_ids(win):
    """The scene's own answer to 'what does the viewport draw': the
    VISIBLE nodes that carry a display body (the fillet nodes' own
    materialized body, the sdf nodes' — the update_meshes dispatch)."""
    out = []
    for i in range(win._scene.node_count()):
        nd = win._scene.node_at(i)
        if nd is None or not nd.visible or nd.kind == "sketch":
            continue
        if nd.kind == "fillet":
            if getattr(nd, "fillet_brep", None) is not None:
                out.append(nd.id)
        elif nd.kind == "sdf":
            out.append(nd.id)
    return out


def _two_fillet_chain(win, radius=0.5):
    """box -> fillet#1 (vertical2) -> fillet#2 (vertical0, the
    SURVIVING edge on the current state) — through the dialog wiring
    both times (the user's own gestures). Returns (box, f1, f2)."""
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, box_id)
    assert _click_edge(dlg, "box#{}:vertical2".format(box_id))
    _apply_dialog(dlg, radius)
    f1 = win._selected_node_id
    dlg2 = _open_fillet_dialog(win, f1)
    assert _click_edge(dlg2, "box#{}:vertical0".format(box_id))
    _apply_dialog(dlg2, radius)
    f2 = win._selected_node_id
    return box_id, f1, f2


# ===========================================================================
# DIFETTO A — the twelve edges, each with its verdict pinned
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_twelve_edges_declared_at_the_pick(workbench_app):
    """The enumeration the maintainer's hands ran, with the
    expectations pinned: the 4 verticals IN CLASS, the 8 ring edges
    OUTSIDE with the family named — SAID ON THE ROW, never silent."""
    from PySide6.QtCore import Qt
    win = workbench_app
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, box_id)

    names = _dialog_edge_names(dlg)
    assert len(names) == 12
    assert sorted(n.split(":")[1] for n in names) == sorted(
        ["bottom{}".format(i) for i in range(4)] +
        ["top{}".format(i) for i in range(4)] +
        ["vertical{}".format(i) for i in range(4)])

    in_class, outside = [], []
    for i in range(dlg._edges.count()):
        item = dlg._edges.item(i)
        name = item.data(Qt.UserRole)
        text = item.text()
        if ":vertical" in name:
            assert "IN CLASS" in text, (name, text)
            in_class.append(name)
        else:
            assert "OUT" in text, (name, text)
            # the family NAMED — the kernel's own words on the row
            assert "not vertical" in text, (name, text)
            outside.append(name)
    assert len(in_class) == 4 and len(outside) == 8


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_admission_door_says_the_wall(workbench_app):
    """The dry run the rows read: the Apply's own path (the
    discovery, the dispatch, the kernel's admission) WITHOUT the
    install — the kernel's words verbatim, the band's bound carried."""
    win = workbench_app
    box_id = _make_guide_box(win)
    v2 = "box#{}:vertical2".format(box_id)

    a = win._scene.fillet_admission(v2, 0.5)
    assert a["admitted"] is True
    assert a["reason"] == ""
    assert a["operand_id"] == box_id

    a = win._scene.fillet_admission(
        "box#{}:bottom0".format(box_id), 0.5)
    assert a["admitted"] is False
    assert "not vertical" in a["reason"]

    # the band speaks too (the typed bound inside the words)
    a = win._scene.fillet_admission(v2, 5.0)
    assert a["admitted"] is False
    assert "fillet_radius_exceeds_face" in a["reason"]

    # a dead name (no body carries it) names the closure violation
    a = win._scene.fillet_admission("box#99:vertical2", 0.5)
    assert a["admitted"] is False
    assert a["reason"]


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_admission_door_rides_the_chain(workbench_app):
    """The door after a fillet exists: the surviving reference
    resolves on the CURRENT STATE (the fillet node, named); the dead
    one refuses with the killer and the cause — the chain declared
    BEFORE any Apply."""
    win = workbench_app
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, box_id)
    assert _click_edge(dlg, "box#{}:vertical2".format(box_id))
    _apply_dialog(dlg, 0.5)
    f1 = win._selected_node_id

    a = win._scene.fillet_admission(
        "box#{}:vertical0".format(box_id), 0.5)
    assert a["admitted"] is True
    assert a["operand_id"] == f1, (
        "the surviving reference must resolve on the CURRENT STATE "
        "(the fillet node), not the superseded box")

    a = win._scene.fillet_admission(
        "box#{}:vertical2".format(box_id), 0.5)
    assert a["admitted"] is False
    assert "died" in a["reason"] or "VANISHED" in a["reason"]
    # the killer NAMED (the fillet that consumed the edge)
    assert "Fillet" in a["reason"]


# ===========================================================================
# DIFETTO B — the chain: one body in the viewport, always
# ===========================================================================

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_chain_consumes_the_current_state(workbench_app):
    """The maintainer's act: fillet, then fillet again. The second
    consumes the CURRENT STATE (fillet#1's body): the counts are the
    CHAIN's (12/18/8, the volume 7 + pi/4 — two wedges), and the
    viewport carries ONE body — never the two overlapping cubes."""
    from CAD_0 import report as rep
    win = workbench_app
    box_id, f1, f2 = _two_fillet_chain(win)

    node1 = win._scene.get_node(f1)
    node2 = win._scene.get_node(f2)

    # THE CHAIN: fillet#2's operand IS fillet#1 (the current state)
    assert node2.fillet_operand_id == f1
    assert node1.fillet_operand_id == box_id

    # THE BODY: the chain's own counts and volume (both fillets at
    # r = 0.5 on the [-1,1]^3 box: 8 - 2 * (2 * 0.25 * (1 - pi/4)))
    d = rep.body_report_data(node2.fillet_brep)
    assert (d["V"], d["E"], d["F"], d["H"]) == (12, 18, 8, 0)
    assert d["chi"] == 2 and d["all_exact"]
    expected = 8.0 - 2.0 * (2.0 * 0.25 * (1.0 - math.pi / 4.0))
    assert abs(float(d["volume"]) - expected) < 1e-12

    # ONE BODY in the viewport: the box hidden (fillet#1's Apply),
    # fillet#1 hidden (fillet#2's Apply — the pattern rides the
    # chain), fillet#2 alone visible.
    assert win._scene.get_node(box_id).visible is False
    assert win._scene.get_node(f1).visible is False
    assert win._scene.get_node(f2).visible is True
    assert _visible_body_ids(win) == [f2]
    # the frame's buffers: the one body's triangles alone
    assert win._viewport._node_ranges.get(f2) is not None
    assert win._viewport._node_ranges.get(f1) is None
    assert win._viewport._node_ranges.get(box_id) is None
    cm = node2.cached_mesh
    assert cm is not None and cm.triangle_count() > 0
    # the frame's mirror (PERF 1: one entry per visible feature): the
    # one body's triangles alone — the whole-scene concatenation dead
    entries = {e.node_id: e for e in win._viewport._draw_nodes}
    assert list(entries) == [f2]
    assert entries[f2].triangle_count == cm.triangle_count()

    # THE UNDO returns fillet#1's body (the box stays hidden —
    # fillet#1's own Apply still owns it); the redo hides again.
    undo_before = win._command_stack.undo_count()
    win.on_undo()
    assert win._command_stack.undo_count() == undo_before - 1
    assert win._scene.get_node(f1).visible is True
    assert _visible_body_ids(win) == [f1]
    win.on_redo()
    assert _visible_body_ids(win) == [f2]


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_dead_edge_refuses_by_name(workbench_app):
    """The VANISHED reference (the edge the first fillet consumed):
    refused BY NAME — the killer, the cause, the remedy — never the
    silent sibling the un-cured tip built on the hidden box."""
    win = workbench_app
    box_id = _make_guide_box(win)
    dlg = _open_fillet_dialog(win, box_id)
    assert _click_edge(dlg, "box#{}:vertical2".format(box_id))
    _apply_dialog(dlg, 0.5)
    f1 = win._selected_node_id
    node_count = win._scene.node_count()

    # The dead reference, fired the way a hand fires it: the BOX's
    # own list still offers the name (the superseded operand's honest
    # query — its own body carries it) — the row itself DECLARES the
    # death, and the Apply refuses, the dialog STAYS OPEN, the words
    # carry the killer and the cause.
    dlg2 = _open_fillet_dialog(win, f1)
    # The combo's honest surface: switch to the superseded operand
    # (the box) — its list is the OLD state's, the dead name still in.
    for i in range(dlg2._combo.count()):
        if dlg2._combo.itemData(i) == box_id:
            dlg2._combo.setCurrentIndex(i)
            break
    row_text = None
    from PySide6.QtCore import Qt
    for i in range(dlg2._edges.count()):
        if dlg2._edges.item(i).data(Qt.UserRole) == \
                "box#{}:vertical2".format(box_id):
            row_text = dlg2._edges.item(i).text()
            break
    assert row_text is not None
    # THE WALL SAID ON THE ROW (defect A's declaration, one link
    # over): the dead edge's row carries the VANISHED verdict.
    assert "OUT" in row_text
    assert "VANISHED" in row_text or "died" in row_text
    assert _click_edge(dlg2, "box#{}:vertical2".format(box_id))
    _apply_dialog(dlg2, 0.25)
    # the dialog STAYS OPEN (the refusal is the visible state) and
    # nothing was written: no node, no selection change.
    assert dlg2.result() != dlg2.DialogCode.Accepted
    status = dlg2._status.text()
    assert "died" in status or "VANISHED" in status
    assert "Fillet" in status
    # nothing was created
    assert win._scene.node_count() == node_count
    assert win._selected_node_id == f1


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_upstream_radius_edit_reruns_the_chain(workbench_app):
    """The chain's DAG discipline: an edit on fillet#1's radius
    re-runs the downstream fillet#2 — its body follows the new
    operand (the counts fixed, the volume the closed form at the new
    radius), never silently stale."""
    from CAD_0 import report as rep
    win = workbench_app
    box_id, f1, f2 = _two_fillet_chain(win)

    # fillet#1's radius edited through the panel's own editor
    assert _select_row(win, f1)
    editor = _panel_field(win, "radius")
    assert editor is not None
    editor.setValue(0.25)

    d = rep.body_report_data(win._scene.get_node(f2).fillet_brep)
    assert (d["V"], d["E"], d["F"]) == (12, 18, 8)
    # r1 = 0.25, r2 = 0.5: 8 - 2*(0.25^2 + 0.5^2)*(1 - pi/4)*... —
    # per fillet: length 2 * r^2 * (1 - pi/4)
    w1 = 2.0 * 0.25 * 0.25 * (1.0 - math.pi / 4.0)
    w2 = 2.0 * 0.5 * 0.5 * (1.0 - math.pi / 4.0)
    expected = 8.0 - w1 - w2
    assert abs(float(d["volume"]) - expected) < 1e-12
    # the chain's hiding unmoved by the edit
    assert _visible_body_ids(win) == [f2]


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_report_declares_the_chain(workbench_app):
    """The panel's report says WHAT the fillet consumes (the m5
    report's own rows, the chain's declaration): the 'Consumes' row
    names the operand the discovery answered — the current state for
    the chain's second link, the box for the first."""
    win = workbench_app
    box_id, f1, f2 = _two_fillet_chain(win)

    assert _select_row(win, f2)
    consumes = _row_value(win, "Consumes")
    assert consumes is not None
    assert win._scene.get_node(f1).name in consumes
    assert "current state" in consumes

    assert _select_row(win, f1)
    consumes1 = _row_value(win, "Consumes")
    assert consumes1 is not None
    assert win._scene.get_node(box_id).name in consumes1


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_chain_round_trips_the_file(workbench_app):
    """The chain in the native format (the recipe-is-the-truth
    discipline): the loader's pass re-discovers each link through
    the chain-aware find — the same counts, the same volume, the
    hiding flags restored (one body in the viewport after the
    reload, exactly as before it)."""
    import tempfile
    import os
    import CAD_0
    from CAD_0 import report as rep
    win = workbench_app
    box_id, f1, f2 = _two_fillet_chain(win)
    d = rep.body_report_data(win._scene.get_node(f2).fillet_brep)

    path = os.path.join(tempfile.mkdtemp(), "chain.CAD_0")
    CAD_0.io.save_native(path, win._scene)
    from CAD_0 import scene as scene_mod
    sc2 = scene_mod.Scene()
    CAD_0.io.load_native(path, sc2)

    n2 = sc2.get_node(f2)
    assert n2 is not None and n2.kind == "fillet"
    d2 = rep.body_report_data(n2.fillet_brep)
    assert (d2["V"], d2["E"], d2["F"], d2["chi"]) == \
        (d["V"], d["E"], d["F"], d["chi"])
    assert float(d2["volume"]) == float(d["volume"])
    assert n2.fillet_operand_id == f1
    # the hiding flags round-trip: one body visible, the same one
    assert sc2.get_node(box_id).visible is False
    assert sc2.get_node(f1).visible is False
    assert sc2.get_node(f2).visible is True
