"""G1 unit 6 — the UI smoke test (pattern D6, offscreen).

Drives the REAL MainWindow (offscreen QPA, the established driver
pattern) through the report's app-level contract (ADR-0032 D8 +
D10.6, the order's AREA 4):

  * the Debug report presents the fields for the GUIDE body — the
    counts with the H term in the chi label, the genus, the mass
    properties with the readable inertia form and the all_exact flag,
    the per-kind provenance tallies;
  * the property panel shows the read-only mass rows on selection
    change (the subtract's exact numbers; the box's fitted-exact
    numbers; the sketch has none);
  * the honesty of the pattern, STRUCTURALLY: no read-only row can
    emit parameter_changed — a spy records zero emissions across
    every selection, and the mass rows carry no editor (the panel's
    _editors registry holds only the editable parameters);
  * the refused-exact body (the v0.3-style subtract) says
    approximated with its own polyhedral numbers — never an exact in
    disguise — and its report shows the extraction path with the
    min |f| margin.

Requires the compiled extension and PySide6; runs under
QT_QPA_PLATFORM=offscreen with the EGL software-GL workaround.
"""

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
def main_window(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # No modal dialogs in headless runs (the navigation fixture's
    # documented pattern).
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
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
    """The gesture (the use-case test's own): press at the centre,
    release on the rim — OUTER by default."""
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    canvas.set_tool("circle")
    QTest.mousePress(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*c))
    QTest.mouseMove(canvas, QPoint(*rim))
    QTest.mouseRelease(canvas, Qt.LeftButton, Qt.NoModifier, QPoint(*rim))


def _tree_ids(win):
    from PySide6.QtCore import Qt
    ids = []
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        ids.append(it.data(0, Qt.UserRole))
    return ids


def _select(win, node_id) -> bool:
    from PySide6.QtCore import Qt
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.data(0, Qt.UserRole) == node_id:
            win._feature_tree.setCurrentItem(it)
            return True
    return False


def _panel_rows(win):
    """(label, text, is_editor) per row — the read-only rows are
    QLabels, the editable ones are editors (spinboxes / line edits)."""
    from PySide6.QtWidgets import QFormLayout, QLabel
    rows = []
    form = win._property_panel._form
    for r in range(form.rowCount()):
        item = form.itemAt(r, QFormLayout.ItemRole.FieldRole)
        if item is None:
            continue
        w = item.widget()
        if w is None:
            continue
        lbl = form.labelForField(w)
        rows.append((lbl.text() if lbl else "?", w.text()
                     if isinstance(w, QLabel) else None,
                     not isinstance(w, QLabel)))
    return rows


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_report_presents_the_guide_body_fields(main_window):
    """AREA 4, smoke 1: the report presents the fields for the guide
    body — the grown Debug report (mass + provenance beside chi with
    the H term and genus, as always)."""
    from PySide6.QtCore import Qt
    from CAD_0 import math as _math
    import CAD_0

    win = main_window
    assert win._has_kernel

    # The guide chain through the app's own core methods: box
    # (extent 1.0 -> [-1,1]^3), sketch circle r=0.25 centred,
    # extrude 3.0 +Z, subtract (0, 0, -1.5).
    win._create_registered("box")
    box_id = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.text(1) == "SDF":
            box_id = it.data(0, Qt.UserRole)
    assert box_id is not None
    assert _select(win, box_id)
    for axis in ("extent_x", "extent_y", "extent_z"):
        win._on_parameter_changed(axis, 1.0)

    win.on_create_sketch()
    sketch_id = max(win._sketch_views)
    canvas = win._sketch_views[sketch_id]
    canvas.resize(800, 600)
    canvas.show()
    canvas._center = (0.0, 0.0)
    canvas._scale = 400.0  # 100 px = 0.25 world
    _draw_circle(canvas, (400, 300), (500, 300))

    prism_id = win._do_extrude(sketch_id, 3.0, 1)
    assert prism_id is not None
    hole_id = win._do_subtract(box_id, prism_id, 0.0, 0.0, -1.5)
    assert hole_id is not None

    node = win._scene.get_node(hole_id)
    assert node.exact_brep is not None  # the exact chain materialized

    text = win._body_report_text(node)
    assert text is not None
    # The topology block: the counts and the H term IN the label (the
    # arithmetic beside it for the hand-check).
    assert "V = 12, E = 17, F = 7, H (inner loops) = 2" in text
    assert "\u03c7 = V - E + F - H = 12 - 17 + 7 - 2 = 0" in text
    assert "Genus = 1" in text
    assert "Closed orientable (is_closed_orientable): yes" in text
    # The mass block: volume, centroid, the readable inertia
    # (diagonal + products), the visible exact flag.
    assert "Mass properties (density 1, divergence theorem): " \
           "exact (all_exact = True)" in text
    assert "Volume = 7.60730091" in text
    assert "Centroid = (" in text
    assert "diagonal  Ixx =" in text
    assert "products  Ixy =" in text
    # The provenance block: n exact / m approximated per kind.
    assert "Provenance (Layer 2): 36 subshapes exact / " \
           "0 approximated, max tolerance 0" in text
    assert "vertices  12 exact / 0 approximated" in text
    assert "edges     17 exact / 0 approximated" in text
    assert "faces     7 exact / 0 approximated" in text
    # The path: the node's OWN exact materialization (never a
    # re-extraction of the SDF twin).
    assert "Path: exact materialization (HybridBody" in text


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_panel_shows_readonly_numbers_on_selection_change(main_window):
    """AREA 4, smoke 2 + 3: the panel shows the read-only numbers at
    every selection change — and NO read-only field can emit
    parameter_changed (the honesty of the pattern, tested both
    behaviourally and structurally)."""
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QLabel, QTreeWidgetItem
    import CAD_0

    win = main_window
    assert win._has_kernel

    # The same guide chain (the previous test's construction).
    win._create_registered("box")
    box_id = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.text(1) == "SDF":
            box_id = it.data(0, Qt.UserRole)
    assert _select(win, box_id)
    for axis in ("extent_x", "extent_y", "extent_z"):
        win._on_parameter_changed(axis, 1.0)

    win.on_create_sketch()
    sketch_id = max(win._sketch_views)
    canvas = win._sketch_views[sketch_id]
    canvas.resize(800, 600)
    canvas.show()
    canvas._center = (0.0, 0.0)
    canvas._scale = 400.0
    _draw_circle(canvas, (400, 300), (500, 300))

    prism_id = win._do_extrude(sketch_id, 3.0, 1)
    hole_id = win._do_subtract(box_id, prism_id, 0.0, 0.0, -1.5)

    # The spy: every emission is a failure of the pattern.
    emitted = []
    win._property_panel.parameter_changed.connect(
        lambda name, value: emitted.append((name, value)))

    # The SUBTRACT: the editable offsets, the operand summary, and
    # the read-only mass rows (exact numbers — the report's own).
    assert _select(win, hole_id)
    rows = _panel_rows(win)
    by_label = {label: text for label, text, _ in rows}
    assert by_label.get("Volume", "").startswith("7.6073")
    assert by_label.get("Mass provenance") == "exact"
    assert "Inertia (Ixx, Iyy, Izz)" in by_label
    assert "Centroid" in by_label
    # The mass rows are READ-ONLY (QLabels, not editors) — the
    # structural honesty: a field that cannot edit cannot write.
    for label, _text, is_editor in rows:
        if label in ("Volume", "Centroid", "Inertia (Ixx, Iyy, Izz)",
                     "Mass provenance"):
            assert not is_editor, label
    # ...and the panel's editor registry holds only the offsets: no
    # mass row is an editor, so none can emit parameter_changed.
    for name in win._property_panel._editors:
        assert name in ("offset_x", "offset_y", "offset_z"), name

    # The BOX (no exact_brep): the dispatching fit's exact numbers.
    assert _select(win, box_id)
    by_label = {label: text for label, text, _ in _panel_rows(win)}
    assert by_label.get("Volume") == "8"
    assert by_label.get("Mass provenance") == "exact"

    # The PRISM (the extrude's own exact materialization).
    assert _select(win, prism_id)
    by_label = {label: text for label, text, _ in _panel_rows(win)}
    assert by_label.get("Volume", "").startswith("0.5890")  # pi r^2 h
    assert by_label.get("Mass provenance") == "exact"

    # The SKETCH: no body, no mass rows.
    assert _select(win, sketch_id)
    by_label = {label: text for label, text, _ in _panel_rows(win)}
    assert "Volume" not in by_label
    assert "Mass provenance" not in by_label

    # The honesty, behaviourally: across every selection (and back),
    # ZERO parameter_changed emissions from the read-only rows.
    for nid in (hole_id, box_id, sketch_id, prism_id, hole_id):
        assert _select(win, nid)
    assert emitted == [], emitted


@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_refused_body_says_approximated(main_window):
    """AREA 2's honesty clause, in the panel and the report: a body
    whose exact path refuses (the v0.3-style subtract — the Y-axis
    primitive-cylinder tool) says approximated with its own
    polyhedral numbers — never an exact in disguise."""
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QTreeWidgetItem
    import CAD_0

    win = main_window
    assert win._has_kernel

    win._create_registered("box")
    box_id = None
    for i in range(win._feature_tree.topLevelItemCount()):
        it = win._feature_tree.topLevelItem(i)
        if it.text(1) == "SDF":
            box_id = it.data(0, Qt.UserRole)
    assert _select(win, box_id)
    for axis in ("extent_x", "extent_y", "extent_z"):
        win._on_parameter_changed(axis, 1.0)

    # The Y-axis primitive cylinder (NOT a Z-axis prism: the exact
    # subtract's own pinned refusal class).
    cyl = win._scene.add_node("Cyl", CAD_0.sdf.cylinder(0.25, 2.0))
    it = QTreeWidgetItem(["Cyl", "SDF"])
    it.setData(0, Qt.UserRole, cyl)
    win._feature_tree.addTopLevelItem(it)
    did = win._scene.add_operation_node("Sub2", box_id, cyl,
                                        0.0, 0.0, 0.0)
    it2 = QTreeWidgetItem(["Sub2", "CSG"])
    it2.setData(0, Qt.UserRole, did)
    win._feature_tree.addTopLevelItem(it2)

    node = win._scene.get_node(did)
    assert node.exact_brep is None  # refused, honestly
    assert node.hybrid_refusal != ""

    # The panel: approximated, with the polyhedral numbers.
    emitted = []
    win._property_panel.parameter_changed.connect(
        lambda name, value: emitted.append((name, value)))
    assert _select(win, did)
    by_label = {label: text for label, text, _ in _panel_rows(win)}
    assert by_label.get("Mass provenance") == "approximated"
    assert by_label.get("Volume", "").startswith("7.6")  # the grid's own
    assert emitted == []

    # The report: the extraction path (faceted), the min |f| margin,
    # the APPROXIMATED flag — the honest lines, all visible.
    text = win._body_report_text(node)
    assert text is not None
    assert "Path: faceted extraction (MC33)" in text
    assert "APPROXIMATED (all_exact = False" in text
    assert "min |f| on grid (P2 margin):" in text
    assert "0 subshapes exact / " in text
