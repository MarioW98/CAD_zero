"""CAD_0.ui.fillet_dialog — the fillet's edge picker (G2-u3 m5,
ADR-0039 D6 rung 5: Feature -> Fillet, "select an edge, set the
radius"; G5 m5: the picker passes from 4-in to 12-in — the
CONTEMPORANEA's multi-selection and the wall's demolition DECLARED
at the pick surface, ADR-0043's own Consequences).

THE SELECTION IS THE BY-NAME SURFACE, never a parse of the render: the
dialog enumerates the operand's NAMED edges through the kernel's own
query (``CAD_0.brep.edge_query`` — the structural ShapeId's canonical
render, the very string the recipe will store), so the user picks the
edge by the NAME the kernel uses and the reference crosses to the
recipe CHARACTER FOR CHARACTER. The viewport has no edge rendering to
click (the fillet node itself carries no display mesh — the m4 design)
and the endpoints ride the rows only as an identification aid, never
as the reference.

THE VERTICAL-ONLY WALL IS DOWN FOR THE CUBE (G5 m5, the m4 register's
own promise kept here): every row's verdict is the kernel's own
admission through the WIDENED dispatch — the verticals say IN CLASS
(the class-1 walk), the ring edges say IN CLASS too (the m2 walk one
axis over, the m3 corner trigger inside it at an occupied corner).
The rows OUT of the box's twelve are DECLARED with their family (the
skewed dihedral is stage 2's seat, D1-OUT/D8 — never a silent
refusal), the same m6-correttiva-ter discipline.

THE CONTEMPORANEA (G5 m5, ADR-0043 D5.1/D6.6): two or more selected
rows apply as the MULTI RECIPE — the edge->radius map, the ANGLES LAW
DECLARED BEFORE EVERY APPLY (the ADR's own register): as the
selection changes the dialog speaks the law's own verdict on the
map (``fillet_admission_multi`` — the same discovery, the same law,
the same machine the Apply walks, nothing installed): one radius per
corner-connected component (the connected edge-corner graph forces
one radius), the ring consumed by THE ACT indivisible (the whole
cube or the partial world of the verticals), the operand ONE body.
One selected row applies as the single recipe (the sequential
world, the corner-freedom the kernel's own admission guards).

The operand's exact side comes from the same doors the scene's
discovery runs: the node's own hybrid (``exact_brep`` — the subtract
chain) or the bridge's feature-aware primitive fit
(``sdf_to_brep_primitive(body, node.id)`` — the fitted box, named
under the DAG identity the find will match). An SDF node that fits no
primitive is not a candidate — the honest quiet answer, never a
faceted guess (a faceted body carries no names).

THE REFUSALS IN WORDS, with the bound carried: every refusal the
kernel makes (the typed bands — fillet_radius_exceeds_face,
fillet_exits_annulus, fillet_eats_wall; the class families; the dead
reference) crosses the binding with the offending number and the limit
already inside the message — the dialog QUOTES it verbatim in its own
status area, never a paraphrase that invents, never a bare "error".
"""

from __future__ import annotations

from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
)

# The row's own data roles: the reference (the recipe's field, the
# m5 contract) and the base text (the name + the endpoints aid) the
# verdict annotation APPENDS to — never replaces (m6-correttiva-ter).
_ROLE_REFERENCE = Qt.UserRole
_ROLE_VERDICT = Qt.UserRole + 1
_ROLE_BASE = Qt.UserRole + 2

# The radius editor's starting value — DECLARED, not magic: the radius
# is the user's authored parameter (the recipe's own typed field); the
# editor must start somewhere, and the honest somewhere is a small
# in-band value for every guide case the rung reaches (the box's reach
# 2, the rim's band r < 0.5). The bands' own bounds stay the kernel's
# to speak — in the refusal, with the numbers, when the value violates
# them.
FILLET_RADIUS_DEFAULT = 0.1


def _operand_body(scene, node):
    """The operand's exact side for the picker — the SAME body the
    scene's discovery will search: the node's own hybrid when it has
    one, else the bridge's feature-aware primitive fit named under the
    node's DAG id. None when the node owns no exact side (the honest
    quiet answer — not a candidate)."""
    import CAD_0

    exact = getattr(node, "exact_brep", None)
    if exact is not None:
        return exact
    if getattr(node, "kind", None) != "sdf":
        return None
    try:
        bridge_mod = getattr(CAD_0, "bridge", None) or CAD_0._CAD_0.bridge
    except Exception:
        return None
    return bridge_mod.sdf_to_brep_primitive(node.body, node.id)


def operand_candidates(scene):
    """The nodes the fillet can be asked about, in document order:
    every node whose exact side answers (an exact operation, a fitted
    primitive). The kernel's own admission decides the rest — the
    dialog offers the surface, the refusals stay the kernel's words."""
    out = []
    if scene is None:
        return out
    for i in range(scene.node_count()):
        node = scene.node_at(i)
        if node is None:
            continue
        if _operand_body(scene, node) is not None:
            out.append(node)
    return out


def _verdict_text(verdict: dict) -> str:
    """The row's own short verdict — the kernel's words, the first
    clause (the full text rides the row's verdict data and the
    status area on selection). The wall NAMED, never paraphrased
    into vagueness: the family the kernel refused with is the family
    the row says (m6-correttiva-ter, defect A's declaration)."""
    if verdict.get("admitted"):
        op = verdict.get("operand_name") or "?"
        return ("v1: IN CLASS — the rolling ball takes this edge "
                "(resolves on {})".format(op))
    reason = verdict.get("reason") or ""
    first = reason.split(" - ")[0]
    return "v1: OUT — {}".format(first)


class FilletDialog(QDialog):
    """Feature -> Fillet: the edge picker and the radius field.

    The Apply routes are INJECTED (the window's own command-wrapped
    ``_apply_fillet`` and ``_apply_fillet_multi``): the dialog never
    touches the scene itself — it collects the selection (the edges'
    NAMES, character for character) and the radius, and hands them to
    the one route that owns the write. ONE selected row applies as the
    single recipe (the sequential world); TWO OR MORE as the
    contemporanea (the edge->radius map, the derived chain replayed).
    A refusal (RuntimeError/ValueError) is quoted in the status area
    and the dialog STAYS OPEN — the user adjusts the radius; a success
    closes the dialog (the feature is on the tree).
    """

    def __init__(self, scene, selected_node_id: Optional[int],
                 apply_route: Callable[[str, float], int],
                 apply_route_multi: Optional[
                     Callable[[List[Tuple[str, float]], str], int]] = None,
                 parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle(
            "Fillet — select edges, set the radius")
        self._scene = scene
        self._selected_node_id = selected_node_id
        self._apply_route = apply_route
        self._apply_route_multi = apply_route_multi
        self._bodies = {}          # node id -> the operand's exact body
        self._edge_names = {}      # node id -> [the named edges' strings]

        layout = QVBoxLayout(self)

        # ---- The operand (the find's own subject) --------------------
        self._combo = QComboBox(self)
        self._combo.currentIndexChanged.connect(self._reload_edges)
        layout.addWidget(QLabel(
            "Operand (the body whose exact side carries the edge):",
            self))
        layout.addWidget(self._combo)

        # ---- The edge list — the kernel's own names ------------------
        # MULTI-SELECTABLE (G5 m5): two or more rows apply as the
        # contemporanea (the edge->radius map).
        layout.addWidget(QLabel(
            "Edges — the kernel's own names (the reference the recipe "
            "stores, character for character). Select one edge for "
            "the single fillet, two or more for the contemporanea "
            "(the derived chain: the verticals, then the ring's act):",
            self))
        self._edges = QListWidget(self)
        self._edges.setSelectionMode(QListWidget.ExtendedSelection)
        self._edges.itemSelectionChanged.connect(self._on_edge_selected)
        layout.addWidget(self._edges)
        self._unnamed = QLabel("", self)
        self._unnamed.setStyleSheet("color: gray;")
        layout.addWidget(self._unnamed)

        # ---- The radius — the authored parameter ---------------------
        self._radius = QDoubleSpinBox(self)
        # The house editor shape (the property panel's own float
        # field): commit-only (keyboardTracking off — a half-typed
        # number is not a modification), the panel's range and step.
        self._radius.setRange(0.0, 1e9)
        self._radius.setDecimals(6)
        self._radius.setSingleStep(0.1)
        self._radius.setKeyboardTracking(False)
        self._radius.setValue(FILLET_RADIUS_DEFAULT)
        # The bands speak with the radius (the typed bounds are
        # dynamic): every committed edit re-annotates the rows — the
        # verdict on the row is always the CURRENT radius's own
        # (m6-correttiva-ter, the wall never stale).
        self._radius.valueChanged.connect(self._annotate_rows)
        layout.addWidget(QLabel("Radius (the rolling ball's r):", self))
        layout.addWidget(self._radius)

        # ---- The reference, shown character for character ------------
        self._reference = QLabel("Reference: — (pick an edge)", self)
        self._reference.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._reference)

        # ---- The refusals, in words (the kernel's own) ---------------
        self._status = QLabel("", self)
        self._status.setWordWrap(True)
        self._status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._status)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        buttons.button(QDialogButtonBox.Ok).setText("&Apply")
        buttons.accepted.connect(self._on_apply)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self._populate()

    # ----- the population (the query's own answers) -------------------

    def _populate(self) -> None:
        self._combo.clear()
        self._bodies.clear()
        self._edge_names.clear()
        candidates = operand_candidates(self._scene)
        if not candidates:
            self._status.setText(
                "No body with an exact side in the scene — the fillet "
                "consumes the exact side (an exact subtract, a fitted "
                "primitive). Remedy: model the operand as one of those "
                "(the m4 register's own words), then re-open Feature → "
                "Fillet.")
            return
        for node in candidates:
            body = _operand_body(self._scene, node)
            self._bodies[node.id] = body
            import CAD_0
            brep_mod = getattr(CAD_0, "brep", None) or CAD_0._CAD_0.brep
            names = []
            for i in range(brep_mod.edge_count(body)):
                info = brep_mod.edge_query(body, i)
                if info["name"]:
                    names.append(info["name"])
            self._edge_names[node.id] = names
            label = "{} (id {}) — {} edge(s) named".format(
                node.name, node.id, len(names))
            self._combo.addItem(label, node.id)
        # The pre-selection: the tree's current selection when it is a
        # candidate (the natural act: the user selected the body, then
        # fired the menu).
        if self._scene is not None and self._selected_node_id is not None:
            for i in range(self._combo.count()):
                if self._combo.itemData(i) == self._selected_node_id:
                    self._combo.setCurrentIndex(i)
                    break
        self._reload_edges()

    def _reload_edges(self) -> None:
        self._edges.clear()
        self._unnamed.setText("")
        self._status.setText("")
        node_id = self._combo.currentData()
        if node_id is None or node_id not in self._bodies:
            self._on_edge_selected()
            return
        body = self._bodies[node_id]
        import CAD_0
        brep_mod = getattr(CAD_0, "brep", None) or CAD_0._CAD_0.brep
        unnamed = 0
        listed = set()
        for i in range(brep_mod.edge_count(body)):
            info = brep_mod.edge_query(body, i)
            if not info["name"]:
                unnamed += 1
                continue
            # The seam-wired PAIRS taken WHOLE (the morse-5 register's
            # own rule): ONE row per logical name — both materialized
            # halves answer to it, the pick is BY NAME.
            if info["name"] in listed:
                continue
            listed.add(info["name"])
            # The endpoints ride the row as an identification aid in
            # the EDGE TABLE'S own format (the vertex addresses — the
            # query's honest answer; the reference is the NAME, never
            # the geometry).
            ends = "{} - {}".format(info["v0"], info["v1"])
            base = "{}   [{}]".format(info["name"], ends)
            item = QListWidgetItem(base)
            item.setData(_ROLE_REFERENCE, info["name"])
            item.setData(_ROLE_BASE, base)
            self._edges.addItem(item)
        if unnamed:
            self._unnamed.setText(
                "{} edge(s) honestly unnamed — not referenceable (the "
                "fillet references the edge BY NAME)".format(unnamed))
        self._annotate_rows()
        self._on_edge_selected()

    # ----- the wall said on the row (m6-correttiva-ter, defect A) ----

    def _annotate_rows(self) -> None:
        """Every row carries the kernel's own verdict — the admission
        DRY RUN, the Apply's own path without the install (the same
        discovery, the same dispatch, the same admission the Apply
        walks). THE WALL IS DOWN FOR THE CUBE (G5 m5, the m4
        register's own promise kept here): the verticals say IN
        CLASS (the class-1 walk), the RING edges say IN CLASS TOO
        (the m2 walk one axis over through the widened dispatch, the
        m3 corner trigger inside it) — the vertical-only wall's
        demolition DECLARED at the pick surface, never silent; the
        rows OUT of the box's twelve speak their family (the skewed
        dihedral is stage 2's seat, D1-OUT/D8 — DICHIARATO, never a
        mute refusal), the dead references say VANISHED with the
        killer, the bands speak with their bounds at the CURRENT
        radius. The operand the reference resolves on is named too —
        the chain's current state (defect B declared at the same
        surface)."""
        if self._scene is None or not hasattr(
                self._scene, "fillet_admission"):
            return
        radius = self.selected_radius()
        for i in range(self._edges.count()):
            item = self._edges.item(i)
            name = item.data(_ROLE_REFERENCE)
            base = item.data(_ROLE_BASE) or name
            try:
                verdict = self._scene.fillet_admission(name, radius)
            except (ValueError, RuntimeError):
                item.setText(base)
                continue
            item.setData(_ROLE_VERDICT, verdict)
            item.setText("{}   {}".format(base, _verdict_text(verdict)))

    # ----- the selection and the application --------------------------

    def _on_edge_selected(self) -> None:
        items = self._edges.selectedItems()
        if not items:
            self._reference.setText(
                "Reference: — (pick an edge)")
            return
        if len(items) == 1:
            # The NAME, character for character — the same string the
            # recipe will store and the kernel will resolve.
            self._reference.setText(
                "Reference: {}".format(
                    items[0].data(_ROLE_REFERENCE)))
            # The verdict's own words in the status area (the row
            # carries the short form; the selection carries the
            # kernel's FULL text — the bound, the remedy —
            # m6-correttiva-ter).
            verdict = items[0].data(_ROLE_VERDICT)
            if isinstance(verdict, dict):
                if verdict.get("admitted"):
                    self._status.setText(
                        "In class — resolves on {!r} (the chain's "
                        "current state).".format(
                            verdict.get("operand_name")))
                else:
                    self._status.setText(verdict.get("reason") or "")
            return
        # TWO OR MORE — THE LAW DECLARED BEFORE EVERY APPLY (ADR-0043's
        # own register: "il picker passa da 4-in a 12-in con la legge
        # degli angoli dichiarata prima di ogni Apply"): the map's own
        # verdict spoken as the selection changes — the same discovery,
        # the same angles law, the same machine the Apply walks,
        # nothing installed (the admission's own dry run).
        refs = [i.data(_ROLE_REFERENCE) for i in items]
        self._reference.setText(
            "References: {} spigolo(i) — the contemporanea "
            "(the edge->radius map)".format(len(refs)))
        verdict = self._multi_verdict(refs)
        if verdict.get("admitted"):
            self._status.setText(
                "In class — the contemporanea of {} spigoli resolves "
                "on {!r} (the angles law green at this radius: one "
                "radius per corner-connected component).".format(
                    len(refs), verdict.get("operand_name")))
        else:
            self._status.setText(verdict.get("reason") or "")

    def _multi_verdict(self, refs) -> dict:
        """The map's admission — the verdict the VALUE, never a throw
        (the binding's own contract; the absent surface answered as
        the honest not-admitted)."""
        if self._scene is None or not hasattr(
                self._scene, "fillet_admission_multi"):
            return {"admitted": False,
                    "reason": "the multi admission is not available "
                              "in this build",
                    "operand_id": 0, "operand_name": ""}
        radius = self.selected_radius()
        return self._scene.fillet_admission_multi(
            [(r, radius) for r in refs])

    def selected_references(self):
        """The selected rows' references, in ROW ORDER (the authored
        order — the vertical passage replays in it)."""
        return [self._edges.item(i).data(_ROLE_REFERENCE)
                for i in range(self._edges.count())
                if self._edges.item(i).isSelected()]

    def selected_reference(self) -> Optional[str]:
        refs = self.selected_references()
        return refs[0] if refs else None

    def selected_radius(self) -> float:
        return float(self._radius.value())

    def _on_apply(self) -> None:
        refs = self.selected_references()
        if not refs:
            self._status.setText(
                "Pick an edge first — the reference is the recipe's own "
                "field (the grammar's form, matched whole).")
            return
        radius = self.selected_radius()
        if len(refs) == 1:
            # The single recipe (the sequential world, the m2/m3
            # kernel doors through the widened dispatch).
            try:
                self._apply_route(refs[0], radius)
            except (ValueError, RuntimeError) as exc:
                # The kernel's refusal, QUOTED verbatim — the band's
                # bound is already inside the message (the offending
                # number and the limit); the dialog stays open for the
                # next try. (Both exception kinds cross the binding:
                # the class refusals as ValueError, the scene's own
                # doors as RuntimeError — the bridge's documented
                # contract.)
                self._status.setText(str(exc))
                return
            self.accept()
            return
        # THE CONTEMPORANEA (two or more): the multi route — the law
        # declared first (the dry run's own words, never a silent
        # kernel refusal), then the apply.
        if self._apply_route_multi is None:
            self._status.setText(
                "The contemporanea is not available in this build — "
                "the multi recipe needs the v0.8 kernel surface.")
            return
        verdict = self._multi_verdict(refs)
        if not verdict.get("admitted"):
            self._status.setText(
                verdict.get("reason") or
                "the map's admission refused")
            return
        try:
            self._apply_route_multi(
                [(r, radius) for r in refs], "FilletMulti")
        except (ValueError, RuntimeError) as exc:
            self._status.setText(str(exc))
            return
        self.accept()
