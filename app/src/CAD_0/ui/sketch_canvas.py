"""CAD_0.ui.sketch_canvas — the 2D sketch canvas (ADR-0030 D5, unit 4).

The binding interaction specification, implemented. The governing
principle (D5's preamble, stated once and applied everywhere):

> Direct manipulation is the outer face; the recipe is the real
> object. Every graphical gesture is a write of parameters into the
> sketch node's recipe. No free geometric state exists that the
> document cannot rewrite.

The canvas shows exactly one of three things: (a) a rendering of
recipe parameters, (b) a transient preview of the gesture in
progress, (c) an acceptance glyph for a suggestion. There is no
fourth thing; a fourth thing is a bug. The grid, the background and
the in-canvas status bar are the pinned chrome of D5.3's aspect
table, not a fourth thing.

QPainter only — no GL (the draft's rendering decision stands). The
view is the world XY plane seen from +Z: +X right, +Y up. Pan and
zoom are transient view state, never document state; a reopened
sketch fits its content (D5.6).

Class validation (D5.5) calls the KERNEL's predicates through the
bindings — ``scene.sketch_entity_error`` — never a UI-side rulebook.
The definition state (D5.3) is ``scene.sketch_chain_closed`` — the
same chain_is_closed behind the extrude's closure precondition. The
picking/snapping measures (the exact per-entity formulas of D5.2)
live here: they measure CURVES, never the composed field ("the field
is not a measuring stick").
"""

from __future__ import annotations

import math
from typing import Optional

from PySide6.QtCore import QPoint, QPointF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QLineEdit, QWidget

# --- The pinned aspect table (D5.3) — normative, not a theme ------------

COLOR_BACKGROUND = QColor("#1E1E1E")     # flat
COLOR_OUTER_CLOSED = QColor("#FFFFFF")   # closed chain, outer role, 2px solid
COLOR_HOLE_CLOSED = QColor("#7E57C2")    # closed chain, hole role, 2px solid
COLOR_UNDER_DEFINED = QColor("#E8A33D")  # open chain: dash [6,3], wins over role
COLOR_SELECTED = QColor("#4FC1FF")       # 3px solid + 6x6 px square handles
COLOR_PREVIEW = QColor("#9AA0A6")        # dash [2,2], 1px
COLOR_GLYPH = QColor("#4FC1FF")          # 5px circle marker + the value as text
COLOR_GRID = QColor("#3A3F44")           # dots, 1 world-unit pitch (fixed)
COLOR_REJECTION = QColor("#FF6E6E")      # canvas status bar, until next gesture

GRID_PITCH = 1.0          # world units (fixed — D5.7 owns configurability)
GESTURE_THRESHOLD_PX = 2.0    # what counts as degenerate (D5.5)
SELECT_THRESHOLD_PX = 10.0    # nearest-entity picking (D5.2)
SUGGEST_HV_PX = 6.0           # horizontal / vertical (D5.4)
SUGGEST_COINCIDE_PX = 8.0     # endpoint coincidence (D5.4)
SUGGEST_GRID_PX = 6.0         # grid point (D5.4)

ARC_RENDER_SEGMENTS = 96      # polyline sampling for rendering only


# --- Pure geometry helpers (the D5.2 measuring stick) --------------------
#
# Exact per-entity formulas on the recipe's dict form (what
# scene.sketch_chains returns). They measure the CURVE — never the
# composed field.

def entity_endpoints(e: dict):
    """The (start, end) endpoints of a segment/arc; None for a circle
    (a circle is a closed loop with no endpoints to connect)."""
    if e["kind"] == "segment":
        return (tuple(e["a"]), tuple(e["b"]))
    if e["kind"] == "arc":
        a0 = e["start_angle"]
        a1 = a0 + e["sweep"]
        cx, cy = e["center"]
        r = e["radius"]
        return ((cx + r * math.cos(a0), cy + r * math.sin(a0)),
                (cx + r * math.cos(a1), cy + r * math.sin(a1)))
    return None


def point_segment_distance(p, a, b) -> float:
    """The exact point-segment distance (clamped projection)."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_arc_distance(p, center, radius, a0, sweep) -> float:
    """The exact CLAMPED arc distance: the radial distance when the
    point's angle lies in the span, else the distance to the nearer
    endpoint."""
    cx, cy = center
    px, py = p
    vx, vy = px - cx, py - cy
    d = math.hypot(vx, vy)
    if d <= 0.0:
        return radius  # the center: |0 - r|
    angle = math.atan2(vy, vx)
    rel = (angle - a0) % (2.0 * math.pi)
    if rel <= sweep:
        return abs(d - radius)
    # Clamp to the nearer endpoint of the span.
    span = ((rel, False), (2.0 * math.pi - rel, True))
    _, at_end = min(span)
    edge = a0 + sweep if at_end else a0
    ex = cx + radius * math.cos(edge)
    ey = cy + radius * math.sin(edge)
    return math.hypot(px - ex, py - ey)


def entity_curve_distance(e: dict, p) -> float:
    """Distance from p to the entity's CURVE, the exact per-entity
    formula (D5.2's picking rule — never the composed field)."""
    if e["kind"] == "segment":
        return point_segment_distance(p, tuple(e["a"]), tuple(e["b"]))
    if e["kind"] == "circle":
        cx, cy = e["center"]
        return abs(math.hypot(p[0] - cx, p[1] - cy) - e["radius"])
    return point_arc_distance(p, tuple(e["center"]), e["radius"],
                              e["start_angle"], e["sweep"])


def segment_length(e: dict) -> float:
    a, b = e["a"], e["b"]
    return math.hypot(b[0] - a[0], b[1] - a[1])


def arc_mid_angle(e: dict) -> float:
    return e["start_angle"] + 0.5 * e["sweep"]


def polar(center, radius, angle):
    return (center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle))


# --- The named-parameter tables (D5.1 — the names are the spec) ----------

ENTITY_PARAM_ORDER = {
    "segment": ["x0", "y0", "x1", "y1", "interior_left"],
    "circle": ["cx", "cy", "r", "hole"],
    "arc": ["cx", "cy", "r", "a0", "a1", "hole"],
}


def params_from_entity(e: dict) -> dict:
    """The recipe's entity as the named-parameter table the panel and
    the inline editor address (D5.1: the names, as serialized)."""
    if e["kind"] == "segment":
        return {"x0": float(e["a"][0]), "y0": float(e["a"][1]),
                "x1": float(e["b"][0]), "y1": float(e["b"][1]),
                "interior_left": bool(e["interior_left"])}
    if e["kind"] == "circle":
        return {"cx": float(e["center"][0]), "cy": float(e["center"][1]),
                "r": float(e["radius"]), "hole": bool(e["hole"])}
    a1 = e["start_angle"] + e["sweep"]
    return {"cx": float(e["center"][0]), "cy": float(e["center"][1]),
            "r": float(e["radius"]), "a0": float(e["start_angle"]),
            "a1": float(a1), "hole": bool(e["hole"])}


def entity_label(e: dict, chain: int, index: int) -> str:
    """A stable, human-readable name (the certificate's own format)."""
    kind = e["kind"]
    return "{}[chain {} #{}]".format(kind, chain, index)




def _kernel_entity_error(entity) -> str:
    """The kernel's per-entity class check (D5.5), via the bound
    module function — ONE opinion, called before every write."""
    import CAD_0
    return CAD_0.scene.sketch_entity_error(entity)


def make_entity(kind: str, params: dict):
    """The named-parameter table -> the kernel entity, via the bound
    factories (the single construction route — the canvas, the panel
    and the main window all build entities through here). a1 is
    folded into the sweep: the recipe stores (a0, sweep), the panel
    and the gesture speak a0/a1."""
    import CAD_0
    scene = CAD_0.scene
    if kind == "segment":
        return scene.sketch_segment(
            (params["x0"], params["y0"]),
            (params["x1"], params["y1"]),
            params.get("interior_left", True))
    if kind == "circle":
        return scene.sketch_circle(
            (params["cx"], params["cy"]), params["r"],
            params.get("hole", False))
    a0 = params["a0"]
    sweep = (params["a1"] - a0) % (2.0 * math.pi)
    return scene.sketch_arc(
        (params["cx"], params["cy"]), params["r"],
        params.get("hole", False), a0, sweep)


class SketchCanvas(QWidget):
    """A VIEW of one sketch node (D5.6): nothing more, with exactly
    the privileges of nothing more. Writes go through the shared
    command stack (the single history — the canvas keeps no private
    one); every completed gesture, released drag and text edit is ONE
    entry; an aborted anything is zero entries."""

    # (chain, index) of the selected entity, or None.
    entity_selected = Signal(object)
    # Emitted after every committed write (the recipe changed).
    sketch_edited = Signal()

    TOOLS = ("select", "segment", "circle", "arc")

    def __init__(self, scene, command_stack, node_id: int,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._scene = scene
        self._stack = command_stack
        self._node_id = node_id
        self.setMinimumSize(320, 240)
        self.setMouseTracking(True)

        # Transient view state (never document state).
        self._scale = 60.0            # pixels per world unit
        self._center = (0.0, 0.0)     # world point at the widget centre

        # Interaction state.
        self._tool = "select"
        self._gesture = None          # the in-progress creation gesture
        self._selected: Optional[tuple] = None   # (chain, index)
        self._drag = None             # the in-progress handle drag
        self._status_message = ""     # the pinned rejection channel
        self._inline_editor: Optional[QLineEdit] = None
        self._pan = None              # middle-button pan anchor

        self._fit_content()

    # --- public surface (the view's contract) ---------------------------

    def node_id(self) -> int:
        return self._node_id

    def set_tool(self, tool: str) -> None:
        if tool not in self.TOOLS:
            return
        self._abort_any_interaction()
        self._tool = tool
        self.update()

    def tool(self) -> str:
        return self._tool

    @property
    def status_message(self) -> str:
        """The pinned rejection channel (D5.5): the message persists
        until the next gesture begins. Asserted by the D6 smoke."""
        return self._status_message

    def selected_entity(self):
        return self._selected

    def entity_aspects(self) -> dict:
        """Per-entity aspect class, the PURE function of the recipe
        state the D5.3 table pins (asserted by the D6 smoke):

          under_defined — the entity's chain is open (the amber dash
                          wins over the role color);
          outer_closed  — closed chain, outer role (#FFFFFF solid);
          hole_closed   — closed chain, hole role (#7E57C2 solid).

        Selection is a view state, layered separately (the selected
        entity also renders with its handles), not an aspect class.
        """
        aspects = {}
        chains = self._scene.sketch_chains(self._node_id)
        for ci, chain in enumerate(chains):
            closed = self._scene.sketch_chain_closed(self._node_id, ci)
            for ei, e in enumerate(chain):
                if not closed:
                    cls = "under_defined"
                elif e["kind"] != "segment" and e.get("hole"):
                    cls = "hole_closed"
                else:
                    cls = "outer_closed"
                aspects[(ci, ei)] = cls
        return aspects

    def open_chain_gaps(self) -> list:
        """(chain, midpoint_world, distance) for every open joint of
        every open chain — the distance-to-closure the canvas shows
        (D5's amendment: the extrude demands closure, the canvas
        shows the distance to it)."""
        gaps = []
        chains = self._scene.sketch_chains(self._node_id)
        for ci, chain in enumerate(chains):
            if self._scene.sketch_chain_closed(self._node_id, ci):
                continue
            if not chain:
                continue
            n = len(chain)
            for i in range(n):
                a = chain[i]
                b = chain[(i + 1) % n]
                ea, eb = entity_endpoints(a), entity_endpoints(b)
                if ea is None or eb is None:
                    continue
                dist = math.hypot(eb[0][0] - ea[1][0], eb[0][1] - ea[1][1])
                if dist > 1e-5:
                    mid = (0.5 * (ea[1][0] + eb[0][0]),
                           0.5 * (ea[1][1] + eb[0][1]))
                    gaps.append((ci, mid, dist))
        return gaps

    def dimension_labels(self) -> dict:
        """The visible dimension labels: the RADIUS (editable) and the
        measured LENGTH (read-only — writing it needs a hidden anchor
        choice, D5.2's rule). {label_text: (world_pos, value)}."""
        labels = {}
        chains = self._scene.sketch_chains(self._node_id)
        for ci, chain in enumerate(chains):
            for ei, e in enumerate(chain):
                if e["kind"] == "segment":
                    a, b = tuple(e["a"]), tuple(e["b"])
                    mid = (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
                    labels["length = {:.6g}".format(segment_length(e))] = \
                        (mid, segment_length(e))
                else:
                    mid = polar(tuple(e["center"]), e["radius"],
                                arc_mid_angle(e) if e["kind"] == "arc"
                                else 0.0)
                    labels["r = {:.6g}".format(e["radius"])] = (mid, e["radius"])
        return labels

    def handles_for(self, e: dict) -> list:
        """(name, world_pos, role) per the D5.2 pinned table:

          segment: endpoint0 (x0,y0) / endpoint1 (x1,y1)
          circle:  center (cx,cy) / radius_point (cx+r, cy)
          arc:     center / radius_point (mid-angle) /
                   endpoint_a0 / endpoint_a1
        """
        out = []
        if e["kind"] == "segment":
            out.append(("endpoint0", tuple(e["a"]), "point"))
            out.append(("endpoint1", tuple(e["b"]), "point"))
        elif e["kind"] == "circle":
            out.append(("center", tuple(e["center"]), "point"))
            out.append(("radius_point",
                        (e["center"][0] + e["radius"], e["center"][1]),
                        "radius"))
        else:
            out.append(("center", tuple(e["center"]), "point"))
            out.append(("radius_point",
                        polar(tuple(e["center"]), e["radius"],
                              arc_mid_angle(e)), "radius"))
            a0, a1 = e["start_angle"], e["start_angle"] + e["sweep"]
            out.append(("endpoint_a0",
                        polar(tuple(e["center"]), e["radius"], a0), "angle"))
            out.append(("endpoint_a1",
                        polar(tuple(e["center"]), e["radius"], a1), "angle"))
        return out

    # --- coordinate transform --------------------------------------------

    def screen_to_world(self, sx: float, sy: float):
        return ((sx - 0.5 * self.width()) / self._scale + self._center[0],
                -(sy - 0.5 * self.height()) / self._scale + self._center[1])

    def world_to_screen(self, wx: float, wy: float):
        return ((wx - self._center[0]) * self._scale + 0.5 * self.width(),
                -(wy - self._center[1]) * self._scale + 0.5 * self.height())

    def px(self, world: float) -> float:
        """World length -> screen pixels at the current zoom."""
        return world * self._scale

    def world(self, pixels: float) -> float:
        """Screen pixels -> world length at the current zoom."""
        return pixels / self._scale

    def _fit_content(self) -> None:
        """A reopened/opened sketch fits its content (D5.6); an empty
        one opens on the origin at the default scale."""
        chains = self._scene.sketch_chains(self._node_id) \
            if self._scene is not None else []
        xs, ys = [], []
        for chain in chains:
            for e in chain:
                if e["kind"] == "segment":
                    xs += [e["a"][0], e["b"][0]]
                    ys += [e["a"][1], e["b"][1]]
                else:
                    c, r = e["center"], e["radius"]
                    xs += [c[0] - r, c[0] + r]
                    ys += [c[1] - r, c[1] + r]
                    if e["kind"] == "arc":
                        for pt in entity_endpoints(e):
                            xs.append(pt[0])
                            ys.append(pt[1])
        if not xs:
            self._center, self._scale = (0.0, 0.0), 60.0
            return
        w, h = max(self.width(), 320), max(self.height(), 240)
        span_x = max(max(xs) - min(xs), 1e-3)
        span_y = max(max(ys) - min(ys), 1e-3)
        self._center = (0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys)))
        self._scale = max(24.0, min(0.8 * w / span_x, 0.8 * h / span_y))

    # --- the recipe read surface -----------------------------------------

    def _chains(self) -> list:
        return self._scene.sketch_chains(self._node_id)

    def _entity(self, chain: int, index: int) -> Optional[dict]:
        chains = self._chains()
        if 0 <= chain < len(chains) and 0 <= index < len(chains[chain]):
            return chains[chain][index]
        return None

    def _entity_at(self, world_pos):
        """Nearest entity within the selection threshold, the exact
        per-entity formulas (D5.2). Returns (chain, index, dist)."""
        best = None
        for ci, chain in enumerate(self._chains()):
            for ei, e in enumerate(chain):
                d = entity_curve_distance(e, world_pos)
                if d <= self.world(SELECT_THRESHOLD_PX) and \
                        (best is None or d < best[2]):
                    best = (ci, ei, d)
        return best

    def _all_endpoints(self):
        """(chain, index, point) for every existing entity endpoint
        (the coincidence candidate's material)."""
        out = []
        for ci, chain in enumerate(self._chains()):
            for ei, e in enumerate(chain):
                ends = entity_endpoints(e)
                if ends is None:
                    continue
                out.append((ci, ei, ends[0]))
                out.append((ci, ei, ends[1]))
        return out

    # --- suggestions (D5.4 — offered, never applied) ----------------------

    def _candidates_for_point(self, cursor_world, anchor_world,
                              kinds=("horizontal", "vertical",
                                      "coincidence", "grid")):
        """The candidate set for a completing point, nearest first.
        The anchor is the gesture's fixed point (None when the
        completion writes no point — the arc's final click writes an
        ANGLE, so only the exact endpoint coincidence applies there:
        a grid/H/V point would write a value the recipe cannot take).
        Returns [(kind, screen_dist, value, text, structural_chain)].
        """
        out = []
        # Horizontal / vertical: the anchor's coordinate, verbatim.
        if anchor_world is not None and \
                "horizontal" in kinds and "vertical" in kinds:
            dy_px = self.px(abs(cursor_world[1] - anchor_world[1]))
            if dy_px < SUGGEST_HV_PX:
                out.append(("horizontal", dy_px,
                            (cursor_world[0], anchor_world[1]),
                            "y = {:.6g}".format(anchor_world[1]), None))
            dx_px = self.px(abs(cursor_world[0] - anchor_world[0]))
            if dx_px < SUGGEST_HV_PX:
                out.append(("vertical", dx_px,
                            (anchor_world[0], cursor_world[1]),
                            "x = {:.6g}".format(anchor_world[0]), None))
        # Endpoint coincidence: the existing endpoint's coordinates.
        if "coincidence" in kinds:
            for ci, ei, pt in self._all_endpoints():
                d_px = self.px(math.hypot(cursor_world[0] - pt[0],
                                          cursor_world[1] - pt[1]))
                if d_px < SUGGEST_COINCIDE_PX:
                    out.append(("coincidence", d_px, tuple(pt),
                                "({}, {:.6g})".format(pt[0], pt[1]), ci))
        # Grid point: the quantized coordinates (fixed 1-unit pitch).
        if "grid" in kinds:
            gx = round(cursor_world[0] / GRID_PITCH) * GRID_PITCH
            gy = round(cursor_world[1] / GRID_PITCH) * GRID_PITCH
            d_px = self.px(math.hypot(cursor_world[0] - gx,
                                      cursor_world[1] - gy))
            if d_px < SUGGEST_GRID_PX:
                out.append(("grid", d_px, (gx, gy),
                            "({}, {})".format(gx, gy), None))
        out.sort(key=lambda c: c[1])
        return out

    def _active_suggestion(self):
        """The glyph's candidate (the nearest under threshold), or
        None. Computed live during the gesture for rendering; the
        completing release/click re-derives it and honours SHIFT."""
        g = self._gesture
        if g is None:
            return None
        if g["kind"] == "segment":
            return self._candidates_for_point(g["cursor"], g["a"])
        if g["kind"] == "arc" and g["phase"] == "sweep":
            return self._candidates_for_point(
                g["cursor"], None, kinds=("coincidence",))
        return None

    # --- the rejection channel (D5.5) ------------------------------------

    def _reject(self, message: str) -> None:
        """A visible refusal: the message names the refused value, in
        the pinned channel, persisting until the next gesture."""
        self._status_message = message
        self._gesture = None
        self._drag = None
        self.update()

    # --- event handlers ---------------------------------------------------

    def mousePressEvent(self, event) -> None:
        pos = event.position()
        if event.button() == Qt.RightButton:
            # Right-click aborts the interaction: no entity, no write,
            # no undo entry (D5.1). Not a context menu (D5.4).
            self._abort_any_interaction()
            return
        if event.button() == Qt.MiddleButton:
            self._pan = (pos.x(), pos.y(), self._center)
            return
        if event.button() != Qt.LeftButton:
            return
        world = self.screen_to_world(pos.x(), pos.y())

        # A running inline editor commits on focus-out/return; a new
        # press cancels it (an aborted edit writes nothing).
        self._cancel_inline_editor()

        if self._tool in ("segment", "circle", "arc") and \
                self._gesture is None and self._drag is None:
            self._status_message = ""  # a new gesture clears the channel
            if self._tool == "arc":
                self._gesture = {"kind": "arc", "phase": "radius",
                                 "center": world, "cursor": world}
            else:
                self._gesture = {"kind": self._tool, "a": world,
                                 "cursor": world}
            self.update()
            return

        # The arc's second phase: the click fixes a1.
        if self._tool == "arc" and self._gesture is not None and \
                self._gesture["kind"] == "arc" and \
                self._gesture["phase"] == "sweep":
            self._complete_arc(world, event.modifiers())
            return

        # Select tool: press on a handle starts the drag (D5.2).
        if self._selected is not None:
            e = self._entity(*self._selected)
            if e is not None:
                for name, hw, role in self.handles_for(e):
                    hs = self.world_to_screen(*hw)
                    if math.hypot(hs[0] - pos.x(), hs[1] - pos.y()) <= 6.0:
                        self._status_message = ""
                        self._drag = {"handle": name, "role": role,
                                      "entity": self._selected,
                                      "start": world, "cursor": world}
                        self.update()
                        return
        # Press on an entity selects it (the release without a drag).
        hit = self._entity_at(world)
        if hit is not None:
            self._select((hit[0], hit[1]))
        else:
            self._select(None)
        self.update()

    def mouseMoveEvent(self, event) -> None:
        pos = event.position()
        if self._pan is not None:
            dx = pos.x() - self._pan[0]
            dy = pos.y() - self._pan[1]
            self._center = (self._pan[2][0] - self.world(dx),
                            self._pan[2][1] + self.world(dy))
            self.update()
            return
        world = self.screen_to_world(pos.x(), pos.y())
        if self._gesture is not None:
            self._gesture["cursor"] = world
            self.update()
            return
        if self._drag is not None:
            self._drag["cursor"] = world
            self.update()
            return
        self.update()  # hover: the glyph follows the cursor

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton:
            self._pan = None
            return
        if event.button() != Qt.LeftButton:
            return
        pos = event.position()
        world = self.screen_to_world(pos.x(), pos.y())
        if self._gesture is not None:
            if self._gesture["kind"] == "segment":
                self._complete_segment(world, event.modifiers())
            elif self._gesture["kind"] == "circle":
                self._complete_circle(world)
            elif self._gesture["kind"] == "arc" and \
                    self._gesture["phase"] == "radius":
                self._finish_arc_radius(world)
            return
        if self._drag is not None:
            self._commit_drag(world)
            return
        # A bare click on the select tool already selected/deselected
        # at press time; nothing to do at release.

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape:
            # ESC aborts the interaction: parameters untouched, zero
            # undo entries (D5.1/D5.2). An abort is not an undo.
            self._abort_any_interaction()
            return
        super().keyPressEvent(event)

    def wheelEvent(self, event) -> None:
        # Transient zoom at the cursor (never document state).
        pos = event.position()
        before = self.screen_to_world(pos.x(), pos.y())
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self._scale = max(2.0, min(4000.0, self._scale * factor))
        after = self.screen_to_world(pos.x(), pos.y())
        self._center = (self._center[0] + before[0] - after[0],
                        self._center[1] + before[1] - after[1])
        self.update()

    def mouseDoubleClickEvent(self, event) -> None:
        """Double-click on a dimension = the text edit of the value
        (D5.2): an inline editor on an endpoint handle (the point's
        coordinates) or on the radius label (r). The measured LENGTH
        is read-only — its edit needs a hidden anchor choice."""
        if event.button() != Qt.LeftButton or self._selected is None:
            return
        pos = event.position()
        e = self._entity(*self._selected)
        if e is None:
            return
        # Handles first: the endpoint editor ("x, y").
        for name, hw, role in self.handles_for(e):
            if role != "point":
                continue
            hs = self.world_to_screen(*hw)
            if math.hypot(hs[0] - pos.x(), hs[1] - pos.y()) <= 8.0:
                params = params_from_entity(e)
                if name == "endpoint0":
                    initial = "{:.6g}, {:.6g}".format(params["x0"],
                                                      params["y0"])
                else:
                    initial = "{:.6g}, {:.6g}".format(params["x1"],
                                                      params["y1"])
                self._open_inline_editor(
                    QPoint(int(hs[0]) + 8, int(hs[1]) - 8), initial,
                    "point", name)
                return
        # Then the dimension labels: the radius is editable.
        for text, (wpos, value) in self.dimension_labels().items():
            if not text.startswith("r ="):
                continue
            ls = self.world_to_screen(*wpos)
            if math.hypot(ls[0] - pos.x(), ls[1] - pos.y()) <= 14.0:
                self._open_inline_editor(
                    QPoint(int(ls[0]) + 8, int(ls[1]) - 8),
                    "{:.6g}".format(value), "radius", None)
                return
        # The length label: read-only, said visibly.
        for text, (wpos, _value) in self.dimension_labels().items():
            if not text.startswith("length ="):
                continue
            ls = self.world_to_screen(*wpos)
            if math.hypot(ls[0] - pos.x(), ls[1] - pos.y()) <= 14.0:
                self._status_message = (
                    "length is a measured, read-only dimension: writing "
                    "it needs a hidden anchor choice (D5.2)")
                self.update()
                return

    # --- gesture completion (creation, D5.1) ------------------------------

    def _anchor_chain(self, point):
        """The chain of the entity whose endpoint the gesture's ANCHOR
        sits on EXACTLY (within the kernel's joint tolerance). This is
        not a snap and not a threshold: the pressed values coincide
        verbatim, and the chain reflects the values (a chain is
        ordered endpoint-to-endpoint — an entity whose start IS an
        existing endpoint belongs to that chain). Progressive drawing
        lands here; the ACCEPTED release coincidence (D5.4's explicit
        structural effect) is the other, user-facing chaining path."""
        best = None
        for ci, _ei, pt in self._all_endpoints():
            d = math.hypot(point[0] - pt[0], point[1] - pt[1])
            if d <= 1e-5 and (best is None or d < best[1]):
                best = (ci, d)
        return best[0] if best is not None else None

    def _complete_segment(self, world, modifiers) -> None:
        g = self._gesture
        a = g["a"]
        # The gesture threshold (2 px): a degenerate release refuses.
        if self.px(math.hypot(world[0] - a[0], world[1] - a[1])) < \
                GESTURE_THRESHOLD_PX:
            self._reject("segment refused: the release is within the "
                         "2 px gesture threshold of the press "
                         "(degenerate segment)")
            return
        # The suggestion: SHIFT at the release accepts the candidate,
        # no SHIFT writes the pointer's raw values verbatim (D5.4).
        b = tuple(world)
        chain_target = None
        cands = self._candidates_for_point(world, a)
        if cands and modifiers & Qt.ShiftModifier:
            kind, _d, value, _text, structural = cands[0]
            b = (float(value[0]), float(value[1]))
            if kind == "coincidence":
                chain_target = structural
        if chain_target is None:
            chain_target = self._anchor_chain(a)
        entity = self._make_entity("segment", {
            "x0": a[0], "y0": a[1], "x1": b[0], "y1": b[1],
            "interior_left": True})
        self._create_entity(entity, chain_target)

    def _complete_circle(self, world) -> None:
        g = self._gesture
        c = g["a"]
        r = math.hypot(world[0] - c[0], world[1] - c[1])
        if self.px(r) < GESTURE_THRESHOLD_PX:
            self._reject("circle refused: r = {:.6g} is within the 2 px "
                         "gesture threshold of the centre (r ~ 0)"
                         .format(r))
            return
        # New circles are OUTER by a declared, visible default (D5.1);
        # auto-classification by containment is the forbidden silent
        # write. No point-coincidence candidates apply to a radius.
        entity = self._make_entity("circle", {
            "cx": c[0], "cy": c[1], "r": r, "hole": False})
        self._create_entity(entity, None)

    def _finish_arc_radius(self, world) -> None:
        g = self._gesture
        c = g["center"]
        r = math.hypot(world[0] - c[0], world[1] - c[1])
        if self.px(r) < GESTURE_THRESHOLD_PX:
            self._reject("arc refused: r = {:.6g} is within the 2 px "
                         "gesture threshold of the centre (r ~ 0)"
                         .format(r))
            return
        a0 = math.atan2(world[1] - c[1], world[0] - c[0])
        g["phase"] = "sweep"
        g["radius"] = r
        g["a0"] = a0
        g["cursor"] = tuple(world)
        self.update()

    def _complete_arc(self, world, modifiers) -> None:
        g = self._gesture
        c, r, a0 = g["center"], g["radius"], g["a0"]
        a1 = math.atan2(world[1] - c[1], world[0] - c[0])
        sweep = (a1 - a0) % (2.0 * math.pi)
        chain_target = None
        # The coincidence candidate may land the endpoint exactly: it
        # writes r and a1 together (both named, the glyph shows both).
        cands = self._candidates_for_point(world, None,
                                            kinds=("coincidence",))
        if cands and modifiers & Qt.ShiftModifier:
            kind, _d, value, _text, structural = cands[0]
            if kind == "coincidence":
                ex, ey = value
                r = math.hypot(ex - c[0], ey - c[1])
                a1 = math.atan2(ey - c[1], ex - c[0])
                sweep = (a1 - a0) % (2.0 * math.pi)
                chain_target = structural
        # Degenerate sweep: a1 at a0 (either way around) refuses.
        start_pt = polar(c, r, a0)
        end_pt = polar(c, r, a0 + sweep)
        if math.hypot(end_pt[0] - start_pt[0],
                      end_pt[1] - start_pt[1]) < self.world(GESTURE_THRESHOLD_PX):
            self._reject("arc refused: the sweep is degenerate (a1 within "
                         "the 2 px gesture threshold of a0, or at >= 2*pi "
                         "minus that)")
            return
        entity = self._make_entity("arc", {
            "cx": c[0], "cy": c[1], "r": r, "a0": a0, "a1": a0 + sweep,
            "hole": False})
        if chain_target is None:
            # The arc's start endpoint (at a0) may sit exactly on an
            # existing endpoint (the phase-1 release landed there):
            # the same verbatim-coincidence chaining as a segment's
            # anchor.
            chain_target = self._anchor_chain(polar(c, r, a0))
        self._create_entity(entity, chain_target)

    def _make_entity(self, kind: str, params: dict):
        return make_entity(kind, params)

    def _create_entity(self, entity, chain_target) -> None:
        """The completed gesture: the kernel's class check first
        (D5.5 — no second copy), then ONE undoable write. An accepted
        coincidence appends to the connected entity's chain (the
        structural effect that closes contours); anything else opens
        (or reuses) a fresh chain."""
        error = _kernel_entity_error(entity)
        if error:
            self._reject("gesture refused: " + error)
            return
        chains = self._chains()
        if chain_target is None:
            # No structural target: a new chain, reusing an empty
            # trailing one if the recipe happens to carry it.
            chain_target = len(chains)
            if chains and not chains[-1]:
                chain_target = len(chains) - 1
        state = {"chain": None, "index": None}

        def execute():
            chain = chain_target
            if state["chain"] is not None:
                chain = state["chain"]
                if chain > len(self._chains()):
                    chain = len(self._chains())
            self._scene.sketch_add_entity(self._node_id, chain, entity)
            state["chain"] = chain
            state["index"] = len(self._chains()[chain]) - 1
            self._after_write()
            self._select((chain, state["index"]))

        def undo():
            if state["chain"] is None:
                return
            self._scene.sketch_remove_entity(
                self._node_id, state["chain"], state["index"])
            self._select(None)
            self._after_write()

        self._gesture = None
        if self._stack is not None:
            self._stack.push(execute, undo, "Sketch entity")
        else:
            execute()

    # --- handle drag commit (editing, D5.2) --------------------------------

    def _commit_drag(self, world) -> None:
        d = self._drag
        chain, index = d["entity"]
        e = self._entity(chain, index)
        if e is None:
            self._drag = None
            return
        params = params_from_entity(e)
        handle, role = d["handle"], d["role"]
        if e["kind"] == "segment":
            if handle == "endpoint0":
                params["x0"], params["y0"] = world
            else:
                params["x1"], params["y1"] = world
        elif e["kind"] == "circle":
            if handle == "center":
                params["cx"], params["cy"] = world
            else:
                params["r"] = math.hypot(world[0] - params["cx"],
                                         world[1] - params["cy"])
        else:  # arc
            if handle == "center":
                params["cx"], params["cy"] = world
            elif handle == "radius_point":
                params["r"] = math.hypot(world[0] - params["cx"],
                                         world[1] - params["cy"])
            elif handle == "endpoint_a0":
                params["a0"] = math.atan2(world[1] - params["cy"],
                                          world[0] - params["cx"])
            else:  # endpoint_a1 — r held (the pinned table)
                params["a1"] = math.atan2(world[1] - params["cy"],
                                          world[0] - params["cx"])
        self._drag = None
        # The degenerate drags refuse visibly: the radius point onto
        # the centre (r ~ 0), the a1 endpoint onto a0.
        if "r" in params and self.px(params["r"]) < GESTURE_THRESHOLD_PX:
            self._reject("drag refused: the radius point landed within "
                         "the 2 px gesture threshold of the centre "
                         "(r ~ 0) — parameters untouched")
            return
        if e["kind"] == "arc":
            sweep = (params["a1"] - params["a0"]) % (2.0 * math.pi)
            sp = polar((params["cx"], params["cy"]), params["r"], params["a0"])
            ep = polar((params["cx"], params["cy"]), params["r"],
                       params["a0"] + sweep)
            if math.hypot(ep[0] - sp[0], ep[1] - sp[1]) < \
                    self.world(GESTURE_THRESHOLD_PX):
                self._reject("drag refused: the sweep is degenerate "
                             "(a1 at a0, or at 2*pi) — parameters "
                             "untouched")
                return
        self._push_entity_edit(chain, index, e, params,
                               "Drag " + handle)

    # --- the ONE write route (gesture, drag and text) ----------------------

    def _push_entity_edit(self, chain: int, index: int, old_entity: dict,
                          new_params: dict, description: str) -> None:
        """Every parameter write goes through here — the panel's text
        route and the canvas's gesture route are ONE route (D5.2:
        one truth, two affordances). The kernel's class check runs
        BEFORE the write; the edit is ONE undo entry."""
        kind = old_entity["kind"]
        new_entity = self._make_entity(kind, new_params)
        # The undo restores the OLD entity: captured as a kernel entity
        # (the readable dict form goes back through the same factory).
        old_ent = self._make_entity(kind, params_from_entity(old_entity))
        error = _kernel_entity_error(new_entity)
        if error:
            self._reject("edit refused: " + error)
            return

        def apply(entity):
            self._scene.sketch_set_entity(self._node_id, chain, index,
                                           entity)
            self._after_write()

        if self._stack is not None:
            self._stack.push(lambda: apply(new_entity),
                             lambda: apply(old_ent), description)
        else:
            apply(new_entity)

    def edit_selected_params(self, new_params: dict, description: str) -> None:
        """The MainWindow text route (property panel / inline editor):
        writes the selected entity's parameters through the same
        command path a released drag uses."""
        if self._selected is None:
            return
        chain, index = self._selected
        self.edit_entity_params(chain, index, new_params, description)

    def edit_entity_params(self, chain: int, index: int,
                           updates: dict, description: str) -> None:
        """Write an entity's named parameters through the ONE route
        (D5.2: gesture and text are one truth, two affordances). The
        kernel's class check runs BEFORE the write; the edit is ONE
        undo entry; a refusal leaves the previous value standing."""
        e = self._entity(chain, index)
        if e is None:
            return
        merged = params_from_entity(e)
        merged.update(updates)
        self._push_entity_edit(chain, index, e, merged, description)

    # --- the inline text editor (D5.2) -------------------------------------

    def _open_inline_editor(self, at: QPoint, initial: str, mode: str,
                            handle: Optional[str]) -> None:
        self._cancel_inline_editor()
        edit = QLineEdit(self)
        edit.setText(initial)
        edit.move(at)
        edit.resize(140, 24)
        edit.setFocus()
        edit.show()
        edit.returnPressed.connect(
            lambda: self._inline_edit_done(mode, handle, True))
        edit.editingFinished.connect(
            lambda: self._inline_edit_done(mode, handle, False))
        self._inline_editor = edit
        self._inline_mode = mode
        self._inline_handle = handle

    def _cancel_inline_editor(self) -> None:
        if self._inline_editor is not None:
            edit = self._inline_editor
            self._inline_editor = None
            self._inline_mode = None
            self._inline_handle = None
            edit.deleteLater()
            self.update()

    def _inline_edit_done(self, mode, handle, from_return: bool) -> None:
        edit = self._inline_editor
        if edit is None or self._inline_mode != mode or \
                self._inline_handle != handle:
            return
        text = edit.text().strip()
        if not from_return:
            # editingFinished also fires on focus-out: an edit that
            # was not committed by Return writes nothing (abort).
            self._cancel_inline_editor()
            return
        self._inline_editor = None
        self._inline_mode = None
        self._inline_handle = None
        edit.deleteLater()
        if self._selected is None:
            return
        chain, index = self._selected
        e = self._entity(chain, index)
        if e is None:
            return
        try:
            if mode == "point":
                parts = text.replace(",", " ").split()
                if len(parts) != 2:
                    raise ValueError
                x = float(parts[0])
                y = float(parts[1])
                if not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError
                if handle == "endpoint0":
                    self.edit_selected_params(
                        {"x0": x, "y0": y}, "Edit endpoint 0")
                else:
                    self.edit_selected_params(
                        {"x1": x, "y1": y}, "Edit endpoint 1")
            else:
                r = float(text)
                if not math.isfinite(r):
                    raise ValueError
                self.edit_selected_params({"r": r}, "Edit radius")
        except ValueError:
            # The field refuses: r <= 0, NaN, empty or malformed input
            # leaves the previous value standing (D5.5).
            self._reject("text edit refused: '{}' is not a finite "
                         "value the recipe can take — the previous "
                         "value stands".format(text))

    # --- selection -----------------------------------------------------------

    def _select(self, target) -> None:
        self._selected = target
        self.entity_selected.emit(target)
        self.update()

    def select_entity(self, chain: Optional[int], index: Optional[int]) -> None:
        """Programmatic selection (tests, main window)."""
        self._select((chain, index) if chain is not None else None)

    # --- shared helpers -------------------------------------------------------

    def _abort_any_interaction(self) -> None:
        """ESC / right-click: no entity, no write, no undo entry, the
        parameters untouched. An abort is not an undo."""
        self._cancel_inline_editor()
        changed = self._gesture is not None or self._drag is not None or \
            self._pan is not None
        self._gesture = None
        self._drag = None
        self._pan = None
        if changed:
            self.update()

    def _after_write(self) -> None:
        """The post-write pass: the DAG re-materializes every
        dependent (the established propagation), the view repaints,
        the main window refreshes its rows."""
        self._scene.rebuild_operations()
        self.update()
        self.sketch_edited.emit()

    # --- rendering ------------------------------------------------------------

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), COLOR_BACKGROUND)
        self._draw_grid(p)
        aspects = self.entity_aspects()
        chains = self._chains()
        for ci, chain in enumerate(chains):
            for ei, e in enumerate(chain):
                self._draw_entity(p, e, aspects.get((ci, ei),
                                                    "outer_closed"),
                                  selected=self._selected == (ci, ei))
        self._draw_gaps(p)
        self._draw_dimension_labels(p)
        self._draw_preview(p)
        self._draw_glyph(p)
        self._draw_handles(p)
        self._draw_status(p)
        p.end()

    def _draw_grid(self, p) -> None:
        pitch_px = self.px(GRID_PITCH)
        if pitch_px < 4.0:
            return  # below one dot per 4 px: nothing legible to draw
        x0, _y0 = self.screen_to_world(0, 0)
        x1, _y1 = self.screen_to_world(self.width(), self.height())
        i0, i1 = math.ceil(x0 / GRID_PITCH), math.floor(x1 / GRID_PITCH)
        j0, j1 = math.ceil(_y1 / GRID_PITCH), math.floor(_y0 / GRID_PITCH)
        if (i1 - i0) * (j1 - j0) > 4000:
            return  # the same legibility guard, hard-capped
        p.setPen(COLOR_GRID)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                sx, sy = self.world_to_screen(i * GRID_PITCH, j * GRID_PITCH)
                p.drawPoint(QPoint(int(sx), int(sy)))

    def _pen(self, color: QColor, width: float, style=None):
        pen = QPen(color, int(width))
        if style == "dash":
            pen.setStyle(Qt.DashLine)
            pen.setDashPattern([2, 2])
        elif style == "amber_dash":
            pen.setStyle(Qt.CustomDashLine)
            pen.setDashPattern([6, 3])
        return pen

    def _draw_entity(self, p, e: dict, aspect: str,
                     selected: bool) -> None:
        if selected:
            pen = self._pen(COLOR_SELECTED, 3)
        elif aspect == "under_defined":
            pen = self._pen(COLOR_UNDER_DEFINED, 2, "amber_dash")
        elif aspect == "hole_closed":
            pen = self._pen(COLOR_HOLE_CLOSED, 2)
        else:
            pen = self._pen(COLOR_OUTER_CLOSED, 2)
        p.setPen(pen)
        self._stroke_entity(p, e)

    def _stroke_entity(self, p, e: dict) -> None:
        if e["kind"] == "segment":
            a = self.world_to_screen(*e["a"])
            b = self.world_to_screen(*e["b"])
            p.drawLine(QPoint(int(a[0]), int(a[1])),
                       QPoint(int(b[0]), int(b[1])))
        elif e["kind"] == "circle":
            c = self.world_to_screen(*e["center"])
            r = self.px(e["radius"])
            p.drawEllipse(QPointF(c[0], c[1]), r, r)
        else:
            a0, sweep = e["start_angle"], e["sweep"]
            steps = max(8, int(ARC_RENDER_SEGMENTS * sweep /
                               (2.0 * math.pi)))
            pts = [self.world_to_screen(*polar(e["center"], e["radius"],
                                               a0 + sweep * k / steps))
                   for k in range(steps + 1)]
            for k in range(steps):
                p.drawLine(QPoint(int(pts[k][0]), int(pts[k][1])),
                           QPoint(int(pts[k + 1][0]), int(pts[k + 1][1])))

    def _draw_preview(self, p) -> None:
        """Category (b): the transient gesture/drag preview — grey,
        dash [2,2], 1 px. It writes nothing; it is not in the recipe."""
        p.setPen(self._pen(COLOR_PREVIEW, 1, "dash"))
        g = self._gesture
        if g is not None:
            if g["kind"] == "segment":
                a = self.world_to_screen(*g["a"])
                b = self.world_to_screen(*g["cursor"])
                p.drawLine(QPoint(int(a[0]), int(a[1])),
                           QPoint(int(b[0]), int(b[1])))
            elif g["kind"] == "circle":
                c = self.world_to_screen(*g["a"])
                r = self.px(math.hypot(g["cursor"][0] - g["a"][0],
                                       g["cursor"][1] - g["a"][1]))
                p.drawEllipse(QPointF(c[0], c[1]), r, r)
            elif g["kind"] == "arc":
                c = self.world_to_screen(*g["center"])
                if g["phase"] == "radius":
                    a = self.world_to_screen(*g["cursor"])
                    p.drawLine(QPoint(int(c[0]), int(c[1])),
                               QPoint(int(a[0]), int(a[1])))
                else:
                    r = self.px(g["radius"])
                    p.drawEllipse(QPointF(c[0], c[1]), r, r)
                    sweep = (math.atan2(g["cursor"][1] - g["center"][1],
                                        g["cursor"][0] - g["center"][0])
                             - g["a0"]) % (2.0 * math.pi)
                    steps = max(8, int(ARC_RENDER_SEGMENTS * sweep /
                                       (2.0 * math.pi)))
                    pts = [self.world_to_screen(*polar(g["center"],
                                                       g["radius"],
                                                       g["a0"] + sweep * k
                                                       / steps))
                           for k in range(steps + 1)]
                    for k in range(steps):
                        p.drawLine(
                            QPoint(int(pts[k][0]), int(pts[k][1])),
                            QPoint(int(pts[k + 1][0]), int(pts[k + 1][1])))
        d = self._drag
        if d is not None:
            e = self._entity(*d["entity"])
            if e is not None:
                params = params_from_entity(e)
                handle = d["handle"]
                w = d["cursor"]
                if e["kind"] == "segment":
                    if handle == "endpoint0":
                        params["x0"], params["y0"] = w
                    else:
                        params["x1"], params["y1"] = w
                elif e["kind"] == "circle":
                    if handle == "center":
                        params["cx"], params["cy"] = w
                    else:
                        params["r"] = math.hypot(w[0] - params["cx"],
                                                 w[1] - params["cy"])
                else:
                    if handle == "center":
                        params["cx"], params["cy"] = w
                    elif handle == "radius_point":
                        params["r"] = math.hypot(w[0] - params["cx"],
                                                 w[1] - params["cy"])
                    elif handle == "endpoint_a0":
                        params["a0"] = math.atan2(w[1] - params["cy"],
                                                  w[0] - params["cx"])
                    else:
                        params["a1"] = math.atan2(w[1] - params["cy"],
                                                  w[0] - params["cx"])
                self._stroke_entity(p, self._make_entity(e["kind"], params))

    def _draw_glyph(self, p) -> None:
        """Category (c): the acceptance glyph — the nearest candidate
        as a 5 px marker plus THE VALUE IT WOULD WRITE as text (D5.4:
        seeing what you accept is the rule)."""
        cands = self._active_suggestion()
        if not cands:
            return
        _kind, _d, value, text, _structural = cands[0]
        s = self.world_to_screen(*value)
        pen = QPen(COLOR_GLYPH, 1)
        p.setPen(pen)
        p.drawEllipse(QPointF(s[0], s[1]), 5.0, 5.0)
        p.drawText(QPoint(int(s[0]) + 8, int(s[1]) - 8),
                   "SHIFT: " + text)

    def _draw_handles(self, p) -> None:
        if self._selected is None:
            return
        e = self._entity(*self._selected)
        if e is None:
            return
        p.setPen(QPen(COLOR_SELECTED, 1))
        p.setBrush(COLOR_SELECTED)
        for _name, hw, _role in self.handles_for(e):
            s = self.world_to_screen(*hw)
            p.drawRect(int(s[0]) - 3, int(s[1]) - 3, 6, 6)

    def _draw_dimension_labels(self, p) -> None:
        p.setPen(QColor("#BDBDBD"))
        for text, (wpos, _value) in self.dimension_labels().items():
            s = self.world_to_screen(*wpos)
            p.drawText(QPoint(int(s[0]) + 6, int(s[1]) - 6), text)

    def _draw_gaps(self, p) -> None:
        """The distance to closure, on the entity that carries the
        gap (D5.3): the amber worklight, a measure not a judgment."""
        p.setPen(COLOR_UNDER_DEFINED)
        for _ci, mid, dist in self.open_chain_gaps():
            s = self.world_to_screen(*mid)
            p.drawText(QPoint(int(s[0]), int(s[1]) + 14),
                       "gap {:.4g}".format(dist))

    def _draw_status(self, p) -> None:
        """The pinned rejection channel: #FF6E6E, in the canvas status
        bar, persisting until the next gesture begins (D5.5)."""
        if not self._status_message:
            return
        p.setPen(COLOR_REJECTION)
        p.drawText(self.rect().adjusted(8, 0, -8, -8),
                   Qt.AlignBottom | Qt.AlignLeft | Qt.TextWordWrap,
                   self._status_message)
