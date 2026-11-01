"""H2 unit 6, morse 6 (FINALE) — IL QUARTETTO D5 through the bindings
(ADR-0036 D5 + D6.5, the accepted act): the exit criterion of the
unit — the naive demonstrated failing, the name demonstrated holding.

The C++ side pins the whole quartet (tests/unit/brep/
test_h2_u6_m6_d5_quartet.cpp — the regeneration with the co-edge leg,
the two topological edits with the coherent tavole, the pair
class-1/class-4, the naive counterproof with the mis-reference
captured). What THESE tests pin is what crosses the binding intact —
the maintainer's own click-test included:

  * the guide case built, the RADIUS EDIT executed, the resolution
    interrogated, the class REPORT read as text (class 1, silent —
    and the report says so in the words the UI and the feature's log
    read);
  * THE CLICK-TEST, dal vivo: the D5.4 scenario built THROUGH THE
    SCENE — the DAG ids wired (the live body answers
    ``sub#<id>:isect[ext#<id>:cap(hi) × ext#<id>:lat(0)]@hi``) — the
    rim picked once, the square sketch edited into the PENTAGON (the
    closing segment redirected, one entity appended: the add-at-end
    flow, the covered family), the DAG rebuilt, the reference
    RESOLVED class 1, and the downstream feature re-running on the
    new geometry WITHOUT reselection. THE PAIR, in the same test: the
    queried face shows the name of the rim CHE HA TENUTO, and the
    query by position of the OLD id shows L'ENTITÀ SBAGLIATA (the
    pentagon's last vertical, unnamed) — the visible pair is the
    proof of the unit: red-on-address, green-on-name.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import geometry as geom_mod
from CAD_0 import scene as scene_mod

Vec3f = CAD_0.math.Vec3f
V3 = Vec3f

RIM_HI = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"


def _circle_chains(radius=0.25):
    return [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), radius, False)])]


def _square_target_chains(h=0.5):
    """The D5.4 target: the square prism's sketch (chain of 4 — the
    same convex planar class as the box)."""
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((-h, -h), (h, -h), True),
        scene_mod.sketch_segment((h, -h), (h, h), True),
        scene_mod.sketch_segment((h, h), (-h, h), True),
        scene_mod.sketch_segment((-h, h), (-h, -h), True),
    ])]


def _pentagon_target_chains(h=0.5):
    """THE EDIT (the add-at-end family): the same first three segments
    (the roots 0..2 untouched), the closing segment REDIRECTED to the
    new vertex P (a parameter edit on root 3), ONE segment appended
    (P back to the start — root 4 born). No root moved."""
    P = (-0.85, 0.0)
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((-h, -h), (h, -h), True),
        scene_mod.sketch_segment((h, -h), (h, h), True),
        scene_mod.sketch_segment((h, h), (-h, h), True),
        scene_mod.sketch_segment((-h, h), P, True),
        scene_mod.sketch_segment(P, (-h, -h), True),
    ])]


def _guide_case(chains):
    """The D1 chain through the bindings: box#1 -> sk#2 -> ext#3 ->
    sub#4 (the m5 pattern, the tool recipe the EDIT KNOB)."""
    box = brep_mod.make_box(V3(1.0, 1.0, 1.0), feature=1)
    tool = brep_mod.make_prism(chains, 3.0, +1, feature=3)
    result = brep_mod.exact_subtract(
        box, tool, V3(0.0, 0.0, -1.5), feature=4,
        target=brep_mod.FeatureKind.Box,
        tool=brep_mod.FeatureKind.Extrude)
    return box, tool, result


_SUB_OPS = [(1, brep_mod.FeatureKind.Box),
            (3, brep_mod.FeatureKind.Extrude)]


def _subtract_generations(old, new, kind=brep_mod.FeatureKind.Subtract,
                          operands=_SUB_OPS):
    """The generations the resolver reads (the m5 pattern)."""
    box_o, tool_o, result_o = old
    box_n, tool_n, result_n = new
    tav = brep_mod.subtract_descent(result_o, box_o, tool_o)
    gen_old = brep_mod.Generation(None, tav, kind, operands)
    gen_new = brep_mod.Generation(result_n, None, kind, operands)
    return gen_old, gen_new


def test_d53_the_radius_edit_the_report_as_text():
    """The guide case built, the radius edit executed, the resolution
    interrogated, the class report read as text: class 1, silent — the
    feature re-runs without reselection, the new geometry live at the
    resolved address (the seam-wired pair answering)."""
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_circle_chains(0.30))
    gen_old, gen_new = _subtract_generations(old, new)

    pick = brep_mod.edge_query(old[2], 14)
    assert pick["name"] == RIM_HI
    ref = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 0, 0])

    ctx = brep_mod.EditContext(no_shift=True)  # the parameter edit
    r = brep_mod.resolve(ref, gen_old, gen_new, ctx)

    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["name"] == RIM_HI
    # THE REPORT AS TEXT — the words the UI and the feature's log read.
    assert r["text"].startswith("reference_resolved: " + RIM_HI)
    assert "silently" in r["text"]

    # The feature re-runs on the new geometry: the pair answers, the
    # new radius is live, the old radius is gone.
    ids = brep_mod.query_edge_by_name(new[2], ref)
    assert ids == [14, 15]
    top = brep_mod.face_query(new[2], 1)
    assert top["inner_loop_count"] == 1
    radii = [ce["pcurve"]["radius"]
             for loop in top["loops"] for ce in loop["coedges"]
             if ce["pcurve"] is not None
             and ce["pcurve"]["type"] == "arc"]
    assert radii and all(abs(x - 0.30) < 1e-6 for x in radii)
    assert all(abs(x - 0.25) > 1e-6 for x in radii)


def test_d54_click_test_the_pentagon_red_on_address_green_on_name():
    """THE CLICK-TEST, dal vivo: the D5.4 scenario through the scene —
    the DAG ids wired — the rim picked once, the square edited into
    the pentagon (the add-at-end flow), the DAG rebuilt, the reference
    RESOLVED class 1, the feature re-running WITHOUT reselection. THE
    PAIR, in the same test: the name answers at the NEW addresses
    while the OLD address answers the WRONG entity — the visible pair
    is the proof of the unit."""
    s = scene_mod.Scene()
    sk1 = s.add_sketch_node("Target", _square_target_chains())
    ex1 = s.add_extrude_node("Ext2", sk1, 1.0, 1)
    sk2 = s.add_sketch_node("Tool", _circle_chains())
    ex2 = s.add_extrude_node("Ext3", sk2, 3.0, 1)
    sub = s.add_operation_node("Sub", ex1, ex2, 0.0, 0.0, -1.5)
    node = s.get_node(sub)
    assert node.exact_brep is not None

    # THE WIRING'S OWN PIN: the live body answers the DAG names.
    live = node.exact_brep
    rim_name = ("sub#%d:isect[ext#%d:cap(hi) × ext#%d:lat(0)]@hi"
                % (sub, ex1, ex2))
    pick = brep_mod.edge_query(live, 14)
    assert pick["name"] == rim_name
    assert brep_mod.face_query(live, 5)["name"] == "ext#%d:cap(hi)" % ex1
    assert brep_mod.face_query(live, 6)["name"] == "ext#%d:lat(0)" % ex2

    # The scenario's counts, pinned: the prism target is the box's own
    # class (12/17/7/2, chi = 0); the born half-arcs at 12-15 (E_A =
    # 12), the @hi pair at 14/15 — the pair the reference answers.
    before = brep_mod.body_query(live)
    assert (before["V"], before["E"], before["F"]) == (12, 17, 7)
    assert before["chi"] == 0
    assert brep_mod.query_edge_by_name(live, geom_mod.ShapeId(
        geom_mod.FeatureId(sub), pick["shape_id"]["path"])) == [14, 15]

    # The downstream feature's reference, picked ONCE (the click's own
    # structural answer).
    reference = geom_mod.ShapeId(
        geom_mod.FeatureId(sub), pick["shape_id"]["path"])

    # The OLD generation's tavola, derived before the edit (a value).
    tool_body = s.get_node(ex2).exact_brep
    target_body = s.get_node(ex1).exact_brep
    old_tav = brep_mod.subtract_descent(live, target_body, tool_body)
    assert len(old_tav) == 47  # the guide case's own count

    # THE EDIT: the closing segment redirected + one appended (the
    # canvas's add-at-end flow: no root moved, root 4 born) — then the
    # DAG rebuild pass.
    s.sketch_set_entity(sk1, 0, 3, scene_mod.sketch_segment(
        (-0.5, 0.5), (-0.85, 0.0), True))
    s.sketch_add_entity(sk1, 0, scene_mod.sketch_segment(
        (-0.85, 0.0), (-0.5, -0.5), True))
    out = s.rebuild_operations()
    assert out["rebuilt"] >= 1 and not out["dangling"]
    new_live = s.get_node(sub).exact_brep
    assert new_live is not None

    # THE COUNTS moved exactly as the pinned formula says:
    # V: 8+4 -> 10+4, E: 12+5 -> 15+5, F: 6+1 -> 7+1, chi = 0.
    after = brep_mod.body_query(new_live)
    assert (after["V"], after["E"], after["F"]) == (14, 20, 8)
    assert after["chi"] == 0

    # THE RESOLUTION: the old tavola + the new body + the edit context
    # (the add-at-end flow's no-shift declaration) — class 1, silent.
    ops = [(ex1, brep_mod.FeatureKind.Extrude),
           (ex2, brep_mod.FeatureKind.Extrude)]
    gen_old = brep_mod.Generation(
        None, old_tav, brep_mod.FeatureKind.Subtract, ops)
    gen_new = brep_mod.Generation(
        new_live, None, brep_mod.FeatureKind.Subtract, ops)
    ctx = brep_mod.EditContext(no_shift=True)
    r = brep_mod.resolve(reference, gen_old, gen_new, ctx)

    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["name"] == rim_name  # the DAG name, unchanged
    assert r["text"].startswith("reference_resolved: " + rim_name)

    # ---- GREEN ON NAME: the feature re-runs WITHOUT reselection ----
    # The resolved name answers the seam-wired PAIR at the NEW
    # addresses (17/18), and the rim is live right there (the pierced
    # cap(hi)'s inner loop, radius 0.25).
    ids = brep_mod.query_edge_by_name(new_live, reference)
    assert ids == [17, 18]
    cap_hi = brep_mod.face_query(new_live, 6)
    assert cap_hi["name"] == "ext#%d:cap(hi)" % ex1  # the face held
    assert cap_hi["inner_loop_count"] == 1
    radii = [ce["pcurve"]["radius"]
             for loop in cap_hi["loops"] for ce in loop["coedges"]
             if ce["pcurve"] is not None
             and ce["pcurve"]["type"] == "arc"]
    assert radii and all(abs(x - 0.25) < 1e-6 for x in radii)

    # ---- RED ON ADDRESS: the OLD id shows L'ENTITÀ SBAGLIATA ----
    # The entity that today answers to the id 14 is the pentagon's
    # LAST VERTICAL edge (the closing joint): unnamed (honestly empty
    # — the prism's verticals are outside v1's extrude vocabulary),
    # the WRONG endpoints — a fillet that referenced by address would
    # now silently fillet the wrong edge. THE PAIR, in one place.
    wrong = brep_mod.edge_query(new_live, 14)
    assert wrong["name"] == ""            # not the rim — not nameable
    assert wrong["v0"] == 8 and wrong["v1"] == 9
    assert wrong["shape_id"]["path"] == []  # no identity at all
    assert brep_mod.edge_query(new_live, 17)["name"] == rim_name
    # The mis-reference, captured: the old address's entity is NOT the
    # referenced rim (a different kind of edge entirely — a kept
    # vertical of the target, not a born arc of the subtract).
    assert wrong["shape_id"]["path"] != pick["shape_id"]["path"]
    assert pick["v0"] != wrong["v0"]
