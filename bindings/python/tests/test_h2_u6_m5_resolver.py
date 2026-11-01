"""H2 unit 6, morse 5 — THE RESOLVER through the bindings
(ADR-0036 D3 + D6.4, the accepted act).

The C++ side pins the whole contract (tests/unit/brep/
test_h2_u6_m5_resolver.cpp — the four classes with their payloads,
the guard with the injected defect, the roots rule with the door
closed, the compile probes, the two surfaces). What THESE tests pin
is what crosses the binding intact — the maintainer's own
click-test included:

  * ``CAD_0.brep.resolve`` — the machine itself, on the guide case:
    the class-1 silence (the radius edit, the feature re-runs),
    the class-3 payload (lat(0) E lat(1) nominated), the class-4
    payload (the dead rim, the cause, the remedy), the class-2
    machinery (the map declared, synthetically), the guard (the
    closure defect, never a "not found");
  * THE CLICK-TEST, dal vivo: the guide case built THROUGH THE
    SCENE — the DAG ids wired (the wiring's own pin: the live body
    answers ``sub#<id>:isect[box#<id>:top × ext#<id>:lat(0)]@hi``)
    — the rim picked once, the radius edited through the sketch
    surface, the DAG rebuilt, the reference RESOLVED, and the
    downstream feature re-running on the new geometry WITHOUT
    reselection (the by-name query answering the seam-wired PAIR);
  * the resolution's REPORT as text — the words the UI and the
    feature's log read;
  * the two query surfaces: ``query_edge_by_name`` /
    ``query_face_by_name`` (the structural name -> the addresses,
    the loud miss) beside the untouched positional queries (the
    pick's own door).

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import pytest

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


def _lens_chains():
    """The two-arc chain (the circle re-authored): two circles
    crossing at both endpoints — the honest two-arc class (the
    same-circle split is refused by the certificate)."""
    r = 0.3
    c1, c2 = (-0.12, 0.0), (0.12, 0.0)
    y = math.sqrt(r * r - 0.12 * 0.12)
    a, bb = (0.0, y), (0.0, -y)
    ang_b1 = math.atan2(bb[1] - c1[1], bb[0] - c1[0])
    ang_a1 = math.atan2(a[1] - c1[1], a[0] - c1[0])
    span1 = ang_a1 - ang_b1
    if span1 < 0.0:
        span1 += 2.0 * math.pi
    ang_a2 = math.atan2(a[1] - c2[1], a[0] - c2[0])
    ang_b2 = math.atan2(bb[1] - c2[1], bb[0] - c2[0])
    span2 = ang_b2 - ang_a2
    if span2 < 0.0:
        span2 += 2.0 * math.pi
    return [scene_mod.SketchChain([
        scene_mod.sketch_arc(c1, r, False, ang_b1, span1),
        scene_mod.sketch_arc(c2, r, False, ang_a2, span2),
    ])]


def _square_chains(half=0.2):
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((-half, -half), (half, -half), True),
        scene_mod.sketch_segment((half, -half), (half, half), True),
        scene_mod.sketch_segment((half, half), (-half, half), True),
        scene_mod.sketch_segment((-half, half), (-half, -half), True),
    ])]


def _d_chains():
    """The 'D' profile: the diameter segment + the upper half-arc
    (the arc at ROOT 1 — the entity the root map is about)."""
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((-1.0, 0.0), (1.0, 0.0), True),
        scene_mod.sketch_arc((0.0, 0.0), 1.0, False, 0.0, math.pi),
    ])]


def _d_split_chains():
    """The split 'D': the diameter split in three — the SAME arc now
    at ROOT 3 (the shift: 1 -> 3)."""
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((-1.0, 0.0), (-1.0 / 3.0, 0.0), True),
        scene_mod.sketch_segment((-1.0 / 3.0, 0.0), (1.0 / 3.0, 0.0), True),
        scene_mod.sketch_segment((1.0 / 3.0, 0.0), (1.0, 0.0), True),
        scene_mod.sketch_arc((0.0, 0.0), 1.0, False, 0.0, math.pi),
    ])]


def _guide_case(chains):
    """The D1 chain through the bindings: box#1 -> sk#2 -> ext#3 ->
    sub#4 (the m4 pattern, the tool recipe the EDIT KNOB)."""
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
    """The generations the resolver reads: the old side (the tavola
    + the grammar), the new side (the body + the grammar). The
    grammar is the REFERENCE's own — the subtract's instance
    registry for its born entities (the rims, the seam), the
    ORIGINAL creator's for an inherited one (the wall is the
    EXTRUDE's face)."""
    box_o, tool_o, result_o = old
    box_n, tool_n, result_n = new
    tav = brep_mod.subtract_descent(result_o, box_o, tool_o)
    gen_old = brep_mod.Generation(None, tav, kind, operands)
    gen_new = brep_mod.Generation(result_n, None, kind, operands)
    return gen_old, gen_new


def test_class_1_the_radius_edit_resolves_silently():
    """The radius grows within the class: the rim re-derives
    identically (a STRUCTURAL fact) — reference_resolved, and the
    feature re-runs WITHOUT reselection, the new geometry live at
    the resolved address (the seam-wired PAIR answering)."""
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_circle_chains(0.30))
    gen_old, gen_new = _subtract_generations(old, new)

    # THE PICK: the reference is the rim's own structural answer.
    pick = brep_mod.edge_query(old[2], 14)
    assert pick["name"] == RIM_HI
    ref = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 0, 0])
    assert pick["shape_id"]["path"] == [1, 1, 0, 0]

    ctx = brep_mod.EditContext(no_shift=True)  # the parameter edit
    r = brep_mod.resolve(ref, gen_old, gen_new, ctx)

    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["id"]["path"] == [1, 1, 0, 0]
    assert r["name"] == RIM_HI
    assert r["text"].startswith("reference_resolved: " + RIM_HI)

    # The feature re-runs on the new geometry: the by-name surface
    # answers the PAIR (one logical entity, both halves).
    ids = brep_mod.query_edge_by_name(new[2], ref)
    assert ids == [14, 15]
    # The new radius, live: the pierced face's inner loop arcs.
    top = brep_mod.face_query(new[2], 1)
    assert top["inner_loop_count"] == 1
    radii = [ce["pcurve"]["radius"]
             for loop in top["loops"] for ce in loop["coedges"]
             if ce["pcurve"] is not None
             and ce["pcurve"]["type"] == "arc"]
    assert radii and all(abs(x - 0.30) < 1e-6 for x in radii)


def test_class_3_the_two_arcs_are_ambiguous_and_nominated():
    """The circle re-authored as a chain of two arcs: the wall
    reference resolves AMBIGUOUS — lat(0) E lat(1) nominated, the
    remedy with them, never a coin flip."""
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_lens_chains())
    assert brep_mod.face_count(new[2]) == 8  # ONE wall became TWO
    # The wall's grammar is the EXTRUDE's — the ORIGINAL creator's
    # (an inherited entity: the subtract's face table is honestly
    # empty, it births no faces).
    gen_old, gen_new = _subtract_generations(
        old, new, kind=brep_mod.FeatureKind.Extrude, operands=[])

    pick = brep_mod.face_query(old[2], 6)
    assert pick["name"] == "ext#3:lat(0)"
    ref = geom_mod.ShapeId(geom_mod.FeatureId(3), [0, 0])

    r = brep_mod.resolve(ref, gen_old, gen_new, brep_mod.EditContext())

    assert r["kind"] == "ambiguous"
    assert r["code"] == "reference_ambiguous"
    assert r["candidate_names"] == ["ext#3:lat(0)", "ext#3:lat(1)"]
    assert [c["path"] for c in r["candidates"]] == [[0, 0], [0, 1]]
    assert r["entity_name"] == "ext#3:lat(0)"
    assert "reselect" in r["remedy"]
    # The report names every candidate, verbatim.
    assert "ext#3:lat(0)" in r["text"] and "ext#3:lat(1)" in r["text"]


def test_class_4_the_square_chain_vanishes_with_cause_and_remedy():
    """The circle replaced by a square chain: the rim reference
    resolves VANISHED — what died (the row), why (the authored
    entity was replaced), the remedy. Never a mute 'not found'."""
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_square_chains())
    gen_old, gen_new = _subtract_generations(old, new)

    pick = brep_mod.edge_query(old[2], 14)
    assert pick["name"] == RIM_HI
    ref = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 0, 0])

    r = brep_mod.resolve(ref, gen_old, gen_new, brep_mod.EditContext())

    assert r["kind"] == "vanished"
    assert r["code"] == "reference_vanished"
    assert r["row"]["name"] == RIM_HI
    assert r["row"]["relation"] == "born"
    assert r["why"] == "the authored entity was replaced"
    assert "reselect" in r["remedy"] and "revert" in r["remedy"]
    assert "the authored entity was replaced" in r["text"]


def test_class_2_the_machinery_with_the_map_declared():
    """The synthetic exerciser (THE REGISTER NOTE: v1 authored flows
    produce no instance — registered boundary, ADR-0036 D3.2; the
    machinery and this test ship, the real maps arrive with the
    real cases): one living descendant, a different name, the map
    declared — reference_renamed."""
    old_tool = brep_mod.make_prism(_d_chains(), 2.0, +1, feature=3)
    new_tool = brep_mod.make_prism(_d_split_chains(), 2.0, +1,
                                   feature=3)
    assert brep_mod.face_query(old_tool, 1)["name"] == "ext#3:lat(1)"
    assert brep_mod.face_query(new_tool, 3)["name"] == "ext#3:lat(3)"

    tav = brep_mod.extrude_descent(old_tool, _d_chains(), 3, 2)
    gen_old = brep_mod.Generation(
        old_tool, tav, brep_mod.FeatureKind.Extrude)
    gen_new = brep_mod.Generation(
        new_tool, None, brep_mod.FeatureKind.Extrude)

    ref = geom_mod.ShapeId(geom_mod.FeatureId(3), [0, 1])
    ctx = brep_mod.EditContext(root_map=[(0, 1, 0, 3)], sketch=2)

    r = brep_mod.resolve(ref, gen_old, gen_new, ctx)

    assert r["kind"] == "renamed"
    assert r["code"] == "reference_renamed"
    assert r["from"]["path"] == [0, 1]
    assert r["to"]["path"] == [0, 3]
    assert r["from_name"] == "ext#3:lat(1)"
    assert r["to_name"] == "ext#3:lat(3)"
    assert r["map"] == "sk#2:1 -> sk#2:3"
    assert "sk#2:1 -> sk#2:3" in r["text"]
    # The renamed reference ANSWERS on the new body.
    assert brep_mod.query_face_by_name(new_tool,
                                       geom_mod.ShapeId(geom_mod.FeatureId(3), [0, 3])) == [3]


def test_the_guard_calls_the_orphan_a_defect():
    """A reference with NO ROW is a closure defect — surfaced as the
    defect it is, never a 'not found'."""
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_circle_chains(0.30))
    gen_old, gen_new = _subtract_generations(old, new)

    ghost = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 7, 7])
    r = brep_mod.resolve(ghost, gen_old, gen_new, brep_mod.EditContext())

    assert r["kind"] == "closure_defect"
    assert r["code"] == "closure_defect"
    assert r["orphan"]["path"] == [1, 1, 7, 7]
    assert "closure violation" in r["text"]
    assert "not a user error" in r["text"]


def test_the_roots_rule_the_door_is_closed():
    """The shift WITHOUT a map never produces a positional match:
    the honest class (ambiguous, all four walls nominated), never
    the new lat(3) answering alone for the old lat(1) — and never
    the positional lat(1) -> lat(1) either."""
    old_tool = brep_mod.make_prism(_d_chains(), 2.0, +1, feature=3)
    new_tool = brep_mod.make_prism(_d_split_chains(), 2.0, +1,
                                   feature=3)
    tav = brep_mod.extrude_descent(old_tool, _d_chains(), 3, 2)
    gen_old = brep_mod.Generation(
        old_tool, tav, brep_mod.FeatureKind.Extrude)
    gen_new = brep_mod.Generation(
        new_tool, None, brep_mod.FeatureKind.Extrude)

    ref = geom_mod.ShapeId(geom_mod.FeatureId(3), [0, 1])
    r = brep_mod.resolve(ref, gen_old, gen_new, brep_mod.EditContext())

    assert r["kind"] == "ambiguous"
    assert r["candidate_names"] == [
        "ext#3:lat(0)", "ext#3:lat(1)", "ext#3:lat(2)", "ext#3:lat(3)"]

    # The benign radius edit itself needs the declaration: without
    # it the resolver does not assume the edit's class.
    old = _guide_case(_circle_chains(0.25))
    new = _guide_case(_circle_chains(0.30))
    gen_old2, gen_new2 = _subtract_generations(old, new)
    rim = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 0, 0])
    r2 = brep_mod.resolve(rim, gen_old2, gen_new2,
                          brep_mod.EditContext())
    assert r2["kind"] == "ambiguous"
    assert r2["candidate_names"] == [RIM_HI]


def test_the_by_name_surface_beside_the_positional_one():
    """Two planes, two doors: the by-name queries (the seam-wired
    pair answers BOTH halves; the loud miss refuses) beside the
    untouched positional queries (the pick's own door)."""
    box, tool, result = _guide_case(_circle_chains(0.25))

    rim = geom_mod.ShapeId(geom_mod.FeatureId(4), [1, 1, 0, 0])
    seam = geom_mod.ShapeId(geom_mod.FeatureId(4), [2, 0, 0])
    wall = geom_mod.ShapeId(geom_mod.FeatureId(3), [0, 0])

    assert brep_mod.query_edge_by_name(result, rim) == [14, 15]
    assert brep_mod.query_edge_by_name(result, seam) == [16]
    assert brep_mod.query_face_by_name(result, wall) == [6]

    # The pick's door, intact: the address -> the name at pick time.
    assert brep_mod.edge_query(result, 14)["name"] == RIM_HI
    assert brep_mod.face_query(result, 6)["name"] == "ext#3:lat(0)"

    # The loud miss: a name this body does not carry refuses with
    # the diagnosis and the remedy — never an empty answer.
    foreign = geom_mod.ShapeId(geom_mod.FeatureId(9), [1, 1, 0, 0])
    with pytest.raises(ValueError) as ei:
        brep_mod.query_edge_by_name(result, foreign)
    assert "remedy" in str(ei.value)
    with pytest.raises(ValueError):
        brep_mod.query_face_by_name(result, foreign)


def test_the_click_test_the_scene_the_edit_the_feature_re_runs():
    """THE CLICK-TEST, dal vivo: the guide case built THROUGH THE
    SCENE — the DAG ids wired through the hybrid materialization
    (the live body answers the DAG names) — the rim picked once,
    the radius edited through the sketch surface, the DAG rebuilt,
    the reference RESOLVED, and the downstream feature re-running
    on the new geometry WITHOUT reselection."""
    s = scene_mod.Scene()
    box_id = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    sk_id = s.add_sketch_node(
        "S", [scene_mod.SketchChain(
            [scene_mod.sketch_circle((0.0, 0.0), 0.25, False)])])
    ex_id = s.add_extrude_node("Extrude", sk_id, 3.0, 1)
    sub_id = s.add_operation_node("Sub", box_id, ex_id, 0.0, 0.0,
                                  -1.5)
    node = s.get_node(sub_id)
    assert node.exact_brep is not None

    # THE WIRING'S OWN PIN: the live body answers the DAG names —
    # the node ids in every part (the bridge's box, the extrude's
    # prism, the subtract's born rims: all wired through).
    live = node.exact_brep
    assert brep_mod.face_query(live, 1)["name"] == \
        "box#%d:top" % box_id
    assert brep_mod.face_query(live, 6)["name"] == \
        "ext#%d:lat(0)" % ex_id
    rim_name = ("sub#%d:isect[box#%d:top × ext#%d:lat(0)]@hi"
                % (sub_id, box_id, ex_id))
    pick = brep_mod.edge_query(live, 14)
    assert pick["name"] == rim_name

    # The downstream feature's reference, picked ONCE (the
    # structural answer, the click's own).
    reference = geom_mod.ShapeId(
        geom_mod.FeatureId(sub_id), pick["shape_id"]["path"])

    # The OLD GENERATION'S TAVOLA, derived before the edit (a value
    # — it survives the rebuild; the old body itself is not needed:
    # the tavola's rows carry the identities).
    tool_body = s.get_node(ex_id).exact_brep
    box_body = bridge_mod.sdf_to_brep(s.get_node(box_id).body)
    old_tav = brep_mod.subtract_descent(live, box_body, tool_body)
    assert len(old_tav) == 47  # the guide case's own count

    # THE EDIT: the radius, through the sketch surface (a parameter
    # edit — the same entity kind at the same position: the covered
    # flow, the no-shift declaration the edit path emits).
    s.sketch_set_entity(
        sk_id, 0, 0,
        scene_mod.sketch_circle((0.0, 0.0), 0.30, False))
    s.rebuild_operations()
    new_live = s.get_node(sub_id).exact_brep
    assert new_live is not None

    # THE RESOLUTION: the old tavola + the new body + the edit
    # context — reference_resolved, silently.
    ops = [(box_id, brep_mod.FeatureKind.Box),
           (ex_id, brep_mod.FeatureKind.Extrude)]
    gen_old = brep_mod.Generation(
        None, old_tav, brep_mod.FeatureKind.Subtract, ops)
    gen_new = brep_mod.Generation(
        new_live, None, brep_mod.FeatureKind.Subtract, ops)
    ctx = brep_mod.EditContext(no_shift=True)
    r = brep_mod.resolve(reference, gen_old, gen_new, ctx)

    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["name"] == rim_name  # the DAG name, unchanged

    # THE FEATURE RE-RUNS, without reselection: the resolved name
    # answers the seam-wired PAIR on the new body, and the new
    # radius is live right there (the fillet's own input curve).
    ids = brep_mod.query_edge_by_name(new_live, reference)
    assert ids == [14, 15]
    top = brep_mod.face_query(new_live, 1)
    radii = [ce["pcurve"]["radius"]
             for loop in top["loops"] for ce in loop["coedges"]
             if ce["pcurve"] is not None
             and ce["pcurve"]["type"] == "arc"]
    assert radii and all(abs(x - 0.30) < 1e-6 for x in radii)
    # The old radius is gone: the edit is VISIBLE at the resolved
    # address (the feature computed on the new geometry, not a
    # cached body).
    assert all(abs(x - 0.25) > 1e-6 for x in radii)
