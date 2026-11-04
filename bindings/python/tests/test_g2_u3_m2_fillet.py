"""G2 unit 3, morso 2 — IL FILLET, IL MORSO 1 through the bindings
(ADR-0039 D1.1, the cylindrical flank): the ladder's own words —
"the digest, the bindings reachable from Python" (ADR-0039 D6, m2's
tail). The C++ suite pins the whole derivation (tests/unit/brep/
test_g2_u3_m2_fillet.cpp — the golden 10/15/7 with the census 30/2,
the tavola's 35 rows, the counterproofs red on the naive, the
determinism N=10, the frame rule pointwise). What THESE tests pin is
what crosses the binding intact — the m6 click-test's kernel half,
the UI's half declared and deferred to its own rung:

  * the guide case built from Python: the reference taken FROM THE
    QUERY (the shape_id dict the edge query returns — the reference
    is the structural ShapeId, never an address, never a parse of
    the render), the fillet run, the counts and the volume read back
    through the plain-value queries;
  * the names, character for character (the flank, the two tangent
    wires, the arcs honestly unnamed, the shrink faces' inheritance);
  * the unnamed mode (body_kind omitted) honest beside the named one;
  * the band's refusal with the bound carried (the radius, the reach,
    the epsilon — every number in the message);
  * the radius edit through the resolver (class 1 silent) with the
    feature re-run on the new radius WITHOUT reselection — the
    reference taken once, the edit answering reference_resolved.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod

Vec3f = CAD_0.math.Vec3f

# The guide case's numbers (ADR-0039 D1.1's table, the ADR the only
# source — the classical forms ARE the tests):
VOLUME = 7.5 + math.pi / 8.0          # 8 - 2*r^2*(1-pi/4), r=0.5
FLANK = "fil#2:flank[box#1:back × box#1:right]"
WIRE_BACK = "fil#2:isect[box#1:back × " + FLANK + "]"
WIRE_RIGHT = "fil#2:isect[box#1:right × " + FLANK + "]"


def _guide_box():
    """box#1 — the factory body, the DAG id 1 (the chain the m2
    vocabulary reads: box#1 -> fil#2)."""
    return brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)


def _vertical2_reference(box):
    """The by-name reference, taken FROM THE QUERY (the shape_id the
    edge query returns — the structural form; the string render is
    the human's, the machine's is the dict)."""
    for i in range(brep_mod.edge_count(box)):
        e = brep_mod.edge_query(box, i)
        if e["name"] == "box#1:vertical2":
            sid = e["shape_id"]
            return geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
    pytest.fail("box#1:vertical2 not found on the factory box")


def test_the_guide_case_counts_and_volume():
    box = _guide_box()
    ref = _vertical2_reference(box)
    out = brep_mod.fillet_edge(box, ref, 0.5, feature=2,
                               body_kind=brep_mod.FeatureKind.Box)

    q = brep_mod.body_query(out)
    assert q["V"] == 10
    assert q["E"] == 15
    assert q["F"] == 7
    assert q["H"] == 0
    assert q["chi"] == 2
    assert q["genus"] == 0
    assert q["orientable"] is True
    assert q["loops_closed"] is True

    # The census through the query surface: right 4 + back 4 + top 5
    # + bottom 5 + front 4 + left 4 + flank 4 = 30 / 2 = 15.
    total = 0
    for f in range(7):
        info = brep_mod.face_query(out, f)
        assert len(info["loops"]) == 1
        total += len(info["loops"][0]["coedges"])
    assert total == 30
    assert len(brep_mod.face_query(out, 6)["loops"][0]["coedges"]) == 4

    mp = brep_mod.mass_properties(out)
    assert mp["all_exact"] is True
    assert mp["volume"] == pytest.approx(VOLUME, rel=1e-12)
    # The wedge the same arithmetic read twice: 8 - V == 2*r^2*(1-pi/4)
    wedge = 8.0 - mp["volume"]
    assert wedge == pytest.approx(2 * 0.25 * (1 - math.pi / 4), rel=1e-12)
    # The naive reads RED by discrimination (the C++ counterproof's
    # numbers, crossed intact): the square-cut 7.5 is not the answer,
    # the full-cylinder pi/2 is not the answer.
    assert mp["volume"] != pytest.approx(7.5)
    assert wedge != pytest.approx(math.pi / 2)


def test_the_names_and_the_inheritance():
    box = _guide_box()
    ref = _vertical2_reference(box)
    out = brep_mod.fillet_edge(box, ref, 0.5, feature=2,
                               body_kind=brep_mod.FeatureKind.Box)

    assert brep_mod.face_query(out, 6)["name"] == FLANK
    # The two tangent wires: the level ABSENT, the tangency IS the
    # level; the second operand the SAME feature's flank.
    names = {brep_mod.edge_query(out, i)["name"] for i in range(15)}
    assert WIRE_BACK in names
    assert WIRE_RIGHT in names
    # The quarter arcs honestly unnamed; the shrink faces and the
    # re-spanned edges KEEP their names (the stamp copy).
    assert brep_mod.face_query(out, 1)["name"] == "box#1:top"
    assert brep_mod.face_query(out, 3)["name"] == "box#1:back"
    assert brep_mod.face_query(out, 5)["name"] == "box#1:right"
    assert "box#1:top2" in names
    assert "box#1:bottom2" in names

    # The by-name queries answer the addresses (the name plane's
    # door, crossed intact).
    flank_id = brep_mod.face_query(out, 6)["shape_id"]
    hits = brep_mod.query_face_by_name(
        out, geom_mod.ShapeId(geom_mod.FeatureId(flank_id["feature"]),
                              flank_id["path"]))
    assert hits == [6]


def test_the_unnamed_mode_honest():
    box = _guide_box()
    ref = _vertical2_reference(box)
    out = brep_mod.fillet_edge(box, ref, 0.5, feature=2)
    # The born entities read honestly empty; the inheritance flows
    # (a stamp copy, no table involved).
    assert brep_mod.face_query(out, 6)["name"] == ""
    for i in (11, 12, 13, 14):
        assert brep_mod.edge_query(out, i)["name"] == ""
    assert brep_mod.face_query(out, 3)["name"] == "box#1:back"
    # The unnamed mode is STILL the same topology: the counts hold.
    assert brep_mod.body_query(out)["E"] == 15


def test_the_band_refusal_carries_the_bound():
    box = _guide_box()
    ref = _vertical2_reference(box)
    # r = 2.5 against the box's reach 2: the tangent line would exit
    # the face — the refusal names the member, the radius, the bound
    # and the epsilon (never a dry string without the number).
    with pytest.raises(ValueError) as exc_info:
        brep_mod.fillet_edge(box, ref, 2.5, feature=2,
                             body_kind=brep_mod.FeatureKind.Box)
    msg = str(exc_info.value)
    assert "fillet_radius_exceeds_face" in msg
    assert "2.500000" in msg
    assert "2.000000" in msg
    # The two rim-class members stay registered in their seats (m3):
    # the box class cannot meet them, the payload never names them.
    assert "fillet_exits_annulus" not in msg
    assert "fillet_eats_wall" not in msg


def test_the_radius_edit_class1_silent_the_feature_reruns():
    # The reference picked ONCE, the radius edited in band (0.5 ->
    # 0.25): the resolver answers class 1 silent, the feature re-runs
    # on the new radius WITHOUT reselection — the counts stay 10/15/7
    # for every in-band r, the volume the closed form's own value.
    box = _guide_box()
    ref = _vertical2_reference(box)
    tav = brep_mod.factory_descent(box)
    old = brep_mod.Generation(box, tav, brep_mod.FeatureKind.Box)
    box_again = _guide_box()
    new = brep_mod.Generation(box_again, None, brep_mod.FeatureKind.Box)
    ctx = brep_mod.EditContext(no_shift=True)
    r = brep_mod.resolve(ref, old, new, ctx)
    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["name"] == "box#1:vertical2"

    for radius in (0.25, 0.5, 0.9):
        out = brep_mod.fillet_edge(box_again, ref, radius, feature=2,
                                   body_kind=brep_mod.FeatureKind.Box)
        q = brep_mod.body_query(out)
        assert q["V"] == 10
        assert q["E"] == 15
        assert q["F"] == 7
        assert q["chi"] == 2
        mp = brep_mod.mass_properties(out)
        assert mp["all_exact"] is True
        assert mp["volume"] == pytest.approx(
            8.0 - 2.0 * radius * radius * (1.0 - math.pi / 4.0),
            rel=1e-12)

    # The same radius twice: the same names (the digest's own bar,
    # read through the query surface — the names pure functions of
    # the recipe).
    a = brep_mod.fillet_edge(box_again, ref, 0.5, feature=2,
                             body_kind=brep_mod.FeatureKind.Box)
    b = brep_mod.fillet_edge(box_again, ref, 0.5, feature=2,
                             body_kind=brep_mod.FeatureKind.Box)
    names_a = [brep_mod.edge_query(a, i)["name"] for i in range(15)]
    names_b = [brep_mod.edge_query(b, i)["name"] for i in range(15)]
    assert names_a == names_b
    assert (brep_mod.face_query(a, 6)["name"]
            == brep_mod.face_query(b, 6)["name"])
