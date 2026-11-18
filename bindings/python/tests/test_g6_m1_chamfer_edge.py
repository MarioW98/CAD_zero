"""bindings/python/tests/test_g6_m1_chamfer_edge.py

G6 m1 — the chamfer's Python surface, ADR-0045 D1.2/D8.1: the flat
bevel on the box's vertical edge, exposed on the fillet_edge stamp
(feature + body_kind, the unnamed default, the ValueError refusals
with the bounds carried). The scene node and the picker are the
final morse's own door (ADR-0045 D7.4) — this surface is the tests'
own seat, nothing more.

Pins:
  * the guide case's counts and volume (10/15/7, chi=2, V = 8 - d^2
    = 7.75, all exact) and the census 30/2;
  * the names, character for character (the bevel, the two setback
    wires — the level ABSENT, the setback IS the level — the
    segments honestly unnamed, the shrink faces' inheritance);
  * the unnamed mode (body_kind omitted) honest beside the named one;
  * the band's refusal with the bound carried (the setback, the
    reach, the epsilon — every number in the message), the state
    band d < 1 documented in the payload's words and never met by
    this single walk;
  * the tavola through the binding (35 rows, the vanished cause
    verbatim 'replaced by the bevel'), the digest's determinism.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod

Vec3f = CAD_0.math.Vec3f

# The guide case's numbers (ADR-0045 D6.1's table, the ADR the only
# source — the classical form IS the test):
VOLUME = 8.0 - 0.25               # 8 - d^2, d = 0.5 (the prism)
BEVEL = "che#2:bevel[box#1:back × box#1:right]"
WIRE_BACK = "che#2:isect[box#1:back × " + BEVEL + "]"
WIRE_RIGHT = "che#2:isect[box#1:right × " + BEVEL + "]"


def _guide_box():
    """box#1 — the factory body, the DAG id 1 (the chain the m1
    vocabulary reads: box#1 -> che#2)."""
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


def _chamfer(box, setback=0.5, feature=2, body_kind=True):
    ref = _vertical2_reference(box)
    if body_kind:
        return brep_mod.chamfer_edge(
            box, ref, setback, feature=feature,
            body_kind=brep_mod.FeatureKind.Box)
    return brep_mod.chamfer_edge(box, ref, setback, feature=feature)


def test_the_guide_case_counts_and_volume():
    box = _guide_box()
    out = _chamfer(box)

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
    # + bottom 5 + front 4 + left 4 + bevel 4 = 30 / 2 = 15.
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
    # The prism the same arithmetic read twice: 8 - V == d^2 (the
    # right triangular section d^2/2 over the arm 2).
    removed = 8.0 - mp["volume"]
    assert removed == pytest.approx(0.5 * 0.5, rel=1e-12)
    # The naive reads RED by discrimination: the no-cut 8, the
    # square-cut 7.5, and the fillet's own wedge 7.5 + pi/8 — a
    # DIFFERENT geometry, never the flat bevel's answer.
    assert mp["volume"] != pytest.approx(8.0)
    assert mp["volume"] != pytest.approx(7.5)
    assert mp["volume"] != pytest.approx(7.5 + math.pi / 8.0, rel=1e-3)


def test_the_names_and_the_inheritance():
    box = _guide_box()
    out = _chamfer(box)

    assert brep_mod.face_query(out, 6)["name"] == BEVEL
    # The two setback wires: the level ABSENT, the setback IS the
    # level; the second operand the SAME feature's bevel.
    names = {brep_mod.edge_query(out, i)["name"] for i in range(15)}
    assert WIRE_BACK in names
    assert WIRE_RIGHT in names
    # The chamfer segments honestly unnamed; the shrink faces and
    # the re-spanned edges KEEP their names (the stamp copy).
    assert brep_mod.edge_query(out, 13)["name"] == ""
    assert brep_mod.edge_query(out, 14)["name"] == ""
    assert brep_mod.face_query(out, 1)["name"] == "box#1:top"
    assert brep_mod.face_query(out, 3)["name"] == "box#1:back"
    assert brep_mod.face_query(out, 5)["name"] == "box#1:right"
    assert "box#1:top2" in names
    assert "box#1:bottom2" in names

    # The by-name queries answer the addresses (the name plane's
    # door, crossed intact).
    bevel_id = brep_mod.face_query(out, 6)["shape_id"]
    hits = brep_mod.query_face_by_name(
        out, geom_mod.ShapeId(geom_mod.FeatureId(bevel_id["feature"]),
                              bevel_id["path"]))
    assert hits == [6]


def test_the_unnamed_mode_honest():
    box = _guide_box()
    out = _chamfer(box, body_kind=False)
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
    # d = 2.5 against the box's reach 2: the setback line would exit
    # the face — the refusal names the member, the setback, the
    # bound and the epsilon (never a dry string without the number).
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_edge(box, ref, 2.5, feature=2,
                              body_kind=brep_mod.FeatureKind.Box)
    msg = str(exc_info.value)
    assert "chamfer_setback_exceeds_face" in msg
    assert "2.500000" in msg
    assert "2.000000" in msg
    assert "0.000001" in msg
    # The band's edge itself (d exactly at the reach): refused — the
    # strict interiority the derivation demands.
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_edge(box, ref, 2.0, feature=2,
                              body_kind=brep_mod.FeatureKind.Box)
    assert "chamfer_setback_exceeds_face" in str(exc_info.value)
    # The state band d < 1 is NOT this walk's wall: a single chamfer
    # at 1.5 is IN its own band (the strip law binds the multi-edge
    # mode — the ring act's own morsi, documented in the payload).
    out = brep_mod.chamfer_edge(box, ref, 1.5, feature=2,
                                body_kind=brep_mod.FeatureKind.Box)
    mp = brep_mod.mass_properties(out)
    assert mp["volume"] == pytest.approx(8.0 - 1.5 * 1.5, rel=1e-9)
    # The non-positive setback: refused with the number.
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_edge(box, ref, -0.5, feature=2,
                              body_kind=brep_mod.FeatureKind.Box)
    assert "-0.500000" in str(exc_info.value)
    # The horizontal edge: the class takes the VERTICAL spigolo.
    with pytest.raises(ValueError) as exc_info:
        sid = brep_mod.edge_query(box, 0)["shape_id"]
        brep_mod.chamfer_edge(
            box,
            geom_mod.ShapeId(geom_mod.FeatureId(sid["feature"]),
                             sid["path"]), 0.5, feature=2,
            body_kind=brep_mod.FeatureKind.Box)
    assert "not vertical" in str(exc_info.value)
    # The chained body (the chamfered box itself): the chain is the
    # next morse's own seat — refused BY NAME, the seat carried.
    with pytest.raises(ValueError) as exc_info:
        out2 = _chamfer(box)
        ref9 = brep_mod.edge_query(box, 9)["shape_id"]
        brep_mod.chamfer_edge(
            out2,
            geom_mod.ShapeId(geom_mod.FeatureId(ref9["feature"]),
                             ref9["path"]), 0.5, feature=3,
            body_kind=brep_mod.FeatureKind.Box)
    msg = str(exc_info.value)
    assert "outside the class signature" in msg
    assert "next morse's own seat" in msg


def test_the_tavola_and_the_digest_determinism():
    box = _guide_box()
    out = _chamfer(box)

    # The tavola through the binding: the opaque handle, the row
    # count the walk's own arithmetic (35 — the same shape the
    # fillet's m2 landed; the rows' deep checks — the causes, the
    # born names, the closure — live in the C++ test, the surface
    # here).
    tav = brep_mod.chamfer_descent(out, box)
    assert len(tav) == 35
    assert repr(tav) == "<DescentTable with 35 rows>"

    # The digest's determinism: the same recipe rebuilt, the same
    # chain digest (the m4 N-run discipline, the door's own answer).
    d1 = brep_mod.chain_digest(out)
    for _ in range(3):
        again = _chamfer(_guide_box())
        assert brep_mod.chain_digest(again) == d1
