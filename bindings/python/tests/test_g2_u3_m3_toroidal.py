"""G2 unit 3, morso 3 — IL FILLET, IL MORSO 2 through the bindings
(ADR-0039 D1.2, the chiral class): the rim fillet reachable from
Python (the order's own BINDING clause). The C++ suite pins the whole
derivation (tests/unit/brep/test_g2_u3_m3_toroidal.cpp — the golden
14/20/9 with the census 40 and the closure 9/22/16, the volume
(8-0.375*pi)-V_r, the chiral pins, the counterproofs, the bands'
bounds, the determinism N=10). What THESE tests pin is what crosses
the binding intact:

  * the guide case built from Python: the reference taken FROM THE
    QUERY (the rim is the seam-wired arc PAIR — one logical name,
    taken whole), the fillet run, the counts and the volume read back
    through the plain-value queries;
  * the names, character for character (the flank, the two tangent
    wires with the level absent, the flank's own seam, the shrink
    faces' inheritance);
  * the unnamed mode (the naming context omitted) honest beside the
    named one;
  * the two bands' refusals with their bounds carried (the radius,
    the bound and the epsilon — every number in the message), the
    concave family by name, the seam by name;
  * the radius edit class 1 SILENT on the rim too — the reference
    taken once, the edit answering reference_resolved, the feature
    re-run on the new radius without reselection.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod

Vec3f = CAD_0.math.Vec3f

# The guide case's numbers (ADR-0039 D1.2's table):
BLIND_VOLUME = 8.0 - 0.375 * math.pi              # 6.8219027549...
V_R = 2.0 * math.pi * (0.5 * 0.25 * 0.25 * (1.0 - math.pi / 4.0) +
                       0.25 ** 3 * (5.0 / 6.0 - math.pi / 4.0))
VOLUME = BLIND_VOLUME - V_R                       # 6.7750597585...
FLANK = "fil#5:flank[box#1:top × ext#3:lat(0)]"
WIRE_PLANE = f"fil#5:isect[box#1:top × {FLANK}]"
WIRE_WALL = f"fil#5:isect[ext#3:lat(0) × {FLANK}]"
SEAM = f"fil#5:seam[{FLANK}]"
RIM = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"


def _blind(offset=(0.0, 0.0, -0.5)):
    """The u2 guide body: box#1 -> ext#3 -> sub#4 (the tool: the
    circle r = 0.5, prism height 2, the world offset below)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = [CAD_0.scene.SketchChain(
        [CAD_0.scene.sketch_circle((0.0, 0.0), 0.5, False)])]
    tool = brep_mod.make_prism(chains, 2.0, +1, feature=3)
    return brep_mod.exact_subtract(
        box, tool, Vec3f(*offset),
        target=brep_mod.FeatureKind.Box,
        tool=brep_mod.FeatureKind.Extrude, feature=4)


def _rim_reference(blind):
    """The by-name reference, taken FROM THE QUERY: the rim is the
    seam-wired arc PAIR — ONE logical name, TWO materialized edges
    (the pair taken whole, the morse-5 register's own rule)."""
    for i in range(brep_mod.edge_count(blind)):
        e = brep_mod.edge_query(blind, i)
        if e["name"] == RIM:
            sid = e["shape_id"]
            return geom_mod.ShapeId(geom_mod.FeatureId(sid["feature"]),
                                    sid["path"])
    pytest.fail("the pierced rim was not found by name")


def _fillet(blind, ref, radius=0.25, named=True):
    return brep_mod.fillet_rim_edge(
        blind, ref, radius, feature=5,
        body_kind=brep_mod.FeatureKind.Box if named else None,
        far_feature=3,
        far_body_kind=brep_mod.FeatureKind.Extrude if named else None)


def test_the_guide_case_counts_and_volume():
    blind = _blind()
    ref = _rim_reference(blind)
    out = _fillet(blind, ref)

    q = brep_mod.body_query(out)
    assert q["V"] == 14
    assert q["E"] == 20
    assert q["F"] == 9
    assert q["H"] == 1
    assert q["chi"] == 2
    assert q["genus"] == 0
    assert q["orientable"] is True
    assert q["loops_closed"] is True

    # The census: top 6 + bottom 4 + four sides 16 + wall 6 + cap 2 +
    # flank 6 = 40 = 20 x 2, every edge two opposite co-edges.
    total = 0
    for f in range(9):
        info = brep_mod.face_query(out, f)
        total += sum(len(lp["coedges"]) for lp in info["loops"])
    assert total == 40
    assert len(brep_mod.face_query(out, 8)["loops"][0]["coedges"]) == 6

    mp = brep_mod.mass_properties(out)
    assert mp["all_exact"] is True
    assert mp["volume"] == pytest.approx(VOLUME, rel=1e-12)
    # The wedge the same arithmetic read twice: the removed volume.
    assert BLIND_VOLUME - mp["volume"] == pytest.approx(V_R, rel=1e-12)
    # The centroid ON THE AXIS (the axisymmetry).
    assert mp["centroid"][0] == pytest.approx(0.0, abs=1e-12)
    assert mp["centroid"][1] == pytest.approx(0.0, abs=1e-12)
    # The naive reads RED by discrimination: the full torus
    # 2*pi^2*R*r^2 is NOT the flank's flux; the blind's own volume is
    # not the answer.
    assert 0.925275412602127 != pytest.approx(0.448519633389893)
    assert mp["volume"] != pytest.approx(BLIND_VOLUME)


def test_the_names_and_the_inheritance():
    blind = _blind()
    ref = _rim_reference(blind)
    out = _fillet(blind, ref)

    assert brep_mod.face_query(out, 8)["name"] == FLANK
    names = {brep_mod.edge_query(out, i)["name"] for i in range(20)}
    # The two tangent wires: the level ABSENT, the tangency IS the
    # level; the second operand the SAME feature's flank.
    assert WIRE_PLANE in names
    assert WIRE_WALL in names
    # The flank's own seam (the sub's seam form).
    assert SEAM in names
    # The shrink faces and the re-spanned edges KEEP their names.
    assert brep_mod.face_query(out, 1)["name"] == "box#1:top"
    assert brep_mod.face_query(out, 6)["name"] == "ext#3:lat(0)"
    assert brep_mod.face_query(out, 7)["name"] == "ext#3:cap(lo)"
    assert "sub#4:seam[ext#3:lat(0)]" in names
    assert "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]" in names

    # The by-name queries: the rim's PAIR answers whole (two halves).
    wire_id = brep_mod.edge_query(out, 15)["shape_id"]
    hits = brep_mod.query_edge_by_name(
        out, geom_mod.ShapeId(geom_mod.FeatureId(wire_id["feature"]),
                              wire_id["path"]))
    assert sorted(hits) == [15, 16]


def test_the_unnamed_mode_honest():
    blind = _blind()
    ref = _rim_reference(blind)
    out = _fillet(blind, ref, named=False)
    assert brep_mod.face_query(out, 8)["name"] == ""
    for i in (15, 16, 17, 18, 19):
        assert brep_mod.edge_query(out, i)["name"] == ""
    # The inheritance flows (a stamp copy, no table involved).
    assert brep_mod.face_query(out, 6)["name"] == "ext#3:lat(0)"
    assert (brep_mod.edge_query(out, 14)["name"]
            == "sub#4:seam[ext#3:lat(0)]")
    # The unnamed mode is STILL the same topology.
    assert brep_mod.body_query(out)["E"] == 20


def test_the_bands_refusals_carry_the_bounds():
    blind = _blind()
    ref = _rim_reference(blind)
    # THE ANNULUS BAND: r = 0.7 against the reach 1 - a = 0.5.
    with pytest.raises(ValueError) as exc_info:
        _fillet(blind, ref, radius=0.7)
    msg = str(exc_info.value)
    assert "fillet_exits_annulus" in msg
    assert "0.700000" in msg
    assert "0.500000" in msg
    assert "0.000001" in msg

    # THE WALL BAND: the SHORT-wall blind body (the floor at 0.75:
    # the wall's height 0.25): r = 0.3 meets ONLY the wall's band.
    short = _blind(offset=(0.0, 0.0, 0.75))
    short_ref = _rim_reference(short)
    with pytest.raises(ValueError) as exc_info:
        _fillet(short, short_ref, radius=0.3)
    msg = str(exc_info.value)
    assert "fillet_eats_wall" in msg
    assert "0.300000" in msg
    assert "0.250000" in msg


def test_the_families_refuse_by_name():
    blind = _blind()
    ref = _rim_reference(blind)

    # THE CONCAVE FAMILY: the floor rim (the carried cap x the wall)
    # refused by name, the queue's entry carried.
    floor = None
    for i in range(brep_mod.edge_count(blind)):
        e = brep_mod.edge_query(blind, i)
        if e["name"] == "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]":
            sid = e["shape_id"]
            floor = geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
            break
    assert floor is not None
    with pytest.raises(ValueError) as exc_info:
        _fillet(blind, floor, radius=0.25)
    msg = str(exc_info.value)
    assert "fillet_out_of_class_pair" in msg
    assert "CONCAVE" in msg

    # THE SEAM: a fillet asked on the seam refused with the edge named.
    seam = None
    for i in range(brep_mod.edge_count(blind)):
        e = brep_mod.edge_query(blind, i)
        if e["name"] == "sub#4:seam[ext#3:lat(0)]":
            sid = e["shape_id"]
            seam = geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
            break
    assert seam is not None
    with pytest.raises(ValueError) as exc_info:
        _fillet(blind, seam, radius=0.25)
    assert "seam" in str(exc_info.value)


def test_the_radius_edit_class1_silent_the_feature_reruns():
    # The reference picked ONCE; the radius edited in band: the
    # resolver answers class 1 silent, the feature re-runs WITHOUT
    # reselection — the counts stay 14/20/9 for every in-band r, the
    # volume the closed form's own value each time.
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = [CAD_0.scene.SketchChain(
        [CAD_0.scene.sketch_circle((0.0, 0.0), 0.5, False)])]
    tool = brep_mod.make_prism(chains, 2.0, +1, feature=3)
    blind = brep_mod.exact_subtract(
        box, tool, Vec3f(0.0, 0.0, -0.5),
        target=brep_mod.FeatureKind.Box,
        tool=brep_mod.FeatureKind.Extrude, feature=4)
    ref = _rim_reference(blind)
    tav = brep_mod.subtract_descent(blind, box, tool)
    old = brep_mod.Generation(
        None, tav, brep_mod.FeatureKind.Subtract,
        [(1, brep_mod.FeatureKind.Box),
         (3, brep_mod.FeatureKind.Extrude)])
    blind2 = _blind()
    new = brep_mod.Generation(
        blind2, None, brep_mod.FeatureKind.Subtract,
        [(1, brep_mod.FeatureKind.Box),
         (3, brep_mod.FeatureKind.Extrude)])
    ctx = brep_mod.EditContext(no_shift=True)
    r = brep_mod.resolve(ref, old, new, ctx)
    assert r["kind"] == "resolved"
    assert r["code"] == "reference_resolved"
    assert r["name"] == RIM

    for radius in (0.125, 0.25, 0.4):
        out = _fillet(blind2, ref, radius=radius)
        q = brep_mod.body_query(out)
        assert q["V"] == 14
        assert q["E"] == 20
        assert q["F"] == 9
        assert q["chi"] == 2
        mp = brep_mod.mass_properties(out)
        assert mp["all_exact"] is True
        v_r = 2.0 * math.pi * (
            0.5 * radius * radius * (1.0 - math.pi / 4.0) +
            radius ** 3 * (5.0 / 6.0 - math.pi / 4.0))
        assert mp["volume"] == pytest.approx(
            BLIND_VOLUME - v_r, rel=1e-12)

    # The same radius twice: the same names (the digest's own bar).
    a = _fillet(blind2, ref, radius=0.25)
    b = _fillet(blind2, ref, radius=0.25)
    names_a = [brep_mod.edge_query(a, i)["name"] for i in range(20)]
    names_b = [brep_mod.edge_query(b, i)["name"] for i in range(20)]
    assert names_a == names_b
    assert (brep_mod.face_query(a, 8)["name"]
            == brep_mod.face_query(b, 8)["name"])
