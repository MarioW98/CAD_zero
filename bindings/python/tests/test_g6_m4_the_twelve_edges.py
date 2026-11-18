"""G6 m4 — the twelve edges (ADR-0045 D4.1/D8.4), the Python seat:
the complete operation's counts and volume, the one feature's
twelve bevels, the mismatch and the collapse's refusals. The deep
contracts (the oracle at zero shared code, the sweep, the tavola,
the doctored reads) are the C++ test's own file; this surface pins
the bindings' doors.
"""

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod

Vec3f = CAD_0.math.Vec3f

# The guide case's numbers (the derivation the ADR's D6.3 pins and
# the C++ oracle verifies at 1e-12): V = 8 - 12d^2 + 6d^3, d = 0.5.
VOLUME = 8.0 - 12.0 * 0.25 + 6.0 * 0.125


def _reference(body, name):
    """The by-name reference, taken FROM THE QUERY (the structural
    shape_id the edge query returns — the string render the
    human's, the machine's the dict)."""
    for i in range(brep_mod.edge_count(body)):
        e = brep_mod.edge_query(body, i)
        if e["name"] == name:
            sid = e["shape_id"]
            return geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
    pytest.fail(name + " not found on the body")


def _box():
    return brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)


def _four(d=0.5):
    """The four-vertical chain (16/24/10), built door-by-door — the
    ring act's own carrier."""
    cur = _box()
    for k in range(4):
        cur = brep_mod.chamfer_edge(
            cur,
            _reference(cur, "box#1:vertical{}".format(k)), d,
            feature=2 + k,
            body_kind=brep_mod.FeatureKind.Box)
    return cur


def test_the_complete_operation_lands_the_cube():
    box = _box()
    out = brep_mod.chamfer_twelve_edges(
        box, 0.5, feature=2, body_kind=brep_mod.FeatureKind.Box)
    q = brep_mod.body_query(out)
    # THE COUNTS — the act's own arithmetic from the four-vertical
    # state: V-16+32 / E-16+40 / F+8.
    assert (q["V"], q["E"], q["F"]) == (32, 48, 18)
    assert q["chi"] == 2
    assert q["H"] == 0
    # THE VOLUME — the closed form, exact at 1e-12.
    mp = brep_mod.mass_properties(out)
    assert mp["volume"] == pytest.approx(VOLUME, rel=1e-12)
    assert mp["all_exact"] is True
    # THE SOURCE UNTOUCHED (the door's transaction: the box still
    # the box).
    assert (brep_mod.body_query(box)["V"],
            brep_mod.body_query(box)["E"],
            brep_mod.body_query(box)["F"]) == (8, 12, 6)


def test_the_names_one_feature_twelve_bevels():
    out = brep_mod.chamfer_twelve_edges(
        _box(), 0.5, feature=2, body_kind=brep_mod.FeatureKind.Box)
    names = [brep_mod.face_query(out, f)["name"]
             for f in range(brep_mod.body_query(out)["F"])]
    bevels = [x for x in names if ":bevel[" in x]
    # The twelve bevels under the ONE feature, distinct by their
    # operand pairs (the m1 guide case's own among them — the
    # vertical2 walk's operand pair).
    assert len(bevels) == 12
    assert all(x.startswith("che#2:bevel[") for x in bevels)
    assert "che#2:bevel[box#1:back × box#1:right]" in bevels
    assert len(set(bevels)) == 12
    # The miter lines' role, twenty-four of them.
    miters = [brep_mod.edge_query(out, i)["name"]
              for i in range(brep_mod.body_query(out)["E"])]
    miters = [x for x in miters if ":miter[" in x]
    assert len(miters) == 24
    assert all(x.startswith("che#2:miter[") for x in miters)


def test_the_ring_act_door_and_its_walls():
    four = _four(0.5)
    q = brep_mod.body_query(four)
    assert (q["V"], q["E"], q["F"]) == (16, 24, 10)
    out = brep_mod.chamfer_ring_act(
        four, 0.5, feature=6, body_kind=brep_mod.FeatureKind.Box)
    q = brep_mod.body_query(out)
    assert (q["V"], q["E"], q["F"]) == (32, 48, 18)
    assert q["chi"] == 2
    mp = brep_mod.mass_properties(out)
    assert mp["volume"] == pytest.approx(VOLUME, rel=1e-12)

    # THE MISMATCH — the named refusal with both values (the chain
    # at 0.4, the act asked at 0.5).
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_ring_act(
            _four(0.4), 0.5, feature=6,
            body_kind=brep_mod.FeatureKind.Box)
    msg = str(exc_info.value)
    assert "chamfer_corner_setback_mismatch" in msg
    assert "0.500000" in msg and "0.400000" in msg

    # THE STAGE BAND — the collapse at d=1: the two readings in the
    # payload (the faces' reach 2 would admit it, the stage's 1
    # refuses — the body is not the limit).
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_ring_act(
            four, 1.0, feature=6, body_kind=brep_mod.FeatureKind.Box)
    msg = str(exc_info.value)
    assert "chamfer_setback_exceeds_face" in msg
    assert "1.000000" in msg and "2.000000" in msg

    # THE STATE WALL — the ring act on the fresh box names its own
    # door (the vertical passage first).
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_ring_act(
            _box(), 0.5, feature=6,
            body_kind=brep_mod.FeatureKind.Box)
    assert "after the FOUR VERTICAL CHAMFERS" in str(exc_info.value)


def test_the_collapse_refused_at_the_door():
    # The d = 1 collapse: the first vertical admits it (the single
    # edge's own band), the SECOND walk refuses at THE STRIP LAW's
    # own reading — the m1 machinery reads the CURRENT face's reach,
    # and the first strip's retreat has tightened it to exactly 2-d
    # (one wall before the stage band would name the collapse; the
    # bound carried in the payload). The door publishes nothing.
    for d in (1.0, 1.5):
        with pytest.raises(ValueError) as exc_info:
            brep_mod.chamfer_twelve_edges(
                _box(), d, feature=2,
                body_kind=brep_mod.FeatureKind.Box)
        msg = str(exc_info.value)
        assert "chamfer_setback_exceeds_face" in msg
        assert "the setback line" in msg
        assert "{:.6f}".format(d) in msg

    # The non-positive and the non-finite at the door's own gate.
    for bad in (0.0, -0.5, float("nan")):
        with pytest.raises(ValueError) as exc_info:
            brep_mod.chamfer_twelve_edges(
                _box(), bad, feature=2,
                body_kind=brep_mod.FeatureKind.Box)
        assert "finite and > 0" in str(exc_info.value)
