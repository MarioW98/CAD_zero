"""G6 m3 — the corner act at one corner (ADR-0045 D4/D8.3), the
Python seat: the counts, the volume's closed form, the names, the
mismatch and the dead reference. The deep contracts (the sweep's
48 chains, the oracle at zero shared code, the doctored reads) are
the C++ test's own file; this surface pins the binding's door.
"""

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod

Vec3f = CAD_0.math.Vec3f

# The guide case's numbers (the derivation the ADR's D4 demands and
# the C++ oracle pins at 1e-12): V = 8 - 3d^2 + (3/4)d^3, d = 0.5.
VOLUME = 8.0 - 3.0 * 0.25 + 0.75 * 0.125


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


def _guide(d=0.5):
    """box#1 -> che#2 (vertical2, the plain walk, 10/15/7)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    first = brep_mod.chamfer_edge(
        box, _reference(box, "box#1:vertical2"), d, feature=2,
        body_kind=brep_mod.FeatureKind.Box)
    return box, first


def _act(first, d=0.5, ring="box#1:top1"):
    """The act at the corner (1,1,1): the reference the ring edge
    top x right (the Y-parallel role of the sweep)."""
    return brep_mod.chamfer_corner_act(
        first, _reference(first, ring), d, feature=3,
        body_kind=brep_mod.FeatureKind.Box)


def test_the_corner_act_lands_the_chain():
    box, first = _guide()
    assert (brep_mod.body_query(first)["V"],
            brep_mod.body_query(first)["E"],
            brep_mod.body_query(first)["F"]) == (10, 15, 7)
    out = _act(first)
    q = brep_mod.body_query(out)
    # THE COUNTS — the act's own arithmetic from the k=1 chain:
    # V-4+8 / E-3+9 / F+2.
    assert (q["V"], q["E"], q["F"]) == (14, 21, 9)
    assert q["chi"] == 2
    assert q["H"] == 0
    # THE VOLUME — the closed form, exact (the miter's quota the
    # inclusion-exclusion at the corner derives).
    mp = brep_mod.mass_properties(out)
    assert mp["volume"] == pytest.approx(VOLUME, rel=1e-12)
    assert mp["all_exact"] is True


def test_the_names_the_miter_and_the_walls():
    box, first = _guide()
    out = _act(first)
    # The two ring bevels born, the vertical bevel's name KEPT (the
    # Absorbed edit, never Born-sotto-falso).
    names = [brep_mod.face_query(out, f)["name"]
             for f in range(brep_mod.body_query(out)["F"])]
    assert "che#2:bevel[box#1:back × box#1:right]" in names
    assert "che#3:bevel[box#1:top × box#1:right]" in names
    assert "che#3:bevel[box#1:top × box#1:back]" in names
    # A miter line's name carries the new role (the bevel x bevel
    # concurrence at the miter vertex).
    e12 = brep_mod.edge_query(out, 12)["name"]
    assert e12.startswith("che#3:miter[")
    assert "bevel[box#1:back × box#1:right]" in e12

    # THE MISMATCH — the named refusal with both values in the
    # payload (the vertical bevel's own 0.5 against the asked 0.4).
    with pytest.raises(ValueError) as exc_info:
        _act(first, d=0.4)
    msg = str(exc_info.value)
    assert "chamfer_corner_setback_mismatch" in msg
    assert "0.400000" in msg and "0.500000" in msg

    # THE DEAD REFERENCE — the ring edge consumed by the act: the
    # OLD structural id against the new body, the kernel's own
    # refusal (never the name scan's).
    dead_ref = _reference(first, "box#1:top1")
    with pytest.raises(ValueError) as exc_info:
        brep_mod.chamfer_corner_act(
            out, dead_ref, 0.5, feature=4,
            body_kind=brep_mod.FeatureKind.Box)
    assert "names no edge of this body" in str(exc_info.value)
