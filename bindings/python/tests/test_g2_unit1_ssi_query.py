"""G2 unit 1 — the SSI table through the bindings (ADR-0033 D5.1).

The C++ side pins the derivations, the degenerates, the re-frame and
the budget (tests/unit/brep/test_g2_unit1_ssi_table.cpp). What THESE
tests pin is what crosses the binding intact — the proof that the
table is QUERYABLE from Python BEFORE any boolean consumes it (the
unit-2 face-query precedent, the order's own visible surface):

  * ``CAD_0.brep.ssi_query(body_a, face_a, body_b, face_b)`` — the
    intersections answer as plain values: the kind, the 3D curves
    (circles with their frames, lines with point+direction), the
    p-curves in BOTH domains (the same payload dicts the face query
    serves), the re-frame declaration when an entry derived one, and
    the Layer-2 provenance (exact/0 — the closed forms' own record);
  * ``CAD_0.brep.translate_body`` — the exact side's own placement
    operation (the subtract's tool-offset path, G1 unit 4) exposed so
    the author bodies can be POSITIONED for the query: the sphere
    pairs, the offset cylinder pairs — the bridge's primitive fit
    only serves origin-centered bodies;
  * the classical formulas readable from Python: the plane×sphere
    radius √(R²−d²), the sphere×sphere radical plane, the
    cylinder×cylinder mid-plane lines;
  * the wall: the out-of-table refusal is a RETURNED RECORD (a
    recorded answer, not a missing one) naming the pair, the class
    and the remedies — the Viviani case pinned here;
  * determinism: the same operands answer the same dict twice.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import sdf as sdf_mod

Vec3f = CAD_0.math.Vec3f

TOL = 1e-5
SQRT3 = math.sqrt(3.0)


def _close(a: float, b: float, tol: float = TOL) -> bool:
    return abs(a - b) <= tol


def _close3(a, b, tol: float = TOL) -> bool:
    return all(_close(x, y, tol) for x, y in zip(a, b))


def _exact_box(half):
    return bridge_mod.sdf_to_brep(sdf_mod.box(half))


def _exact_sphere(r: float):
    return bridge_mod.sdf_to_brep(sdf_mod.sphere(r))


def _exact_cylinder(r: float, h: float):
    return bridge_mod.sdf_to_brep(sdf_mod.cylinder(r, h))


def _sphere_face(body) -> int:
    for i in range(brep_mod.face_count(body)):
        if brep_mod.face_query(body, i)["family"] == "sphere":
            return i
    raise AssertionError("no sphere face")


def _cylinder_face(body) -> int:
    for i in range(brep_mod.face_count(body)):
        if brep_mod.face_query(body, i)["family"] == "cylinder":
            return i
    raise AssertionError("no cylinder face")


def _box_posx_face(body) -> int:
    for i in range(brep_mod.face_count(body)):
        if brep_mod.face_query(body, i)["family"] == "plane":
            return i  # any plane serves the d<R golden; the C++ pins the +X
    raise AssertionError("no plane face")


def test_plane_sphere_circle_readable_from_python():
    box = _exact_box(Vec3f(1.0, 2.0, 3.0))
    sph = _exact_sphere(2.0)

    q = brep_mod.ssi_query(box, 5, sph, 0)  # face 5 = the +X face
    assert q["kind"] == "circle"
    assert q["exact"] is True
    assert q["linear_tolerance"] == 0.0

    (curve,) = q["curves"]
    assert curve["type"] == "circle"
    assert _close(curve["radius"], SQRT3)
    assert _close3(curve["center"], (1.0, 0.0, 0.0))

    # The plane-domain p-curve: the Circle2, the same payload the face
    # query serves.
    plane_pc = q["operands"][0]["pcurves"][0]
    assert plane_pc["type"] == "circle"
    assert _close(plane_pc["radius"], SQRT3)

    # The sphere-domain p-curve: the latitude Line2 of the RE-FRAMED
    # domain (pole = the plane normal), v = arccos(1/2) = π/3.
    sph_op = q["operands"][1]
    assert sph_op["reframed"] is True
    assert _close3(sph_op["frame_pole"], (1.0, 0.0, 0.0))
    lat = sph_op["pcurves"][0]
    assert lat["type"] == "line"
    assert _close(lat["a"][1], math.pi / 3)
    assert _close(lat["b"][1], math.pi / 3)
    assert _close(lat["b"][0] - lat["a"][0], 2 * math.pi)


def test_sphere_sphere_radical_plane_with_translate_body():
    s1 = _exact_sphere(2.0)
    s2 = _exact_sphere(1.0)
    brep_mod.translate_body(s2, Vec3f(2.5, 0.0, 0.0))

    q = brep_mod.ssi_query(s1, _sphere_face(s1), s2, _sphere_face(s2))
    assert q["kind"] == "circle"

    (curve,) = q["curves"]
    # h = (D² + R₁² − R₂²)/2D = 1.85, ρ = √(4 − 1.85²)
    assert _close3(curve["center"], (1.85, 0.0, 0.0), 1e-4)
    assert _close(curve["radius"], math.sqrt(0.5775), 1e-4)

    # Both spheres re-framed to the center line.
    for op in q["operands"]:
        assert op["reframed"] is True
        assert _close3(op["frame_pole"], (1.0, 0.0, 0.0))


def test_cylinder_sphere_coaxial_pair():
    cyl = _exact_cylinder(1.0, 2.0)
    sph = _exact_sphere(2.0)

    q = brep_mod.ssi_query(cyl, _cylinder_face(cyl), sph, _sphere_face(sph))
    assert q["kind"] == "circle_pair"
    assert len(q["curves"]) == 2
    for c in q["curves"]:
        assert c["type"] == "circle"
        assert _close(c["radius"], 1.0)
    assert _close3(q["curves"][0]["center"], (0.0, SQRT3, 0.0))
    assert _close3(q["curves"][1]["center"], (0.0, -SQRT3, 0.0))

    # The stored pole (+Y) already serves the axis: NO re-frame.
    assert q["operands"][1]["reframed"] is False


def test_cylinder_cylinder_mid_plane_lines():
    c1 = _exact_cylinder(1.0, 2.0)
    c2 = _exact_cylinder(1.0, 2.0)
    brep_mod.translate_body(c2, Vec3f(0.8, 0.0, 0.0))

    q = brep_mod.ssi_query(c1, _cylinder_face(c1), c2, _cylinder_face(c2))
    assert q["kind"] == "line_pair"
    assert len(q["curves"]) == 2
    zhalf = math.sqrt(1.0 - 0.16)
    for c in q["curves"]:
        assert c["type"] == "line"
        assert _close3(c["dir"], (0.0, 1.0, 0.0))
        assert _close(c["point"][0], 0.4)
        assert _close(abs(c["point"][2]), zhalf)
    assert _close(q["curves"][0]["point"][2], +zhalf)
    assert _close(q["curves"][1]["point"][2], -zhalf)

    # The vertical Line2 p-curves span the face's authored window
    # (the factory wall's [−h, +h] = [−1, +1]).
    for op in q["operands"]:
        for pc in op["pcurves"]:
            assert pc["type"] == "line"
            assert _close(pc["a"][1], -1.0)
            assert _close(pc["b"][1], +1.0)


def test_the_wall_is_a_returned_answer():
    # The Viviani case: the sphere off the cylinder's axis.
    cyl = _exact_cylinder(1.0, 2.0)
    sph = _exact_sphere(2.0)
    brep_mod.translate_body(sph, Vec3f(0.3, 0.0, 0.0))

    q = brep_mod.ssi_query(cyl, _cylinder_face(cyl), sph, _sphere_face(sph))
    assert q["kind"] == "out_of_table"
    assert q["curves"] == []
    assert "Viviani" in q["message"]
    assert "Remedy" in q["message"]

    # The unequal-radii parallel pair: the acceptance's corrected
    # disposition, named honestly.
    c1 = _exact_cylinder(1.0, 2.0)
    c2 = _exact_cylinder(1.2, 2.0)
    brep_mod.translate_body(c2, Vec3f(0.8, 0.0, 0.0))
    q = brep_mod.ssi_query(c1, _cylinder_face(c1), c2, _cylinder_face(c2))
    assert q["kind"] == "out_of_table"
    assert "unequal radii" in q["message"]
    assert "two lines" in q["message"]


def test_bad_face_id_is_a_loud_valueerror():
    s1 = _exact_sphere(2.0)
    s2 = _exact_sphere(2.0)
    try:
        brep_mod.ssi_query(s1, 9, s2, 0)
    except ValueError as e:
        assert "no face with id" in str(e)
    else:
        raise AssertionError("expected ValueError for the bad face id")


def test_determinism_the_same_dict_twice():
    box = _exact_box(Vec3f(1.0, 2.0, 3.0))
    sph = _exact_sphere(2.0)
    a = brep_mod.ssi_query(box, 5, sph, 0)
    b = brep_mod.ssi_query(box, 5, sph, 0)
    assert a == b
