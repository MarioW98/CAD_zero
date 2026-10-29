"""G2 unit 2 — the extended booleans through the bindings (ADR-0033 D5.2).

The C++ side pins the derivations, the gates and the assemblies
(tests/unit/brep/test_g2_unit2_booleans_ext.cpp). What THESE tests pin
is what crosses the binding intact — the proof that the operations
CONSUME the table from Python, on author bodies, BEFORE any UI
consumes them:

  * ``CAD_0.brep.exact_union / exact_intersect / exact_subtract`` —
    the three operations callable with (body_a, body_b, offset, tol):
    the sphere×sphere classical union, the lens, the bitten sphere;
    the bead / the rivet / the core (the coaxial circle prism); the
    truncation family (the box); the SECOND cut on a pierced target
    (the two-holes body, the unit's guide case);
  * the bodies QUERYABLE after the operations: ``body_query`` (the
    counts, chi, the provenance tallies), ``face_query`` (the
    families, the loops, the typed p-curves), ``mass_properties``
    (the trimmed-sphere family: the classical cap/lens/bead formulas
    readable from Python);
  * the refusals as ValueErrors carrying the actionable message (the
    pair, the class, the remedies) — the non-annular crossings, the
    Viviani wall, the frame budget, the blind tool;
  * determinism (ADR-0014): the same operands assemble the same body
    twice — the digest comparison;
  * the operand-order independence of the union (the canonical
    identity before the keying).

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import scene as scene_mod
from CAD_0 import sdf as sdf_mod

Vec3f = CAD_0.math.Vec3f
SketchChain = scene_mod.SketchChain
sketch_circle = scene_mod.sketch_circle

TOL = 1e-4
PI = math.pi


def _close(a: float, b: float, tol: float = TOL) -> bool:
    return abs(a - b) <= tol


def _sphere(r: float, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0):
    body = bridge_mod.sdf_to_brep(sdf_mod.sphere(r))
    if cx or cy or cz:
        brep_mod.translate_body(body, Vec3f(cx, cy, cz))
    return body


def _box(half):
    return bridge_mod.sdf_to_brep(sdf_mod.box(half))


def _circle_prism(r: float, z_lo: float, z_hi: float, cx: float = 0.0,
                  cy: float = 0.0):
    """A Z-axis circle prism spanning [z_lo, z_hi] — the AUTHOR path:
    the sketch chain (the kernel's own circle entity), the scene's
    exact extrude (the HybridBody's exact B-Rep), then the placement
    (the subtract's tool-offset path)."""
    sc = scene_mod.Scene()
    sid = sc.add_sketch_node("S", [SketchChain([sketch_circle((cx, cy),
                                                              r, False)])])
    eid = sc.add_extrude_node("Extrude", sid, z_hi - z_lo, 1)
    n = sc.get_node(eid)
    assert n.exact_brep is not None and n.hybrid_refusal == ""
    tool = n.exact_brep
    brep_mod.translate_body(tool, Vec3f(0.0, 0.0, z_lo))
    return tool


def _cap_volume(R: float, h: float) -> float:
    """The classical spherical cap — the TEST, never the implementation."""
    return PI * h * h * (3.0 * R - h) / 3.0


def _digest(body) -> str:
    """The full query answer as a comparable string (repr of plain
    values — the bit-identity oracle for the determinism checks)."""
    out = [repr(brep_mod.body_query(body))]
    for face_id in range(brep_mod.face_count(body)):
        out.append(repr(brep_mod.face_query(body, face_id)))
    return "\n".join(out)


def test_union_the_classical():
    a = _sphere(1.0)
    b = _sphere(0.8, cx=1.2)
    u = brep_mod.exact_union(a, b)
    q = brep_mod.body_query(u)
    assert q["V"] == 4
    assert q["E"] == 4
    assert q["F"] == 2
    assert q["chi"] == 2
    # The classical volume: V1 + V2 - the two caps the union absorbs.
    want = (4.0 / 3.0) * PI * (1.0 + 0.512) - _cap_volume(1.0, 0.25) \
        - _cap_volume(0.8, 0.35)
    mp = brep_mod.mass_properties(u)
    assert _close(mp["volume"], want)
    assert mp["all_exact"] is True
    # Two trimmed sphere faces — the re-framed survivors.
    fams = [brep_mod.face_query(u, i)["family"] for i in range(2)]
    assert fams == ["sphere", "sphere"]
    # The operand-order independence (the canonical identity).
    u2 = brep_mod.exact_union(b, a)
    assert _digest(u) == _digest(u2)


def test_the_lens_and_the_bitten_sphere():
    a = _sphere(1.0)
    b = _sphere(0.8, cx=1.2)
    lens = brep_mod.exact_intersect(a, b)
    q = brep_mod.body_query(lens)
    assert q["V"] == 4
    assert q["chi"] == 2
    want = _cap_volume(1.0, 0.25) + _cap_volume(0.8, 0.35)
    assert _close(brep_mod.mass_properties(lens)["volume"], want)

    bitten = brep_mod.exact_subtract(a, b)
    q = brep_mod.body_query(bitten)
    assert q["V"] == 3  # the pole + the rim split pair
    assert q["E"] == 3
    assert q["F"] == 2
    assert q["chi"] == 2
    assert _close(brep_mod.mass_properties(bitten)["volume"],
                  _cap_volume(1.0, 1.75))
    fams = [brep_mod.face_query(bitten, i)["family"] for i in range(2)]
    assert sorted(fams) == ["plane", "sphere"]


def test_the_bead_and_its_wall():
    a = _sphere(1.0)
    tool = _circle_prism(0.4, -2.0, 2.0)
    bead = brep_mod.exact_subtract(a, tool)
    q = brep_mod.body_query(bead)
    assert q["V"] == 4
    assert q["E"] == 6
    assert q["F"] == 2
    assert q["chi"] == 0   # genus 1: the through-hole
    assert q["genus"] == 1
    # THE DISCRIMINATOR: one CylinderZ wall (the faceted extract would
    # answer planar strips) beside the trimmed spherical zone.
    fams = sorted(brep_mod.face_query(bead, i)["family"] for i in range(2))
    assert fams == ["cylinder", "sphere"]
    # The classical volume: the sphere minus the core.
    w = math.sqrt(1.0 - 0.16)
    core = PI * 0.16 * 2.0 * w + 2.0 * _cap_volume(1.0, 1.0 - w)
    assert _close(brep_mod.mass_properties(bead)["volume"],
                  (4.0 / 3.0) * PI - core)
    # The p-curves typed on both faces (the trimming contract).
    for i in range(2):
        info = brep_mod.face_query(bead, i)
        for loop in info["loops"]:
            for ce in loop["coedges"]:
                assert ce["pcurve"] is not None


def test_the_rivet_and_the_core():
    a = _sphere(1.0)
    tool = _circle_prism(0.4, -2.0, 2.0)
    rivet = brep_mod.exact_union(a, tool)
    q = brep_mod.body_query(rivet)
    assert (q["V"], q["E"], q["F"]) == (8, 11, 5)
    assert q["chi"] == 2
    w = math.sqrt(1.0 - 0.16)
    core = PI * 0.16 * 2.0 * w + 2.0 * _cap_volume(1.0, 1.0 - w)
    want = (4.0 / 3.0) * PI + PI * 0.16 * 4.0 - core
    assert _close(brep_mod.mass_properties(rivet)["volume"], want)

    core_body = brep_mod.exact_intersect(a, tool)
    q = brep_mod.body_query(core_body)
    assert (q["V"], q["E"], q["F"]) == (6, 7, 3)
    assert _close(brep_mod.mass_properties(core_body)["volume"], core)


def test_the_truncation_family():
    a = _sphere(1.0)
    box = _box(Vec3f(1.5, 1.5, 1.5))
    far = brep_mod.exact_subtract(a, box, Vec3f(0.0, 0.0, 2.2))
    q = brep_mod.body_query(far)
    assert (q["V"], q["E"], q["F"]) == (3, 3, 2)
    assert _close(brep_mod.mass_properties(far)["volume"],
                  _cap_volume(1.0, 1.7))
    near = brep_mod.exact_intersect(a, box, Vec3f(0.0, 0.0, 2.2))
    assert _close(brep_mod.mass_properties(near)["volume"],
                  _cap_volume(1.0, 0.3))
    # The dome: the box's bottom face gains the latitude inner loop.
    dome = brep_mod.exact_union(a, box, Vec3f(0.0, 0.0, 2.2))
    q = brep_mod.body_query(dome)
    assert (q["V"], q["E"], q["F"]) == (11, 15, 7)
    assert q["H"] == 1
    want = 27.0 + (4.0 / 3.0) * PI - _cap_volume(1.0, 0.3)
    assert _close(brep_mod.mass_properties(dome)["volume"], want)


def test_the_two_hole_body():
    target = _box(Vec3f(1.0, 1.0, 1.0))
    t1 = _circle_prism(0.25, -1.5, 1.5, cx=-0.4)
    t2 = _circle_prism(0.25, -1.5, 1.5, cx=0.4)
    first = brep_mod.exact_subtract(target, t1)
    q1 = brep_mod.body_query(first)
    assert (q1["V"], q1["E"], q1["F"]) == (12, 17, 7)
    assert q1["H"] == 2
    assert q1["chi"] == 0

    second = brep_mod.exact_subtract(first, t2)
    q2 = brep_mod.body_query(second)
    # THE DERIVED TABLE: 16/22/8/4, chi = -2, genus 2.
    assert (q2["V"], q2["E"], q2["F"]) == (16, 22, 8)
    assert q2["H"] == 4
    assert q2["chi"] == -2
    assert q2["genus"] == 2
    assert brep_mod.mass_properties(second)["all_exact"] is True
    # TWO distinct cylinder walls; the pierced faces carry 2 inner
    # loops each (the discriminators).
    walls = 0
    pierced2 = 0
    for i in range(q2["F"]):
        info = brep_mod.face_query(second, i)
        if info["family"] == "cylinder":
            walls += 1
        if info["inner_loop_count"] == 2:
            pierced2 += 1
    assert walls == 2
    assert pierced2 == 2
    # The classical volume: abc - 2 pi r^2 c.
    assert _close(brep_mod.mass_properties(second)["volume"],
                  8.0 - 2.0 * PI * 0.0625 * 2.0)
    # Determinism: the whole chain, bit-identical.
    first_b = brep_mod.exact_subtract(target, t1)
    second_b = brep_mod.exact_subtract(first_b, t2)
    assert _digest(second) == _digest(second_b)


def test_the_refusals_carry_the_words():
    a = _sphere(1.0)
    # The Viviani wall (non-coaxial).
    tool = _circle_prism(0.4, -2.0, 2.0, cx=0.3)
    try:
        brep_mod.exact_subtract(a, tool)
        raise AssertionError("the viviani case must refuse")
    except ValueError as ex:
        assert "Viviani" in str(ex)
        assert "Remedy" in str(ex)
    # The blind tool.
    tool = _circle_prism(0.4, -0.5, 2.5)
    try:
        brep_mod.exact_subtract(a, tool)
        raise AssertionError("the blind case must refuse")
    except ValueError as ex:
        assert "blind" in str(ex)
    # The frame budget: the box whose x AND y walls cut.
    box = _box(Vec3f(0.6, 0.6, 2.5))
    try:
        brep_mod.exact_subtract(a, box)
        raise AssertionError("the two-family case must refuse")
    except ValueError as ex:
        assert "frame budget" in str(ex)
        assert "one axis family" in str(ex)
    # The non-annular crossings (the unequal radii NAME the offset
    # plane; the word "untypable" never appears).
    target = _box(Vec3f(1.0, 1.0, 1.0))
    t1 = _circle_prism(0.25, -1.5, 1.5, cx=-0.4)
    t_cross = _circle_prism(0.35, -1.5, 1.5, cx=-0.1)
    first = brep_mod.exact_subtract(target, t1)
    try:
        brep_mod.exact_subtract(first, t_cross)
        raise AssertionError("the crossing case must refuse")
    except ValueError as ex:
        msg = str(ex)
        assert "non-annular" in msg
        assert "offset plane" in msg
        assert "two typed lines" in msg
        assert "untypable" not in msg


def test_the_dispositions():
    a = _sphere(1.0)
    # The co-radial dedup: the union keeps ONE surface.
    b = _sphere(1.0)
    u = brep_mod.exact_union(a, b)
    q = brep_mod.body_query(u)
    assert (q["V"], q["E"], q["F"]) == (2, 1, 1)
    # The co-radial subtract refuses (the empty).
    try:
        brep_mod.exact_subtract(a, b)
        raise AssertionError("the coincident subtract must refuse")
    except ValueError as ex:
        assert "co-radial" in str(ex)
        assert "dedup" in str(ex)
    # The nested union keeps the container.
    inner = _sphere(0.3, cx=0.2)
    u2 = brep_mod.exact_union(a, inner)
    assert _digest(u2) == _digest(a)
    # The separate union refuses (the disconnected).
    far = _sphere(0.8, cx=5.0)
    try:
        brep_mod.exact_union(a, far)
        raise AssertionError("the separate union must refuse")
    except ValueError as ex:
        assert "disconnected" in str(ex)
    # The separate subtract is the miss (the exact copy).
    s = brep_mod.exact_subtract(a, far)
    assert _digest(s) == _digest(a)


def test_determinism_every_op():
    a = _sphere(1.0)
    b = _sphere(0.8, cx=1.2)
    tool = _circle_prism(0.4, -2.0, 2.0)
    box = _box(Vec3f(1.5, 1.5, 1.5))
    ops = [
        (brep_mod.exact_union, (a, b, Vec3f(0, 0, 0))),
        (brep_mod.exact_intersect, (a, b, Vec3f(0, 0, 0))),
        (brep_mod.exact_subtract, (a, b, Vec3f(0, 0, 0))),
        (brep_mod.exact_union, (a, tool, Vec3f(0, 0, 0))),
        (brep_mod.exact_intersect, (a, tool, Vec3f(0, 0, 0))),
        (brep_mod.exact_subtract, (a, tool, Vec3f(0, 0, 0))),
        (brep_mod.exact_subtract, (a, box, Vec3f(0, 0, 2.2))),
        (brep_mod.exact_union, (a, box, Vec3f(0, 0, 2.2))),
    ]
    for fn, args in ops:
        r1 = fn(*args)
        r2 = fn(*args)
        assert _digest(r1) == _digest(r2)
