"""G1 unit 4 — the exact subtract through the bindings (ADR-0032 D7).

The C++ side pins the SSI entry, the class check, the builder's tables
and the counts (tests/unit/brep/test_g1_unit4_subtract.cpp). What
THESE tests pin is what crosses the binding intact — the H1
consumer's view of the perforated body:

  * ``SceneNode.exact_brep`` on a SUBTRACT node — the HybridBody's
    B-Rep member, read through the unit-2 query surface: the wall is
    ONE cylinder-family face (the today world's faceted extract of
    the same body is N planar strips), the pierced faces carry INNER
    LOOPS whose Arc2 p-curves carry the sketch's radius with
    ``hole=True`` (the flipped region bit — the material is outside
    the profile now);
  * the wall's own p-curves are Line2s with ``interior_left=False``
    — the region-on-right convention the unit discovers (the prism's
    physical walk, re-framed by the flipped orientation);
  * the honest refusal: an out-of-class target keeps its SDF, the
    reason visible in ``hybrid_refusal`` — never a silent downgrade;
  * determinism (ADR-0014): a radius edit cycle regenerates the exact
    B-Rep bit-identically (the query digest).

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import scene

V3 = CAD_0.math.Vec3f

PI = 3.14159265358979


def _close(a: float, b: float, tol: float = 1e-5) -> bool:
    return abs(a - b) <= tol


def _circle_sketch(center=(0.0, 0.0), radius=0.25):
    return [scene.SketchChain([scene.sketch_circle(center, radius, False)])]


def _guide_scene():
    """The guide case: the box [-1,1]^3 minus the circle prism r=0.25
    through Z (the tool offset lands its span on [-1.5, 1.5])."""
    s = scene.Scene()
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node("S", _circle_sketch())
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    did = s.add_operation_node("Sub", box, eid, 0.0, 0.0, -1.5)
    return s, box, sid, eid, did


def _query_digest(body) -> str:
    out = []
    for face_id in range(brep_mod.face_count(body)):
        out.append(repr(brep_mod.face_query(body, face_id)))
    return "\n".join(out)


def test_guide_case_end_to_end():
    s, _, _, _, did = _guide_scene()
    n = s.get_node(did)

    # The HybridBody landed on the SUBTRACT node: both materializations,
    # no refusal.
    assert n.exact_brep is not None
    assert n.hybrid_refusal == ""

    # F = 7 (4 sides + 2 pierced + 1 wall) — the pinned table's face
    # count, read through the binding.
    assert brep_mod.face_count(n.exact_brep) == 7

    # THE exact claim: the hole's wall is ONE cylinder-family face
    # (today's faceted bridge of the same body: N planar strips).
    families = {}
    for fid in range(7):
        q = brep_mod.face_query(n.exact_brep, fid)
        families[q["family"]] = families.get(q["family"], 0) + 1
        assert q["exact"] is True
        assert q["linear_tolerance"] == 0.0
    assert families == {"plane": 6, "cylinder": 1}

    # The wall (face 6): the seam-wired window walk, six Line2 p-curves,
    # every one region-on-RIGHT (interior_left=False) — the convention
    # the unit pins.
    wall = brep_mod.face_query(n.exact_brep, 6)
    assert wall["loop_count"] == 1
    ces = wall["loops"][0]["coedges"]
    assert len(ces) == 6
    for ce in ces:
        pc = ce["pcurve"]
        assert pc is not None and pc["type"] == "line"
        assert pc["interior_left"] is False
    # The seam's two co-edges live in this ONE loop, opposite flags.
    seam = [ce for ce in ces if ce["edge"] == ces[0]["edge"]]
    assert len(seam) == 2
    assert seam[0]["reversed"] != seam[1]["reversed"]

    # The pierced faces (0 = bottom -Z, 1 = top +Z): the OUTER loop is
    # the factory's own (4 Line2, material-on-left, bit-identical to
    # the box's); the INNER loop is the intersection circle split at
    # the seam pair — two Arc2 with the sketch's radius, hole=True.
    for fid in (0, 1):
        q = brep_mod.face_query(n.exact_brep, fid)
        assert q["family"] == "plane"
        assert q["loop_count"] == 2
        assert q["inner_loop_count"] == 1
        outer = q["loops"][0]["coedges"]
        assert len(outer) == 4
        for ce in outer:
            pc = ce["pcurve"]
            assert pc is not None and pc["type"] == "line"
            assert pc["interior_left"] is True
        inner = q["loops"][1]["coedges"]
        assert len(inner) == 2
        for ce in inner:
            pc = ce["pcurve"]
            assert pc is not None and pc["type"] == "arc"
            assert _close(pc["radius"], 0.25)
            assert _close(pc["sweep"], PI)
            assert pc["hole"] is True
        # The inner loop's p-curves chain endpoint-to-endpoint in the
        # domain (the DIRECTION convention): arc i's END is arc
        # (i+1)'s START.
        starts, ends = [], []
        for ce in inner:
            pc = ce["pcurve"]
            a0 = pc["start_angle"]
            a1 = a0 + pc["sweep"]
            starts.append((pc["center"][0] + pc["radius"] * math.cos(a0),
                           pc["center"][1] + pc["radius"] * math.sin(a0)))
            ends.append((pc["center"][0] + pc["radius"] * math.cos(a1),
                         pc["center"][1] + pc["radius"] * math.sin(a1)))
        for i in range(len(inner)):
            assert _close(ends[i][0], starts[(i + 1) % len(inner)][0])
            assert _close(ends[i][1], starts[(i + 1) % len(inner)][1])
    # The wall's usage of the same edges is oppositely oriented (the
    # radial XOR — read by hand here): every inner-loop co-edge's
    # (edge, reversed) pair has its mirrored twin among the wall's.
    wall_edges = {(ce["edge"], ce["reversed"]) for ce in ces}
    top_inner = brep_mod.face_query(n.exact_brep, 1)["loops"][1]["coedges"]
    for ce in top_inner:
        assert (ce["edge"], not ce["reversed"]) in wall_edges


def test_exact_wall_vs_faceted_today():
    s, _, _, _, did = _guide_scene()
    n = s.get_node(did)

    opts = bridge_mod.SdfToBrepOptions()
    opts.fit_primitives = False
    opts.march_resolution = 24
    faceted = bridge_mod.sdf_to_brep_faceted(n.body, opts)
    assert faceted is not None
    assert brep_mod.face_count(faceted) > 7
    for fid in range(brep_mod.face_count(faceted)):
        q = brep_mod.face_query(faceted, fid)
        assert q["family"] == "plane"
        assert q["exact"] is False
        assert q["linear_tolerance"] > 0.0
        for loop in q["loops"]:
            for ce in loop["coedges"]:
                assert ce["pcurve"] is None


def test_out_of_class_target_refusal_visible():
    # A sphere target: convex but not planar-faced — the refusal is
    # recorded, the SDF path stands, nothing regressed.
    s = scene.Scene()
    sph = s.add_node("Sphere", CAD_0.sdf.sphere(1.0))
    sid = s.add_sketch_node("S", _circle_sketch())
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    did = s.add_operation_node("Sub", sph, eid, 0.0, 0.0, -1.5)
    n = s.get_node(did)

    assert n.exact_brep is None
    assert n.hybrid_refusal != ""
    assert "non-planar" in n.hybrid_refusal
    # The SDF side materialized regardless.
    assert not n.operation_pending
    assert not n.operand_missing


def test_blind_hole_refusal_visible():
    # The tool ends inside the box: the blind hole's actionable
    # refusal, end to end through the node.
    s = scene.Scene()
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node("S", _circle_sketch())
    eid = s.add_extrude_node("Extrude", sid, 1.6, 1)
    did = s.add_operation_node("Sub", box, eid, 0.0, 0.0, -1.2)
    n = s.get_node(did)

    assert n.exact_brep is None
    assert "BLIND" in n.hybrid_refusal
    # The remedy is named.
    assert "Remedy" in n.hybrid_refusal


def test_radius_edit_determinism():
    s, _, sid, _, did = _guide_scene()
    n = s.get_node(did)
    assert n.exact_brep is not None
    d1 = _query_digest(n.exact_brep)

    # Edit the radius (still strictly inside), then back: the exact
    # B-Rep regenerates bit-identically.
    circle = s.sketch_chains(sid)[0][0]
    s.sketch_set_entity(sid, 0, 0, scene.sketch_circle(
        (circle["center"][0], circle["center"][1]), 0.4, False))
    s.rebuild_operations()
    assert s.get_node(did).exact_brep is not None
    assert _query_digest(s.get_node(did).exact_brep) != d1

    circle2 = s.sketch_chains(sid)[0][0]
    s.sketch_set_entity(sid, 0, 0, scene.sketch_circle(
        (circle2["center"][0], circle2["center"][1]), 0.25, False))
    s.rebuild_operations()
    assert _query_digest(s.get_node(did).exact_brep) == d1
