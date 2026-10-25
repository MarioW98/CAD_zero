"""G1 unit 3 — the exact extrude through the bindings (ADR-0032 D6).

The C++ side pins the p-curve tables, the conventions and the counts
(tests/unit/brep/test_g1_unit3_extrude.cpp). What THESE tests pin is
what crosses the binding intact — the proof that the unit's contract
is USABLE from Python, not only from C++ (the H1 consumer's view):

  * ``SceneNode.exact_brep`` — the extrude node's exact B-Rep (the
    HybridBody's B-Rep member), read through the unit-2 query
    surface: families, loops, per-co-edge p-curve types, exact/0;
  * the circle's prism: ONE cylindrical wall (the exact claim) vs
    the faceted bridge of the same profile's SDF (the today world);
  * the ring (the tested multi-chain flow): the SDF field UNCHANGED,
    the exact B-Rep added — the caps carry the hole as an inner
    Circle2 loop;
  * the honesty chain: the extruded prism says exact, the subtracted
    body's bridge extract says approximated with no p-curves;
  * the out-of-class profile (a notch-side arc): the SDF stands, the
    exact side's refusal is recorded (``hybrid_refusal``);
  * determinism (ADR-0014): a radius edit cycle regenerates the
    exact B-Rep bit-identically, and a save/load round-trip
    re-materializes it identically (the format does not move).

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import sdf as sdf_mod

scene = CAD_0.scene
sketch_segment = scene.sketch_segment
sketch_circle = scene.sketch_circle
sketch_arc = scene.sketch_arc
SketchChain = scene.SketchChain
V3 = CAD_0.math.Vec3f

PI = 3.14159265358979
TWO_PI = 6.283185307179586


def _close(a: float, b: float, tol: float = 1e-5) -> bool:
    return abs(a - b) <= tol


def _circle_sketch(center=(0.5, 0.25), radius=1.25):
    return [SketchChain([sketch_circle(center, radius, False)])]


def _rect_sketch(lo=(-2.0, -1.0), hi=(2.0, 1.0)):
    return [SketchChain([
        sketch_segment(lo, (hi[0], lo[1]), True),
        sketch_segment((hi[0], lo[1]), hi, True),
        sketch_segment(hi, (lo[0], hi[1]), True),
        sketch_segment((lo[0], hi[1]), lo, True),
    ])]


def _ring_sketch():
    return _rect_sketch() + [SketchChain([sketch_circle((0.0, 0.0), 0.2, True)])]


def _notch_sketch():
    # The square [0,3]x[0,2] with a bite at the top-right corner:
    # walked clockwise around the material (interior_left=False),
    # the bite arc walked CCW with the material on its right
    # (hole=True) — OUT of the exact class, IN the SDF class.
    return [SketchChain([
        sketch_segment((0.0, 0.0), (0.0, 2.0), False),
        sketch_segment((0.0, 2.0), (1.6, 2.0), False),
        sketch_arc((3.0, 2.0), 1.4, True, PI, 0.5 * PI),
        sketch_segment((3.0, 0.6), (3.0, 0.0), False),
        sketch_segment((3.0, 0.0), (0.0, 0.0), False),
    ])]


def _query_digest(body) -> str:
    """The full query answer as a comparable string (repr of plain
    values — the bit-identity oracle for the determinism checks)."""
    out = []
    for face_id in range(brep_mod.face_count(body)):
        q = brep_mod.face_query(body, face_id)
        out.append(repr(q))
    return "\n".join(out)


def test_circle_prism_exact_brep_end_to_end():
    s = scene.Scene()
    sid = s.add_sketch_node("S", _circle_sketch())
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    n = s.get_node(eid)

    # The HybridBody landed: the exact B-Rep is present, no refusal.
    assert n.exact_brep is not None
    assert n.hybrid_refusal == ""

    # THE exact claim: the lateral face is ONE cylinder (the today
    # world's faceted extract of the same profile's SDF is N planar
    # strips — the contrast test below).
    assert brep_mod.face_count(n.exact_brep) == 3
    wall = brep_mod.face_query(n.exact_brep, 0)
    assert wall["family"] == "cylinder"
    assert wall["loop_count"] == 1
    ces = wall["loops"][0]["coedges"]
    assert len(ces) == 6  # the seam-wired rectangle walk
    for ce in ces:
        pc = ce["pcurve"]
        assert pc is not None and pc["type"] == "line"
        assert pc["interior_left"] is True

    # The seam pair: one edge used forward and reversed in the loop.
    seam = [ce for ce in ces if ce["edge"] == ces[0]["edge"]]
    assert len(seam) == 2
    assert seam[0]["reversed"] != seam[1]["reversed"]

    # The caps: plane discs, two pi-sweep arcs each, exact/0.
    for cap_id in (1, 2):
        cap = brep_mod.face_query(n.exact_brep, cap_id)
        assert cap["family"] == "plane"
        assert cap["loop_count"] == 1
        assert cap["inner_loop_count"] == 0
        assert cap["exact"] is True
        assert cap["linear_tolerance"] == 0.0
        arcs = cap["loops"][0]["coedges"]
        assert len(arcs) == 2
        for ce in arcs:
            pc = ce["pcurve"]
            assert pc is not None and pc["type"] == "arc"
            assert _close(pc["sweep"], PI)
            assert pc["hole"] is False


def test_circle_prism_vs_faceted_today():
    # The same profile's SDF through the faceted bridge: N planar
    # strips, no p-curves, approximated with the declared grid
    # tolerance. Both honest; only one exact.
    s = scene.Scene()
    sid = s.add_sketch_node("S", _circle_sketch())
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    n = s.get_node(eid)

    opts = bridge_mod.SdfToBrepOptions()
    opts.fit_primitives = False
    opts.march_resolution = 24
    faceted = bridge_mod.sdf_to_brep_faceted(n.body, opts)
    assert faceted is not None
    assert brep_mod.face_count(faceted) > 3
    for face_id in range(brep_mod.face_count(faceted)):
        q = brep_mod.face_query(faceted, face_id)
        assert q["family"] == "plane"
        assert q["exact"] is False
        assert q["linear_tolerance"] > 0.0
        for loop in q["loops"]:
            for ce in loop["coedges"]:
                assert ce["pcurve"] is None


def test_segment_prism_query():
    s = scene.Scene()
    sid = s.add_sketch_node("S", _rect_sketch())
    eid = s.add_extrude_node("Extrude", sid, 1.5, 1)
    n = s.get_node(eid)
    assert n.exact_brep is not None

    # 4 planar laterals + 2 caps; every lateral a 4-Line2 rectangle
    # walk; the caps' loops the chain itself, 4 Line2 each.
    assert brep_mod.face_count(n.exact_brep) == 6
    for face_id in range(6):
        q = brep_mod.face_query(n.exact_brep, face_id)
        assert q["family"] == "plane"
        assert q["exact"] is True
        ces = q["loops"][0]["coedges"]
        assert len(ces) == 4
        for ce in ces:
            pc = ce["pcurve"]
            assert pc is not None and pc["type"] == "line"

    # The lateral rectangle chains endpoint-to-endpoint.
    for face_id in range(4):
        ces = brep_mod.face_query(n.exact_brep, face_id)["loops"][0]["coedges"]
        pts = [(ce["pcurve"]["a"], ce["pcurve"]["b"]) for ce in ces]
        for i in range(4):
            b = pts[i][1]
            a_next = pts[(i + 1) % 4][0]
            assert _close(b[0], a_next[0]) and _close(b[1], a_next[1])


def test_ring_keeps_sdf_and_gains_exact_brep():
    # The TESTED multi-chain flow (test_sketch_scene's ring): the SDF
    # field is UNCHANGED (the display path never moved) and the exact
    # B-Rep is added beside it.
    s = scene.Scene()
    sid = s.add_sketch_node("S", _ring_sketch())
    assert s.sketch_closed(sid)
    eid = s.add_extrude_node("Extrude", sid, 1.5, 1)
    n = s.get_node(eid)

    # The SDF: pinned analytic samples for THIS rectangle
    # [−2,2]×[−1,1] + the r=0.2 hole (the max-composition, by hand):
    # mid-slab inside the material the hole's complement field wins
    # (0.2 − 0.7 = −0.5); the hole's centre reads the hole's own
    # +0.2 (no material); below the bottom cap the slab wins.
    assert _close(n.body.value(V3(0.7, 0.0, 0.75)), -0.5, 1e-6)
    assert _close(n.body.value(V3(0.0, 0.0, 0.75)), 0.2, 1e-6)
    assert _close(n.body.value(V3(0.7, 0.0, -0.1)), 0.1, 1e-6)

    # The exact side: 4 laterals + the hole's cylindrical wall + 2
    # caps; each cap carries the hole as an INNER Circle2 loop.
    assert n.exact_brep is not None
    assert brep_mod.face_count(n.exact_brep) == 7
    hole_wall = brep_mod.face_query(n.exact_brep, 4)
    assert hole_wall["family"] == "cylinder"
    assert len(hole_wall["loops"][0]["coedges"]) == 4

    for cap_id in (5, 6):
        cap = brep_mod.face_query(n.exact_brep, cap_id)
        assert cap["family"] == "plane"
        assert cap["loop_count"] == 2
        assert cap["inner_loop_count"] == 1
        assert cap["loops"][0]["is_outer"] is True
        assert cap["loops"][1]["is_outer"] is False
        # The outer loop: the rectangle chain (4 Line2).
        assert len(cap["loops"][0]["coedges"]) == 4
        # The inner loop: ONE co-edge, the Circle2 with hole=True —
        # the annulus is the entity's region (the first Circle2
        # writer).
        inner = cap["loops"][1]["coedges"]
        assert len(inner) == 1
        pc = inner[0]["pcurve"]
        assert pc is not None and pc["type"] == "circle"
        assert _close(pc["radius"], 0.2)
        assert pc["hole"] is True


def test_honesty_chain_prism_exact_subtracted_approximated():
    # The guide case's shape: a box, a circle sketch, the extrude,
    # the subtract consuming the extrude's SDF side.
    s = scene.Scene()
    bid = s.add_node("Box", sdf_mod.box(V3(1.5, 1.5, 1.5)))
    sid = s.add_sketch_node("S", _circle_sketch((0.0, 0.0), 0.4))
    eid = s.add_extrude_node("Extrude", sid, 4.0, 1)
    did = s.add_operation_node("Sub", bid, eid, 0.0, 0.0, 1.0)

    ex = s.get_node(eid)
    dn = s.get_node(did)

    # The prism says exact.
    assert ex.exact_brep is not None
    q = brep_mod.face_query(ex.exact_brep, 0)
    assert q["exact"] is True
    assert q["linear_tolerance"] == 0.0

    # (G1 unit 4 amended this clause: the subtract's exact side
    # LANDED. This scene's tool spans z in [1, 5] against the box's
    # [-1.5, 1.5] - a BLIND hole, and the exact subtract refuses it
    # honestly: the actionable reason is recorded, the SDF display
    # path stands. The in-class through-hole materializes exact and
    # is pinned by unit 4's own suite.)
    assert dn.exact_brep is None
    assert "BLIND" in dn.hybrid_refusal
    assert "Remedy" in dn.hybrid_refusal

    # Its bridge extract stays faceted/approximated — the honesty
    # chain, unchanged (the unit-1 provenance rules propagate the
    # worst).
    opts = bridge_mod.SdfToBrepOptions()
    opts.fit_primitives = False
    opts.march_resolution = 20
    faceted = bridge_mod.sdf_to_brep_faceted(dn.body, opts)
    assert faceted is not None
    for face_id in range(brep_mod.face_count(faceted)):
        q = brep_mod.face_query(faceted, face_id)
        assert q["exact"] is False
        assert q["linear_tolerance"] > 0.0
        for loop in q["loops"]:
            for ce in loop["coedges"]:
                assert ce["pcurve"] is None


def test_out_of_class_refusal_recorded():
    s = scene.Scene()
    sid = s.add_sketch_node("S", _notch_sketch())
    # The creation SUCCEEDS (the SDF certifies — no visible change to
    # the flow); the exact side's refusal is recorded, not thrown.
    eid = s.add_extrude_node("Extrude", sid, 1.0, 1)
    n = s.get_node(eid)

    assert n.exact_brep is None
    assert n.hybrid_refusal != ""
    assert "CCW" in n.hybrid_refusal
    assert "Remedy" in n.hybrid_refusal

    # The SDF stands (the display continues).
    assert n.operand_missing is False
    assert n.operation_pending is False
    assert n.body.value(V3(1.5, 1.0, 0.5)) < 0.0  # inside the material

    # An in-class extrude in the SAME scene lands its hybrid (the
    # refusal is per-node, never global).
    rid = s.add_sketch_node("R", _rect_sketch())
    reid = s.add_extrude_node("RE", rid, 1.0, 1)
    assert s.get_node(reid).exact_brep is not None


def test_radius_edit_cycle_bit_identical():
    s = scene.Scene()
    sid = s.add_sketch_node("S", _circle_sketch((0.5, 0.25), 1.25))
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    n = s.get_node(eid)
    assert n.exact_brep is not None

    before = _query_digest(n.exact_brep)

    # The edit: the radius 1.25 -> 1.1.
    s.sketch_set_entity(sid, 0, 0, sketch_circle((0.5, 0.25), 1.1, False))
    r1 = s.rebuild_operations()
    assert r1["rebuilt"] == 1
    mid = _query_digest(n.exact_brep)
    assert mid != before  # the edit really re-materialized

    # The edit back: both materializations regenerate BIT-IDENTICALLY
    # (the query's plain values are the serialization).
    s.sketch_set_entity(sid, 0, 0, sketch_circle((0.5, 0.25), 1.25, False))
    r2 = s.rebuild_operations()
    assert r2["rebuilt"] == 1
    assert _query_digest(n.exact_brep) == before


def test_save_load_rematerializes_identically():
    # The format does not move (D11): the exact B-Rep is derived
    # state, re-materialized on load — deterministically.
    s = scene.Scene()
    sid = s.add_sketch_node("S", _circle_sketch((0.5, 0.25), 1.25))
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    n = s.get_node(eid)
    digest = _query_digest(n.exact_brep)

    camera = {
        "target": (0.5, -0.25, 1.0),
        "distance": 8.25,
        "yaw": 0.625,
        "pitch": -0.125,
        "fov_deg": 55.0,
    }
    with tempfile.TemporaryDirectory(prefix="cad0_u3_") as tmp:
        path = Path(tmp) / "proj.CAD_0"
        CAD_0.io.save(path, s, camera)

        s2 = scene.Scene()
        CAD_0.io.load(path, s2)
        n2 = s2.get_node(eid)
        assert n2 is not None
        assert n2.exact_brep is not None
        assert _query_digest(n2.exact_brep) == digest
