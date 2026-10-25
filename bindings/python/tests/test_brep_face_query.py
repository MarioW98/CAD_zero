"""G1 unit 2 — the face query through the bindings (ADR-0032 D1.1, H1 taking shape).

The C++ side pins the p-curve tables and the conventions
(tests/unit/brep/test_g1_unit2_pcurves.cpp). What THESE tests pin is
what crosses the binding intact — the proof that the contract is
USABLE from Python, not only from C++:

  * ``CAD_0.brep.face_query`` / ``CAD_0.brep.face_count`` exist and
    return plain values (the preprocessor's "what is this face?"):
    family, loop counts, per-co-edge p-curve type + readable
    parameters, provenance;
  * the exact primitives (built via ``CAD_0.bridge.sdf_to_brep``,
    whose primitive-fit path returns FACTORY bodies — the delegation)
    read: every co-edge carries a typed p-curve, exact/0;
  * the box's first face: the pinned Line2 chain, chained
    endpoint-to-endpoint in the domain;
  * the sphere: the seam-wired pair on the slit rectangle, chained
    across the periodic seam (u differing by exactly 2π);
  * the cylinder: the wall's six-Line2 rectangle walk and the caps'
    two π-sweep Arc2 pair;
  * the faceted world reads honestly: family "plane", pcurve None,
    approximated with the declared grid tolerance;
  * a bad face id raises ValueError carrying the remedy.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import sdf as sdf_mod

Vec3f = CAD_0.math.Vec3f

PI = 3.14159265358979
TWO_PI = 6.283185307179586

TOL = 1e-5


def _close(a: float, b: float, tol: float = TOL) -> bool:
    return abs(a - b) <= tol


def _close_pt(a, b, tol: float = TOL) -> bool:
    return _close(a[0], b[0], tol) and _close(a[1], b[1], tol)


def _pt(v) -> tuple[float, float]:
    return (float(v[0]), float(v[1]))


def _exact_box(half=Vec3f(1.0, 2.0, 3.0)):
    return bridge_mod.sdf_to_brep(sdf_mod.box(half))


def _exact_sphere(r: float = 2.0):
    return bridge_mod.sdf_to_brep(sdf_mod.sphere(r))


def _exact_cylinder(r: float = 1.0, h: float = 2.0):
    return bridge_mod.sdf_to_brep(sdf_mod.cylinder(r, h))


def _faceted_opts() -> bridge_mod.SdfToBrepOptions:
    opts = bridge_mod.SdfToBrepOptions()
    opts.march_resolution = 16
    opts.fit_primitives = False
    opts.compute_normals = True
    return opts


def test_face_count_and_query_shape_box():
    body = _exact_box()
    assert brep_mod.face_count(body) == 6

    q = brep_mod.face_query(body, 0)
    assert q["family"] == "plane"
    assert q["has_surface"] is True
    assert q["loop_count"] == 1
    assert q["outer_loop_count"] == 1
    assert q["inner_loop_count"] == 0
    assert q["exact"] is True
    assert q["linear_tolerance"] == 0.0
    assert len(q["loops"]) == 1
    assert q["loops"][0]["is_outer"] is True

    ces = q["loops"][0]["coedges"]
    assert len(ces) == 4
    # The pinned first co-edge of the bottom face: edge 3 reversed,
    # line from (-1, 2) to (-1, -2), material on the left — the same
    # golden the C++ suite pins.
    c0 = ces[0]
    assert c0["edge"] == 3
    assert c0["reversed"] is True
    pc = c0["pcurve"]
    assert pc is not None and pc["type"] == "line"
    assert _close_pt(pc["a"], (-1.0, 2.0))
    assert _close_pt(pc["b"], (-1.0, -2.0))
    assert pc["interior_left"] is True


def test_box_lines_chain_endpoint_to_endpoint():
    body = _exact_box()
    for face_id in range(brep_mod.face_count(body)):
        q = brep_mod.face_query(body, face_id)
        ces = q["loops"][0]["coedges"]
        pts = []
        for ce in ces:
            pc = ce["pcurve"]
            assert pc is not None and pc["type"] == "line"
            assert pc["interior_left"] is True
            pts.append((_pt(pc["a"]), _pt(pc["b"])))
        n = len(pts)
        assert n == 4
        for i in range(n):
            assert _close_pt(pts[i][1], pts[(i + 1) % n][0]), (
                f"face {face_id}: co-edge {i} does not chain"
            )


def test_sphere_seam_pair_on_the_slit_rectangle():
    body = _exact_sphere()
    assert brep_mod.face_count(body) == 1
    q = brep_mod.face_query(body, 0)
    assert q["family"] == "sphere"
    assert q["loop_count"] == 1

    ces = q["loops"][0]["coedges"]
    assert len(ces) == 2
    fwd, rev = ces[0]["pcurve"], ces[1]["pcurve"]
    assert fwd["type"] == "line" and rev["type"] == "line"
    assert fwd["interior_left"] is True
    assert rev["interior_left"] is True
    # The forward copy walks the window's RIGHT edge (u = 2π), the
    # reversed copy the LEFT edge (u = 0) — north→south and back.
    assert _close_pt(fwd["a"], (TWO_PI, 0.0))
    assert _close_pt(fwd["b"], (TWO_PI, PI))
    assert _close_pt(rev["a"], (0.0, PI))
    assert _close_pt(rev["b"], (0.0, 0.0))
    # The seam chains across the periodic identification: the walk
    # crosses the poles, where the domain jumps by exactly one period.
    assert _close(fwd["b"][0] - rev["a"][0], TWO_PI)
    assert _close(rev["b"][0] - fwd["a"][0], -TWO_PI)


def test_cylinder_wall_rectangle_walk_and_cap_arcs():
    body = _exact_cylinder()
    assert brep_mod.face_count(body) == 3

    wall = brep_mod.face_query(body, 0)
    assert wall["family"] == "cylinder"
    assert wall["loop_count"] == 1
    ces = wall["loops"][0]["coedges"]
    assert len(ces) == 6
    # The rectangle walk in the (θ, axis) window [0, 2π] × [-1, 1]:
    # right seam up, top west, left seam down, bottom east — chained
    # with NO seam jumps (the rim arcs span the full window).
    corners = [
        (TWO_PI, -1.0), (TWO_PI, 1.0), (PI, 1.0), (0.0, 1.0),
        (0.0, -1.0), (PI, -1.0),
    ]
    for i, ce in enumerate(ces):
        pc = ce["pcurve"]
        assert pc is not None and pc["type"] == "line", f"co-edge {i}"
        assert pc["interior_left"] is True
        assert _close_pt(pc["a"], corners[i]), f"co-edge {i} start"
        assert _close_pt(pc["b"], corners[(i + 1) % 6]), f"co-edge {i} end"

    for cap_id in (1, 2):
        cap = brep_mod.face_query(body, cap_id)
        assert cap["family"] == "plane"
        assert cap["loop_count"] == 1
        arcs = cap["loops"][0]["coedges"]
        assert len(arcs) == 2
        for arc_ce in arcs:
            pc = arc_ce["pcurve"]
            assert pc is not None and pc["type"] == "arc"
            assert _close_pt(pc["center"], (0.0, 0.0))
            assert _close(pc["radius"], 1.0)
            assert pc["hole"] is False
            assert _close(pc["sweep"], PI)
        assert _close(arcs[0]["pcurve"]["start_angle"], -PI / 2)
        assert _close(arcs[1]["pcurve"]["start_angle"], PI / 2)


def test_faceted_world_reads_honestly():
    blend = sdf_mod.smooth_union(
        sdf_mod.sphere(0.8), sdf_mod.sphere(0.6), 0.2
    )
    body = bridge_mod.sdf_to_brep_faceted(blend, _faceted_opts())
    assert brep_mod.face_count(body) > 0

    for face_id in range(min(4, brep_mod.face_count(body))):
        q = brep_mod.face_query(body, face_id)
        assert q["family"] == "plane"
        assert q["loop_count"] == 1
        assert q["outer_loop_count"] == 1
        assert q["exact"] is False
        assert q["linear_tolerance"] > 0.0
        for loop in q["loops"]:
            for ce in loop["coedges"]:
                assert ce["pcurve"] is None


def test_bad_face_id_is_a_loud_valueerror():
    body = _exact_box()
    with pytest.raises(ValueError, match="no face with id"):
        brep_mod.face_query(body, 99)
    with pytest.raises(ValueError, match="no face with id"):
        brep_mod.face_query(body, 6)
    # The valid range is [0, face_count): the last face queries fine.
    assert brep_mod.face_query(body, 5)["family"] == "plane"


def test_sdf_to_brep_dispatch_returns_factory_bodies():
    # The primitive-fit path is a DELEGATION: the body it returns is
    # a factory product, so it carries the factory's p-curves and the
    # factory's exact/0 provenance — while the faceted fallback (the
    # bridge's own writing) carries none of the former and declares
    # the latter. Both pinned above; this test pins the dispatch
    # itself: same SDF, different options, two different worlds.
    sph = sdf_mod.sphere(1.5)
    exact = bridge_mod.sdf_to_brep(sph)
    q = brep_mod.face_query(exact, 0)
    assert q["family"] == "sphere"
    assert q["exact"] is True
    assert all(
        ce["pcurve"] is not None for lo in q["loops"] for ce in lo["coedges"]
    )

    faceted = bridge_mod.sdf_to_brep(sph, _faceted_opts())
    qf = brep_mod.face_query(faceted, 0)
    assert qf["exact"] is False
    assert all(
        ce["pcurve"] is None for lo in qf["loops"] for ce in lo["coedges"]
    )
