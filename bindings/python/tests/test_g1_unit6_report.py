"""G1 unit 6 — the report through the bindings (ADR-0032 D8 + D10.6).

The C++ side pins the body query's data (tests/unit/brep/
test_g1_unit6_report.cpp). What THESE tests pin is the report as its
consumers read it — the module the Debug menu, the panel and the
example all render through, so no two surfaces can disagree on a
number:

  * the wiring: ``CAD_0.brep.body_query`` and ``CAD_0.report`` exist
    (the AttributeError discriminator of D9);
  * THE GUIDE CASE: the v0.4 flow whose report data IS the D7.5
    table — counts 12/17/7/2, chi = 0, genus 1, the volume and the
    axial I_zz, the provenance all exact;
  * the rendered TEXT: the fields the Debug menu shows, including the
    honest chi label WITH the H term (the formula's own, since unit
    1) and the visible exact/approximated flag;
  * the honesty: the faceted extraction reports the polyhedral value
    with all_exact=False and the approximated tallies — never an
    exact in disguise;
  * the goldens: the v0.4 document opens, materializes exact, and its
    report agrees with the table; the v0.3 document opens, its exact
    path refuses (the refusal visible), and the faceted report agrees
    where it can (chi = 0, genus 1) while declaring itself
    approximated where it must;
  * determinism (ADR-0014): re-runs are bit-identical;
  * the refusal (P2): the report data carries the mass refusal
    visibly, never a wrong number.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

from pathlib import Path

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import report as report_mod
from CAD_0 import scene

V3 = CAD_0.math.Vec3f
PI = 3.141592653589793

_ROOT = Path(__file__).resolve().parents[3]


def _close(a: float, b: float, rel: float = 1e-6,
           abs_tol: float = 1e-6) -> bool:
    return abs(a - b) <= max(abs_tol, rel * max(abs(a), abs(b)))


def _guide_scene(radius: float = 0.25):
    """The guide case: the box [-1,1]^3 minus the circle prism through
    Z (the v0.4 golden's own flow, the D7.5 table's own geometry)."""
    s = scene.Scene()
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node(
        "S", [scene.SketchChain(
            [scene.sketch_circle((0.0, 0.0), radius, False)])])
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    did = s.add_operation_node("Sub", box, eid, 0.0, 0.0, -1.5)
    return s, sid, did


def test_the_api_is_exposed():
    # The wiring discriminators (D9): the query and the module.
    assert hasattr(brep_mod, "body_query")
    assert hasattr(report_mod, "body_report_data")
    assert hasattr(report_mod, "body_report_text")
    assert hasattr(report_mod, "provenance_totals")


def test_guide_case_body_query_the_whole_answer():
    s, _sid, did = _guide_scene()
    n = s.get_node(did)
    assert n.exact_brep is not None

    q = brep_mod.body_query(n.exact_brep)
    assert q["V"] == 12
    assert q["E"] == 17
    assert q["F"] == 7
    assert q["H"] == 2
    assert q["chi"] == 0
    assert q["genus"] == 1
    assert q["orientable"] is True
    assert q["loops_closed"] is True
    # The D8 tallies: all exact, nothing approximated, tol 0.
    for kind in ("vertices", "edges", "faces"):
        assert q[kind]["approximated"] == 0
        assert q[kind]["max_linear_tolerance"] == 0.0
    assert q["vertices"]["exact"] == 12
    assert q["edges"]["exact"] == 17
    assert q["faces"]["exact"] == 7


def test_guide_case_report_data_is_the_d75_table():
    s, _sid, did = _guide_scene()
    data = report_mod.body_report_data(s.get_node(did).exact_brep)

    # abc = 8, r = 0.25, h = 2 (the pierced span).
    assert data["mass_error"] is None
    assert data["all_exact"] is True
    assert _close(data["volume"], 8.0 - PI * 0.0625 * 2.0)
    cx, cy, cz = data["centroid"]
    assert abs(cx) <= 1e-8 and abs(cy) <= 1e-8 and abs(cz) <= 1e-8
    izz = data["inertia"][2][2]
    assert _close(izz, 8.0 * 8.0 / 12.0 - PI * 0.25 ** 4 * 2.0 / 2.0,
                  1e-5)
    # The symmetric body: the products vanish.
    assert abs(data["inertia"][0][1]) <= 1e-8

    # The D8 aggregates (36 subshapes, all exact).
    n_exact, n_approx, max_tol = report_mod.provenance_totals(data)
    assert (n_exact, n_approx, max_tol) == (36, 0, 0.0)


def test_guide_case_report_text_the_debug_menu_surface():
    s, _sid, did = _guide_scene()
    text = report_mod.body_report_text(
        s.get_node(did).exact_brep,
        "exact materialization (HybridBody, bridge tolerance 0)")

    # The structural block, with the H term IN THE LABEL (honest
    # since unit 1 — the pierced faces make the term visible).
    assert "V = 12, E = 17, F = 7, H (inner loops) = 2" in text
    assert "\u03c7 = V - E + F - H = 12 - 17 + 7 - 2 = 0" in text
    assert "Genus = 1" in text
    assert "Closed orientable (is_closed_orientable): yes" in text

    # The mass block: the readable form (diagonal + products, not a
    # wall of floats) and the visible flag.
    assert "Mass properties (density 1, divergence theorem): " \
           "exact (all_exact = True)" in text
    assert "Volume = 7.60730091" in text
    assert "diagonal  Ixx =" in text
    assert "products  Ixy =" in text

    # The provenance block (D8: per body, per kind).
    assert "Provenance (Layer 2): 36 subshapes exact / " \
           "0 approximated, max tolerance 0" in text
    assert "vertices  12 exact / 0 approximated" in text
    assert "edges     17 exact / 0 approximated" in text
    assert "faces     7 exact / 0 approximated" in text

    # The path label the caller provides.
    assert text.startswith("Path: exact materialization")


def test_the_faceted_world_says_approximated():
    # The guide body through the FACETED bridge (fit off, resolution
    # 24): the polyhedral value, all_exact=False, every subshape
    # approximated with the grid's declared bound — never an exact in
    # disguise.
    s, _sid, did = _guide_scene()
    opts = bridge_mod.SdfToBrepOptions()
    opts.fit_primitives = False
    opts.march_resolution = 24
    faceted = bridge_mod.sdf_to_brep_faceted(s.get_node(did).body, opts)
    assert faceted is not None

    data = report_mod.body_report_data(faceted)
    assert data["mass_error"] is None
    assert data["all_exact"] is False
    assert _close(data["volume"], 8.0 - PI * 0.0625 * 2.0, 2e-2, 2e-2)
    # Same topology, coarser geometry.
    assert data["chi"] == 0 and data["genus"] == 1
    n_exact, n_approx, max_tol = report_mod.provenance_totals(data)
    assert n_exact == 0 and n_approx > 0 and max_tol > 0.0

    text = report_mod.body_report_text(faceted, "faceted extraction")
    assert "APPROXIMATED (all_exact = False" in text
    assert "0 subshapes exact / " in text


def test_the_multi_shell_genus_is_n_d():
    # Two disjoint spheres through the faceted bridge: chi = 4 > 2,
    # the genus is NOT derived — rendered as n/d, never a fabricated
    # half-integer (the ADR-0027 D6 rule; the C++ suite pins the
    # counts on the union body, this pins the Python rendering).
    a = CAD_0.sdf.sphere(1.0)
    b = CAD_0.sdf.translate(
        CAD_0.sdf.sphere(1.0), V3(4.0, 0.0, 0.0))
    two = CAD_0.sdf.union(a, b)
    opts = bridge_mod.SdfToBrepOptions()
    body = bridge_mod.sdf_to_brep(two, opts)
    assert body is not None

    q = brep_mod.body_query(body)
    assert q["chi"] == 4
    assert q["orientable"] is True
    assert q["genus"] is None

    text = report_mod.body_report_text(body)
    assert "Genus = n/d (multi-shell? chi > 2" in text


def test_the_mass_refusal_crosses_verbatim():
    # The P2 shape: when the mass API refuses, the report data
    # carries the diagnosis verbatim and the mass keys stay None —
    # never a wrong number, never a silent gap. The sanctioned
    # trigger the suite can reach honestly: a Layer-3 tolerance so
    # wide that tol^2 bands every loop's domain area as degenerate
    # (the Green kernels' own precondition) refuses with the
    # diagnosis inside — verified empirically before pinning.
    s, _sid, did = _guide_scene()
    body = s.get_node(did).exact_brep
    with pytest.raises(ValueError):
        brep_mod.mass_properties(body, 1e7)
    # The report module surfaces it, verbatim, without raising.
    data = report_mod.body_report_data(body, 1e7)
    assert data["mass_error"] is not None
    assert "degenerate" in data["mass_error"].lower()
    assert data["volume"] is None and data["all_exact"] is None
    text = report_mod.body_report_text(body, tol=1e7)
    assert "Mass properties: n/d" in text


def test_determinism_bit_identical():
    s, _sid, did = _guide_scene()
    body = s.get_node(did).exact_brep
    d1 = report_mod.body_report_data(body)
    d2 = report_mod.body_report_data(body)
    assert d1 == d2
    assert report_mod.body_report_text(body) == \
        report_mod.body_report_text(body)

    # A fresh scene, the same recipe: the same report bit-identically.
    s2, _sid2, did2 = _guide_scene()
    d3 = report_mod.body_report_data(s2.get_node(did2).exact_brep)
    assert d3 == d1


def test_golden_v0_4_report_agrees_with_the_table():
    path = _ROOT / "tests" / "golden" / "native_v0_4_0.CAD_0"
    assert path.exists()
    s = scene.Scene()
    CAD_0.io.load(path, s)

    sub = s.get_node(4)
    assert sub is not None
    assert sub.exact_brep is not None  # materialized exact on load

    data = report_mod.body_report_data(sub.exact_brep)
    assert data["V"] == 12 and data["E"] == 17 and data["F"] == 7
    assert data["H"] == 2 and data["chi"] == 0 and data["genus"] == 1
    assert data["all_exact"] is True
    assert _close(data["volume"], 8.0 - PI * 0.0625 * 2.0)
    assert _close(data["inertia"][2][2],
                  8.0 * 8.0 / 12.0 - PI * 0.25 ** 4 * 2.0 / 2.0, 1e-5)


def test_golden_v0_3_the_honest_faceted_twin():
    path = _ROOT / "tests" / "golden" / "native_v0_3_0.CAD_0"
    assert path.exists()
    s = scene.Scene()
    CAD_0.io.load(path, s)

    sub = s.get_node(3)
    assert sub is not None
    # The Y-axis tool: the exact path refuses, the refusal visible.
    assert sub.exact_brep is None
    assert sub.hybrid_refusal != ""

    # The report path for this body is the dispatching extraction —
    # faceted (the tool is not a prism, the result not a primitive):
    # same topology (chi = 0, genus 1), every number approximated.
    opts = bridge_mod.SdfToBrepOptions()
    faceted = bridge_mod.sdf_to_brep(sub.body, opts)
    data = report_mod.body_report_data(faceted)
    assert data["chi"] == 0 and data["genus"] == 1
    assert data["all_exact"] is False
    n_exact, n_approx, max_tol = report_mod.provenance_totals(data)
    assert n_exact == 0 and n_approx > 0 and max_tol > 0.0
    assert _close(data["volume"], 8.0 - PI * 0.0625 * 2.0, 2e-2, 2e-2)


def test_the_example_walks_the_chain_in_ci():
    """AREA 4: the example runs in the CI (a suite member, not a
    document) — every D7.5 row PASSES, the edit is bit-identical, the
    undo is ONE entry."""
    import subprocess
    import sys

    example = _ROOT / "examples" / "sketch_hole.py"
    assert example.exists()
    result = subprocess.run(
        [sys.executable, str(example)],
        capture_output=True, text=True, timeout=600,
        cwd=str(_ROOT))
    out = result.stdout
    assert result.returncode == 0, result.stderr[-2000:]
    # The two tables (before and after the radius edit).
    assert out.count("LA TAVOLA DI D7.5") == 2
    assert out.count("PASS") >= 20  # 10 rows x 2 tables
    assert "DIFF:" not in out
    # The edit's discipline.
    assert "bit-identical" in out
    assert "ONE undo entry" in out or "1 entry" in out
    # The guide numbers, visible for the hand-check.
    assert "7.60730091" in out
    assert "12 - 17 + 7 - 2 = 0" in out


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
