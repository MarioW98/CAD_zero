"""H2-u5 m2 — the union door through the bindings (ADR-0037, the overlay).

The C++ side pins the door's goldens (tests/unit/brep/test_boolean.cpp:
D5's numbers written before the code, red on the pre-overlay tree,
green with the overlay). What THESE tests pin is the door REACHABLE
from Python — the three cases of the wall map's one refusing brick,
the report reading the FUSED faces, and the maintainer's click-test
recipe acting itself:

  * ``CAD_0.brep.bool_union`` exists and answers the three door
    cases: P4 the grid-straddle (8/12/6, the honest box
    [-1,1.5]x[-1,1]x[-1,1] where yesterday's caller met the
    RuntimeError refusal), P8 the diagonal stair (20/32/14), P11 the
    flush-interior containment (8/12/6, A whole);
  * the report reads the fused faces: ``CAD_0.report``'s face table
    renders one row per face of the sewn body, the data's counts
    agreeing with the body's own;
  * the input gate's refusal KIND still honest from Python: a
    curved-surface operand refuses with ValueError, never silently.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import report as report_mod

Vec3f = CAD_0.math.Vec3f


def _placed_box(half, shift):
    """The factory box, translated in place (the C++ pinned suites'
    own vertex-shift pattern, through the exposed translate)."""
    body = brep_mod.make_box(Vec3f(*half))
    brep_mod.translate_body(body, Vec3f(*shift))
    return body


def _counts(body):
    q = brep_mod.body_query(body)
    return q["V"], q["E"], q["F"], q["chi"], q["orientable"], q["loops_closed"]


def test_the_door_is_reachable_from_python():
    """The three door cases answer — the wall's refusal is gone."""
    a = brep_mod.make_box(Vec3f(1, 1, 1))

    # P4: A = [-1,1]^3, B = [-0.5,1.5]x[-1,1]x[-1,1] — the aligned
    # coplanar ±Y/±Z pairs CROSS; yesterday this raised
    # result_edge_not_manifold ("8 of 24") from this very call.
    u = brep_mod.bool_union(a, _placed_box((1, 1, 1), (0.5, 0.0, 0.0)))
    v, e, f, chi, orientable, loops = _counts(u)
    assert (v, e, f, chi) == (8, 12, 6, 2)
    assert orientable and loops

    # P8: B offset (0.5, 0, 0.5) — the diagonal, the stair prism.
    u8 = brep_mod.bool_union(a, _placed_box((1, 1, 1), (0.5, 0.0, 0.5)))
    v8, e8, f8, chi8, orientable8, loops8 = _counts(u8)
    assert (v8, e8, f8, chi8) == (20, 32, 14, 2)
    assert orientable8 and loops8

    # P11: B = [-0.5,0.5]x[-1,1]x[-0.5,0.5] strictly inside A, the
    # tops' section contained — the union answers A whole.
    u11 = brep_mod.bool_union(a, brep_mod.make_box(Vec3f(0.5, 1.0, 0.5)))
    v11, e11, f11, chi11, orientable11, loops11 = _counts(u11)
    assert (v11, e11, f11, chi11) == (8, 12, 6, 2)
    assert orientable11 and loops11


def test_the_report_reads_the_fused_faces():
    """The report's surfaces render the sewn body's own faces."""
    a = brep_mod.make_box(Vec3f(1, 1, 1))
    b = _placed_box((1, 1, 1), (0.5, 0.0, 0.0))
    u = brep_mod.bool_union(a, b)

    # The D7.5-style face table: one row per face, in address order.
    table = report_mod.face_table_text(u)
    lines = [ln for ln in table.splitlines() if ln.strip()]
    assert len(lines) == 1 + 6  # the header + the six fused faces

    # The body report's counts agree with the body's own — the
    # fused caps read as faces like any other (the overlay's
    # fragments are faces, one logical region per pair).
    data = report_mod.body_report_data(u)
    assert data["V"] == 8
    assert data["E"] == 12
    assert data["F"] == 6
    assert data["chi"] == 2
    assert data["orientable"]
    assert data["loops_closed"]

    # THE CLICK-TEST, acted here: the honest box [-1,1.5]x[-1,1]x
    # [-1,1] where yesterday's call raised the refusal — its
    # census read from the report (8/12/6, χ=2, closed). The MASS
    # surface refuses this body class honestly (the polygon path's
    # faces carry no p-curves — the exact mass invariant's own
    # wall, the pre-existing boundary the stacked union meets too):
    # the refusal IS the report's answer, carried verbatim (the P2
    # pattern), never a disguised number.
    assert data["mass_error"] is not None
    assert "no p-curve" in data["mass_error"]
    assert data["volume"] is None
    assert data["all_exact"] is None


def test_the_input_gate_still_refuses_by_kind():
    """The door did not open the class: a curved-surface operand
    still refuses with ValueError (the input gate's own class)."""
    a = brep_mod.make_box(Vec3f(1, 1, 1))
    sphere = CAD_0.bridge.sdf_to_brep(CAD_0.sdf.sphere(2.0))
    with pytest.raises(ValueError) as excinfo:
        brep_mod.bool_union(a, sphere)
    assert "input_surface_not_plane" in str(excinfo.value)
