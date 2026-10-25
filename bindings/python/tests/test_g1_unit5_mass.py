"""G1 unit 5 — the exact mass properties through the bindings (ADR-0032 D4).

The C++ side pins the machinery and the classical-formula goldens
(tests/unit/brep/test_g1_unit5_mass.cpp). What THESE tests pin is the
contract as the final consumer reads it — the numbers, readable from
Python, end-to-end through the scene:

  * the wiring: ``CAD_0.brep.mass_properties`` exists (today it does
    not — the AttributeError discriminator of D9);
  * THE GUIDE CASE: the v0.4 flow (sketch -> extrude -> subtract)
    whose exact B-Rep answers with the hand-checkable table —
    V = abc - pi r^2 h, the centroid on the symmetry axis,
    I_zz = abc(a^2+b^2)/12 - pi r^4 h/2 (density 1);
  * the honesty: the same node's SDF through the faceted bridge says
    the polyhedral value with ``all_exact=False``; the exact side
    says the closed form with ``all_exact=True`` — never mixed;
  * determinism (ADR-0014): re-runs are bit-identical.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import scene

V3 = CAD_0.math.Vec3f

PI = 3.141592653589793


# The precision doctrine (the C++ suite's own): the machinery's
# arithmetic is closed-form and double, and the INPUTS are the stored
# trimming parameters — the p-curves' floats (the float-pi sweeps,
# the projected frames). The honest comparison bound is a few float
# ulps, 1e-6 relative — still five orders tighter than any faceted
# extract.
def _close(a: float, b: float, rel: float = 1e-6, abs_tol: float = 1e-6) -> bool:
    return abs(a - b) <= max(abs_tol, rel * max(abs(a), abs(b)))


def _guide_scene():
    """The guide case: the box [-1,1]^3 minus the circle prism r=0.25
    through Z (the v0.4 golden's own flow)."""
    s = scene.Scene()
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node(
        "S", [scene.SketchChain([scene.sketch_circle((0.0, 0.0), 0.25, False)])])
    eid = s.add_extrude_node("Extrude", sid, 3.0, 1)
    did = s.add_operation_node("Sub", box, eid, 0.0, 0.0, -1.5)
    return s, did


def test_the_api_is_exposed():
    # The wiring discriminator (D9): absent today, present after the
    # unit lands — the AttributeError the order names.
    assert hasattr(brep_mod, "mass_properties")


def test_guide_case_numbers_end_to_end():
    s, did = _guide_scene()
    n = s.get_node(did)
    assert n.exact_brep is not None

    mp = brep_mod.mass_properties(n.exact_brep)

    # D7.5's table, hand-checkable: abc=8, r=0.25, h=2.
    assert mp["all_exact"] is True
    assert _close(mp["volume"], 8.0 - PI * 0.0625 * 2.0)
    cx, cy, cz = mp["centroid"]
    assert _close(cx, 0.0, 1e-6, 1e-9)
    assert _close(cy, 0.0, 1e-6, 1e-9)
    assert _close(cz, 0.0, 1e-6, 1e-9)
    ixx = mp["inertia_about_centroid"][0][0]
    izz = mp["inertia_about_centroid"][2][2]
    assert _close(izz, 8.0 * 8.0 / 12.0 - PI * 0.25**4 * 2.0 / 2.0, 1e-5)
    assert _close(ixx, 8.0 * 8.0 / 12.0 - PI * 0.0625 * 2.0 * (3 * 0.0625 + 4) / 12.0, 1e-5)
    # The symmetry: the products of inertia vanish on the symmetric body.
    ixy = mp["inertia_about_centroid"][0][1]
    assert _close(ixy, 0.0, 1e-6, 1e-9)


def test_honesty_faceted_vs_exact():
    s, did = _guide_scene()
    n = s.get_node(did)
    assert n.exact_brep is not None
    mp_exact = brep_mod.mass_properties(n.exact_brep)
    assert mp_exact["all_exact"] is True

    opts = bridge_mod.SdfToBrepOptions()
    opts.fit_primitives = False
    opts.march_resolution = 24
    faceted = bridge_mod.sdf_to_brep_faceted(n.body, opts)
    assert faceted is not None
    mp_faceted = brep_mod.mass_properties(faceted)
    assert mp_faceted["all_exact"] is False
    assert _close(mp_faceted["volume"], 8.0 - PI * 0.0625 * 2.0, 2e-2)


def test_determinism_bit_identical():
    s, did = _guide_scene()
    n = s.get_node(did)
    assert n.exact_brep is not None
    mp1 = brep_mod.mass_properties(n.exact_brep)
    mp2 = brep_mod.mass_properties(n.exact_brep)
    assert repr(mp1) == repr(mp2)
    assert mp1["volume"] == mp2["volume"]

    # A fresh scene, the same recipe: the same numbers bit-identically.
    s2, did2 = _guide_scene()
    n2 = s2.get_node(did2)
    mp3 = brep_mod.mass_properties(n2.exact_brep)
    assert mp3["volume"] == mp1["volume"]
    assert mp3["centroid"] == mp1["centroid"]
    assert mp3["inertia_about_centroid"] == mp1["inertia_about_centroid"]
