"""SDF → B-Rep bridge (ADR-0026) — Python binding tests.

Covers the BINDING layer only; the C++ conversion contract (pinned
counts, T1-T11) lives in tests/unit/bridge/test_bridge.cpp. What these
tests pin is what the binding is responsible for:

  * the report's grid-INDEPENDENT invariants cross the binding intact:
    chi == 2, closed (is_closed_manifold), loops (loops_are_closed) —
    on the primitive path (sphere/box/cylinder fit) and on the faceted
    path (fit_primitives=False), including an off-grid translated
    sphere (ADR-0026 input contract P2 neighborhood);
  * min_abs_field — the machine-readable P2 margin (ADR-0026
    Consequences left-over, resolved as a report field): frozen for
    the pinned sphere grid and path-independent;
  * the faceted numbers are structurally coherent (closed-triangle-mesh
    identities E = 3F/2, V = F/2 + 2) — the counts themselves are the
    C++ suite's contract (ADR-0026 D2: V/E/F depend on the grid);
  * an empty SDFBody raises the bridge's std::invalid_argument AS a
    ValueError carrying the ORIGINAL message — function-name prefix,
    diagnosis and remedy all intact, no generic wrapping;
  * sdf_to_brep_faceted returns an opaque, Python-owned BRepBody handle
    that can simply be dropped.

Requires only the compiled extension (no numpy, no PySide6) so it runs
in the CI pytest step like the other binding tests.
"""

from __future__ import annotations

import gc

import pytest

import CAD_0
from CAD_0 import bridge as bridge_mod
from CAD_0 import sdf as sdf_mod

Vec3f = CAD_0.math.Vec3f


def _faceted_opts() -> bridge_mod.SdfToBrepOptions:
    """Options forcing the faceted path at a small resolution."""
    opts = bridge_mod.SdfToBrepOptions()
    opts.march_resolution = 16
    opts.fit_primitives = False
    opts.compute_normals = True
    return opts


def test_options_fields_default_and_settable():
    opts = bridge_mod.SdfToBrepOptions()
    assert opts.march_resolution == 48
    assert opts.fit_primitives is True
    assert opts.compute_normals is True
    opts.march_resolution = 16
    opts.fit_primitives = False
    opts.compute_normals = False
    assert opts.march_resolution == 16
    assert opts.fit_primitives is False
    assert opts.compute_normals is False


def test_report_sphere_primitive_path():
    rep = bridge_mod.sdf_to_brep_report(sdf_mod.sphere(1.0))
    assert rep["fit"] == "sphere"
    assert rep["chi"] == 2
    assert rep["closed"] is True
    assert rep["loops"] is True
    assert rep["V"] > 0 and rep["E"] > 0 and rep["F"] > 0


def test_report_box_and_cylinder_primitive_paths():
    rep_box = bridge_mod.sdf_to_brep_report(sdf_mod.box(Vec3f(1.0, 2.0, 3.0)))
    assert rep_box["fit"] == "box"
    assert rep_box["chi"] == 2
    assert rep_box["closed"] is True
    assert rep_box["loops"] is True

    rep_cyl = bridge_mod.sdf_to_brep_report(sdf_mod.cylinder(0.5, 2.0))
    assert rep_cyl["fit"] == "cylinder"
    assert rep_cyl["chi"] == 2
    assert rep_cyl["closed"] is True
    assert rep_cyl["loops"] is True


def test_report_sphere_faceted_path():
    rep = bridge_mod.sdf_to_brep_report(sdf_mod.sphere(1.0), _faceted_opts())
    assert rep["fit"] == "faceted"
    assert rep["chi"] == 2
    assert rep["closed"] is True
    assert rep["loops"] is True
    # Closed-triangle-mesh identities (structural coherence, not pinned
    # counts — those are the C++ suite's contract, ADR-0026 D5):
    assert rep["E"] == 3 * rep["F"] // 2
    assert rep["V"] == rep["F"] // 2 + 2


def test_report_sphere_off_grid_same_invariants():
    # A sphere centered off any sampling-grid multiple (ADR-0026 P2
    # neighborhood): the invariants must not care.
    off = sdf_mod.translate(sdf_mod.sphere(1.0), Vec3f(0.13, 0.29, 0.07))
    rep = bridge_mod.sdf_to_brep_report(off, _faceted_opts())
    assert rep["fit"] == "faceted"
    assert rep["chi"] == 2
    assert rep["closed"] is True
    assert rep["loops"] is True
    assert rep["E"] == 3 * rep["F"] // 2
    assert rep["V"] == rep["F"] // 2 + 2


def test_empty_input_original_exception_message():
    empty = sdf_mod.SDFBody()
    with pytest.raises(ValueError) as excinfo:
        bridge_mod.sdf_to_brep_report(empty)
    msg = str(excinfo.value)
    # The bridge's std::invalid_argument must cross the binding
    # UNWRAPPED: the C++ function-name prefix, the diagnosis and the
    # remedy are all part of the message contract (ADR-0026 acceptance
    # correction 2). A generic wrap would erase exactly these.
    assert msg.startswith("sdf_to_brep_faceted:")
    assert "nothing to convert" in msg
    assert "Give the SDF a non-empty interior" in msg

    with pytest.raises(ValueError):
        bridge_mod.sdf_to_brep_faceted(empty)


def test_faceted_returns_opaque_python_owned_handle():
    body = bridge_mod.sdf_to_brep_faceted(sdf_mod.sphere(1.0), _faceted_opts())
    assert "BRepBody" in repr(body)
    # Python owns the fresh body exclusively: dropping the last
    # reference destroys it, nothing dangles, nothing leaks.
    del body
    gc.collect()


def test_report_min_abs_field_frozen_p2_margin():
    # The ADR-0026 Consequences left-over ("machine-readable min |f|"),
    # resolved as a report field: the P2 margin of the pinned sphere
    # (r=1, res=16 — the T1 grid), measured once and frozen. The
    # literal below is the exact float32 value widened to float64 (the
    # binding converts C++ float -> Python float); determinism is the
    # ADR-0026 D5 arithmetic class (the build enforces
    # -ffp-contract=off). Path-independent: the primitive path reports
    # the same margin at the same resolution.
    rep = bridge_mod.sdf_to_brep_report(sdf_mod.sphere(1.0), _faceted_opts())
    assert rep["min_abs_field"] > 0.0
    assert rep["min_abs_field"] == 0.0039359331130981445

    prim = bridge_mod.SdfToBrepOptions()
    prim.march_resolution = 16
    prim.fit_primitives = True
    rep_prim = bridge_mod.sdf_to_brep_report(sdf_mod.sphere(1.0), prim)
    assert rep_prim["fit"] == "sphere"
    assert rep_prim["min_abs_field"] == rep["min_abs_field"]


def test_report_via_scene_node_body():
    # The exact call the app's Debug action makes: a scene node's SDF
    # body (reference into the C++ scene) feeding the report.
    scene = CAD_0.scene.Scene()
    nid = scene.add_node("sphere", sdf_mod.sphere(1.0))
    node = scene.get_node(nid)
    rep = bridge_mod.sdf_to_brep_report(node.body)
    assert rep["fit"] == "sphere"
    assert rep["chi"] == 2
    assert rep["closed"] is True
    assert rep["loops"] is True
