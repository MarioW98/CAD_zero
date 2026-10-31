"""H2 unit 6, morso 3 — the extrude's names through the bindings
(ADR-0036 D1/D6.2, the accepted act + the morse-3 register).

The C++ side pins the whole contract (tests/unit/brep/
test_h2_u6_m3_extrude_names.cpp — the exact names, the purity, the
round-trip, the depths, the tavola with its FROMs, the closures).
What THESE tests pin is what crosses the binding intact, the
maintainer's own click-test included:

  * ``CAD_0.brep.make_prism`` — the exact extrude, now a Python
    surface: the sketch chains (``CAD_0.scene``'s own values) become
    the prism body, and every face and the seam edge carry the
    WALK-ROLE NAME beside the address;
  * THE CLICK-TEST: the guide case's tool (the cylinder from the
    circle, ``feature=3`` — the DAG identity the binding carries)
    interrogated live answers ``ext#3:lat(0)`` and its caps with
    their names, plus the seam;
  * the multi-chain prism: ``c0:lat(0..3)``, the hole's
    ``c1:lat(0)`` / ``c1:seam``, the caps unprefixed;
  * the shape_id STRUCTURE crosses as plain values (the path of
    role-table indices; the depth — the parametric lat at 2
    single-chain, 3 multi-chain, the register's declaration);
  * the REPORT prints the name column (``CAD_0.report.
    face_table_text``); ``body_report_text`` does not move;
  * purity from Python: two extrusions of the same recipe, the
    identical table;
  * the honest empty: the walk's other edges read ``""`` — never a
    guess;
  * the refusals cross as ValueError, diagnosis + remedy verbatim.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import report as report_mod
from CAD_0 import scene as scene_mod


def _circle_chains():
    """The guide case's tool recipe: one solid circle chain."""
    return [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.5, 0.25), 1.25, False)])]


def _ring_chains():
    """The multi-chain recipe: the rectangle + a hole circle."""
    rect = scene_mod.SketchChain([
        scene_mod.sketch_segment((-2.0, -1.0), (2.0, -1.0), True),
        scene_mod.sketch_segment((2.0, -1.0), (2.0, 1.0), True),
        scene_mod.sketch_segment((2.0, 1.0), (-2.0, 1.0), True),
        scene_mod.sketch_segment((-2.0, 1.0), (-2.0, -1.0), True),
    ])
    hole = scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.2, True)])
    return [rect, hole]


def test_make_prism_is_a_surface_and_the_tool_answers_the_walk_roles():
    tool = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    assert brep_mod.face_count(tool) == 3
    assert brep_mod.edge_count(tool) == 5
    assert brep_mod.face_query(tool, 0)["name"] == "ext#3:lat(0)"
    assert brep_mod.face_query(tool, 1)["name"] == "ext#3:cap(lo)"
    assert brep_mod.face_query(tool, 2)["name"] == "ext#3:cap(hi)"
    assert brep_mod.edge_query(tool, 4)["name"] == "ext#3:seam"
    # The walk's other edges (the rim halves): honestly unnamed.
    for i in range(4):
        assert brep_mod.edge_query(tool, i)["name"] == ""


def test_the_click_test_the_live_tool_answers_lat0_and_its_caps():
    """The maintainer's click-test, pinned: the guide case's tool
    interrogated from the live extrude shows ext#3:lat(0) and its
    caps with their names — the identity the DAG carries, not a
    positional id."""
    tool = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    names = [brep_mod.face_query(tool, i)["name"] for i in range(3)]
    assert names == ["ext#3:lat(0)", "ext#3:cap(lo)", "ext#3:cap(hi)"]
    assert brep_mod.edge_query(tool, 4)["name"] == "ext#3:seam"


def test_the_multi_chain_prism_answers_the_grouped_roles():
    tube = brep_mod.make_prism(_ring_chains(), 1.5, +1, feature=5)
    assert brep_mod.face_count(tube) == 7
    expected = [
        "ext#5:c0:lat(0)", "ext#5:c0:lat(1)", "ext#5:c0:lat(2)",
        "ext#5:c0:lat(3)", "ext#5:c1:lat(0)",
        "ext#5:cap(lo)", "ext#5:cap(hi)",
    ]
    for i, name in enumerate(expected):
        assert brep_mod.face_query(tube, i)["name"] == name
    assert brep_mod.edge_query(tube, 14)["name"] == "ext#5:c1:seam"
    for i in range(14):
        assert brep_mod.edge_query(tube, i)["name"] == ""


def test_the_shape_id_crosses_as_plain_values():
    tool = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    wall = brep_mod.face_query(tool, 0)
    sid = wall["shape_id"]
    assert sid["feature"] == 3
    assert sid["path"] == [0, 0]  # [lat's table row, the chain entity]
    assert sid["generation"] == 0
    cap = brep_mod.face_query(tool, 1)
    assert cap["shape_id"]["path"] == [1]  # cap(lo)'s table row
    seam = brep_mod.edge_query(tool, 4)
    assert seam["shape_id"]["path"] == [0]  # the seam's table row

    # The register's depth declaration, where it lands: the lat at 2
    # (single-chain), 3 (multi-chain — the extrusion case's maximum
    # v1 depth), the caps and the seam shallower.
    tube = brep_mod.make_prism(_ring_chains(), 1.5, +1, feature=5)
    assert len(brep_mod.face_query(tool, 0)["shape_id"]["path"]) == 2
    for i in range(5):
        q = brep_mod.face_query(tube, i)
        assert len(q["shape_id"]["path"]) == 3
    assert len(brep_mod.face_query(tube, 5)["shape_id"]["path"]) == 1
    assert len(brep_mod.edge_query(tube, 14)["shape_id"]["path"]) == 2


def test_the_report_prints_the_name_column():
    tool = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    table = report_mod.face_table_text(tool)
    lines = table.splitlines()
    assert lines[0] == "Faces (address | name | family | loops | provenance):"
    assert len(lines) == 1 + 3
    assert lines[1] == "  0 | ext#3:lat(0) | cylinder | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[2] == "  1 | ext#3:cap(lo) | plane | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[3] == "  2 | ext#3:cap(hi) | plane | 1 loop(s) (1 outer, 0 inner) | exact"
    # The existing report does not move: body_report_text keeps its
    # own shape (the name column is a NEW surface beside it).
    text = report_mod.body_report_text(tool)
    assert "V = 4, E = 5, F = 3" in text
    assert "ext#3:" not in text


def test_purity_two_extrusions_one_answer():
    a = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    b = brep_mod.make_prism(_circle_chains(), 3.0, +1, feature=3)
    names_a = [brep_mod.face_query(a, i)["name"] for i in range(3)]
    names_b = [brep_mod.face_query(b, i)["name"] for i in range(3)]
    assert names_a == names_b
    assert report_mod.face_table_text(a) == report_mod.face_table_text(b)


def test_the_refusals_cross_as_value_errors():
    # The slab contract's own refusal, verbatim through the binding.
    with pytest.raises(ValueError) as exc_info:
        brep_mod.make_prism(_circle_chains(), 0.0, +1)
    assert "distance" in str(exc_info.value)
    assert "Remedy" in str(exc_info.value)
    with pytest.raises(ValueError):
        brep_mod.make_prism(_circle_chains(), 1.0, 0)
