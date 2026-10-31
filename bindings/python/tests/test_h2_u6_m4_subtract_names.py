"""H2 unit 6, morso 4 — the subtract named through the bindings
(ADR-0036 D6.3, the accepted act + the morse-4 register).

The C++ side pins the whole contract (tests/unit/brep/
test_h2_u6_m4_subtract_names.cpp — the D1 table's eight rows, the
47-row tavola, the two-sided closure with its counterproofs, the
round-trip, the purity, the address plan, the Layer-2 bill). What
THESE tests pin is what crosses the binding intact, the
maintainer's own click-test included:

  * ``CAD_0.brep.exact_subtract`` gains the NAMING CONTEXT
    (``feature`` + the two operands' creating ``FeatureKind``s):
    with it the born edges carry the RELATIONAL names —
    ``sub#4:isect[box#1:top × ext#3:lat(0)]@hi``, the fillet's own
    string; without it (the default) they read honestly empty, the
    wall's inheritance flowing in both modes;
  * THE CLICK-TEST: the guide case's hole interrogated from the
    live answers ``box#1:top`` with its inner loop (absorbed, NOT
    renamed), and the rim answers ``sub#4:isect[...]@hi`` — both
    materialized halves under the ONE logical name (ADR-0023: the
    pair is the name's materialized form, not two names);
  * ``shape_id`` crosses as plain values: the rim's path
    ``[1, 1, 0, 0]`` — the isect entry, the box's top row, ext's
    lat row, the chain entity — depth 4, the derived maximum, and
    the seam's ``[2, 0, 0]`` at depth 3;
  * THE REPORT: ``face_table_text`` (the D6.2 column, replicated)
    and the NEW ``edge_table_text`` (the D7.5 table's edge leg —
    where the rim's fillet name lives) both print the guide case's
    chain verbatim, deterministically;
  * the honest boundaries: the unnamed mode's empties, the
    half-context refusal, the no-table kind refusal — loud, never
    a guess;
  * purity from Python: two materializations of the whole chain
    render the identical tables.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import report as report_mod
from CAD_0 import scene as scene_mod

Vec3f = CAD_0.math.Vec3f

RIM_HI = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"
RIM_LO = "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo"
SEAM = "sub#4:seam[ext#3:lat(0)]"


def _guide_chains():
    """The guide case's tool recipe (the G1 unit-4 class): the solid
    circle at the origin, r = 0.25, strictly interior to the box."""
    return [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.25, False)])]


def _guide_case(named=True):
    """The D1 chain through the bindings: box#1 -> sk#2 (the recipe)
    -> ext#3 -> sub#4. `named` selects the naming context."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = _guide_chains()
    tool = brep_mod.make_prism(chains, 3.0, +1, feature=3)
    if named:
        result = brep_mod.exact_subtract(
            box, tool, Vec3f(0.0, 0.0, -1.5),
            feature=4, target=brep_mod.FeatureKind.Box,
            tool=brep_mod.FeatureKind.Extrude)
    else:
        result = brep_mod.exact_subtract(box, tool, Vec3f(0.0, 0.0, -1.5))
    return box, chains, tool, result


def test_the_guide_case_answers_the_d1_names():
    box, chains, tool, result = _guide_case()
    assert brep_mod.face_count(result) == 7
    assert brep_mod.edge_count(result) == 17
    # The absorbed, NOT renamed; the kept; the wall's inheritance.
    assert brep_mod.face_query(result, 0)["name"] == "box#1:bottom"
    assert brep_mod.face_query(result, 1)["name"] == "box#1:top"
    for i, token in ((2, "front"), (3, "back"), (4, "left"), (5, "right")):
        assert brep_mod.face_query(result, i)["name"] == "box#1:" + token
    assert brep_mod.face_query(result, 6)["name"] == "ext#3:lat(0)"
    # The born edges: the rim circles' halves (ONE logical name each,
    # both halves answering to it) and the seam.
    assert brep_mod.edge_query(result, 12)["name"] == RIM_LO
    assert brep_mod.edge_query(result, 13)["name"] == RIM_LO
    assert brep_mod.edge_query(result, 14)["name"] == RIM_HI
    assert brep_mod.edge_query(result, 15)["name"] == RIM_HI
    assert brep_mod.edge_query(result, 16)["name"] == SEAM


def test_the_click_test_the_live_hole_and_the_live_rim():
    """The maintainer's click-test, pinned: the hole's face
    interrogated from the live answers 'box#1:top' with its inner
    loop, and the rim answers the fillet's string."""
    box, chains, tool, result = _guide_case()
    hole = brep_mod.face_query(result, 1)
    assert hole["name"] == "box#1:top"
    assert hole["loop_count"] == 2
    assert hole["inner_loop_count"] == 1
    assert hole["family"] == "plane"
    rim = brep_mod.edge_query(result, 14)
    assert rim["name"] == RIM_HI
    assert rim["manifold"]
    # The other half of the SAME rim answers the SAME name — the
    # seam-wired pair is the materialized form (ADR-0023).
    assert brep_mod.edge_query(result, 15)["name"] == rim["name"]


def test_the_shape_id_crosses_as_plain_values():
    box, chains, tool, result = _guide_case()
    rim = brep_mod.edge_query(result, 14)
    sid = rim["shape_id"]
    assert sid["feature"] == 4
    # [the isect@hi row, the box's "top" row, ext's lat row, the
    # chain entity] — depth 4, the derived maximum (kMaxDepth = 8).
    assert sid["path"] == [1, 1, 0, 0]
    assert sid["generation"] == 0
    seam = brep_mod.edge_query(result, 16)
    assert seam["shape_id"]["path"] == [2, 0, 0]
    # The wall's inherited stamp IS the operand's own.
    wall = brep_mod.face_query(result, 6)
    operand = brep_mod.face_query(tool, 0)
    assert wall["shape_id"]["path"] == operand["shape_id"]["path"]
    assert wall["shape_id"]["feature"] == 3


def test_the_unnamed_mode_is_honest():
    box, chains, tool, result = _guide_case(named=False)
    for i in range(12, 17):
        assert brep_mod.edge_query(result, i)["name"] == ""
    # The inheritance flows regardless — a stamp copy, no table.
    assert brep_mod.face_query(result, 6)["name"] == "ext#3:lat(0)"
    assert brep_mod.face_query(result, 1)["name"] == "box#1:top"


def test_the_half_context_and_the_tableless_kinds_refuse():
    box, chains, tool, result = _guide_case()
    with pytest.raises(ValueError) as exc_info:
        brep_mod.exact_subtract(
            box, tool, Vec3f(0.0, 0.0, -1.5),
            target=brep_mod.FeatureKind.Box)
    assert "complete or absent" in str(exc_info.value)
    with pytest.raises(ValueError) as exc_info:
        brep_mod.exact_subtract(
            box, tool, Vec3f(0.0, 0.0, -1.5), feature=4,
            target=brep_mod.FeatureKind.Sketch,
            tool=brep_mod.FeatureKind.Extrude)
    assert "no naming table" in str(exc_info.value)


def test_the_class_checks_still_refuse_verbatim():
    """The kernel's own refusals cross as ValueErrors, diagnosis and
    remedy verbatim (the blind hole — the tool ends inside)."""
    box, chains, tool, result = _guide_case()
    blind = brep_mod.make_prism(_guide_chains(), 1.6, +1, feature=3)
    with pytest.raises(ValueError) as exc_info:
        brep_mod.exact_subtract(
            box, blind, Vec3f(0.0, 0.0, -1.2),
            feature=4, target=brep_mod.FeatureKind.Box,
            tool=brep_mod.FeatureKind.Extrude)
    assert "BLIND" in str(exc_info.value)
    assert "Remedy" in str(exc_info.value)


def test_the_report_prints_the_column_both_legs():
    box, chains, tool, result = _guide_case()
    faces = report_mod.face_table_text(result)
    lines = faces.splitlines()
    assert lines[0] == "Faces (address | name | family | loops | provenance):"
    assert len(lines) == 1 + 7
    assert lines[1] == "  0 | box#1:bottom | plane | 2 loop(s) (1 outer, 1 inner) | exact"
    assert lines[2] == "  1 | box#1:top | plane | 2 loop(s) (1 outer, 1 inner) | exact"
    assert lines[7] == "  6 | ext#3:lat(0) | cylinder | 1 loop(s) (1 outer, 0 inner) | exact"

    edges = report_mod.edge_table_text(result)
    elines = edges.splitlines()
    assert elines[0] == "Edges (address | name | endpoints | manifold | provenance):"
    assert len(elines) == 1 + 17
    # The fillet's strings live HERE — the rim and the seam, the
    # maintainer's click-test on the edge leg.
    assert elines[13] == "  12 | " + RIM_LO + " | 10 - 8 | yes | exact"
    assert elines[15] == "  14 | " + RIM_HI + " | 11 - 9 | yes | exact"
    assert elines[16] == "  15 | " + RIM_HI + " | 9 - 11 | yes | exact"
    assert elines[17] == "  16 | " + SEAM + " | 8 - 9 | yes | exact"
    # The extrude's walk edges read honestly unnamed (the dash).
    assert elines[1] == "  0 | box#1:bottom0 | 0 - 1 | yes | exact"


def test_purity_two_materializations_one_answer():
    a = _guide_case()
    b = _guide_case()
    faces_a = report_mod.face_table_text(a[3])
    faces_b = report_mod.face_table_text(b[3])
    assert faces_a == faces_b
    assert report_mod.edge_table_text(a[3]) == report_mod.edge_table_text(b[3])
    # And the digests agree: the whole chain, both legs.
    assert report_mod.face_table_text(a[0]) == report_mod.face_table_text(b[0])
    assert report_mod.edge_table_text(a[2]) == report_mod.edge_table_text(b[2])
