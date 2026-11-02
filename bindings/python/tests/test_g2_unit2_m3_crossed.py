"""G2 unit 2, morso 3 — the crossed corner through the bindings
(ADR-0038 D1.2/D2/D3/D5, the accepted act + the m3 outcome note).

The C++ side pins the whole contract (tests/unit/brep/
test_g2_unit2_m3_crossed.cpp — the golden 12/18/8/H=0 with the
derivation reproduced, the eight faces' re-authored loops, the D3
names, the 64-row tavola with the closure green, the digest and the
N=10 determinism, the counterproof and the borders). What THESE
tests pin is what crosses the binding intact:

  * THE NOTCH BUILT FROM PYTHON: box#1 -> ext#3 -> sub#4, the
    square tool at the (+X,+Y) corner of the top face — the same
    call the CROSSED pin refused yesterday answers today;
  * THE REPORT: ``edge_table_text`` prints the notch rims
    (``sub#4:isect[box#1:top × ext#3:lat(0)]@hi`` and its
    siblings), the exit edges (the UNLEVELLED isect — the level
    absent, the pair's planes ARE the line) and the walls'
    inheritance; ``face_table_text`` prints the absorbed faces'
    own names and the re-windowed laterals;
  * THE joint(3) INTERROGATED FROM THE LIVE: the structural
    by-name query (ShapeId equality, never a parsed string) — the
    creating feature ext#3 and the path [kExtJointRole, 3] find
    the re-timed tool's joint vertical, the vocabulary row's first
    instance;
  * the honest boundary: the unnamed mode leaves the born rims and
    exits empty, the joint's birth name and the walls' inheritance
    flowing regardless;
  * purity: two materializations, identical tables.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import report as report_mod
from CAD_0 import scene as scene_mod

Vec3f = CAD_0.math.Vec3f

RIM_HI_P = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"
RIM_HI_Q = "sub#4:isect[box#1:top × ext#3:lat(3)]@hi"
RIM_LO_P = "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo"
RIM_LO_Q = "sub#4:isect[box#1:bottom × ext#3:lat(3)]@lo"
EXIT_P = "sub#4:isect[box#1:right × ext#3:lat(0)]"
EXIT_Q = "sub#4:isect[box#1:back × ext#3:lat(3)]"
JOINT = "ext#3:joint(3)"


def _corner_chains():
    """The guide case's tool recipe: the square [0.5, 1.5]^2, the
    CCW walk (entity 0 = the y=0.5 edge east, 3 = the x=0.5 edge
    south — the tool's joints J_0..J_3 with J_3 = (0.5, 0.5))."""
    return [scene_mod.SketchChain([
        scene_mod.sketch_segment((0.5, 0.5), (1.5, 0.5), True),
        scene_mod.sketch_segment((1.5, 0.5), (1.5, 1.5), True),
        scene_mod.sketch_segment((1.5, 1.5), (0.5, 1.5), True),
        scene_mod.sketch_segment((0.5, 1.5), (0.5, 0.5), True),
    ])]


def _guide_case(named=True):
    """The D1 chain: box#1 -> ext#3 -> sub#4, the tool translated
    to z in [-1.5, 1.5] (a strict through-span over the box)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = _corner_chains()
    tool = brep_mod.make_prism(chains, 3.0, +1, feature=3)
    if named:
        result = brep_mod.exact_subtract(
            box, tool, Vec3f(0.0, 0.0, -1.5),
            feature=4, target=brep_mod.FeatureKind.Box,
            tool=brep_mod.FeatureKind.Extrude)
    else:
        result = brep_mod.exact_subtract(box, tool, Vec3f(0.0, 0.0, -1.5))
    return box, chains, tool, result


def test_the_notch_answers_the_golden_from_python():
    box, chains, tool, result = _guide_case()
    # THE GOLDEN (the C++ derivation's own counts): 12/18/8, H = 0,
    # chi = 2 — the dent family; the volume abc - (the corner
    # column) = 8 - 0.5 = 7.5.
    summary = brep_mod.body_query(result)
    assert summary["V"] == 12
    assert summary["E"] == 18
    assert summary["F"] == 8
    assert summary["H"] == 0
    assert summary["chi"] == 2
    assert summary["orientable"]
    mp = brep_mod.mass_properties(result)
    assert abs(mp["volume"] - 7.5) < 1e-9
    assert mp["all_exact"]


def test_the_report_prints_the_notch_names():
    box, chains, tool, result = _guide_case()
    faces = report_mod.face_table_text(result)
    lines = faces.splitlines()
    assert lines[0] == "Faces (address | name | family | loops | provenance):"
    assert len(lines) == 1 + 8
    # The absorbed faces keep the factory's names; the walls inherit
    # the tool's own laterals (the re-windowed ones).
    assert lines[1] == "  0 | box#1:bottom | plane | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[2] == "  1 | box#1:top | plane | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[7] == "  6 | ext#3:lat(0) | plane | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[8] == "  7 | ext#3:lat(3) | plane | 1 loop(s) (1 outer, 0 inner) | exact"

    edges = report_mod.edge_table_text(result)
    elines = edges.splitlines()
    assert elines[0] == "Edges (address | name | endpoints | manifold | provenance):"
    assert len(elines) == 1 + 18
    # The notch's born names live HERE — the rims (levelled), the
    # exits (UNLEVELLED: the pair's planes ARE the line) and the
    # joint's birth name (the vocabulary row's first instance).
    assert elines[13] == "  12 | " + RIM_LO_P + " | 8 - 6 | yes | exact"
    assert elines[14] == "  13 | " + RIM_LO_Q + " | 10 - 8 | yes | exact"
    assert elines[15] == "  14 | " + RIM_HI_P + " | 7 - 9 | yes | exact"
    assert elines[16] == "  15 | " + RIM_HI_Q + " | 9 - 11 | yes | exact"
    assert elines[17] == "  16 | " + EXIT_P + " | 6 - 7 | yes | exact"
    assert elines[18] == "  17 | " + EXIT_Q + " | 10 - 11 | yes | exact"
    assert elines[12] == "  11 | " + JOINT + " | 8 - 9 | yes | exact"
    # The box's own names carried (the survivors; the dead column's
    # vertical2 is NOWHERE).
    assert elines[1] == "  0 | box#1:bottom0 | 0 - 1 | yes | exact"
    assert "vertical2" not in edges


def test_the_joint_interrogated_from_the_live():
    """The vocabulary row's first instance, queried structurally —
    the resolver's own read (ShapeId equality, never the render):
    the creating feature ext#3 and the path [joint, 3]."""
    box, chains, tool, result = _guide_case()
    found = brep_mod.query_edge_by_name(
        result, CAD_0.geometry.ShapeId(
            CAD_0.geometry.FeatureId(3), [1, 3]))
    assert len(found) == 1
    assert found[0] == 11
    # And by the rendered name, for the report's own eye.
    assert brep_mod.edge_query(result, 11)["name"] == JOINT
    # The walls' inheritance flows the same way.
    face = brep_mod.query_face_by_name(
        result, CAD_0.geometry.ShapeId(
            CAD_0.geometry.FeatureId(3), [0, 0]))
    assert len(face) == 1
    assert face[0] == 6


def test_the_unnamed_mode_honest_boundaries():
    box, chains, tool, result = _guide_case(named=False)
    # The born rims/exits read honestly EMPTY (no sub context
    # involved); the joint's birth name and the walls' inheritance
    # flow regardless (stamp copies and the extrude's own table).
    for i in (12, 13, 14, 15, 16, 17):
        assert brep_mod.edge_query(result, i)["name"] == ""
    assert brep_mod.edge_query(result, 11)["name"] == JOINT
    assert brep_mod.face_query(result, 6)["name"] == "ext#3:lat(0)"
    assert brep_mod.face_query(result, 7)["name"] == "ext#3:lat(3)"
    # The address plane untouched by the naming (the m4 pin).
    assert brep_mod.face_count(result) == 8
    assert brep_mod.edge_count(result) == 18


def test_the_still_refused_families_keep_their_words():
    """The borders: the mid-edge crossing (the pin's own circle),
    the tangential byte-identical, the blind wall alive — the door
    opened exactly where the derivation admits it."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0))

    # The mid-edge crossing: the pin :952's own circle.
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.9, 0.0), 0.35, False)])]
    tool = brep_mod.make_prism(chains, 3.0, +1)
    try:
        brep_mod.exact_subtract(box, tool, Vec3f(0.0, 0.0, -1.5))
        raised = False
        msg = ""
    except ValueError as e:
        raised = True
        msg = str(e)
    assert raised
    assert "CROSSES" in msg
    assert "Remedy" in msg

    # The blind-crossed composite: the notch footprint with the
    # blind span — the queued family, its own honest name.
    tool2 = brep_mod.make_prism(_corner_chains(), 2.0, +1)
    try:
        brep_mod.exact_subtract(box, tool2, Vec3f(0.0, 0.0, -0.5))
        raised = False
        msg = ""
    except ValueError as e:
        raised = True
        msg = str(e)
    assert raised
    assert "COMPOSITE" in msg
    assert "Remedy" in msg


def test_purity_two_materializations_one_answer():
    a = _guide_case()
    b = _guide_case()
    assert report_mod.face_table_text(a[3]) == report_mod.face_table_text(b[3])
    assert report_mod.edge_table_text(a[3]) == report_mod.edge_table_text(b[3])
    assert report_mod.edge_table_text(a[2]) == report_mod.edge_table_text(b[2])
