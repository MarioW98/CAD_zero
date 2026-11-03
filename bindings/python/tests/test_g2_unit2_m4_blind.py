"""G2 unit 2, morso 4 (the unit's finale) — the blind hole through
the bindings (ADR-0038 D1.1/D2/D3/D5 + the m4 decoupling rule).

The C++ side pins the whole contract (tests/unit/brep/
test_g2_unit2_m4_blind_door.cpp — the golden 12/17/8/H=1 with the
derivation reproduced, the decoupling's authoring pinned co-edge by
co-edge, the D3 names with the grammar's FIRST INTRA-BODY row, the
47-row tavola with the closure green, the digest and the N=10
determinism, the counterproof and the borders). What THESE tests
pin is what crosses the binding intact:

  * THE BLIND HOLE BUILT FROM PYTHON: box#1 -> ext#3 -> sub#4, the
    circle tool r = 0.5 with the blind span z in [-0.5, 1.5] — the
    same call the BLIND pin refused yesterday answers today, the
    counts and the volume the golden 8 - 0.375pi;
  * THE REPORT: ``edge_table_text`` prints the floor rim (the
    INTRA-BODY row — ``sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]``,
    the level absent: the cap's plane IS the level), the pierced rim
    and the seam in the G1 forms; ``face_table_text`` prints the
    absorbed cap's own name (ext#3:cap(lo) — identity is where a
    thing was born) beside the wall's inheritance;
  * THE CAP INTERROGATED WITH ITS OWN NAME: the structural by-name
    query (ShapeId equality, never a parsed string) — the creating
    feature ext#3 and the path [kExtCapLoRole] find the absorbed
    face alive in the vivo; the floor rim's logical ShapeId finds
    BOTH halves of the seam-wired pair;
  * THE CLICK-TEST: the maintainer's own gesture — the scene builds
    the chain (Box -> Sketch -> Extrude -> Sub, the blind span), the
    node's exact B-Rep materializes, and the BLIND'S VOLUME IS READ
    FROM THE LIVE (8 - 0.375pi, all exact);
  * the honest boundary: the unnamed mode leaves the born rims and
    the seam empty, the inheritances flowing regardless;
  * the still-refused families keep their words (the both-inside
    cavity's own name, the chain blind's own row);
  * purity: two materializations, identical tables.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import report as report_mod
from CAD_0 import scene as scene_mod

Vec3f = CAD_0.math.Vec3f

RIM_FLOOR = "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]"
RIM_TOP = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"
SEAM = "sub#4:seam[ext#3:lat(0)]"
CAP_LO = "ext#3:cap(lo)"
LAT_0 = "ext#3:lat(0)"

GOLDEN_VOLUME = 8.0 - 0.375 * 3.14159265358979


def _guide_case(named=True):
    """The D1 chain: box#1 -> ext#3 -> sub#4, the tool translated to
    z in [-0.5, 1.5] (the bottom cap INSIDE — the floor at -0.5, the
    pierced quota z_top = 1: the blind_lo guide case)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    tool = brep_mod.make_prism(chains, 2.0, +1, feature=3)
    if named:
        result = brep_mod.exact_subtract(
            box, tool, Vec3f(0.0, 0.0, -0.5),
            feature=4, target=brep_mod.FeatureKind.Box,
            tool=brep_mod.FeatureKind.Extrude)
    else:
        result = brep_mod.exact_subtract(box, tool, Vec3f(0.0, 0.0, -0.5))
    return box, chains, tool, result


def test_the_blind_hole_answers_the_golden_from_python():
    box, chains, tool, result = _guide_case()
    # THE GOLDEN (the C++ derivation's own counts): 12/17/8, H = 1,
    # chi = 2 — the dent family; the volume abc - pi r^2 (z_top -
    # z_cap) = 8 - 0.375pi — the DECOUPLING's own answer (the m2
    # wall's inverted read is the other authoring's fact, pinned in
    # the wall's own file, untouched).
    summary = brep_mod.body_query(result)
    assert summary["V"] == 12
    assert summary["E"] == 17
    assert summary["F"] == 8
    assert summary["H"] == 1
    assert summary["chi"] == 2
    assert summary["orientable"]
    mp = brep_mod.mass_properties(result)
    assert abs(mp["volume"] - GOLDEN_VOLUME) < 1e-9
    assert mp["all_exact"]
    # The centroid on the axis, the cavity's own z removed.
    assert abs(mp["centroid"][0]) < 1e-9
    assert abs(mp["centroid"][1]) < 1e-9


def test_the_report_prints_the_blind_names():
    box, chains, tool, result = _guide_case()
    faces = report_mod.face_table_text(result)
    lines = faces.splitlines()
    assert lines[0] == "Faces (address | name | family | loops | provenance):"
    assert len(lines) == 1 + 8
    # The pierced top absorbed (its inner loop); the box's own kept;
    # the wall and the CAP absorbed, the cap keeping its OWN name
    # (ext#3:cap(lo) — the carried face, alive in the vivo).
    assert lines[2] == "  1 | box#1:top | plane | 2 loop(s) (1 outer, 1 inner) | exact"
    assert lines[7] == "  6 | " + LAT_0 + " | cylinder | 1 loop(s) (1 outer, 0 inner) | exact"
    assert lines[8] == "  7 | " + CAP_LO + " | plane | 1 loop(s) (1 outer, 0 inner) | exact"

    edges = report_mod.edge_table_text(result)
    elines = edges.splitlines()
    assert elines[0] == "Edges (address | name | endpoints | manifold | provenance):"
    assert len(elines) == 1 + 17
    # THE FLOOR RIM — the grammar's FIRST INTRA-BODY row (both halves
    # the one logical name); the pierced rim and the seam in the G1
    # forms.
    # The endpoints in the walk table's own interleaved layout
    # (2i/2i+1 = J_i at the window's low/high end: 8 = S_c, 9 = S_t,
    # 10 = A_c, 11 = A_t).
    assert elines[13] == "  12 | " + RIM_FLOOR + " | 10 - 8 | yes | exact"
    assert elines[14] == "  13 | " + RIM_FLOOR + " | 8 - 10 | yes | exact"
    assert elines[15] == "  14 | " + RIM_TOP + " | 11 - 9 | yes | exact"
    assert elines[16] == "  15 | " + RIM_TOP + " | 9 - 11 | yes | exact"
    assert elines[17] == "  16 | " + SEAM + " | 8 - 9 | yes | exact"
    # The box's own names carried.
    assert elines[1] == "  0 | box#1:bottom0 | 0 - 1 | yes | exact"


def test_the_cap_interrogated_with_its_own_name():
    """The absorbed carried face, found structurally — the resolver's
    own read (ShapeId equality, never the render): the creating
    feature ext#3 and the path [kExtCapLoRole]; the floor rim's
    logical ShapeId (the intra-body row's path) finds BOTH halves."""
    box, chains, tool, result = _guide_case()
    # ext#3:cap(lo) — kExtCapLoRole = 1.
    face = brep_mod.query_face_by_name(
        result, CAD_0.geometry.ShapeId(
            CAD_0.geometry.FeatureId(3), [1]))
    assert len(face) == 1
    assert face[0] == 7
    # The floor rim: sub#4, [kSubIsectIntraRole, cap(lo), lat(0), 0].
    found = brep_mod.query_edge_by_name(
        result, CAD_0.geometry.ShapeId(
            CAD_0.geometry.FeatureId(4), [4, 1, 0, 0]))
    assert len(found) == 2
    assert found[0] == 12
    assert found[1] == 13
    # And by the rendered name, for the report's own eye.
    assert brep_mod.edge_query(result, 12)["name"] == RIM_FLOOR
    assert brep_mod.face_query(result, 7)["name"] == CAP_LO


def test_the_click_test_the_blind_volume_read_from_the_live():
    """The maintainer's own gesture, end to end through the node: the
    scene builds the chain (Box -> Sketch -> Extrude -> Sub with the
    blind span), the subtract node's exact B-Rep materializes (the
    refusal the v0.3-era node recorded is GONE — the door opened),
    and the blind's volume is read from the LIVE."""
    s = scene_mod.Scene()
    box_node = s.add_node("Box", CAD_0.sdf.box(Vec3f(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node(
        "S", [scene_mod.SketchChain(
            [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])])
    eid = s.add_extrude_node("Extrude", sid, 2.0, 1)
    did = s.add_operation_node("Sub", box_node, eid, 0.0, 0.0, -0.5)
    n = s.get_node(did)

    # The door opened: the exact B-Rep is THERE (the m2 era recorded
    # the orientation wall's refusal here — the same scene, the same
    # recipe, the opposite outcome).
    assert n.exact_brep is not None
    assert n.hybrid_refusal == ""
    summary = brep_mod.body_query(n.exact_brep)
    assert summary["V"] == 12
    assert summary["E"] == 17
    assert summary["F"] == 8
    assert summary["H"] == 1
    assert summary["chi"] == 2
    # THE VOLUME READ FROM THE LIVE.
    mp = brep_mod.mass_properties(n.exact_brep)
    assert abs(mp["volume"] - GOLDEN_VOLUME) < 1e-9
    assert mp["all_exact"]
    # The report prints the floor rim's intra-body row from the live.
    edges = report_mod.edge_table_text(n.exact_brep)
    assert RIM_FLOOR in edges
    assert CAP_LO in edges


def test_the_unnamed_mode_honest_boundaries():
    box, chains, tool, result = _guide_case(named=False)
    # The born rims and the seam read honestly EMPTY (no sub context
    # involved); the wall's and the cap's inheritances flow
    # regardless (stamp copies, no table involved).
    for i in (12, 13, 14, 15, 16):
        assert brep_mod.edge_query(result, i)["name"] == ""
    assert brep_mod.face_query(result, 6)["name"] == LAT_0
    assert brep_mod.face_query(result, 7)["name"] == CAP_LO
    # The address plane untouched by the naming.
    assert brep_mod.face_count(result) == 8
    assert brep_mod.edge_count(result) == 17


def test_the_still_refused_families_keep_their_words():
    """The borders: the both-caps-inside cavity refuses with ITS OWN
    NAME (the two-shell body); the CHAIN blind refuses with its own
    row (the per-entity rim projection the walk table queues)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0))

    # The both-caps-inside cavity: z in [-0.4, 0.4].
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.25, False)])]
    tool = brep_mod.make_prism(chains, 0.8, +1)
    try:
        brep_mod.exact_subtract(box, tool, Vec3f(0.0, 0.0, -0.4))
        raised = False
        msg = ""
    except ValueError as e:
        raised = True
        msg = str(e)
    assert raised
    assert "BLIND" in msg
    assert "TWO-SHELL" in msg
    assert "Remedy" in msg

    # The chain blind: the square tool with the blind span.
    tool2 = brep_mod.make_prism(
        [scene_mod.SketchChain([
            scene_mod.sketch_segment((-0.3, -0.3), (0.3, -0.3), True),
            scene_mod.sketch_segment((0.3, -0.3), (0.3, 0.3), True),
            scene_mod.sketch_segment((0.3, 0.3), (-0.3, 0.3), True),
            scene_mod.sketch_segment((-0.3, 0.3), (-0.3, -0.3), True),
        ])], 1.6, +1)
    try:
        brep_mod.exact_subtract(box, tool2, Vec3f(0.0, 0.0, -1.2))
        raised = False
        msg = ""
    except ValueError as e:
        raised = True
        msg = str(e)
    assert raised
    assert "BLIND" in msg
    assert "CHAIN" in msg
    assert "Remedy" in msg


def test_purity_two_materializations_one_answer():
    a = _guide_case()
    b = _guide_case()
    assert report_mod.face_table_text(a[3]) == report_mod.face_table_text(b[3])
    assert report_mod.edge_table_text(a[3]) == report_mod.edge_table_text(b[3])
    assert report_mod.edge_table_text(a[2]) == report_mod.edge_table_text(b[2])
