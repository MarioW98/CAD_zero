"""H2 unit 6, morso 2 — the factory roots' names through the bindings
(ADR-0036 D1, the accepted act).

The C++ side pins the whole contract (tests/unit/brep/
test_h2_u6_m2_factory_names.cpp — the exact names, the purity, the
round-trip, the structural equality, the tavola). What THESE tests
pin is what crosses the binding intact, the maintainer's own
click-test included:

  * ``CAD_0.brep.face_query`` carries ``name`` and ``shape_id`` —
    the canonical render beside the address, and the structure as
    plain values (feature + path + generation);
  * the CLICK-TEST: the box interrogated from the live conversion
    (``CAD_0.bridge.sdf_to_brep`` — the primitive-fit path returns
    the FACTORY body, the delegation) answers bottom / top / front
    / back / left / right, in shell order;
  * ``CAD_0.brep.edge_query`` / ``edge_count`` exist and carry the
    edge names (the fillet's references are edge-granular — D4's
    first consumer);
  * the sphere and the cylinder answer with ADR-0023's own tokens
    (sphere/seam; lateral/top/bot, top1/top2/bot1/bot2/seam);
  * the REPORT prints the name column (``CAD_0.report.
    face_table_text`` — a NEW surface beside body_report_text, which
    does not move);
  * purity from Python: two conversions of the same SDF body render
    the identical table, and the name digest is stable;
  * the honest empty: the FACETED extraction reads name "" — never
    a guess (the v1 named classes are the three factories);
  * the bridge's primitive fit stamps feature 0 (the default — no
    DAG identity is supplied on that path; registered in the
    morse's report).

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import bridge as bridge_mod
from CAD_0 import report as report_mod
from CAD_0 import sdf as sdf_mod

Vec3f = CAD_0.math.Vec3f

BOX_FACES = ["bottom", "top", "front", "back", "left", "right"]
BOX_EDGES = [
    "bottom0", "bottom1", "bottom2", "bottom3",
    "top0", "top1", "top2", "top3",
    "vertical0", "vertical1", "vertical2", "vertical3",
]


def _box(half=Vec3f(1.0, 2.0, 3.0)):
    return bridge_mod.sdf_to_brep(sdf_mod.box(half))


def test_face_query_carries_the_name_and_the_structure():
    body = _box()
    assert brep_mod.face_count(body) == 6
    for i, token in enumerate(BOX_FACES):
        q = brep_mod.face_query(body, i)
        # The bridge's primitive fit stamps the DEFAULT feature 0 —
        # the DAG identity is the caller's to supply on the paths
        # that have one; here the token is the pin.
        assert q["name"] == "box#0:" + token
        sid = q["shape_id"]
        assert sid["feature"] == 0
        assert sid["path"] == [i]
        assert sid["generation"] == 0


def test_the_click_test_the_live_box_answers_six_names():
    """The maintainer's click-test, pinned: the box interrogated from
    the live conversion shows bottom/top/front/back/left/right."""
    body = _box()
    names = [brep_mod.face_query(body, i)["name"] for i in range(6)]
    assert [n.split(":")[1] for n in names] == BOX_FACES


def test_edge_query_carries_the_names():
    body = _box()
    assert brep_mod.edge_count(body) == 12
    for i, token in enumerate(BOX_EDGES):
        q = brep_mod.edge_query(body, i)
        assert q["name"] == "box#0:" + token
        assert q["manifold"] is True
        assert q["coedge_count"] == 2
    # The loud miss, the face query's own culture.
    with pytest.raises(ValueError):
        brep_mod.edge_query(body, 12)


def test_the_sphere_and_cylinder_answer_adr_0023_tokens():
    sph = bridge_mod.sdf_to_brep(sdf_mod.sphere(1.5))
    assert brep_mod.face_query(sph, 0)["name"] == "sph#0:sphere"
    assert brep_mod.edge_query(sph, 0)["name"] == "sph#0:seam"

    cyl = bridge_mod.sdf_to_brep(sdf_mod.cylinder(0.5, 2.0))
    assert brep_mod.face_query(cyl, 0)["name"] == "cyl#0:lateral"
    assert brep_mod.face_query(cyl, 1)["name"] == "cyl#0:top"
    assert brep_mod.face_query(cyl, 2)["name"] == "cyl#0:bot"
    for i, token in enumerate(["top1", "top2", "bot1", "bot2", "seam"]):
        assert brep_mod.edge_query(cyl, i)["name"] == "cyl#0:" + token


def test_the_report_prints_the_name_column():
    body = _box()
    table = report_mod.face_table_text(body)
    lines = table.splitlines()
    assert lines[0] == "Faces (address | name | family | loops | provenance):"
    assert len(lines) == 1 + 6
    for i, token in enumerate(BOX_FACES):
        row = lines[1 + i]
        assert row.startswith("  {} | box#0:{} | plane | ".format(i, token))
        assert row.endswith(" | exact")
    # The existing report does not move: body_report_text keeps its
    # own shape (the name column is a NEW surface beside it).
    text = report_mod.body_report_text(body)
    assert "V = 8, E = 12, F = 6" in text
    assert "box#0:" not in text


def test_purity_two_conversions_one_answer():
    a = _box()
    b = _box()
    names_a = [brep_mod.face_query(a, i)["name"] for i in range(6)]
    names_b = [brep_mod.face_query(b, i)["name"] for i in range(6)]
    assert names_a == names_b
    assert report_mod.face_table_text(a) == report_mod.face_table_text(b)


def test_the_faceted_world_reads_honestly_unnamed():
    # The MC33 extraction (no primitive fit): plane-per-triangle
    # faces, no p-curves — and NO names, the honest empty, never a
    # guess (the v1 named classes are the three factories).
    faceted = bridge_mod.sdf_to_brep_faceted(sdf_mod.sphere(1.0))
    for i in range(brep_mod.face_count(faceted)):
        q = brep_mod.face_query(faceted, i)
        assert q["name"] == ""
        assert q["shape_id"]["path"] == []
