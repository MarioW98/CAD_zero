"""G5 m5 — the contemporanea's Python surface (ADR-0043 D6.5/D6.6):
the recipe authored from Python, the verdicts as VALUES (never
throws), the round-trip through the v0.8 door, the uniform edit.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

try:
    import CAD_0
except Exception:  # pragma: no cover
    pytest.skip("CAD_0 bindings not built", allow_module_level=True)


TOKS = ["bottom0", "bottom1", "bottom2", "bottom3",
        "top0", "top1", "top2", "top3",
        "vertical0", "vertical1", "vertical2", "vertical3"]


def guide_scene():
    s = CAD_0.scene.Scene()
    s.add_node("Cube", CAD_0.sdf.box(CAD_0.math.Vec3f(1.0, 1.0, 1.0)))
    return s


def twelve(r=0.5, odd=None):
    out = []
    for k, t in enumerate(TOKS):
        out.append(("box#1:" + t, odd if (odd is not None and k == 8)
                    else r))
    return out


def test_the_twelve_through_the_recipe():
    s = guide_scene()
    fid = s.add_fillet_multi_node("", twelve())
    n = s.get_node(fid)
    assert n.kind == "fillet_multi"
    assert len(n.fillet_multi_entries) == 12
    assert n.fillet_multi_entries[0][0] == "box#1:bottom0"
    assert n.fillet_multi_entries[8][0] == "box#1:vertical0"
    body = n.fillet_brep
    assert body is not None
    assert CAD_0.brep.edge_count(body) == 48  # 24V/48E/26F: the digest of the act
    assert CAD_0.brep.edge_count(body) == 48
    assert CAD_0.brep.face_count(body) == 26
    mp = CAD_0.brep.mass_properties(body)
    import math
    assert mp["volume"] == pytest.approx(
        4.0 + 11.0 * math.pi / 12.0, rel=1e-12)


def test_the_law_refused_by_name_never_at_the_kernel():
    s = guide_scene()
    with pytest.raises((ValueError, RuntimeError)) as ei:
        s.add_fillet_multi_node("", twelve(odd=0.4))
    msg = str(ei.value)
    assert "fillet_corner_radii_mismatch" in msg
    assert msg.startswith("attach_fillet_multi:")
    assert "fillet_ring_act:" not in msg
    assert "fillet_edge:" not in msg
    assert s.node_count() == 1


def test_the_verdicts_are_values_never_throws():
    s = guide_scene()
    v = s.fillet_admission_multi(twelve())
    assert v["admitted"] is True and v["reason"] == ""
    assert v["operand_name"] == "Cube"
    bad = s.fillet_admission_multi(twelve(odd=0.4))
    assert bad["admitted"] is False
    assert "fillet_corner_radii_mismatch" in bad["reason"]
    # The empty and duplicate maps: still values.
    assert s.fillet_admission_multi([])["admitted"] is False
    dup = s.fillet_admission_multi(
        [("box#1:vertical0", 0.5), ("box#1:vertical0", 0.5)])
    assert dup["admitted"] is False
    assert "twice" in dup["reason"]


def test_the_single_admissions_12_in():
    s = guide_scene()
    for t in TOKS:
        v = s.fillet_admission("box#1:" + t, 0.3)
        assert v["admitted"], t


def test_the_uniform_edit_and_the_round_trip(tmp_path):
    s = guide_scene()
    fid = s.add_fillet_multi_node("", twelve())
    assert s.set_fillet_multi_radius(fid, 0.3) == "reference_resolved"
    n = s.get_node(fid)
    assert CAD_0.brep.edge_count(n.fillet_brep) == 48
    import math
    a = 2.0 - 2.0 * 0.3
    want = a ** 3 + 6 * a * a * 0.3 + 3 * math.pi * a * 0.09 + \
        (4.0 / 3.0) * math.pi * 0.027
    mp = CAD_0.brep.mass_properties(n.fillet_brep)
    assert mp["volume"] == pytest.approx(want, rel=1e-12)

    # The v0.8 door: save -> load, the recipe entry for entry.
    p = str(tmp_path / "g5m5.CAD_0")
    CAD_0.io.save_native(p, s)
    assert CAD_0.io.NATIVE_VERSION == "0.8.0"
    s2 = CAD_0.scene.Scene()
    CAD_0.io.load_native(p, s2)
    n2 = s2.get_node(fid)
    assert n2 is not None and n2.kind == "fillet_multi"
    assert list(n2.fillet_multi_entries) == list(n.fillet_multi_entries)
    p2 = str(tmp_path / "g5m5_b.CAD_0")
    CAD_0.io.save_native(p2, s2)
    assert open(p, "rb").read() == open(p2, "rb").read()
