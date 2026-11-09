"""Native project format (.CAD_0) — Python round-trip tests.

Covers the binding layer (CAD_0.io.save/load) that the desktop app uses:
  * scene + camera + node metadata round-trip (ids, next_id, builder, params)
  * bit-exact SDF evaluation after a save/load cycle
  * atomic save leaves no .tmp residue
  * corrupted / unknown-op / incompatible-version files raise RuntimeError
    with explicit messages and leave the target scene untouched

Requires only numpy (no PySide6) so these run in CI.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
from pathlib import Path

import pytest

import CAD_0
from CAD_0 import sdf as sdf_mod
from CAD_0 import scene as scene_mod


def _make_scene() -> scene_mod.Scene:
    scene = scene_mod.Scene()
    nid = scene.add_node("part", sdf_mod.sphere(1.5))
    node = scene.get_node(nid)
    node.builder = "sphere"
    node.set_params({"radius": 1.5})
    return scene


def _camera() -> dict:
    return {
        "target": (0.5, -0.25, 1.0),
        "distance": 8.25,
        "yaw": 0.625,
        "pitch": -0.125,
        "fov_deg": 55.0,
    }


@pytest.fixture()
def scene():
    return _make_scene()


@pytest.fixture()
def saved_path(scene):
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "proj.CAD_0"
    CAD_0.io.save(path, scene, _camera())
    return path


def test_native_version():
    # The constant is built from the C++ kNativeVersion, never hardcoded
    # on the Python side. History: 0.2.0 was the binary blob container
    # (ADR-0007a); 0.3.0 the operation recipes (ADR-0027 D4); 0.4.0 the
    # sketcher's document layer — sketch nodes and extrude recipes
    # (ADR-0030 D4); 0.5.0 the constraints riding the sketch payload
    # as named fields (ADR-0040 D3.1, G4 m4).
    assert CAD_0.io.NATIVE_VERSION == "0.7.0"


def test_roundtrip_scene_and_metadata(saved_path):
    loaded = scene_mod.Scene()
    cam = CAD_0.io.load(saved_path, loaded)

    assert loaded.size() == 1
    node = loaded.node_at(0)
    assert node.name == "part"
    assert node.id == 1
    assert node.builder == "sphere"
    assert dict(node.params) == {"radius": 1.5}
    assert loaded.next_id() == 2

    # camera round-trips exactly
    assert cam["target"] == pytest.approx((0.5, -0.25, 1.0))
    assert cam["distance"] == pytest.approx(8.25)
    assert cam["yaw"] == pytest.approx(0.625)
    assert cam["pitch"] == pytest.approx(-0.125)
    assert cam["fov_deg"] == pytest.approx(55.0)


def test_roundtrip_evaluation_bit_exact(scene, saved_path):
    loaded = scene_mod.Scene()
    CAD_0.io.load(saved_path, loaded)

    p = CAD_0.math.Vec3f(0.7, -0.3, 0.9)
    assert loaded.node_at(0).body.value(p) == scene.node_at(0).body.value(p)
    # bit-exact (not just approx): same float bits
    a = loaded.node_at(0).body.value(p)
    b = scene.node_at(0).body.value(p)
    assert math.copysign(1.0, a) == math.copysign(1.0, b)


def test_composite_tree_roundtrip():
    # A composite body exercising operators + transforms.
    body = sdf_mod.union(
        sdf_mod.subtract(sdf_mod.box(CAD_0.math.Vec3f(1, 1, 1)),
                         sdf_mod.cylinder(0.5, 2.0)),
        sdf_mod.translate(sdf_mod.torus(1.0, 0.3),
                          CAD_0.math.Vec3f(2, 0, 0)))
    scene = scene_mod.Scene()
    scene.add_node("composite", body)
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "c.CAD_0"
    CAD_0.io.save(path, scene)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    p = CAD_0.math.Vec3f(1.2, 0.1, 0.3)
    assert loaded.node_at(0).body.value(p) == body.value(p)


def test_atomic_save_no_tmp_residue(saved_path):
    assert saved_path.exists()
    assert not Path(str(saved_path) + ".tmp").exists()


def test_ids_and_next_id_roundtrip():
    scene = scene_mod.Scene()
    scene.add_node("a", sdf_mod.sphere(1.0))
    scene.add_node("b", sdf_mod.sphere(2.0))
    scene.remove_node(1)  # ids now: 2 (next_id = 3)
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "ids.CAD_0"
    CAD_0.io.save(path, scene)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    assert loaded.size() == 1
    assert loaded.node_at(0).id == 2
    assert loaded.next_id() == 3
    # new nodes never collide with restored ids
    assert loaded.add_node("c", sdf_mod.sphere(1.0)) >= 3


def test_corrupt_json_raises_and_scene_untouched(saved_path, scene):
    target = _make_scene()
    # v0.2.0 container: truncating the FILE now trips the frame length
    # validation (declared lengths no longer add up) — still an explicit
    # RuntimeError, and still transactional.
    data = saved_path.read_bytes()
    saved_path.write_bytes(data[: len(data) // 3])
    with pytest.raises(RuntimeError, match="truncated or corrupt"):
        CAD_0.io.load(saved_path, target)
    assert target.size() == 1  # untouched

    # The legacy pure-JSON parse-error path (framing '{') stays covered.
    legacy = Path(tempfile.mkdtemp(prefix="cad0_corrupt_")) / "legacy.CAD_0"
    legacy.write_text('{"format": "CAD_0 native", "version": "0.1.0"')
    with pytest.raises(RuntimeError, match="parse error"):
        CAD_0.io.load(legacy, target)
    assert target.size() == 1  # untouched


def test_unknown_op_raises_explicitly():
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "bad.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "0.1.0", '
        '"scene": {"next_id": 2, "nodes": [{"id": 1, "body": {"op": "warp_drive"}}]}}'
    )
    with pytest.raises(RuntimeError, match='unknown SDF op "warp_drive"'):
        CAD_0.io.load(path, scene_mod.Scene())


def test_out_of_class_scale_zero_raises_value_error():
    """ADR-0029 D5: a scale s=0 node in a file is an explicit load error.

    The constructor's std::invalid_argument joins the ADR-0007 explicit
    error family and reaches the user radius (the Python binding) as
    ValueError carrying the COMPLETE bridge-style message — operator,
    parameter, value, supported class, diagnosis, remedy — not a bare
    type. The load stays transactional: the scene is untouched.
    """
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "s0.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "0.1.0", '
        '"scene": {"next_id": 2, "nodes": [{"id": 1, "body": {'
        '"op": "scale", "s": 0.0, '
        '"child": {"op": "sphere", "radius": 1.0}}}]}}'
    )
    scene = scene_mod.Scene()
    scene.add_node("keep", sdf_mod.sphere(1.0))

    with pytest.raises(ValueError) as excinfo:
        CAD_0.io.load(path, scene)
    msg = str(excinfo.value)
    # The full message: operator, parameter, supported class, value...
    assert "ScaleSDF: scale factor s must be finite and > 0, got 0.000000" in msg
    # ...diagnosis (why s <= 0 is outside the describable class)...
    assert "flips the field's sign" in msg
    # ...and the remedy.
    assert "Remedy: scale by s > 0" in msg
    # Transactional: the failed load left the scene untouched.
    assert scene.size() == 1
    assert scene.node_at(0).name == "keep"


def test_incompatible_version_raises_with_no_migrator():
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "v2.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "2.0.0", '
        '"scene": {"next_id": 1, "nodes": []}}'
    )
    with pytest.raises(RuntimeError, match="no migrator registered"):
        CAD_0.io.load(path, scene_mod.Scene())


def test_patch_version_compatibility():
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "p.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "0.1.3", '
        '"scene": {"next_id": 1, "nodes": []}}'
    )
    scene = scene_mod.Scene()  # must not raise — PATCH is compatible
    CAD_0.io.load(path, scene)
    assert scene.size() == 0


def test_scene_change_callback_roundtrip():
    scene = scene_mod.Scene()
    events = []
    scene.set_change_callback(lambda kind, nid: events.append((str(kind), nid)))
    nid = scene.add_node("x", sdf_mod.sphere(1.0))
    assert events == [("SceneChange.NodeAdded", nid)]

    # None detaches (window close path)
    scene.set_change_callback(None)
    scene.add_node("y", sdf_mod.sphere(1.0))
    assert len(events) == 1


# ---- post-load parametric editing (factory re-wiring) ------------------------
#
# Diagnostic for the suspected bug: after load, SceneNode::factory was empty
# (a std::function cannot be serialized), so Scene.set_node_param() updated
# the stored params but silently did NOT rebuild the body nor flag
# mesh_dirty. The desktop UI hides this by rebuilding bodies in Python
# (main_window._rebuild_node) — the tests below pin BOTH paths so the
# kernel contract and the UI path stay correct.


def _save_ui_shaped_project(path) -> int:
    """Save a project exactly the way the desktop app does: add_node +
    builder tag + params (no factory — the UI never sets one)."""
    scene = scene_mod.Scene()
    nid = scene.add_node("part", sdf_mod.sphere(1.5))
    node = scene.get_node(nid)
    node.builder = "sphere"
    node.set_params({"radius": 1.5})
    CAD_0.io.save(path, scene)
    return nid


def test_kernel_param_edit_after_load():
    # FAILS without the factory re-wiring fix in the native loader.
    path = Path(tempfile.mkdtemp(prefix="cad0_edit_")) / "proj.CAD_0"
    nid = _save_ui_shaped_project(path)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)

    # The viewport consumes the post-load dirty flag on first paint.
    loaded.update_meshes(16)
    node = loaded.node_at(0)
    assert node.mesh_dirty is False

    # Kernel-level edit (what Scene.set_node_param documents).
    loaded.set_node_param(nid, "radius", 3.0)

    assert node.mesh_dirty is True  # re-tessellation flagged
    assert dict(node.params) == {"radius": 3.0}
    # Body rebuilt bit-exactly from the new params.
    expected = sdf_mod.sphere(3.0)
    for p in (CAD_0.math.Vec3f(0, 0, 0), CAD_0.math.Vec3f(1.4, -0.2, 0.7),
              CAD_0.math.Vec3f(-1.5, 1.5, 1.5)):
        assert node.body.value(p) == expected.value(p)
    assert node.body.bounds().max.x == pytest.approx(3.0)

    # The re-tessellated mesh actually follows the edit.
    loaded.update_meshes(16)
    assert node.cached_mesh.vertex_count() > 0
    # Every vertex of sphere(3) mesh lies inside the radius-3 ball.
    import numpy as np
    assert float(np.abs(node.cached_mesh.positions).max()) <= 3.0 + 1e-3
    assert float(np.abs(node.cached_mesh.positions).max()) > 2.0  # grew vs r=1.5


def test_kernel_param_edit_survives_resave():
    path = Path(tempfile.mkdtemp(prefix="cad0_edit2_")) / "proj.CAD_0"
    nid = _save_ui_shaped_project(path)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    loaded.set_node_param(nid, "radius", 3.0)
    CAD_0.io.save(path, loaded)

    again = scene_mod.Scene()
    CAD_0.io.load(path, again)
    node = again.node_at(0)
    assert dict(node.params) == {"radius": 3.0}
    expected = sdf_mod.sphere(3.0)
    p = CAD_0.math.Vec3f(0.7, -0.3, 0.9)
    assert node.body.value(p) == expected.value(p)


def test_constraints_ride_the_payload_v0_5():
    # G4 m4 (ADR-0040 D3.1): the sketch's constraint list rides the
    # payload beside the chains — named fields (kind, the operands as
    # canonical indices chain -> entity -> field, the value, the
    # tangent's case bit, the pin with its parameter index), never a
    # raw struct. The binding surface round-trips it exactly; the
    # re-save is byte-identical (the digest proof at the Python door).
    er = scene_mod.SketchEntityRef
    pr = scene_mod.SketchPointRef
    scene = scene_mod.Scene()
    chains = [scene_mod.SketchChain([
        scene_mod.sketch_segment((0.0, 0.0), (4.0, 0.0), True),
        scene_mod.sketch_segment((4.0, 0.0), (4.0, 3.0), True),
        scene_mod.sketch_segment((4.0, 3.0), (0.0, 0.0), True),
    ])]
    constraints = [
        scene_mod.constraint_coincidence(
            pr(er(0, 0), True), pr(er(0, 1), False)),
        scene_mod.constraint_coincidence(
            pr(er(0, 1), True), pr(er(0, 2), False)),
        scene_mod.constraint_length(er(0, 0), 4.0),
        scene_mod.constraint_length(er(0, 1), 3.0),
        scene_mod.constraint_perpendicular(er(0, 0), er(0, 1)),
        scene_mod.constraint_pin(
            scene_mod.SketchParamRef(er(0, 0), 0), 0.0),
    ]
    sid = scene.add_sketch_node("S", chains, constraints)
    assert sid == 1

    path = Path(tempfile.mkdtemp(prefix="cad0_cstr_")) / "proj.CAD_0"
    CAD_0.io.save(path, scene, _camera())

    loaded = scene_mod.Scene()
    cam = CAD_0.io.load(path, loaded)
    back = loaded.sketch_constraints(1)
    assert len(back) == 6
    kinds = [str(c.kind) for c in back]
    assert kinds == ["coincidence", "coincidence", "length", "length",
                     "perpendicular", "pin"]
    # Field for field: the operands as canonical indices, the values
    # exact, the pin's parameter index stable (the circle-free segment
    # x0 — the same field before and after the round-trip).
    assert back[0].a.entity.chain == 0 and back[0].a.entity.entity == 0
    assert back[0].a.end is True
    assert back[0].b.entity.entity == 1
    assert back[2].value == 4.0
    assert back[3].value == 3.0
    assert back[5].pin.entity.entity == 0
    assert back[5].pin.field == 0
    assert back[5].value == 0.0

    # The session-level surface hydrates from the payload (the m4
    # workbench's own construction route). The bill is the honest one
    # for this mix: 12 DoF (three segments), 8 consumed (two
    # coincidences 4 + two lengths 2 + the perpendicular 1 + the pin
    # 1) — 4 free, reported, never hidden.
    from CAD_0.ui.sketch_workbench import SketchSolverSession
    session = SketchSolverSession(loaded, 1)
    assert session.constraint_count() == 6
    assert session.bill_text() == "12 - 8 = 4 free"

    # THE RE-SAVE: byte-identical (the digest proof).
    path2 = path.parent / "proj_rt.CAD_0"
    CAD_0.io.save(path2, loaded, cam)
    assert path2.read_bytes() == path.read_bytes()


def test_ui_path_param_edit_after_load():
    # The exact API sequence main_window._rebuild_node() uses — the UI
    # compensation path. Must keep working alongside the kernel fix.
    path = Path(tempfile.mkdtemp(prefix="cad0_edit3_")) / "proj.CAD_0"
    nid = _save_ui_shaped_project(path)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    loaded.update_meshes(16)
    node = loaded.node_at(0)
    assert node.mesh_dirty is False

    # UI editing path: rebuild in Python, assign, sync metadata.
    node.body = sdf_mod.sphere(3.0)  # binding setter flags mesh_dirty
    node.mesh_dirty = True
    node.builder = "sphere"
    node.set_params({"radius": 3.0})

    assert node.mesh_dirty is True
    loaded.update_meshes(16)
    assert node.cached_mesh.vertex_count() > 0
    expected = sdf_mod.sphere(3.0)
    p = CAD_0.math.Vec3f(1.4, -0.2, 0.7)
    assert node.body.value(p) == expected.value(p)


def test_no_builder_tag_node_is_non_parametric_after_load():
    # Composite body (no builder tag): loads + round-trips, but
    # set_node_param updates the stored params only.
    body = sdf_mod.subtract(sdf_mod.box(CAD_0.math.Vec3f(1, 1, 1)),
                            sdf_mod.sphere(0.75))
    scene = scene_mod.Scene()
    scene.add_node("composite", body)
    path = Path(tempfile.mkdtemp(prefix="cad0_edit4_")) / "c.CAD_0"
    CAD_0.io.save(path, scene)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    loaded.update_meshes(16)
    node = loaded.node_at(0)
    assert node.mesh_dirty is False

    p = CAD_0.math.Vec3f(0.5, 0.5, 0.5)
    before = node.body.value(p)
    loaded.set_node_param(1, "radius", 3.0)

    assert node.body.value(p) == before  # body untouched
    assert node.mesh_dirty is False      # no spurious re-tessellation
    assert dict(node.params) == {"radius": 3.0}


# ---- depth limits (stack-overflow hardening) ----------------------------------
#
# Diagnostic: the JSON parser and the SDF deserializer were recursive with
# no depth guard — a hostile .CAD_0 file with ~100k nested arrays crashed
# the whole process (stack overflow) before any error handling could run.
# These tests pin the contract through the same CAD_0.io.load API the
# desktop app uses: explicit RuntimeError naming the limit, scene left
# untouched (transactional load), and legitimate deep trees still load.


def test_json_100k_nested_arrays_raises_not_crashes():
    # The hostile-input case: must raise RuntimeError, never segfault
    # the Python process. (Before the guard this test crashed pytest.)
    # The document starts with '{' so it passes the framing check and
    # reaches the JSON parser's depth guard.
    path = Path(tempfile.mkdtemp(prefix="cad0_deep_")) / "hostile.CAD_0"
    path.write_text('{"x":' + "[" * 100_000 + "1" + "]" * 100_000 + "}")

    target = scene_mod.Scene()
    target.add_node("keepme", sdf_mod.sphere(1.0))
    with pytest.raises(RuntimeError, match="too deeply nested"):
        CAD_0.io.load(path, target)
    assert target.size() == 1  # transactional: scene untouched


def test_sdf_tree_over_limit_raises_explicit_max_depth():
    # 200 translate wrappers: JSON depth ~204 (under the JSON guard) but
    # 200 SDF levels — the SDF-layer limit must fire with the limit in
    # the message.
    body = '{"op":"translate","offset":[0,0,0],"child":' * 200 + \
           '{"op":"sphere","radius":1.0}' + "}" * 200
    path = Path(tempfile.mkdtemp(prefix="cad0_deep2_")) / "deep.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "0.1.0", '
        f'"scene": {{"next_id": 2, "nodes": [{{"id": 1, "body": {body}}}]}}}}'
    )
    with pytest.raises(RuntimeError, match=r"too deeply nested \(max 128"):
        CAD_0.io.load(path, scene_mod.Scene())


def test_legitimate_deep_tree_loads_and_evaluates_bit_exact():
    # 50 alternating translate/scale levels: deep for a real project, far
    # inside the limits — the guards must not break legitimate files.
    body = sdf_mod.sphere(1.0)
    for i in range(50):
        if i % 2 == 0:
            body = sdf_mod.translate(body, CAD_0.math.Vec3f(0.01 * i, 0, 0))
        else:
            body = sdf_mod.scale(body, 1.0 + 0.001 * i)
    scene = scene_mod.Scene()
    scene.add_node("deep", body)
    path = Path(tempfile.mkdtemp(prefix="cad0_deep3_")) / "legit.CAD_0"
    CAD_0.io.save(path, scene)

    loaded = scene_mod.Scene()
    CAD_0.io.load(path, loaded)
    loaded.update_meshes(16)
    node = loaded.node_at(0)
    assert node.mesh_dirty is False
    assert len(node.cached_mesh.positions) > 0

    for p in (CAD_0.math.Vec3f(0, 0, 0), CAD_0.math.Vec3f(0.9, -0.2, 0.4)):
        assert node.body.value(p) == body.value(p)



# ===========================================================================
# G2-u3 m4 (ADR-0039 D6 rung 4, the REBASED bump v0.6.0): the fillet
# recipe on the file — the BINDING's own proof. The reference survives
# the round-trip CHARACTER FOR CHARACTER; the door's refusal is
# readable from the binding; the radius edit after the load answers
# reference_resolved (the class-1 silent edit, one scene mutation).
# ===========================================================================
RIM_V0_6 = "sub#8:isect[box#7:top \u00d7 ext#6:lat(0)]@hi"


def _blind_chain_scene():
    """The m3 guide body on the scene (ids 1-5): the box [-1,1]^3, the
    circle r=0.5 at the origin, the prism [0,2] offset (0,0,-0.5) - the
    BLIND hole (the floor at -0.5, the rim at z=1), then the rim fillet
    discovered by the find (never typed by hand)."""
    from CAD_0 import math as math_mod
    from CAD_0 import brep as brep_mod

    sc = scene_mod.Scene()
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    sk = sc.add_sketch_node("Blind_Sketch", chains, [])
    ex = sc.add_extrude_node("Blind_Extrude", sketch=sk,
                             distance=2.0, direction=1)
    bx = sc.add_node("Blind_Box", sdf_mod.box(math_mod.Vec3f(1.0, 1.0, 1.0)))
    node = sc.get_node(bx)
    node.builder = "box"
    node.set_params({"extent_x": 1.0, "extent_y": 1.0, "extent_z": 1.0})
    sub = sc.add_operation_node("Blind_Subtract", target=bx, tool=ex,
                                 offset_z=-0.5)
    for nid in (sk, ex, bx):
        sc.set_visible(nid, False)
    blind = sc.get_node(sub)
    assert blind.exact_brep is not None
    rim = None
    for i in range(brep_mod.edge_count(blind.exact_brep)):
        e = brep_mod.edge_query(blind.exact_brep, i)
        if "@hi" in e["name"] and "isect" in e["name"]:
            rim = e["name"]
            break
    # The grammar's own derivation, checked (the DAG ids are the node
    # ids - the morse-5 rule): never typed by hand.
    expected = f"sub#{sub}:isect[box#{bx}:top \u00d7 ext#{ex}:lat(0)]@hi"
    assert rim == expected, (rim, expected)
    fid = sc.add_fillet_node("Rim_Fillet", rim, 0.25)
    assert fid == 5
    return sc, fid, rim


def test_fillet_recipe_rides_the_payload_v0_6(tmp_path):
    # The save/load of a filleted body from Python: the round-trip
    # reachable, the recipe's fields back exactly, the numbers unmoved
    # by the trip, the re-save byte-identical.
    import CAD_0
    from CAD_0 import brep as brep_mod

    sc, fid, rim = _blind_chain_scene()
    path = tmp_path / "fillet.CAD_0"
    CAD_0.io.save(path, sc)
    first = path.read_bytes()

    sc2 = scene_mod.Scene()
    cam = CAD_0.io.load(path, sc2)
    f = sc2.get_node(fid)
    # THE REFERENCE, character for character; the radius its own field.
    assert f.fillet_edge == rim
    assert f.fillet_edge.encode("utf-8").decode("utf-8") == rim
    assert f.fillet_radius == 0.25
    assert f.fillet_refusal == ""
    assert f.fillet_brep is not None
    q = brep_mod.body_query(f.fillet_brep)
    assert (q["V"], q["E"], q["F"], q["H"], q["chi"]) == (14, 20, 9, 1, 2)
    mp = brep_mod.mass_properties(f.fillet_brep, 1e-9)
    assert mp["all_exact"] is True
    vol = (8.0 - 0.375 * 3.141592653589793) - 2.0 * 3.141592653589793 * (
        0.5 * 0.25 * 0.25 * (1.0 - 3.141592653589793 / 4.0) +
        0.25 ** 3 * (5.0 / 6.0 - 3.141592653589793 / 4.0))
    assert abs(mp["volume"] - vol) < 1e-12

    # The re-save is byte-identical (the digests' own floor).
    CAD_0.io.save(path, sc2, cam)
    assert path.read_bytes() == first


def test_fillet_v0_5_door_readable_from_the_binding():
    # The door's refusal readable from the binding: the v0.5 golden
    # (frozen history) is refused BY NAME with the remedy - never a
    # best-effort load.
    import CAD_0

    from pathlib import Path as _P
    sc = scene_mod.Scene()
    with pytest.raises(RuntimeError) as ei:
        CAD_0.io.load(
            _P(__file__).resolve().parents[3]
            / "tests" / "golden" / "native_v0_5_0.CAD_0",
            sc)
    msg = str(ei.value)
    assert "the format moved to v0.6: re-apply the fillets" in msg
    # The door names the file's version and THIS build's (K1 m3:
    # the current number is 0.7.0 — the remedy's own words
    # still speak v0.6, the era the fillet recipe entered).
    assert "0.5.0" in msg and "0.7.0" in msg
    # The scene untouched (the load is transactional).
    assert sc.node_count() == 0


def test_fillet_radius_edit_after_the_load(tmp_path):
    # The class-1 SILENT edit after the load: ONE scene mutation, the
    # answer reference_resolved, the counts fixed, the volume the
    # closed form's own value at the new r.
    import CAD_0
    from CAD_0 import brep as brep_mod

    sc, fid, rim = _blind_chain_scene()
    path = tmp_path / "fillet.CAD_0"
    CAD_0.io.save(path, sc)
    sc2 = scene_mod.Scene()
    CAD_0.io.load(path, sc2)

    ans = sc2.set_fillet_radius(fid, 0.125)
    assert ans == "reference_resolved"
    f = sc2.get_node(fid)
    assert f.fillet_refusal == ""
    q = brep_mod.body_query(f.fillet_brep)
    assert (q["V"], q["E"], q["F"], q["H"], q["chi"]) == (14, 20, 9, 1, 2)
    mp = brep_mod.mass_properties(f.fillet_brep, 1e-9)
    r = 0.125
    vol = (8.0 - 0.375 * 3.141592653589793) - 2.0 * 3.141592653589793 * (
        0.5 * r * r * (1.0 - 3.141592653589793 / 4.0) +
        r ** 3 * (5.0 / 6.0 - 3.141592653589793 / 4.0))
    assert abs(mp["volume"] - vol) < 1e-12
