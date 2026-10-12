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


def test_native_version_is_010():
    assert CAD_0.io.NATIVE_VERSION == "0.1.0"


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
    text = saved_path.read_text()
    saved_path.write_text(text[: len(text) // 3])  # truncate mid-document
    with pytest.raises(RuntimeError, match="parse error"):
        CAD_0.io.load(saved_path, target)
    assert target.size() == 1  # untouched


def test_unknown_op_raises_explicitly():
    path = Path(tempfile.mkdtemp(prefix="cad0_native_")) / "bad.CAD_0"
    path.write_text(
        '{"format": "CAD_0 native", "version": "0.1.0", '
        '"scene": {"next_id": 2, "nodes": [{"id": 1, "body": {"op": "warp_drive"}}]}}'
    )
    with pytest.raises(RuntimeError, match='unknown SDF op "warp_drive"'):
        CAD_0.io.load(path, scene_mod.Scene())


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
    path = Path(tempfile.mkdtemp(prefix="cad0_deep_")) / "hostile.CAD_0"
    path.write_text("[" * 100_000 + "1" + "]" * 100_000)

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
