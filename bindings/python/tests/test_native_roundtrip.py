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
