"""Builder registry — single-source-of-truth tests (FASE 2).

The parametric builder names / parameters / defaults used to exist in TWO
copies that could diverge: core/io/src/native.cpp (make_builder_factory)
and app/src/CAD_0/ui/main_window.py (_make_default_params / _make_body /
_rebuild_node / on_create_*). FASE 2 moves the single source into the C++
kernel (core/scene/builders.hpp) and exposes it through the bindings so
the UI consumes ONLY the registry.

These tests pin:
  * the kernel registry API exists and is self-consistent
    (tags, specs, defaults, make_body);
  * an unregistered builder tag fails with an explicit error on both
    sides (kernel API and UI helpers);
  * the desktop UI no longer contains hardcoded parameter tables and
    delegates default_params / make_body to the kernel registry;
  * round-trip behavior is unchanged (the load path keeps re-wiring
    factories — see test_native_roundtrip.py).

Requires only numpy for the kernel part; the UI-source checks read the
checked-out source file, so they work everywhere. UI method-level checks
import PySide6 and skip gracefully without it (CI matrix).
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

import CAD_0
from CAD_0 import scene as scene_mod
from CAD_0 import sdf as sdf_mod

# The UI source as checked out (tests run from the repo root in CI;
# __file__ = <repo>/bindings/python/tests/test_builder_registry.py).
REPO_ROOT = Path(__file__).resolve().parents[3]
UI_MAIN_WINDOW = REPO_ROOT / "app" / "src" / "CAD_0" / "ui" / "main_window.py"

# Hardcoded parameter-table markers that must NOT appear in the UI source
# once it delegates to the kernel registry: param names + literal default
# values. (Plain `self._node_params.get(node_id)` bookkeeping is fine —
# the marker targets the param-NAME-in-code pattern, e.g.
# params.get("radius", 1.0).)
PARAM_TABLE_MARKERS = (
    "extent_x",
    "extent_y",
    "extent_z",
    "major_radius",
    "minor_radius",
    '"radius": 1.0',
    '"radius": 0.5',
    '"height": 2.0',
    'params.get("',
    "params.get('",
)

EXPECTED_TAGS = ["sphere", "box", "cylinder", "torus"]


# ---- kernel registry API ------------------------------------------------------


def test_registry_tags():
    tags = list(scene_mod.builder_tags())
    assert tags == EXPECTED_TAGS


def test_registry_specs_are_self_consistent():
    specs = list(scene_mod.builder_specs())
    assert [s["tag"] for s in specs] == EXPECTED_TAGS
    for spec in specs:
        assert spec["display_name"]  # non-empty UI label
        # default_params(tag) is exactly the spec's param schema.
        assert dict(scene_mod.default_params(spec["tag"])) == dict(spec["params"])
        assert len(spec["params"]) >= 1


def test_default_params_are_sane_numbers():
    # Values arrive as float32 -> Python doubles, so compare with approx.
    defaults = {
        tag: dict(scene_mod.default_params(tag)) for tag in scene_mod.builder_tags()
    }
    assert defaults["sphere"] == {"radius": pytest.approx(1.0)}
    assert defaults["box"] == {
        "extent_x": pytest.approx(0.5),
        "extent_y": pytest.approx(0.5),
        "extent_z": pytest.approx(0.5),
    }
    assert defaults["cylinder"] == {
        "radius": pytest.approx(0.5),
        "height": pytest.approx(2.0),
    }
    assert defaults["torus"] == {
        "major_radius": pytest.approx(1.0),
        "minor_radius": pytest.approx(0.3),
    }


def test_make_body_builds_a_solid_for_every_builder():
    for tag in scene_mod.builder_tags():
        body = scene_mod.make_body(tag, scene_mod.default_params(tag))
        # Non-degenerate bounds around the origin (note: a torus does NOT
        # contain the origin — the point sits in the hole).
        m = body.bounds().max
        assert (m.x, m.y, m.z) == (m.x, m.y, m.z)  # finite
        assert m.x > 0.0 and m.y >= 0.0 and m.z > 0.0

        # Empty params fall back to the SAME defaults (single source).
        empty = scene_mod.make_body(tag, {})
        assert empty.value(CAD_0.math.Vec3f(0, 0, 0)) == body.value(
            CAD_0.math.Vec3f(0, 0, 0)
        )

        # An edited param actually changes the field. Vec3f has no __eq__
        # in the bindings, so compare bounds component-wise; the SDF value
        # at the origin is NOT a good probe (box = min over the axes).
        def bmax(body):
            m = body.bounds().max
            return (m.x, m.y, m.z)

        for key in dict(scene_mod.default_params(tag)):
            edited = scene_mod.make_body(tag, {key: 4.5})
            assert bmax(edited) != bmax(body), (tag, key)


def test_make_body_matches_the_sdf_module_primitives():
    # The registry build must agree with the plain sdf constructors.
    p = CAD_0.math.Vec3f(0.7, -0.3, 0.9)
    assert scene_mod.make_body("sphere", {"radius": 2.0}).value(p) == \
        sdf_mod.sphere(2.0).value(p)
    assert scene_mod.make_body(
        "box", {"extent_x": 1.0, "extent_y": 0.5, "extent_z": 0.25}
    ).value(p) == sdf_mod.box(CAD_0.math.Vec3f(1.0, 0.5, 0.25)).value(p)
    assert scene_mod.make_body(
        "cylinder", {"radius": 0.5, "height": 3.0}
    ).value(p) == sdf_mod.cylinder(0.5, 3.0).value(p)
    assert scene_mod.make_body(
        "torus", {"major_radius": 1.5, "minor_radius": 0.4}
    ).value(p) == sdf_mod.torus(1.5, 0.4).value(p)


def test_unknown_builder_tag_fails_explicitly_kernel_side():
    for call in (
        lambda: scene_mod.default_params("warp_drive"),
        lambda: scene_mod.make_body("warp_drive", {}),
    ):
        with pytest.raises(RuntimeError, match="unknown builder.*warp_drive"):
            call()
    assert scene_mod.has_builder("warp_drive") is False
    assert scene_mod.has_builder("sphere") is True


# ---- UI: no hardcoded tables, delegation to the kernel -----------------------


def test_ui_source_has_no_hardcoded_param_tables():
    source = UI_MAIN_WINDOW.read_text()
    for marker in PARAM_TABLE_MARKERS:
        assert marker not in source, (
            f"main_window.py still hardcodes parameter-table marker {marker!r} — "
            "parametric builders must come from the kernel registry "
            "(CAD_0.scene), not from UI tables"
        )


def test_ui_source_delegates_to_the_kernel_registry():
    source = UI_MAIN_WINDOW.read_text()
    # The delegation calls must be present (the single-source contract).
    assert "CAD_0.scene.default_params(" in source
    assert "CAD_0.scene.make_body(" in source
    # The Create menu must be registry-driven, not a fixed action list.
    assert "builder_specs()" in source


def test_ui_helpers_delegate_and_raise_explicitly():
    pytest.importorskip("PySide6")  # main_window imports Qt at module load
    from types import SimpleNamespace

    from CAD_0.ui import main_window as mw

    # _make_body does not touch instance state — call it through the class.
    body = mw.MainWindow._make_body(None, "sphere", {"radius": 2.5})
    expected = sdf_mod.sphere(2.5)
    p = CAD_0.math.Vec3f(0.9, -0.2, 0.7)
    assert body.value(p) == expected.value(p)

    # _make_default_params only needs the two bookkeeping dicts.
    fake = SimpleNamespace(_node_types={}, _node_params={})
    mw.MainWindow._make_default_params(fake, "box", 42)
    assert fake._node_types[42] == "box"
    assert fake._node_params[42] == dict(scene_mod.default_params("box"))

    # Unknown builder: explicit error from the kernel on the make path...
    with pytest.raises(RuntimeError, match="unknown builder"):
        mw.MainWindow._make_body(None, "warp_drive", {})
    # ...and graceful non-parametric behavior on the defaults path
    # (mirrors the kernel load contract: unknown tag = not editable,
    #  but the file still opens — ADR-0007 PATCH compatibility).
    fake2 = SimpleNamespace(_node_types={}, _node_params={})
    mw.MainWindow._make_default_params(fake2, "warp_drive", 7)
    assert fake2._node_types[7] == "warp_drive"
    assert 7 not in fake2._node_params


def test_roundtrip_still_uses_builder_metadata():
    # The persisted builder tag + params still round-trip and the loaded
    # node is editable through the kernel (the registry feeds the load
    # path's factory re-wiring — detailed coverage lives in
    # test_native_roundtrip.py; this pins the wiring end-to-end).
    scene = scene_mod.Scene()
    nid = scene.add_node("part", scene_mod.make_body("sphere", {"radius": 1.5}))
    node = scene.get_node(nid)
    node.builder = "sphere"
    node.set_params(dict(scene_mod.default_params("sphere")) | {"radius": 1.5})

    path = Path(__file__).parent / "_registry_roundtrip.CAD_0"
    CAD_0.io.save(path, scene)
    try:
        loaded = scene_mod.Scene()
        CAD_0.io.load(path, loaded)
        loaded.set_node_param(nid, "radius", 3.0)
        expected = sdf_mod.sphere(3.0)
        q = CAD_0.math.Vec3f(1.2, 0.1, -0.4)
        assert loaded.node_at(0).body.value(q) == expected.value(q)
        assert math.copysign(1.0, loaded.node_at(0).body.value(q)) == \
            math.copysign(1.0, expected.value(q))
    finally:
        path.unlink(missing_ok=True)
