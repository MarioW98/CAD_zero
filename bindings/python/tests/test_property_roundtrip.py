"""Property-based round-trip family (mini-traccia PARTE 2, format area,
Python-binding side).

Generated scenes — parametric primitives with random builder params,
names, visibility and camera — must round-trip byte-identically through
the binding layer the desktop app uses (CAD_0.io.save / load):

  * save -> load -> save reproduces the same bytes;
  * the extraction summary (per-node id/name/builder/params/visibility,
    node count, next_id, camera) is identical before and after.

25 members, FIXED, seeded through property_prng.SeededPrng (the seed is
part of the test; every assertion message carries seed + iteration +
parameters + the violated check). The operation-recipe side of the
format family is covered by the C++ suite (tests/unit/io/test_native.cpp
"property: generated scenes round-trip byte-identically..."), which can
drive scene::add_operation_node directly.

Requires only the core bindings (no PySide6), like test_native_roundtrip.

Declared generator ranges (outside = generator bug):
  * sphere radius [0.3, 1.5]; box half-extent per axis [0.2, 1.0];
    cylinder radius [0.15, 0.5] and height [0.8, 3.0]
  * camera: target [-2, 2], distance [4, 12], yaw [-3, 3],
    pitch [-1.2, 1.2], fov [35, 70]
"""

from __future__ import annotations

import struct
from pathlib import Path

import CAD_0
from CAD_0 import math as math_mod
from CAD_0 import sdf as sdf_mod
from CAD_0 import scene as scene_mod

from property_prng import SeededPrng, case

SEED = 0x5EED0A01
ITERATIONS = 25


def _f32(v: float) -> float:
    """Round to the float32 the native format stores.

    ParamValue is a 32-bit float (and so is every camera field), so the
    generator produces float32-exact values: the equality checks then
    stay EXACT — no approx — which is the whole point of the family
    (generator class discipline, not a tolerance).
    """
    return struct.unpack("<f", struct.pack("<f", v))[0]


def _camera(rng: SeededPrng) -> dict:
    return {
        "target": (
            _f32(rng.frange(-2.0, 2.0)),
            _f32(rng.frange(-2.0, 2.0)),
            _f32(rng.frange(-2.0, 2.0)),
        ),
        "distance": _f32(rng.frange(4.0, 12.0)),
        "yaw": _f32(rng.frange(-3.0, 3.0)),
        "pitch": _f32(rng.frange(-1.2, 1.2)),
        "fov_deg": _f32(rng.frange(35.0, 70.0)),
    }


def test_generated_scenes_roundtrip_byte_identical(tmp_path: Path) -> None:
    rng = SeededPrng(SEED)
    path1 = tmp_path / "property_a.CAD_0"
    path2 = tmp_path / "property_b.CAD_0"

    for it in range(ITERATIONS):
        # --- generate -----------------------------------------------------
        scene = scene_mod.Scene()
        n_prim = 1 + rng.int_below(4)
        made = []
        for k in range(n_prim):
            kind = rng.int_below(3)
            if kind == 0:
                r = _f32(rng.frange(0.3, 1.5))
                body = sdf_mod.sphere(r)
                builder, params = "sphere", {"radius": r}
            elif kind == 1:
                ex = _f32(rng.frange(0.2, 1.0))
                ey = _f32(rng.frange(0.2, 1.0))
                ez = _f32(rng.frange(0.2, 1.0))
                body = sdf_mod.box(math_mod.Vec3f(ex, ey, ez))
                builder = "box"
                params = {"extent_x": ex, "extent_y": ey, "extent_z": ez}
            else:
                r = _f32(rng.frange(0.15, 0.5))
                h = _f32(rng.frange(0.8, 3.0))
                body = sdf_mod.cylinder(r, h)
                builder, params = "cylinder", {"radius": r, "height": h}
            name = f"{builder}{k}"
            nid = scene.add_node(name, body)
            node = scene.get_node(nid)
            node.builder = builder
            node.set_params(params)
            visible = rng.coin()
            scene.set_visible(nid, visible)
            made.append((nid, name, builder, params, visible))
        cam = _camera(rng)

        # --- save -> load -> save ----------------------------------------
        CAD_0.io.save(path1, scene, cam)
        loaded = scene_mod.Scene()
        cam1 = CAD_0.io.load(path1, loaded)
        CAD_0.io.save(path2, loaded, cam1)

        bytes1 = path1.read_bytes()
        bytes2 = path2.read_bytes()

        params_diag = {
            "n_prim": n_prim,
            "nodes": [(m[1], m[2], m[3], m[4]) for m in made],
            "camera": cam,
        }

        assert bytes1 == bytes2, case(
            SEED, it, params_diag, "save -> load -> save must be byte-identical"
        )

        # --- extraction summary identical ---------------------------------
        assert loaded.size() == len(made), case(
            SEED, it, params_diag, "loaded node count == generated node count"
        )
        assert loaded.next_id() == scene.next_id(), case(
            SEED, it, params_diag, "next_id survives the round-trip"
        )
        for m in made:
            nid, name, builder, params, visible = m
            node = loaded.get_node(nid)
            assert node is not None, case(
                SEED, it, params_diag, f"node id {nid} survives the round-trip"
            )
            assert node.name == name, case(
                SEED, it, params_diag, f"node {nid} name round-trips"
            )
            assert node.builder == builder, case(
                SEED, it, params_diag, f"node {nid} builder tag round-trips"
            )
            assert dict(node.params) == params, case(
                SEED, it, params_diag, f"node {nid} params round-trips exactly"
            )
            assert bool(node.visible) == visible, case(
                SEED, it, params_diag, f"node {nid} visibility round-trips"
            )
        for key in ("target", "distance", "yaw", "pitch", "fov_deg"):
            assert cam1[key] == cam[key], case(
                SEED, it, params_diag, f"camera.{key} round-trips exactly (float32)"
            )
