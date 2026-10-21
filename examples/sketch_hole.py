"""Example: the parametric hole, end-to-end (ADR-0030 D5.8).

Sketch a circle -> extrude it -> subtract it from a box = the
parametric hole, by the book. This is the CLI face of the same recipe
the canvas draws: the example proves the document model (naming, DAG,
format, bridge), the canvas proves the interaction model, D6 asserts
both.

The whole stack in one script:
  * entity creation on a sketch node (the recipe, with names);
  * the extrude recipe (distance, direction) -> the prism operation;
  * the subtract referencing the EXTRUDE's FeatureId (genealogy:
    sketch -> extrude -> subtract);
  * an edit of the circle's radius propagating bit-identically
    through the chain (the DAG pass re-materializes extrude and
    subtract from the recipe alone);
  * a v0.4 save/load round-trip with the same report;
  * the genus-1 extraction report of the pierced box.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import CAD_0
from CAD_0 import math as _math
from CAD_0 import scene as _scene


def val(body, x: float, y: float, z: float) -> float:
    return body.value(_math.Vec3f(x, y, z))


def report(body) -> str:
    """The Debug -> Extract SDF -> B-Rep report (the same surface the
    UI's Debug menu uses)."""
    bridge = CAD_0.bridge
    opts = bridge.SdfToBrepOptions()
    rep = bridge.sdf_to_brep_report(body, opts)
    lines = [
        "Path: {}".format(rep["fit"]),
        "V = {}, E = {}, F = {}".format(rep["V"], rep["E"], rep["F"]),
        "chi = V - E + F = {}".format(rep["chi"]),
        "Genus = {}".format(rep["genus"]),
        "Closed orientable: {}".format(rep["orientable"]),
    ]
    return "\n".join(lines)


def main() -> None:
    s = _scene.Scene()

    # 1. The sketch: one circle, OUTER by the declared default (the
    #    recipe's names: center, radius, hole — D4's serialized form).
    circle = _scene.sketch_circle((0.0, 0.0), 0.35, False)
    sketch_id = s.add_sketch_node("Hole profile",
                                  [_scene.SketchChain([circle])])
    assert s.sketch_closed(sketch_id)  # a circle closes itself

    # 2. The extrude recipe: distance 3.0, +Z. The prism enters the
    #    tree as an OPERATION node referencing the sketch's FeatureId.
    prism_id = s.add_extrude_node("Hole prism", sketch_id, 3.0, 1)
    prism = s.get_node(prism_id)
    assert prism.operation["type"] == "extrude"
    assert prism.operation["sketch"] == sketch_id

    # 3. The subtract: box minus prism — the tool is the EXTRUDE's
    #    FeatureId (the genealogy the canvas's use case consumes).
    box_id = s.add_node("Box", CAD_0.sdf.box(_math.Vec3f(1.2, 1.2, 1.2)))
    hole_id = s.add_operation_node("The hole", box_id, prism_id,
                                   0.3, 0.0, -1.5)
    result = s.rebuild_operations()
    assert result["rebuilt"] == 2 and not result["dangling"]

    body = s.get_node(hole_id).body
    print("The parametric hole (box - extruded circle):")
    print("  on the hole's axis, inside the removed prism: "
          "f = {:.6g}".format(val(body, 0.3, 0.0, 0.0)))
    print("  (expected +0.35: the negated prism depth — the hole)")

    # 4. THE EDIT: the circle's radius (a recipe parameter, by name).
    #    The DAG pass re-materializes extrude and subtract from the
    #    recipe alone — the same state, the same prism, bit-identical
    #    (via-agnostic: gesture, text or code reach the same bytes).
    s.sketch_set_entity(sketch_id, 0, 0,
                        _scene.sketch_circle((0.0, 0.0), 0.5, False))
    s.rebuild_operations()
    after_edit = val(s.get_node(hole_id).body, 0.3, 0.0, 0.0)
    print("  after the radius edit (0.35 -> 0.5): "
          "f = {:.6g}".format(after_edit))
    assert after_edit == 0.5  # bit-identical propagation

    # 5. The v0.4 round-trip: save, load into a fresh scene, the same
    #    report (the recipe is the document).
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sketch_hole.CAD_0"
        CAD_0.io.save(path, s)
        s2 = _scene.Scene()
        CAD_0.io.load(path, s2)
        loaded = val(s2.get_node(hole_id).body, 0.3, 0.0, 0.0)
        assert s2.sketch_chains(s2.get_node(prism_id).operation["sketch"])
        assert loaded == after_edit  # the same bytes out
        print("  v0.4 round-trip: f = {:.6g} (identical)".format(loaded))

    # 6. The genus-1 extraction report of the pierced box.
    print("\nSDF -> B-Rep extraction report (the pierced box):")
    print(report(s.get_node(hole_id).body))
    print("\nThe parametric hole, by the book: draw the circle on the "
          "canvas,\nextrude it, subtract it — this script is the same "
          "recipe in code.")


if __name__ == "__main__":
    main()
