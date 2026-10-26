"""Example: the parametric hole, end-to-end — THE GUIDE CASE OF G1
(ADR-0030 D5.8, grown into ADR-0032 D7.5 + D10.6).

Sketch a circle -> extrude it -> subtract it from a box = the
parametric hole, by the book. This is the CLI face of the same recipe
the canvas draws: the example proves the document model (naming, DAG,
format, bridge), the canvas proves the interaction model, D6 asserts
both. G1 unit 6 grows it into the phase's own verification: the
printed D7.5 TABLE, analytic expected values side by side with the
REPORTED ones (the same report surface the Debug menu renders —
CAD_0.report, the single consumer of the mass API and the body
query), every row PASSING or naming its difference. Then the radius
edit: the numbers update, bit-identical to a fresh recomputation,
ONE undo entry — and the undo returns the first table's own numbers.

The whole stack in one script:
  * entity creation on a sketch node (the recipe, with names);
  * the extrude recipe (distance, direction) -> the prism operation;
  * the subtract referencing the EXTRUDE's FeatureId (genealogy:
    sketch -> extrude -> subtract), the exact chain materializing the
    HybridBody (the exact B-Rep beside the SDF, bridge tolerance 0);
  * THE D7.5 TABLE: counts 12/17/7/2, chi = 0, genus 1, the volume
    abc - pi r^2 h, the axial I_zz = abc(a^2+b^2)/12 - pi r^4 h/2,
    the provenance (all exact) — expected vs reported, row by row;
  * THE EDIT: the circle's radius through ONE undo entry (the command
    stack), the table re-printed with the updated numbers, the fresh
    recomputation agreeing bit-identically, the undo returning;
  * a v0.4 save/load round-trip with the same report (the recipe is
    the document).
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import CAD_0
from CAD_0 import math as _math
from CAD_0 import report as _report
from CAD_0 import scene as _scene

PI = 3.141592653589793

# The guide geometry (the v0.4 golden's own): the box [-1,1]^3 —
# a = b = c = 2 — with the through hole COAXIAL (the centered circle,
# the axial formula's own case) and h the pierced span (the box's
# thickness: the prism is 3 long, only the 2 inside the box is
# removed).
A = B = C = 2.0
R = 0.25
H_SPAN = 2.0
R_EDITED = 0.5


def val(body, x: float, y: float, z: float) -> float:
    return body.value(_math.Vec3f(x, y, z))


def close(a: float, b: float,
          rel: float = 1e-6, abs_tol: float = 1e-6) -> bool:
    """The precision doctrine (the suites' own): the machinery is
    closed-form double, the INPUTS are the stored float trimming
    parameters — a few float ulps is the honest bound."""
    return abs(a - b) <= max(abs_tol, rel * max(abs(a), abs(b)))


def guide_scene(radius: float):
    """box -> sketch -> circle(radius) -> extrude 3 +Z -> subtract
    (0, 0, -1.5): the guide chain, materializing the exact B-Rep."""
    s = _scene.Scene()
    box = s.add_node("Box", CAD_0.sdf.box(_math.Vec3f(1.0, 1.0, 1.0)))
    sid = s.add_sketch_node(
        "Hole profile",
        [_scene.SketchChain([
            _scene.sketch_circle((0.0, 0.0), radius, False)])])
    eid = s.add_extrude_node("Hole prism", sid, 3.0, 1)
    did = s.add_operation_node("The hole", box, eid, 0.0, 0.0, -1.5)
    r = s.rebuild_operations()
    assert r["rebuilt"] == 2 and not r["dangling"]
    return s, sid, did


def table_rows(radius: float, data: dict) -> list:
    """The D7.5 rows: (quantity, expected text, expected value,
    reported value, comparator). The expected column is the analytic
    table; the reported column is the report's own data — no third
    column exists."""
    abc = A * B * C
    removed = PI * radius * radius * H_SPAN            # pi r^2 h
    izz = (abc * (A * A + B * B) / 12.0
           - PI * radius ** 4 * H_SPAN / 2.0)          # abc(a2+b2)/12 - pi r4 h/2
    cx, cy, cz = data["centroid"]
    inertia = data["inertia"]
    n_exact, n_approx, _max_tol = _report.provenance_totals(data)
    return [
        ("V", "12", 12, data["V"], "eq"),
        ("E", "17", 17, data["E"], "eq"),
        ("F", "7", 7, data["F"], "eq"),
        ("H (inner loops)", "2", 2, data["H"], "eq"),
        ("chi = V - E + F - H", "0", 0, data["chi"], "eq"),
        ("genus", "1", 1, data["genus"], "eq"),
        ("volume = abc - pi r^2 h", "{:.9g}".format(abc - removed),
         abc - removed, data["volume"], "close"),
        ("centroid (by symmetry)", "(0, 0, 0)", (0.0, 0.0, 0.0),
         (cx, cy, cz), "close3"),
        ("I_zz = abc(a^2+b^2)/12 - pi r^4 h/2", "{:.9g}".format(izz),
         izz, inertia[2][2], "close"),
        ("provenance", "all exact ({} subshapes)".format(
            12 + 17 + 7), (True, 0), (data["all_exact"], n_approx),
         "prov"),
    ]


def print_table(radius: float, data: dict) -> bool:
    """Print the D7.5 table, side by side; every row PASSES or NAMES
    its difference. Returns True when every row passed."""
    print("\nLA TAVOLA DI D7.5 (r = {}) — analitico vs reportato:"
          .format(radius))
    print("  {:34s} {:>22s} {:>22s}  {}".format(
        "quantity", "expected", "reported", "verdict"))
    all_pass = True
    for name, exp_text, exp, rep, how in table_rows(radius, data):
        if how == "eq":
            ok = exp == rep
        elif how == "close":
            ok = close(exp, rep)
        elif how == "close3":
            # The centroid's components sit NEAR ZERO (the symmetric
            # body): relative bounds are meaningless there, so the
            # comparison is absolute at 1e-8 — a few ulps of the float
            # trimming inputs (the radius and the float-pi sweeps are
            # stored as float32, and the residue scales with them),
            # five orders under the faceted path's declared grid
            # tolerance.
            ok = all(close(e, r, 1e-6, 1e-8) for e, r in zip(exp, rep))
        else:  # prov: (all_exact, n_approximated)
            ok = (rep[0] is True) and (rep[1] == 0)
        if ok:
            verdict = "PASS"
        else:
            verdict = "DIFF: expected {}, reported {}".format(exp_text, rep)
            all_pass = False
        rep_text = rep if not isinstance(rep, float) \
            else "{:.9g}".format(rep)
        if isinstance(rep, tuple) and how == "close3":
            rep_text = "({:.2g}, {:.2g}, {:.2g})".format(*rep)
        if how == "prov":
            rep_text = ("all exact" if rep[0] else "APPROXIMATED") + \
                " ({} approximated)".format(rep[1])
        print("  {:34s} {:>22s} {:>22s}  {}".format(
            name, exp_text, str(rep_text), verdict))
    return all_pass


def main() -> None:
    # 1-3. The chain (the guide geometry).
    s, sid, did = guide_scene(R)
    hole = s.get_node(did)
    assert hole.exact_brep is not None, \
        "the guide chain must materialize the exact B-Rep"

    # The SDF twin's own check: on the hole's axis, inside the removed
    # prism, f = the radius (the negated prism wins the max).
    on_axis = val(hole.body, 0.0, 0.0, 0.0)
    print("The parametric hole (box - extruded circle), the guide case:")
    print("  on the hole's axis, inside the removed prism: "
          "f = {:.6g} (expected {}; the SDF twin agrees)".format(
              on_axis, R))
    assert on_axis == R

    # 4. THE TABLE (before the edit): the report's data IS the
    # reported column.
    data = _report.body_report_data(hole.exact_brep)
    ok_before = print_table(R, data)

    # 5. THE EDIT: the radius, ONE undo entry (the command stack —
    # the same discipline the canvas and the panel use: one completed
    # edit = one write = one undo entry).
    stack = CAD_0.commands.CommandStack(100)

    def apply_radius(rr: float) -> None:
        s.sketch_set_entity(sid, 0, 0,
                            _scene.sketch_circle((0.0, 0.0), rr, False))
        s.rebuild_operations()

    stack.push(lambda: apply_radius(R_EDITED),
               lambda: apply_radius(R),
               "Set radius = {}".format(R_EDITED))
    assert stack.undo_count() == 1, "the edit must be ONE undo entry"

    print("\nThe edit (radius {} -> {}): ONE undo entry, "
          "the numbers update —".format(R, R_EDITED))
    edited = s.get_node(did)
    assert val(edited.body, 0.0, 0.0, 0.0) == R_EDITED  # bit-identical
    data_edited = _report.body_report_data(edited.exact_brep)
    ok_edited = print_table(R_EDITED, data_edited)

    # 6. The fresh recomputation agrees BIT-IDENTICALLY (ADR-0014):
    # a new scene, the same recipe with the edited radius — the same
    # numbers, not the same objects.
    s2, _sid2, did2 = guide_scene(R_EDITED)
    data_fresh = _report.body_report_data(s2.get_node(did2).exact_brep)
    assert data_fresh["volume"] == data_edited["volume"]
    assert data_fresh["centroid"] == data_edited["centroid"]
    assert data_fresh["inertia"] == data_edited["inertia"]
    print("\n  the edit's numbers vs a fresh recomputation: "
          "bit-identical (volume {:.9g})".format(data_edited["volume"]))

    # 7. The undo: ONE entry, and the first table's numbers return
    # (bit-identically — the recipe is the source of truth).
    stack.undo()
    undone = s.get_node(did)
    data_undone = _report.body_report_data(undone.exact_brep)
    assert data_undone["volume"] == data["volume"]
    assert data_undone["inertia"] == data["inertia"]
    print("  after undo ({} entr{}): the first table's numbers return, "
          "bit-identical".format(
              stack.undo_count() + stack.redo_count(),
              "y" if stack.undo_count() + stack.redo_count() == 1 else "ies"))

    # 8. The v0.4 round-trip: save, load into a fresh scene, the same
    # report (the recipe is the document).
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sketch_hole.CAD_0"
        CAD_0.io.save(path, s)
        s3 = _scene.Scene()
        CAD_0.io.load(path, s3)
        loaded = s3.get_node(did)
        assert loaded.exact_brep is not None
        data_loaded = _report.body_report_data(loaded.exact_brep)
        assert data_loaded["volume"] == data["volume"]
        assert data_loaded["inertia"] == data["inertia"]
        print("  v0.4 round-trip: the same report, the same numbers "
              "(volume {:.9g})".format(data_loaded["volume"]))

    # 9. The report itself — the same surface the Debug menu shows.
    print("\nThe body report (the Debug menu's own surface):")
    print(_report.body_report_text(
        s.get_node(did).exact_brep,
        "exact materialization (HybridBody, bridge tolerance 0)"))

    assert ok_before and ok_edited, \
        "the D7.5 table must PASS on every row — a DIFF row is a bug"
    print("\nThe parametric hole, by the book: draw the circle on the "
          "canvas,\nextrude it, subtract it — this script is the same "
          "recipe in code,\nand the table above is the hand-check.")


if __name__ == "__main__":
    main()
