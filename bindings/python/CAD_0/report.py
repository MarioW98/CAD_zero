"""CAD_0.report — the body report (G1 unit 6, ADR-0032 D8 + D10.6).

THE consumer of the mass-properties API (unit 5) and the body query
(unit 6, aggregating the unit-2 provenance fields): the Debug report
in the UI, the guide-case example and the property panel's read-only
rows all render through this module — there is no second opinion, and
never a number recalculated elsewhere. Every value below is
``CAD_0.brep.mass_properties``'s or ``CAD_0.brep.body_query``'s own
answer, verbatim.

Kernel-only (no PySide6, no numpy): the example runs it headless, the
UI runs it beside the tree. Errors keep the P2 culture: a mass
refusal (the empty body, a non-closed shell, an out-of-class face, a
degenerate volume) is REPORTED as a visible ``mass_error`` line with
the kernel's own diagnosis and remedy — never a wrong number, never a
silent gap.

Determinism (ADR-0014): both APIs are closed-form and
order-stable, so ``body_report_data`` is bit-identical across calls
and processes — the report the example prints is the report the tests
pin, is the report the UI shows.
"""

from __future__ import annotations

from typing import Any, Optional


def _brep_module():
    """The brep submodule — the app package re-exports it; the bare
    extension keeps it on _CAD_0 (the main_window fallback pattern)."""
    import CAD_0
    mod = getattr(CAD_0, "brep", None)
    if mod is None:
        mod = CAD_0._CAD_0.brep
    return mod


def body_report_data(brep_body: Any, tol: float = 1e-6) -> dict:
    """The report's data, as plain values.

    The topology and provenance keys mirror ``CAD_0.brep.body_query``
    verbatim (V, E, F, H, chi, orientable, loops_closed, genus — None
    when the ADR-0027 D6 rule does not derive it — and the per-kind
    tallies); the mass keys mirror ``CAD_0.brep.mass_properties``
    verbatim (volume, centroid, inertia_about_centroid, all_exact).
    When the mass API refuses, ``mass_error`` carries the kernel's
    diagnosis verbatim and the mass keys are None — the honesty
    contract, not a gap.

    Deterministic: the same body answers the same dict, bit-identical.
    """
    brep = _brep_module()
    q = brep.body_query(brep_body)

    out: dict = {
        "V": q["V"], "E": q["E"], "F": q["F"], "H": q["H"],
        "chi": q["chi"],
        "orientable": q["orientable"],
        "loops_closed": q["loops_closed"],
        "genus": q["genus"],  # None when not derived (multi-shell n/d)
        "provenance": {
            "vertices": q["vertices"],
            "edges": q["edges"],
            "faces": q["faces"],
        },
        "volume": None,
        "centroid": None,
        "inertia": None,
        "all_exact": None,
        "mass_error": None,
    }

    try:
        mp = brep.mass_properties(brep_body, tol)
    except (ValueError, RuntimeError) as exc:
        # The P2 pattern, visible: the refusal IS the answer — the
        # diagnosis and remedy are already inside the message.
        out["mass_error"] = str(exc)
        return out

    out["volume"] = mp["volume"]
    out["centroid"] = mp["centroid"]
    out["inertia"] = mp["inertia_about_centroid"]
    out["all_exact"] = mp["all_exact"]
    return out


def provenance_totals(data: dict) -> tuple:
    """The D8 aggregates from a body_report_data dict: (n_exact,
    n_approximated, max_linear_tolerance) over ALL subshape kinds."""
    n_exact = 0
    n_approx = 0
    max_tol = 0.0
    for kind in ("vertices", "edges", "faces"):
        t = data["provenance"][kind]
        n_exact += t["exact"]
        n_approx += t["approximated"]
        max_tol = max(max_tol, t["max_linear_tolerance"])
    return n_exact, n_approx, max_tol


def face_table_text(brep_body: Any) -> str:
    """The per-face table with the NAME column (ADR-0036 D4, morso 2).

    One row per face, in address order: the address, the name (the
    canonical render — "- " when honestly unnamed: the v1 named
    classes are the three factories), the surface family, the loop
    counts, and the Layer-2 provenance. Every value is
    ``CAD_0.brep.face_query``'s own answer, verbatim — pure string
    building beside ``body_report_text``, which does not move (the
    existing report's goldens are untouched; this is a NEW surface).

    Deterministic (ADR-0014): the names carry no floating-point
    content, so the table is bit-identical across calls, processes
    and platforms.
    """
    brep = _brep_module()
    n = brep.face_count(brep_body)
    lines: list = ["Faces (address | name | family | loops | provenance):"]
    for i in range(n):
        q = brep.face_query(brep_body, i)
        name = q["name"] if q["name"] else "-"
        loops = "{} loop(s) ({} outer, {} inner)".format(
            q["loop_count"], q["outer_loop_count"], q["inner_loop_count"])
        prov = "exact" if q["exact"] else "approx({:g})".format(
            q["linear_tolerance"])
        lines.append("  {} | {} | {} | {} | {}".format(
            i, name, q["family"], loops, prov))
    return "\n".join(lines)


def _fmt(x: float) -> str:
    """Readable, not a wall: 9 significant digits (the machinery's
    double precision, without the float-noise tail)."""
    return format(float(x), ".9g")


def body_report_text(brep_body: Any,
                     path_label: Optional[str] = None,
                     tol: float = 1e-6) -> str:
    """The rendered report — what the Debug menu shows and the example
    prints. Pure string building over ``body_report_data``; the caller
    may prepend a ``path_label`` line (how the body was obtained: the
    exact materialization, a primitive fit, a faceted extraction)."""
    d = body_report_data(brep_body, tol)

    lines: list = []
    if path_label is not None:
        lines.append(f"Path: {path_label}")

    # The structural block, as always — with the H term in the label
    # (honest since unit 1 extended the formula: the guide case's
    # pierced faces make the term visible, the arithmetic sits beside
    # it for the hand-check).
    v, e, f, h = d["V"], d["E"], d["F"], d["H"]
    lines.append(f"V = {v}, E = {e}, F = {f}, H (inner loops) = {h}")
    lines.append(f"\u03c7 = V - E + F - H = {v} - {e} + {f} - {h} = {d['chi']}")
    if d["genus"] is None:
        lines.append("Genus = n/d (multi-shell? chi > 2 \u2014 components "
                     "counter deferred)")
    else:
        lines.append(f"Genus = {d['genus']}")
    lines.append("Closed orientable (is_closed_orientable): "
                 + ("yes" if d["orientable"] else "NO"))
    lines.append("Loops (loops_are_closed): "
                 + ("yes" if d["loops_closed"] else "NO"))

    # The mass block (unit 5's API, verbatim): a refusal is a visible
    # line, never a gap.
    if d["mass_error"] is not None:
        lines.append(f"Mass properties: n/d \u2014 {d['mass_error']}")
    else:
        flag = ("exact (all_exact = True)" if d["all_exact"]
                else "APPROXIMATED (all_exact = False \u2014 the "
                     "polyhedral value, never mixed)")
        cx, cy, cz = d["centroid"]
        inertia = d["inertia"]
        ixx, iyy, izz = inertia[0][0], inertia[1][1], inertia[2][2]
        ixy, ixz, iyz = inertia[0][1], inertia[0][2], inertia[1][2]
        lines.append(f"Mass properties (density 1, divergence theorem): "
                     f"{flag}")
        lines.append(f"  Volume = {_fmt(d['volume'])}")
        lines.append(f"  Centroid = ({_fmt(cx)}, {_fmt(cy)}, {_fmt(cz)})")
        lines.append("  Inertia about centroid:")
        lines.append(f"    diagonal  Ixx = {_fmt(ixx)}  Iyy = {_fmt(iyy)}"
                     f"  Izz = {_fmt(izz)}")
        lines.append(f"    products  Ixy = {_fmt(ixy)}  Ixz = {_fmt(ixz)}"
                     f"  Iyz = {_fmt(iyz)}")

    # The provenance block (D8): "is this edge exact?" answered per
    # body, per kind, with the max declared tolerance.
    n_exact, n_approx, max_tol = provenance_totals(d)
    lines.append(f"Provenance (Layer 2): {n_exact} subshapes exact / "
                 f"{n_approx} approximated, max tolerance "
                 f"{_fmt(max_tol)}")
    for kind in ("vertices", "edges", "faces"):
        t = d["provenance"][kind]
        lines.append(f"  {kind:9s} {t['exact']} exact / "
                     f"{t['approximated']} approximated")

    return "\n".join(lines)
