"""K1 m6 — THE SURVIVAL LEDGER THROUGH THE REPORT'S OWN DOOR
(ADR-0041 D1.4): the ledger callable from the report surface — the
SAME door the C++ suite consumes (CAD_0/brep/survival_ledger.hpp,
one module), exposed on CAD_0.brep as survival_ledger. The C++ suite
pins the whole contract (tests/unit/brep/test_k1_m6_survival_ledger.
cpp — the baseline with the two pinned references, the five-member
corpus with the classes and the deaths named, the anonymous-death
refusal, the machinery's classes 3/4, the determinism with the FNV
pin). What THESE tests pin is what the report reads:

  * the member-1 walk through the door: the silent radius edit (the
    chain re-run at r=0.125) — both references reference_resolved,
    the rim's death at the MEAL registered with the killer (fil#5),
    the cause verbatim ("replaced by the flank") and the step;
  * the fraction WITH the register: the pair (resolved, total) AND
    the deaths' register in the same verdict — the metric's own law
    ("a survival number without the death ledger is a lie of
    omission"), readable riga per riga at this door;
  * the discipline: a death without its killer declared is REFUSED
    (ValueError, the diagnosis naming the anonymous death) — the
    report can never render a fraction over an anonymous death.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import pytest

import CAD_0
from CAD_0 import brep as brep_mod
from CAD_0 import geometry as geom_mod
from CAD_0 import scene as scene_mod

Vec3f = CAD_0.math.Vec3f

RIM = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"


def _chain5(radius=0.25):
    """The guide chain's bodies: box#1 -> ext#3 -> sub#4 -> fil#5
    (the m5 test's own route — the baseline's chain)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    tool = brep_mod.make_prism(chains, 2.0, +1, feature=3)
    blind = brep_mod.exact_subtract(
        box, tool, Vec3f(0.0, 0.0, -0.5),
        target=brep_mod.FeatureKind.Box,
        tool=brep_mod.FeatureKind.Extrude, feature=4)
    rim = None
    v2 = None
    for i in range(brep_mod.edge_count(blind)):
        e = brep_mod.edge_query(blind, i)
        if e["name"] == RIM:
            sid = e["shape_id"]
            rim = geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
        if e["name"] == "box#1:vertical2":
            sid = e["shape_id"]
            v2 = geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
    assert rim is not None, "the pierced rim was not found by name"
    assert v2 is not None, "vertical2 was not found by name"
    result = brep_mod.fillet_rim_edge(
        blind, rim, radius, feature=5,
        body_kind=brep_mod.FeatureKind.Box,
        far_feature=3, far_body_kind=brep_mod.FeatureKind.Extrude)
    return box, tool, blind, result, rim, v2


def _member1_evidence(killer="fil#5"):
    """The member-1 evidence: the silent radius edit (r 0.25 ->
    0.125) and the fillet's meal — the caller holds the chains, the
    tavole and the edit contexts; the door consumes the declarations."""
    box_o, tool_o, blind_o, _result_o, rim, v2 = _chain5(0.25)
    _box_e, _tool_e, blind_e, result_e, _rim_e, _v2_e = _chain5(0.125)

    sub_tav = brep_mod.subtract_descent(blind_o, box_o, tool_o)
    fil_tav = brep_mod.fillet_descent(result_e, blind_e)

    member = {
        "name": "the silent radius edit: fil#5 at r=0.125",
        "kind": "silent_radius_edit",
        "digest": brep_mod.chain_digest(result_e),
        "references": [
            {"name": "box#1:vertical2", "id": v2,
             "kind": brep_mod.FeatureKind.Box},
            {"name": RIM, "id": rim,
             "kind": brep_mod.FeatureKind.Subtract,
             "operands": [(1, brep_mod.FeatureKind.Box),
                          (3, brep_mod.FeatureKind.Extrude)]},
        ],
        "steps": [
            {"kind": "edit",
             "name": "sub#4 re-materialized (the edit touches "
                     "fil#5 alone)",
             "refs": [
                 {"reference": 0, "old_tavola": sub_tav,
                  "new_body": blind_e,
                  "edit": brep_mod.EditContext(no_shift=True)},
                 {"reference": 1, "old_tavola": sub_tav,
                  "new_body": blind_e,
                  "edit": brep_mod.EditContext(no_shift=True)},
             ]},
            {"kind": "meal",
             "name": "fil#5 at r=0.125",
             "killer": killer,
             "refs": [
                 {"reference": 0, "old_tavola": fil_tav,
                  "new_body": result_e},
                 {"reference": 1, "old_tavola": fil_tav,
                  "new_body": result_e},
             ]},
        ],
    }
    return member


def test_the_member_through_the_door():
    """The member-1 walk: both references reference_resolved, the
    rim's death at the meal registered with the killer, the cause
    verbatim and the step — the edit's class and the feature's act,
    each in its own place."""
    ledger = brep_mod.survival_ledger([_member1_evidence()])

    assert ledger["total"] == 2
    assert ledger["resolved"] == 2
    assert ledger["renamed"] == 0
    m = ledger["members"][0]
    assert m["kind"] == "silent_radius_edit"
    assert m["entries"][0]["code"] == "reference_resolved"
    assert m["entries"][0]["dead"] is False
    assert m["entries"][1]["code"] == "reference_resolved"
    # ...and the death at the MEAL: the class untouched (the edit's
    # own answer), the death on the entry beside it.
    assert m["entries"][1]["dead"] is True
    assert m["entries"][1]["killer"] == "fil#5"
    assert m["entries"][1]["cause"] == "replaced by the flank"
    assert m["entries"][1]["step"] == "fil#5 at r=0.125"
    assert m["entries"][1]["died"] == RIM


def test_the_fraction_with_the_register():
    """The metric's own law at the report's door: the fraction (a
    PAIR, never a naked number) AND the deaths' register in the SAME
    verdict — readable riga per riga, both or neither."""
    ledger = brep_mod.survival_ledger([_member1_evidence()])

    assert ledger["fraction"] == (2, 2)
    deaths = ledger["deaths"]
    assert len(deaths) == 1
    d = deaths[0]
    assert d["reference"] == RIM
    assert d["killer"] == "fil#5"
    assert d["cause"] == "replaced by the flank"
    # The register as text, one line per death — the report's render.
    text = ledger["deaths_text"]
    assert text.count("\n") == 1
    assert "fil#5" in text
    assert "replaced by the flank" in text
    assert RIM in text
    # The whole ledger's text carries both: the members' rows and
    # the fraction's own pair.
    whole = ledger["text"]
    assert "survival fraction: 2/2" in whole
    assert "deaths: 1" in whole
    assert "reference_resolved" in whole


def test_the_anonymous_death_refused():
    """The discipline at the report's door: a death without its
    killer declared is REFUSED — ValueError, the diagnosis naming
    the anonymous death and the remedy. The report can never render
    a fraction over an anonymous death."""
    evidence = _member1_evidence(killer="")  # the killer withheld
    with pytest.raises(ValueError, match="a death without its name"):
        brep_mod.survival_ledger([evidence])
    # The control: the SAME evidence with the killer whole registers
    # (the refusal is the anonymity's own, never the scenario's).
    ledger = brep_mod.survival_ledger([_member1_evidence()])
    assert ledger["fraction"] == (2, 2)
    assert len(ledger["deaths"]) == 1


def test_the_determinism_at_the_door():
    """The same evidence, twice: the door's text identical (the
    corpus's own determinism clause, ADR-0014's trivial case at the
    report's surface)."""
    a = brep_mod.survival_ledger([_member1_evidence()])
    b = brep_mod.survival_ledger([_member1_evidence()])
    assert a["text"] == b["text"]
    assert a["deaths_text"] == b["deaths_text"]
    assert a["fraction"] == b["fraction"]
