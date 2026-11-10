"""K1 m5 — THE CHAIN GATE THROUGH THE REPORT'S OWN DOOR
(ADR-0041 D1.3): the gate callable from the report surface — the
SAME door the C++ suite consumes (CAD_0/brep/chain_gate.hpp, one
module), exposed on CAD_0.brep as chain_gate / chain_digest. The
C++ suite pins the whole contract (tests/unit/brep/
test_k1_m5_chain_gate.cpp — the baseline chain-5 with the gate
after every step, the tavola-bearing census with the D3 classes,
the drift against the re-run, the five corruptions each caught
with its named message). What THESE tests pin is what the report
reads:

  * the chain-5's every body step gated through the binding: the
    counts, H, chi — the plain values the report renders;
  * the honest not_applicable (the census and the drift read the
    step's tavola and the re-run's digest — the body-alone
    evidence does not reach them, said, never hidden);
  * the digest: deterministic across two builds of the same
    recipe (the report's own drift floor), the summary line
    carrying the body's numbers.

Requires only the compiled extension (no numpy, no PySide6).
"""

from __future__ import annotations

import math

import pytest

import CAD_0
from CAD_0 import brep as brep_mod

Vec3f = CAD_0.math.Vec3f

RIM = "sub#4:isect[box#1:top × ext#3:lat(0)]@hi"


def _chain5(radius=0.25):
    """The guide chain's bodies: box#1 -> ext#3 -> sub#4 -> fil#5
    (the sketch's chains beside, the report's own route)."""
    box = brep_mod.make_box(Vec3f(1.0, 1.0, 1.0), feature=1)
    chains = [CAD_0.scene.SketchChain(
        [CAD_0.scene.sketch_circle((0.0, 0.0), 0.5, False)])]
    tool = brep_mod.make_prism(chains, 2.0, +1, feature=3)
    blind = brep_mod.exact_subtract(
        box, tool, Vec3f(0.0, 0.0, -0.5),
        target=brep_mod.FeatureKind.Box,
        tool=brep_mod.FeatureKind.Extrude, feature=4)
    rim = None
    for i in range(brep_mod.edge_count(blind)):
        e = brep_mod.edge_query(blind, i)
        if e["name"] == RIM:
            sid = e["shape_id"]
            from CAD_0 import geometry as geom_mod
            rim = geom_mod.ShapeId(
                geom_mod.FeatureId(sid["feature"]), sid["path"])
            break
    assert rim is not None, "the pierced rim was not found by name"
    result = brep_mod.fillet_rim_edge(
        blind, rim, radius, feature=5,
        body_kind=brep_mod.FeatureKind.Box,
        far_feature=3, far_body_kind=brep_mod.FeatureKind.Extrude)
    return box, tool, blind, result


def test_the_gate_on_every_body_step():
    """The chain-5 gated through the report's door: pass, the
    counts, H, chi — and the honest n/a for the census and the
    drift (this door's evidence is the body alone)."""
    box, tool, blind, result = _chain5()

    for body, (v, e, f, h, chi) in (
            (box, (8, 12, 6, 0, 2)),
            (tool, (4, 5, 3, 0, 2)),
            (blind, (12, 17, 8, 1, 2)),
            (result, (14, 20, 9, 1, 2))):
        g = brep_mod.chain_gate(body)
        assert g["pass"] is True
        assert g["failures"] == []
        assert (g["v"], g["e"], g["f"], g["h"], g["chi"]) == \
            (v, e, f, h, chi)
        # The honest n/a: the census and the drift read evidence
        # this door does not hold — SAID, never silently skipped.
        assert sorted(g["not_applicable"]) == ["census", "drift"]


def test_the_digest_deterministic_across_two_builds():
    """The report's own drift floor: the same recipe built twice,
    every body's digest bit-identical (ADR-0014's N-step clause at
    the report's door)."""
    a = _chain5()
    b = _chain5()
    for body_a, body_b in zip(a, b):
        da = brep_mod.chain_digest(body_a)
        db = brep_mod.chain_digest(body_b)
        assert da == db
        assert da  # never empty
    # The summary line carries the body's own numbers.
    line = brep_mod.chain_digest(a[2]).splitlines()[0]
    assert line == "body V=12 E=17 F=8 H=1 chi=2"


def test_the_edit_visible_exactly_where_it_acts():
    """The discrimination the report can make with the digest
    alone: the radius edit (0.25 -> 0.125) leaves the upstream
    steps' digests UNCHANGED and changes the edited step's — an
    edit is not a drift, and the digest tells them apart."""
    original = _chain5(0.25)
    edited = _chain5(0.125)
    # The upstream steps: bit-identical.
    for body_o, body_e in zip(original[:3], edited[:3]):
        assert brep_mod.chain_digest(body_o) == \
            brep_mod.chain_digest(body_e)
    # The edited step: the digest moves, the counts stay fixed.
    assert brep_mod.chain_digest(original[3]) != \
        brep_mod.chain_digest(edited[3])
    for body in (original[3], edited[3]):
        g = brep_mod.chain_gate(body)
        assert g["pass"] is True
        assert (g["v"], g["e"], g["f"], g["h"], g["chi"]) == \
            (14, 20, 9, 1, 2)
