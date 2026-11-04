# bindings/python/tests/test_sketch_solver.py
#
# G4 m2 — the constraint solver's bindings surface (ADR-0040 D2, the
# H2-u6 discipline: queryable from Python BEFORE the UI consumes it).
# The deep golden suite — the eight derived numbers, the naive
# counterproof, the determinism leg, the downstream edit's quartet —
# lives in the C++ doctest file (tests/unit/sdf/test_sketch2d_solver.cpp);
# THIS file proves the surface a Python consumer drives: the
# constraints, the bill, the admission's record, the solve's three
# honest classes, the bands as the vocabulary's own row.
import pytest

import CAD_0

scene = CAD_0.scene
sketch_segment = scene.sketch_segment
sketch_circle = scene.sketch_circle
SketchChain = scene.SketchChain

ER = scene.SketchEntityRef
PR = scene.SketchPointRef
PAR = scene.SketchParamRef

END, START = True, False


def drawn_triangle():
    """Golden 1's drawn point (ADR-0040 D5.1): the gesture's landing,
    the joints' copies slightly off."""
    return [SketchChain([
        sketch_segment((0.10, 0.05), (4.15, -0.05), True),
        sketch_segment((4.10, 0.02), (0.05, 3.10), True),
        sketch_segment((0.08, 3.15), (0.12, -0.02), True),
    ])]


def triangle_constraints(ca):
    e0, e1, e2 = ER(0, 0), ER(0, 1), ER(0, 2)
    return [
        scene.constraint_coincidence(PR(e0, END), PR(e1, START)),
        scene.constraint_coincidence(PR(e1, END), PR(e2, START)),
        scene.constraint_coincidence(PR(e2, END), PR(e0, START)),
        scene.constraint_length(e0, 4.0),
        scene.constraint_length(e1, 5.0),
        scene.constraint_length(e2, ca),
        scene.constraint_pin(PAR(e0, 0), 0.0),
        scene.constraint_pin(PAR(e0, 1), 0.0),
        scene.constraint_pin(PAR(e0, 3), 0.0),
    ]


def admit_triangle(m, ca=3.0):
    for c in triangle_constraints(ca):
        adm = m.add_constraint(c)
        assert adm.admitted, adm.message


def seg_a(dump, chain, i):
    return dump[chain][i]["a"]


def seg_b(dump, chain, i):
    return dump[chain][i]["b"]


def test_the_bands_are_the_vocabulary_row():
    # ADR-0035 D2's SOLVE members, re-homed from the sketch kernel's
    # own constants: one value per world, two consumers, zero drift.
    assert scene.eps_solve_lin == pytest.approx(1e-5)
    assert scene.eps_solve_ang == pytest.approx(1e-6)


def test_the_bill_and_the_solve_golden_1():
    m = scene.SketchSolveModel(drawn_triangle())
    bill = m.bill()
    assert (bill.dof, bill.consumed, bill.free(), bill.full()) == (12, 0, 12, False)
    admit_triangle(m)
    bill = m.bill()
    assert (bill.dof, bill.consumed, bill.free(), bill.full()) == (12, 12, 0, True)
    assert bill.describe() == "12 - 12 = 0 (FULL)"

    r = m.solve()
    assert r.status == scene.SketchSolveStatus.solved
    assert r.iterations == 4  # D5.1's pinned count
    assert len(m.constraints()) == 9  # append-only, authored order
    # the solution, coordinate for coordinate (the 3-4-5 identity,
    # machine-exact through the store)
    dump = m.chains_dump()
    a, b = seg_a(dump, 0, 0), seg_b(dump, 0, 0)
    c = seg_b(dump, 0, 1)
    assert a[0] == pytest.approx(0.0, abs=1e-9)
    assert a[1] == pytest.approx(0.0, abs=1e-9)
    assert b[0] == pytest.approx(4.0, abs=1e-9)
    assert b[1] == pytest.approx(0.0, abs=1e-9)
    assert c[0] == pytest.approx(0.0, abs=1e-9)
    assert c[1] == pytest.approx(3.0, abs=1e-9)


def test_over_constrained_refused_with_the_culprit():
    m = scene.SketchSolveModel(drawn_triangle())
    admit_triangle(m)
    adm = m.add_constraint(
        scene.constraint_distance(PR(ER(0, 0), END), PR(ER(0, 1), END), 5.0))
    assert not adm.admitted
    assert (adm.rank_before, adm.rank_after, adm.consumption) == (12, 12, 1)
    # the culprit named, the bill, the remedy — the class's payload
    assert "sketch_over_constrained" in adm.message
    assert "the distance on (s0.end, s1.end) = 5" in adm.message
    assert "12 - 12 = 0 (FULL)" in adm.message
    assert "Remedy" in adm.message
    # the 13th never entered the list
    assert len(m.constraints()) == 9


def test_not_solvable_with_the_diagnosis():
    m = scene.SketchSolveModel(drawn_triangle())
    admit_triangle(m, ca=10.0)  # 4 + 5 < 10
    r = m.solve()
    assert r.status == scene.SketchSolveStatus.not_solvable
    assert r.iterations == 35  # D5.6's pinned count
    assert r.residual_norm == pytest.approx(0.408248, abs=1e-4)
    assert "4+5 < 10" in r.diagnosis
    assert "no triangle with these sides exists" in r.diagnosis
    # the bounded exit: the flat chain's closest approach
    assert r.exit_params[2] == pytest.approx(4.1667, abs=1e-3)
    assert r.exit_params[6] == pytest.approx(9.5, abs=1e-3)
    # the residual per constraint, the largest named
    assert len(r.violations) == 12
    big = r.violations[r.largest]
    assert abs(big.value) == pytest.approx(0.166667, abs=1e-3)
    # the recipe untouched: the drawn values stand (the recipe's
    # own float32 resolution — the store's quantizer, ADR-0014)
    dump = m.chains_dump()
    assert seg_a(dump, 0, 0) == pytest.approx((0.10, 0.05), abs=1e-7)
    assert seg_b(dump, 0, 0) == pytest.approx((4.15, -0.05), abs=1e-7)


def test_degenerate_the_kernel_words_verbatim():
    chains = [SketchChain([sketch_segment((1.0, 0.0), (5.0, 0.0), True)])]
    m = scene.SketchSolveModel(chains)
    adm = m.add_constraint(
        scene.constraint_coincidence(PR(ER(0, 0), START), PR(ER(0, 0), END)))
    assert adm.admitted
    r = m.solve()
    assert r.status == scene.SketchSolveStatus.degenerate
    assert r.iterations == 3  # D5.7's pinned count
    # the payload is the kernel's own class VERBATIM
    assert "a degenerate segment has no normal to orient" in r.degenerate_message
    assert "give the segment a non-zero extent" in r.degenerate_message
    assert r.degenerate_entity == "chain 0, entity 0"
    # the recipe untouched (the refusal precedes the write)
    dump = m.chains_dump()
    assert dump[0][0]["a"] == pytest.approx((1.0, 0.0), abs=1e-7)
    assert dump[0][0]["b"] == pytest.approx((5.0, 0.0), abs=1e-7)


def test_the_tangent_case_bit_and_the_under_constrained_report():
    chains = [SketchChain([
        sketch_segment((2.20, 0.10), (1.90, 5.10), True),
        sketch_circle((-0.05, 2.45), 2.10, True),
    ])]
    bit = scene.derive_tangent_case_bit(chains, ER(0, 0), ER(0, 1))
    assert bit == 1  # ESTERNO: the center on the left-normal side
    m = scene.SketchSolveModel(chains)
    assert m.add_constraint(scene.constraint_tangent(ER(0, 0), ER(0, 1), bit)).admitted
    assert m.add_constraint(scene.constraint_radius(ER(0, 1), 2.0)).admitted
    bill = m.bill()
    assert (bill.dof, bill.consumed, bill.free()) == (7, 2, 5)
    r = m.solve()
    assert r.status == scene.SketchSolveStatus.solved
    assert r.iterations == 19  # D5.2's pinned count
    assert r.bill.free() == 5  # the under-constrained REPORTED


def test_the_value_rules_refuse_with_the_remedies():
    with pytest.raises(Exception):
        scene.constraint_distance(PR(ER(0, 0), END), PR(ER(0, 1), END), 0.0)
    with pytest.raises(Exception):
        scene.constraint_length(ER(0, 0), 0.0)
    with pytest.raises(Exception):
        scene.constraint_radius(ER(0, 1), -1.0)
    with pytest.raises(Exception):
        scene.constraint_tangent(ER(0, 0), ER(0, 1), 0)  # the case bit is ±1


def test_the_edit_family_revalue_and_remove():
    m = scene.SketchSolveModel(drawn_triangle())
    admit_triangle(m)
    assert m.solve().status == scene.SketchSolveStatus.solved
    # the dimension edit: |BC| 5 -> 5.5 (the lengths are indices 3..5)
    assert m.revalue_constraint(4, 5.5)
    r = m.solve()
    assert r.status == scene.SketchSolveStatus.solved
    assert r.iterations == 5  # D5.8's pinned count
    dump = m.chains_dump()
    c = seg_b(dump, 0, 1)
    assert c[0] == pytest.approx(-0.65625, abs=1e-6)
    assert c[1] == pytest.approx(2.927342811749249, abs=1e-6)
    # the remove: the bill returns
    assert m.remove_constraint(0)  # one coincidence
    bill = m.bill()
    assert (bill.consumed, bill.free()) == (10, 2)
    assert not m.remove_constraint(99)  # out of range: honestly false


def test_the_empty_constraint_sketch_solves():
    # The additivity clause's extreme: a sketch with NO constraints
    # (every pre-solver sketch in every file) SOLVES — the
    # propagation world survives the landing day (D2.1's own law).
    m = scene.SketchSolveModel(drawn_triangle())
    r = m.solve()
    assert r.status == scene.SketchSolveStatus.solved
    assert r.bill.free() == 12
    dump = m.chains_dump()
    assert seg_a(dump, 0, 0) == pytest.approx((0.10, 0.05), abs=1e-7)  # nothing moved
