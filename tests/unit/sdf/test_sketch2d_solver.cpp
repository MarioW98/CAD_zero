// tests/unit/sdf/test_sketch2d_solver.cpp
//
// G4 m2 — THE EIGHT GOLDENS of ADR-0040 D5, each landing green by
// itself with the derived numbers asserted at the bands' margins.
// The pre-registration is scripts/adr0040_derivation.py (its output
// measured again on this tree, not repeated by rote): the DoF
// counts, the consumptions (the numerical ranks), the bills, the
// iteration counts, the bands' margins — every number below was
// DERIVED before the code existed, and the code reproduces it.
//
// RED-FIRST (the protocol of always, ADR-0034 D4/0035 D5): on the
// pre-m2 tree this file does not COMPILE — the constraint API does
// not exist there; the red is the absence itself, the file's own
// compile naming it. The NAIVE COUNTERPROOF (D5.4) demonstrates the
// red WITHIN the green file: the same length assertions that pass
// through the solver FAIL on the naive's frozen geometry — the
// discrimination, measured.
//
// The golden declaration (pre-registered, D5): no existing golden
// moves (the solver is additive — the parametric hole's own tests
// stand untouched); the eight are NEW oracles; determinism from the
// first landing (no prior solve state to move).
#include <doctest/doctest.h>

#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_solver.hpp"

// D5.3/D5.8's materialization legs: the existing pinned formula
// (V = 2n, E = 3n, F = n+2, χ = 2) consumed exactly as the ADR
// declares — the solved recipe materializes like today's authored
// one (the parallel discipline's promise, kept by construction).
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace CAD_0::sdf;
namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
using CAD_0::geometry::FeatureId;

namespace {

constexpr SketchEnd kS = SketchEnd::Start;
constexpr SketchEnd kE = SketchEnd::End;

// Golden 1's drawn point, verbatim from the derivation: the
// gesture's landing, the joints' copies slightly off.
std::vector<SketchChain> drawn_triangle() {
    return {SketchChain{{
        sketch_segment({0.10f, 0.05f}, {4.15f, -0.05f}, true),
        sketch_segment({4.10f, 0.02f}, {0.05f, 3.10f}, true),
        sketch_segment({0.08f, 3.15f}, {0.12f, -0.02f}, true),
    }}};
}

void admit_triangle(SketchSolveModel& m, double ca) {
    for (const SketchConstraint& c : {
             constraint_coincidence({{0, 0}, kE}, {{0, 1}, kS}),
             constraint_coincidence({{0, 1}, kE}, {{0, 2}, kS}),
             constraint_coincidence({{0, 2}, kE}, {{0, 0}, kS}),
             constraint_length({0, 0}, 4.0),
             constraint_length({0, 1}, 5.0),
             constraint_length({0, 2}, ca),
             constraint_pin({{0, 0}, 0}, 0.0),
             constraint_pin({{0, 0}, 1}, 0.0),
             constraint_pin({{0, 0}, 3}, 0.0),
         })
        m.add_constraint(c); // every admission asserted below by the bill
}

double seg_len(const SketchSegment& s) {
    const double dx = double(s.b.x) - double(s.a.x);
    const double dy = double(s.b.y) - double(s.a.y);
    return std::sqrt(dx * dx + dy * dy);
}

// Value-identical recipes: the determinism leg's own comparator. The
// comparison is FIELD-WISE on purpose: every sketch payload struct
// carries tail padding after its orientation bool — bytes the language
// leaves unspecified at aggregate initialization, identical across two
// constructions only by luck (GCC's stack reuse masked it; CLANG's
// did not — the two-compiler discipline's own catch). The determinism
// contract covers the VALUES the recipe serializes (the floats and
// the orientation bits), and THAT is what is compared, field for
// field, byte for byte.
bool entity_value_identical(const SketchEntity& a, const SketchEntity& b) {
    if (a.kind != b.kind) return false;
    const auto f32_same = [](float x, float y) { return std::memcmp(&x, &y, 4) == 0; };
    switch (a.kind) {
    case SketchEntity::Kind::Segment:
        return f32_same(a.segment.a.x, b.segment.a.x) &&
               f32_same(a.segment.a.y, b.segment.a.y) &&
               f32_same(a.segment.b.x, b.segment.b.x) &&
               f32_same(a.segment.b.y, b.segment.b.y) &&
               a.segment.interior_left == b.segment.interior_left;
    case SketchEntity::Kind::Circle:
        return f32_same(a.circle.center.x, b.circle.center.x) &&
               f32_same(a.circle.center.y, b.circle.center.y) &&
               f32_same(a.circle.radius, b.circle.radius) &&
               a.circle.hole == b.circle.hole;
    case SketchEntity::Kind::Arc:
        return f32_same(a.arc.center.x, b.arc.center.x) &&
               f32_same(a.arc.center.y, b.arc.center.y) &&
               f32_same(a.arc.radius, b.arc.radius) &&
               f32_same(a.arc.start_angle, b.arc.start_angle) &&
               f32_same(a.arc.sweep, b.arc.sweep) &&
               a.arc.hole == b.arc.hole;
    }
    return false;
}

bool chains_byte_identical(const std::vector<SketchChain>& a,
                           const std::vector<SketchChain>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t c = 0; c < a.size(); ++c) {
        if (a[c].entities.size() != b[c].entities.size()) return false;
        for (std::size_t e = 0; e < a[c].entities.size(); ++e)
            if (!entity_value_identical(a[c].entities[e], b[c].entities[e]))
                return false;
    }
    return true;
}

} // namespace

// ===========================================================================
// D1 — THE CONSUMPTIONS: each kind's rows raise the rank by exactly
// its declared consumption (the numerical-rank derivation, read back
// through the admission's own verdicts)
// ===========================================================================

TEST_CASE("D1: the consumptions — every kind raises the rank by its own") {
    // A generic 3-segment sketch (12 DoF) — each kind admitted on
    // operands it binds, the admission record carrying the verdict.
    SketchSolveModel m(drawn_triangle());

    // coincidence: consumes 2
    const SketchAdmission coin = m.add_constraint(
        constraint_coincidence({{0, 0}, kE}, {{0, 1}, kS}));
    REQUIRE(coin.admitted);
    CHECK(coin.rank_before == 0);
    CHECK(coin.rank_after == 2);
    CHECK(coin.consumption == 2);

    // parallel: consumes 1 (Angular)
    const SketchAdmission par = m.add_constraint(
        constraint_parallel({0, 0}, {0, 2}));
    REQUIRE(par.admitted);
    CHECK(par.rank_before == 2);
    CHECK(par.rank_after == 3);
    CHECK(par.consumption == 1);

    // perpendicular: consumes 1
    const SketchAdmission perp = m.add_constraint(
        constraint_perpendicular({0, 1}, {0, 2}));
    REQUIRE(perp.admitted);
    CHECK(perp.rank_before == 3);
    CHECK(perp.rank_after == 4);

    // tangent: line x circle — a fresh sketch (the triangle has no circle)
    SketchSolveModel mt({SketchChain{{
        sketch_segment({0.0f, -1.0f}, {3.0f, 1.5f}, true),
        sketch_circle({1.0f, 3.0f}, 1.25f, false),
    }}});
    const SketchAdmission tan = mt.add_constraint(
        constraint_tangent({0, 0}, {0, 1}, +1));
    REQUIRE(tan.admitted);
    CHECK(tan.rank_before == 0);
    CHECK(tan.rank_after == 1);

    // length: consumes 1
    const SketchAdmission len = m.add_constraint(constraint_length({0, 0}, 4.0));
    REQUIRE(len.admitted);
    CHECK(len.rank_before == 4);
    CHECK(len.rank_after == 5);

    // radius: consumes 1 (on the tangent's sketch)
    const SketchAdmission rad = mt.add_constraint(constraint_radius({0, 1}, 1.25));
    REQUIRE(rad.admitted);
    CHECK(rad.rank_before == 1);
    CHECK(rad.rank_after == 2);

    // distance: consumes 1
    const SketchAdmission dist = m.add_constraint(
        constraint_distance({{0, 0}, kE}, {{0, 1}, kE}, 5.0));
    REQUIRE(dist.admitted);
    CHECK(dist.rank_before == 5);
    CHECK(dist.rank_after == 6);

    // pin: consumes 1 (the raw-parameter dimension, D4)
    const SketchAdmission pin = m.add_constraint(constraint_pin({{0, 0}, 0}, 0.0));
    REQUIRE(pin.admitted);
    CHECK(pin.rank_before == 6);
    CHECK(pin.rank_after == 7);

    // THE BILL: consumed == the sum of what THIS model admitted
    // (coin 2 + par 1 + perp 1 + len 1 + dist 1 + pin 1 = 7; the
    // tangent and the radius live on mt above), and the admission
    // guarantees consumed == rank (the never-lying clause).
    const SketchBill bill = m.bill();
    CHECK(bill.dof == 12);
    CHECK(bill.consumed == 7);
    CHECK(bill.free() == 5);
}

// ===========================================================================
// D5.1 — IL TRIANGOLO SEMPLICE: the bill 12-12=0, four iterations,
// the solution derived coordinate for coordinate, N=10 identical
// ===========================================================================

TEST_CASE("D5.1: the triangle — 12-12=0, 4 iterations, the 3-4-5 identity, 10/10 identical") {
    SketchSolveModel m(drawn_triangle());
    admit_triangle(m, 3.0);

    const SketchBill bill = m.bill();
    CHECK(bill.dof == 12);
    CHECK(bill.consumed == 12);
    CHECK(bill.full());
    CHECK(bill.describe() == "12 - 12 = 0 (FULL)");
    // the rank at admission (the drawn point): full — the ADR's own leg
    // (each admission raised the rank; the bill and the rank agree).

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(r.iterations == 4); // the pinned count, derived before the code

    // THE SOLUTION, COORDINATE FOR COORDINATE: A=(0,0), B=(4,0), C=(0,3)
    // — the identity 3-4-5, machine-exact through the store.
    const SketchSegment& s0 = r.chains[0].entities[0].segment;
    const SketchSegment& s1 = r.chains[0].entities[1].segment;
    const SketchSegment& s2 = r.chains[0].entities[2].segment;
    CHECK(double(s0.a.x) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s0.a.y) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s0.b.x) == doctest::Approx(4.0).epsilon(1e-9));
    CHECK(double(s0.b.y) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s1.b.x) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s1.b.y) == doctest::Approx(3.0).epsilon(1e-9));

    // the measured lengths: 4, 5, 3 EXACT
    CHECK(seg_len(s0) == doctest::Approx(4.0).epsilon(1e-12));
    CHECK(seg_len(s1) == doctest::Approx(5.0).epsilon(1e-12));
    CHECK(seg_len(s2) == doctest::Approx(3.0).epsilon(1e-12));

    // THE DETERMINISM LEG: N solves from the same drawn point →
    // byte-identical (the canonical initial point, the canonical
    // order, no random restarts — ADR-0014 on the solve).
    std::vector<SketchChain> first = r.chains;
    for (int i = 0; i < 10; ++i) {
        SketchSolveModel m2(drawn_triangle());
        admit_triangle(m2, 3.0);
        const SketchSolveResult r2 = m2.solve();
        REQUIRE(r2.status == SketchSolveStatus::Solved);
        CHECK(r2.iterations == 4);
        CHECK(chains_byte_identical(first, r2.chains));
    }
}

// ===========================================================================
// D5.2 — LA TANGENTE: the case bit at creation, the bill 7-2=5
// REPORTED, 19 iterations, the tangency point derived
// ===========================================================================

TEST_CASE("D5.2: the tangent — the case bit, 7-2=5 free REPORTED, 19 iterations, T derived") {
    std::vector<SketchChain> chains{SketchChain{{
        sketch_segment({2.20f, 0.10f}, {1.90f, 5.10f}, true),
        sketch_circle({-0.05f, 2.45f}, 2.10f, true),
    }}};

    // THE CASE BIT DERIVED AT CREATION from the current geometry (the
    // side the center is on): ESTERNO, s = +1 — DATA, never chosen.
    const int bit = derive_tangent_case_bit(chains, {0, 0}, {0, 1});
    CHECK(bit == +1);

    SketchSolveModel m(chains);
    REQUIRE(m.add_constraint(constraint_tangent({0, 0}, {0, 1}, bit)).admitted);
    REQUIRE(m.add_constraint(constraint_radius({0, 1}, 2.0)).admitted);

    const SketchBill bill = m.bill();
    CHECK(bill.dof == 7);
    CHECK(bill.consumed == 2);
    CHECK(bill.free() == 5); // the under-constrained's honest count

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(r.iterations == 19); // the pinned count (the honest margin: 64/19 = 3.4x)
    CHECK(r.bill.free() == 5); // the count REPORTED, never hidden

    // the solved configuration (the derivation's own numbers, at the
    // store's resolution): the line a=(2.1303, 0.0960), the circle
    // C=(0.0197, 2.4540), r=2.
    const SketchSegment& s = r.chains[0].entities[0].segment;
    const SketchCircle& c = r.chains[0].entities[1].circle;
    CHECK(double(s.a.x) == doctest::Approx(2.1303257181686779).epsilon(1e-6));
    CHECK(double(s.a.y) == doctest::Approx(0.096011936770909).epsilon(1e-6));
    CHECK(double(c.center.x) == doctest::Approx(0.019674281769126).epsilon(1e-6));
    CHECK(double(c.center.y) == doctest::Approx(2.453988063229091).epsilon(1e-6));
    CHECK(double(c.radius) == doctest::Approx(2.0).epsilon(1e-12));

    // IL PUNTO DI TANGENZA DERIVATO: T = the foot of the perpendicular
    // from C — |T − C| = r, T on the line (the cross ~ 0), T on the
    // segment's span (the certificate's business, checked and green).
    const double dx = double(s.b.x) - double(s.a.x);
    const double dy = double(s.b.y) - double(s.a.y);
    const double len = std::sqrt(dx * dx + dy * dy);
    const double nhx = -dy / len, nhy = dx / len;
    const double sd = (double(c.center.x) - double(s.a.x)) * nhx +
                      (double(c.center.y) - double(s.a.y)) * nhy;
    const double Tx = double(c.center.x) - sd * nhx;
    const double Ty = double(c.center.y) - sd * nhy;
    CHECK(std::abs(sd - double(c.radius)) <= kSolveEpsLin); // the band holds
    CHECK(Tx == doctest::Approx(2.0175590258).epsilon(1e-6));
    CHECK(Ty == doctest::Approx(2.5459475629).epsilon(1e-6));
    // T on the line: the cross of (T − a) with the direction
    const double cr = (Tx - double(s.a.x)) * dy / len -
                      (Ty - double(s.a.y)) * dx / len;
    CHECK(std::abs(cr) < 1e-6);
    // T on the span: the fraction in [0, 1] (0.4896 of the length)
    const double t = (Tx - double(s.a.x)) * dx / len +
                     (Ty - double(s.a.y)) * dy / len;
    CHECK(t >= 0.0);
    CHECK(t <= len);
    CHECK(t / len == doctest::Approx(0.489597).epsilon(1e-4));
}

// ===========================================================================
// D5.3 — IL RETTANGOLO VINCOLATO: the implicit set counted (11), the
// dimensions w=6 h=4, the pins 3: the bill 16-16=0 RETURNS, 5
// iterations, the exact corners, the materialization's counts 8/12/6
// ===========================================================================

TEST_CASE("D5.3: the rectangle — the implicit set, 16-16=0, 5 iterations, 8/12/6") {
    std::vector<SketchChain> chains{SketchChain{{
        sketch_segment({0.10f, 0.05f}, {6.10f, -0.05f}, true),
        sketch_segment({6.05f, 0.02f}, {5.90f, 4.10f}, true),
        sketch_segment({5.95f, 4.15f}, {-0.10f, 3.95f}, true),
        sketch_segment({-0.05f, 4.05f}, {0.12f, -0.02f}, true),
    }}};
    SketchSolveModel m(chains);

    // THE MACRO'S IMPLICIT SET (D1's composition): 4 coincidences (8)
    // + 2 parallels (2) + 1 perpendicular (1) = 11 — the rectangle is
    // NOT a fourth entity kind.
    for (const SketchConstraint& c : {
             constraint_coincidence({{0, 0}, kE}, {{0, 1}, kS}),
             constraint_coincidence({{0, 1}, kE}, {{0, 2}, kS}),
             constraint_coincidence({{0, 2}, kE}, {{0, 3}, kS}),
             constraint_coincidence({{0, 3}, kE}, {{0, 0}, kS}),
             constraint_parallel({0, 0}, {0, 2}),
             constraint_parallel({0, 1}, {0, 3}),
             constraint_perpendicular({0, 0}, {0, 1}),
         })
        REQUIRE(m.add_constraint(c).admitted);
    {
        const SketchBill b = m.bill();
        CHECK(b.dof == 16);
        CHECK(b.consumed == 11); // the implicit set, counted
        CHECK(b.free() == 5);
    }

    // + the dimensions w=6, h=4 (2) + the pins 3 → 16-16 = 0 FULL —
    // LA BOLLETTA TORNA.
    for (const SketchConstraint& c : {
             constraint_length({0, 0}, 6.0),
             constraint_length({0, 1}, 4.0),
             constraint_pin({{0, 0}, 0}, 0.0),
             constraint_pin({{0, 0}, 1}, 0.0),
             constraint_pin({{0, 0}, 3}, 0.0),
         })
        REQUIRE(m.add_constraint(c).admitted);
    const SketchBill bill = m.bill();
    CHECK(bill.dof == 16);
    CHECK(bill.consumed == 16);
    CHECK(bill.full());
    CHECK(bill.describe() == "16 - 16 = 0 (FULL)");

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(r.iterations == 5); // the pinned count

    // the solved rectangle EXACT at the pinned placement
    const SketchSegment& s0 = r.chains[0].entities[0].segment;
    const SketchSegment& s1 = r.chains[0].entities[1].segment;
    const SketchSegment& s3 = r.chains[0].entities[3].segment;
    CHECK(double(s0.a.x) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s0.a.y) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s0.b.x) == doctest::Approx(6.0).epsilon(1e-9));
    CHECK(double(s0.b.y) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s1.b.x) == doctest::Approx(6.0).epsilon(1e-9));
    CHECK(double(s1.b.y) == doctest::Approx(4.0).epsilon(1e-9));
    CHECK(double(s3.a.x) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(double(s3.a.y) == doctest::Approx(4.0).epsilon(1e-9));

    // THE MATERIALIZATION'S COUNTS by the existing pinned formula
    // (n = 4): V=8, E=12, F=6, χ=2 — the box's own class, unchanged
    // (the solved recipe materializes exactly as the authored one).
    auto prism = b::make_prism(r.chains, 1.5, +1);
    CHECK(prism->vertex_count() == 8);
    CHECK(prism->edge_count() == 12);
    CHECK(prism->face_count() == 6);
}

// ===========================================================================
// D5.4 — LA CONTROPROVA NAIVE: the sketch WITHOUT the solver — the
// dimensions ignored (display-only, D5.2's read-only rule of
// ADR-0030), the geometry stays where it is. THE DISCRIMINATION: the
// same assertions that pass through the solver FAIL on the naive —
// measured here, on the naive's own numbers.
// ===========================================================================

TEST_CASE("D5.4: the naive counterproof — the frozen geometry's lengths, the discrimination") {
    const std::vector<SketchChain> drawn = drawn_triangle();

    // THE NAIVE's OWN NUMBERS (the derivation's measured values): the
    // geometry stays at the drawn values — the dimensions are
    // display-only (writing a length requires choosing which endpoint
    // stays, an interaction decision, not a parameter write —
    // ADR-0030 D5.2's own rule, cited by ADR-0040 D5.4).
    const double AB = seg_len(drawn[0].entities[0].segment);
    const double BC = seg_len(drawn[0].entities[1].segment);
    const double CA = seg_len(drawn[0].entities[2].segment);
    CHECK(AB == doctest::Approx(4.0512343797909312).epsilon(1e-6));
    CHECK(BC == doctest::Approx(5.0881135993607689).epsilon(1e-6));
    CHECK(CA == doctest::Approx(3.1702523558858844).epsilon(1e-6));

    // THE RED, DEMONSTRATED: the dimension assertions (|AB|=4,
    // |BC|=5, |CA|=3) FAIL on the naive — each off by the measured
    // deltas (+5.1e-2, +8.8e-2, +1.7e-1 — the derivation's numbers).
    CHECK_FALSE(AB == doctest::Approx(4.0).epsilon(1e-9));
    CHECK_FALSE(BC == doctest::Approx(5.0).epsilon(1e-9));
    CHECK_FALSE(CA == doctest::Approx(3.0).epsilon(1e-9));
    CHECK(AB - 4.0 == doctest::Approx(+5.123e-2).epsilon(1e-3));
    CHECK(BC - 5.0 == doctest::Approx(+8.811e-2).epsilon(1e-3));
    CHECK(CA - 3.0 == doctest::Approx(+1.703e-1).epsilon(1e-3));

    // THE GREEN, THROUGH THE SOLVER: the same assertions on the solved
    // recipe — 4, 5, 3 EXACT (D5.1's own leg, restated as the pair's
    // second half: the test that fails on the naive passes on the
    // real — the discrimination, measured, not asserted by fiat).
    SketchSolveModel m(drawn);
    admit_triangle(m, 3.0);
    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(seg_len(r.chains[0].entities[0].segment) ==
          doctest::Approx(4.0).epsilon(1e-12));
    CHECK(seg_len(r.chains[0].entities[1].segment) ==
          doctest::Approx(5.0).epsilon(1e-12));
    CHECK(seg_len(r.chains[0].entities[2].segment) ==
          doctest::Approx(3.0).epsilon(1e-12));
}

// ===========================================================================
// D5.5 — IL SOVRABBONDANTE: the 13th refused at admission BY NAME,
// with the bill and the spectrum's tail; the dependent angular pair
// (the rank test's own honesty: one relative angle, one DoF)
// ===========================================================================

TEST_CASE("D5.5: the over-constrained — the 13th refused with the culprit and the spectrum") {
    SketchSolveModel m(drawn_triangle());
    admit_triangle(m, 3.0); // 12/12 FULL

    // the 13th: the distance dimension on (s0.end, s1.end) = 5 — the
    // duplicate of the length c4 the user adds thinking it needed.
    const SketchAdmission a = m.add_constraint(
        constraint_distance({{0, 0}, kE}, {{0, 1}, kE}, 5.0));

    // THE RANK TEST at the canonical point: the 12 admitted rows rank
    // 12; the 13-row matrix rank 12 — the candidate does not raise
    // the admitted set's rank: REFUSED, sketch_over_constrained.
    CHECK_FALSE(a.admitted);
    CHECK(a.rank_before == 12);
    CHECK(a.rank_after == 12);
    CHECK(a.consumption == 1);

    // the spectrum's tail (the evidence): the augmented matrix's two
    // smallest singular values — the derivation's own numbers.
    CHECK(a.tail0 == doctest::Approx(3.791e-01).epsilon(1e-3));
    CHECK(a.tail1 == doctest::Approx(2.342e-01).epsilon(1e-3));

    // THE PAYLOAD: the culprit (kind, operands, value), the bill, the
    // remedy — never a silence, never an exception.
    CHECK(a.message.find("sketch_over_constrained") != std::string::npos);
    CHECK(a.message.find("the distance on (s0.end, s1.end) = 5") != std::string::npos);
    CHECK(a.message.find("12 - 12 = 0 (FULL)") != std::string::npos);
    CHECK(a.message.find("Remedy") != std::string::npos);

    // the 13th never entered the list (the over-constrained never
    // reaches the solver).
    CHECK(m.constraints().size() == 9);

    // A FULL sketch refuses EVERY further constraint by the same
    // test — including the duplicate pin (the same parameter, a
    // different value: the row cannot raise the full rank).
    CHECK_FALSE(m.add_constraint(constraint_length({0, 1}, 4.0)).admitted);
    CHECK_FALSE(m.add_constraint(constraint_pin({{0, 0}, 0}, 1.0)).admitted);

    // THE DEPENDENT ANGULAR PAIR (the rank test's own honesty): the
    // parallel and the perpendicular on the SAME pair bind the same
    // single relative angle — the second's row is a multiple of the
    // first's gradient at every generic point: REFUSED as redundant
    // at admission, the global inconsistency never the solver's to
    // lie about (the instruments' division of labor).
    SketchSolveModel mp({SketchChain{{
        sketch_segment({0.0f, 0.0f}, {4.0f, 0.30f}, true),
        sketch_segment({0.5f, 0.2f}, {3.8f, 0.9f}, true),
    }}});
    REQUIRE(mp.add_constraint(constraint_parallel({0, 0}, {0, 1})).admitted);
    const SketchAdmission second = mp.add_constraint(
        constraint_perpendicular({0, 0}, {0, 1}));
    CHECK_FALSE(second.admitted);
    CHECK(second.rank_before == 1);
    CHECK(second.rank_after == 1);
}

// ===========================================================================
// D5.6 — LA NON-CONVERGENZA: the 4/5/10 triangle — full rank at
// admission (the rank test cannot see global inconsistency), 35
// iterations STALLED, the residual per constraint, the bounded exit,
// the diagnosis. MAI un crash.
// ===========================================================================

TEST_CASE("D5.6: the not-solvable — 4/5/10: 35 stalled, the payload, the diagnosis") {
    SketchSolveModel m(drawn_triangle());
    admit_triangle(m, 10.0); // |CA| = 10: 4 + 5 < 10

    // the rank at admission: 12/12 FULL, admitted — the rank test
    // cannot see the global inconsistency (that is the SOLVE's
    // question, the instruments' division of labor).
    CHECK(m.bill().full());

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::NotSolvable);
    CHECK(r.iterations == 35); // the pinned count (stalled: the honest early answer)
    CHECK(r.residual_norm == doctest::Approx(0.408248).epsilon(1e-4)); // |F| = sqrt(1/6)

    // THE RESIDUAL PER CONSTRAINT (the derivation's own table): the
    // coin rows 1 at -0.166667, the coin rows 2 at 0, the lengths
    // +0.166667/+0.166667/-0.166667, the pins at 0.
    REQUIRE(r.violations.size() == 12);
    for (int i = 0; i < 6; ++i) { // the three coincidences, two rows each
        CHECK(std::abs(r.violations[i].value) ==
              doctest::Approx(i % 2 == 0 ? 0.166667 : 0.0).epsilon(1e-3));
        CHECK(!r.violations[i].angular); // the Linear world
    }
    CHECK(r.violations[6].value == doctest::Approx(+0.166667).epsilon(1e-3));
    CHECK(r.violations[7].value == doctest::Approx(+0.166667).epsilon(1e-3));
    CHECK(r.violations[8].value == doctest::Approx(-0.166667).epsilon(1e-3));
    for (int i = 9; i < 12; ++i)
        CHECK(std::abs(r.violations[i].value) == doctest::Approx(0.0).epsilon(1e-6));

    // THE LARGEST NAMED (the constraint that does not hold)
    CHECK(std::abs(r.violations[r.largest].value) ==
          doctest::Approx(0.166667).epsilon(1e-3));

    // THE VALUES AT THE EXIT BOUNDED: the flat chain A=(0,0),
    // B=(4.1667, 0), C=(9.5, 0) — the damped least-squares closest
    // approach, never a diverged number reported.
    REQUIRE(r.exit_params.size() == 12);
    CHECK(r.exit_params[0] == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(r.exit_params[1] == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(r.exit_params[2] == doctest::Approx(4.1667).epsilon(1e-3));
    CHECK(r.exit_params[6] == doctest::Approx(9.5).epsilon(1e-3));
    CHECK(r.exit_params[7] == doctest::Approx(0.0).epsilon(1e-3));

    // THE DIAGNOSIS, derived: the violated inequality names itself.
    CHECK(r.diagnosis.find("4+5 < 10") != std::string::npos);
    CHECK(r.diagnosis.find("no triangle with these sides exists") != std::string::npos);

    // the recipe UNTOUCHED (never an approximate solution passed off
    // as solved — the drawn values stand).
    const SketchSegment& s0 = m.chains()[0].entities[0].segment;
    CHECK(double(s0.a.x) == doctest::Approx(0.10).epsilon(1e-7));
    CHECK(double(s0.b.x) == doctest::Approx(4.15).epsilon(1e-7));
}

// ===========================================================================
// D5.7 — IL DEGENERE: the self-coincidence zeroes the segment — the
// landing at 1 ulp, the STORE at exactly 3.0f, the kernel's own class
// VERBATIM, the recipe untouched (the refusal precedes the write)
// ===========================================================================

TEST_CASE("D5.7: the degenerate — the float32 store's sharpest case, the kernel's words") {
    SketchSolveModel m({SketchChain{{
        sketch_segment({1.0f, 0.0f}, {5.0f, 0.0f}, true),
    }}});

    // the bill 4 - 2 = 2 free (reported)
    REQUIRE(m.add_constraint(
                constraint_coincidence({{0, 0}, kS}, {{0, 0}, kE})).admitted);
    const SketchBill bill = m.bill();
    CHECK(bill.dof == 4);
    CHECK(bill.consumed == 2);
    CHECK(bill.free() == 2);

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Degenerate);
    CHECK(r.iterations == 3); // the pinned count

    // THE LANDING (the derivation's sharpest case): the min-norm
    // midpoint — the endpoints at 3.0 ± 2.1e-11 in double, 1 ulp
    // apart; the STORE rounds BOTH to 3.0f, the stored segment is
    // EXACTLY zero-length (the pre-store doubles carried on the
    // result — the machine-exact landing, visible here).
    REQUIRE(r.exit_params.size() == 4);
    CHECK(std::abs(r.exit_params[0] - 3.0) < 1e-9);
    CHECK(std::abs(r.exit_params[2] - 3.0) < 1e-9);
    CHECK(std::abs(r.exit_params[0] - r.exit_params[2]) <= 8.9e-16); // 1 ulp at 3.0 (2 for safety)

    // THE VERIFICATION ON THE STORED BYTES: the construction check
    // p0 != p1 REFUSES — the payload is the kernel's own class
    // VERBATIM ("a degenerate segment has no normal to orient...
    // give the segment a non-zero extent").
    CHECK(r.degenerate_entity == "chain 0, entity 0");
    CHECK(r.degenerate_message.find("p0 != p1") != std::string::npos);
    CHECK(r.degenerate_message.find(
              "a degenerate segment has no normal to orient") != std::string::npos);
    CHECK(r.degenerate_message.find(
              "give the segment a non-zero extent") != std::string::npos);

    // THE RECIPE UNTOUCHED (the refusal precedes the write): the
    // drawn 1->5 segment stands.
    const SketchSegment& s = m.chains()[0].entities[0].segment;
    CHECK(double(s.a.x) == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(double(s.b.x) == doctest::Approx(5.0).epsilon(1e-12));
    CHECK(seg_len(s) == doctest::Approx(4.0).epsilon(1e-12));
}

// ===========================================================================
// D5.8 — L'EDIT A VALLE: the whole act, first time live — the solved
// triangle extruded (6/9/5, χ=2), the dimension edited 5 -> 5.5, the
// re-solve from the pre-edit solved geometry (5 iterations, the new C
// at the closed form), the names re-derived IDENTICALLY, the resolver
// CLASS 1 SILENT, N rebuilds bit-identical.
// ===========================================================================

TEST_CASE("D5.8: the downstream edit — the quartet's class-1 leg, live from the sketch side") {
    // 1. THE SOLVE (golden 1's output — the recipe's new truth).
    SketchSolveModel m(drawn_triangle());
    admit_triangle(m, 3.0);
    REQUIRE(m.solve().status == SketchSolveStatus::Solved);

    // 2. THE EXTRUDE (distance 2, +Z): the prism 6/9/5, χ=2 by the
    // n = 3 formula — the names ext#3:lat(0..2), cap(lo), cap(hi).
    const FeatureId sketch_id{2};
    const FeatureId ext_id{3};
    auto prism_old = b::make_prism(m.chains(), 2.0, +1, ext_id);
    REQUIRE(prism_old->vertex_count() == 6);
    REQUIRE(prism_old->edge_count() == 9);
    REQUIRE(prism_old->face_count() == 5);
    std::string digest_old;
    for (b::FaceId i = 0; i < prism_old->face_count(); ++i) {
        digest_old += std::to_string(i);
        digest_old += '|';
        digest_old += b::query_face(*prism_old, i).name;
        digest_old += '\n';
    }
    CHECK(digest_old ==
          "0|ext#3:lat(0)\n"
          "1|ext#3:lat(1)\n"
          "2|ext#3:lat(2)\n"
          "3|ext#3:cap(lo)\n"
          "4|ext#3:cap(hi)\n");

    // THE REFERENCE: a downstream feature holding ext#3:lat(1) BY
    // NAME (the structural ShapeId — the pick's own identity).
    const CAD_0::geometry::ShapeId reference = [&] {
        for (b::FaceId i = 0; i < prism_old->face_count(); ++i)
            if (b::query_face(*prism_old, i).name == "ext#3:lat(1)")
                return b::query_face(*prism_old, i).id;
        return b::query_face(*prism_old, 1).id;
    }();

    // 3. THE EDIT: the dimension |BC| 5 -> 5.5 (the lengths are the
    // authored indices 3..5 — the re-value is D3.3's own family).
    REQUIRE(m.revalue_constraint(4, 5.5));

    // 4. THE RE-SOLVE from the canonical initial point (the pre-edit
    // SOLVED geometry): 5 iterations, the new C at the closed form
    // x = (9+16-30.25)/8, y = sqrt(9 - x^2), the branch y > 0 (the
    // initial point's side) — deterministic.
    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(r.iterations == 5);
    const double xc = (3.0 * 3.0 + 4.0 * 4.0 - 5.5 * 5.5) / (2.0 * 4.0);
    const double yc = std::sqrt(9.0 - xc * xc);
    CHECK(xc == doctest::Approx(-0.65625).epsilon(1e-12));
    const SketchSegment& s1 = r.chains[0].entities[1].segment;
    CHECK(double(s1.b.x) == doctest::Approx(xc).epsilon(1e-6)); // the store's resolution
    CHECK(double(s1.b.y) == doctest::Approx(yc).epsilon(1e-6));

    // 5. THE NAMES RE-DERIVED IDENTICALLY (the roots 0..2 unmoved:
    // the edit is value-level, no-shift) and the regeneration WITHOUT
    // reselection: the resolver answers CLASS 1, SILENT.
    auto prism_new = b::make_prism(r.chains, 2.0, +1, ext_id);
    REQUIRE(prism_new->vertex_count() == 6);
    REQUIRE(prism_new->edge_count() == 9);
    REQUIRE(prism_new->face_count() == 5);
    std::string digest_new;
    for (b::FaceId i = 0; i < prism_new->face_count(); ++i) {
        digest_new += std::to_string(i);
        digest_new += '|';
        digest_new += b::query_face(*prism_new, i).name;
        digest_new += '\n';
    }
    CHECK(digest_new == digest_old); // the names identical, byte for byte

    // the generations the resolver reads (the m5 shape): the OLD side
    // with its tavola, the NEW side with the stamps. The old chains:
    // golden 1's solved output — rebuilt from the same deterministic
    // solve (the recipe's own truth, bit-identical by D5.1's leg).
    std::vector<SketchChain> chains_old;
    {
        SketchSolveModel m_old(drawn_triangle());
        admit_triangle(m_old, 3.0);
        REQUIRE(m_old.solve().status == SketchSolveStatus::Solved);
        chains_old = m_old.chains();
    }
    b::DescentTable tavola = b::extrude_descent(*prism_old, chains_old, ext_id, sketch_id);
    const b::Generation old_gen{prism_old.get(), &tavola, &n::kExtNaming, false};
    const b::Generation new_gen{prism_new.get(), nullptr, &n::kExtNaming, false};

    // the edit context: the no-shift declaration (the v1-covered
    // flow — the value-level edit, D3.3's derivation).
    const b::EditContext edit_ctx{/*roots_no_shift=*/true, {}};

    const b::Resolution res = b::resolve(reference, old_gen, new_gen, edit_ctx);
    REQUIRE(res.kind == b::Resolution::Kind::Resolved); // CLASSE 1
    CHECK(res.resolved.name == "ext#3:lat(1)");
    CHECK(res.resolved.id.subshape == reference.subshape);

    // 6. N REBUILDS BIT-IDENTICAL (the whole act, deterministic end
    // to end: the solve, the extrude, the names).
    for (int i = 0; i < 10; ++i) {
        SketchSolveModel mi(drawn_triangle());
        admit_triangle(mi, 3.0);
        REQUIRE(mi.solve().status == SketchSolveStatus::Solved);
        REQUIRE(mi.revalue_constraint(4, 5.5));
        const SketchSolveResult ri = mi.solve();
        REQUIRE(ri.status == SketchSolveStatus::Solved);
        CHECK(ri.iterations == 5);
        CHECK(chains_byte_identical(ri.chains, r.chains));
        auto prism_i = b::make_prism(ri.chains, 2.0, +1, ext_id);
        std::string digest_i;
        for (b::FaceId f = 0; f < prism_i->face_count(); ++f) {
            digest_i += std::to_string(f);
            digest_i += '|';
            digest_i += b::query_face(*prism_i, f).name;
            digest_i += '\n';
        }
        CHECK(digest_i == digest_old);
    }
}

// ===========================================================================
// IL CASO VICINO ALLA BANDA — the angular margin MEASURED: the store
// path's noise vs the declared band (the 8.4x margin's own proof),
// the re-homing pinned at compile time (the band IS the kernel's
// constant — where declared, not where convenient), and the
// comfortable case's contrast.
// ===========================================================================

TEST_CASE("The near-band case — the angular margin measured, the band where declared") {
    // THE RE-HOMING, pinned at compile time (ADR-0035 D2's SOLVE row:
    // one value per world, two consumers, ZERO drift — a violation of
    // this pin is a defect of the morse, not a style).
    static_assert(kSolveEpsLin == static_cast<double>(kJointCoincidenceTol),
                  "eps_solve^lin must BE kJointCoincidenceTol — re-homed, never copied");
    static_assert(kSolveEpsAng == static_cast<double>(kParallelAngleTol),
                  "eps_solve^ang must BE kParallelAngleTol — re-homed, never copied");
    static_assert(kSolveEpsLin == static_cast<double>(1e-5f),
                  "the Linear member's declared value (the float constant's own)");
    static_assert(kSolveEpsAng == static_cast<double>(1e-6f),
                  "the Angular member's declared value (the float constant's own)");

    // THE TIGHTEST REALISTIC CASE: a full-rank system solved at
    // GENERIC coordinates (nothing exactly representable) — the
    // solved values pass through the float32 store, and the ANGULAR
    // row's violation on the STORED bytes is the noise the band must
    // hold. The derived noise bound: two O(1) unit components, each
    // float32 — 2 * 2^-24 = 1.19e-7; the declared margin 1e-6 / bound
    // = 8.4x (the honest worst case, declared in the ADR's table).
    constexpr double kUnitCrossNoiseBound = 2.0 * 5.9604644775390625e-8; // 2 * 2^-24

    SketchSolveModel m({SketchChain{{
        sketch_segment({0.30f, 0.70f}, {4.90f, 1.50f}, true),
        sketch_segment({1.20f, 0.10f}, {3.30f, 0.60f}, true),
    }}});
    for (const SketchConstraint& c : {
             constraint_pin({{0, 0}, 0}, 0.30),
             constraint_pin({{0, 0}, 1}, 0.70),
             constraint_pin({{0, 0}, 2}, 4.90),
             constraint_pin({{0, 0}, 3}, 1.50),
             constraint_pin({{0, 1}, 0}, 1.20),
             constraint_pin({{0, 1}, 1}, 0.10),
             constraint_parallel({0, 0}, {0, 1}),
             constraint_length({0, 1}, 2.0),
         })
        REQUIRE(m.add_constraint(c).admitted);
    CHECK(m.bill().describe() == "8 - 8 = 0 (FULL)");

    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);

    // the solved end: start + 2 * u0 — GENERIC coordinates (the
    // derivation's own landing, the pre-store double, full precision
    // measured on this tree).
    REQUIRE(r.exit_params.size() == 8);
    CHECK(r.exit_params[6] == doctest::Approx(3.17042351).epsilon(1e-8));
    CHECK(r.exit_params[7] == doctest::Approx(0.442682356).epsilon(1e-8));

    // THE ANGULAR VIOLATION ON THE STORED BYTES, measured: the unit
    // cross of the two stored segments. The band holds where
    // declared: the measurement sits under the derived noise bound,
    // and the bound sits under the band with the declared 8.4x
    // margin — the comfortable case (the axis-aligned rectangle at
    // exactly-representable coordinates measures ~1e-24) proves
    // nothing; THIS case is the proof.
    const SketchSegment& sa = r.chains[0].entities[0].segment;
    const SketchSegment& sb = r.chains[0].entities[1].segment;
    const auto unit = [](const SketchSegment& s) {
        const double dx = double(s.b.x) - double(s.a.x);
        const double dy = double(s.b.y) - double(s.a.y);
        const double nn = std::sqrt(dx * dx + dy * dy);
        return std::pair<double, double>{dx / nn, dy / nn};
    };
    const auto [u0x, u0y] = unit(sa);
    const auto [u1x, u1y] = unit(sb);
    const double stored_cross = u0x * u1y - u0y * u1x;
    CHECK(std::abs(stored_cross) <= kUnitCrossNoiseBound); // <= 1.19e-7
    CHECK(std::abs(stored_cross) <= kSolveEpsAng);         // the band holds
    // the margin, MEASURED against the derived bound (the honest
    // worst case): 1e-6 / (2 * 2^-24) = 8.3886 — the ADR's "8.4x"
    // is this number rounded — the band holds where it is declared,
    // not where it is convenient.
    CHECK(kSolveEpsAng / kUnitCrossNoiseBound ==
          doctest::Approx(8.3886).epsilon(1e-3));
}

// ===========================================================================
// THE ADDITIVITY CLAUSE — the propagation world survives the solver's
// landing day: the empty-constraint sketch (every pre-solver sketch
// in every file) and the parametric hole's own under-constrained
// world SOLVE, the values stand, the count is reported.
// ===========================================================================

TEST_CASE("The additivity clause — the pre-solver world solves, untouched") {
    // The empty-constraint sketch: nothing to hold — SOLVED, all 12
    // free, nothing moved (the extreme under-constrained case).
    SketchSolveModel m(drawn_triangle());
    const SketchSolveResult r = m.solve();
    REQUIRE(r.status == SketchSolveStatus::Solved);
    CHECK(r.iterations == 0);
    CHECK(r.bill.free() == 12);
    CHECK(chains_byte_identical(r.chains, drawn_triangle()));

    // THE PARAMETRIC HOLE'S OWN WORLD (ADR-0030's guide case, the
    // recipe the solver was waiting for): the circle with one radius
    // dimension — 3 - 1 = 2 free, REPORTED, the free values at their
    // authored homes (the panel edits them, the propagation flows).
    SketchSolveModel hole({SketchChain{{
        sketch_circle({0.0f, 0.0f}, 0.5f, true),
    }}});
    REQUIRE(hole.add_constraint(constraint_radius({0, 0}, 0.5)).admitted);
    CHECK(hole.bill().describe() == "3 - 1 = 2 free");
    const SketchSolveResult rh = hole.solve();
    REQUIRE(rh.status == SketchSolveStatus::Solved);
    CHECK(rh.bill.free() == 2);
    // the authored geometry already satisfies: nothing moved.
    const SketchCircle& c = rh.chains[0].entities[0].circle;
    CHECK(double(c.radius) == doctest::Approx(0.5).epsilon(1e-12));

    // THE EDIT the hole's own example practices: the radius 0.5 ->
    // 0.75 through the constraint (the dimension IS the recipe's
    // parameter, D3.1) — the solve moves the value, the store holds.
    REQUIRE(hole.revalue_constraint(0, 0.75));
    const SketchSolveResult r2 = hole.solve();
    REQUIRE(r2.status == SketchSolveStatus::Solved);
    CHECK(double(r2.chains[0].entities[0].circle.radius) ==
          doctest::Approx(0.75).epsilon(1e-12));

    // THE VALUE RULES' REMEDIES (the ADR's own words): v = 0 is the
    // coincidence's question; the zero length has no direction; the
    // zero radius is Disc2D's own refusal; the case bit is ±1.
    REQUIRE_THROWS_AS(constraint_distance({{0, 0}, kE}, {{0, 1}, kE}, 0.0),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(constraint_length({0, 0}, 0.0), std::invalid_argument);
    REQUIRE_THROWS_AS(constraint_radius({0, 0}, -1.0), std::invalid_argument);
    REQUIRE_THROWS_AS(constraint_tangent({0, 0}, {0, 1}, 0),
                      std::invalid_argument);
    try {
        (void)constraint_distance({{0, 0}, kE}, {{0, 1}, kE}, 0.0);
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("use the coincidence") != std::string::npos);
        CHECK(msg.find("the right tool for v = 0") != std::string::npos);
    }
}
