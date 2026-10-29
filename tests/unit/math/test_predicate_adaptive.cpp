// tests/unit/math/test_predicate_adaptive.cpp
//
// THE DISCRIMINATING TESTS OF THE ADAPTIVITY — H1 (ADR-0034 D2), the
// proof of the vendoring.
//
// The vendored predicates.c.h is the authentic Shewchuk robust.c: the
// ADAPTIVE implementation (error-bound filters over exact expansion
// arithmetic). The file that preceded it in this directory was a
// hand-written non-adaptive stub — naive determinants in double — that
// the register called "vendored Shewchuk". These tests are the
// discrimination between the two:
//
//   * the PINNED VECTORS: near-degenerate inputs (points nearly
//     collinear at ulp scale; orient3d nearly coplanar; incircle
//     nearly cocircular) where the naive double determinant and the
//     exact rational determinant DISAGREE — the naive returns a false
//     zero or the wrong sign, the adaptive returns the exact sign.
//     Every vector is pinned with its construction, its exact rational
//     determinant (computed with exact Fraction arithmetic over the
//     doubles' own values — the doubles ARE exact rationals, so the
//     expected sign is a theorem, not a measurement), and the naive's
//     documented failure on it.
//   * THE NEGATIVE BLOCK: the same vectors through the naive
//     arithmetic — Shewchuk's own *fast (non-robust) variants'
//     formulas, reproduced as local helpers — CHECKED to fail where
//     the vendored predicate passes. If the tree ever loses the
//     adaptive path (a stub swapped back in, a "simplification" that
//     keeps only the fast filter), THIS block fails: the naive cannot
//     pass the discrimination by construction.
//   * THE EXACT-ZERO PINS: exactly-degenerate inputs (dyadic
//     collinear/coplanar/cocircular sets, and the ulp-scale midpoint
//     construction) where the adaptive returns EXACTLY 0.0 — the
//     expansion arithmetic terminates on the true zero rather than
//     guessing at the noise floor.
//
// The vectors were found by the H1 search (exact rational arithmetic
// against the naive formulas, deterministic enumeration of ulp-perturbed
// decimal families); the search's derivation lives with the H1
// delivery notes and the values are pinned here verbatim.
#include <doctest/doctest.h>

#include "CAD_0/math/predicate.hpp"

#include <cmath>

using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;
using CAD_0::math::predicate::incircle2d;
using CAD_0::math::predicate::initialize;
using CAD_0::math::predicate::orient2d;
using CAD_0::math::predicate::orient3d;

namespace {

// The naive arithmetic, VERBATIM from Shewchuk's own non-robust *fast
// variants (the formulas the historical stub imitated — with a sign
// error in orient3d's middle term, corrected here to isolate the
// ADAPTIVITY question: the discrimination below must hold against a
// mathematically-correct naive, which is the strongest honest
// comparison). Local helpers: the vendored file's own *fast functions
// live in the CAD_0_math TU with external linkage.
double naive_orient2d(const Vec2d& pa, const Vec2d& pb, const Vec2d& pc) {
    const double acx = pa.x - pc.x, bcx = pb.x - pc.x;
    const double acy = pa.y - pc.y, bcy = pb.y - pc.y;
    return acx * bcy - acy * bcx;
}

double naive_orient3d(const Vec3d& pa, const Vec3d& pb,
                      const Vec3d& pc, const Vec3d& pd) {
    const double adx = pa.x - pd.x, bdx = pb.x - pd.x, cdx = pc.x - pd.x;
    const double ady = pa.y - pd.y, bdy = pb.y - pd.y, cdy = pc.y - pd.y;
    const double adz = pa.z - pd.z, bdz = pb.z - pd.z, cdz = pc.z - pd.z;
    return adx * (bdy * cdz - bdz * cdy)
         + bdx * (cdy * adz - cdz * ady)
         + cdx * (ady * bdz - adz * bdy);
}

double naive_incircle(const Vec2d& pa, const Vec2d& pb,
                      const Vec2d& pc, const Vec2d& pd) {
    const double adx = pa.x - pd.x, ady = pa.y - pd.y;
    const double bdx = pb.x - pd.x, bdy = pb.y - pd.y;
    const double cdx = pc.x - pd.x, cdy = pc.y - pd.y;
    const double abdet = adx * bdy - bdx * ady;
    const double bcdet = bdx * cdy - cdx * bdy;
    const double cadet = cdx * ady - adx * cdy;
    const double alift = adx * adx + ady * ady;
    const double blift = bdx * bdx + bdy * bdy;
    const double clift = cdx * cdx + cdy * cdy;
    return alift * bcdet + blift * cadet + clift * abdet;
}

} // namespace

// ---------------------------------------------------------------------------
// orient2d — nearly collinear at ulp scale: the naive's false zero
// ---------------------------------------------------------------------------
TEST_CASE("predicate adaptive: orient2d nearly-collinear, naive false zero") {
    initialize();

    // Construction: the mirror pair (a, b), (b, a) with the third point
    // one ulp off the diagonal in x. Exact rational determinant:
    //   -2^-55 = -2.7755575615628914e-17 (negative — a RIGHT turn).
    // Naive double: detleft and detright both round to the same value,
    // the subtraction reads exactly 0.0 — a FALSE collinearity.
    const Vec2d a1{0.1, 1.1};
    const Vec2d b1{1.1, 0.1};
    const Vec2d c1{0.6000000000000001, 0.6};
    CHECK(orient2d(a1, b1, c1) < 0.0);           // vendored: the exact sign
    CHECK(naive_orient2d(a1, b1, c1) == 0.0);    // naive: the false zero

    // Same family, larger scale. Exact: -1.3322676295501878e-16; the
    // naive again reads exactly 0.0.
    const Vec2d a2{0.2, 2.6};
    const Vec2d b2{2.6, 0.2};
    const Vec2d c2{1.4000000000000001, 1.4};
    CHECK(orient2d(a2, b2, c2) < 0.0);
    CHECK(naive_orient2d(a2, b2, c2) == 0.0);
}

// ---------------------------------------------------------------------------
// orient2d — exactly collinear: the adaptive answers the exact 0
// ---------------------------------------------------------------------------
TEST_CASE("predicate adaptive: orient2d exact-zero pins") {
    initialize();

    // Dyadic collinear points (0.5,0.5), (0.75,0.75), (1,1): the exact
    // determinant is 0 and the naive happens to agree — the pin that
    // the ADAPTIVE also returns the exact 0.0 (never a noise-floor
    // residue).
    const Vec2d a{0.5, 0.5};
    const Vec2d b{0.75, 0.75};
    const Vec2d c{1.0, 1.0};
    CHECK(orient2d(a, b, c) == 0.0);

    // The ulp-scale midpoint construction: c = (0.75, 0.75) is the
    // EXACT dyadic midpoint of a and b (a + b = (1.0 + 2^-52... the
    // coordinates here double exactly), so the exact determinant is 0
    // even though a and b are one ulp apart in their coordinates. The
    // adaptive PROVES the collinearity; a noise-floor implementation
    // would answer noise.
    const Vec2d m1{0.5, 0.5};
    const Vec2d m2{0.5000000000000001, 0.5000000000000001};
    const Vec2d m3{0.75, 0.75};
    CHECK(orient2d(m1, m2, m3) == 0.0);
}

// ---------------------------------------------------------------------------
// orient3d — nearly coplanar: the naive's false zero and wrong sign
// ---------------------------------------------------------------------------
TEST_CASE("predicate adaptive: orient3d nearly-coplanar discriminated") {
    initialize();

    // Wrong ZERO: points built on the main diagonal with ulp offsets.
    // Exact rational determinant: -9.860761315262648e-32 (negative).
    // The naive double reads exactly 0.0 — a FALSE coplanarity, the
    // class of defect that misclassifies a fragment on the wrong side
    // of a face in a boolean.
    const Vec3d p0a{0.0, 0.0, 0.0};
    const Vec3d p0b{0.5, 0.5, 0.5};
    const Vec3d p0c{1.0, 1.0000000000000004, 1.0};
    const Vec3d p0d{1.5, 1.5, 1.5000000000000004};
    CHECK(orient3d(p0a, p0b, p0c, p0d) < 0.0);
    CHECK(naive_orient3d(p0a, p0b, p0c, p0d) == 0.0);

    // Wrong SIGN: the same construction at 1e-3 scale. Exact rational
    // determinant: -1.003609e-35 (negative). The naive reads
    // +1.27e-24 — POSITIVE: its products' rounding error (products of
    // ~1e-6 scale carry ~1e-22 absolute error) is TWENTY-TWO ORDERS
    // above the true determinant, and lands on the wrong side.
    const Vec3d p1a{0.0, 0.0, 0.0};
    const Vec3d p1b{0.001, 0.001, 0.001};
    const Vec3d p1c{0.002, 0.0020000000000001002, 0.002};
    const Vec3d p1d{0.003, 0.003, 0.0030000000000001002};
    CHECK(orient3d(p1a, p1b, p1c, p1d) < 0.0);
    CHECK(naive_orient3d(p1a, p1b, p1c, p1d) > 0.0);  // the wrong sign

    // The sign CONVENTION pin (corrected with the vendoring — the
    // historical stub's orient3d carried a sign error in its middle
    // term and the old unit test pinned its output): pa,pb,pc CCW seen
    // from +Z, pd on the +Z side → pd is ABOVE the plane → orient3d <
    // 0 (predicate.hpp's documented convention; Shewchuk's own: "a
    // positive value if pd lies below the plane").
    const Vec3d t1{0, 0, 0};
    const Vec3d t2{1, 0, 0};
    const Vec3d t3{0, 1, 0};
    const Vec3d t4{0, 0, 1};
    CHECK(orient3d(t1, t2, t3, t4) < 0.0);
    // and the exact coplanar set: the exact 0.0, not noise.
    const Vec3d t5{1, 1, 0};
    CHECK(orient3d(t1, t2, t3, t5) == 0.0);
}

// ---------------------------------------------------------------------------
// incircle — nearly cocircular: the naive's wrong sign
// ---------------------------------------------------------------------------
TEST_CASE("predicate adaptive: incircle nearly-cocircular discriminated") {
    initialize();

    // Three points on the unit circle at decimal angles 0.1, 1.2, 2.7
    // rad (their doubles are NOT exactly on any common circle), and a
    // fourth at decimal angle 0.9 rad. Exact rational determinant:
    // -5.613485e-18 — pd lies (exactly, as rationals) a hair OUTSIDE
    // the circle through pa,pb,pc. The naive reads +5.55e-17:
    // INSIDE, the wrong side — at ulp scale the naive's verdict is
    // pure noise.
    const Vec2d c1a{0.9950041652780258, 0.09983341664682815};
    const Vec2d c1b{0.3623577544766736, 0.9320390859672263};
    const Vec2d c1c{-0.9040721420170612, 0.4273798802338298};
    const Vec2d c1d{0.6216099682706644, 0.7833269096274834};
    CHECK(incircle2d(c1a, c1b, c1c, c1d) < 0.0);   // vendored: outside
    CHECK(naive_incircle(c1a, c1b, c1c, c1d) > 0.0);  // naive: wrong side

    // Same family, angles 1.1/3.3/5.9 with the fourth at 2.3 rad.
    // Exact: -1.707063e-17 (outside); naive: +4.44e-16 (inside).
    const Vec2d c2a{0.4535961214255773, 0.8912073600614354};
    const Vec2d c2b{-0.9874797699088649, -0.1577456941432482};
    const Vec2d c2c{0.9274784307440359, -0.373876664830236};
    const Vec2d c2d{-0.6662760212798241, 0.7457052121767203};
    CHECK(incircle2d(c2a, c2b, c2c, c2d) < 0.0);
    CHECK(naive_incircle(c2a, c2b, c2c, c2d) > 0.0);

    // The exact-cocircular pin: (1,0), (0,1), (-1,0) on the unit
    // circle, (0,-1) exactly on it too — the exact 0.0.
    const Vec2d z1{1.0, 0.0};
    const Vec2d z2{0.0, 1.0};
    const Vec2d z3{-1.0, 0.0};
    const Vec2d z4{0.0, -1.0};
    CHECK(incircle2d(z1, z2, z3, z4) == 0.0);
}

// ---------------------------------------------------------------------------
// The float overloads promote and stay discriminating (the wrapper's
// contract: float in, double arithmetic, exact sign out).
// ---------------------------------------------------------------------------
TEST_CASE("predicate adaptive: float overloads promote losslessly") {
    initialize();

    // The same V1 family as above, but authored in float32: the
    // promotion float→double is exact, so the predicate consumes the
    // exact float rationals. The float inputs are themselves further
    // from the diagonal than the double ones (float's ulp is coarser),
    // so this pins the OVERLOAD's plumbing (same sign through both
    // paths on the same near-degenerate class), not the double
    // vectors' exact values.
    const CAD_0::math::Vec2f fa{0.1f, 1.1f};
    const CAD_0::math::Vec2f fb{1.1f, 0.1f};
    const CAD_0::math::Vec2f fc{std::nextafterf(0.6f, 1.0f), 0.6f};
    const double via_float = orient2d(fa, fb, fc);
    const double via_double =
        orient2d(Vec2d{fa.x, fa.y}, Vec2d{fb.x, fb.y}, Vec2d{fc.x, fc.y});
    CHECK(via_float == via_double);
    CHECK(via_float != 0.0);
}
