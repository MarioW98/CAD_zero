// tests/unit/sdf/test_operators.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/math/vec.hpp"

#include <cmath>

using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("Union: two spheres — value at midpoint") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto u = sdf_union(std::move(a), std::move(b));
    // The union of two unit spheres centered at origin is just a unit sphere.
    CHECK(u.value({0, 0, 0}) == doctest::Approx(-1.0f));
}

TEST_CASE("Union: shifted spheres") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto u = sdf_union(std::move(a), std::move(b));
    // Midpoint between the two spheres
    const float v = u.value({1.5f, 0, 0});
    // At x=1.5, distance to sphere A (center 0, r 1) is 0.5
    // Distance to sphere B (center 3, r 1) is 0.5
    // Union (min) = 0.5
    CHECK(v == doctest::Approx(0.5f).epsilon(1e-5f));
}

TEST_CASE("Subtract: cube minus sphere") {
    auto box = make_box({1, 1, 1});
    auto sph = make_sphere(0.5f);
    auto cut = sdf_subtract(std::move(box), std::move(sph));
    // Center of the cube — sphere center is also at origin.
    // Subtraction = intersect(a, complement(b)) = max(a_value, -b_value).
    // a (box) at center = -1 (inside)
    // b (sphere r=0.5) at center = -0.5 (inside)
    // -b = 0.5
    // max(-1, 0.5) = 0.5  → outside the result
    const float v = cut.value({0, 0, 0});
    CHECK(v == doctest::Approx(0.5f).epsilon(1e-5f));
}

TEST_CASE("Intersect: two unit spheres overlap fully") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto i = sdf_intersect(std::move(a), std::move(b));
    // Intersection of identical spheres = the sphere itself
    CHECK(i.value({0, 0, 0}) == doctest::Approx(-1.0f));
}

TEST_CASE("SmoothMin: k=0 equals hard union") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.0f);
    auto hard = sdf_union(make_sphere(1.0f),
                         sdf_translate(make_sphere(1.0f), {3, 0, 0}));
    CHECK(smin.value({1.5f, 0, 0}) == doctest::Approx(hard.value({1.5f, 0, 0})).epsilon(1e-4f));
}

TEST_CASE("SmoothMin: k>0 produces smoother blend") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.5f);
    // At the midpoint, the smooth union should produce a value slightly
    // LESS than the hard union (i.e. a wider "inside" region).
    const float v_smooth = smin.value({1.5f, 0, 0});
    auto ha = make_sphere(1.0f);
    auto hb = sdf_translate(make_sphere(1.0f), {3, 0, 0});
    auto hard = sdf_union(std::move(ha), std::move(hb));
    const float v_hard = hard.value({1.5f, 0, 0});
    CHECK(v_smooth <= v_hard + 1e-5f);
}

// ===========================================================================
// SmoothMin k>0 regression (added 2026-10-16, after the clamp-sign bug)
//
// The bug: for k>0 the clamp argument had the sign flipped, so the blend
// followed the LARGER child (a smooth MAX — an inflated intersection)
// instead of the smaller (min = union). It survived the whole suite
// because the tests above sample only the SYMMETRIC midpoint (a == b),
// where max and min coincide. The regressions below pin the ASYMMETRIC
// regions, where the two operators differ. A regression of an operator
// belongs to the operator's suite — the bridge tests (T7-T10) that
// caught the bug originally only see it through extraction counts.
//
// Configuration: two unit spheres, centers (0,0,0) and (1.25, 0, 0),
// k = 0.4. Every constant is exact in binary floating point, so the
// expected values are hand-derived and bit-exact where claimed.
// ===========================================================================

TEST_CASE("SmoothMin regression: clamped regions follow the SMALLER child (a != b)") {
    // Center of A: a = -1, b = |1.25| - 1 = 0.25 → b - a = 1.25 > k →
    //   h = 0 → v = a + diff·0 - 0 = a, BIT-EXACT.
    // Center of B: a = 0.25, b = -1 → a - b = 1.25 > k →
    //   h = 1 → v = a + diff·1 - 0 = b, BIT-EXACT.
    // Under the bug both returned the LARGER child (+0.25 — the center
    // of a sphere evaluated OUTSIDE its own smooth union).
    auto smin = sdf_smooth_union(
        make_sphere(1.0f),
        sdf_translate(make_sphere(1.0f), {1.25f, 0.0f, 0.0f}), 0.4f);

    CHECK(smin.value({0.0f, 0.0f, 0.0f}) == -1.0f);
    CHECK(smin.value({1.25f, 0.0f, 0.0f}) == -1.0f);
}

TEST_CASE("SmoothMin regression: symmetric waist blends to a - k/16") {
    // Midpoint (0.625, 0, 0): a = b = -0.375 exactly → h = 1/2 and
    //   v = a - k·h·(1-h)/4 = a - k/16 = -0.375 - 0.025 = -0.4
    // (hand-derived; the only float noise is k = fl(0.4), ~1e-8
    // relative — hence the tight Approx instead of ==).
    auto smin = sdf_smooth_union(
        make_sphere(1.0f),
        sdf_translate(make_sphere(1.0f), {1.25f, 0.0f, 0.0f}), 0.4f);

    CHECK(smin.value({0.625f, 0.0f, 0.0f}) == doctest::Approx(-0.4f).epsilon(1e-6f));
}

TEST_CASE("SmoothMin regression: asymmetric blend point vs hand-derived value") {
    // p = (0.5, 0, 0): a = -0.5, b = -0.25 (both exact), diff = 0.25 <
    // k → blend region. h = 0.5 - 0.5·(0.25/0.4) = 0.1875. Hand-derived:
    //   v = a + diff·h - k·h·(1-h)/4
    //     = -0.5 + 0.25·0.1875 - 0.4·0.1875·0.8125/4
    //     = -0.468359375
    // (exact in the reals; fl(0.4) adds ~1e-8 relative noise).
    auto smin = sdf_smooth_union(
        make_sphere(1.0f),
        sdf_translate(make_sphere(1.0f), {1.25f, 0.0f, 0.0f}), 0.4f);

    CHECK(smin.value({0.5f, 0.0f, 0.0f}) == doctest::Approx(-0.468359375f).epsilon(1e-6f));
}

TEST_CASE("SmoothMin regression: analytic gradient matches FD in the asymmetric blend band") {
    // The sign fix also re-derived the chain-rule coefficient
    // (5/8 → 3/8)·(2h-1). A wrong coefficient produces a DIRECTION error
    // of order 0.25·|grad_b - grad_a| ≈ 0.3 — three orders above FD
    // noise. Central differences of value() must therefore match the
    // analytic sample().gradient at ASYMMETRIC points of the blend band
    // (the symmetric waist has 2h-1 = 0 and cannot see the coefficient).
    auto smin = sdf_smooth_union(
        make_sphere(1.0f),
        sdf_translate(make_sphere(1.0f), {1.25f, 0.0f, 0.0f}), 0.4f);

    const float h = 1e-3f;
    const Vec3f pts[] = {
        {0.45f, 0.0f, 0.0f}, {0.50f, 0.0f, 0.0f}, {0.55f, 0.0f, 0.0f},
        {0.60f, 0.0f, 0.0f}, {0.65f, 0.0f, 0.0f}, {0.70f, 0.0f, 0.0f},
        {0.75f, 0.0f, 0.0f}, {0.80f, 0.0f, 0.0f}, {0.55f, 0.10f, 0.0f},
        {0.65f, -0.10f, 0.0f}, {0.60f, 0.0f, 0.15f}, {0.50f, 0.20f, -0.10f},
    };
    for (const auto& p : pts) {
        const auto s = smin.sample(p);
        const Vec3f fd{
            smin.value(p + Vec3f{ h, 0, 0}) - smin.value(p + Vec3f{-h, 0, 0}),
            smin.value(p + Vec3f{0,  h, 0}) - smin.value(p + Vec3f{0, -h, 0}),
            smin.value(p + Vec3f{0, 0,  h}) - smin.value(p + Vec3f{0, 0, -h}),
        };
        const Vec3f fd_grad = fd * (0.5f / h);
        const Vec3f err = s.gradient - fd_grad;
        INFO("p =", p.x, p.y, p.z, " analytic:", s.gradient.x, s.gradient.y,
             s.gradient.z, " fd:", fd_grad.x, fd_grad.y, fd_grad.z);
        CHECK(err.length() < 1e-3f);
    }
}

TEST_CASE("Operators: lipschitz is composed correctly") {
    auto a = make_sphere(1.0f);
    auto b = make_sphere(1.0f);
    auto u = sdf_union(std::move(a), std::move(b));
    CHECK(u.lipschitz() == 1.0f);
}

// ----------------------------------------------------------------------
// Gradient normalization regression tests (added after the bug fix)
// ----------------------------------------------------------------------
TEST_CASE("Subtraction: gradient magnitude is ≤ 1 (Lipschitz invariant)") {
    // The SubtractionSDF must return a unit gradient, even when its
    // children are composited (here: a box minus a sphere).
    auto box = make_box({1, 1, 1});
    auto sph = make_sphere(0.5f);
    auto cut = sdf_subtract(std::move(box), std::move(sph));

    // Sample at a point where the subtraction surface is active
    // (just outside the sphere, inside the box, on the -X side).
    const auto s = cut.sample({-0.6f, 0.0f, 0.0f});
    CHECK(s.gradient.length() <= 1.0f + 1e-5f);
}

TEST_CASE("Subtraction: gradient on the b-side is -∇b normalized") {
    // Box minus sphere: at a point well inside the sphere (so the
    // subtraction surface is the sphere), the gradient should be
    // the outward normal of the sphere, but negated (because we are
    // inside b, and the result surface is "−b").
    auto box = make_box({2, 2, 2});
    auto sph = make_sphere(1.0f);
    auto cut = sdf_subtract(std::move(box), std::move(sph));
    // At point (0, 0.5, 0) — inside the sphere (distance to sphere
    // center is 0.5 < 1.0), inside the box.
    // The subtraction result: max(box_value, -sphere_value).
    //   box_value at (0, 0.5, 0) = -1 (inside box).
    //   sphere_value at (0, 0.5, 0) = 0.5 - 1.0 = -0.5 (inside sphere).
    //   -sphere_value = 0.5 (outside the result).
    // So result.value = 0.5, and the gradient should be -∇sphere = -(0, 1, 0).
    const auto s = cut.sample({0.0f, 0.5f, 0.0f});
    CHECK(s.value == doctest::Approx(0.5f).epsilon(1e-4f));
    CHECK(s.gradient.x == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(s.gradient.y == doctest::Approx(-1.0f).epsilon(1e-4f));
    CHECK(s.gradient.z == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(s.gradient.length() == doctest::Approx(1.0f).epsilon(1e-4f));
}

TEST_CASE("Union: gradient is normalized even with composite children") {
    // Two spheres overlapping → the union returns whichever sphere is
    // closer, and the gradient should be that sphere's outward normal
    // (already unit length). This test guards against future regressions
    // where the union might propagate an un-normalized gradient.
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {0.5f, 0.0f, 0.0f});
    auto u = sdf_union(std::move(a), std::move(b));
    // Sample at (2, 0, 0) — closest to sphere b.
    const auto s = u.sample({2.0f, 0.0f, 0.0f});
    CHECK(s.gradient.length() <= 1.0f + 1e-5f);
}

// ===========================================================================
// Task 1 — SmoothMinSDF Lipschitz bound regression tests
//
// The previous implementation reported
//     lipschitz() = max(L_a, L_b) + (k > 0 ? 0.25 : 0)
// but this was both (a) undocumented and (b) not actually verified
// against the field's true gradient. We now provide a numerical
// Lipschitz test that samples |∇f| via finite differences on a dense
// grid centered around the blend region, and asserts that the
// declared lipschitz() upper-bounds every measured |∇f|.
// ===========================================================================

TEST_CASE("SmoothMin: lipschitz() is a valid upper bound for k>0") {
    // Two unit spheres offset by 3 along X; the blend region is
    // centered around (1.5, 0, 0). At the blend, the field can
    // briefly grow steeper than 1 because the smooth-min interpolates
    // between ∇a and ∇b with a k-dependent weight. Empirically the
    // maximum |∇f| for k=0.5 is ~1.80; we use the conservative bound
    // L = max(L_a, L_b) + 1.0 = 2.0.
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3.0f, 0.0f, 0.0f});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.5f);

    const float L = smin.lipschitz();
    // After the fix, lipschitz() = 2.5 · max(L_a, L_b) = 2.5 (see
    // operators.hpp derivation). The previous value 2.0 was wrong.
    CHECK(L == doctest::Approx(2.5f).epsilon(1e-5f));

    // Sample |∇f| via central differences on a 21×7×7 grid centered
    // on the blend region (1.5, 0, 0).
    const float h = 1e-3f;  // FD step
    float max_grad = 0.0f;
    for (int i = 0; i < 21; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                const float x = 1.5f + static_cast<float>(i - 10) * 0.10f;
                const float y = static_cast<float>(j - 3) * 0.10f;
                const float z = static_cast<float>(k - 3) * 0.10f;
                const Vec3f p{x, y, z};
                const Vec3f gx{
                    smin.value(p + Vec3f{ h, 0, 0}) - smin.value(p + Vec3f{-h, 0, 0}),
                    smin.value(p + Vec3f{0,  h, 0}) - smin.value(p + Vec3f{0, -h, 0}),
                    smin.value(p + Vec3f{0, 0,  h}) - smin.value(p + Vec3f{0, 0, -h}),
                };
                const float gx_len = (gx * (0.5f / h)).length();
                if (gx_len > max_grad) max_grad = gx_len;
            }
        }
    }
    INFO("max |∇f| measured:", max_grad, " declared L:", L);
    CHECK(max_grad <= L + 1e-3f);  // small ε for FD noise
}

TEST_CASE("SmoothMin: k=0 has lipschitz() = max(L_a, L_b)") {
    // k=0 → hard min → no extra gradient contribution.
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3.0f, 0.0f, 0.0f});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.0f);
    CHECK(smin.lipschitz() == doctest::Approx(1.0f).epsilon(1e-5f));
}

TEST_CASE("SmoothMin: gradient magnitude respects lipschitz bound for analytic gradient") {
    // The analytic gradient returned by sample() must also respect
    // the declared Lipschitz constant (no defensive over-shoot).
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3.0f, 0.0f, 0.0f});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.5f);

    const float L = smin.lipschitz();
    float max_grad = 0.0f;
    for (int i = 0; i < 21; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                const float x = 1.5f + static_cast<float>(i - 10) * 0.10f;
                const float y = static_cast<float>(j - 3) * 0.10f;
                const float z = static_cast<float>(k - 3) * 0.10f;
                const Vec3f p{x, y, z};
                const auto s = smin.sample(p);
                const float gl = s.gradient.length();
                if (gl > max_grad) max_grad = gl;
            }
        }
    }
    INFO("max analytic |∇f|:", max_grad, " declared L:", L);
    CHECK(max_grad <= L + 1e-5f);
}

TEST_CASE("SmoothMin: value at blend midpoint matches IQ formula") {
    // At the midpoint (1.5, 0, 0):
    //   a = 0.5, b = 0.5, k = 0.5
    //   h = clamp(0.5 + 0.5·(0.5-0.5)/0.5, 0, 1) = 0.5
    //   v = 0.5 + 0·0.5 - 0.5·0.5·0.5·0.25 = 0.5 - 0.03125 = 0.46875
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {3.0f, 0.0f, 0.0f});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), 0.5f);
    CHECK(smin.value({1.5f, 0.0f, 0.0f}) == doctest::Approx(0.46875f).epsilon(1e-5f));
}

// ===========================================================================
// Bug 1 — SmoothMinSDF analytic-vs-FD gradient coherence
//
// The previous implementation returned ∇v = (1-h)∇a + h∇b, omitting the
// (5/8)·(∇b-∇a)·(2h-1) term that comes from the chain rule through h.
// The comment claiming "(1-2h) cancels exactly" was FALSE.
//
// This test samples the field on a fine grid (step ≤ 0.01) inside the
// blend band, computes |∇v| via central finite differences, and asserts
// that the analytic gradient returned by sample() agrees with the FD
// gradient to within a tight tolerance. The previous implementation
// fails this test in the blend band (where 2h-1 ≠ 0).
// ===========================================================================

TEST_CASE("SmoothMin: analytic gradient agrees with finite-difference gradient") {
    // Two unit spheres offset by 0.8 along X; blend band is around (0.4, 0, 0).
    const float k = 0.5f;
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {0.8f, 0.0f, 0.0f});
    auto smin = sdf_smooth_union(std::move(a), std::move(b), k);

    const float h_step = 1e-4f;   // FD step
    float max_diff = 0.0f;        // max ||∇analytic - ∇FD||
    float max_grad = 0.0f;        // max |∇| (FD)
    Vec3f worst_p{0, 0, 0};
    Vec3f worst_ga{0, 0, 0};
    Vec3f worst_gfd{0, 0, 0};

    // Dense grid centered on the blend region (0.4, 0, 0), step 0.01.
    for (int i = 0; i <= 80; ++i) {
        for (int j = 0; j <= 20; ++j) {
            for (int kk = 0; kk <= 20; ++kk) {
                const float x = 0.4f + (static_cast<float>(i - 40) * 0.01f);
                const float y = (static_cast<float>(j - 10) * 0.01f);
                const float z = (static_cast<float>(kk - 10) * 0.01f);
                const Vec3f p{x, y, z};

                // Skip points where the field is far from the blend band
                // (where h is clamped and the test is trivially true).
                const float va = smin.value(p);
                if (std::abs(va) > 2.0f * k) continue;

                // Skip points near the blend-band BOUNDARY (|b-a| ≈ k).
                // At the boundary the field has a kink (gradient is
                // discontinuous: ∇h transitions from (0.5/k)·(∇b-∇a) to 0),
                // so FD averages across the discontinuity and won't match
                // the analytic gradient. We exclude a margin of 0.05·k
                // around the boundary.
                auto sa = make_sphere(1.0f);
                auto sb = sdf_translate(make_sphere(1.0f), {0.8f, 0.0f, 0.0f});
                const float diff = std::abs(sb.value(p) - sa.value(p));
                if (diff > 0.95f * k) continue;

                // Finite-difference gradient (central differences).
                const Vec3f gfd{
                    (smin.value(p + Vec3f{ h_step, 0, 0}) -
                     smin.value(p + Vec3f{-h_step, 0, 0})) * (0.5f / h_step),
                    (smin.value(p + Vec3f{0,  h_step, 0}) -
                     smin.value(p + Vec3f{0, -h_step, 0})) * (0.5f / h_step),
                    (smin.value(p + Vec3f{0, 0,  h_step}) -
                     smin.value(p + Vec3f{0, 0, -h_step})) * (0.5f / h_step),
                };

                // Analytic gradient from sample().
                const auto s = smin.sample(p);
                const Vec3f ga = s.gradient;

                const float diff_g = (ga - gfd).length();
                if (diff_g > max_diff) {
                    max_diff = diff_g;
                    worst_p = p;
                    worst_ga = ga;
                    worst_gfd = gfd;
                }
                const float gfd_len = gfd.length();
                if (gfd_len > max_grad) max_grad = gfd_len;
            }
        }
    }
    INFO("max ||∇analytic - ∇FD|| =", max_diff,
         " max |∇FD| =", max_grad,
         " worst p =", worst_p.x, worst_p.y, worst_p.z,
         " ∇analytic =", worst_ga.x, worst_ga.y, worst_ga.z,
         " ∇FD =", worst_gfd.x, worst_gfd.y, worst_gfd.z);
    // Tight tolerance: the analytic gradient must agree with FD to
    // within 0.05 (5% of unit gradient) in the interior of the blend band.
    CHECK(max_diff <= 0.05f);
}

TEST_CASE("SmoothMin: lipschitz bound holds for k in {0.1, 0.5, 1.0} and offsets {0.5k, k, 2k}") {
    // For each (k, offset) pair, sample the field on a dense grid and
    // verify max|∇v| (FD) ≤ declared L + ε.
    struct TestCase { float k; float offset; };
    const TestCase cases[] = {
        {0.1f, 0.05f}, {0.1f, 0.10f}, {0.1f, 0.20f},
        {0.5f, 0.25f}, {0.5f, 0.50f}, {0.5f, 1.00f},
        {1.0f, 0.50f}, {1.0f, 1.00f}, {1.0f, 2.00f},
    };

    const float h_step = 1e-4f;
    for (const auto& tc : cases) {
        auto a = make_sphere(1.0f);
        auto b = sdf_translate(make_sphere(1.0f), {tc.offset, 0.0f, 0.0f});
        auto smin = sdf_smooth_union(std::move(a), std::move(b), tc.k);

        const float L = smin.lipschitz();
        float max_grad = 0.0f;
        // Grid centered on the blend midpoint.
        const float cx = tc.offset * 0.5f;
        // Use a grid proportional to k (so we always sample inside the
        // blend band). 41×11×11 = 4961 points.
        for (int i = 0; i <= 40; ++i) {
            for (int j = 0; j <= 10; ++j) {
                for (int kk = 0; kk <= 10; ++kk) {
                    const float x = cx + (static_cast<float>(i - 20) * (tc.k * 0.05f));
                    const float y = (static_cast<float>(j - 5) * (tc.k * 0.05f));
                    const float z = (static_cast<float>(kk - 5) * (tc.k * 0.05f));
                    const Vec3f p{x, y, z};
                    const Vec3f gfd{
                        (smin.value(p + Vec3f{ h_step, 0, 0}) -
                         smin.value(p + Vec3f{-h_step, 0, 0})) * (0.5f / h_step),
                        (smin.value(p + Vec3f{0,  h_step, 0}) -
                         smin.value(p + Vec3f{0, -h_step, 0})) * (0.5f / h_step),
                        (smin.value(p + Vec3f{0, 0,  h_step}) -
                         smin.value(p + Vec3f{0, 0, -h_step})) * (0.5f / h_step),
                    };
                    const float gl = gfd.length();
                    if (gl > max_grad) max_grad = gl;
                }
            }
        }
        INFO("k =", tc.k, " offset =", tc.offset,
             " L =", L, " max|∇FD| =", max_grad);
        CHECK(max_grad <= L + 1e-3f);  // small ε for FD noise
    }
}

TEST_CASE("SmoothMin: UnionSDF and SubtractionSDF propagate Lipschitz correctly") {
    // Verify that Union and Subtraction report max(L_a, L_b) — they
    // pick one of the two children's gradients, so the result's
    // Lipschitz is the max of the children's.
    auto sph = make_sphere(1.0f);
    CHECK(sph.lipschitz() == 1.0f);

    auto twisted = sdf_twist(make_box({1, 1, 1}), 0.5f);
    CHECK(twisted.lipschitz() > 1.0f);  // Twist increases L

    auto u = sdf_union(make_sphere(1.0f), sdf_twist(make_box({1, 1, 1}), 0.5f));
    CHECK(u.lipschitz() == doctest::Approx(twisted.lipschitz()).epsilon(1e-5f));

    auto sub = sdf_subtract(make_sphere(1.0f), sdf_twist(make_box({1, 1, 1}), 0.5f));
    CHECK(sub.lipschitz() == doctest::Approx(twisted.lipschitz()).epsilon(1e-5f));

    auto inter = sdf_intersect(make_sphere(1.0f), sdf_twist(make_box({1, 1, 1}), 0.5f));
    CHECK(inter.lipschitz() == doctest::Approx(twisted.lipschitz()).epsilon(1e-5f));
}

// ===========================================================================
// Property-based families (mini-traccia PARTE 2, area 1 — SDF operators).
//
// Infrastructure: tests/property/property_prng.hpp — seeded, deterministic,
// portable (hand-rolled mappings over the standardized mt19937 core). The
// seed is part of each test and is printed on every failure together with
// the iteration, the full tree (describe()) and the violated check, so a
// failure is a reproducible configuration, like a pinned diagnosis.
//
// 25 iterations per family, FIXED (PARTE 3) — the suite may not grow
// silently.
//
// Supported input class, declared up front (an input outside these
// ranges would be a GENERATOR bug, classified as such — never reported
// as a kernel bug):
//   * sphere radius            r  in [0.30, 1.20]
//   * box half-extent (per axis) e in [0.25, 1.00]
//   * cylinder radius          r  in [0.25, 0.70], height h in [0.60, 1.60]
//   * translate offset (per axis) in [-1.50, 1.50]
//   * tree operators: union / subtract / translate only, depth <= 3
//     (<= 8 leaves) — exactly the mini-traccia family composition; the
//     smooth union gets its own dedicated k = 0 property below.
// ===========================================================================

#include "property/property_prng.hpp"

#include <cmath>
#include <vector>

namespace {

namespace prop = CAD_0::test::property;

// One generated leaf, with everything needed to sample NEAR ITS SURFACE
// (the asymmetric-pair machinery below).
struct LeafRec {
    enum class Kind { Sphere, Box, Cylinder } kind;
    Vec3f center{};
    Vec3f dim{};  // sphere: (r,r,r); box: extent; cylinder: (r, h, r)
};

LeafRec gen_leaf(prop::Prng& prng) {
    LeafRec lf;
    lf.center = Vec3f{prng.frange(-1.5f, 1.5f),
                      prng.frange(-1.5f, 1.5f),
                      prng.frange(-1.5f, 1.5f)};
    switch (prng.int_below(3u)) {
    case 0:
        lf.kind = LeafRec::Kind::Sphere;
        lf.dim = Vec3f{prng.frange(0.30f, 1.20f), 0.0f, 0.0f};
        break;
    case 1:
        lf.kind = LeafRec::Kind::Box;
        lf.dim = Vec3f{prng.frange(0.25f, 1.00f),
                       prng.frange(0.25f, 1.00f),
                       prng.frange(0.25f, 1.00f)};
        break;
    default:
        lf.kind = LeafRec::Kind::Cylinder;
        lf.dim = Vec3f{prng.frange(0.25f, 0.70f), prng.frange(0.60f, 1.60f), 0.0f};
        break;
    }
    return lf;
}

SDFBody leaf_body(const LeafRec& lf) {
    switch (lf.kind) {
    case LeafRec::Kind::Sphere:
        return sdf_translate(make_sphere(lf.dim.x), lf.center);
    case LeafRec::Kind::Box:
        return sdf_translate(make_box(lf.dim), lf.center);
    case LeafRec::Kind::Cylinder:
        return sdf_translate(make_cylinder(lf.dim.x, lf.dim.y), lf.center);
    }
    return make_sphere(0.5f);  // unreachable; silences -Wreturn-type
}

// The declared blend range for the k > 0 extension families below
// (FASE 1 of the property extension): k in [0.10, 1.00]. Margins:
//   * k small (0.10) -> the blend band |f_a - f_b| < k is narrow and
//     the field stays close to the hard min (dip <= k/16 = 0.00625).
//   * k large (1.00) -> the band is wide; see the bounds() note in the
//     extension header below the three properties.
constexpr float kBlendKMin = 0.10f;
constexpr float kBlendKMax = 1.00f;

// One recorded smooth_min node of a generated tree: the leaves under
// each operand, for crevice-targeted sampling (the band |f_a - f_b| < k
// — the only region where the blend deviates from the hard min — opens
// between the two operand subtrees).
struct BlendRec {
    std::vector<LeafRec> side_a, side_b;
};

struct TreeSpec {
    SDFBody body;
    Bboxf bounds;
    std::vector<LeafRec> leaves;
    std::vector<BlendRec> blends;  // non-empty only with allow_blend
    float blend_k_sum = 0.0f;      // sum of the blend k's (dilation envelope = sum/16)
};

// allow_blend = false (default) reproduces the ORIGINAL family bit
// exactly: one coin draw per internal node, union or subtract — the
// pinned seeds of the properties above keep their meaning. With
// allow_blend = true the draw becomes a 3-way choice adding smooth_min
// nodes with k in the declared range (FASE 1).
TreeSpec gen_tree(prop::Prng& prng, int depth, bool allow_blend = false) {
    TreeSpec spec;
    if (depth == 0 || prng.int_below(4u) == 0u) {
        spec.leaves.push_back(gen_leaf(prng));
        spec.body = leaf_body(spec.leaves.back());
    } else {
        TreeSpec a = gen_tree(prng, depth - 1, allow_blend);
        TreeSpec b = gen_tree(prng, depth - 1, allow_blend);
        if (!allow_blend) {
            if (prng.coin()) {
                spec.body = sdf_union(std::move(a.body), std::move(b.body));
            } else {
                spec.body = sdf_subtract(std::move(a.body), std::move(b.body));
            }
        } else {
            const auto op = prng.int_below(3u);
            if (op == 0u) {
                spec.body = sdf_union(std::move(a.body), std::move(b.body));
            } else if (op == 1u) {
                spec.body = sdf_subtract(std::move(a.body), std::move(b.body));
            } else {
                const float k = prng.frange(kBlendKMin, kBlendKMax);
                spec.body = sdf_smooth_union(std::move(a.body), std::move(b.body), k);
                spec.blends.push_back(BlendRec{a.leaves, b.leaves});
                spec.blend_k_sum += k;
            }
        }
        spec.leaves = std::move(a.leaves);
        spec.leaves.insert(spec.leaves.end(),
                           std::make_move_iterator(b.leaves.begin()),
                           std::make_move_iterator(b.leaves.end()));
        spec.blends.insert(spec.blends.end(),
                           std::make_move_iterator(a.blends.begin()),
                           std::make_move_iterator(a.blends.end()));
        spec.blends.insert(spec.blends.end(),
                           std::make_move_iterator(b.blends.begin()),
                           std::make_move_iterator(b.blends.end()));
        spec.blend_k_sum += a.blend_k_sum + b.blend_k_sum;
    }
    // The kernel's own bounds() is the source of truth for the family.
    spec.bounds = spec.body.bounds();
    return spec;
}

Vec3f random_dir(prop::Prng& prng) {
    for (;;) {
        const Vec3f d{prng.frange(-1.0f, 1.0f),
                      prng.frange(-1.0f, 1.0f),
                      prng.frange(-1.0f, 1.0f)};
        const float l = d.length();
        if (l > 0.2f && l < 1.0f) return d * (1.0f / l);
    }
}

float axis_of(const Vec3f& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

// A point on (or within eps of) THIS leaf's surface sheet, chosen so
// that the point may land on either side of the sheet (inside/outside):
// the near-surface shell of a SPECIFIC leaf. Geometric exactness of the
// sheet is not required — only proximity to one leaf's boundary, far
// from the other leaves' — that is what makes the cross-leaf pairs
// straddle the CSG creases.
Vec3f near_surface(const LeafRec& lf, prop::Prng& prng, float eps) {
    switch (lf.kind) {
    case LeafRec::Kind::Sphere: {
        const float r = lf.dim.x * (1.0f + prng.frange(-eps, eps));
        return lf.center + random_dir(prng) * r;
    }
    case LeafRec::Kind::Box: {
        // One dominant face: on `axis` at +- extent*(1 +- eps); the
        // other two coordinates are uniform inside the face.
        const int axis = static_cast<int>(prng.int_below(3u));
        const float s = (prng.coin() ? 1.0f : -1.0f) *
                        (1.0f + prng.frange(-eps, eps));
        const Vec3f e = lf.dim;
        Vec3f local{prng.frange(-e.x, e.x),
                    prng.frange(-e.y, e.y),
                    prng.frange(-e.z, e.z)};
        local = Vec3f{axis == 0 ? e.x * s : local.x,
                      axis == 1 ? e.y * s : local.y,
                      axis == 2 ? e.z * s : local.z};
        return lf.center + local;
    }
    case LeafRec::Kind::Cylinder: {
        // Lateral sheet: radius r*(1 +- eps), y uniform in the height.
        const float r = lf.dim.x * (1.0f + prng.frange(-eps, eps));
        const float h2 = lf.dim.y * 0.5f;
        const Vec3f d = random_dir(prng);
        const float rl = std::sqrt(d.x * d.x + d.z * d.z);
        const float rx = rl > 1e-6f ? d.x / rl : 1.0f;
        const float rz = rl > 1e-6f ? d.z / rl : 0.0f;
        return lf.center + Vec3f{rx * r, prng.frange(-h2, h2), rz * r};
    }
    }
    return lf.center;
}

// A point on THIS leaf's boundary shell, on the side FACING `target`
// (eps = relative radial slack). Same exactness note as near_surface:
// only proximity matters. These points sit at the entrance of the
// crevice between the two operands of a blend — where the blend band
// (|f_a - f_b| < k) actually lives. Ray/surface hit per leaf kind:
// sphere exact; box = first face plane crossed; cylinder = lateral
// sheet vs caps, first crossed (the finite cylinder is convex, so the
// exit hit is the min of the two parameters).
Vec3f surface_toward(const LeafRec& lf, const Vec3f& target, prop::Prng& prng, float eps) {
    Vec3f dir = target - lf.center;
    const float dl = dir.length();
    if (dl < 1e-6f) {
        dir = random_dir(prng);
    } else {
        dir = dir * (1.0f / dl);
    }
    float t_hit = 1e30f;
    switch (lf.kind) {
    case LeafRec::Kind::Sphere:
        t_hit = lf.dim.x;
        break;
    case LeafRec::Kind::Box: {
        const float d[3] = {std::abs(dir.x), std::abs(dir.y), std::abs(dir.z)};
        const float e[3] = {lf.dim.x, lf.dim.y, lf.dim.z};
        for (int i = 0; i < 3; ++i) {
            if (d[i] > 1e-6f) t_hit = std::min(t_hit, e[i] / d[i]);
        }
        break;
    }
    case LeafRec::Kind::Cylinder: {
        const float rl = std::sqrt(dir.x * dir.x + dir.z * dir.z);
        if (rl > 1e-6f) t_hit = std::min(t_hit, lf.dim.x / rl);
        if (std::abs(dir.y) > 1e-6f) {
            t_hit = std::min(t_hit, lf.dim.y * 0.5f / std::abs(dir.y));
        }
        break;
    }
    }
    if (t_hit > 1e29f) t_hit = lf.dim.x;  // defensive: dir ~ tangential
    const float slack = 1.0f + prng.frange(-eps, eps);
    return lf.center + dir * (t_hit * slack);
}

Bboxf expand_bbox(const Bboxf& b, float frac) {
    const Vec3f c{(b.min.x + b.max.x) * 0.5f,
                  (b.min.y + b.max.y) * 0.5f,
                  (b.min.z + b.max.z) * 0.5f};
    const Vec3f h{(b.max.x - b.min.x) * 0.5f * (1.0f + frac),
                  (b.max.y - b.min.y) * 0.5f * (1.0f + frac),
                  (b.max.z - b.min.z) * 0.5f * (1.0f + frac)};
    return Bboxf{c - h, c + h};
}

Vec3f uniform_in(prop::Prng& prng, const Bboxf& b) {
    return Vec3f{prng.frange(b.min.x, b.max.x),
                 prng.frange(b.min.y, b.max.y),
                 prng.frange(b.min.z, b.max.z)};
}

// The pair categories, declared (the smin lesson made structural):
//   * "uniform"    — both points uniform in the padded hull: broad
//                    coverage, catches gross violations.
//   * "local"      — small separation rho in [1e-3, 0.25]: probes the
//                    inequality at short range, where the divide by
//                    |p - q| is most sensitive.
//   * "cross-leaf" — THE ASYMMETRIC CATEGORY: each point sits on the
//                    near-surface shell (eps up to 8%) of a DIFFERENT
//                    leaf, independently inside/outside. These pairs
//                    straddle the crease of the piecewise min/max —
//                    exactly the regime where the 2026-10-16 smooth-min
//                    clamp-sign bug lived unnoticed under symmetric
//                    sampling. Uniform/local sampling misses it.
//   * "crease"     — blended trees only: both endpoints on the FACING
//                    sheets of the two operands of one blend node
//                    (surface_toward). These pairs straddle the blend
//                    band |f_a - f_b| < k — the only region where the
//                    smooth min deviates from the hard min, and where
//                    its gradient legitimately exceeds 1.
struct PointPair {
    Vec3f p, q;
    const char* how;
};

// blended = false (default) keeps the original 3-category draw — the
// pinned seeds above replay bit-identically. blended = true adds the
// crease category (4-way draw).
std::vector<PointPair> gen_pairs(const TreeSpec& t, prop::Prng& prng, int n,
                                 bool blended = false) {
    std::vector<PointPair> out;
    out.reserve(static_cast<std::size_t>(n));
    const Bboxf bb = expand_bbox(t.bounds, 0.25f);
    for (int i = 0; i < n; ++i) {
        const auto mode = prng.int_below(blended ? 4u : 3u);
        if (mode == 0u) {
            out.push_back({uniform_in(prng, bb), uniform_in(prng, bb), "uniform"});
        } else if (mode == 1u) {
            const Vec3f p = uniform_in(prng, bb);
            const float rho = prng.frange(1e-3f, 0.25f);
            out.push_back({p, p + random_dir(prng) * rho, "local"});
        } else if (mode == 3u && !t.blends.empty()) {
            const auto nb = static_cast<std::uint32_t>(t.blends.size());
            const BlendRec& br = t.blends[prng.int_below(nb)];
            const auto na = static_cast<std::uint32_t>(br.side_a.size());
            const auto nbs = static_cast<std::uint32_t>(br.side_b.size());
            const LeafRec& la = br.side_a[prng.int_below(na)];
            const LeafRec& lb = br.side_b[prng.int_below(nbs)];
            out.push_back({surface_toward(la, lb.center, prng, 0.08f),
                           surface_toward(lb, la.center, prng, 0.08f), "crease"});
        } else if (t.leaves.size() >= 2) {
            const auto na = static_cast<std::uint32_t>(t.leaves.size());
            const std::uint32_t a = prng.int_below(na);
            std::uint32_t b = prng.int_below(na);
            if (b == a) b = (a + 1u) % na;
            out.push_back({near_surface(t.leaves[a], prng, 0.08f),
                           near_surface(t.leaves[b], prng, 0.08f), "cross-leaf"});
        } else {
            out.push_back({uniform_in(prng, bb), uniform_in(prng, bb), "uniform"});
        }
    }
    return out;
}

} // namespace

// --- Property (a): Lipschitz -----------------------------------------------
// For every member of the seeded tree family and every sampled pair:
//   |f(p) - f(q)| <= lipschitz() * |p - q|   (+ a pinned float slack).
TEST_CASE("property: Lipschitz bound holds over seeded random CSG trees") {
    const std::uint32_t seed = 0x5EED0001u;
    prop::Prng prng(seed);
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec t = gen_tree(prng, 3);
        const float L = t.body.lipschitz();
        INFO("seed=", seed, " iter=", iter, " L=", L);
        INFO("tree: ", t.body.describe());
        for (const PointPair& pp : gen_pairs(t, prng, 60)) {
            const double fp = t.body.value(pp.p);
            const double fq = t.body.value(pp.q);
            const Vec3f d3 = pp.p - pp.q;
            const double dist = std::sqrt(static_cast<double>(d3.x * d3.x) +
                                          static_cast<double>(d3.y * d3.y) +
                                          static_cast<double>(d3.z * d3.z));
            INFO("pair(", pp.how, "): p=(", pp.p.x, ",", pp.p.y, ",", pp.p.z,
                 ") q=(", pp.q.x, ",", pp.q.y, ",", pp.q.z,
                 ") f(p)=", fp, " f(q)=", fq, " |p-q|=", dist);
            // Float-evaluation slack, pinned with its margin: in this
            // sampling volume |f| <= ~5, so ulp(|f|) <= ~4.8e-7 and the
            // two evaluations + the distance contribute a few ulp —
            // 1e-5 is ~20x the worst single-evaluation rounding, two
            // orders of magnitude below any real Lipschitz violation.
            // The contract itself is exact; only the arithmetic is not.
            INFO("check: |f(p) - f(q)| <= lipschitz()*|p-q| + 1e-5 (pinned float slack)");
            CHECK(std::abs(fp - fq) <= static_cast<double>(L) * dist + 1e-5);
        }
    }
}

// --- Property (b): smooth_min(k=0) == hard min, point per point ----------
// Bit-exact: both operators must reduce to min(a(p), b(p)) with the
// same tie semantics (the first operand wins a tie in both — see
// UnionSDF::sample and the k <= 0 branch of SmoothMinSDF::sample), and
// a(p)/b(p) are the same float expressions in both trees, so equal
// bits are REQUIRED, not approximated.
TEST_CASE("property: smooth_min(k=0) equals the hard union pointwise, bit-exact") {
    const std::uint32_t seed = 0x5EED0002u;
    prop::Prng prng(seed);
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec ta = gen_tree(prng, 2);
        const TreeSpec tb = gen_tree(prng, 2);
        // SDFBody's copy constructor deep-clones; the composition
        // helpers take operands by value.
        auto smooth = sdf_smooth_union(ta.body, tb.body, 0.0f);
        auto hard = sdf_union(ta.body, tb.body);
        INFO("seed=", seed, " iter=", iter);
        INFO("A: ", ta.body.describe());
        INFO("B: ", tb.body.describe());
        const Bboxf bb = expand_bbox(
            Bboxf{Vec3f{std::min(ta.bounds.min.x, tb.bounds.min.x),
                              std::min(ta.bounds.min.y, tb.bounds.min.y),
                              std::min(ta.bounds.min.z, tb.bounds.min.z)},
                        Vec3f{std::max(ta.bounds.max.x, tb.bounds.max.x),
                              std::max(ta.bounds.max.y, tb.bounds.max.y),
                              std::max(ta.bounds.max.z, tb.bounds.max.z)}},
            0.25f);
        for (int k = 0; k < 40; ++k) {
            // Asymmetric sampling: near A's leaves, near B's leaves,
            // and uniform in the shared hull — the regime where the
            // clamp-sign bug hid (2026-10-16).
            Vec3f p;
            const auto mode = prng.int_below(3u);
            if (mode == 0u || ta.leaves.empty()) {
                p = uniform_in(prng, bb);
            } else if (mode == 1u || tb.leaves.empty()) {
                p = near_surface(ta.leaves[prng.int_below(
                                      static_cast<std::uint32_t>(ta.leaves.size()))],
                                 prng, 0.08f);
            } else {
                p = near_surface(tb.leaves[prng.int_below(
                                      static_cast<std::uint32_t>(tb.leaves.size()))],
                                 prng, 0.08f);
            }
            const float vs = smooth.value(p);
            const float vh = hard.value(p);
            INFO("p=(", p.x, ",", p.y, ",", p.z, ") smooth=", vs, " hard=", vh);
            INFO("check: smooth_min(k=0).value(p) == union.value(p), bit-exact");
            CHECK(vs == vh);
        }
    }
}

// --- Property (c): strict positivity outside bounds() --------------------
// P1-flavored enclosure contract at the field level: every point at
// least delta (in [1e-3, 0.75], declared margin) outside the tree's
// own bounds() must evaluate strictly positive. Valid by construction
// for this family: union keeps the hull, subtract keeps A's bounds,
// translate shifts them — and every primitive's bounds() contains its
// zero sub-level set strictly.
TEST_CASE("property: f is strictly positive outside the tree bounds") {
    const std::uint32_t seed = 0x5EED0003u;
    prop::Prng prng(seed);
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec t = gen_tree(prng, 3);
        const Bboxf bb = t.bounds;
        INFO("seed=", seed, " iter=", iter,
             " bounds=[", bb.min.x, ",", bb.min.y, ",", bb.min.z, "]..[",
             bb.max.x, ",", bb.max.y, ",", bb.max.z, "]");
        INFO("tree: ", t.body.describe());
        for (int k = 0; k < 40; ++k) {
            const int axis = static_cast<int>(prng.int_below(3u));
            const bool lo_face = prng.coin();
            const float delta = prng.frange(1e-3f, 0.75f);
            // Tangential coordinates: uniform inside the face.
            Vec3f p = uniform_in(prng, bb);
            const float face = axis_of(lo_face ? bb.min : bb.max, axis);
            const float out = face + (lo_face ? -delta : delta);
            p = Vec3f{axis == 0 ? out : p.x,
                      axis == 1 ? out : p.y,
                      axis == 2 ? out : p.z};
            INFO("k=", k, " axis=", axis, " lo_face=", lo_face,
                 " delta=", delta, " p=(", p.x, ",", p.y, ",", p.z, ")",
                 " f=", t.body.value(p));
            INFO("check: f(p) > 0 for p outside bounds() by delta >= 1e-3");
            CHECK(t.body.value(p) > 0.0f);
        }
    }
}

// ===========================================================================
// Property families — EXTENSION: smooth blends, smin k > 0 (FASE 1).
//
// The generator above is EXTENDED, not replaced: gen_tree(...,
// allow_blend = true) adds smooth_min nodes with k in the declared range
// [kBlendKMin, kBlendKMax] = [0.10, 1.00]; with allow_blend = false
// (default) the draw sequence is bit-identical to the original family,
// so the pinned seeds of the properties above keep their meaning.
//
// Algebra of the kernel's polynomial (operators.hpp), u = (f_b-f_a)/k:
//     v - min(f_a, f_b) = k * phi(u),  phi(u) = (8u - 7u^2 - 1)/16
//   * dip BELOW min:   max k/16 at u = 0 (equal fields) — the blend
//     dilates the solid;
//   * tuck ABOVE min:  max 9k/112 at |u| = 4/7 — the flank slightly
//     thinner than the hard union;
//   * v == min at |u| = 1/7 and at the band edge |u| = 1.
//
// EFFECT ON bounds() — verified before these trees are used anywhere
// else (FASE 1.1). field.hpp declares bounds() as "bounding box of the
// region where the field can be NEGATIVE". SmoothMinSDF::bounds()
// returns the plain hull of the children's bounds, but the k/16 dip
// means the negative region can escape the hull by up to k/16 per
// blend level wherever the two operand fields stay within k/7 of each
// other up to the hull boundary (near-coincident surfaces; nested
// blends accumulate the envelope sum(k_i)/16). Empirical pins
// (standalone driver smin_slope_probe.cpp, kept with the other
// measurement drivers OUTSIDE the repo; GCC 14.2 Release
// -ffp-contract=off):
//   concentric unit spheres, blend k: zero crossing at EXACTLY
//     k=0.10 -> r=1.006250    k=0.50 -> r=1.031250    k=1.00 -> r=1.062500
//   (each = 1 + k/16), while bounds() = [-1,1]^3 and the mesh-extract
//   volume adds only a 1%-of-extent pad (0.02 here): the escape exceeds
//   the pad for k >~ 0.32 at this extent. This is a GAP against the
//   field.hpp bounds() contract — REPORTED, not fixed here (a bounds()
//   change would shift every pinned MC grid layout; the fix needs its
//   own decision + sweep). Consequences in this suite:
//     * property (c) above does NOT transfer to blended trees; the
//       blended positivity property (g) below uses the explicit
//       sum(k_i)/16 envelope instead;
//     * the bridge smooth-union family (test_bridge.cpp) uses chain
//       geometries whose near-coincident surfaces (crevices/necks) are
//       strictly INTERIOR to the hull — probe (C): min f on the padded
//       hull faces = +0.0155..+0.0202 — so the extraction volume is
//       safe for that declared class.
//
// EMPIRICAL SLOPE of the polynomial (FASE 1.3, dedicated guinea pigs —
// the standalone driver named above): two unit spheres (gap 0.4 and
// 0.05), dense plane + axis sampling, k sweep:
//     k=0.10 -> max|grad v| = 1.749998     k=0.25 -> 1.746500
//     k=0.50 -> 1.749996     k=0.70 -> 1.749996     k=1.00 -> 1.749995
// The observed maximum sits ON the analytic sup 7/4 = 1.75 — approached
// at the band edge |x| = k/2 on the axis, where the two sphere
// gradients are anti-aligned — and is k-independent, exactly as the
// kernel's derivation states. The DECLARED bound (2.5 * max, see
// SmoothMinSDF::lipschitz) therefore has ~43% headroom over the worst
// observation: RELIABLE. No fallback test bound is needed; per the
// extension rules the fallback (2.5*max computed by the test) would
// only be adopted — documented as such — if the declared bound failed.
// ===========================================================================

// --- Property (d): Lipschitz on blended trees ------------------------------
// Same contract as property (a), on the blended family — the bound is
// THE KERNEL'S OWN DECLARED tree.lipschitz(): if this fails, the
// declared bound is the bug (the ray marcher steps on that number).
TEST_CASE("property: Lipschitz bound holds over seeded random blended CSG trees (smin k > 0)") {
    const std::uint32_t seed = 0x5EED0011u;
    prop::Prng prng(seed);
    int iters_with_blend = 0;
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec t = gen_tree(prng, 3, /*allow_blend=*/true);
        if (!t.blends.empty()) ++iters_with_blend;
        const float L = t.body.lipschitz();
        INFO("seed=", seed, " iter=", iter, " L=", L,
             " sum(k)=", t.blend_k_sum, " blends=", t.blends.size());
        INFO("tree: ", t.body.describe());
        for (const PointPair& pp : gen_pairs(t, prng, 60, /*blended=*/true)) {
            const double fp = t.body.value(pp.p);
            const double fq = t.body.value(pp.q);
            const Vec3f d3 = pp.p - pp.q;
            const double dist = std::sqrt(static_cast<double>(d3.x * d3.x) +
                                          static_cast<double>(d3.y * d3.y) +
                                          static_cast<double>(d3.z * d3.z));
            INFO("pair(", pp.how, "): p=(", pp.p.x, ",", pp.p.y, ",", pp.p.z,
                 ") q=(", pp.q.x, ",", pp.q.y, ",", pp.q.z,
                 ") f(p)=", fp, " f(q)=", fq, " |p-q|=", dist);
            // Same pinned float slack as property (a), same rationale.
            INFO("check: |f(p) - f(q)| <= lipschitz()*|p-q| + 1e-5 (pinned float slack)");
            CHECK(std::abs(fp - fq) <= static_cast<double>(L) * dist + 1e-5);
        }
    }
    // Family liveness for THIS seed (deterministic). Branching
    // analysis: P(member has >= 1 smin) follows f(d) = 3/4*[1/3 +
    // 2/3*(1-(1-f(d-1))^2)], f(0)=0 -> f(3) ~= 0.61, so ~15 of 25
    // members carry a blend (14 observed on this seed); the rest keep
    // the unblended contract covered under fresh seeds. Guard well
    // below the expectation, above degeneration.
    INFO("check: >= 10 of 25 members contain at least one smooth_min node");
    CHECK(iters_with_blend >= 10);
}

// --- Property (e): gradient magnitude vs the declared bound ---------------
// |grad f| <= tree.lipschitz() at sampled points — including crease
// points, where the blend gradient legitimately exceeds 1 (observed sup
// 7/4 per blend level; the declared bound keeps 2.5x per level).
TEST_CASE("property: gradient magnitude stays under the declared Lipschitz on blended trees") {
    const std::uint32_t seed = 0x5EED0012u;
    prop::Prng prng(seed);
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec t = gen_tree(prng, 3, /*allow_blend=*/true);
        const float L = t.body.lipschitz();
        INFO("seed=", seed, " iter=", iter, " L=", L,
             " sum(k)=", t.blend_k_sum, " blends=", t.blends.size());
        INFO("tree: ", t.body.describe());
        const Bboxf bb = expand_bbox(t.bounds, 0.25f);
        for (int j = 0; j < 60; ++j) {
            const auto mode = prng.int_below(3u);
            Vec3f p;
            const char* how = "uniform";
            if (mode == 1u) {
                const auto nl = static_cast<std::uint32_t>(t.leaves.size());
                p = near_surface(t.leaves[prng.int_below(nl)], prng, 0.08f);
                how = "near-surface";
            } else if (mode == 2u && !t.blends.empty()) {
                const auto nb = static_cast<std::uint32_t>(t.blends.size());
                const BlendRec& br = t.blends[prng.int_below(nb)];
                const auto na = static_cast<std::uint32_t>(br.side_a.size());
                const auto nbs = static_cast<std::uint32_t>(br.side_b.size());
                const LeafRec& la = br.side_a[prng.int_below(na)];
                const LeafRec& lb = br.side_b[prng.int_below(nbs)];
                p = surface_toward(la, lb.center, prng, 0.08f);
                how = "crease";
            } else {
                p = uniform_in(prng, bb);
            }
            const float gl = t.body.sample(p).gradient.length();
            INFO("point(", how, "): p=(", p.x, ",", p.y, ",", p.z,
                 ") |grad f|=", gl, " L=", L);
            // Pinned float slack, same discipline as (d): the contract is
            // exact, only the arithmetic is not.
            INFO("check: |grad f(p)| <= lipschitz() + 1e-5 (pinned float slack)");
            CHECK(gl <= L + 1e-5f);
        }
    }
}

// --- Property (f): guinea-pig slope sweep (the pinned empirical envelope) --
// The dedicated measurement of FASE 1.3, replayed as a regression: the
// max observed slope must sit UNDER the 7/4 analytic envelope (and the
// sweep must actually REACH the steep band edge, else the probe is
// dead). Plus the concentric pair that pins the k/16 dip geometry.
TEST_CASE("smin guinea pigs: empirical max slope pins the 7/4 analytic envelope (k sweep)") {
    // Two unit spheres at gap 0.4 (the probe verified the plane-sweep
    // maximum coincides with the axis). Axis sweep, step 2.5e-4: the
    // nearest sample to the band edge |x| = k/2 is within 1.25e-4, so
    // the observed max is >= 1.75 * (1 - 2*1.25e-4/0.10) = 1.7456.
    for (float k : {0.10f, 0.25f, 0.50f, 0.70f, 1.00f}) {
        auto a = sdf_translate(make_sphere(1.0f), Vec3f{-1.2f, 0, 0});
        auto b = sdf_translate(make_sphere(1.0f), Vec3f{+1.2f, 0, 0});
        auto v = sdf_smooth_union(std::move(a), std::move(b), k);
        float best = 0.0f;
        float at = 0.0f;
        for (double x = -2.2; x <= 2.2; x += 0.00025) {
            const Vec3f p{static_cast<float>(x), 0.0f, 0.0f};
            const float gl = v.sample(p).gradient.length();
            if (gl > best) { best = gl; at = p.x; }
        }
        INFO("k=", k, " max|grad v| on axis = ", best, " at x=", at);
        INFO("check: max|grad v| <= 7/4 + 1e-4 (analytic envelope)");
        CHECK(best <= 1.75f + 1e-4f);
        INFO("check: max|grad v| >= 1.70 (the sweep reaches the steep band edge)");
        CHECK(best >= 1.70f);
    }
    // The concentric pair: f_a == f_b everywhere (u = 0), so the field
    // is exactly f - k/16 and the zero set is the sphere r = 1 + k/16.
    // The crossing being OUTSIDE bounds() = [-1,1]^3 is the documented
    // contract gap — see the extension header (reported, not endorsed).
    for (float k : {0.10f, 0.50f, 1.00f}) {
        auto v = sdf_smooth_union(make_sphere(1.0f), make_sphere(1.0f), k);
        double lo = 1.0, hi = 1.0 + k / 8.0;
        for (int it = 0; it < 80; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (v.value(Vec3f{static_cast<float>(mid), 0, 0}) < 0.0f) lo = mid;
            else hi = mid;
        }
        const double crossing = 0.5 * (lo + hi);
        INFO("k=", k, " zero crossing at r = ", crossing,
             " (1 + k/16 = ", 1.0 + k / 16.0, ")");
        INFO("check: |crossing - (1 + k/16)| <= 1e-5 (the dip depth is exactly k/16)");
        CHECK(std::abs(crossing - (1.0 + k / 16.0)) <= 1e-5);
        INFO("check: crossing > 1 (the blended solid exits bounds() — documented gap, reported)");
        CHECK(crossing > 1.0);
    }
}

// --- Property (g): positivity outside bounds beyond the blend envelope -----
// f >= min(leaves) - sum(k_i)/16 holds by induction over the operators
// (union: equality; subtract: max(a,-b) >= min(a,b); smin: the k/16
// dip; translate: shift). Every leaf is a true-distance primitive, so a
// point outside the HULL by delta is outside every leaf's bounds by >=
// delta and each leaf field is >= delta there. Hence f(p) > 0 whenever
// delta > sum(k_i)/16: this VERIFIES the documented bounds() dilation
// effect from the safe side, per member, envelope included.
TEST_CASE("property: blended trees stay positive outside bounds() beyond the blend-dilation envelope") {
    const std::uint32_t seed = 0x5EED0013u;
    prop::Prng prng(seed);
    for (int iter = 0; iter < 25; ++iter) {
        const TreeSpec t = gen_tree(prng, 3, /*allow_blend=*/true);
        const float envelope = t.blend_k_sum / 16.0f;
        const Bboxf bb = t.bounds;
        INFO("seed=", seed, " iter=", iter, " sum(k)=", t.blend_k_sum,
             " envelope=sum(k)/16=", envelope,
             " bounds=[", bb.min.x, ",", bb.min.y, ",", bb.min.z, "]..[",
             bb.max.x, ",", bb.max.y, ",", bb.max.z, "]");
        INFO("tree: ", t.body.describe());
        for (int j = 0; j < 40; ++j) {
            const int axis = static_cast<int>(prng.int_below(3u));
            const bool lo_face = prng.coin();
            const float delta = prng.frange(envelope + 5e-3f, envelope + 0.30f);
            Vec3f p = uniform_in(prng, bb);
            const float face = axis_of(lo_face ? bb.min : bb.max, axis);
            const float out = face + (lo_face ? -delta : delta);
            p = Vec3f{axis == 0 ? out : p.x,
                      axis == 1 ? out : p.y,
                      axis == 2 ? out : p.z};
            INFO("j=", j, " axis=", axis, " lo_face=", lo_face,
                 " delta=", delta, " p=(", p.x, ",", p.y, ",", p.z, ")",
                 " f=", t.body.value(p));
            INFO("check: f(p) > 0 for p outside bounds() by delta > sum(k)/16");
            CHECK(t.body.value(p) > 0.0f);
        }
    }
}
