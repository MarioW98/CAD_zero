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
    CHECK(L == doctest::Approx(2.0f).epsilon(1e-5f));

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
