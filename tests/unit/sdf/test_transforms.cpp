// tests/unit/sdf/test_transforms.cpp
#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("Translate: shifts the field") {
    auto sph = make_sphere(1.0f);
    auto t   = sdf_translate(std::move(sph), {5, 0, 0});
    CHECK(t.value({5, 0, 0}) == doctest::Approx(-1.0f));
    CHECK(t.value({6, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(t.value({0, 0, 0}) == doctest::Approx(4.0f).epsilon(1e-5f));
}

TEST_CASE("Translate: bounds move with the field") {
    auto sph = make_sphere(2.0f);
    auto t   = sdf_translate(std::move(sph), {3, 0, 0});
    auto b = t.bounds();
    CHECK(b.min.x == doctest::Approx(1.0f));
    CHECK(b.max.x == doctest::Approx(5.0f));
}

TEST_CASE("Scale: scales the field") {
    auto sph = make_sphere(1.0f);
    auto s   = sdf_scale(std::move(sph), 2.0f);
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(s.value({3, 0, 0}) == doctest::Approx(1.0f));
}

TEST_CASE("Rotate: rotates the field") {
    auto box = make_box({1, 1, 1});
    // Rotate 90 degrees around Z. A box of extent 1,1,1 looks the same
    // after this rotation, so we expect the field at (1,0,0) to equal
    // the field at (0,1,0) before rotation (i.e. 0).
    auto q = Quatf::from_axis_angle({0, 0, 1}, 3.14159265358979f / 2.0f);
    auto r = sdf_rotate(std::move(box), q);
    CHECK(r.value({1, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(r.value({0, 1, 0}) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("Twist: changes the field along the twist axis") {
    auto box = make_box({1, 5, 1});
    auto t   = sdf_twist(std::move(box), 0.5f);
    // Sanity: at origin the twist is zero so the field equals the original box's
    CHECK(t.value({0, 0, 0}) < 0.0f); // Inside the box
    CHECK(t.lipschitz() > 1.0f); // Twist increases lipschitz constant
}

TEST_CASE("Translate + Scale composition") {
    auto sph = make_sphere(1.0f);
    auto s   = sdf_scale(std::move(sph), 2.0f);
    auto st  = sdf_translate(std::move(s), {10, 0, 0});
    CHECK(st.value({10, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(st.value({12, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(st.value({0, 0, 0})  == doctest::Approx(8.0f).epsilon(1e-5f));
}

// ===========================================================================
// ScaleSDF Lipschitz invariant regression tests (added after the bug fix).
//
// Bug: the previous implementation reported lipschitz = scale · child_lipschitz.
// For a unit-Lipschitz child (e.g. SphereSDF) scaled by s > 1, this gave
// lipschitz = s, which is wrong. The scaled field f(x) = s · g(x/s) has
// ∇f(x) = ∇g(x/s), so |∇f| = |∇g| ≤ lipschitz(g). The Lipschitz constant
// of the scaled field is the same as the child's, NOT scaled.
// ===========================================================================

TEST_CASE("ScaleSDF: lipschitz is preserved (not multiplied by scale)") {
    auto sph = make_sphere(1.0f);
    auto s2  = sdf_scale(make_sphere(1.0f), 2.0f);   // scale > 1
    auto s05 = sdf_scale(make_sphere(1.0f), 0.5f);   // scale < 1
    CHECK(sph.lipschitz() == 1.0f);
    CHECK(s2.lipschitz()  == 1.0f);  // was 2.0f before fix
    CHECK(s05.lipschitz() == 1.0f);  // was 0.5f before fix (also wrong!)
}

TEST_CASE("ScaleSDF: gradient magnitude is ≤ 1 (coherent with lipschitz)") {
    // For a unit-Lipschitz child, the scaled field must also have |∇f| ≤ 1.
    auto s = sdf_scale(make_sphere(1.0f), 2.0f);
    const Vec3f pts[] = {
        {0.0f, 0.0f, 0.0f},    // center
        {1.0f, 0.0f, 0.0f},    // on the surface
        {2.0f, 0.0f, 0.0f},    // outside
        {3.0f, 0.0f, 0.0f},    // far outside
    };
    for (const auto& p : pts) {
        const auto samp = s.sample(p);
        const float gl = samp.gradient.length();
        INFO("point:", p.x, p.y, p.z, " |g| =", gl);
        CHECK(gl <= 1.0f + 1e-5f);
    }
}

TEST_CASE("ScaleSDF: scaled sphere value is correct") {
    // Sanity: the value at the new surface (radius 2) is 0, and the value
    // at the center is -2 (the new radius, not -1 as for the unscaled sphere).
    auto s = sdf_scale(make_sphere(1.0f), 2.0f);
    CHECK(s.value({0, 0, 0}) == doctest::Approx(-2.0f));
    CHECK(s.value({2, 0, 0}) == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(s.value({3, 0, 0}) == doctest::Approx(1.0f).epsilon(1e-5f));
}

// ===========================================================================
// TwistSDF warp-contract regressions (ADR-0029 MVP unit 1 — D1 bounds,
// D2 Lipschitz + gradient). One pin per contract method, each freezing
// the exact failure the audit found and the exact behavior the contract
// now guarantees:
//   (a) bounds — the audit's guinea pig, run under BOTH twist signs
//       (k-blindness is the theorem, executed);
//   (b) lipschitz — the old declaration's failure regime, measured;
//   (c) gradient — the shear term, derived and checked against finite
//       differences component-wise.
// ===========================================================================

TEST_CASE("TwistSDF: bounds — the audit's guinea pig under both twist signs (ADR-0029 D1)") {
    // The audit's exemplar: a sphere r = 0.25 translated to (2, 5, 0),
    // twisted with k = ±1. The y = 5 slice rotates by ±5 rad, carrying
    // the disk center to (2·cos5, 5, ±2·sin5) ≈ (0.567, 5, ∓1.918) —
    // deep outside the OLD bound (the child box, x ∈ [1.75, 2.25]).
    // The NEW bound is the cylinder AABB
    //   [−R, R] × [4.75, 5.25] × [−R, R],   R = √(2.25² + 0.25²) = √5.125,
    // the far corner (2.25, ±0.25) of the child box — k-blind: R does
    // not depend on k at all, so the SAME box must come out for both
    // signs (and the k-blind half of the pin below executes the proof:
    // the same outside points stay outside for both signs).
    const float R = std::sqrt(5.125f);   // 2.2638463
    for (const float k : {1.0f, -1.0f}) {
        auto tw = sdf_twist(sdf_translate(make_sphere(0.25f), {2.0f, 5.0f, 0.0f}), k);

        // The NEW box, pinned component-wise — identical for k = ±1.
        const auto b = tw.bounds();
        CHECK(b.min.x == doctest::Approx(-R).epsilon(1e-6f));
        CHECK(b.max.x == doctest::Approx( R).epsilon(1e-6f));
        CHECK(b.min.y == doctest::Approx(4.75f).epsilon(1e-6f));  // Y preserved
        CHECK(b.max.y == doctest::Approx(5.25f).epsilon(1e-6f));
        CHECK(b.min.z == doctest::Approx(-R).epsilon(1e-6f));
        CHECK(b.max.z == doctest::Approx( R).epsilon(1e-6f));

        // (i) The OLD contract violation, frozen as the regression: the
        // twisted sphere's center at height y = 5 is the child center
        // (2, 0) forward-rotated by θ = k·5, so sampling there
        // un-rotates exactly back onto the child center: f = −0.25.
        // The point lies OUTSIDE the old (child) box — the pre-fix
        // bounds() claimed the field could not be negative there.
        const float th = 5.0f * k;
        const Vec3f rotated_center{2.0f * std::cos(th), 5.0f, 2.0f * std::sin(th)};
        REQUIRE(rotated_center.x < 1.75f);   // outside the old box
        CHECK(tw.value(rotated_center) == doctest::Approx(-0.25f).epsilon(1e-4f));

        // (ii) The NEW box is sound, strictly, and k-blind: f > 0 at
        // the SAME outside points for BOTH signs. This is the proof
        // made executable: a point outside the cylinder AABB
        // un-rotates (rotation preserves radius and Y) to a point
        // still outside the child box, where the child's own contract
        // gives f > 0. The exact values are sign-independent and
        // pinned: 2.402705 (radial exit, r = 2.4 > R) and 1.356438
        // (y exit, y = 5.5 > 5.25 — Y is preserved exactly, so this
        // samples the child at y = 5.5).
        CHECK(tw.value(Vec3f{2.4f, 5.0f, 0.0f}) == doctest::Approx(2.402705f).epsilon(1e-4f));
        CHECK(tw.value(Vec3f{2.0f, 5.5f, 0.0f}) == doctest::Approx(1.356438f).epsilon(1e-4f));
        CHECK(tw.value(Vec3f{2.4f, 5.0f, 0.0f}) > 0.0f);
        CHECK(tw.value(Vec3f{2.0f, 5.5f, 0.0f}) > 0.0f);
    }

    // k = 0 passthrough: the identity warp returns the child's box
    // verbatim — exact, not conservative.
    auto sph0 = sdf_translate(make_sphere(0.25f), {2.0f, 5.0f, 0.0f});
    const auto cb = sph0.bounds();
    auto tw0 = sdf_twist(std::move(sph0), 0.0f);
    const auto b0 = tw0.bounds();
    CHECK(b0.min.x == cb.min.x);  CHECK(b0.max.x == cb.max.x);
    CHECK(b0.min.y == cb.min.y);  CHECK(b0.max.y == cb.max.y);
    CHECK(b0.min.z == cb.min.z);  CHECK(b0.max.z == cb.max.z);
}

TEST_CASE("TwistSDF: lipschitz — σ measured in the old failure regime (ADR-0029 D2)") {
    // The old declaration L = L_child·(1 + 2|k|) is dimensionally blind:
    // the true factor depends on the dimensionless k·r, and it failed as
    // an upper bound for k·r ≳ 0.2 — any child reaching r ≈ 4 at
    // k = 0.05 breaks it; +16% at r = 10. This pin measures the true
    // slope in exactly that regime and freezes both facts LIVE:
    //   * the measured slope stays UNDER the new declaration
    //     L = σ(k, r_box), r_box = √2·R of the new bounds box — the
    //     contract;
    //   * the measured slope EXCEEDS the old 1.1 — the old declaration
    //     would have failed here (the marcher would have over-stepped).
    //
    // Why the sampling is directed (where the slope lives): the
    // Jacobian's top singular direction at radius r mixes the
    // tangential xz direction with Y, so the slope approaches σ(k, r)
    // only where the child gradient has a tangential component AND a
    // y component. A unit sphere at (10, 0, 0) supplies both on its
    // tangency circles — the points u with (u.xz − c.xz)·u.xz = 0,
    // where the sphere's radial-from-center gradient is exactly
    // tangential w.r.t. the twist axis:
    //   u.xz = 5·(1 + cos φ, ±sin φ)   (radius 10·cos(φ/2)),
    // with the gradient's y component set by the height y. Both
    // circles are sampled (the orientation that aligns with the shear
    // is the mirrored one; sampling both makes the pin robust to that
    // choice). All sampled points stay INSIDE the declared region —
    // the new bounds box: y ∈ [−1, 1], r ≤ R = √(11² + 1²) ≈ 11.05.
    const float k = 0.05f;
    auto tw = sdf_twist(sdf_translate(make_sphere(1.0f), {10.0f, 0.0f, 0.0f}), k);

    // The new declaration, pinned to the closed form: R = √122,
    // r_box = √244 ≈ 15.6205, t = (k·r_box)² = 0.61,
    // σ = √(1 + (t + √(t² + 4t))/2) ≈ 1.464058.
    const float L = tw.lipschitz();
    CHECK(L == doctest::Approx(1.464058f).epsilon(1e-4f));

    // The measured slope: central finite differences on VALUES ONLY
    // (no use of the gradient code under test) — the FD gradient at p
    // is the local pair-Lipschitz, and sup|∇f| is the field's slope.
    // FD budget (the smin-fix method): h = 1e-3 — float32 roundoff
    // ≈ ε·|f|/(2h) ≈ 2e-4 at |f| ≈ 3, truncation O(h²·|f'''|) ≪ 1e-5
    // on this smooth regime — total ≲ 2e-4; the margins asserted
    // below are three orders above it.
    const float h = 1e-3f;
    float max_slope = 0.0f;
    Vec3f worst_p{0, 0, 0};
    const float phis[] = {3.0f, 4.0f, 6.0f, 8.0f, 10.0f, 15.0f, 20.0f, 30.0f};
    const float ys[]   = {0.25f, 0.35f, 0.5f, 0.7f, 0.9f};
    for (const float deg : phis) {
        for (const float zsign : {1.0f, -1.0f}) {
            for (const float y : ys) {
                const float phi = deg * 3.14159265358979f / 180.0f;
                const float ux = 5.0f * (1.0f + std::cos(phi));
                const float uz = zsign * 5.0f * std::sin(phi);
                // World query: forward-rotate u by θ = k·y — sample()
                // un-rotates by exactly this, so the child-frame point
                // is the intended u.
                const float th = k * y;
                const Vec3f p{ux * std::cos(th) - uz * std::sin(th), y,
                              ux * std::sin(th) + uz * std::cos(th)};
                const Vec3f gfd{
                    (tw.value(p + Vec3f{ h, 0, 0}) - tw.value(p + Vec3f{-h, 0, 0})) * (0.5f / h),
                    (tw.value(p + Vec3f{0,  h, 0}) - tw.value(p + Vec3f{0, -h, 0})) * (0.5f / h),
                    (tw.value(p + Vec3f{0, 0,  h}) - tw.value(p + Vec3f{0, 0, -h})) * (0.5f / h),
                };
                const float slope = gfd.length();
                if (slope > max_slope) { max_slope = slope; worst_p = p; }
            }
        }
    }
    INFO("measured max slope =", max_slope, " at p = (", worst_p.x, ",", worst_p.y, ",", worst_p.z, ")",
         " declared L =", L);
    CHECK(max_slope <= L + 1e-3f);  // FD-noise ε — the new declaration holds
    CHECK(max_slope > 1.1f);        // the OLD declaration (1 + 2|k| = 1.1) fails, live
}

TEST_CASE("TwistSDF: gradient — the shear term, derived and pinned against FD (ADR-0029 D2)") {
    // Off-axis points where the child gradient has a NON-ZERO tangential
    // component — the regime where the pre-ADR-0029 gradient (g.y alone,
    // no shear) was wrong. Child: box {2,2,2}. The queries are chosen in
    // CHILD frame first, on the smooth sheet where the box field is
    // exactly x − 2 with gradient exactly (1, 0, 0) (outside the +X
    // face only: x > 2, |y| < 2, |z| < 2), then mapped forward to the
    // world query p = (R(+θ)·u.xz, u.y) with θ = k·u.y — sample()
    // un-rotates by exactly θ, so the child-frame point IS u.
    //
    // DERIVATION of the expected gradient at u = (3, 1, 0.5), θ = k·1:
    // the child gradient there is g = (1, 0, 0) exactly and f = 1
    // exactly, so the chain rule gives
    //   ∂f/∂p.y = g.y + k·(u.z·g.x − u.x·g.z) = k·u.z = ±0.5
    //   ∂f/∂p.x = g.x·cos θ − g.z·sin θ = cos 1   (sign-independent)
    //   ∂f/∂p.z = g.x·sin θ + g.z·cos θ = ±sin 1
    // i.e. ∇f = (cos 1, ±0.5, ±sin 1), |∇f| = √1.25 ≈ 1.118 — the y
    // component IS the shear term: the pre-fix code returned 0 there.
    for (const float k : {1.0f, -1.0f}) {
        auto tw = sdf_twist(make_box({2, 2, 2}), k);
        const Vec3f u{3.0f, 1.0f, 0.5f};
        const float th = k * u.y;
        const Vec3f p{u.x * std::cos(th) - u.z * std::sin(th), u.y,
                      u.x * std::sin(th) + u.z * std::cos(th)};

        // The un-rotated point is u and the value is exactly 1.
        CHECK(tw.value(p) == doctest::Approx(1.0f).epsilon(1e-5f));

        const auto s = tw.sample(p);
        CHECK(s.gradient.x == doctest::Approx(0.54030231f).epsilon(1e-5f));      // cos 1
        CHECK(s.gradient.y == doctest::Approx(0.5f * k).epsilon(1e-5f));         // THE SHEAR
        CHECK(s.gradient.z == doctest::Approx(0.84147098f * k).epsilon(1e-5f));  // sin 1
        // The field.hpp invariant: |∇f| ≤ lipschitz().
        CHECK(s.gradient.length() <= tw.lipschitz());

        // Component-wise vs central finite differences (the smin-fix
        // method). FD budget, derived: float32 roundoff
        // ε·|f|/(2h) ≈ 6e-5 at |f| ≈ 1 with h = 1e-3; truncation
        // h²·O(k³·r³)/6 ≈ 5e-6 on this smooth sheet (the field is
        // LINEAR in the child frame; the only curvature is the trig of
        // the warp itself) — total ≈ 6.5e-5, pinned at 1e-3 ≈ 15×
        // margin. The missing-shear bug this pin must catch is 0.5 in
        // the y component — three orders above the tolerance. The
        // stencil stays on the smooth sheet: |Δu| ≤ h·σ(k, r) ≈ 3e-3
        // against margins ≥ 1 to the sheet boundaries.
        const float hstep = 1e-3f;
        const Vec3f gfd{
            (tw.value(p + Vec3f{ hstep, 0, 0}) - tw.value(p + Vec3f{-hstep, 0, 0})) * (0.5f / hstep),
            (tw.value(p + Vec3f{0,  hstep, 0}) - tw.value(p + Vec3f{0, -hstep, 0})) * (0.5f / hstep),
            (tw.value(p + Vec3f{0, 0,  hstep}) - tw.value(p + Vec3f{0, 0, -hstep})) * (0.5f / hstep),
        };
        INFO("k =", k, " analytic: (", s.gradient.x, s.gradient.y, s.gradient.z, ")",
             " fd: (", gfd.x, gfd.y, gfd.z, ")");
        CHECK(std::abs(s.gradient.x - gfd.x) <= 1e-3f);
        CHECK(std::abs(s.gradient.y - gfd.y) <= 1e-3f);
        CHECK(std::abs(s.gradient.z - gfd.z) <= 1e-3f);
    }
}

// ===========================================================================
// ADR-0029 D5 — construction-time validation. One TEST_CASE per node of
// the D5 table: out-of-class input → std::invalid_argument with the
// bridge-style message; borderline IN-class input → no exception.
// ===========================================================================

TEST_CASE("TranslateSDF: null child and non-finite offset rejected (ADR-0029 D5)") {
    CHECK_THROWS_WITH_AS(TranslateSDF(nullptr, {1, 0, 0}),
        doctest::Contains("TranslateSDF: child must be a constructed (non-null) SDF child"),
        std::invalid_argument);
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    const float inf_f = std::numeric_limits<float>::infinity();
    CHECK_THROWS_WITH_AS(sdf_translate(make_sphere(1.0f), {nan_f, 0, 0}),
        doctest::Contains("TranslateSDF: offset must be finite on every axis"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_translate(make_sphere(1.0f), {0, 0, inf_f}),
        doctest::Contains("TranslateSDF: offset must be finite on every axis"),
        std::invalid_argument);
    // The message prints the whole vector: the offending component is
    // visible in context.
    CHECK_THROWS_WITH_AS(sdf_translate(make_sphere(1.0f), {0, nan_f, 0}),
        doctest::Contains("got (0.000000, nan, 0.000000)"),
        std::invalid_argument);
    CHECK_NOTHROW(sdf_translate(make_sphere(1.0f), {-1e30f, 0, 0}));
}

TEST_CASE("RotateSDF: null child and non-unit quaternion rejected (ADR-0029 D5)") {
    CHECK_THROWS_WITH_AS(RotateSDF(nullptr, Quatf::identity()),
        doctest::Contains("RotateSDF: child must be a constructed (non-null) SDF child"),
        std::invalid_argument);

    // Non-unit: a rotate that scales by |q|^2 — norm 2 and norm 0.5.
    CHECK_THROWS_WITH_AS(sdf_rotate(make_sphere(1.0f), Quatf{2.0f, 0, 0, 0}),
        doctest::Contains("RotateSDF: quaternion q must have unit norm within 1e-6"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_rotate(make_sphere(1.0f), Quatf{0.5f, 0, 0, 0}),
        doctest::Contains("RotateSDF: quaternion q must have unit norm within 1e-6"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_rotate(make_sphere(1.0f), Quatf{0.5f, 0, 0, 0}),
        doctest::Contains("got 0.500000"), std::invalid_argument);
    // NaN components fail the norm check (the negated comparison).
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_WITH_AS(sdf_rotate(make_sphere(1.0f), Quatf{nan_f, 0, 0, 0}),
        doctest::Contains("RotateSDF: quaternion q must have unit norm within 1e-6"),
        std::invalid_argument);

    // Tolerance boundary: |q| = 1 ± 5e-7 is IN-class (serialization
    // round-trips at ~1e-9), |q| = 1 + 2e-6 is out.
    const Quatf unit = Quatf::from_axis_angle(Vec3f{1, 2, 3}.normalized(), 0.6f);
    const Quatf within = Quatf{unit.w * (1.0f + 5e-7f), unit.x * (1.0f + 5e-7f),
                               unit.y * (1.0f + 5e-7f), unit.z * (1.0f + 5e-7f)};
    const Quatf beyond = Quatf{unit.w * (1.0f + 2e-6f), unit.x * (1.0f + 2e-6f),
                               unit.y * (1.0f + 2e-6f), unit.z * (1.0f + 2e-6f)};
    CHECK_NOTHROW(sdf_rotate(make_sphere(1.0f), within));
    CHECK_THROWS_WITH_AS(sdf_rotate(make_sphere(1.0f), beyond),
        doctest::Contains("must have unit norm within 1e-6"),
        std::invalid_argument);
    CHECK_NOTHROW(sdf_rotate(make_sphere(1.0f), Quatf::identity()));
}

TEST_CASE("ScaleSDF: null child and s <= 0 rejected — the motivating rejection (ADR-0029 D5)") {
    CHECK_THROWS_WITH_AS(ScaleSDF(nullptr, 2.0f),
        doctest::Contains("ScaleSDF: child must be a constructed (non-null) SDF child"),
        std::invalid_argument);
    // s = 0: the motivating finding — 1/s = inf, NaN fields everywhere.
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f), 0.0f),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f), 0.0f),
        doctest::Contains("got 0.000000"), std::invalid_argument);
    // s < 0: the sign flip that turns the negative region into the
    // unbounded complement of the mirrored child.
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f), -2.0f),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);
    // NaN and +inf: caught by the finite arm ("all imply finite").
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f), nan_f),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f),
                                   std::numeric_limits<float>::infinity()),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);
    // The remedy says what to do instead.
    CHECK_THROWS_WITH_AS(sdf_scale(make_sphere(1.0f), 0.0f),
        doctest::Contains("Remedy: scale by s > 0"),
        std::invalid_argument);
    CHECK_NOTHROW(sdf_scale(make_sphere(1.0f), 0.25f));
}

TEST_CASE("TwistSDF: null child and non-finite k rejected (ADR-0029 D5)") {
    CHECK_THROWS_WITH_AS(TwistSDF(nullptr, 0.5f),
        doctest::Contains("TwistSDF: child must be a constructed (non-null) SDF child"),
        std::invalid_argument);
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_WITH_AS(sdf_twist(make_sphere(1.0f), nan_f),
        doctest::Contains("TwistSDF: twist rate k must be finite"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(sdf_twist(make_sphere(1.0f),
                                   -std::numeric_limits<float>::infinity()),
        doctest::Contains("TwistSDF: twist rate k must be finite"),
        std::invalid_argument);
    // Any finite k is in-class: negative (counter-twist) and tiny.
    CHECK_NOTHROW(sdf_twist(make_sphere(1.0f), -1.0f));
    CHECK_NOTHROW(sdf_twist(make_sphere(1.0f), 1e-30f));
}

TEST_CASE("D5 borderline: twist k = 0 identity stays legal (ADR-0029 D5)") {
    // The audit near-hit: k = 0 is the identity warp. Constructed and
    // pinned as a passthrough — the field is the child's, bit-for-bit.
    auto child = make_box({1, 5, 1});
    const auto plain = child.sample({0.3f, -2.1f, 0.7f});
    auto twisted = sdf_twist(std::move(child), 0.0f);
    const auto warped = twisted.sample({0.3f, -2.1f, 0.7f});
    CHECK(warped.value == plain.value);
    CHECK(warped.gradient.x == plain.gradient.x);
    CHECK(warped.gradient.y == plain.gradient.y);
    CHECK(warped.gradient.z == plain.gradient.z);
    // And bounds() returns the child's box verbatim (the k = 0
    // passthrough pinned in the ADR-0029 unit-1 fix).
    auto b2 = sdf_twist(make_box({1, 5, 1}), 0.0f);
    CHECK(b2.bounds().min == Vec3f{-1, -5, -1});
    CHECK(b2.bounds().max == Vec3f{1, 5, 1});
}
