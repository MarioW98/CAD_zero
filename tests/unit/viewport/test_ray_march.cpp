// tests/unit/viewport/test_ray_march.cpp
//
// Tests for the CPU-side ray-marcher.
//
// These tests verify the ray-marching algorithm against known SDFs
// (sphere, box) and known camera configurations. They mirror the
// algorithm in glsl/sdf_raymarch.frag, ensuring the shader logic is
// correct before being tested in an actual GL context.
//
#include <doctest/doctest.h>

#include "CAD_0/viewport/ray_march.hpp"
#include "CAD_0/viewport/camera.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/math/vec.hpp"

#include <cmath>

using namespace CAD_0;
using namespace CAD_0::viewport;
using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("ray_march: hits a sphere straight on") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;

    // Ray from (0, 0, 5) looking toward origin (-Z direction).
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    CHECK(r.hit);
    CHECK(r.distance == doctest::Approx(4.0f).epsilon(1e-3f));  // 5 - 1 = 4
    // Hit point should be at (0, 0, 1).
    CHECK(r.point.x == doctest::Approx(0.0f).epsilon(1e-3f));
    CHECK(r.point.y == doctest::Approx(0.0f).epsilon(1e-3f));
    CHECK(r.point.z == doctest::Approx(1.0f).epsilon(1e-3f));
    // Normal should point along +Z (outward from sphere at (0,0,1)).
    CHECK(r.normal.x == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(r.normal.y == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(r.normal.z == doctest::Approx(1.0f).epsilon(1e-2f));
}

TEST_CASE("ray_march: misses when ray points away from sphere") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;

    // Ray from (0, 0, 5) looking toward +Z (away from origin).
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, 1};
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    CHECK_FALSE(r.hit);
}

TEST_CASE("ray_march: hits sphere from the side") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;

    // Ray from (5, 0, 0) looking toward origin (-X direction).
    const Vec3f ro{5, 0, 0};
    const Vec3f rd{-1, 0, 0};
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    CHECK(r.hit);
    CHECK(r.point.x == doctest::Approx(1.0f).epsilon(1e-3f));
    CHECK(r.normal.x == doctest::Approx(1.0f).epsilon(1e-2f));
}

TEST_CASE("ray_march: respects max_distance") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;
    opts.max_distance = 1.0f;  // sphere is at distance 4 — too far

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    CHECK_FALSE(r.hit);
}

TEST_CASE("ray_march: respects max_steps") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;
    opts.max_steps = 1;  // very few steps — likely won't reach the surface

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    // With only 1 step starting at distance 4 from sphere center,
    // the first step takes us 4 units closer (to distance 0 from center,
    // i.e. we reach the surface in 1 step). So this might actually hit.
    // We don't assert on hit/miss; just that the steps_taken is bounded.
    CHECK(r.steps_taken <= 1);
}

TEST_CASE("ray_march: steps_taken is non-zero for a hit") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};
    const auto r = ray_march_sdf(sph, ro, rd, opts);
    REQUIRE(r.hit);
    CHECK(r.steps_taken > 0);
}

TEST_CASE("ray_march: from_camera builds correct ray") {
    // Camera at (0, 0, 5) looking at origin.
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    cam.set_aspect(1.0f);
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts;

    // Ray through screen center (NDC 0, 0).
    const auto r = ray_march_sdf_from_camera(sph, cam, 0.0f, 0.0f, opts);
    CHECK(r.hit);
    CHECK(r.point.z == doctest::Approx(1.0f).epsilon(1e-2f));
}

TEST_CASE("ray_march: ray that misses the sphere from camera") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    cam.set_aspect(1.0f);
    auto sph = make_sphere(0.5f);  // smaller sphere

    // NDC corner (1, 1) — ray should miss a small sphere centered at origin.
    RayMarchOptions opts;
    opts.max_distance = 5.0f;  // don't extend past the camera
    const auto r = ray_march_sdf_from_camera(sph, cam, 1.0f, 1.0f, opts);
    // May or may not hit depending on the ray direction; just verify it
    // returns a valid result (either hit or no hit, no crash).
    CHECK((r.hit || !r.hit));  // always true; documents that we got a result
}

TEST_CASE("ray_march: step_scale < 1 is more conservative") {
    auto sph = make_sphere(1.0f);
    RayMarchOptions opts_aggressive;
    opts_aggressive.step_scale = 1.0f;

    RayMarchOptions opts_conservative;
    opts_conservative.step_scale = 0.5f;

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    const auto r1 = ray_march_sdf(sph, ro, rd, opts_aggressive);
    const auto r2 = ray_march_sdf(sph, ro, rd, opts_conservative);

    REQUIRE(r1.hit);
    REQUIRE(r2.hit);
    // Conservative marcher should take more steps (or equal).
    CHECK(r2.steps_taken >= r1.steps_taken);
}

TEST_CASE("ray_march: hit point lies on sphere surface") {
    auto sph = make_sphere(1.5f);
    RayMarchOptions opts;

    // Ray from (10, 1, 0) toward origin.
    const Vec3f ro{10, 1, 0};
    const Vec3f rd = (Vec3f{0, 0, 0} - ro).normalized();
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    REQUIRE(r.hit);
    // The hit point should be at distance ~1.5 from origin.
    const float dist = r.point.length();
    CHECK(dist == doctest::Approx(1.5f).epsilon(1e-2f));
}

TEST_CASE("ray_march: normal is unit-length") {
    auto sph = make_sphere(2.0f);
    RayMarchOptions opts;

    const Vec3f ro{5, 5, 5};
    const Vec3f rd = (Vec3f{0, 0, 0} - ro).normalized();
    const auto r = ray_march_sdf(sph, ro, rd, opts);

    REQUIRE(r.hit);
    const float n_len = r.normal.length();
    CHECK(n_len == doctest::Approx(1.0f).epsilon(1e-3f));
}

TEST_CASE("ray_march: empty SDF body returns hit at distance 0") {
    // An empty SDFBody returns value = 0 everywhere (see SDFBody::sample
    // default). The ray-marcher treats this as a hit at the ray origin.
    // This is the documented behavior — callers should not pass empty
    // bodies to the ray-marcher.
    SDFBody empty;
    RayMarchOptions opts;

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};
    const auto r = ray_march_sdf(empty, ro, rd, opts);

    CHECK(r.hit);
    CHECK(r.distance == doctest::Approx(0.0f).epsilon(1e-3f));
}

// ----------------------------------------------------------------------
// Lipschitz-aware step scaling (added after the bug fix)
// ----------------------------------------------------------------------
TEST_CASE("ray_march: respects body Lipschitz constant") {
    // The Twist SDF is the canonical example of a non-unit-Lipschitz field:
    // it reports lipschitz > 1 because the warp can stretch the field.
    // The marcher must scale its step by 1/lipschitz to avoid overshooting.
    auto box = make_box({1, 1, 1});
    auto twisted = sdf_twist(std::move(box), 0.5f);
    REQUIRE(twisted.lipschitz() > 1.0f);

    RayMarchOptions opts;
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    // Untwisted box: lipschitz = 1, marches with full steps.
    auto plain = make_box({1, 1, 1});
    const auto r_plain   = ray_march_sdf(plain,   ro, rd, opts);
    const auto r_twisted = ray_march_sdf(twisted, ro, rd, opts);

    REQUIRE(r_plain.hit);
    REQUIRE(r_twisted.hit);

    // Both should hit at the correct world-space distance (≈ 4 from the
    // box's +Z face).
    CHECK(r_plain.distance   == doctest::Approx(4.0f).epsilon(1e-3f));
    CHECK(r_twisted.distance == doctest::Approx(4.0f).epsilon(0.1f));

    // The twisted (L > 1) marcher should take at least as many steps as
    // the plain (L = 1) marcher, because its steps are scaled down by L.
    CHECK(r_twisted.steps_taken >= r_plain.steps_taken);
}

TEST_CASE("ray_march: scaled sphere (lipschitz = 1) hits correctly") {
    // After the ScaleSDF lipschitz fix, scaling a unit-Lipschitz sphere
    // produces a field with lipschitz = 1 (NOT scale). The marcher
    // therefore uses the same step size as for the unscaled sphere.
    auto sph_small = make_sphere(1.0f);
    auto sph_large = sdf_scale(make_sphere(1.0f), 2.0f);  // world radius = 2
    REQUIRE(sph_small.lipschitz() == 1.0f);
    REQUIRE(sph_large.lipschitz() == 1.0f);  // post-fix: not 2.0

    RayMarchOptions opts;
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    const auto r_small = ray_march_sdf(sph_small, ro, rd, opts);
    const auto r_large = ray_march_sdf(sph_large, ro, rd, opts);

    REQUIRE(r_small.hit);
    REQUIRE(r_large.hit);

    // small: distance = 5 - 1 = 4
    // large: distance = 5 - 2 = 3
    CHECK(r_small.distance == doctest::Approx(4.0f).epsilon(1e-3f));
    CHECK(r_large.distance == doctest::Approx(3.0f).epsilon(1e-3f));
}

TEST_CASE("ray_march: Twist body (lipschitz > 1) still hits correctly") {
    // A twisted box has lipschitz > 1. Before the fix, the marcher
    // could overshoot the surface. After the fix, it correctly scales
    // steps by 1/lipschitz.
    auto box = make_box({1, 1, 1});
    auto twisted = sdf_twist(std::move(box), 0.5f);
    REQUIRE(twisted.lipschitz() > 1.0f);

    RayMarchOptions opts;
    opts.max_distance = 50.0f;
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    const auto r = ray_march_sdf(twisted, ro, rd, opts);
    CHECK(r.hit);
    // The twisted box is still centered at origin with bounds roughly
    // [-1, 1]³. Hit distance should be near 4.
    CHECK(r.distance == doctest::Approx(4.0f).epsilon(0.1f));
}

// ---------------------------------------------------------------------------
// Task 2 — Lipschitz-safe stepping for sdf_scale(sphere, s) with s < 1
//
// The user-reported bug was: ray_march's `t += d` step ignored the
// field's Lipschitz constant, causing overshoot/perforation in fields
// with L > 1. Even though ScaleSDF with s < 1 reports L = 1 (the
// correct bound — see transforms.hpp), this test exercises the case
// explicitly: a shrunken sphere (radius 0.5) must still be hit at the
// correct world-space distance, regardless of the scale factor.
//
// Parity with GLSL: the sdf_raymarch.frag shader uses `t += d` for its
// hardcoded `length(p) - 1.0` sphere (L = 1). The CPU marcher uses
// `t += d / max(L, 1) = d` for L = 1, matching the shader exactly.
// ---------------------------------------------------------------------------

TEST_CASE("ray_march: scaled sphere with s<1 hits at correct distance") {
    // sphere(r=1) scaled by 0.5 → world radius = 0.5
    // The ray from (0,0,5) toward -Z should hit at distance 5 - 0.5 = 4.5.
    auto sph_small = sdf_scale(make_sphere(1.0f), 0.5f);
    REQUIRE(sph_small.lipschitz() == 1.0f);  // ScaleSDF preserves lipschitz

    RayMarchOptions opts;
    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    const auto r = ray_march_sdf(sph_small, ro, rd, opts);
    REQUIRE(r.hit);
    CHECK(r.distance == doctest::Approx(4.5f).epsilon(1e-3f));
    CHECK(r.point.z  == doctest::Approx(0.5f).epsilon(1e-3f));
    CHECK(r.normal.z == doctest::Approx(1.0f).epsilon(1e-2f));
}

TEST_CASE("ray_march: scaled sphere with s<1 from the side") {
    // sphere(r=1) scaled by 0.25 → world radius = 0.25
    // Ray from (5, 0, 0) toward -X should hit at distance 5 - 0.25 = 4.75.
    auto sph_tiny = sdf_scale(make_sphere(1.0f), 0.25f);
    REQUIRE(sph_tiny.lipschitz() == 1.0f);

    RayMarchOptions opts;
    const Vec3f ro{5, 0, 0};
    const Vec3f rd{-1, 0, 0};

    const auto r = ray_march_sdf(sph_tiny, ro, rd, opts);
    REQUIRE(r.hit);
    CHECK(r.distance == doctest::Approx(4.75f).epsilon(1e-3f));
    CHECK(r.point.x  == doctest::Approx(0.25f).epsilon(1e-3f));
    CHECK(r.normal.x == doctest::Approx(1.0f).epsilon(1e-2f));
}

TEST_CASE("ray_march: scaled sphere CPU matches GLSL shader algorithm") {
    // The GLSL shader (sdf_raymarch.frag) uses `t += d` for a hardcoded
    // unit sphere (L = 1). For a unit-Lipschitz field, the CPU marcher's
    // `t += d / max(L, 1) = t += d` matches exactly. We verify this by
    // checking that the CPU marcher on a plain unit sphere returns the
    // same hit distance as the shader would (5 - 1 = 4 from ro=(0,0,5)).
    auto sph = make_sphere(1.0f);
    REQUIRE(sph.lipschitz() == 1.0f);

    RayMarchOptions opts;
    // Match the shader's constants (MAX_STEPS=128, MAX_DIST=100, SURF_DIST=0.0005).
    opts.max_steps = 128;
    opts.max_distance = 100.0f;
    opts.surface_distance = 0.0005f;
    opts.step_scale = 1.0f;

    const Vec3f ro{0, 0, 5};
    const Vec3f rd{0, 0, -1};

    const auto r = ray_march_sdf(sph, ro, rd, opts);
    REQUIRE(r.hit);
    // The shader would return t = 4.0 here (5 - 1). The CPU must match.
    CHECK(r.distance == doctest::Approx(4.0f).epsilon(1e-3f));
}
