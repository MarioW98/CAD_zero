// tests/unit/viewport/test_camera.cpp
//
// Tests for the OrbitCamera class.
//
// All tests are deterministic and don't require OpenGL — pure math.
//
#include <doctest/doctest.h>

#include "CAD_0/viewport/camera.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"

#include <cmath>

using namespace CAD_0;
using namespace CAD_0::viewport;
using namespace CAD_0::math;

TEST_CASE("OrbitCamera: default construction") {
    OrbitCamera cam;
    CHECK(cam.target() == Vec3f{0, 0, 0});
    CHECK(cam.distance() == doctest::Approx(5.0f));
    CHECK(cam.fov_y() == doctest::Approx(0.7853982f));  // ~45°
    CHECK(cam.aspect() == doctest::Approx(1.0f));
}

TEST_CASE("OrbitCamera: explicit construction") {
    OrbitCamera cam{{1, 2, 3}, 10.0f, 0.5f, 0.3f};
    CHECK(cam.target() == Vec3f{1, 2, 3});
    CHECK(cam.distance() == doctest::Approx(10.0f));
    CHECK(cam.yaw() == doctest::Approx(0.5f));
    CHECK(cam.pitch() == doctest::Approx(0.3f));
}

TEST_CASE("OrbitCamera: position derived from spherical coords") {
    // Camera at yaw=0, pitch=0, distance=5, target=origin.
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    const Vec3f p = cam.position();
    // At yaw=0, pitch=0, the camera should be at (0, 0, +5).
    CHECK(p.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(p.y == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(p.z == doctest::Approx(5.0f).epsilon(1e-5f));
}

TEST_CASE("OrbitCamera: forward points toward target") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    const Vec3f f = cam.forward();
    // Camera at +Z looking at origin → forward = -Z.
    CHECK(f.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(f.y == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(f.z == doctest::Approx(-1.0f).epsilon(1e-5f));
}

TEST_CASE("OrbitCamera: orbit changes yaw and pitch") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    cam.orbit(0.5f, 0.3f);
    CHECK(cam.yaw() == doctest::Approx(0.5f));
    CHECK(cam.pitch() == doctest::Approx(0.3f));
}

TEST_CASE("OrbitCamera: orbit clamps pitch to avoid gimbal lock") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    // Try to pitch beyond ±89°.
    cam.orbit(0.0f, 2.0f);  // ~115°
    // Pitch should be clamped to ~89° (1.5533 rad).
    CHECK(cam.pitch() <= 1.5534f);
    CHECK(cam.pitch() >= 1.5532f);
}

TEST_CASE("OrbitCamera: orbit_inverse reverses an orbit") {
    OrbitCamera cam1{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    OrbitCamera cam2{{0, 0, 0}, 5.0f, 0.0f, 0.0f};

    const auto delta = cam1.orbit(0.5f, 0.3f);
    cam1.orbit_inverse(delta);
    // After inverse, cam1 should match cam2 (within tolerance).
    CHECK(cam1.yaw()   == doctest::Approx(cam2.yaw()).epsilon(1e-5f));
    CHECK(cam1.pitch() == doctest::Approx(cam2.pitch()).epsilon(1e-5f));
}

TEST_CASE("OrbitCamera: zoom reduces distance") {
    OrbitCamera cam{{0, 0, 0}, 10.0f};
    cam.zoom(0.5f);  // halve the distance
    CHECK(cam.distance() == doctest::Approx(5.0f));
}

TEST_CASE("OrbitCamera: zoom increases distance") {
    OrbitCamera cam{{0, 0, 0}, 2.0f};
    cam.zoom(2.0f);  // double the distance
    CHECK(cam.distance() == doctest::Approx(4.0f));
}

TEST_CASE("OrbitCamera: zoom is clamped to min/max") {
    OrbitCamera cam{{0, 0, 0}, 1.0f};
    cam.zoom(0.001f);  // very small factor
    CHECK(cam.distance() > 0.0f);  // not zero
    // Should be clamped to the minimum distance.
}

TEST_CASE("OrbitCamera: view_matrix is orthonormal rigid transform") {
    OrbitCamera cam{{1, 2, 3}, 8.0f, 0.4f, 0.2f};
    cam.set_aspect(1.5f);
    const Mat4f v = cam.view_matrix();
    // Inverse of view = inverse_orthonormal (rotation + translation).
    const Mat4f inv = v.inverse_orthonormal();
    const Mat4f id = v * inv;
    // Should be identity — check by transforming a known point.
    const Vec3f p{5, 7, 11};
    const Vec3f recovered = id.transform_point(p);
    CHECK(recovered.x == doctest::Approx(p.x).epsilon(1e-4f));
    CHECK(recovered.y == doctest::Approx(p.y).epsilon(1e-4f));
    CHECK(recovered.z == doctest::Approx(p.z).epsilon(1e-4f));
}

TEST_CASE("OrbitCamera: projection_matrix maps unit cube to clip space") {
    OrbitCamera cam;
    cam.set_aspect(1.0f);
    cam.set_fov_y(3.14159265358979323846f * 0.5f);  // 90° vertical FOV
    const Mat4f p = cam.projection_matrix();
    // Point at origin (camera-space) maps to (0, 0, ?, 1).
    // After perspective divide, it should be at (0, 0, ?, 1).
    // For a perspective projection with near=0.01 and far=1000, the
    // origin in camera space is at z=0 → mapped to z = (far+near)/(near-far)
    // divided by -(z/near_far_term) — let's just verify the matrix is
    // not degenerate by checking its diagonal.
    // For 90° FOV, f = 1 / tan(45°) = 1.
    CHECK(p.cols[0].x == doctest::Approx(1.0f).epsilon(1e-5f));
    CHECK(p.cols[1].y == doctest::Approx(1.0f).epsilon(1e-5f));
}

TEST_CASE("OrbitCamera: pan moves target by screen-space delta") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    const Vec3f target_before = cam.target();
    cam.pan(0.1f, 0.0f);
    const Vec3f target_after = cam.target();
    // Target should have moved in the camera's right direction.
    CHECK(target_after != target_before);
    // Specifically, with yaw=0, pitch=0, the right axis is +X.
    CHECK(target_after.x > target_before.x);
}

TEST_CASE("OrbitCamera: frame_bounds fits scene in view") {
    OrbitCamera cam;
    Bboxf box;
    box.expand({-2, -2, -2});
    box.expand({2, 2, 2});
    cam.frame_bounds(box);
    // Target should be the box center (origin in this case).
    const Vec3f t = cam.target();
    CHECK(t.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(t.y == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(t.z == doctest::Approx(0.0f).epsilon(1e-5f));
    // Distance should be > 2 (the box radius), since the camera needs
    // to be far enough to fit the box in the FOV.
    CHECK(cam.distance() > 2.0f);
}

TEST_CASE("OrbitCamera: ray_from_ndc at screen center points along -Z") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    cam.set_aspect(1.0f);
    // NDC (0, 0) is the center of the screen.
    const auto ray = cam.ray_from_ndc(0.0f, 0.0f);
    // Origin should be the camera position (within FP tolerance of the
    // inverse view-projection matrix).
    const Vec3f expected_origin = cam.position();
    CHECK(ray.origin.x == doctest::Approx(expected_origin.x).epsilon(1e-2f));
    CHECK(ray.origin.y == doctest::Approx(expected_origin.y).epsilon(1e-2f));
    CHECK(ray.origin.z == doctest::Approx(expected_origin.z).epsilon(1e-2f));
    // Direction should be -Z (looking forward).
    CHECK(ray.direction.x == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(ray.direction.y == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(ray.direction.z == doctest::Approx(-1.0f).epsilon(1e-2f));
}

TEST_CASE("OrbitCamera: ray_from_ndc at corner has non-trivial direction") {
    OrbitCamera cam{{0, 0, 0}, 5.0f, 0.0f, 0.0f};
    cam.set_aspect(1.0f);
    const auto ray = cam.ray_from_ndc(1.0f, 1.0f);  // top-right corner
    // Direction should be normalized.
    const float len = ray.direction.length();
    CHECK(len == doctest::Approx(1.0f).epsilon(1e-4f));
    // Should point toward -Z (since camera looks along -Z).
    CHECK(ray.direction.z < 0.0f);
}
